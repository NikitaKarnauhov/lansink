/*
    player.cpp

    Copyright (c) 2013, Nikita Karnauhov

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS `AS IS'
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
*/

#include "player.h"

#include <list>
#include <set>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <alsa/asoundlib.h>

#include "alsa.h"
#include "formats.h"
#include "settings.h"

struct Samples {
    lansink::Packet_Kind kind;
    uint64_t cTimestamp;
    std::string data;
    size_t cOffset;
    bool bRunning;

    Samples(lansink::Packet &_packet) :
        kind(_packet.kind()), cTimestamp(_packet.timestamp()),
        data(std::move(*_packet.mutable_samples())), cOffset(0),
        bRunning(_packet.running())
    {
    }

    bool operator <(const Samples &_other) const {
        if (cTimestamp + cOffset != _other.cTimestamp + cOffset)
            return cTimestamp + cOffset < _other.cTimestamp + cOffset;

        return kind > _other.kind;
    }

    // TODO rename
    const char *getData(size_t _cFrameBytes) const {
        return data.c_str() + cOffset*_cFrameBytes;
    }

    size_t getFrameCount(size_t _cFrameBytes) const {
        return data.size()/_cFrameBytes - cOffset;
    }
};

template<class _T>
struct PtrLess {
    bool operator ()(const _T *_pLhs, const _T *_pRhs) const {
        return *_pLhs < *_pRhs;
    }
};

typedef std::set<Samples *, PtrLess<Samples> > SampleQueue;

constexpr size_t c_cBaseFramesAdjustmentPacketCount = 20;
constexpr snd_pcm_sframes_t c_nBaseFramesAdjustmentThreshold = 10;

class Player::Impl {
public:
    Impl(uint64_t _cStreamId, Log &_log) :
        m_cStreamId(_cStreamId), m_pPcm(nullptr), m_cBufferSize(2048),
        m_cPeriodSize(512), m_format(SND_PCM_FORMAT_UNKNOWN), m_cRate(0), m_cChannelCount(0),
        m_cFrameBytes(0), m_cFramesWritten(0), m_pLog(&_log), m_bReady(false), m_bPaused(false),
        m_bClosed(false), m_bStarted(false), m_nLastError(0), m_bProcessCommands(false),
        m_nBufferedFrames(0), m_averageDelay(c_cBaseFramesAdjustmentPacketCount) {}

    ~Impl() {
        if (m_pPcm) {
            ALSA::drain(m_pPcm);
            ALSA::close(m_pPcm);
        }
        m_pPcm = nullptr;
    }

    static Player *get(lansink::Packet &_packet, Log &_log);
    static void remove(uint64_t _cId);
    static bool remove_stopped(TimePoint &_closeTime);

    bool is_prepared() const {
        return m_pPcm != nullptr;
    }

    void init(lansink::Packet &_packet);
    void run();
    void play(lansink::Packet &_packet);
    uint64_t get_id() const { return m_cStreamId; }

private:
    // TODO rearrange fields.
    uint64_t m_cStreamId;
    snd_pcm_t *m_pPcm;
    snd_pcm_uframes_t m_cBufferSize;
    snd_pcm_uframes_t m_cPeriodSize;
    SampleQueue m_queue;
    snd_pcm_format_t m_format;
    unsigned int m_cRate;
    size_t m_cChannelCount;
    size_t m_cFrameBytes;
    size_t m_cFramesWritten;
    snd_pcm_sframes_t m_nFramesBase;
    std::thread m_worker;
    mutable std::mutex m_mutex;
    Log *m_pLog;
    bool m_bReady;
    bool m_bPaused;
    bool m_bClosed;
    bool m_bStarted;
    std::condition_variable m_dataAvailable;
    int m_nLastError;
    bool m_bProcessCommands;
    snd_pcm_sframes_t m_nBufferedFrames;
    MovingAverage<snd_pcm_sframes_t> m_averageDelay;

    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::duration<int, std::milli> MilliSeconds;
    typedef std::chrono::time_point<Clock> TimePoint;

    TimePoint m_lastWrite, m_startTime;
    Clock::duration m_elapsed;

    typedef std::map<uint64_t, Player *> Players;
    static Players s_players;
    static std::mutex s_playerMapMutex;

    void _add_samples(size_t _cFrames);
    void _add_silence(size_t _cFrames);
    void _handle_commands();
    bool _handle_command(lansink::Packet_Kind _kind, Samples *_pPacket);
    const Samples *_find_first_data_packet() const;
    snd_pcm_sframes_t _get_buffered_frames();
};

Player *Player::Impl::get(lansink::Packet &_packet, Log &_log) {
    std::lock_guard<std::mutex> lock(s_playerMapMutex);

    auto player = s_players.insert(std::make_pair(_packet.stream(), nullptr));

    if (player.second) {
        if (g_settings.bExclusive && s_players.size() > 1) {
            s_players.erase(player.first);
            return nullptr;
        }

        player.first->second = new Player(_packet.stream(), _log);
    }

    return player.first->second;
}

void Player::Impl::remove(uint64_t _cId) {
    std::lock_guard<std::mutex> lock(s_playerMapMutex);
    auto iPlayer = s_players.find(_cId);

    if (iPlayer == s_players.end())
        return;

    delete iPlayer->second;
    s_players.erase(iPlayer);
}

bool Player::Impl::remove_stopped(TimePoint &_closeTime) {
    std::lock_guard<std::mutex> lock(s_playerMapMutex);
    bool bNoErrors = true;

    _closeTime = Clock::now();

    for (auto iPlayer = s_players.begin(); iPlayer != s_players.end();)
        if (!iPlayer->second->is_prepared()) {
            if (iPlayer->second->m_pImpl->m_worker.joinable())
                iPlayer->second->m_pImpl->m_worker.join();

            if (iPlayer->second->m_pImpl->m_lastWrite < _closeTime)
                _closeTime = iPlayer->second->m_pImpl->m_lastWrite;

            if (iPlayer->second->m_pImpl->m_nLastError != 0)
                bNoErrors = false;

            delete iPlayer->second;
            iPlayer = s_players.erase(iPlayer);
        } else
            ++iPlayer;

    return bNoErrors;
}

void Player::Impl::init(lansink::Packet &_packet) {
    try {
        snd_pcm_hw_params_t *pParams;

        m_cChannelCount = _packet.channels();

        ALSA::open(&m_pPcm, g_settings.strALSADevice.c_str(), SND_PCM_STREAM_PLAYBACK, 0);
        ALSA::hw_params_malloc(&pParams);
        ALSA::hw_params_any(m_pPcm, pParams);
        ALSA::hw_params_set_access(m_pPcm, pParams, SND_PCM_ACCESS_RW_INTERLEAVED);
        ALSA::hw_params_set_format(m_pPcm, pParams, get_format(_packet.format()));
        m_cRate = _packet.rate();
        ALSA::hw_params_set_rate_near(m_pPcm, pParams, &m_cRate, 0);
        ALSA::hw_params_set_channels(m_pPcm, pParams, _packet.channels());
        ALSA::hw_params_set_buffer_size_near(m_pPcm, pParams, &m_cBufferSize);
        ALSA::hw_params_set_period_size_near(m_pPcm, pParams, &m_cPeriodSize, NULL);
        ALSA::hw_params(m_pPcm, pParams);
        ALSA::hw_params_free(pParams);

        snd_pcm_sw_params_t *pSWParams;

        snd_pcm_sw_params_alloca(&pSWParams);
        ALSA::sw_params_current(m_pPcm, pSWParams);
        ALSA::sw_params_set_start_threshold(m_pPcm, pSWParams, m_cBufferSize*2);
        ALSA::sw_params(m_pPcm, pSWParams);

        ALSA::prepare(m_pPcm);

        m_cFrameBytes = ALSA::format_physical_width(get_format(_packet.format()))*_packet.channels()/8;
        m_cFramesWritten = 0;
        m_nFramesBase = std::numeric_limits<snd_pcm_sframes_t>::max();
        m_bReady = true;
        m_bProcessCommands = false;
        m_queue.clear();
        m_elapsed = Clock::duration(0);
        m_startTime = TimePoint();
        m_bPaused = true;
        m_bStarted = false;
        m_nBufferedFrames = 0;
        m_averageDelay.clear();
        m_pLog->info("Opened ALSA device \"%s\"", g_settings.strALSADevice.c_str());
    } catch (std::exception &e) {
        m_pLog->error(e.what());
        m_bReady = false;

        if (m_pPcm) {
            ALSA::close(m_pPcm);
            m_pPcm = nullptr;
        }
    }
}

void Player::Impl::play(lansink::Packet &_packet) {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_bClosed)
        return;

    m_pLog->debug("version = %u; stream = %llu; kind = %d, format = %s, channels = %u; rate = %u; timestamp = %llu; frames = %u",
            _packet.version(), _packet.stream(), _packet.kind(), _packet.format().c_str(),
            _packet.channels(), _packet.rate(), _packet.timestamp(), _packet.samples().size());

    if (!m_bStarted) {
        m_nFramesBase = std::min<snd_pcm_sframes_t>(_packet.timestamp(), m_nFramesBase);
        m_cFramesWritten = m_nFramesBase;
    }

    if (_packet.kind() != lansink::Packet_Kind_DATA) {
        if (m_queue.insert(new Samples(_packet)).second)
            m_bProcessCommands = true;
    } else {
        Samples *pSamples = new Samples(_packet);

        if (pSamples->bRunning == m_bPaused)
            m_bProcessCommands = true;

        m_queue.insert(pSamples);
    }

    if (!m_queue.empty())
        m_dataAvailable.notify_all();
}

void Player::Impl::run() {
    m_nLastError = 0;

    m_worker = std::thread([&]() {
        try {
            bool bPrevPaused = m_bPaused;

            while (true) {
                // TODO check if device can be paused.
                if (m_bStarted && m_bPaused != bPrevPaused) {
                    std::lock_guard<std::mutex> lock(m_mutex);

                    m_pLog->info("%s stream %llu", m_bPaused ? "Pausing" : "Unpausing",
                            m_cStreamId);

                    if (m_bPaused)
                        m_nBufferedFrames = _get_buffered_frames();
                    else
                        m_averageDelay.clear();

                    ALSA::pause(m_pPcm, m_bPaused);
                }

                if (m_bClosed) {
                    std::lock_guard<std::mutex> lock(m_mutex);

                    if (m_nLastError != 0) {
                        m_pLog->info("Closing stream %llu", m_cStreamId);
                        ALSA::close(m_pPcm);
                    } else {
                        m_pLog->info("Draining stream %llu", m_cStreamId);
                        ALSA::drain(m_pPcm);
                        ALSA::close(m_pPcm);
                        // FIXME preserve unplayed queued packets that possibly are from new connection.
                    }

                    m_pPcm = nullptr;
                    break;
                }

                bPrevPaused = m_bPaused;

                if (m_nLastError < 0) {
                    std::unique_lock<std::mutex> lock(m_mutex);

                    m_pLog->info("Waiting for recovery (timeout %d seconds)",
                            g_settings.nRecoveryTimeout);

                    if (m_queue.empty())
                        m_dataAvailable.wait_for(lock,
                                std::chrono::seconds(g_settings.nRecoveryTimeout),
                                [&]() { return !m_queue.empty(); });

                    if (m_queue.empty()) {
                        m_pLog->info("Dropping stream %llu", m_cStreamId);
                        ALSA::close(m_pPcm);
                        m_pPcm = nullptr;
                        break;
                    }

                    // Try to recover if error occurred.
                    m_pLog->info("Recovering...");

                    ALSA::recover(m_pPcm, m_nLastError, true);
                    m_nLastError = 0;
                    m_nBufferedFrames = 0;

                    continue;
                }

                try {
                    ALSA::wait(m_pPcm, 1000);

                    std::unique_lock<std::mutex> lock(m_mutex);
                    snd_pcm_sframes_t nDelay;

                    if (m_queue.empty()) {
                        if (m_bPaused)
                            nDelay = m_cPeriodSize; // Don't wake up early if we're paused anyway.
                        else
                            nDelay = _get_buffered_frames();

                        // Try to wakeup before underrun happens.
                        constexpr int c_nMarginMS = 5;
                        MilliSeconds ms(std::chrono::milliseconds(
                                std::max<int>(0, nDelay*1000/m_cRate - c_nMarginMS)));

                        m_pLog->debug("Queue empty, waiting for data %d ms", ms.count());
                        m_dataAvailable.wait_for(lock, ms,
                                [&]() { return !m_queue.empty(); });
                    }

                    if (m_queue.empty() && m_bPaused)
                        continue;

                    snd_pcm_sframes_t nFrames = ALSA::avail_update(m_pPcm);

                    m_pLog->debug("%ld frames can be written", nFrames);

                    // Otherwise fill up initial portion.
                    if (nFrames > 0 && m_cFramesWritten >= 2*m_cPeriodSize)
                        nFrames = nFrames > (snd_pcm_sframes_t)m_cPeriodSize ? m_cPeriodSize : nFrames;

                    _add_samples(nFrames);
                    m_lastWrite = Clock::now();
                    nDelay = _get_buffered_frames();

#ifndef NDEBUG
                    MilliSeconds ms(std::chrono::duration_cast<MilliSeconds>(Clock::now() - m_lastWrite));
                    m_pLog->debug("Time since last write: %d ms; buffered frames: %ld; state: %d",
                            ms.count(), nDelay, snd_pcm_state(m_pPcm));
#endif

                    if (ALSA::state(m_pPcm) == SND_PCM_STATE_PREPARED && !m_bPaused) {
                        m_pLog->info("Startng playback (delay: %ld)", nDelay);
                        ALSA::start(m_pPcm);
                        m_bStarted = true;
                        m_bPaused = false;
                        bPrevPaused = false;
                    }
                } catch (ALSA::Error &e) {
                    m_pLog->warning(e.what());

                    if (e.get_error() == -EPIPE) {
                        MilliSeconds ms(std::chrono::duration_cast<MilliSeconds>(Clock::now() - m_lastWrite));
                        m_pLog->warning("XRUN: %d ms since last write", ms.count());
                    } else if (e.get_error() == -EIO) {
                        // Cannot recover from EIO: someone tells us device was unplugged.
                        m_bClosed = true;
                    }

                    std::lock_guard<std::mutex> lock(m_mutex);

                    m_nLastError = e.get_error();
                }
            }
        } catch (std::exception &e) {
            m_pLog->error(e.what());
        }
    });
}

bool Player::Impl::_handle_command(lansink::Packet_Kind _kind, Samples *_pPacket) {
    switch (_kind) {
        case lansink::Packet_Kind_PAUSE:
            if (!m_bPaused) {
                m_elapsed += Clock::now() - m_startTime;
                m_startTime = TimePoint();
            }

            m_bPaused = true;
            break;

        case lansink::Packet_Kind_START:
            if (m_bPaused)
                m_startTime = Clock::now();

            m_bPaused = false;
            break;

        case lansink::Packet_Kind_STOP:
            m_bClosed = true;
            break;

        default:
            throw LogicError("Unexpected packet kind %d", _kind);
            return false;
            break;
    }

    return true;
}

void Player::Impl::_handle_commands() {
    for (auto iSamples = m_queue.begin(); iSamples != m_queue.end();) {
        Samples *pSamples = *iSamples;

        if (pSamples->kind == lansink::Packet_Kind_DATA) {
            if (pSamples->bRunning == m_bPaused)
                _handle_command(pSamples->bRunning ? lansink::Packet_Kind_START :
                        lansink::Packet_Kind_PAUSE, pSamples);

            ++iSamples;
            continue;
        }

        if (_handle_command(pSamples->kind, pSamples))
            if (pSamples->kind == lansink::Packet_Kind_STOP)
                return;

        iSamples = m_queue.erase(iSamples);
    }

    m_bProcessCommands = false;
}

void Player::Impl::_add_silence(size_t _cFrames) {
    size_t cPosition = 0;

    if (_cFrames > 0)
        m_pLog->debug("Inserting silence: %lu frames", _cFrames);

    while (cPosition < _cFrames)
        try {
            const size_t cSilenceFrames = _cFrames - cPosition;
            const size_t cSilenceBytes = cSilenceFrames*m_cFrameBytes;
            assert(cSilenceBytes < 1024*1024*100);
            std::unique_ptr<char []> pBuf = std::unique_ptr<char []>(new char[cSilenceBytes]);

            memset(pBuf.get(), 0, cSilenceBytes);
            cPosition += ALSA::writei(m_pPcm, pBuf.get(), cSilenceFrames);
        } catch (ALSA::Error &e) {
           if (e.get_error() == -EPIPE) {
               MilliSeconds ms(std::chrono::duration_cast<MilliSeconds>(Clock::now() - m_lastWrite));
               m_pLog->warning("XRUN: writei(), %d ms since last write", ms.count());
               ALSA::prepare(m_pPcm);
           } else
               throw;
        }
}

snd_pcm_sframes_t Player::Impl::_get_buffered_frames() {
    snd_pcm_sframes_t nDelay;
    const snd_pcm_state_t state = ALSA::state(m_pPcm);

    if (state == SND_PCM_STATE_PREPARED || state == SND_PCM_STATE_RUNNING)
        try {
            ALSA::delay(m_pPcm, &nDelay);
            m_nBufferedFrames = nDelay;
        } catch (ALSA::Error &e) {
            if (e.get_error() == -EIO) {
                m_pLog->warning(e.what());
                nDelay = m_nBufferedFrames;
            } else
                throw;
        }
    else if (state == SND_PCM_STATE_PAUSED)
        nDelay = m_nBufferedFrames;
    else
        nDelay = 0;

    return nDelay;
}

void Player::Impl::_add_samples(size_t _cFrames) {
    assert(m_cFrameBytes > 0);

    if (m_bProcessCommands)
        _handle_commands();

    if (m_queue.empty()) {
        if (!m_bPaused) {
            // TODO don't insert more than period size.
            m_pLog->debug("Avoiding underrun (%lu frames max)", _cFrames);
            _add_silence(_cFrames);
            m_cFramesWritten += _cFrames;
            m_nBufferedFrames += _cFrames;
        }

        return;
    }

    Clock::duration position = m_elapsed;

    if (!m_bPaused)
        position += (Clock::now() - m_startTime);

    const snd_pcm_sframes_t nDelay = _get_buffered_frames();
    typedef Clock::duration::period Period;
    snd_pcm_sframes_t nPosition = nDelay + m_nFramesBase +
            m_cRate*position.count()*Period::num/Period::den;

    // Work around latency introduced by program execution.
    const snd_pcm_sframes_t c_nThreshold = m_cPeriodSize;

    while (!m_queue.empty() && _cFrames > 0) {
        Samples *pSamples = *m_queue.begin();
        const snd_pcm_sframes_t nNext = pSamples->cTimestamp + pSamples->cOffset;
        const snd_pcm_sframes_t nPacketDelay = nNext - nPosition;

        m_pLog->debug("Processing packet: timestamp = %d, offset = %lu, position = %ld, "
                "packet delay = %ld, average delay = %.2f", pSamples->cTimestamp,
                pSamples->cOffset, nPosition, nPacketDelay, m_averageDelay.get());

        if (nNext > nPosition + c_nThreshold) {
            // Pad with silence.
            const size_t cFrames = std::min<size_t>(nNext - nPosition, _cFrames);

            m_pLog->debug("Padding till %ld (%lu frames max)", nNext, _cFrames);
            _add_silence(cFrames);
            m_cFramesWritten += cFrames;
            m_nBufferedFrames += _cFrames;
            nPosition += cFrames;
            _cFrames -= cFrames;

            if (_cFrames == 0)
                break;
        } else if (nNext + c_nThreshold < nPosition) {
            // Shift offset.
            const size_t cFrames = nPosition - nNext;

            m_pLog->debug("Shifting offset till %ld (%lu frames max)", nPosition,
                    pSamples->getFrameCount(m_cFrameBytes));

            if (cFrames >= pSamples->getFrameCount(m_cFrameBytes)) {
                delete pSamples;
                m_queue.erase(m_queue.begin());
                continue;
            }

            pSamples->cOffset += cFrames;
        }

        m_queue.erase(m_queue.begin());

        if (!m_bPaused) {
            m_averageDelay.add(nPacketDelay);

            if (m_averageDelay.is_full() &&
                    std::abs(m_averageDelay.get()) > c_nBaseFramesAdjustmentThreshold)
            {
                const snd_pcm_sframes_t nAdjustment = m_averageDelay.get();
                m_pLog->debug("Adjusting base frame count: %ld", nAdjustment);
                m_nFramesBase += nAdjustment;
                nPosition += nAdjustment;
                m_averageDelay.clear();
            }
        }

        const size_t cWriteSize = std::min(_cFrames, pSamples->getFrameCount(m_cFrameBytes));

        m_pLog->debug("Inserting audio: %lu, %lu (%ld)",
                pSamples->cTimestamp + pSamples->cOffset, cWriteSize, nPosition);

        try {
            const size_t cWritten = ALSA::writei(m_pPcm,
                    pSamples->getData(m_cFrameBytes), cWriteSize);

            pSamples->cOffset += cWritten;
            m_cFramesWritten += cWritten;
            m_nBufferedFrames += _cFrames;
            _cFrames -= cWritten;
            nPosition += cWritten;
        } catch (ALSA::Error &e) {
           if (e.get_error() == -EPIPE) {
               MilliSeconds ms(std::chrono::duration_cast<MilliSeconds>(Clock::now() - m_lastWrite));
               m_pLog->warning("XRUN: writei(), %d ms since last write", ms.count());
               ALSA::prepare(m_pPcm);
           } else
               throw;
        }

        if (pSamples->getFrameCount(m_cFrameBytes) > 0)
            m_queue.insert(pSamples);
        else
            delete pSamples;
    }
}

Player::Impl::Players Player::Impl::s_players{};
std::mutex Player::Impl::s_playerMapMutex{};

Player::Player(uint64_t _cStreamId, Log &_log) : m_pImpl(new Impl(_cStreamId, _log)) {
}

Player::~Player() {
    delete m_pImpl;
}

Player *Player::get(lansink::Packet &_packet, Log &_log, TimePoint &_closeTime) {
    if (!Player::remove_stopped(_closeTime))
        return nullptr;
    return Impl::get(_packet, _log);
}

void Player::remove(Player *_pPlayer) {
    Impl::remove(_pPlayer->get_id());
}

bool Player::remove_stopped(TimePoint &_closeTime) {
    return Impl::remove_stopped(_closeTime);
}

bool Player::is_prepared() const {
    return m_pImpl->is_prepared();
}

void Player::init(lansink::Packet &_packet) {
    m_pImpl->init(_packet);
}

void Player::run() {
    m_pImpl->run();
}

void Player::play(lansink::Packet &_packet) {
    m_pImpl->play(_packet);
}

uint64_t Player::get_id() const {
    return m_pImpl->get_id();
}
