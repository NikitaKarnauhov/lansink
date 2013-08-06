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
#include <chrono>
#include <condition_variable>

#include <alsa/asoundlib.h>

#include "alsa.h"
#include "formats.h"
#include "settings.h"

struct Samples {
    unap::Packet_Kind kind;
    uint64_t cTimestamp;
    std::string data;
    size_t cOffset;

    Samples(unap::Packet &_packet) :
        kind(_packet.kind()), cTimestamp(_packet.timestamp()),
        data(std::move(*_packet.mutable_samples())), cOffset(0)
    {
    }

    bool operator <(const Samples &_other) const {
        return cTimestamp < _other.cTimestamp;
    }

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

class Player::Impl {
public:
    Impl(uint64_t _cStreamId, Log &_log) :
        m_cStreamId(_cStreamId), m_pPcm(nullptr), m_cBufferSize(2048), m_cPeriodSize(512),
        m_format(SND_PCM_FORMAT_UNKNOWN), m_cRate(0), m_cChannelCount(0), m_cFrameBytes(0),
        m_cPosition(0), m_pLog(&_log), m_bReady(false), m_bPaused(false), m_bClosed(false) {}

    ~Impl() {
        if (m_pPcm)
            ALSA::drain(m_pPcm);
        m_pPcm = nullptr;
    }

    static Player *get(unap::Packet &_packet, Log &_log);
    static void remove(uint64_t _cId);
    static void remove_stopped();

    bool is_prepared() const {
        return m_pPcm != nullptr;
    }

    void init(unap::Packet &_packet);
    void run();
    void play(unap::Packet &_packet);
    uint64_t get_id() const { return m_cStreamId; }

private:
    uint64_t m_cStreamId;
    snd_pcm_t *m_pPcm;
    snd_pcm_uframes_t m_cBufferSize;
    snd_pcm_uframes_t m_cPeriodSize;
    SampleQueue m_queue;
    snd_pcm_format_t m_format;
    unsigned int m_cRate;
    size_t m_cChannelCount;
    size_t m_cFrameBytes;
    size_t m_cPosition;
    std::thread m_worker;
    mutable std::mutex m_mutex;
    Log *m_pLog;
    bool m_bReady;
    bool m_bPaused;
    bool m_bClosed;
    std::condition_variable m_dataAvailable;

    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::duration<int, std::milli> Duration;
    typedef std::chrono::time_point<Clock> TimePoint;

    TimePoint m_lastWrite;

    typedef std::map<uint64_t, Player *> Players;
    static Players s_players;
    static std::mutex s_playerMapMutex;

    void _add_samples(size_t _cSamples, bool _bStopWhenEmpty);
};

Player *Player::Impl::get(unap::Packet &_packet, Log &_log) {
    std::lock_guard<std::mutex> lock(s_playerMapMutex);
    auto player = s_players.insert(std::make_pair(_packet.stream(), nullptr));

    if (player.second)
        player.first->second = new Player(_packet.stream(), _log);

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

void Player::Impl::remove_stopped() {
    std::lock_guard<std::mutex> lock(s_playerMapMutex);

    for (auto iPlayer = s_players.begin(); iPlayer != s_players.end();)
        if (!iPlayer->second->is_prepared()) {
            if (iPlayer->second->m_pImpl->m_worker.joinable())
                iPlayer->second->m_pImpl->m_worker.join();
            delete iPlayer->second;
            iPlayer = s_players.erase(iPlayer);
        } else
            ++iPlayer;
}

void Player::Impl::init(unap::Packet &_packet) {
    try {
        snd_pcm_hw_params_t *pParams;

        m_cChannelCount = _packet.channels();

        ALSA::open(&m_pPcm, g_settings.strALSADevice.c_str(), SND_PCM_STREAM_PLAYBACK, 0);
        ALSA::hw_params_malloc(&pParams);
        ALSA::hw_params_any(m_pPcm, pParams);
        ALSA::hw_params_set_access(m_pPcm, pParams, SND_PCM_ACCESS_RW_INTERLEAVED);
        ALSA::hw_params_set_format(m_pPcm, pParams, get_format(_packet.format()));

        unsigned int cRate = _packet.rate();

        ALSA::hw_params_set_rate_near(m_pPcm, pParams, &cRate, 0);
        ALSA::hw_params_set_channels(m_pPcm, pParams, _packet.channels());
        ALSA::hw_params_set_buffer_size_near(m_pPcm, pParams, &m_cBufferSize);
        ALSA::hw_params_set_period_size_near(m_pPcm, pParams, &m_cPeriodSize, NULL);
        ALSA::hw_params(m_pPcm, pParams);
        ALSA::hw_params_free(pParams);

        snd_pcm_sw_params_t *pSWParams;

        snd_pcm_sw_params_alloca(&pSWParams);
        ALSA::sw_params_current(m_pPcm, pSWParams);
        ALSA::sw_params_set_start_threshold(m_pPcm, pSWParams,
                std::numeric_limits<snd_pcm_uframes_t>::max());
        ALSA::sw_params(m_pPcm, pSWParams);

        ALSA::prepare(m_pPcm);

        m_cFrameBytes = ALSA::format_physical_width(get_format(_packet.format()))*_packet.channels()/8;
        m_cPosition = 0;
        m_bReady = true;
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

void Player::Impl::play(unap::Packet &_packet) {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_bClosed)
        return;

    m_pLog->debug("version = %u; stream = %llu; kind = %d, format = %s, channels = %u; rate = %u; timestamp = %llu; frames = %u",
            _packet.version(), _packet.stream(), _packet.kind(), _packet.format().c_str(),
            _packet.channels(), _packet.rate(), _packet.timestamp(), _packet.samples().size());

    if (_packet.kind() != unap::Packet_Kind_DATA && m_cPosition != 0)
        _packet.set_timestamp(m_cPosition);

    if (m_cPosition <= _packet.timestamp()) {
        m_queue.insert(new Samples(_packet));

        if (m_cPosition < (*m_queue.begin())->cTimestamp)
            m_cPosition = (*m_queue.begin())->cTimestamp;
    }

    if (!m_queue.empty())
        m_dataAvailable.notify_all();
}

void Player::Impl::run() {
    m_worker = std::thread([&]() {
        try {
            int nLastError = 0;
            bool bPrevPaused = false;

            while (true) {
                {
                    std::lock_guard<std::mutex> lock(m_mutex);

                    if (m_bPaused != bPrevPaused) {
                        m_pLog->info("%s stream %llu", m_bPaused ? "Pausing" : "Unpausing",
                                m_cStreamId);
                        ALSA::pause(m_pPcm, m_bPaused);
                    }

                    if (m_bClosed) {
                        m_pLog->info("Closing stream %llu", m_cStreamId);
                        ALSA::close(m_pPcm);
                        m_pPcm = nullptr;
                        break;
                    }

                    bPrevPaused = m_bPaused;
                }

                if (nLastError < 0) {
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
                        break;
                    }

                    // Try to recover if error occurred.
                    m_pLog->info("Recovering...");

                    ALSA::recover(m_pPcm, nLastError, true);
                    nLastError = 0;

                    continue;
                }

                try {
                    ALSA::wait(m_pPcm, 1000);

                    std::lock_guard<std::mutex> lock(m_mutex);

                    if (m_queue.empty())
                        continue;

                    snd_pcm_sframes_t nFrames = ALSA::avail_update(m_pPcm);

                    m_pLog->debug("%d frames can be written", nFrames);

                    // Otherwise fill up initial portion.
                    if (nFrames > 0 && m_cPosition >= 2*m_cPeriodSize)
                        nFrames = nFrames > (snd_pcm_sframes_t)m_cPeriodSize ? m_cPeriodSize : nFrames;

                    snd_pcm_sframes_t nDelay;

#ifndef NDEBUG
                    ALSA::delay(m_pPcm, &nDelay);
                    Duration ms(std::chrono::duration_cast<Duration>(Clock::now() - m_lastWrite));
                    m_pLog->debug("Time since last write: %d ms; delay: %d; state: %d",
                            ms.count(), nDelay, snd_pcm_state(m_pPcm));
#endif

                    _add_samples(nFrames, true);
                    m_lastWrite = Clock::now();

                    ALSA::delay(m_pPcm, &nDelay);

                    if (ALSA::state(m_pPcm) == SND_PCM_STATE_PREPARED &&
                            nDelay >= (snd_pcm_sframes_t)m_cBufferSize/2)
                    {
                        m_pLog->info("Startng playback (delay: %d)", nDelay);
                        ALSA::start(m_pPcm);
                    }
                } catch (ALSA::Error &e) {
                    m_pLog->warning(e.what());

                    if (e.get_error() == -EPIPE) {
                        Duration ms(std::chrono::duration_cast<Duration>(Clock::now() - m_lastWrite));
                        m_pLog->warning("XRUN: %d ms since last write", ms.count());
                    }

                    nLastError = e.get_error();
                }
            }
        } catch (std::exception &e) {
            m_pLog->error(e.what());
        }
    });
}

void Player::Impl::_add_samples(size_t _cSamples, bool _bStopWhenEmpty) {
    const size_t cEnd = m_cPosition + _cSamples;

    assert(m_cFrameBytes > 0);

    // Handle control packets.
    for (auto iSamples = m_queue.begin(); iSamples != m_queue.end();) {
        Samples *pSamples = *iSamples;

        if (pSamples->kind == unap::Packet_Kind_DATA) {
            ++iSamples;
            continue;
        }

        switch (pSamples->kind) {
            case unap::Packet_Kind_PAUSE:
                m_bPaused = true;
                break;
            case unap::Packet_Kind_UNPAUSE:
                m_bPaused = false;
                break;
            case unap::Packet_Kind_STOP:
                m_bClosed = true;
                return;
            default:
                throw LogicError("Unexpected packet kind %d", pSamples->kind);
                break;
        }

        iSamples = m_queue.erase(iSamples);
    }

    if (m_bPaused)
        return;

    while (m_cPosition < cEnd) {
        Samples *pSamples = m_queue.empty() ? nullptr : *m_queue.begin();
        size_t cNext = cEnd;

        if (pSamples) {
            cNext = pSamples->cTimestamp + pSamples->cOffset;
            m_queue.erase(m_queue.begin());
        }

        if (m_cPosition < cNext)
            m_pLog->debug("Inserting silence: %d, %d", m_cPosition, cNext - m_cPosition);

        // Insert silence instead of missing samples.
        try {
            while (m_cPosition < cNext) {
                std::unique_ptr<char []> pBuf = std::unique_ptr<char []>(new char[cEnd - m_cPosition]);
                const int nWritten = ALSA::writei(m_pPcm, pBuf.get(), cEnd - m_cPosition);

                m_cPosition += nWritten;

                if (m_cPosition >= cEnd)
                    break;
            }
        } catch (ALSA::Error &e) {
           if (e.get_error() == -EPIPE) {
               Duration ms(std::chrono::duration_cast<Duration>(Clock::now() - m_lastWrite));
               m_pLog->warning("XRUN: writei(), %d ms since last write", ms.count());
               ALSA::prepare(m_pPcm);
               m_cPosition = cNext;
           } else
               throw;
        }

        if (!pSamples)
            break;

        // Fill up buffer.
        const size_t cFramesQueued = pSamples->data.size()/m_cFrameBytes - pSamples->cOffset;
        const size_t cFramesRequested = cEnd - m_cPosition;

        if (const size_t cWriteSize = std::min(cFramesRequested, cFramesQueued)) {
            m_pLog->debug("Inserting audio: %d, %d (%d)",
                    pSamples->cTimestamp + pSamples->cOffset, cWriteSize, m_cPosition);

            try {
                const size_t cBytesWritten = ALSA::writei(m_pPcm,
                        pSamples->getData(m_cFrameBytes), cWriteSize);

                pSamples->cOffset += cBytesWritten;
                m_cPosition += cBytesWritten;
            } catch (ALSA::Error &e) {
               if (e.get_error() == -EPIPE) {
                   Duration ms(std::chrono::duration_cast<Duration>(Clock::now() - m_lastWrite));
                   m_pLog->warning("XRUN: writei(), %d ms since last write", ms.count());
                   ALSA::prepare(m_pPcm);
               } else
                   throw;
            }
        }

        if (pSamples->cOffset*m_cFrameBytes < pSamples->data.size())
            m_queue.insert(pSamples);
        else
            delete pSamples;

        if (m_queue.empty() && _bStopWhenEmpty)
            break;
    }
}

Player::Impl::Players Player::Impl::s_players{};
std::mutex Player::Impl::s_playerMapMutex{};

Player::Player(uint64_t _cStreamId, Log &_log) : m_pImpl(new Impl(_cStreamId, _log)) {
}

Player::~Player() {
    delete m_pImpl;
}

Player *Player::get(unap::Packet &_packet, Log &_log) {
    Player::remove_stopped();
    return Impl::get(_packet, _log);
}

void Player::remove(Player *_pPlayer) {
    Impl::remove(_pPlayer->get_id());
}

void Player::remove_stopped() {
    Impl::remove_stopped();
}

bool Player::is_prepared() const {
    return m_pImpl->is_prepared();
}

void Player::init(unap::Packet &_packet) {
    m_pImpl->init(_packet);
}

void Player::run() {
    m_pImpl->run();
}

void Player::play(unap::Packet &_packet) {
    m_pImpl->play(_packet);
}

uint64_t Player::get_id() const {
    return m_pImpl->get_id();
}
