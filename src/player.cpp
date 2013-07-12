/*
 * player.cpp
 *
 *  Created on: May 4, 2013
 *      Author: nikita.karnauhov@gmail.com
 */

#include "player.h"

#include <list>
#include <set>
#include <thread>
#include <mutex>

#include <alsa/asoundlib.h>

#include "alsa_wrapper.h"
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
        m_cStreamId(_cStreamId), m_pPcm(nullptr), m_cBufferSize(4096), m_cPeriodSize(256),
        m_format(SND_PCM_FORMAT_UNKNOWN), m_cRate(0), m_cChannelCount(0), m_cFrameBytes(0),
        m_cPosition(0), m_pLog(&_log), m_bReady(false), m_bPaused(false), m_bClosed(false) {}

    ~Impl() {
        ALSA::drain(m_pPcm);
        m_pPcm = nullptr;
    }

    static Player *get(unap::Packet &_packet, Log &_log);

    bool is_prepared() const {
        return m_pPcm != nullptr;
    }

    void init(unap::Packet &_packet);
    void run();
    void play(unap::Packet &_packet);

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

void Player::Impl::init(unap::Packet &_packet) {
    try {
        snd_pcm_hw_params_t *pParams;

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
        ALSA::prepare(m_pPcm);

        m_cFrameBytes = ALSA::format_physical_width(get_format(_packet.format()))*_packet.channels()/8;
        m_cPosition = 0;
        m_bReady = true;
        m_pLog->info("Opened ALSA device \"%s\"", g_settings.strALSADevice.c_str());
    } catch (std::exception &e) {
        m_pLog->error(e.what());
        m_bReady = false;
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

        if (m_cPosition == 0)
            m_cPosition = _packet.timestamp();
    }

    while (m_cPosition < 2*m_cPeriodSize) {
        if (m_queue.empty())
            return;

        _add_samples(2*m_cPeriodSize - m_cPosition, true);
    }

    // Dump data.
//        static int nCount = 0;
//        std::stringstream ss;
//        ss << "dump_" << std::setw(10) << std::setfill('0') << ++nCount << ".pcm";
//        std::ofstream ofs(ss.str());
//        ofs.write(_packet.samples().c_str(), _packet.samples().size());
//        size_t cFrames = _packet.samples().size()/4;
//
//        std::cerr << cFrames << std::endl;
//
//        if ((err = snd_pcm_writei(playback_handle, _packet.samples().c_str(), cFrames)) != cFrames) {
//            fprintf (stderr, "write to audio interface failed (%s)\n",
//                 snd_strerror (err));
//        }
}

void Player::Impl::run() {
    m_worker = std::thread([&]() {
        try {
            int nLastError = 0;
            constexpr size_t c_cMaxRetries = 1000;
            size_t cRetry = 0;
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
                        break;
                    }

                    bPrevPaused = m_bPaused;
                }

                if (nLastError < 0) {
                    bool bEmpty = false;

                    ++cRetry;

                    {
                        std::lock_guard<std::mutex> lock(m_mutex);
                        bEmpty = m_queue.empty();
                    }

                    // Sleep until recovered.
                    if (bEmpty) {
                        if (cRetry > c_cMaxRetries) {
                            m_pLog->info("Dropping stream %llu", m_cStreamId);
                            ALSA::close(m_pPcm);
                            break;
                        }

                        // TODO any way to auto-release mutex before going to sleep? condwait?
                        m_pLog->log(cRetry == 1 ? llInfo : llDebug, "Sleeping until recovered...");
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        continue;
                    }

                    // Try to recover if error occurred.
                    m_pLog->info("Recovering...");

                    ALSA::recover(m_pPcm, nLastError, true);
                    nLastError = 0;

                    // TODO timeout to drop the stream.
                    continue;
                }

                try {
                    ALSA::wait(m_pPcm, 1000);

                    // Initial portion should be filled by play().
                    if (m_cPosition == 0)
                        continue;

                    std::lock_guard<std::mutex> lock(m_mutex);

                    snd_pcm_sframes_t nFrames = ALSA::avail_update(m_pPcm);

                    if (nFrames > 0 && !m_queue.empty()) {
                        nFrames = nFrames > (snd_pcm_sframes_t)m_cPeriodSize ? m_cPeriodSize : nFrames;
                        _add_samples(nFrames, true);
                    }
                } catch (ALSA::Error &e) {
                    m_pLog->warning(e.what());
                    nLastError = e.getError();
                    continue;
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
        while (m_cPosition < cNext) {
            static char buf[16] = {0};

            if (ALSA::writei(m_pPcm, buf, 1) != 1)
                break;

            ++m_cPosition;

            if (m_cPosition >= cEnd)
                break;
        }

        if (!pSamples)
            break;

        // Fill up buffer.
        const size_t cFramesQueued = pSamples->data.size()/m_cFrameBytes - pSamples->cOffset;
        const size_t cFramesRequested = cEnd - m_cPosition;

        if (const size_t cWriteSize = std::min(cFramesRequested, cFramesQueued)) {
            m_pLog->debug("Inserting audio: %d, %d (%d)",
                    pSamples->cTimestamp + pSamples->cOffset, cWriteSize, m_cPosition);

            const size_t cBytesWritten = ALSA::writei(m_pPcm,
                    pSamples->getData(m_cFrameBytes), cWriteSize);

            pSamples->cOffset += cBytesWritten;
            m_cPosition += cBytesWritten;
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
    return Impl::get(_packet, _log);
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
