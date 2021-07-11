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
#include <algorithm>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <cmath>

#include "utils.h"
#include "sink.h"
#include "settings.h"

struct Samples {
    lansink::Packet_Kind kind;
    uint64_t cTimestamp;
    std::string data;
    size_t cOffset = 0;
    bool bRunning;
    bool bDraining;

    Samples(lansink::Packet &_packet) :
        kind(_packet.kind()), cTimestamp(_packet.timestamp()),
        data(std::move(*_packet.mutable_samples())),
        bRunning(_packet.kind() != lansink::Packet_Kind_CACHE),
        bDraining(_packet.kind() == lansink::Packet_Kind_DRAIN)
    {
        if (kind == lansink::Packet_Kind_CACHE)
            kind = lansink::Packet_Kind_DATA;
        else if (kind == lansink::Packet_Kind_DRAIN)
            kind = lansink::Packet_Kind_STOP;
    }

    bool operator <(const Samples &_other) const {
        if (cTimestamp + cOffset != _other.cTimestamp + cOffset)
            return cTimestamp + cOffset < _other.cTimestamp + cOffset;

        // So that STOP packet comes the last with any given timestamp.
        return kind < _other.kind;
    }

    const char *get_data(size_t _cFrameBytes) const {
        return data.c_str() + cOffset*_cFrameBytes;
    }

    size_t get_frame_count(size_t _cFrameBytes) const {
        return data.size()/_cFrameBytes - cOffset;
    }
};

class SampleQueue : private std::set<Samples *, PtrLess<Samples> > {
private:
    using Base = std::set<Samples *, PtrLess<Samples> >;

public:
    using Iterator = Base::iterator;
    using ReverseIterator = Base::reverse_iterator;

    ~SampleQueue() { clear(); }

    void init(size_t _cFrameBytes, size_t _cRate);
    bool empty() const { return Base::empty(); }
    size_t size() const { return Base::size(); }
    size_t ms() const;
    bool has_commands() const { return m_bHaveCommands; }
    void clear_commands() { m_bHaveCommands = false; }
    Iterator begin() const { return Base::begin(); }
    Iterator end() const { return Base::end(); }
    ReverseIterator rbegin() const { return Base::rbegin(); }
    ReverseIterator rend() const { return Base::rend(); }
    Samples *front() const { return *Base::begin(); }
    void push(lansink::Packet &_packet, bool _bPaused);
    void push(Samples *_pSamples);
    Samples *pop();
    Iterator erase(Iterator _i);
    Iterator erase(Iterator _iBegin, Iterator _iEnd);
    void clear();

private:
    size_t m_cBytesPerSecond = 0;
    size_t m_cTotalBytes = 0;
    bool m_bHaveCommands = false;

    Iterator _erase(Iterator _i);
};

constexpr size_t c_cAveragePacketCount = 20;
constexpr long c_nBaseFramesAdjustmentThreshold = 10;

class Player::Impl {
public:
    Impl(uint64_t _cStreamId, Log &_log, const std::string &_strSender) :
        m_pSink(create_alsa_sink(_log)), m_cStreamId(_cStreamId),
        m_log(_log, LogDecorator{_strSender})
    {
    }

    ~Impl() {
        assert(!m_worker.joinable());
        delete m_pSink;
    }

    static Player *get(lansink::Packet &_packet, Log &_log,
            const std::string &_strSender);
    static void remove(uint64_t _cId);
    static bool remove_stopped(TimePoint &_closeTime);

    bool is_prepared() const {
        return m_pSink->is_prepared();
    }

    void init(lansink::Packet &_packet);
    void run();
    void play(lansink::Packet &_packet);
    uint64_t get_id() const { return m_cStreamId; }

private:
    // TODO rearrange fields.
    Sink *m_pSink;
    uint64_t m_cStreamId;
    SampleQueue m_queue;
    long m_nFramesWritten = -1;
    long m_nFramesBase = 0;
    long m_nLastFramesBuffered = 0;
    long m_nLastFramesWritten = 0;
    std::thread m_worker;
    mutable std::mutex m_mutex;
    mutable DecoratedLog m_log;
    bool m_bPaused = false;
    bool m_bEmulatedPause = false;
    bool m_bClosed = false;
    bool m_bStarted = false;
    bool m_bDraining = false;
    std::condition_variable m_dataAvailable;
    int m_nLastError = 0;
    MovingAverage<long> m_averageDelay{c_cAveragePacketCount};
    MovingAverage<long> m_averageSize{c_cAveragePacketCount};
    LostPacketDetector m_lostPacketDetector;
    size_t m_cFramesPadding = 0;
    size_t m_cFramesSkipped = 0;
    size_t m_cPacketsSkipped = 0;

    using Clock = Player::Clock;
    using TimePoint = Player::TimePoint;
    using MilliSeconds = std::chrono::milliseconds;

    TimePoint m_lastWrite, m_reportTime, m_xrunTime;

    using Players = std::map<uint64_t, Player *>;
    static Players s_players;
    static std::mutex s_playerMapMutex;

    void _prepare(bool _bDiscardQueue = true);
    void _add_samples(size_t _cFrames);
    void _add_silence(size_t _cFrames);
    bool _handle_commands();
    bool _handle_command(lansink::Packet_Kind _kind, Samples *_pPacket);
    const Samples *_find_first_data_packet() const;
    long _get_buffered_frames() const;
    long _get_available_frames(bool _bSync) const;
    long _estimate_position();
    void _add_frames_written(long _nFrames);
    void _set_frames_written(long _nFrames);
    void _log_stats();
    void _add_stats(const lansink::Packet &_packet);

    struct LogDecorator {
        const std::string m_strSender;
        std::string operator()(const std::string &_strMessage) {
            return _strMessage + " [" + m_strSender + "] ";
        }
    };
};

void SampleQueue::init(size_t _cFrameBytes, size_t _cRate) {
    m_cBytesPerSecond = _cFrameBytes*_cRate;
    clear();
}

void SampleQueue::push(lansink::Packet &_packet, bool _bPaused) {
    if (_packet.kind() != lansink::Packet_Kind_DATA &&
            _packet.kind() != lansink::Packet_Kind_CACHE)
    {
        if (Base::insert(new Samples(_packet)).second)
            m_bHaveCommands = true;
    } else {
        Samples *const pSamples = new Samples(_packet);

        if (pSamples->bRunning == _bPaused)
            m_bHaveCommands = true;

        if (Base::insert(pSamples).second)
            m_cTotalBytes += pSamples->data.size();
        else
            delete pSamples;
    }
}

void SampleQueue::push(Samples *_pSamples) {
    if (_pSamples->kind == lansink::Packet_Kind_DATA)
        m_cTotalBytes += _pSamples->data.size();
    Base::insert(_pSamples);
}

void SampleQueue::clear() {
    m_cTotalBytes = 0;
    m_bHaveCommands = false;
    for (Samples *const pSamples : *this)
        delete pSamples;
    Base::clear();
}

Samples *SampleQueue::pop() {
    Samples *const pSamples = front();
    _erase(Base::begin());
    return pSamples;
}

SampleQueue::Iterator SampleQueue::_erase(Iterator _i) {
    if ((*_i)->kind == lansink::Packet_Kind_DATA) {
        assert(m_cTotalBytes >= (*_i)->data.size());
        m_cTotalBytes -= (*_i)->data.size();
    }
    return Base::erase(_i);
}

SampleQueue::Iterator SampleQueue::erase(Iterator _i) {
    Samples *const pSamples = *_i;
    _i = _erase(_i);
    delete pSamples;
    return _i;
}

SampleQueue::Iterator SampleQueue::erase(
        Iterator _iBegin, Iterator _iEnd)
{
    while (_iBegin != _iEnd)
        _iBegin = erase(_iBegin);
    return _iBegin;
}

size_t SampleQueue::ms() const {
    assert(m_cBytesPerSecond > 0);
    return m_cTotalBytes*1000/m_cBytesPerSecond;
}

Player *Player::Impl::get(lansink::Packet &_packet, Log &_log,
        const std::string &_strSender)
{
    std::lock_guard<std::mutex> lock(s_playerMapMutex);

    auto player = s_players.insert(std::make_pair(_packet.stream(), nullptr));

    if (player.second) {
        if (g_settings.bExclusive && s_players.size() > 1) {
            s_players.erase(player.first);
            return nullptr;
        }

        player.first->second = new Player(_packet.stream(), _log, _strSender);
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

void Player::Impl::_prepare(bool _bDiscardQueue /*= true*/) {
    if (m_pSink->prepare()) {
        m_nFramesWritten = 0;
        m_nFramesBase = std::numeric_limits<long>::max();
        m_nLastFramesBuffered = 0;
        m_nLastFramesWritten = 0;
        m_lastWrite = TimePoint();
        m_xrunTime = TimePoint();
        m_reportTime = Clock::now();
        m_bPaused = true;
        m_bEmulatedPause = false;
        m_bStarted = false;
        m_bClosed = false;
        m_bDraining = false;
        m_averageDelay.clear();
        m_averageSize.clear();

        if (_bDiscardQueue)
            m_queue.init(m_pSink->get_frame_bytes(), m_pSink->get_rate());
    }
}

void Player::Impl::init(lansink::Packet &_packet) {
    m_pSink->init(_packet.channels(), _packet.rate(), _packet.format());
    _prepare();
}

void Player::Impl::_log_stats() {
    const size_t cFrameBytes = m_pSink->get_frame_bytes();

    if (!m_lostPacketDetector.empty()) {
        const auto stats = m_lostPacketDetector.collect();
        m_log.info("Received: %u packets; %u frames; %u bytes",
                stats.cPackets, stats.cSize, stats.cSize*cFrameBytes);
        m_log.info("Lost: %u packets; %u frames; %u bytes",
                stats.cPacketsLost, stats.cSizeLost,
                stats.cSizeLost*cFrameBytes);
        m_log.info("Duplicates: %u packets; %u frames; %u bytes",
                stats.cPacketsDuplicated, stats.cSizeDuplicated,
                stats.cSizeDuplicated*cFrameBytes);
        m_log.info("Overlaps: %u packets; %u frames; %u bytes",
                stats.cPacketsOverlapped, stats.cSizeOverlapped,
                stats.cSizeOverlapped*cFrameBytes);
    }

    m_log.info("Skipped: %u packets; %u frames; %u bytes",
            m_cPacketsSkipped, m_cFramesSkipped,
            m_cFramesSkipped*cFrameBytes);
    m_log.info("Padding: %u frames; %u bytes",
            m_cFramesPadding, m_cFramesPadding*cFrameBytes);
    m_cFramesPadding = 0;
    m_cFramesSkipped = 0;
    m_cPacketsSkipped = 0;
}

void Player::Impl::_add_stats(const lansink::Packet &_packet) {
    const size_t cFrameBytes = m_pSink->get_frame_bytes();

    if (m_lostPacketDetector.empty()) {
        const size_t cBytesPerMinute = m_pSink->get_rate()*61*cFrameBytes;
        const size_t cBytesPerPacket = std::max<size_t>(
                1024, _packet.samples().size());
        m_lostPacketDetector.reserve(cBytesPerMinute/cBytesPerPacket);
    }

    m_lostPacketDetector.add(_packet.timestamp(),
            _packet.samples().size()/cFrameBytes);
}

void Player::Impl::play(lansink::Packet &_packet) {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_bClosed)
        return;

    m_log.debug("Received: version = %u; stream = %llu; kind = %d, format = %s, "
            "channels = %u; rate = %u; timestamp = %llu; bytes = %u",
            _packet.version(), _packet.stream(), _packet.kind(),
            _packet.format().c_str(), _packet.channels(), _packet.rate(),
            _packet.timestamp(), _packet.samples().size());

    if (!m_bStarted) {
        m_nFramesBase = std::min<long>(_packet.timestamp(), m_nFramesBase);
        m_nFramesWritten = 0;
        m_nLastFramesBuffered = 0;
        m_nLastFramesWritten = 0;
    }

    if (!m_queue.has_commands() && (int)m_queue.ms() > 1000*g_settings.nBufferedTime) {
        m_log.warning("Discarding queue due to overrun (%u ms queued, %d ms max)",
                m_queue.ms(), 1000*g_settings.nBufferedTime);
        m_queue.clear();
    }

    if (_packet.kind() == _packet.DATA || _packet.kind() == _packet.CACHE)
        _add_stats(_packet);

    m_queue.push(_packet, m_bPaused);

    const std::chrono::minutes mins =
        std::chrono::duration_cast<std::chrono::minutes>(Clock::now() - m_reportTime);

    if (mins.count() > 0) {
        m_log.info("%u packets queued (%u ms)", m_queue.size(), m_queue.ms());
        _log_stats();
        m_reportTime = Clock::now();
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
                if (m_bStarted && m_bPaused != bPrevPaused) {
                    std::lock_guard<std::mutex> lock(m_mutex);

                    m_log.info("%s stream %llu", m_bPaused ? "Pausing" : "Unpausing",
                            m_cStreamId);

                    if (m_pSink->can_be_paused()) {
                        if (!m_bPaused)
                            m_averageDelay.clear();

                        m_pSink->pause(m_bPaused);
                    } else if (m_bPaused) {
                        m_log.warning("Device cannot be paused, emulating pause");
                        m_bEmulatedPause = true;
                    } else
                        m_bEmulatedPause = false;
                }

                if (m_bClosed) {
                    std::lock_guard<std::mutex> lock(m_mutex);

                    if (m_bDraining && m_nLastError == 0 && !m_bPaused) {
                        m_log.info("Draining stream %llu", m_cStreamId);

                        // Work around pulse_drain() lock up in PulseAudio output module.
                        // ALSA::drain(m_pPcm);

                        try {
                            const long nFrames = m_pSink->get_buffer_size() -
                                    _get_available_frames(true);
                            const MilliSeconds ms(std::chrono::milliseconds(nFrames*1000/m_pSink->get_rate()));

                            m_log.debug("Waiting %llu ms for playback to finish", ms.count());
                            std::this_thread::sleep_for(ms);
                        } catch (...) {
                            // Do nothing.
                        }

                        m_bDraining = false;
                    }

                    m_log.info("Closing stream %llu", m_cStreamId);
                    m_pSink->close();

                    if (!m_queue.empty()) {
                        // Already got new data from the stream, probably a rewind happened.
                        _prepare(false);

                        if (m_pSink->is_prepared()) {
                            m_log.info("New data after STOP message, reusing stream %llu", m_cStreamId);
                            bPrevPaused = m_bPaused;
                            m_nFramesBase = m_queue.front()->cTimestamp;
                            m_nFramesWritten = 0;
                            m_nLastFramesBuffered = 0;
                            m_nLastFramesWritten = 0;
                            continue;
                        }
                    }

                    break;
                }

                bPrevPaused = m_bPaused;

                if (m_nLastError < 0) {
                    std::unique_lock<std::mutex> lock(m_mutex);

                    m_log.info("Waiting for recovery (timeout %d seconds)",
                            g_settings.nRecoveryTimeout);

                    if (m_queue.empty())
                        m_dataAvailable.wait_for(lock,
                                std::chrono::seconds(g_settings.nRecoveryTimeout),
                                [&]() { return !m_queue.empty(); });

                    if (m_queue.empty()) {
                        m_log.info("Dropping stream %llu", m_cStreamId);
                        m_pSink->close();
                        break;
                    }

                    // Try to recover if error occurred.
                    m_log.info("Recovering...");

                    m_pSink->recover(m_nLastError);
                    m_nLastError = 0;
                    _set_frames_written(-1);

                    continue;
                }

                try {
                    std::unique_lock<std::mutex> lock(m_mutex);

                    if (!m_queue.empty() || !m_bPaused) {
                        long nFrames = _get_available_frames(true);

                        m_log.debug("%ld frames can be written", nFrames);

                        // Otherwise fill up initial portion.
                        if (nFrames > 0 && m_nLastFramesWritten >=
                                (long)(2*m_pSink->get_period_size()))
                            nFrames = nFrames > (long)m_pSink->get_period_size() ?
                                    m_pSink->get_period_size() : nFrames;

#ifndef NDEBUG
                        if (m_lastWrite != TimePoint()) {
                            const long nDelay = _get_buffered_frames();
                            long nFrames = _get_available_frames(true);
                            MilliSeconds ms(std::chrono::duration_cast<MilliSeconds>(Clock::now() - m_lastWrite));
                            m_log.debug("Time since last write: %d ms; delay: %ld; available frames: %ld",
                                    ms.count(), nDelay, nFrames);
                            m_pSink->report_state();
                        }
#endif

                        // Check uncaught underruns.
                        if (m_pSink->is_underrun()) {
                            m_log.warning("Uncaught XRUN");
                            m_pSink->prepare();
                            continue;
                        }

                        _add_samples(nFrames);

                        if (m_nFramesWritten >= 0) {
                            m_lastWrite = Clock::now();
                            m_nLastFramesWritten = m_nFramesWritten;
                            m_nLastFramesBuffered = _get_buffered_frames();
                        }

                        if (m_pSink->is_buffering() && !m_bPaused) {
                            m_log.info("Starting playback (delay: %ld)", _get_buffered_frames());
                            m_pSink->start();
                            m_bStarted = true;
                            bPrevPaused = false;
                        }
                    }

                    // Try to wakeup before underrun happens.
                    constexpr int c_nMarginMS = 20;
                    const long nMarginFrames = c_nMarginMS*m_pSink->get_rate()/1000;

                    // Don't wake up early if we're paused anyway.
                    const long nAvailableFrames = _get_available_frames(false);
                    const long nDelay = m_bPaused ? m_pSink->get_period_size() :
                            std::max<int>(0, m_pSink->get_buffer_size() - nAvailableFrames - nMarginFrames);
                    MilliSeconds ms((int)nDelay*1000/m_pSink->get_rate());

                    if (ms.count() > 0) {
                        m_log.debug("Waiting for data %d ms, %u packets queued, available = %ld, margin = %ld",
                                ms.count(), m_queue.size(), nAvailableFrames, nMarginFrames);
                        m_dataAvailable.wait_for(lock, ms);
                    } else if (m_queue.empty())
                        std::this_thread::yield();
                } catch (Sink::Error &e) {
                    m_log.warning(e.what());

                    if (e.is_underrun()) {
                        MilliSeconds ms(std::chrono::duration_cast<MilliSeconds>(Clock::now() - m_lastWrite));
                        m_log.warning("XRUN: %d ms since last write", ms.count());
                    } else if (e.is_fatal()) {
                        std::lock_guard<std::mutex> lock(m_mutex);
                        m_bClosed = true;
                        m_queue.clear();
                    }

                    std::lock_guard<std::mutex> lock(m_mutex);

                    m_nLastError = e.get_error();
                }
            }
        } catch (std::exception &e) {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_log.error(e.what());
            m_log.info("Closing stream %llu due to error", m_cStreamId);
            m_pSink->close();
            m_bClosed = true;
            m_queue.clear();
        }
    });

    // FIXME terminate called without an active exception
}

bool Player::Impl::_handle_command(lansink::Packet_Kind _kind, Samples *_pPacket) {
    switch (_kind) {
        case lansink::Packet_Kind_PAUSE:
            m_bPaused = true;
            break;

        case lansink::Packet_Kind_START:
            m_bPaused = false;
            break;

        case lansink::Packet_Kind_STOP:
            m_bClosed = true;
            m_bDraining = _pPacket->bDraining;
            break;

        default:
            throw LogicError("Unexpected packet kind %d", _kind);
            return false;
            break;
    }

    return true;
}

bool Player::Impl::_handle_commands() {
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
            if (pSamples->kind == lansink::Packet_Kind_STOP) {
                // Clear queue up to the last STOP packet.
                auto iLast = std::find_if(m_queue.rbegin(), m_queue.rend(),
                        [](const Samples *_pSamples) {
                            return _pSamples->kind == lansink::Packet_Kind_STOP;
                        });

                m_queue.erase(m_queue.begin(), iLast.base());
                break;
            }

        iSamples = m_queue.erase(iSamples);
    }

    m_queue.clear_commands();
    return !m_bClosed;
}

void Player::Impl::_add_silence(size_t _cFrames) {
    if (_cFrames == 0)
        return;

    size_t cPosition = 0;
    bool bUnderrunHandled = false;

    m_log.debug("Inserting silence: %lu frames", _cFrames);

    while (cPosition < _cFrames)
        try {
            const size_t cSilenceFrames = _cFrames - cPosition;
            const size_t cSilenceBytes = cSilenceFrames*m_pSink->get_frame_bytes();
            assert(cSilenceBytes < 1024*1024*100);
            std::unique_ptr<char []> pBuf = std::unique_ptr<char []>(new char[cSilenceBytes]);

            memset(pBuf.get(), 0, cSilenceBytes);
            const long nWritten = m_pSink->write(pBuf.get(), cSilenceFrames);

            if (nWritten == 0)
                throw Sink::Error(Sink::Error::seUnderrun, 0, "sink doesn't accept data");

            cPosition += nWritten;
            _add_frames_written(nWritten);
        } catch (Sink::Error &e) {
           if (!bUnderrunHandled && e.is_underrun()) {
               MilliSeconds ms(std::chrono::duration_cast<MilliSeconds>(Clock::now() - m_lastWrite));
               m_log.warning("XRUN: write(), %d ms since last write", ms.count());
               m_pSink->prepare();
               _set_frames_written(-1);
               bUnderrunHandled = true;
           } else
               throw;
        }
}

long Player::Impl::_get_buffered_frames() const {
    return m_pSink->get_delay();
}

long Player::Impl::_get_available_frames(bool _bSync) const {
    return m_pSink->get_avail(_bSync);
}

void Player::Impl::_add_frames_written(long _nFrames) {
    if (m_nFramesWritten >= 0)
        m_nFramesWritten += _nFrames;
}

void Player::Impl::_set_frames_written(long _nFrames) {
    m_nFramesWritten = _nFrames;
}

long Player::Impl::_estimate_position() {
    if (m_nFramesWritten < 0) {
        const Clock::duration timeSince = Clock::now() - m_lastWrite;
        using Period = Clock::duration::period;
        const long nFramesSince =
                m_pSink->get_rate()*timeSince.count()*Period::num/Period::den;
        const long nFramesSinceXRun =
                std::max<long>(0, nFramesSince - m_nLastFramesBuffered);

        m_log.debug("Estimated position after underrun: "
                "frames since last write = %d, frames since xrun = %d, "
                "last frames buffered = %ld, last frames written = %ld",
                nFramesSince, nFramesSinceXRun, m_nLastFramesBuffered,
                m_nLastFramesWritten);

        _set_frames_written(m_nLastFramesWritten + nFramesSinceXRun);
    }

    m_log.debug("Estimated position: frames base = %d, frames written = %d, "
                "last frames buffered = %ld, last frames written = %ld",
                m_nFramesBase, m_nFramesWritten, m_nLastFramesBuffered,
                m_nLastFramesWritten);

    return m_nFramesBase + m_nFramesWritten;
}

void Player::Impl::_add_samples(size_t _cFrames) {
    assert(m_pSink->get_frame_bytes() > 0);

    if (!m_queue.empty())
        m_xrunTime = TimePoint();

    if (m_queue.has_commands())
        if (!_handle_commands())
            return;

    if (m_bEmulatedPause) {
        _cFrames = std::max<size_t>(m_pSink->get_period_size(), _cFrames);
        _add_silence(_cFrames);
        return;
    }

    if (m_queue.empty()) {
        if (!m_bPaused) {
            _cFrames = std::max<size_t>(m_pSink->get_period_size(), _cFrames);

            if (m_averageSize.is_full())
                _cFrames = std::min<size_t>(_cFrames, std::ceil(
                        m_averageSize.get()/m_pSink->get_frame_bytes()));

            if (m_xrunTime != TimePoint()) {
                const auto seconds(std::chrono::duration_cast<std::chrono::seconds>(
                        Clock::now() - m_xrunTime));
                if (seconds.count() > g_settings.nRecoveryTimeout)
                    throw RuntimeError("No data for %ld seconds", seconds.count());
            } else {
                m_log.warning("Avoiding underrun (%lu frames max, timeout %d seconds)",
                        _cFrames, g_settings.nRecoveryTimeout);
                m_xrunTime = Clock::now();
            }

            _add_silence(_cFrames);
            m_cFramesPadding += _cFrames;
        }

        return;
    }


    // Work around latency introduced by program execution.
    const long c_nThreshold = m_pSink->get_period_size();

    while (!m_queue.empty() && _cFrames > 0) {
        Samples *pSamples = m_queue.pop();
        const long nNext = pSamples->cTimestamp + pSamples->cOffset;
        long nPosition = _estimate_position();
        const long nPacketDelay = nNext - nPosition;

        m_log.debug("Processing packet: timestamp = %d, offset = %lu, position = %ld, "
                "packet delay = %ld, average delay = %.2f", pSamples->cTimestamp,
                pSamples->cOffset, nPosition, nPacketDelay, m_averageDelay.get());

        if (!m_bPaused) {
            m_averageDelay.add(nPacketDelay);
            m_averageSize.add(pSamples->data.size());

            if (m_averageDelay.is_full() &&
                    std::abs(m_averageDelay.get()) > c_nBaseFramesAdjustmentThreshold)
            {
                const long nAdjustment = m_averageDelay.get();
                m_log.debug("Adjusting base frame count: %ld", nAdjustment);
                m_nFramesBase += nAdjustment;
                nPosition += nAdjustment;
                m_averageDelay.clear();
            }
        }

        if (nNext > nPosition + c_nThreshold) {
            // Pad with silence.
            const size_t cFrames = std::min<size_t>(nNext - nPosition, _cFrames);

            m_log.warning("Padding till %ld (%lu frames max)", nNext, _cFrames);
            _add_silence(cFrames);
            m_cFramesPadding += cFrames;
            nPosition += cFrames;
            _cFrames -= cFrames;

            if (_cFrames == 0) {
                m_queue.push(pSamples);
                break;
            }
        } else if (nNext + c_nThreshold < nPosition) {
            // Shift offset.
            const size_t cFrames = nPosition - nNext;
            const size_t cFramesPacket = pSamples->get_frame_count(
                    m_pSink->get_frame_bytes());

            m_log.warning("Shifting offset till %ld (%lu frames max)",
                    nPosition, cFramesPacket);

            if (cFrames >= cFramesPacket) {
                m_cFramesSkipped += cFramesPacket;
                if (pSamples->cOffset == 0)
                    ++m_cPacketsSkipped;
                delete pSamples;
                continue;
            }

            pSamples->cOffset += cFrames;
            m_cFramesSkipped += cFrames;
        }

        const size_t cWriteSize = std::min(_cFrames, pSamples->get_frame_count(m_pSink->get_frame_bytes()));

        m_log.debug("Inserting audio: %lu, %lu (%ld)",
                pSamples->cTimestamp + pSamples->cOffset, cWriteSize, nPosition);

        try {
            const size_t cWritten = m_pSink->write(pSamples->get_data(
                    m_pSink->get_frame_bytes()), cWriteSize);

            if (cWritten == 0) {
                const auto seconds(std::chrono::duration_cast<std::chrono::seconds>(
                        Clock::now() - m_lastWrite));
                if (seconds.count() > g_settings.nRecoveryTimeout)
                    throw Sink::Error(Sink::Error::seUnderrun, 0,
                            "sink doesn't accept data");
            }

            pSamples->cOffset += cWritten;
            _add_frames_written(cWritten);
            _cFrames -= cWritten;
        } catch (Sink::Error &e) {
           if (e.is_underrun()) {
               MilliSeconds ms(std::chrono::duration_cast<MilliSeconds>(Clock::now() - m_lastWrite));
               m_log.warning("XRUN: write(), %d ms since last write", ms.count());
               m_pSink->prepare();
               _set_frames_written(-1);
           } else {
               m_queue.push(pSamples);
               throw;
           }
        }

        if (pSamples->get_frame_count(m_pSink->get_frame_bytes()) > 0)
            m_queue.push(pSamples);
        else
            delete pSamples;
    }
}

Player::Impl::Players Player::Impl::s_players{};
std::mutex Player::Impl::s_playerMapMutex{};

Player::Player(uint64_t _cStreamId, Log &_log, const std::string &_strSender) :
        m_pImpl(new Impl(_cStreamId, _log, _strSender))
{
}

Player::~Player() {
    delete m_pImpl;
}

Player *Player::get(lansink::Packet &_packet, Log &_log, TimePoint &_closeTime,
        const std::string &_strSender)
{
    if (!Player::remove_stopped(_closeTime))
        return nullptr;
    return Impl::get(_packet, _log, _strSender);
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
