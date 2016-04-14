/*
    utils.h

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

#ifndef LANSINK_UTILS_H_
#define LANSINK_UTILS_H_

#include <sys/socket.h>

#include <cassert>
#include <string>
#include <deque>
#include <vector>
#include <algorithm>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

std::string format(size_t _cMaxLength, const char *_strFormat, ...);
void recv_all(int _cSock, char *_pBuf, size_t _cLen, int _nFlags);
void send_all(int _cSock, const char *_pBuf, size_t _cLen, int _nFlags);
void sendto_all(int _cSock, const char *_pBuf, size_t _cLen, int _nFlags,
        __CONST_SOCKADDR_ARG _pAddr, socklen_t _cAddrLen);

template<typename _Sample = int, typename _Mean = double>
class MovingAverage {
public:
    typedef _Mean Mean;
    typedef _Sample Sample;

    MovingAverage(size_t _cSize) : m_cSize(_cSize), m_fMean(0) {
        assert(_cSize <= m_samples.max_size());
    }

    void add(_Sample _nSample) {
        if (is_full()) {
            m_fMean -= Mean(m_samples.front())/m_cSize;
            m_samples.pop_front();
        }

        m_fMean += Mean(_nSample)/m_cSize;
        // TODO It's more reasonable to divide _nSample right here unless resize() gets implemented.
        m_samples.push_back(_nSample);
    }

    _Mean get() const {
        return m_fMean;
    }

    void clear() {
        m_fMean = 0;
        m_samples.clear();
    }

    size_t size() const {
        return m_samples.size();
    }

    bool is_full() const {
        return size() == m_cSize;
    }

private:
    size_t m_cSize;
    Mean m_fMean;
    std::deque<Sample> m_samples;
};

template<class _T>
struct PtrLess {
    bool operator ()(const _T *_pLhs, const _T *_pRhs) const {
        return *_pLhs < *_pRhs;
    }
};

template<class _Clock>
class WakeupDetector {
public:
    WakeupDetector() {
        _start();
    }

    ~WakeupDetector() {
        m_bRunning = false;
        m_stop.notify_all();
        if (m_worker.joinable())
            m_worker.join();
    }

    bool check() {
        std::unique_lock<std::mutex> lock(m_mutex);
        return _check();
    }

    void reset() {
        std::unique_lock<std::mutex> lock(m_mutex);
        _reset();
    }

private:
    using Clock = _Clock;
    using Duration = std::chrono::seconds;
    using TimePoint = std::chrono::time_point<Clock>;

    static const Duration c_period;
    static const unsigned c_uThreshold;

    std::thread m_worker;
    std::condition_variable m_stop;
    std::mutex m_mutex;
    TimePoint m_time;
    std::atomic_bool m_bRunning;
    bool m_bTriggered = false;

    bool _check() {
        if (!m_bTriggered && Clock::now() - m_time > c_uThreshold*c_period)
            m_bTriggered = true;
        return m_bTriggered;
    }

    void _reset() {
        m_bTriggered = false;
        m_time = Clock::now();
    }

    void _start() {
        m_bRunning = false;
        m_worker = std::thread([&]() {
            std::unique_lock<std::mutex> lock(m_mutex);
            _reset();
            m_bRunning = true;
            while (!m_stop.wait_for(lock, c_period, [&]() { return !m_bRunning; })) {
                if (!_check())
                    m_time = Clock::now();
            }
        });

        while (!m_bRunning)
            std::this_thread::yield();
    }
};

template<class _Clock>
const typename WakeupDetector<_Clock>::Duration WakeupDetector<_Clock>::c_period{1};

template<class _Clock>
const unsigned WakeupDetector<_Clock>::c_uThreshold{10};

class LostPacketDetector {
public:
    void reserve(size_t _cReservedRecords) {
        m_records.reserve(_cReservedRecords);
    }

    void add(long _nOffset, size_t _cSize) {
        m_records.emplace_back(_nOffset, _cSize);
    }

    struct Stats {
        size_t cPackets = 0;
        size_t cPacketsLost = 0;
        size_t cPacketsDuplicated = 0;
        size_t cPacketsOverlapped = 0;
        size_t cSize = 0;
        size_t cSizeLost = 0;
        size_t cSizeDuplicated = 0;
        size_t cSizeOverlapped = 0;
    };

    Stats collect();

    bool empty() const { return m_records.empty(); }

private:
    struct Record {
        long nOffset = 0;
        size_t cSize = 0;

        Record() = default;

        Record(long _nOffset, size_t _cSize) : nOffset(_nOffset), cSize(_cSize) {}

        bool operator <(const Record & _other) const {
            return nOffset < _other.nOffset;
        }
    };

    std::vector<Record> m_records;
};

#endif /* LANSINK_UTILS_H_ */
