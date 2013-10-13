/*
    sender.cpp

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

#include "sender.h"
#include "formats.h"
#include "exception.h"
#include "alsa.h"

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netdb.h>
#include <poll.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <functional>
#include <thread>
#include <chrono>
#include <list>
#include <mutex>
#include <atomic>
#include <random>

class Sender::Impl {
public:
    typedef Sender::Status Status;

    Impl(Sender *_pPlug);
    ~Impl();

    Status get_status() const;
    void start();
    void stop();
    snd_pcm_sframes_t get_buffer_pointer() const;
    snd_pcm_sframes_t transfer(const char *_pData, size_t _cOffset, size_t _cSize);
    void prepare();
    void drain();
    void pause();
    void unpause();
    snd_pcm_sframes_t get_delay() const;
    void connect();
    const std::vector<unsigned int> &get_format_values();
    size_t get_bytes_per_frame() const;

private:
    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::duration<int, std::milli> Duration;
    typedef std::chrono::time_point<Clock> TimePoint;

    Sender *m_pPlug;
    std::vector<unsigned int> m_formatValues;
    std::string m_strFormat;
    size_t m_cBitsPerSample;
    size_t m_cChannels;
    std::thread m_worker;
    mutable std::mutex m_mutex;
    int m_nSockWorker;
    mutable TimePoint m_startTime;
    TimePoint m_lastFrameTime;
    mutable snd_pcm_sframes_t m_nLastFrames;
    snd_pcm_sframes_t m_nFramesQueued;
    snd_pcm_sframes_t m_nPointer;
    std::list<std::string> m_queue;
    std::unique_ptr<char[]> m_pBuffer;
    mutable Status m_status;
    bool m_bPrepared;
    int m_nSocket;
    static unsigned int s_cSeed;
    static std::default_random_engine s_randomEngine;
    uint64_t m_cStreamId;
    std::atomic_bool m_bStarted;
    bool m_bDraining;

    void _reset(bool _bResetStreamParams);
    std::function<void(void)> _make_worker();
    void _start_worker();
    void _stop_worker();
    void _init_descriptors();
    snd_pcm_sframes_t _estimate_frames() const;
    snd_pcm_sframes_t _get_frames_required() const;
    void _prepare_packet(lansink::Packet &_packet, lansink::Packet_Kind _kind, uint64_t _nTimestamp);
    void _send_packet(const lansink::Packet &_packet);
    void _send_data();
    void _send_pause();
    void _send_start();
    void _send_stop();
    snd_pcm_sframes_t _get_delay() const;
};

unsigned int Sender::Impl::s_cSeed = 0;
std::default_random_engine Sender::Impl::s_randomEngine = std::default_random_engine();

Sender::Impl::Impl(Sender *_pPlug) :
    m_pPlug(_pPlug), m_strFormat(""), m_cBitsPerSample(0), m_cChannels(0),
    m_nSockWorker(0), m_status(Sender::usStopped), m_nSocket(-1),
    m_cStreamId(0), m_bStarted(false), m_bDraining(false)
{
    if (s_cSeed == 0) {
        try {
            std::random_device rd;
            s_cSeed = rd();
        } catch (...) {
            s_cSeed = Clock::now().time_since_epoch().count();
        }

        s_randomEngine.seed(s_cSeed);
    }

    _reset(true);
    _init_descriptors();
}

Sender::Impl::~Impl() {
    if (m_nSocket >= -1)
        close(m_nSocket);
}

Sender::Status Sender::Impl::get_status() const {
    return m_status;
}

void Sender::Impl::start() {
    std::lock_guard<std::mutex> lock(m_mutex);

    m_status = Sender::usRunning;
    m_startTime = Clock::now();
    _start_worker();

    // Sleep a bit since returning pointer() = 0 right after start somehow confuses the user.
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void Sender::Impl::stop() {
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        if (m_status & Sender::usRunning)
            _send_stop();

        // FIXME status gets overwritten somewhere.
        m_status = Sender::usStopped;
    }

    _stop_worker();
}

snd_pcm_sframes_t Sender::Impl::get_buffer_pointer() const {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (!m_bPrepared)
        return 0;

    const snd_pcm_sframes_t nDelay = _get_delay();
    snd_pcm_sframes_t nPointer = snd_pcm_sframes_t(m_pPlug->appl_ptr) - nDelay;

    if (nPointer < 0) {
        nPointer += m_pPlug->get_buffer_size();

        if (nPointer < 0)
            nPointer = 0;
    }

    return nPointer%m_pPlug->get_buffer_size();
}

snd_pcm_sframes_t Sender::Impl::transfer(const char *_pData, size_t _cOffset, size_t _cSize) {
    std::lock_guard<std::mutex> lock(m_mutex);

    const size_t cSizeBytes = _cSize*get_bytes_per_frame();

    if (cSizeBytes == 0)
        return 0;

    const char *pSrc = _pData + _cOffset*get_bytes_per_frame();

    m_lastFrameTime = Clock::now();
    m_queue.emplace_back(pSrc, pSrc + cSizeBytes);
    m_nFramesQueued += _cSize;
    _start_worker();

    return _cSize;
}

void Sender::Impl::prepare() {
    // FIXME race condition.
    // Stop thread first.
    if (m_status & Sender::usRunning) {
        m_status = Sender::usStopping;
        _stop_worker();
        m_status = Sender::usStopped;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    const std::string strOldFormat = m_strFormat;
    const size_t cOldRate = m_cBitsPerSample;
    const size_t cOldChannels = m_cChannels;

    _reset(true);
    m_nFramesQueued = 0;
    m_strFormat = get_format_name(m_pPlug->get_format());
    m_cBitsPerSample = ALSA::format_physical_width(m_pPlug->get_format());
    m_cChannels = m_pPlug->get_channel_count();

    if (m_strFormat != strOldFormat || m_cBitsPerSample != cOldRate ||
            m_cChannels != cOldChannels)
        m_cStreamId = s_randomEngine();

    assert(get_bytes_per_frame() > 0);
    assert(m_pPlug->get_buffer_size() > 0);
    std::unique_ptr<char[]> pBuffer(new char[m_pPlug->get_buffer_size()*get_bytes_per_frame()]);
    m_pBuffer = std::move(pBuffer);
    m_bPrepared = true;
    m_status = Sender::usPaused;
    m_bStarted = false;
    m_bDraining = false;
}

void Sender::Impl::drain() {
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        assert(m_bPrepared);
        m_status = Sender::usStopping;
        m_bDraining = true;
    }

    _stop_worker();
    m_status = Sender::usStopped;
}

void Sender::Impl::pause() {
    std::lock_guard<std::mutex> lock(m_mutex);

    assert(m_bPrepared && m_status == Sender::usRunning);
    m_nLastFrames = _estimate_frames();
    m_status = Sender::usPaused;
    m_startTime = TimePoint();
    _start_worker();
}

void Sender::Impl::unpause() {
    std::lock_guard<std::mutex> lock(m_mutex);

    m_status = Sender::usRunning;
    m_startTime = Clock::now();
    _start_worker();
}

snd_pcm_sframes_t Sender::Impl::_get_delay() const {
    if (!m_bPrepared)
        return m_pPlug->get_buffer_size();

    const snd_pcm_sframes_t nEstimated = _estimate_frames();
    const snd_pcm_sframes_t nDelay = std::max<snd_pcm_sframes_t>(0,
            snd_pcm_sframes_t(m_pPlug->appl_ptr) - nEstimated);

    if (nDelay > (snd_pcm_sframes_t)m_pPlug->get_buffer_size())
        m_pPlug->log.warning("nDelay > get_buffer_size() (%ld > %lu)",
                nDelay, m_pPlug->get_buffer_size());

    if ((m_status & Sender::usRunning) && nDelay == 0) {
        if (m_status != Sender::usUnderrun) {
            m_pPlug->log.warning("XRUN");
            m_nLastFrames = nEstimated;
            m_status = Sender::usUnderrun;
            m_startTime = TimePoint();
        }

        return -EPIPE;
    }

    return nDelay;
}

snd_pcm_sframes_t Sender::Impl::get_delay() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return _get_delay();
}

void Sender::Impl::_reset(bool _bResetStreamParams) {
    m_nPointer = 0;
    m_nLastFrames = 0;
    m_startTime = TimePoint();

    if (_bResetStreamParams) {
        m_nFramesQueued = 0;
        m_strFormat = "";
        m_cBitsPerSample = 0;
        m_queue.clear();
        m_bPrepared = false;
    }
}

void Sender::Impl::_init_descriptors() {
    int fds[2];

    if (socketpair(AF_LOCAL, SOCK_STREAM, 0, fds) != 0)
        throw SystemError("socketpair()");

    for (int fd : fds) {
        int fl;

        if ((fl = fcntl(fd, F_GETFL)) < 0)
            throw SystemError("fcntl()");

        if (fl & O_NONBLOCK)
            break;

        if (fcntl(fd, F_SETFL, fl | O_NONBLOCK) < 0)
            throw SystemError("fcntl()");
    }

    m_pPlug->poll_fd = fds[0];
    m_nSockWorker = fds[1];
    m_pPlug->poll_events = POLLIN;
}

snd_pcm_sframes_t Sender::Impl::_estimate_frames() const {
    if ((m_status & Sender::usRunning) == 0 || m_status == Sender::usPaused ||
            m_status == Sender::usUnderrun)
        return m_nLastFrames;

    // Wasn't started.
    if (m_startTime == TimePoint())
        return m_nLastFrames;

    Duration ms(std::chrono::duration_cast<Duration>(Clock::now() - m_startTime));

    return m_nLastFrames + ms.count()*(m_pPlug->get_rate()/1000.0);
}

snd_pcm_sframes_t Sender::Impl::_get_frames_required() const {
    Duration ms(std::chrono::duration_cast<Duration>(Clock::now() - m_lastFrameTime));
    return ms.count()*m_pPlug->get_rate()/1000;
}

std::function<void(void)> Sender::Impl::_make_worker() {
    return [&]() {
        if (!m_nSockWorker)
            return;

        m_pPlug->log.info("Starting sender thread");
        m_bStarted = true;

        Status prev = m_status;
        const int nSendThresholdFrames = (m_pPlug->nMTU - 100)/get_bytes_per_frame();

        while (m_status & Sender::usRunning) {
            {
                std::lock_guard<std::mutex> lock(m_mutex);

                try {
                    if (prev == Sender::usRunning &&
                            (m_status == Sender::usPaused || m_status == Sender::usUnderrun))
                        _send_pause();
                    else if ((prev == Sender::usPaused || prev == Sender::usUnderrun)
                            && m_status == Sender::usRunning)
                        _send_start();

                    if (!m_queue.empty()) {
                        while (!m_queue.empty() && (
                                m_status != Sender::usRunning ||
                                m_nFramesQueued >= nSendThresholdFrames))
                            _send_data();
                    } else if (m_bPrepared && m_status == Sender::usStopping && _estimate_frames() >= m_nPointer) {
                        _send_stop();
                        m_status = Sender::usStopped;
                        continue;
                    }

                    if (_get_delay() < (snd_pcm_sframes_t)m_pPlug->get_buffer_size()) {
                        // Don't wake up the other party unless there is buffer space available.
                        char buf[1] = {0};
                        write(m_nSockWorker, buf, 1);
                    } else if (m_status == Sender::usUnderrun) {
                        // Resume if buffer filled up after underrun.
                        m_pPlug->log.info("Resuming after XRUN");
                        m_status = Sender::usRunning;
                        m_startTime = Clock::now();
                    }
                } catch (std::exception &e) {
                    m_pPlug->log.error(e.what());
                }

                prev = m_status;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(m_pPlug->nSendPeriod));
        }
    };
}

void Sender::Impl::_start_worker() {
    // _start_worker is never called from worker thread, so no mutex.
    if (!m_bStarted) {
        m_worker = std::thread(_make_worker());

        // Busy-wait while worker initializes.
        while (!m_bStarted)
            std::this_thread::yield();
    }
}

void Sender::Impl::_stop_worker() {
    if (m_worker.joinable()) {
        m_worker.join();
        m_bStarted = false;
    }
}

void Sender::Impl::_prepare_packet(lansink::Packet &_packet, lansink::Packet_Kind _kind,
        uint64_t _nTimestamp)
{
    if (_kind == lansink::Packet_Kind_DATA) {
        if (m_status != Sender::usRunning && m_status != Sender::usStopping)
            _kind = lansink::Packet_Kind_CACHE;
    } else
        _packet.set_samples("");

    _packet.set_version(1);
    _packet.set_stream(m_cStreamId);
    _packet.set_kind(_kind);
    _packet.set_channels(m_cChannels);
    _packet.set_rate(m_pPlug->get_rate());
    _packet.set_format(m_strFormat);
    _packet.set_timestamp(_nTimestamp);
}

void Sender::Impl::_send_packet(const lansink::Packet &_packet) {
    auto pBuf = std::unique_ptr<char[]>(new char[m_pPlug->nMTU]);
    assert(_packet.ByteSize() <= m_pPlug->nMTU);
    _packet.SerializeToArray((void *)pBuf.get(), m_pPlug->nMTU);
    send_all(m_nSocket, pBuf.get(), _packet.ByteSize(), 0);
}

void Sender::Impl::_send_stop() {
    lansink::Packet packet;
    _prepare_packet(packet, lansink::Packet_Kind_STOP, m_nPointer);
    _send_packet(packet);
}

void Sender::Impl::_send_pause() {
    lansink::Packet packet;
    _prepare_packet(packet, lansink::Packet_Kind_PAUSE, m_nPointer);
    _send_packet(packet);
}

void Sender::Impl::_send_start() {
    lansink::Packet packet;
    _prepare_packet(packet, lansink::Packet_Kind_START, m_nPointer);
    _send_packet(packet);
}

void Sender::Impl::_send_data() {
    lansink::Packet packet;

    assert(m_bPrepared);
    _prepare_packet(packet, lansink::Packet_Kind_DATA, m_nPointer);

    const size_t cHeaderSize = packet.ByteSize() + 4; // Account for data length field.
    const size_t cFrameSize = get_bytes_per_frame();
    const size_t cTotal = ((m_pPlug->nMTU - cHeaderSize)/cFrameSize)*cFrameSize;
    size_t cBytes = cTotal;
    auto pBuf = std::unique_ptr<char[]>(new char[m_pPlug->nMTU]); // Reuse as serialization buffer.
    char *pPos = pBuf.get();
    size_t cOffset = 0;

    while (!m_queue.empty() && cBytes > 0) {
        const char *pSource = m_queue.front().data() + cOffset;
        const size_t cAvailable = m_queue.front().size() - cOffset;

        if (cAvailable > 0) {
            const size_t c = std::min(cAvailable, (cBytes/cFrameSize)*cFrameSize);

            if (c == 0)
                throw LogicError("Queue element size isn't multiple of frame size");

            memcpy(pPos, pSource, c);

            pPos += c;
            cBytes -= c;
            cOffset += c;
        }

        if (cOffset == m_queue.front().size()) {
            m_queue.pop_front();
            cOffset = 0;
        }
    }

    if (cOffset > 0 && !m_queue.empty() && cOffset < m_queue.front().size())
        m_queue.front() = m_queue.front().substr(cOffset);

    packet.set_samples((const void *)pBuf.get(), cTotal - cBytes);
    m_nPointer += (cTotal - cBytes)/get_bytes_per_frame();
    m_nFramesQueued -= (cTotal - cBytes)/get_bytes_per_frame();

    // Send.
    assert(packet.ByteSize() <= m_pPlug->nMTU);
    packet.SerializeToArray((void *)pBuf.get(), m_pPlug->nMTU);
    send_all(m_nSocket, pBuf.get(), packet.ByteSize(), 0);
}

void Sender::Impl::connect() {
    struct sockaddr_in name;

    if (m_nSocket >= 0)
        close(m_nSocket);

    m_nSocket = socket(PF_INET, SOCK_DGRAM, 0);

    if (m_nSocket < 0)
        throw SystemError("socket()");

    name.sin_family = AF_INET;
    name.sin_port = htons(m_pPlug->nPort);

    struct hostent *pHost = gethostbyname(m_pPlug->strHost.c_str());

    if (!pHost)
        throw SystemError("gethostbyname()");

    name.sin_addr = *(struct in_addr *)pHost->h_addr;

    if (::connect(m_nSocket, (struct sockaddr *)&name, sizeof(name)) < 0)
        throw SystemError("connect()");
}

const std::vector<unsigned int> &Sender::Impl::get_format_values() {
    m_formatValues.clear();

    for (const std::string &strFormat : m_pPlug->formats) {
        auto iFormat = g_formats.find(strFormat);

        if (iFormat != g_formats.end())
            m_formatValues.push_back(iFormat->second);
    }

    return m_formatValues;
}

size_t Sender::Impl::get_bytes_per_frame() const {
    return m_cChannels*m_cBitsPerSample/8;
}

Sender::Sender() :
        strHost("127.0.0.1"), nPort(26751), nMTU(1500), nSendPeriod(10), m_pImpl(new Impl(this))
{
}

Sender::~Sender() {
    delete m_pImpl;
}

Sender::Status Sender::get_status() const {
    return m_pImpl->get_status();
}

void Sender::start() {
    m_pImpl->start();
}

void Sender::stop() {
    m_pImpl->stop();
}

snd_pcm_sframes_t Sender::get_buffer_pointer() const {
    return m_pImpl->get_buffer_pointer();
}

snd_pcm_sframes_t Sender::transfer(const char *_pData, size_t _cOffset, size_t _cSize) {
    return m_pImpl->transfer(_pData, _cOffset, _cSize);
}

void Sender::prepare() {
    m_pImpl->prepare();
}

void Sender::drain() {
    return m_pImpl->drain();
}

void Sender::pause() {
    m_pImpl->pause();
}

void Sender::unpause() {
    m_pImpl->unpause();
}

snd_pcm_sframes_t Sender::get_delay() const {
    return m_pImpl->get_delay();
}

void Sender::connect() {
    m_pImpl->connect();
}

const std::vector<unsigned int> &Sender::get_format_values() {
    return m_pImpl->get_format_values();
}

size_t Sender::get_bytes_per_frame() const {
    return m_pImpl->get_bytes_per_frame();
}

unsigned int Sender::get_channel_count() const {
    return this->channels;
}

snd_pcm_uframes_t Sender::get_buffer_size() const {
    return this->buffer_size;
}

snd_pcm_uframes_t Sender::get_period_size() const {
    return this->buffer_size;
}

snd_pcm_format_t Sender::get_format() const {
    return this->format;
}

unsigned int Sender::get_rate() const {
    return this->rate;
}
