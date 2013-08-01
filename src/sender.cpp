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
#include <random>

unsigned int UNAP::s_cSeed = 0;
std::default_random_engine UNAP::s_randomEngine = std::default_random_engine();

UNAP::UNAP() :
    strHost("127.0.0.1"), nPort(26751), nMTU(1500), nSendPeriod(10),
    m_strFormat(""), m_cBitsPerSample(0), m_cChannels(0),
    m_nSockWorker(0), m_status(usStopped), m_nSocket(-1),
    m_cStreamId(0)
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

UNAP::~UNAP() {
    if (m_nSocket >= -1)
        close(m_nSocket);
}

UNAP::Status UNAP::get_status() const {
    return m_status;
}

void UNAP::start() {
    _reset(false);
    m_status = usRunning;
    m_startTime = Clock::now();
    m_worker = std::thread(_make_worker());
}

void UNAP::stop() {
    if (m_status & usRunning)
        _send_stop();

    m_status = usStopped;

    if (m_worker.joinable())
        m_worker.join();
}

snd_pcm_sframes_t UNAP::get_buffer_pointer() const {
    if (!m_bPrepared)
        return 0;

    const snd_pcm_sframes_t nDelay = get_delay();
    snd_pcm_sframes_t nPointer = snd_pcm_sframes_t(this->appl_ptr) - nDelay;

    if (nPointer < 0) {
        nPointer += get_buffer_size();

        if (nPointer < 0)
            nPointer = 0;
    }

    return nPointer%get_buffer_size();
}

snd_pcm_sframes_t UNAP::transfer(const char *_pData, size_t _cOffset, size_t _cSize) {
    assert(m_bPrepared);

    std::lock_guard<std::mutex> lock(m_mutex);
    const size_t cSizeBytes = _cSize*get_bytes_per_frame();

    if (cSizeBytes == 0)
        return 0;

    const char *pSrc = _pData + _cOffset*get_bytes_per_frame();

    m_lastFrameTime = Clock::now();
    m_queue.emplace_back(pSrc, pSrc + cSizeBytes);
    m_nFramesQueued += _cSize;

    return _cSize;
}

void UNAP::prepare() {
    const std::string strOldFormat = m_strFormat;
    const size_t cOldRate = m_cBitsPerSample;
    const size_t cOldChannels = m_cChannels;

    _reset(true);
    m_nFramesQueued = 0;
    m_strFormat = get_format_name(get_format());
    m_cBitsPerSample = ALSA::format_physical_width(get_format());
    m_cChannels = get_channel_count();

    if (m_strFormat != strOldFormat || m_cBitsPerSample != cOldRate ||
            m_cChannels != cOldChannels)
        m_cStreamId = s_randomEngine();

    assert(get_bytes_per_frame() > 0);
    assert(get_buffer_size() > 0);
    std::unique_ptr<char[]> pBuffer(new char[get_buffer_size()*get_bytes_per_frame()]);
    m_pBuffer = std::move(pBuffer);
    m_bPrepared = true;
}

void UNAP::drain() {
    assert(m_bPrepared);
    m_status = usStopping;
    if (m_worker.joinable())
        m_worker.join();
}

void UNAP::pause() {
    assert(m_bPrepared && m_status == usRunning);
    m_nLastFrames = _estimate_frames();
    m_status = usPaused;
    m_startTime = TimePoint();
}

void UNAP::unpause() {
    m_status = usRunning;
    m_startTime = Clock::now();
}

snd_pcm_sframes_t UNAP::get_delay() const {
    if (!m_bPrepared)
        return get_buffer_size();

    const snd_pcm_sframes_t nEstimated = _estimate_frames();
    const snd_pcm_sframes_t nDelay = std::max<snd_pcm_sframes_t>(0,
            snd_pcm_sframes_t(this->appl_ptr) - nEstimated);

    if (nDelay > (snd_pcm_sframes_t)get_buffer_size())
        this->log.warning("nDelay > get_buffer_size() (%d > %d)",
                nDelay, get_buffer_size());

    if (nDelay == 0)
       this->log.warning("XRUN, nDelay == 0");

    return nDelay;
}

void UNAP::_reset(bool _bResetStreamParams) {
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

void UNAP::_init_descriptors() {
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

    poll_fd = fds[0];
    m_nSockWorker = fds[1];
    poll_events = POLLIN;
}

snd_pcm_sframes_t UNAP::_estimate_frames() const {
    if ((m_status & usRunning) == 0 || m_status == usPaused)
        return m_nLastFrames;
    Duration ms(std::chrono::duration_cast<Duration>(Clock::now() - m_startTime));
    return m_nLastFrames + ms.count()*(get_rate()/1000.0);
}

snd_pcm_sframes_t UNAP::_get_frames_required() const {
    Duration ms(std::chrono::duration_cast<Duration>(Clock::now() - m_lastFrameTime));
    return ms.count()*get_rate()/1000;
}

std::function<void(void)> UNAP::_make_worker() {
    return [&]() {
        if (!m_nSockWorker)
            return;

        Status prev = usRunning;
        const int nSendThresholdFrames = (this->nMTU - 100)/get_bytes_per_frame();

        while (m_status & usRunning) {
            if (prev == usRunning && m_status == usPaused)
                _send_pause();
            else if (prev == usPaused && m_status == usRunning)
                _send_unpause();

            {
                std::lock_guard<std::mutex> lock(m_mutex);

                if (!m_queue.empty()) {
                    while (!m_queue.empty() && (
                            m_status != UNAP::usRunning ||
                            m_nFramesQueued >= nSendThresholdFrames))
                        _send_data();
                } else if (m_bPrepared && m_status == UNAP::usStopping && _estimate_frames() >= m_nPointer) {
                    _send_stop();
                    m_status = UNAP::usStopped;
                }
            }

            // Don't wake up the other party unless there is buffer space available.
            if (get_delay() < (snd_pcm_sframes_t)get_buffer_size()) {
                char buf[1] = {0};
                write(m_nSockWorker, buf, 1);
            }

            prev = m_status;
            std::this_thread::sleep_for(std::chrono::milliseconds(this->nSendPeriod));
        }
    };
}

void UNAP::_prepare_packet(unap::Packet &_packet, unap::Packet_Kind _kind,
        uint64_t _nTimestamp)
{
    _packet.set_version(1);
    _packet.set_stream(m_cStreamId);
    _packet.set_kind(_kind);
    _packet.set_channels(m_cChannels);
    _packet.set_rate(get_rate());
    _packet.set_format(m_strFormat);
    _packet.set_timestamp(_nTimestamp);

    if (_kind != unap::Packet_Kind_DATA)
        _packet.set_samples("");
}

void UNAP::_send_buffer(const void *_pBuf, size_t _cSize) {
    for (size_t cRetry = 0; cRetry < 5; ++cRetry) {
        if (send(m_nSocket, _pBuf, _cSize, 0) >= 0)
            break;

        if (errno != ECONNREFUSED)
            throw SystemError("send()");

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void UNAP::_send_packet(const unap::Packet &_packet) {
    auto pBuf = std::unique_ptr<char[]>(new char[this->nMTU]);
    assert(_packet.ByteSize() <= this->nMTU);
    _packet.SerializeToArray((void *)pBuf.get(), this->nMTU);
    _send_buffer((const void *)pBuf.get(), _packet.ByteSize());
}

void UNAP::_send_stop() {
    unap::Packet packet;
    _prepare_packet(packet, unap::Packet_Kind_STOP, _estimate_frames());
    _send_packet(packet);
}

void UNAP::_send_pause() {
    unap::Packet packet;
    _prepare_packet(packet, unap::Packet_Kind_PAUSE, _estimate_frames());
    _send_packet(packet);
}

void UNAP::_send_unpause() {
    unap::Packet packet;
    _prepare_packet(packet, unap::Packet_Kind_UNPAUSE, m_nLastFrames);
    _send_packet(packet);
}

void UNAP::_send_data() {
    unap::Packet packet;

    assert(m_bPrepared);
    _prepare_packet(packet, unap::Packet_Kind_DATA, m_nPointer);

    const size_t cHeaderSize = packet.ByteSize() + 4; // Account for data length field.
    const size_t cFrameSize = get_bytes_per_frame();
    const size_t cTotal = ((this->nMTU - cHeaderSize)/cFrameSize)*cFrameSize;
    size_t cBytes = cTotal;
    auto pBuf = std::unique_ptr<char[]>(new char[this->nMTU]); // Reuse as serialization buffer.
    char *pPos = pBuf.get();
    size_t cOffset = 0;

    while (!m_queue.empty() && cBytes > 0) {
        const char *pSource = m_queue.front().data() + cOffset;
        const size_t cAvailable = m_queue.front().size() - cOffset;

        if (cAvailable > 0) {
            const size_t c = std::min(cAvailable, (cBytes/cFrameSize)*cFrameSize);

            if (c == 0)
                throw LogicError("Queue element size isn't mulptiple of frame size");

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
    assert(packet.ByteSize() <= this->nMTU);
    packet.SerializeToArray((void *)pBuf.get(), this->nMTU);
    _send_buffer((const void *)pBuf.get(), packet.ByteSize());
}

void UNAP::connect() {
    struct sockaddr_in name;

    if (m_nSocket >= 0)
        close(m_nSocket);

    m_nSocket = socket(PF_INET, SOCK_DGRAM, 0);

    if (m_nSocket < 0)
        throw SystemError("socket()");

    name.sin_family = AF_INET;
    name.sin_port = htons(this->nPort);

    struct hostent *pHost = gethostbyname(this->strHost.c_str());

    if (!pHost)
        throw SystemError("gethostbyname()");

    name.sin_addr = *(struct in_addr *)pHost->h_addr;

    if (::connect(m_nSocket, (struct sockaddr *)&name, sizeof(name)) < 0)
        throw SystemError("connect()");
}

const std::vector<unsigned int> &UNAP::get_format_values() {
    m_formatValues.clear();

    for (const std::string &strFormat : formats) {
        auto iFormat = g_formats.find(strFormat);

        if (iFormat != g_formats.end())
            m_formatValues.push_back(iFormat->second);
    }

    return m_formatValues;
}

unsigned int UNAP::get_channel_count() const {
    return this->channels;
}

snd_pcm_uframes_t UNAP::get_buffer_size() const {
    return this->buffer_size;
}

snd_pcm_uframes_t UNAP::get_period_size() const {
    return this->buffer_size;
}

size_t UNAP::get_bytes_per_frame() const {
    return m_cChannels*m_cBitsPerSample/8;
}

snd_pcm_format_t UNAP::get_format() const {
    return this->format;
}

unsigned int UNAP::get_rate() const {
    return this->rate;
}
