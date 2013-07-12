/*
 * unap.cpp
 *
 *  Created on: Apr 2, 2013
 *      Author: nikita.karnauhov@gmail.com
 */

#include "unap.h"
#include "formats.h"
#include "exception.h"

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
    strHost("127.0.0.1"), nPort(26751), nMTU(1500), m_nSockWorker(0), m_status(usStopped),
    m_nSocket(-1)
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

    const snd_pcm_sframes_t frames = _estimate_frames();
    snd_pcm_sframes_t nPointer = std::min(frames, m_nPointer)%get_buffer_size();
    std::lock_guard<std::mutex> lock(m_mutex);

    if (nPointer == 0 && m_queue.empty() && m_nPointer > 0 && frames > 0)
        nPointer = get_buffer_size(); // Nothing more to play, buffer must have ran out.

    return nPointer;
}

snd_pcm_sframes_t UNAP::transfer(const char *_pData, size_t _cOffset, size_t _cSize) {
    assert(m_bPrepared);

    std::lock_guard<std::mutex> lock(m_mutex);

    size_t cSizeBytes = std::min<size_t>(_cSize, get_buffer_size() - m_nAvail);

    _cSize = cSizeBytes;
    cSizeBytes *= get_bytes_per_frame();

    if (cSizeBytes == 0)
        return 0;

    char *pStart = m_pBuffer.get();
    char *pDest = m_queue.empty() ? m_pBuffer.get() : m_queue.back().second;
    const char *pSrc = _pData + _cOffset*get_bytes_per_frame();
    const size_t cBufferBytes = get_buffer_size()*get_bytes_per_frame();

    if (pDest < pStart + cBufferBytes) {
        const size_t cPart = std::min(cSizeBytes, cBufferBytes - (pDest - pStart));
        memcpy(pDest, pSrc, cPart);
        m_queue.emplace_back(pDest, pDest + cPart);
        cSizeBytes -= cPart;
        pSrc += cPart;
        assert(m_queue.back().second > m_queue.back().first);
    }

    if (cSizeBytes > 0) {
        assert(cSizeBytes < cBufferBytes);
        memcpy(pStart, pSrc, cSizeBytes);
        m_queue.emplace_back(pStart, pStart + cSizeBytes);
        assert(m_queue.back().second > m_queue.back().first);
    }

    m_nAvail += _cSize;

    return _cSize;
}

void UNAP::prepare() {
    _reset(true);
    m_nAvail = 0;
    m_strFormat = get_format_name(get_format());
    m_cBitsPerSample = snd_pcm_format_physical_width(get_format());
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

snd_pcm_sframes_t UNAP::get_unplayed_frames() const {
    return m_nAvail; // TODO m_nPosition - estimateFrames
}

void UNAP::_reset(bool _bResetStreamParams) {
    m_nPointer = 0;
    m_nLastFrames = 0;
    m_startTime = TimePoint();
    m_cStreamId = s_randomEngine();

    if (_bResetStreamParams) {
        m_nAvail = 0;
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
    if (m_status == usPaused)
        return m_nLastFrames;
    DurationMS ms(std::chrono::duration_cast<DurationMS>(Clock::now() - m_startTime));
    return m_nLastFrames + ms.count()*(get_rate()/1000.0);
}

std::function<void(void)> UNAP::_make_worker() {
    return [&]() {
        if (!m_nSockWorker)
            return;

        Status prev = usRunning;

        while (m_status & usRunning) {
            if (prev == usRunning && m_status == usPaused)
                _send_pause();
            else if (prev == usPaused && m_status == usRunning)
                _send_unpause();

            if (!m_queue.empty()) {
                std::lock_guard<std::mutex> lock(m_mutex);

                while (!m_queue.empty())
                    _send_data();
            } else if (m_bPrepared && m_status == UNAP::usStopping && _estimate_frames() >= m_nPointer) {
                _send_stop();
                m_status = UNAP::usStopped;
            }

            char buf[1] = {0};
            write(m_nSockWorker, buf, 1);
            prev = m_status;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    };
}

void UNAP::_prepare_packet(unap::Packet &_packet, unap::Packet_Kind _kind,
        uint64_t _nTimestamp)
{
    _packet.set_version(1);
    _packet.set_stream(m_cStreamId);
    _packet.set_kind(_kind);
    _packet.set_channels(get_channel_count());
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

    assert(m_queue.front().second > m_queue.front().first);
    assert(m_bPrepared);
    _prepare_packet(packet, unap::Packet_Kind_DATA, m_nPointer);

    const size_t cHeaderSize = packet.ByteSize() + 4; // Account for data length field.
    const size_t cFrameSize = get_bytes_per_frame();
    const size_t cTotal = ((this->nMTU - cHeaderSize)/cFrameSize)*cFrameSize;
    size_t cBytes = cTotal;
    auto pBuf = std::unique_ptr<char[]>(new char[this->nMTU]); // Reuse as serialization buffer.
    char *pPos = pBuf.get();

    while (!m_queue.empty() && cBytes > 0) {
        char *pSource = m_queue.front().first;
        char *pEnd = m_queue.front().second;
        const size_t cAvailable = pEnd - pSource;

        if (cAvailable > 0) {
            const size_t c = std::min(cAvailable, (cBytes/cFrameSize)*cFrameSize);

            if (c == 0)
                throw LogicError("Queue element size isn't mulptiple of frame size");

            memcpy(pPos, pSource, c);

            pSource += c;
            pPos += c;
            cBytes -= c;
        }

        m_queue.front().first = pSource;

        assert(m_queue.front().second >= m_queue.front().first);

        if (pSource == pEnd)
            m_queue.pop_front();
    }

    packet.set_samples((const void *)pBuf.get(), cTotal - cBytes);
    m_nPointer += (cTotal - cBytes)/get_bytes_per_frame();
    m_nAvail -= (cTotal - cBytes)/get_bytes_per_frame();

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
    return get_channel_count()*m_cBitsPerSample/8;
}

snd_pcm_format_t UNAP::get_format() const {
    return this->format;
}

unsigned int UNAP::get_rate() const {
    return this->rate;
}
