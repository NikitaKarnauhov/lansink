/*
    utils.cpp

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

#include <stdarg.h>
#include <errno.h>
#include <sys/socket.h>

#include <memory>
#include <string>
#include <thread>
#include <chrono>

#include "exception.h"

std::string format(size_t _cMaxLength, const char *_strFormat, ...) {
    std::unique_ptr<char []> str(new char[_cMaxLength]);
    va_list args;

    va_start(args, _strFormat);
    str[_cMaxLength - 1] = 0;
    vsnprintf(str.get(), _cMaxLength - 1, _strFormat, args);
    va_end (args);

    return str.get();
}

void recv_all(int _cSock, char *_pBuf, size_t _cLen, int _nFlags) {
    do {
        ssize_t nRead = 0;

        for (int nRetry = 5; nRetry >= 0; --nRetry) {
            nRead = recv(_cSock, _pBuf, _cLen, _nFlags);

            if (nRead >= 0)
                break;

            if (nRead < 0 && (errno != ECONNREFUSED || nRetry == 0))
                throw SystemError("recv()");

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        _cLen -= nRead;
        _pBuf += nRead;
    } while (_cLen > 0);
}

void send_all(int _cSock, const char *_pBuf, size_t _cLen, int _nFlags) {
    do {
        ssize_t nSent = 0;

        for (int nRetry = 5; nRetry >= 0; --nRetry) {
            nSent = send(_cSock, _pBuf, _cLen, _nFlags);

            if (nSent >= 0)
                break;

            if (nSent < 0 && (errno != ECONNREFUSED || nRetry == 0))
                throw SystemError("send()");

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        _cLen -= nSent;
        _pBuf += nSent;
    } while (_cLen > 0);
}

void sendto_all(int _cSock, const char *_pBuf, size_t _cLen, int _nFlags,
        __CONST_SOCKADDR_ARG _pAddr, socklen_t _cAddrLen)
{
    do {
        ssize_t nSent = 0;

        for (int nRetry = 5; nRetry >= 0; --nRetry) {
            nSent = sendto(_cSock, _pBuf, _cLen, _nFlags, _pAddr, _cAddrLen);

            if (nSent >= 0)
                break;

            if (nSent < 0 && (errno != ECONNREFUSED || nRetry == 0))
                throw SystemError("send()");

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        _cLen -= nSent;
        _pBuf += nSent;
    } while (_cLen > 0);
}

auto LostPacketDetector::collect() -> Stats {
    Stats stats;
    std::sort(m_records.begin(), m_records.end());
    if (m_records.empty())
        return stats;
    stats.cPackets = 1;
    stats.cSize = m_records.front().cSize;
    for (size_t c = 1; c < m_records.size(); ++c) {
        const Record & prev = m_records[c - 1],
                & cur = m_records[c];
        if (cur.nOffset == prev.nOffset && cur.cSize == prev.cSize) {
            ++stats.cPacketsDuplicated;
            stats.cSizeDuplicated += cur.cSize;
        } else {
            ++stats.cPackets;
            stats.cSize += cur.cSize;
            if (cur.nOffset > prev.nOffset + (long)prev.cSize) {
                ++stats.cPacketsLost;
                stats.cSizeLost += cur.nOffset - prev.nOffset - prev.cSize;
            } else if (cur.nOffset < prev.nOffset + (long)prev.cSize) {
                ++stats.cPacketsOverlapped;
                stats.cSizeOverlapped += prev.nOffset + prev.cSize - cur.nOffset;
            }
        }
    }
    m_records.clear();
    return stats;
}
