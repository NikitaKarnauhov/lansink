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
#include <assert.h>

#include <string>
#include <deque>

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

#endif /* LANSINK_UTILS_H_ */
