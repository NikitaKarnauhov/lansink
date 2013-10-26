/*
    sink.h

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

#ifndef LANSINK_SINK_H_
#define LANSINK_SINK_H_

#include <stdlib.h>

#include <string>

#include "exception.h"
#include "log.h"

class Sink {
public:
    class Error : public RuntimeError {
    public:
        enum Kind {
            seNormal, seUnderrun, seFatal
        };

        Error(Kind _kind, int _nError, const std::string &_strMessage) :
            RuntimeError(_strMessage), m_kind(_kind), m_nError(_nError)
        {
        }

        int get_error() const {
            return m_nError;
        }

        int get_kind() const {
            return m_kind;
        }

        bool is_normal() const {
            return m_kind == seNormal;
        }

        bool is_underrun() const {
            return m_kind == seUnderrun;
        }

        bool is_fatal() const {
            return m_kind == seFatal;
        }

    private:
        Kind m_kind;
        int m_nError;
    };

    virtual ~Sink() {}

    virtual bool is_prepared() const = 0;
    virtual void init(size_t _cChannels, const size_t _cRate, const std::string &_strFormat) = 0;
    virtual bool prepare() = 0;
    virtual void pause(bool _bEnable) = 0;
    virtual unsigned long get_buffer_size() const = 0;
    virtual unsigned long get_period_size() const = 0;
    virtual unsigned int get_rate() const = 0;
    virtual unsigned int get_frame_bytes() const = 0;
    virtual bool can_be_paused() const = 0;
    virtual long get_delay() = 0;
    virtual long get_avail(bool _bSync) = 0;
    virtual void recover(int _nError) = 0;
    virtual void report_state() = 0;
    virtual bool is_buffering() const = 0;
    virtual bool is_running() const = 0;
    virtual bool is_paused() const = 0;
    virtual bool is_underrun() const = 0;
    virtual void start() = 0;
    virtual void close() = 0;
    virtual long write(const void *_pBuffer, unsigned long _cFrames) = 0;
};

Sink *create_alsa_sink(Log &_log);
Sink *create_pulse_sink(Log &_log);

#endif /* LANSINK_SINK_H_ */
