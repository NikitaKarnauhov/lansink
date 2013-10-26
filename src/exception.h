/*
    exception.h

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

#ifndef LANSINK_EXCEPTION_H_
#define LANSINK_EXCEPTION_H_

#include <string>
#include <stdexcept>

#include <string.h>

#include "utils.h"

template<typename _Base>
class Exception : public _Base {
public:
    template<typename... Args>
    Exception(const char *_strFormat, Args... _args) :
        _Base(format(1024, _strFormat, _args...))
    {
    }

    Exception(const std::string &_strMessage) :
        _Base(_strMessage)
    {
    }
};

using RuntimeError = Exception<std::runtime_error>;
using LogicError = Exception<std::logic_error>;

class SystemError : public RuntimeError {
public:
    template<typename... Args>
    SystemError(const char *_strFormat, Args... _args) :
        RuntimeError((std::string(_strFormat) + ": %s").c_str(), _args..., strerror(errno)),
        m_nError(errno)
    {
    }

    int get_error() const {
        return m_nError;
    }

private:
    int m_nError;
};

#endif /* LANSINK_EXCEPTION_H_ */
