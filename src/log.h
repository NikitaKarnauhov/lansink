/*
    log.h

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

#ifndef UNAP_LOG_H_
#define UNAP_LOG_H_

#include <string>

enum LogLevel {
    llSilent,
    llError,
    llWarning,
    llInfo,
    llDebug,
};

class Log {
public:
    Log();

    // Empty filename means stderr.
    Log(const std::string &_strFilename);
    ~Log();

    void open(const std::string &_strFilename);
    void close(const std::string &_strFilename);

    void setLevel(LogLevel _ll);

    void log(LogLevel _level, const std::string &_strMessage);
    void log(LogLevel _level, const char *_strFormat, ...);

#define LOG_LEVEL(_NAME, _LEVEL)                        \
    void _NAME(const std::string &_strMessage) {        \
        log(_LEVEL, _strMessage);                      \
    }                                                   \
                                                        \
    template<typename... Args>                          \
    void _NAME(const char *_strFormat, Args... _args) { \
        log(_LEVEL, _strFormat, _args...);             \
    }

    LOG_LEVEL(error, llError);
    LOG_LEVEL(warning, llWarning);
    LOG_LEVEL(info, llInfo);
    LOG_LEVEL(debug, llDebug);

private:
    class Impl;
    Impl *m_pImpl;
};

#endif /* UNAP_LOG_H_ */
