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

#ifndef LANSINK_LOG_H_
#define LANSINK_LOG_H_

#include <string>
#include <functional>

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
    LogLevel getLevel() const;

    using Decorator = std::function<std::string(const std::string &)>;

    static std::string emptyDecorator(const std::string &_str) {
        return _str;
    }

    void log(LogLevel _level, const Decorator &_decorator,
            const std::string &_strMessage);
    void log(LogLevel _level, const Decorator &_decorator,
            const char *_strFormat, ...);

#define LOG_LEVEL(_NAME, _LEVEL)                                \
    void _NAME(const std::string &_strMessage) {                \
        if (getLevel() > llSilent && _LEVEL <= getLevel())      \
            log(_LEVEL, emptyDecorator, _strMessage);           \
    }                                                           \
                                                                \
    void _NAME(const Decorator &_decorator,                     \
            const std::string &_strMessage)                     \
    {                                                           \
        if (getLevel() > llSilent && _LEVEL <= getLevel())      \
            log(_LEVEL, _decorator, _strMessage);               \
    }                                                           \
                                                                \
    template<typename... Args>                                  \
    void _NAME(const char *_strFormat, Args... _args) {         \
        if (getLevel() > llSilent && _LEVEL <= getLevel())      \
            log(_LEVEL, emptyDecorator, _strFormat, _args...);  \
    }                                                           \
                                                                \
    template<typename... Args>                                  \
    void _NAME(const Decorator &_decorator,                     \
            const char *_strFormat, Args... _args)              \
    {                                                           \
        if (getLevel() > llSilent && _LEVEL <= getLevel())      \
            log(_LEVEL, _decorator, _strFormat, _args...);      \
    }

    LOG_LEVEL(error, llError);
    LOG_LEVEL(warning, llWarning);
    LOG_LEVEL(info, llInfo);
    LOG_LEVEL(debug, llDebug);
#undef LOG_LEVEL

private:
    class Impl;
    Impl *m_pImpl;
};

class DecoratedLog {
public:
    DecoratedLog(Log &_log, const Log::Decorator &_decorator) :
        m_log(_log), m_decorator(_decorator)
    {
    }

    void log(LogLevel _level, const std::string &_strMessage) {
        m_log.log(_level, m_decorator, _strMessage);
    }

    template<typename... Args>
    void log(LogLevel _level, const char *_strFormat, Args... _args) {
        m_log.log(m_decorator, _strFormat, _args...);
    }

#define WRAP(_NAME)                                     \
    void _NAME(const std::string &_strMessage) {        \
        m_log._NAME(m_decorator, _strMessage);          \
    }                                                   \
                                                        \
    template<typename... Args>                          \
    void _NAME(const char *_strFormat, Args... _args) { \
        m_log._NAME(m_decorator, _strFormat, _args...); \
    }

    WRAP(error);
    WRAP(warning);
    WRAP(info);
    WRAP(debug);
#undef WRAP

private:
    Log &m_log;
    const Log::Decorator m_decorator;
};

#endif /* LANSINK_LOG_H_ */
