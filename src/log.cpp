/*
    log.cpp

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

#include "log.h"
#include "exception.h"

#include <map>
#include <list>
#include <mutex>
#include <chrono>
#include <ctime>
#include <iostream>
#include <fstream>

#include <stdarg.h>

class Log::Impl {
public:
    ~Impl();

    void open(const std::string &_strFilename);
    void close(const std::string &_strFilename);

    void log(LogLevel _level, const std::string &_strMessage);

private:
    std::mutex m_mutex;
    std::map<std::string, std::ostream *> m_streams;
    LogLevel m_level = llSilent;

    friend class Log;
};

Log::Impl::~Impl() {
    for (auto &stream : m_streams)
        if (!stream.first.empty())
            delete stream.second;
}

void Log::Impl::open(const std::string &_strFilename) {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::ostream *&pStream = m_streams[_strFilename];

    if (!pStream) {
        pStream = _strFilename.empty() ? &std::cerr :
                new std::ofstream(_strFilename, std::ios::app);

        if (!pStream->good()) {
            m_streams.erase(_strFilename);
            throw RuntimeError("Cannot open log file: %s", _strFilename.c_str());
        }
    }
}

void Log::Impl::close(const std::string &_strFilename) {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto iStream = m_streams.find(_strFilename);

    if (iStream != m_streams.end()) {
        if (!iStream->first.empty())
            delete iStream->second;
        m_streams.erase(iStream);
    }
}

void Log::Impl::log(LogLevel _level, const std::string &_strMessage) {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_level == llSilent || _level > m_level)
        return;

    if (m_streams.empty())
        return;

    const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    struct tm t;
    constexpr size_t c_cMaxLength = 1024;
    char str[c_cMaxLength];

    localtime_r(&now, &t);
    strftime(str, c_cMaxLength - 1, "%Y-%m-%dT%H:%M:%S%z", &t);
    str[c_cMaxLength - 1] = 0;

    std::string strHeader = std::string(str) + ": ";
    std::string strLevel;

    switch (_level) {
        case llDebug:
            strLevel = "DEBUG: ";
            break;
        case llInfo:
            strLevel = "INFO: ";
            break;
        case llWarning:
            strLevel = "WARNING: ";
            break;
        case llError:
            strLevel = "ERROR: ";
            break;
        default:
            throw LogicError("Unexpected log level %d", _level);
            break;
    }

    std::list<std::string> messages{{strLevel + _strMessage}};

    while (!messages.empty()) {
        for (auto &stream : m_streams) {
            *stream.second << strHeader << messages.front() << std::endl;

            if (!stream.second->good()) {
                messages.push_back(std::string("ERROR: failed writing to ") +
                        (stream.first.empty() ? std::string("(stderr)") : stream.first));

                if (!stream.first.empty())
                    delete stream.second;

                m_streams.erase(stream.first);
            }
        }

        messages.pop_front();
    }
}

Log::Log() : m_pImpl(new Impl) {

}

Log::Log(const std::string &_strFilename) : m_pImpl(new Impl) {
    open(_strFilename);
}

Log::~Log() {
    delete m_pImpl;
}

void Log::open(const std::string &_strFilename) {
    m_pImpl->open(_strFilename);
}

void Log::close(const std::string &_strFilename) {
    m_pImpl->close(_strFilename);
}

void Log::setLevel(LogLevel _ll) {
    m_pImpl->m_level = _ll;
}

void Log::log(LogLevel _level, const std::string &_strMessage) {
    m_pImpl->log(_level, _strMessage);
}

void Log::log(LogLevel _level, const char *_strFormat, ...) {
    constexpr size_t c_cMaxLength = 1024;
    char str[c_cMaxLength];
    va_list args;

    va_start(args, _strFormat);
    str[c_cMaxLength - 1] = 0;
    vsnprintf(str, c_cMaxLength - 1, _strFormat, args);
    va_end (args);
    m_pImpl->log(_level, str);
}

LogLevel Log::getLevel() const {
    return m_pImpl->m_level;
}
