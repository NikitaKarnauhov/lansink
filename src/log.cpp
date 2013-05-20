/*
 * log.cpp
 *
 *  Created on: Apr 29, 2013
 *      Author: nikita.karnauhov@gmail.com
 */

#include "log.h"
#include "exception.h"

#include <map>
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

        if (pStream->bad()) {
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

    switch (_level) {
        case llDebug:
            strHeader += "DEBUG: ";
            break;
        case llInfo:
            strHeader += "INFO: ";
            break;
        case llWarning:
            strHeader += "WARNING: ";
            break;
        case llError:
            strHeader += "ERROR: ";
            break;
    }

    for (auto &stream : m_streams)
        *stream.second << strHeader << _strMessage << std::endl;
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
