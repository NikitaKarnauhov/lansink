/*
 * log.h
 *
 *  Created on: Apr 21, 2013
 *      Author: nikita.karnauhov@gmail.com
 */

#ifndef UNAP_LOG_H_
#define UNAP_LOG_H_

#include <string>

enum LogLevel {
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

    void log(LogLevel _level, const std::string &_strMessage);
    void log(LogLevel _level, const char *_strFormat, ...);

private:
    class Impl;
    Impl *m_pImpl;
};

#endif /* UNAP_LOG_H_ */
