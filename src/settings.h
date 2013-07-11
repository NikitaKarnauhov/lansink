/*
 * settings.h
 *
 *  Created on: May 10, 2013
 *      Author: nikita.karnauhov@gmail.com
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

#include <string>

#include <stdio.h>

#ifdef P_tmpdir
#define UNAP_TEMP_PREFIX P_tmpdir
#else
#define UNAP_TEMP_PREFIX "/tmp"
#endif

#define UNAP_PROGRAM_NAME "unapd"

struct Settings {
    std::string strLogPath = UNAP_TEMP_PREFIX "/" UNAP_PROGRAM_NAME ".log";
    std::string strPIDPath = UNAP_TEMP_PREFIX "/" UNAP_PROGRAM_NAME ".pid";
    int nLogLevel = 0;
    bool bDaemon = false;
};

extern Settings g_settings;

class SettingsParser {
public:
    SettingsParser();
    ~SettingsParser();

    const Settings &get() const;

    void parse_file(const std::string &_strFIlename);
    void parse_option(const std::string &_strOption, const std::string &_strValue);

private:
    class Impl;
    Impl *m_pImpl;
};

#endif /* SETTINGS_H_ */
