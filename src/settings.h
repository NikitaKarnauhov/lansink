/*
    settings.h

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
    std::string strHost = "";
    std::string strALSADevice = "default";
    int nLogLevel = 0;
    int nPort = 26751;
    int nRecoveryTimeout = 120;
    int nOpenTimeout = 2000;
    int nBufferedPackets = 10;
    bool bDaemon = false;
    bool bExclusive = false;
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
