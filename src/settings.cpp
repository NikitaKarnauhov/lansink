/*
 * settings.cpp
 *
 *  Created on: May 14, 2013
 *      Author: nikita.karnauhov@gmail.com
 */

#include "settings.h"

#include <stdexcept>
#include <iostream>
#include <sstream>
#include <fstream>
#include <locale>
#include <memory>

#include "lexer.h"
#include "exception.h"

Settings g_settings;

struct LessIC {
    bool operator ()(const std::wstring &_strLhs, const std::wstring &_strRhs) const {
        return std::lexicographical_compare(_strLhs.begin(), _strLhs.end(),
                _strRhs.begin(), _strRhs.end(),
                [](wchar_t _chLhs, wchar_t _chRhs) {
                    return std::towlower(_chLhs) < std::towlower(_chRhs);
                });
    }
};

struct SettingsParser::Impl {
    Settings settings;

    std::map<std::wstring, bool *, LessIC> boolSettings{
        {L"daemon", &settings.bDaemon}
    };

    std::map<std::wstring, int *, LessIC> intSettings{
        {L"log-level", &settings.nLogLevel}
    };

    std::map<std::wstring, std::string *, LessIC> stringSettings{
        {L"log-path", &settings.strLogPath}
    };

    void parse(std::wistream &_is);
};

void SettingsParser::Impl::parse(std::wistream &_is) {
    lexer::Lexer lexer(_is);

    for (auto iToken = lexer.begin(); iToken != lexer.end(); ++iToken) {
        if (iToken->kind == lexer::EndOfFile)
            break;

        if (iToken->kind == lexer::Whitespace || iToken->kind == lexer::Comment)
            continue;

        if (iToken->kind != lexer::Identifier)
            throw RuntimeError("%d, %d: Identifier expected, got \"%ls\"",
                    iToken->nLine, iToken->nCol, iToken->strData.c_str());

        const lexer::Token &key = *iToken;

        do
            ++iToken;
        while (iToken->kind == lexer::Whitespace);

        if (iToken->kind == lexer::EndOfFile)
            throw RuntimeError("%d, %d: Unexpected end of file", iToken->nLine, iToken->nCol);

        {
            auto iSetting = boolSettings.find(key.strData);

            if (iSetting != boolSettings.end()) {
                const std::set<std::wstring, LessIC> yesStrings{L"1", L"on", L"yes", L"y", L"true"};
                const std::set<std::wstring, LessIC> noStrings{L"0", L"off", L"no", L"n", L"false"};
                auto iValue = yesStrings.find(iToken->strData);

                if (iValue != yesStrings.end()) {
                    *iSetting->second = true;
                    continue;
                }

                if (iValue != noStrings.end()) {
                    *iSetting->second = false;
                    continue;
                }

                throw RuntimeError("%d, %d: Boolean value expected, got \"%ls\"",
                        iToken->nLine, iToken->nCol, iToken->strData.c_str());
            }
        }

        {
            auto iSetting = intSettings.find(key.strData);

            if (iSetting != intSettings.end()) {
                if (iToken->kind != lexer::Integer)
                    throw RuntimeError("%d, %d: Integer value expected, got \"%ls\"",
                            iToken->nLine, iToken->nCol, iToken->strData.c_str());

                std::wstringstream(iToken->strData) >> *iSetting->second;
                continue;
            }
        }

        {
            auto iSetting = stringSettings.find(key.strData);

            if (iSetting != stringSettings.end()) {
                const std::unique_ptr<char []> buf(new char[iToken->strData.size()*6 + 1]);
                const std::locale loc;

                std::use_facet<std::ctype<wchar_t> >(loc).narrow(iToken->strData.data(),
                        iToken->strData.data() + iToken->strData.size() + 1, '?', buf.get());
                *iSetting->second = buf.get();
                continue;
            }
        }

        throw RuntimeError("%d, %d: Unknown config parameter \"%ls\"",
                key.nLine, key.nCol, key.strData.c_str());
    }
}

SettingsParser::SettingsParser() : m_pImpl(new Impl()) {
}

SettingsParser::~SettingsParser() {
    delete m_pImpl;
}

const Settings &SettingsParser::get() const {
    return m_pImpl->settings;
}

void SettingsParser::parse_file(const std::string &_strFilename) {
    std::wifstream is(_strFilename);

    if (is.bad())
        throw std::runtime_error(std::string("Cannot open config file: ") + _strFilename.c_str());

    try {
        m_pImpl->parse(is);
    } catch (std::exception &e) {
        throw RuntimeError("Failed parsing %s: %s", _strFilename.c_str(), e.what());
    }
}

void SettingsParser::parse_option(const std::string &_strOption, const std::string &_strValue) {
    std::wstringstream ss;
    const std::unique_ptr<wchar_t []> buf(new wchar_t[_strOption.size() + _strValue.size() + 4]);
    const std::locale loc;
    const std::string str = _strOption + " \"" + _strValue + "\"";

    std::use_facet<std::ctype<wchar_t> >(loc).widen(str.data(), str.data() + str.size(), buf.get());
    ss << buf.get();
    m_pImpl->parse(ss);
}
