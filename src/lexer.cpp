/*
 * lexer.cpp
 *
 *  Created on: Jul 3, 2012
 *      Author: nikita.karnauhov@gmail.com
 */

#include "lexer.h"

#include <iostream>
#include <functional>
#include <vector>
#include <assert.h>

using namespace lexer;

Lexer::Lexer(std::wistream &_is) :
    m_is(_is), m_ctype(std::use_facet<std::ctype<wchar_t> >(_is.getloc())),
    m_tokenKinds{
        { L",", TK::Comma },
        { L";", TK::Semicolon },
        { L":", TK::Colon },
        { L"|", TK::Pipe },
        { L"=", TK::Assignment },
        { L"!", TK::Bang },
        { L"$", TK::Dollar },
        { L"(", TK::LParen },
        { L")", TK::RParen },
        { L"[", TK::LBracket },
        { L"]", TK::RBracket },
        { L"{", TK::LBrace },
        { L"}", TK::RBrace },
        { L"->", TK::Arrow },
    }
{
}

bool Lexer::_is_space(wchar_t _c) const {
    return m_ctype.is(std::ctype_base::space, _c);
}

bool Lexer::_is_digit(wchar_t _c) const {
    return m_ctype.is(std::ctype_base::digit, _c);
}

bool Lexer::_is_xdigit(wchar_t _c) const {
    return m_ctype.is(std::ctype_base::xdigit, _c);
}

bool Lexer::_is_alpha(wchar_t _c) const {
    return m_ctype.is(std::ctype_base::alpha, _c);
}

bool Lexer::_is_alnum(wchar_t _c) const {
    return m_ctype.is(std::ctype_base::alnum, _c);
}

bool Lexer::_is_raw(wchar_t _c) const {
    switch (_c) {
        case L'`':
        case L'~':
        case L'!':
        case L'@':
        case L'#':
        case L'%':
        case L'^':
        case L'&':
        case L'-':
        case L'+':
        case L'\\':
        case L'<':
        case L'>':
        case L'/':
        case L'?':
        case L'"':
        case L'.':
            return true;
        default:
            return false;
    }
}

std::wstring Lexer::_read_digits() {
    std::wstring s;
    wchar_t c = 0;

    while (m_is.get(c).good()) {
        if (!_is_digit(c)) {
            m_is.unget();
            break;
        }

        s += c;
    }
    
    return s;
}

static
void _append(Token &_tok, wchar_t _c, bool _bDecoded = false) {
    // Recognized patterns: \n, \r\n, \r.

    if (!_bDecoded && (_c == L'\r' || _c == L'\n')) {
        const wchar_t d = _tok.strData.empty() ? 0 : _tok.strData[_tok.strData.size() - 1];

        if (_c == L'\r' || d != L'\r')
            ++_tok.nLine;

        _tok.nCol = 0;
    } else
        ++_tok.nCol;

    _tok.strData += _c;
}

static
void _append(Token &_tok, const std::wstring &_s) {
    _tok.strData.reserve(_tok.strData.size() + _s.size());
    for (wchar_t c : _s)
        _append(_tok, c);
}

bool Lexer::_read_whitespace(Token &_tok) {
    if (!m_is.good() || !_is_space(m_is.peek()))
        return false;

    _tok.kind = TK::Whitespace;
    _tok.strData.clear();

    wchar_t c;

    while (m_is.get(c).good()) {
        if (!_is_space(c)) {
            m_is.unget();
            break;
        }

        _append(_tok, c);
    }

    return true;
}

bool Lexer::_read_comment(Token &_tok) {
    if (!m_is.good() || m_is.peek() != L'/')
        return false;

    m_is.ignore(1);

    if (!m_is.good()) {
        m_is.unget();
        return false;
    }

    enum class CommentKind { Single, Multi } ck;

    if (m_is.peek() == L'/')
        ck = CommentKind::Single;
    else if (m_is.peek() == L'*')
        ck = CommentKind::Multi;
    else {
        m_is.unget(); // Put back the initial slash.
        return false;
    }

    _tok.kind = TK::Comment;
    _append(_tok, L'/');

    wchar_t c;

    if (ck == CommentKind::Single) {
        while (m_is.get(c).good()) {
            if (c == L'\n' || c == L'\r') {
                m_is.unget();
                break;
            }

            _append(_tok, c);
        }
    } else {
        assert(ck == CommentKind::Multi);

        while (m_is.get(c).good()) {
            _append(_tok, c);

            if (c == L'*' && m_is.peek() == L'/') {
                _append(_tok, m_is.get());
                break;
            }
        }
    }

    return true;
}

bool Lexer::_read_integer(Token& _tok) {
    if (!m_is.good() || !_is_digit(m_is.peek()))
        return false;

    wchar_t c = m_is.get();

    _tok.kind = TK::Integer;
    _append(_tok, c);

    if (!m_is.good())
        return true;

    if (c == L'0' && m_is.peek() == L'x') {
        m_is.ignore(1);

        if (_is_xdigit(m_is.peek())) {
            _append(_tok, L'x');

            while (m_is.get(c).good()) {
                if (!_is_xdigit(c)) {
                    m_is.unget();
                    break;
                }

                _append(_tok, c);
            }
        } else
            m_is.unget();
    } else
        _append(_tok, _read_digits());

    return true;
}

bool Lexer::_read_real(Token &_tok) {
    if (!m_is.good() || !(_is_digit(m_is.peek()) || m_is.peek() == L'.'))
        return false;

    wchar_t c = m_is.get();

    if (c == L'.' && !_is_digit(m_is.peek())) {
        m_is.unget();
        return false;
    }

    _tok.kind = TK::Real;
    _append(_tok, c);
    _append(_tok, _read_digits());

    if (m_is.peek() == L'.') {
        _append(_tok, m_is.get());
        _append(_tok, _read_digits());
    } else if (m_is.peek() != L'e' && m_is.peek() != L'E') {
        for (size_t i = 0; i < _tok.strData.size(); ++i)
            m_is.putback(_tok.strData[_tok.strData.size() - i - 1]);
        return false;
    }

    if (m_is.peek() == L'e' || m_is.peek() == L'E') {
        c = m_is.get();

        if (m_is.peek() == L'-' || m_is.peek() == L'+') {
            const wchar_t d = m_is.get();

            if (!_is_digit(m_is.peek())) {
                m_is.putback(d);
                m_is.putback(c);
                return true;
            }

            _append(_tok, c);
            _append(_tok, d);
        } else if (!_is_digit(m_is.peek())) {
            m_is.unget();
            return true;
        } else
            _append(_tok, c);

        _append(_tok, _read_digits());
    }

    return true;
}

bool Lexer::_read_char(Token &_tok) {
    if (m_is.peek() != L'\\') {
        _append(_tok, m_is.get(), true);
        return true;
    }

    m_is.get(); // Back slash.

    auto decode = [&]() {
        const wchar_t d = m_is.get();
        std::wstring s;

        // fragment ESCAPED_CHAR  : '\\' ('\'' | '\"' | '\\' | '0' | 'n' | 't' | 'r');
        // fragment CHAR_CODE     : '\\x' HEX_DIGIT (HEX_DIGIT (HEX_DIGIT HEX_DIGIT?)?)?;
        switch (d) {
            case L'0': return L'\0';
            case L'n': return L'\n';
            case L't': return L'\t';
            case L'r': return L'\r';
            case L'x':
                for (int i = 0; i < 4 && _is_xdigit(m_is.peek()); ++ i)
                    s += m_is.get();

                if (s.empty())
                    break;

                return (wchar_t)wcstol(s.data(), NULL, 16);
        }

        return d;
    };

    _append(_tok, decode(), true);

    return true;
}

bool Lexer::_read_string(Token &_tok) {
    if (!m_is.good() || m_is.peek() != L'"')
        return false;

    _tok.kind = TK::String;
    m_is.get(); // Quote.

    while (m_is.good() && m_is.peek() != L'"')
        if (!_read_char(_tok))
            return false;

    if (m_is.peek() != L'"')
        return false;

    m_is.get(); // Quote.

    return true;
}

bool Lexer::_read_identifier(Token &_tok) {
    if (!m_is.good() || !(_is_alpha(m_is.peek()) || m_is.peek() == L'_' || m_is.peek() == L'-'))
        return false;

    wchar_t c = m_is.get();

    _append(_tok, c);

    if (c == L'_' || c == L'-') {
        while (m_is.get(c).good()) {
            if (c != L'_' && c != L'-') {
                m_is.unget();
                break;
            }

            _append(_tok, c);
        }

        if (!_is_alnum(c)) {
            // Order doesn't matter, it's all underscores.
            for (wchar_t d : _tok.strData)
                m_is.putback(d);
            return false;
        }
    }

    _tok.kind = TK::Identifier;

    while (m_is.get(c).good()) {
        if (!_is_alnum(c) && c != L'_' && c != L'-') {
            m_is.unget();
            break;
        }

        _append(_tok, c);
    }

    return true;
}

bool Lexer::_read_raw(Token &_tok) {
    if (!m_is.good() || !_is_raw(m_is.peek()))
        return false;

    _tok.kind = TK::Raw;

    wchar_t c;

    while (m_is.get(c).good()) {
        if (!_is_raw(c)) {
            m_is.unget();
            break;
        }

        _append(_tok, c);
    }

#if 0
    Token ident;

    if (!_read_identifier(ident)) {
        for (auto i = _tok.strData.rend(); i != _tok.strData.rbegin(); ++i)
            m_is.putback(*i);
        return false;
    }

    _append(_tok, ident.strData));
#endif

    return true;
}

bool Lexer::_read_lexeme(Token &_tok) {
    wchar_t c = m_is.peek();

    if (!m_is.good())
        return false;

    const wchar_t strLead[]{c, 0};
    auto iTK = m_tokenKinds.lower_bound(strLead);

    if (iTK == m_tokenKinds.end() || iTK->first[0] != c)
        return false;

    _append(_tok, c);
    _tok.kind = TK::None;
    m_is.ignore(1);

    if (iTK->first.size() == 1) {
        _tok.kind = iTK->second;
        return true;
    }

    while (m_is.get(c).good()) {
        _append(_tok, c);
        iTK = m_tokenKinds.lower_bound(_tok.strData);

        const size_t cLen = _tok.strData.size();

        if (iTK == m_tokenKinds.end() ||
                _tok.strData.compare(0, cLen, iTK->first, 0, cLen) != 0)
            break;

        if (iTK->first.size() == _tok.strData.size()) {
            _tok.kind = iTK->second;
            break;
        }
    }

    if (_tok.kind == TK::None) {
        for (size_t i = 0; i < _tok.strData.size(); ++i)
            m_is.putback(_tok.strData[_tok.strData.size() - i - 1]);
        return false;
    }

    return true;
}

bool Lexer::_is_eof() const {
    return !m_tokens.empty() && (std::prev(m_tokens.end())->kind == TK::EndOfFile ||
        std::prev(m_tokens.end())->kind == TK::Error);
}

using std::placeholders::_1;

Lexer::Tokens::iterator Lexer::_read() {
    if (m_is.tellg() != m_pos)
        m_is.seekg(m_pos);

    if (_is_eof())
        return std::prev(m_tokens.end());

    if (!m_is.good())
        return m_tokens.insert(Token(TK::EndOfFile, m_pos, L"", m_nLine, m_nCol)).first;

    // Order of reader functions below does matter.
    std::vector<std::function<bool(Token &)> > readers{
        std::bind(&Lexer::_read_whitespace, this, _1),
        std::bind(&Lexer::_read_comment, this, _1),
        std::bind(&Lexer::_read_real, this, _1),
        std::bind(&Lexer::_read_integer, this, _1),
        std::bind(&Lexer::_read_string, this, _1),
        std::bind(&Lexer::_read_lexeme, this, _1),
        std::bind(&Lexer::_read_identifier, this, _1),
        std::bind(&Lexer::_read_raw, this, _1)
    };

    for (auto reader : readers) {
        Token tok(TK::None, m_pos, L"", m_nLine, m_nCol);

        if (reader(tok)) {
            m_pos = m_is.tellg();
            std::swap(m_nLine, tok.nLine);
            std::swap(m_nCol, tok.nCol);
            return m_tokens.insert(tok).first;
        }
    }

    return m_tokens.insert(Token(m_is.good() ? TK::Error : TK::EndOfFile, m_pos, L"", m_nLine, m_nCol)).first;
}

Lexer::iterator Lexer::begin() {
    return iterator(Lexer::IteratorImpl(*this, m_tokens.empty() ? _read() : m_tokens.begin()));
}

Lexer::iterator Lexer::end() {
    return iterator(Lexer::IteratorImpl(*this, m_tokens.end()));
}

int Lexer::define(const std::wstring &_s) {
    const int nTK = m_nDefinition++;
    m_tokenKinds[_s] = nTK;
    return nTK;
}

// IteratorImpl.

Lexer::IteratorImpl::IteratorImpl(const Lexer::IteratorImpl &_other) :
    m_pLexer(_other.m_pLexer), m_nUpdateCount(_other.m_nUpdateCount), m_iToken(_other.m_iToken),
    m_pos(_other.m_pos)
{
}

Lexer::IteratorImpl::IteratorImpl(Lexer &_lexer) :
    m_pLexer(&_lexer), m_nUpdateCount(_lexer.m_nUpdateCount), m_iToken(_lexer.m_tokens.begin()),
    m_pos(0)
{
}

Lexer::IteratorImpl::IteratorImpl(Lexer &_lexer, Lexer::Tokens::iterator _iToken) :
    m_pLexer(&_lexer), m_nUpdateCount(_lexer.m_nUpdateCount), m_iToken(_iToken),
    m_pos(m_iToken == m_pLexer->m_tokens.end() ? std::streampos(0) : m_iToken->pos)
{
}

void Lexer::IteratorImpl::validate() {
    if (m_nUpdateCount < m_pLexer->m_nUpdateCount) {
        if (m_iToken != m_pLexer->m_tokens.end()) {
            Token tok(TK::None, m_pos);

            m_iToken = m_pLexer->m_tokens.lower_bound(tok);

            if (m_iToken == m_pLexer->m_tokens.end() && !m_pLexer->m_tokens.empty())
                m_iToken = std::prev(m_pLexer->m_tokens.end());

            while (m_iToken == m_pLexer->m_tokens.end() || (m_iToken->kind != TK::EndOfFile &&
                    m_iToken->kind != TK::Error &&
                    m_iToken->pos + std::streampos(m_iToken->get_length()) <= m_pos))
            {
                m_iToken = m_pLexer->_read();

                if (m_iToken == m_pLexer->m_tokens.end())
                    break;
            }
        }

        m_nUpdateCount = m_pLexer->m_nUpdateCount;
        m_pos = m_iToken == m_pLexer->m_tokens.end() ? std::streampos(0) : m_iToken->pos;
    }
}

bool Lexer::IteratorImpl::move_next() {
    validate();

    if (m_iToken == m_pLexer->m_tokens.end())
        return false;

    if (m_iToken->kind == TK::EndOfFile || m_iToken->kind == TK::Error) {
        m_pos = 0;
        m_iToken = m_pLexer->m_tokens.end();
        return false;
    }

    ++m_iToken;

    if (m_iToken == m_pLexer->m_tokens.end())
        m_iToken = m_pLexer->_read();

    m_pos = m_iToken == m_pLexer->m_tokens.end() ? std::streampos(0) : m_iToken->pos;

    return m_iToken != m_pLexer->m_tokens.end();
}

bool Lexer::IteratorImpl::move_prev() {
    validate();

    if (m_iToken == m_pLexer->m_tokens.begin())
        return false;

    --m_iToken;
    m_pos = m_iToken->pos;

    return true;
}
