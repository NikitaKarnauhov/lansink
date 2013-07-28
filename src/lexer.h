/*
    lexer.h

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

#ifndef LEXER_H_
#define LEXER_H_

#include <string>
#include <iostream>
#include <set>
#include <locale>
#include <map>

namespace lexer {

enum TK {
    None,

    Whitespace, Comment, Integer, Real, String, Identifier, Raw,

    Comma, Semicolon, Colon, Pipe, Assignment, Bang, Dollar, LParen, RParen,
    LBracket, RBracket, LBrace, RBrace, Arrow,

    EndOfFile,
    Error,
};

struct Position {
    std::streampos pos;
    int nLine, nCol;

    Position(std::streampos _pos = 0, int _nLine = 0, int _nCol = 0) :
        pos(_pos), nLine(_nLine), nCol(_nCol)
    {
    }
};

struct Token : public Position {
    int kind;
    std::wstring strData;

    Token(int _kind = TK::None, std::streampos _pos = 0, const std::wstring _strData = L"",
          int _nLine = 0, int _nCol = 0) :
        Position(_pos, _nLine, _nCol), kind(_kind), strData(_strData)
    {
    }

    bool operator <(const Token &_other) const { return pos < _other.pos; }
    size_t get_length() const { return strData.size(); }
};

template<class _Impl>
struct TokenIterator {
    _Impl impl;

    TokenIterator(_Impl &&_impl) : impl(_impl) {}

    inline TokenIterator &operator ++();
    inline TokenIterator operator ++(int);
    inline TokenIterator &operator --();
    inline TokenIterator operator --(int);
    inline bool operator ==(const TokenIterator &) const;
    inline bool operator !=(const TokenIterator &) const;
    inline const Token &operator *() const;
    inline const Token *operator ->() const;
};

class Lexer {
private:
    class IteratorImpl;

public:
    typedef std::set<Token> Tokens;
    typedef TokenIterator<IteratorImpl> iterator;

    Lexer(std::wistream &_is);
    int define(const std::wstring &_s);

    iterator begin();
    iterator end();

private:
    class IteratorImpl {
    public:
        IteratorImpl(const IteratorImpl &_other);
        IteratorImpl(Lexer &_lexer);
        IteratorImpl(Lexer &_lexer, Tokens::iterator _iToken);
        void validate();
        bool move_next();
        bool move_prev();
        bool equals(const IteratorImpl &_other) const { return m_iToken == _other.m_iToken; }
        const Token &get() const { return *m_iToken; }

    protected:
        Lexer *m_pLexer;
        int m_nUpdateCount;
        Lexer::Tokens::iterator m_iToken;
        std::streampos m_pos;
    };

    std::wistream &m_is;
    const std::ctype<wchar_t> &m_ctype;
    Tokens m_tokens;
    int m_nUpdateCount = 0;
    std::streampos m_pos = 0;
    int m_nLine = 0, m_nCol = 0;
    std::map<std::wstring, int> m_tokenKinds;
    int m_nDefinition = 0;

    void _invalidate();

    // Read and store a single token.
    Tokens::iterator _read();
    bool _read_whitespace(Token &_tok);
    bool _read_comment(Token &_tok);
    bool _read_integer(Token &_tok);
    bool _read_real(Token &_tok);
    bool _read_char(Token &_tok);
    bool _read_string(Token &_tok);
    bool _read_identifier(Token &_tok);
    bool _read_raw(Token &_tok);
    bool _read_lexeme(Token &_tok);

    bool _is_space(wchar_t _c) const;
    bool _is_digit(wchar_t _c) const;
    bool _is_xdigit(wchar_t _c) const;
    bool _is_alpha(wchar_t _c) const;
    bool _is_alnum(wchar_t _c) const;
    bool _is_raw(wchar_t _c) const;

    bool _is_eof() const;

    std::wstring _read_digits();

    friend class Lexer::IteratorImpl;
};

// Implementation.

template<class _Impl>
inline TokenIterator<_Impl> &TokenIterator<_Impl>::operator ++() {
    this->impl.move_next();
    return *this;
}

template<class _Impl>
inline TokenIterator<_Impl> TokenIterator<_Impl>::operator ++(int) {
    TokenIterator<_Impl> copy;
    this->impl.move_next();
    return copy;
}

template<class _Impl>
inline TokenIterator<_Impl> &TokenIterator<_Impl>::operator --() {
    this->impl.move_prev();
    return *this;
}

template<class _Impl>
inline TokenIterator<_Impl> TokenIterator<_Impl>::operator --(int) {
    TokenIterator<_Impl> copy;
    this->impl.move_prev();
    return copy;
}

template<class _Impl>
inline bool TokenIterator<_Impl>::operator ==(const TokenIterator<_Impl> &_other) const {
    return this->impl.equals(_other.impl);
}

template<class _Impl>
inline bool TokenIterator<_Impl>::operator !=(const TokenIterator<_Impl> &_other) const {
    return !this->impl.equals(_other.impl);
}

template<class _Impl>
inline const Token &TokenIterator<_Impl>::operator *() const {
    return this->impl.get();
}

template<class _Impl>
inline const Token *TokenIterator<_Impl>::operator ->() const {
    return &this->impl.get();
}

}

#endif /* LEXER_H_ */
