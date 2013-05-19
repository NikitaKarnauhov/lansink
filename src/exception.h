/*
 * exception.h
 *
 *  Created on: May 19, 2013
 *      Author: nikita.karnauhov@gmail.com
 */

#ifndef EXCEPTION_H_
#define EXCEPTION_H_

#include <string>
#include <stdexcept>

#include "utils.h"

template<typename _Base>
class Exception : public _Base {
public:
    template<typename... Args>
    Exception(const char *_strFormat, Args... _args) :
        _Base(format(1024, _strFormat, _args...))
    {
    }
};

using RuntimeError = Exception<std::runtime_error>;
using LogicError = Exception<std::runtime_error>;

#endif /* EXCEPTION_H_ */
