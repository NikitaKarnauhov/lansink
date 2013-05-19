/*
 * utils.cpp
 *
 *  Created on: May 19, 2013
 *      Author: nikita.karnauhov@gmail.com
 */

#include <stdarg.h>

#include <memory>
#include <string>

std::string format(size_t _cMaxLength, const char *_strFormat, ...) {
    std::unique_ptr<char []> str(new char[_cMaxLength]);
    va_list args;

    va_start(args, _strFormat);
    str[_cMaxLength - 1] = 0;
    vsnprintf(str.get(), _cMaxLength - 1, _strFormat, args);
    va_end (args);

    return str.get();
}
