/*
 * formats.h
 *
 *  Created on: May 5, 2013
 *      Author: nikita.karnauhov@gmail.com
 */

#ifndef FORMATS_H_
#define FORMATS_H_

#include <string>
#include <map>

#include <stdio.h>
#include <stdlib.h>
#include <alsa/input.h>
#include <alsa/output.h>
#include <alsa/global.h>
#include <alsa/conf.h>
#include <alsa/pcm.h>

typedef std::map<std::string, snd_pcm_format_t> Formats;
extern Formats g_formats;

std::string get_format_name(snd_pcm_format_t _format);
snd_pcm_format_t get_format(const std::string &_name);

#endif /* FORMATS_H_ */
