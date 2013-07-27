/*
 * formats.cpp
 *
 *  Created on: May 5, 2013
 *      Author: nikita.karnauhov@gmail.com
 */

#include "formats.h"
#include "alsa.h"

Formats g_formats{
    // Signed 8 bit.
    {"S8", SND_PCM_FORMAT_S8},
    // Unsigned 8 bit.
    {"U8", SND_PCM_FORMAT_U8},
    // Signed 16 bit Little Endian.
    {"S16_LE", SND_PCM_FORMAT_S16_LE},
    // Signed 16 bit Big Endian.
    {"S16_BE", SND_PCM_FORMAT_S16_BE},
    // Unsigned 16 bit Little Endian.
    {"U16_LE", SND_PCM_FORMAT_U16_LE},
    // Unsigned 16 bit Big Endian.
    {"U16_BE", SND_PCM_FORMAT_U16_BE},
    // Signed 24 bit Little Endian using low three bytes in 32-bit word.
    {"S24_LE", SND_PCM_FORMAT_S24_LE},
    // Signed 24 bit Big Endian using low three bytes in 32-bit word.
    {"S24_BE", SND_PCM_FORMAT_S24_BE},
    // Unsigned 24 bit Little Endian using low three bytes in 32-bit word.
    {"U24_LE", SND_PCM_FORMAT_U24_LE},
    // Unsigned 24 bit Big Endian using low three bytes in 32-bit word.
    {"U24_BE", SND_PCM_FORMAT_U24_BE},
    // Signed 32 bit Little Endian.
    {"S32_LE", SND_PCM_FORMAT_S32_LE},
    // Signed 32 bit Big Endian.
    {"S32_BE", SND_PCM_FORMAT_S32_BE},
    // Unsigned 32 bit Little Endian.
    {"U32_LE", SND_PCM_FORMAT_U32_LE},
    // Unsigned 32 bit Big Endian.
    {"U32_BE", SND_PCM_FORMAT_U32_BE},
    // Float 32 bit Little Endian, Range -1.0 to 1.0.
    {"FLOAT_LE", SND_PCM_FORMAT_FLOAT_LE},
    // Float 32 bit Big Endian, Range -1.0 to 1.0.
    {"FLOAT_BE", SND_PCM_FORMAT_FLOAT_BE},
    // Float 64 bit Little Endian, Range -1.0 to 1.0.
    {"FLOAT64_LE", SND_PCM_FORMAT_FLOAT64_LE},
    // Float 64 bit Big Endian, Range -1.0 to 1.0.
    {"FLOAT64_BE", SND_PCM_FORMAT_FLOAT64_BE},
    // IEC-958 Little Endian.
    {"IEC958_SUBFRAME_LE", SND_PCM_FORMAT_IEC958_SUBFRAME_LE},
    // IEC-958 Big Endian.
    {"IEC958_SUBFRAME_BE", SND_PCM_FORMAT_IEC958_SUBFRAME_BE},
    // Mu-Law.
    {"MU_LAW", SND_PCM_FORMAT_MU_LAW},
    // A-Law.
    {"A_LAW", SND_PCM_FORMAT_A_LAW},
    // Ima-ADPCM.
    {"IMA_ADPCM", SND_PCM_FORMAT_IMA_ADPCM},
    // MPEG.
    {"MPEG", SND_PCM_FORMAT_MPEG},
    // GSM.
    {"GSM", SND_PCM_FORMAT_GSM},
    // Special.
    {"SPECIAL", SND_PCM_FORMAT_SPECIAL},
    // Signed 24bit Little Endian in 3bytes format.
    {"S24_3LE", SND_PCM_FORMAT_S24_3LE},
    // Signed 24bit Big Endian in 3bytes format.
    {"S24_3BE", SND_PCM_FORMAT_S24_3BE},
    // Unsigned 24bit Little Endian in 3bytes format.
    {"U24_3LE", SND_PCM_FORMAT_U24_3LE},
    // Unsigned 24bit Big Endian in 3bytes format.
    {"U24_3BE", SND_PCM_FORMAT_U24_3BE},
    // Signed 20bit Little Endian in 3bytes format.
    {"S20_3LE", SND_PCM_FORMAT_S20_3LE},
    // Signed 20bit Big Endian in 3bytes format.
    {"S20_3BE", SND_PCM_FORMAT_S20_3BE},
    // Unsigned 20bit Little Endian in 3bytes format.
    {"U20_3LE", SND_PCM_FORMAT_U20_3LE},
    // Unsigned 20bit Big Endian in 3bytes format.
    {"U20_3BE", SND_PCM_FORMAT_U20_3BE},
    // Signed 18bit Little Endian in 3bytes format.
    {"S18_3LE", SND_PCM_FORMAT_S18_3LE},
    // Signed 18bit Big Endian in 3bytes format.
    {"S18_3BE", SND_PCM_FORMAT_S18_3BE},
    // Unsigned 18bit Little Endian in 3bytes format.
    {"U18_3LE", SND_PCM_FORMAT_U18_3LE},
    // Unsigned 18bit Big Endian in 3bytes format.
    {"U18_3BE", SND_PCM_FORMAT_U18_3BE},
};

std::string get_format_name(snd_pcm_format_t _format) {
    for (const auto &format : g_formats)
        if (format.second == _format)
            return format.first;

    return "";
}

snd_pcm_format_t get_format(const std::string &_name) {
    auto iFormat = g_formats.find(_name);

    if (iFormat == g_formats.end())
        throw ALSA::Error(-1, "Uknown format: %s", _name.c_str());

    return iFormat->second;
}
