/*
    ioplug.cpp

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

#include "sender.h"
#include "formats.h"
#include "exception.h"
#include "alsa.h"

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <assert.h>
#include <poll.h>
#include <unistd.h>

#include <alsa/error.h>

#include <map>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <array>

namespace callbacks {

extern "C" {

int lansink_start(snd_pcm_ioplug_t *_pPlug) {
    Sender *pPlug = (Sender *)_pPlug->private_data;

    try {
        pPlug->start();
        pPlug->log.debug("%s() = %d", __FUNCTION__, 0);
    } catch (std::exception &e) {
        pPlug->log.error(e.what());
    }

    return 0;
}

int lansink_stop(snd_pcm_ioplug_t *_pPlug) {
    Sender *pPlug = (Sender *)_pPlug->private_data;

    try {
        pPlug->stop();
        pPlug->log.debug("%s() = %d", __FUNCTION__, 0);
    } catch (std::exception &e) {
        pPlug->log.error(e.what());
    }

    return 0;
}

snd_pcm_sframes_t lansink_pointer(snd_pcm_ioplug_t *_pPlug) {
    Sender *pPlug = (Sender *)_pPlug->private_data;
    snd_pcm_sframes_t nResult = 0;

    try {
        nResult = pPlug->get_buffer_pointer();
        pPlug->log.debug("%s() = %d", __FUNCTION__, nResult);
    } catch (std::exception &e) {
        pPlug->log.error(e.what());
    }

    return nResult;
}

snd_pcm_sframes_t lansink_transfer(snd_pcm_ioplug_t *_pPlug, const snd_pcm_channel_area_t *_pAreas,
        snd_pcm_uframes_t _cOffset, snd_pcm_uframes_t _cSize)
{
    Sender *pPlug = (Sender *)_pPlug->private_data;
    size_t cFrames = 0;

    try {
        cFrames = pPlug->transfer((const char *)_pAreas->addr, _cOffset, _cSize);

        // Force start playing if buffer is full.
        if (cFrames == 0 && _pPlug->state == SND_PCM_STATE_PREPARED) {
            pPlug->log.debug("Buffer is full, starting playback.");
            ALSA::start(_pPlug->pcm);
        }

        pPlug->log.debug("%s() = %d, _cOffset = %d, _cSize = %d",
                __FUNCTION__, cFrames, _cOffset, _cSize);
    } catch (std::exception &e) {
        pPlug->log.error(e.what());
    }

    return cFrames;
}

int lansink_close(snd_pcm_ioplug_t *_pPlug) {
    Sender *pPlug = (Sender *)_pPlug->private_data;

    try {
        pPlug->stop();
        pPlug->log.debug("%s() = %d", __FUNCTION__, 0);
        delete pPlug;
        _pPlug->private_data = NULL;
        free(pPlug);
    } catch (std::exception &e) {
        pPlug->log.error(e.what());
    }

    return 0;
}

int lansink_prepare(snd_pcm_ioplug_t *_pPlug) {
    Sender *pPlug = (Sender *)_pPlug->private_data;

    try {
        pPlug->prepare();
        pPlug->log.debug("%s() = %d", __FUNCTION__, 0);
    } catch (std::exception &e) {
        pPlug->log.error(e.what());
    }

    return 0;
}

int lansink_drain(snd_pcm_ioplug_t *_pPlug) {
    Sender *pPlug = (Sender *)_pPlug->private_data;

    try {
        pPlug->drain();
        pPlug->log.debug("%s() = %d", __FUNCTION__, 0);
    } catch (std::exception &e) {
        pPlug->log.error(e.what());
    }

    return 0;
}

int lansink_pause(snd_pcm_ioplug_t *_pPlug, int _bEnable) {
    Sender *pPlug = (Sender *)_pPlug->private_data;

    try {
        if (_bEnable)
            pPlug->pause();
        else
            pPlug->unpause();

        pPlug->log.debug("%s() = %d, _bEnable = %d", __FUNCTION__, 0, _bEnable);
    } catch (std::exception &e) {
        pPlug->log.error(e.what());
    }

    return 0;
}

int lansink_poll_revents(snd_pcm_ioplug_t *_pPlug, struct pollfd *_pFD, unsigned int _cFDs,
        unsigned short *_pREvents)
{
    Sender *pPlug = (Sender *)_pPlug->private_data;

    try {
        static char buf[1];

        assert(_pFD && _cFDs == 1 && _pREvents);
        *_pREvents = _pFD[0].revents & ~(POLLIN | POLLOUT);

        if (_pFD[0].revents & POLLIN) {
            read(_pFD[0].fd, buf, 1);
            *_pREvents |= POLLOUT;
            pPlug->log.debug("get_delay() = %d, get_buffer_size() = %d",
                    pPlug->get_delay(), pPlug->get_buffer_size());
        }

        pPlug->log.debug("%s() = %d, _pFD[0].revents = %d, *_pREvents = %d",
                __FUNCTION__, 0, _pFD[0].revents, *_pREvents);
    } catch (std::exception &e) {
        pPlug->log.error(e.what());
    }

    return 0;
}

int lansink_delay(snd_pcm_ioplug_t *_pPlug, snd_pcm_sframes_t *_pnDelay) {
    Sender *pPlug = (Sender *)_pPlug->private_data;

    try {
        *_pnDelay = pPlug->get_delay();
        pPlug->log.debug("%s() = %d; *_pnDelay = %d", __FUNCTION__, 0, *_pnDelay);
    } catch (std::exception &e) {
        pPlug->log.error(e.what());
    }

    return 0;
}

snd_pcm_ioplug_callback_t &get() {
    static snd_pcm_ioplug_callback_t s_callbacks = {0};
    static bool s_bInitialized = false;

    if (!s_bInitialized) {
        s_callbacks.start = callbacks::lansink_start;
        s_callbacks.stop = callbacks::lansink_stop;
        s_callbacks.pointer = callbacks::lansink_pointer;
        s_callbacks.transfer = callbacks::lansink_transfer;
        s_callbacks.close = callbacks::lansink_close;
        s_callbacks.hw_params = nullptr;
        s_callbacks.hw_free = nullptr;
        s_callbacks.sw_params = nullptr;
        s_callbacks.prepare = callbacks::lansink_prepare;
        s_callbacks.drain = callbacks::lansink_drain;
        s_callbacks.pause = callbacks::lansink_pause;
        s_callbacks.resume = nullptr;
        s_callbacks.poll_descriptors_count = nullptr;
        s_callbacks.poll_descriptors = nullptr;
        s_callbacks.poll_revents = callbacks::lansink_poll_revents;
        s_callbacks.dump = nullptr;
        s_callbacks.delay = callbacks::lansink_delay;
        s_bInitialized = true;
    }

    return s_callbacks;
}
}
}

#define ARRAY_SIZE(ary) (sizeof(ary)/sizeof(ary[0]))

static const size_t g_cPeriods = 8;

static
void _set_hw_constraint(struct Sender *_pPlug)
{
    std::array<unsigned int, 4> accesses{
        SND_PCM_ACCESS_MMAP_INTERLEAVED,
        SND_PCM_ACCESS_MMAP_NONINTERLEAVED,
        SND_PCM_ACCESS_RW_INTERLEAVED,
        SND_PCM_ACCESS_RW_NONINTERLEAVED
    };

    ALSA::ioplug_set_param_list(_pPlug, SND_PCM_IOPLUG_HW_ACCESS,
                accesses.size(), accesses.data());
    ALSA::ioplug_set_param_list(_pPlug, SND_PCM_IOPLUG_HW_FORMAT,
            _pPlug->get_format_values().size(), _pPlug->get_format_values().data());
    ALSA::ioplug_set_param_list(_pPlug, SND_PCM_IOPLUG_HW_CHANNELS,
                    _pPlug->channelValues.size(), _pPlug->channelValues.data());
    ALSA::ioplug_set_param_list(_pPlug, SND_PCM_IOPLUG_HW_RATE,
            _pPlug->rateValues.size(), _pPlug->rateValues.data());
    ALSA::ioplug_set_param_minmax(_pPlug, SND_PCM_IOPLUG_HW_PERIODS,
                    g_cPeriods, g_cPeriods);
}

extern "C"
SND_PCM_PLUGIN_DEFINE_FUNC(lansink) {
    snd_config_iterator_t i, next;
    Sender *pPlug = new(calloc(1, sizeof(Sender))) Sender;

    snd_config_t *pRates = nullptr;
    snd_config_t *pChannels = nullptr;
    snd_config_t *pFormats = nullptr;

    try {
        snd_config_for_each(i, next, conf) {
            snd_config_t *pEntry = snd_config_iterator_entry(i);
            const char *strField;
            const char *strValue;

            if (snd_config_get_id(pEntry, &strField) < 0)
                continue;

            if (strcmp(strField, "comment") == 0 || strcmp(strField, "type") == 0)
                continue;

            if (strcmp(strField, "host") == 0) {
                snd_config_get_string(pEntry, &strValue);
                pPlug->strHost = strValue;
                continue;
            }

            if (strcmp(strField, "port") == 0) {
                snd_config_get_integer(pEntry, &pPlug->nPort);
                continue;
            }

            if (strcmp(strField, "rate") == 0) {
                pRates = pEntry;
                continue;
            }

            if (strcmp(strField, "channels") == 0) {
                pChannels = pEntry;
                continue;
            }

            if (strcmp(strField, "format") == 0) {
                pFormats = pEntry;
                continue;
            }

            if (strcmp(strField, "mtu") == 0) {
                snd_config_get_integer(pEntry, &pPlug->nMTU);
                continue;
            }

            if (strcmp(strField, "send_period") == 0) {
                snd_config_get_integer(pEntry, &pPlug->nSendPeriod);
                continue;
            }

            if (strcmp(strField, "log") == 0) {
                snd_config_get_string(pEntry, &strValue);
                pPlug->log.open(strValue);
                pPlug->log.setLevel(llDebug);
                continue;
            }

            if (strcmp(strField, "log_level") == 0) {
                long int nLevel = 0;

                snd_config_get_integer(pEntry, &nLevel);

                if (nLevel < llSilent || nLevel > llDebug)
                    throw RuntimeError("Invalid log level %d (must be between %d and %d)",
                            nLevel, llSilent, llDebug);

                pPlug->log.setLevel(LogLevel(nLevel));
                continue;
            }

            throw RuntimeError("Unknown field %s", strField);
        }

        if (pFormats) {
            if (snd_config_get_type(pFormats) != SND_CONFIG_TYPE_COMPOUND)
                throw RuntimeError("Formats definition must be a compound");

            snd_config_for_each(i, next, pFormats) {
                snd_config_t *pEntry = snd_config_iterator_entry(i);
                const char *strField;
                const char *strValue;

                if (snd_config_get_id(pEntry, &strField) < 0)
                    continue;

                snd_config_get_string(pEntry, &strValue);
                auto iFormat = g_formats.find(strValue);

                if (iFormat == g_formats.end())
                    throw RuntimeError("Unknown format %s", strValue);

                pPlug->formats.push_back(iFormat->first);
            }
        } else
            pPlug->formats = {"U8", "S16_LE", "S16_BE", "S32_LE", "S32_BE", "FLOAT_LE", "FLOAT_BE",
                    "MU_LAW", "A_LAW"};

        if (pChannels) {
            if (snd_config_get_type(pChannels) != SND_CONFIG_TYPE_COMPOUND)
                throw RuntimeError("Channels definition must be a compound");

            snd_config_for_each(i, next, pChannels) {
                snd_config_t *pEntry = snd_config_iterator_entry(i);
                const char *strField;

                if (snd_config_get_id(pEntry, &strField) < 0)
                    continue;

                long nChannels = 0;
                snd_config_get_integer(pEntry, &nChannels);
                pPlug->channelValues.push_back(nChannels);
            }
        } else
            pPlug->channelValues = {1, 2};

        if (pRates) {
            if (snd_config_get_type(pRates) != SND_CONFIG_TYPE_COMPOUND)
                throw RuntimeError("Rates definition must be a compound");

            snd_config_for_each(i, next, pRates) {
                snd_config_t *pEntry = snd_config_iterator_entry(i);
                const char *strField;

                if (snd_config_get_id(pRates, &strField) < 0)
                    continue;

                long nRate = 0;
                snd_config_get_integer(pEntry, &nRate);
                pPlug->rateValues.push_back(nRate);
            }
        } else
            pPlug->rateValues = {44100, 48000};

        pPlug->connect();
        pPlug->version = SND_PCM_IOPLUG_VERSION;
        pPlug->name = "LANSink Protocol";
        pPlug->mmap_rw = 0;
        pPlug->callback = &callbacks::get();
        pPlug->private_data = pPlug;
        ALSA::ioplug_create(pPlug, name, stream, mode);
        _set_hw_constraint(pPlug);
        *pcmp = pPlug->pcm;
    } catch (std::exception &e) {
        SNDERR("%s", e.what());
        pPlug->log.error(e.what());
        return -EINVAL;
    }

    pPlug->log.info("Initialized!");
    return 0;
}

extern "C" {
    SND_PCM_PLUGIN_SYMBOL(lansink);
}
