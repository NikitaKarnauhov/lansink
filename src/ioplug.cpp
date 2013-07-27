/*
 * ioplug.cpp
 *
 *  Created on: Dec 15, 2012
 *      Author: nikita.karnauhov@gmail.com
 */

#include "unap.h"
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

namespace callbacks {

extern "C" {

int unap_start(snd_pcm_ioplug_t *_pPlug) {
    UNAP *pPlug = (UNAP *)_pPlug->private_data;

    try {
        pPlug->start();
        pPlug->log.debug("%s() = %d", __FUNCTION__, 0);
    } catch (std::exception &e) {
        pPlug->log.error(e.what());
    }

    return 0;
}

int unap_stop(snd_pcm_ioplug_t *_pPlug) {
    UNAP *pPlug = (UNAP *)_pPlug->private_data;

    try {
        pPlug->stop();
        pPlug->log.debug("%s() = %d", __FUNCTION__, 0);
    } catch (std::exception &e) {
        pPlug->log.error(e.what());
    }

    return 0;
}

snd_pcm_sframes_t unap_pointer(snd_pcm_ioplug_t *_pPlug) {
    UNAP *pPlug = (UNAP *)_pPlug->private_data;
    snd_pcm_sframes_t nResult = 0;

    try {
        nResult = pPlug->get_buffer_pointer();
        pPlug->log.debug("%s() = %d", __FUNCTION__, nResult);
    } catch (std::exception &e) {
        pPlug->log.error(e.what());
    }

    return nResult;
}

snd_pcm_sframes_t unap_transfer(snd_pcm_ioplug_t *_pPlug, const snd_pcm_channel_area_t *_pAreas,
        snd_pcm_uframes_t _cOffset, snd_pcm_uframes_t _cSize)
{
    UNAP *pPlug = (UNAP *)_pPlug->private_data;
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

int unap_close(snd_pcm_ioplug_t *_pPlug) {
    UNAP *pPlug = (UNAP *)_pPlug->private_data;

    try {
        pPlug->stop();
        pPlug->log.debug("%s() = %d", __FUNCTION__, 0);
        delete pPlug;
        _pPlug->private_data = NULL;
    } catch (std::exception &e) {
        pPlug->log.error(e.what());
    }

    return 0;
}

int unap_prepare(snd_pcm_ioplug_t *_pPlug) {
    UNAP *pPlug = (UNAP *)_pPlug->private_data;

    try {
        pPlug->prepare();
        pPlug->log.debug("%s() = %d", __FUNCTION__, 0);
    } catch (std::exception &e) {
        pPlug->log.error(e.what());
    }

    return 0;
}

int unap_drain(snd_pcm_ioplug_t *_pPlug) {
    UNAP *pPlug = (UNAP *)_pPlug->private_data;

    try {
        pPlug->drain();
        pPlug->log.debug("%s() = %d", __FUNCTION__, 0);
    } catch (std::exception &e) {
        pPlug->log.error(e.what());
    }

    return 0;
}

int unap_pause(snd_pcm_ioplug_t *_pPlug, int _bEnable) {
    UNAP *pPlug = (UNAP *)_pPlug->private_data;

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

int unap_poll_revents(snd_pcm_ioplug_t *_pPlug, struct pollfd *_pFD, unsigned int _cFDs,
        unsigned short *_pREvents)
{
    UNAP *pPlug = (UNAP *)_pPlug->private_data;

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

int unap_delay(snd_pcm_ioplug_t *_pPlug, snd_pcm_sframes_t *_pnDelay) {
    UNAP *pPlug = (UNAP *)_pPlug->private_data;

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
        s_callbacks.start = callbacks::unap_start;
        s_callbacks.stop = callbacks::unap_stop;
        s_callbacks.pointer = callbacks::unap_pointer;
        s_callbacks.transfer = callbacks::unap_transfer;
        s_callbacks.close = callbacks::unap_close;
        s_callbacks.hw_params = nullptr;
        s_callbacks.hw_free = nullptr;
        s_callbacks.sw_params = nullptr;
        s_callbacks.prepare = callbacks::unap_prepare;
        s_callbacks.drain = callbacks::unap_drain;
        s_callbacks.pause = callbacks::unap_pause;
        s_callbacks.resume = nullptr;
        s_callbacks.poll_descriptors_count = nullptr;
        s_callbacks.poll_descriptors = nullptr;
        s_callbacks.poll_revents = callbacks::unap_poll_revents;
        s_callbacks.dump = nullptr;
        s_callbacks.delay = callbacks::unap_delay;
        s_bInitialized = true;
    }

    return s_callbacks;
}
}
}

#define ARRAY_SIZE(ary) (sizeof(ary)/sizeof(ary[0]))

static const size_t g_cPeriods = 8;

static
void _set_hw_constraint(struct UNAP *_pPlug)
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
SND_PCM_PLUGIN_DEFINE_FUNC(unap) {
    snd_config_iterator_t i, next;
    UNAP *pPlug = new(calloc(1, sizeof(UNAP))) UNAP;

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

            if (strcmp(strField, "log") == 0) {
                snd_config_get_string(pEntry, &strValue);
                pPlug->log.open(strValue);
                pPlug->log.setLevel(llDebug);
                continue;
            }

            if (strcmp(strField, "log_level") == 0) {
                long int nLevel = 0;

                snd_config_get_integer(pEntry, &nLevel);

                if (nLevel < llSilent || nLevel >= llDebug)
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

                pPlug->channelValues.emplace_back();
                snd_config_get_integer(pEntry, (long int *)&pPlug->channelValues.back());
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

                pPlug->rateValues.emplace_back();
                snd_config_get_integer(pEntry, (long int *)&pPlug->rateValues.back());
            }
        } else
            pPlug->rateValues = {44100, 48000};

        pPlug->connect();
        pPlug->version = SND_PCM_IOPLUG_VERSION;
        pPlug->name = "UNAP Protocol";
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
    SND_PCM_PLUGIN_SYMBOL(unap);
}
