/*
 * unap.c
 *
 *  Created on: Dec 15, 2012
 *      Author: nikita.karnauhov@gmail.com
 */

#include "unap.h"
#include "formats.h"

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

#define DEBUG() \
    do { \
        std::cerr << __FILE__ << ":" << __LINE__ << ": " << __FUNCTION__ << std::endl << std::flush; \
    } while (0)

namespace callbacks {

extern "C" {

int unap_start(snd_pcm_ioplug_t *_pPlug) {
    UNAP *pPlug = (UNAP *)_pPlug->private_data;
    pPlug->start();
    return 0;
}

int unap_stop(snd_pcm_ioplug_t *_pPlug) {
    UNAP *pPlug = (UNAP *)_pPlug->private_data;
    pPlug->stop();
    return 0;
}

snd_pcm_sframes_t unap_pointer(snd_pcm_ioplug_t *_pPlug) {
    UNAP *pPlug = (UNAP *)_pPlug->private_data;
    return pPlug->get_buffer_pointer();
}

snd_pcm_sframes_t unap_transfer(snd_pcm_ioplug_t *_pPlug, const snd_pcm_channel_area_t *_pAreas,
        snd_pcm_uframes_t _cOffset, snd_pcm_uframes_t _cSize)
{
    UNAP *pPlug = (UNAP *)_pPlug->private_data;
//    std::cerr << "avail = " << snd_pcm_avail_update(io->pcm) << std::endl;
    const size_t cFrames = pPlug->transfer((const char *)_pAreas->addr, _cOffset, _cSize);

    // Force start playing if buffer is full.
    if (cFrames == 0 && _pPlug->state == SND_PCM_STATE_PREPARED)
        if (int nError = snd_pcm_start(_pPlug->pcm))
            return nError;

    return cFrames;
}

int unap_close(snd_pcm_ioplug_t *_pPlug) {
    UNAP *pPlug = (UNAP *)_pPlug->private_data;
    pPlug->stop();
    delete pPlug;
    _pPlug->private_data = NULL;
    return 0;
}

int unap_prepare(snd_pcm_ioplug_t *_pPlug) {
    UNAP *pPlug = (UNAP *)_pPlug->private_data;
    pPlug->prepare();
    return 0;
}

int unap_drain(snd_pcm_ioplug_t *_pPlug) {
    UNAP *pPlug = (UNAP *)_pPlug->private_data;
    pPlug->drain();
    return 0;
}

int unap_pause(snd_pcm_ioplug_t *_pPlug, int _bEnable) {
    UNAP *pPlug = (UNAP *)_pPlug->private_data;

    if (_bEnable)
        pPlug->pause();
    else
        pPlug->unpause();

    return 0;
}

int unap_poll_revents(snd_pcm_ioplug_t *_pPlug, struct pollfd *_pFD, unsigned int _cFDs,
        unsigned short *_pREvents)
{
    static char buf[1];

    assert(_pFD && _cFDs == 1 && _pREvents);
    read(_pFD[0].fd, buf, 1);
    *_pREvents = _pFD[0].revents & ~(POLLIN | POLLOUT);

    if (_pFD[0].revents & POLLIN)
        *_pREvents |= POLLOUT;

    return 0;
}

int unap_delay(snd_pcm_ioplug_t *_pPlug, snd_pcm_sframes_t *_pnDelay) {
    UNAP *pPlug = (UNAP *)_pPlug->private_data;
    *_pnDelay = pPlug->get_unplayed_frames();
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
        s_callbacks.hw_params = nullptr; //callbacks::unapHWParams;
        s_callbacks.hw_free = nullptr; //callbacks::unapHWFree;
        s_callbacks.sw_params = nullptr; //callbacks::unapSWParams;
        s_callbacks.prepare = callbacks::unap_prepare;
        s_callbacks.drain = callbacks::unap_drain;
        s_callbacks.pause = callbacks::unap_pause;
        s_callbacks.resume = nullptr; //callbacks::unapResume;
        s_callbacks.poll_descriptors_count = nullptr; //poll_descriptors_count;
        s_callbacks.poll_descriptors = nullptr; //poll_descriptors;
        s_callbacks.poll_revents = callbacks::unap_poll_revents;
        s_callbacks.dump = nullptr; //callbacks::unapDump;
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
int _set_hw_constraint(struct UNAP *_pPlug)
{
    unsigned int accesses[] = {
        SND_PCM_ACCESS_MMAP_INTERLEAVED,
        SND_PCM_ACCESS_MMAP_NONINTERLEAVED,
        SND_PCM_ACCESS_RW_INTERLEAVED,
        SND_PCM_ACCESS_RW_NONINTERLEAVED
    };

    int err;

    if ((err = snd_pcm_ioplug_set_param_list(_pPlug, SND_PCM_IOPLUG_HW_ACCESS,
            ARRAY_SIZE(accesses), accesses)) < 0 ||
        (err = snd_pcm_ioplug_set_param_list(_pPlug, SND_PCM_IOPLUG_HW_FORMAT,
                _pPlug->get_format_values().size(), _pPlug->get_format_values().data())) < 0 ||
        (err = snd_pcm_ioplug_set_param_list(_pPlug, SND_PCM_IOPLUG_HW_CHANNELS,
                _pPlug->channelValues.size(), _pPlug->channelValues.data())) < 0 ||
        (err = snd_pcm_ioplug_set_param_list(_pPlug, SND_PCM_IOPLUG_HW_RATE,
                _pPlug->rateValues.size(), _pPlug->rateValues.data())) < 0 ||
        (err = snd_pcm_ioplug_set_param_minmax(_pPlug, SND_PCM_IOPLUG_HW_PERIODS,
                g_cPeriods, g_cPeriods)) < 0)
    {
        return err;
    }

    // TODO A52

    return 0;
}

extern "C"
SND_PCM_PLUGIN_DEFINE_FUNC(unap) {
    snd_config_iterator_t i, next;
//    UNAPPlug *pPlug = new UNAPPlug;
    UNAP *pPlug = new(calloc(1, sizeof(UNAP))) UNAP;

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
            pPlug->rateValues.emplace_back();
            snd_config_get_integer(pEntry, (long int *)&pPlug->rateValues.back());
            continue;
        }

        if (strcmp(strField, "channels") == 0) {
            pPlug->channelValues.emplace_back();
            snd_config_get_integer(pEntry, (long int *)&pPlug->channelValues.back());
            continue;
        }

        if (strcmp(strField, "format") == 0) {
            snd_config_get_string(pEntry, &strValue);
            auto iFormat = g_formats.find(strValue);
            if (iFormat == g_formats.end())
                SNDERR("Unknown format %s", strValue);
            else
                pPlug->formats.push_back(iFormat->first);
            continue;
        }

        if (strcmp(strField, "mtu") == 0) {
            snd_config_get_integer(pEntry, &pPlug->nMTU);
            continue;
        }

        SNDERR("Unknown field %s", strField);
        return -EINVAL;
    }

    pPlug->connect();
    pPlug->version = SND_PCM_IOPLUG_VERSION;
    pPlug->name = "UNAP Protocol";
    pPlug->mmap_rw = 0;

    // Sensible defaults for sample format, rate and channel count.
    if (pPlug->get_format_values().empty())
        pPlug->formats = {"U8", "S16_LE", "S16_BE", "S32_LE", "S32_BE", "FLOAT_LE", "FLOAT_BE",
                "MU_LAW", "A_LAW"};

    if (pPlug->channelValues.empty())
        pPlug->channelValues = {1, 2};

    if (pPlug->rateValues.empty())
        pPlug->rateValues = {44100, 48000};

    pPlug->callback = &callbacks::get();
    pPlug->private_data = pPlug;

    const int nError = snd_pcm_ioplug_create(pPlug, name, stream, mode);

    if (nError < 0) {
        delete pPlug;
        fprintf(stderr, "Error %d\n", nError);
        return nError;
    }

    _set_hw_constraint(pPlug);

    *pcmp = pPlug->pcm;

    fprintf(stderr, "Initialized!\n");

    return 0;
}

extern "C" {
    SND_PCM_PLUGIN_SYMBOL(unap);
}
