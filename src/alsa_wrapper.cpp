/*
 * alsa_wrapper.cpp
 *
 *  Created on: May 4, 2013
 *      Author: nikita.karnauhov@gmail.com
 */

#include "alsa_wrapper.h"

#include <stdarg.h>

int ALSA::open(snd_pcm_t **_ppPcm, const char *_strName, snd_pcm_stream_t _stream, int _nMode) {
    const int nResult = snd_pcm_open(_ppPcm, _strName, _stream, _nMode);

    if (nResult < 0)
        throw Error(nResult, "Cannot open audio device %s", _strName);

    return nResult;
}

int ALSA::hw_params_malloc(snd_pcm_hw_params_t **_ppParams) {
    const int nResult = snd_pcm_hw_params_malloc(_ppParams);

    if (nResult < 0)
        throw Error(nResult, "Cannot allocate hardware parameter structure");

    return nResult;
}

int ALSA::hw_params_any(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams) {
    const int nResult = snd_pcm_hw_params_any(_pPcm, _pParams);

    if (nResult < 0)
        throw Error(nResult, "Cannot initialize hardware parameter structure");

    return nResult;
}

int ALSA::hw_params_set_access(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams,
        snd_pcm_access_t _access)
{
    const int nResult = snd_pcm_hw_params_set_access(_pPcm, _pParams, _access);

    if (nResult < 0)
        throw Error(nResult, "Cannot set access type");

    return nResult;
}

int ALSA::hw_params_set_format(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams,
        snd_pcm_format_t _format)
{
    const int nResult = snd_pcm_hw_params_set_format(_pPcm, _pParams, _format);

    if (nResult < 0)
        throw Error(nResult, "Cannot set sample format");

    return nResult;
}

int ALSA::hw_params_set_rate_near(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams,
        unsigned int *_puRate, int *_pnDir)
{
    const int nResult = snd_pcm_hw_params_set_rate_near(_pPcm, _pParams, _puRate, _pnDir);

    if (nResult < 0)
        throw Error(nResult, "Cannot set sample rate");

    return nResult;
}

int ALSA::hw_params_set_channels(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams,
        unsigned int _uChannels)
{
    const int nResult = snd_pcm_hw_params_set_channels(_pPcm, _pParams, _uChannels);

    if (nResult < 0)
        throw Error(nResult, "Cannot set channel count");

    return nResult;
}

int ALSA::hw_params_set_buffer_size_near(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams,
        snd_pcm_uframes_t *_puSize)
{
    const int nResult = snd_pcm_hw_params_set_buffer_size_near(_pPcm, _pParams, _puSize);

    if (nResult < 0)
        throw Error(nResult, "Cannot set buffer size");

    return nResult;
}

int ALSA::hw_params_set_period_size_near(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams,
        snd_pcm_uframes_t *_puSize, int *_pnDir)
{
    const int nResult = snd_pcm_hw_params_set_period_size_near(_pPcm, _pParams, _puSize, _pnDir);

    if (nResult < 0)
        throw Error(nResult, "Cannot set period size");

    return nResult;
}

int ALSA::hw_params(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams) {
    const int nResult = snd_pcm_hw_params(_pPcm, _pParams);

    if (nResult < 0)
        throw Error(nResult, "Cannot set hardware parameters");

    return nResult;
}

void ALSA::hw_params_free(snd_pcm_hw_params_t *_pParams) {
    snd_pcm_hw_params_free(_pParams);
}

int ALSA::prepare(snd_pcm_t *_pPcm) {
    const int nResult = snd_pcm_prepare(_pPcm);

    if (nResult < 0)
        throw Error(nResult, "Cannot prepare audio interface for use");

    return nResult;
}

int ALSA::drain(snd_pcm_t *_pPcm) {
    const int nResult = snd_pcm_drain(_pPcm);

    if (nResult < 0)
        throw Error(nResult, "Cannot drain audio device");

    return nResult;
}

int ALSA::close(snd_pcm_t *_pPcm) {
    const int nResult = snd_pcm_close(_pPcm);

    if (nResult < 0)
        throw Error(nResult, "Cannot close audio device");

    return nResult;
}

snd_pcm_sframes_t ALSA::writei(snd_pcm_t *_pPcm, const void *_pBuffer,
        snd_pcm_uframes_t _cSize)
{
    const int nResult = snd_pcm_writei(_pPcm, _pBuffer, _cSize);

    if (nResult < 0)
        throw Error(nResult, "Failed writing to audio device");

    return nResult;
}

int ALSA::recover(snd_pcm_t *_pPcm, int _nError, bool _bSilent) {
    const int nResult = snd_pcm_recover(_pPcm, _nError, _bSilent);

    if (nResult < 0)
        throw Error(nResult, "Cannot recover audio device");

    return nResult;
}

int ALSA::wait(snd_pcm_t *_pPcm, int _nTimeout) {
    const int nResult = snd_pcm_wait(_pPcm, _nTimeout);

    if (nResult < 0)
        throw Error(nResult, "Failed polling audio device");

    return nResult;
}

snd_pcm_sframes_t ALSA::avail_update(snd_pcm_t *_pPcm) {
    const int nResult = snd_pcm_avail_update(_pPcm);

    if (nResult < 0)
        throw Error(nResult, "Failed querying available buffer");

    return nResult;
}

int ALSA::pause(snd_pcm_t *_pPcm, bool _bEnable) {
    const int nResult = snd_pcm_pause(_pPcm, _bEnable);

    if (nResult < 0)
        throw Error(nResult, "Cannot %s audio device", _bEnable ? "pause" : "unpause");

    return nResult;
}

unsigned int ALSA::format_physical_width(snd_pcm_format_t _format) {
    const int nResult = snd_pcm_format_physical_width(_format);

    if (nResult < 0)
        throw Error(nResult, "Unknown format %d", _format);

    return nResult;
}
