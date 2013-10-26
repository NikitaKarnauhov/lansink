/*
    alsa.cpp

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

#include "alsa.h"

#include <stdarg.h>

int alsa::open(snd_pcm_t **_ppPcm, const char *_strName, snd_pcm_stream_t _stream, int _nMode) {
    const int nResult = snd_pcm_open(_ppPcm, _strName, _stream, _nMode);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_open(name = %s, stream = %d, mode = %d)",
                _strName, _stream, _nMode);

    return nResult;
}

int alsa::hw_params_malloc(snd_pcm_hw_params_t **_ppParams) {
    const int nResult = snd_pcm_hw_params_malloc(_ppParams);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_hw_params_malloc()");

    return nResult;
}

int alsa::hw_params_any(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams) {
    const int nResult = snd_pcm_hw_params_any(_pPcm, _pParams);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_hw_params_any()");

    return nResult;
}

int alsa::hw_params_set_access(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams,
        snd_pcm_access_t _access)
{
    const int nResult = snd_pcm_hw_params_set_access(_pPcm, _pParams, _access);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_hw_params_set_access(_access = %d)", _access);

    return nResult;
}

int alsa::hw_params_set_format(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams,
        snd_pcm_format_t _format)
{
    const int nResult = snd_pcm_hw_params_set_format(_pPcm, _pParams, _format);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_hw_params_set_format(format = %d)", _format);

    return nResult;
}

int alsa::hw_params_set_rate_near(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams,
        unsigned int *_puRate, int *_pnDir)
{
    const int nResult = snd_pcm_hw_params_set_rate_near(_pPcm, _pParams, _puRate, _pnDir);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_hw_params_set_rate_near()");

    return nResult;
}

int alsa::hw_params_set_channels(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams,
        unsigned int _uChannels)
{
    const int nResult = snd_pcm_hw_params_set_channels(_pPcm, _pParams, _uChannels);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_hw_params_set_channels(val = %u)", _uChannels);

    return nResult;
}

int alsa::hw_params_set_buffer_size_near(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams,
        snd_pcm_uframes_t *_puSize)
{
    const int nResult = snd_pcm_hw_params_set_buffer_size_near(_pPcm, _pParams, _puSize);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_hw_params_set_buffer_size_near()");

    return nResult;
}

int alsa::hw_params_set_period_size_near(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams,
        snd_pcm_uframes_t *_puSize, int *_pnDir)
{
    const int nResult = snd_pcm_hw_params_set_period_size_near(_pPcm, _pParams, _puSize, _pnDir);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_hw_params_set_period_size_near()");

    return nResult;
}

bool alsa::hw_params_can_pause(snd_pcm_hw_params_t *_pParams) {
    return snd_pcm_hw_params_can_pause(_pParams) == 1;
}

int alsa::hw_params(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams) {
    const int nResult = snd_pcm_hw_params(_pPcm, _pParams);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_hw_params()");

    return nResult;
}

void alsa::hw_params_free(snd_pcm_hw_params_t *_pParams) {
    snd_pcm_hw_params_free(_pParams);
}

int alsa::prepare(snd_pcm_t *_pPcm) {
    const int nResult = snd_pcm_prepare(_pPcm);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_prepare()");

    return nResult;
}

int alsa::drain(snd_pcm_t *_pPcm) {
    const int nResult = snd_pcm_drain(_pPcm);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_drain()");

    return nResult;
}

int alsa::close(snd_pcm_t *_pPcm) {
    const int nResult = snd_pcm_close(_pPcm);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_close()");

    return nResult;
}

snd_pcm_sframes_t alsa::writei(snd_pcm_t *_pPcm, const void *_pBuffer,
        snd_pcm_uframes_t _cSize)
{
    const int nResult = snd_pcm_writei(_pPcm, _pBuffer, _cSize);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_writei(size = %u)", _cSize);

    return nResult;
}

int alsa::recover(snd_pcm_t *_pPcm, int _nError, bool _bSilent) {
    const int nResult = snd_pcm_recover(_pPcm, _nError, _bSilent);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_recover(error = %d, silent = %d)", _nError, _bSilent);

    return nResult;
}

int alsa::wait(snd_pcm_t *_pPcm, int _nTimeout) {
    const int nResult = snd_pcm_wait(_pPcm, _nTimeout);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_wait(timeout = %d)", _nTimeout);

    return nResult;
}

snd_pcm_sframes_t alsa::avail_update(snd_pcm_t *_pPcm) {
    const int nResult = snd_pcm_avail_update(_pPcm);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_avail_update()");

    return nResult;
}

snd_pcm_sframes_t alsa::avail(snd_pcm_t *_pPcm) {
    snd_pcm_sframes_t nResult = snd_pcm_avail(_pPcm);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_avail()");

    return nResult;
}

int alsa::pause(snd_pcm_t *_pPcm, bool _bEnable) {
    const int nResult = snd_pcm_pause(_pPcm, _bEnable);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_pause(enable = %d)", _bEnable);

    return nResult;
}

unsigned int alsa::format_physical_width(snd_pcm_format_t _format) {
    const int nResult = snd_pcm_format_physical_width(_format);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_format_physical_width(format = %d)", _format);

    return nResult;
}

int alsa::start(snd_pcm_t *_pPcm) {
    const int nResult = snd_pcm_start(_pPcm);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_start()");

    return nResult;
}

int alsa::ioplug_set_param_list(snd_pcm_ioplug_t *_pIO, int _nType,
        unsigned int _cNumList, const unsigned int *_pcList)
{
    const int nResult = snd_pcm_ioplug_set_param_list(_pIO, _nType, _cNumList, _pcList);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_ioplug_set_param_list(type = %d, num_list = %u)",
                _nType, _cNumList);

    return nResult;
}

int alsa::ioplug_set_param_minmax(snd_pcm_ioplug_t *_pIO, int _nType,
        unsigned int _cMin, unsigned int _cMax)
{
    const int nResult = snd_pcm_ioplug_set_param_minmax(_pIO, _nType, _cMin, _cMax);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_ioplug_set_param_minmax(type = %d, min = %u, max = %u)",
                _nType, _cMin, _cMax);

    return nResult;
}

int alsa::ioplug_create(snd_pcm_ioplug_t *_pIO, const char *_strName,
              snd_pcm_stream_t _stream, int _nMode)
{
    const int nResult = snd_pcm_ioplug_create(_pIO, _strName, _stream, _nMode);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_ioplug_create(name = %s, stream = %d, mode = %d)",
                _strName, _stream, _nMode);

    return nResult;
}

int alsa::sw_params_current(snd_pcm_t *_pPcm, snd_pcm_sw_params_t *_pParams) {
    const int nResult = snd_pcm_sw_params_current(_pPcm, _pParams);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_sw_params_current()");

    return nResult;
}

int alsa::sw_params_set_start_threshold(snd_pcm_t *_pPcm,
        snd_pcm_sw_params_t *_pParams, snd_pcm_uframes_t _cVal)
{
    const int nResult = snd_pcm_sw_params_set_start_threshold(_pPcm, _pParams, _cVal);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_sw_params_set_start_threshold(val = %u)", _cVal);

    return nResult;
}

int alsa::sw_params_get_boundary(snd_pcm_sw_params_t *_pParams, snd_pcm_uframes_t *_pVal) {
    const int nResult = snd_pcm_sw_params_get_boundary(_pParams, _pVal);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_sw_params_get_boundary()");

    return nResult;
}

int alsa::sw_params(snd_pcm_t *_pPcm, snd_pcm_sw_params_t *_pParams) {
    const int nResult = snd_pcm_sw_params(_pPcm, _pParams);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_sw_params()");

    return nResult;
}

int alsa::delay(snd_pcm_t *_pPcm, snd_pcm_sframes_t *_pDelay) {
    const int nResult = snd_pcm_delay(_pPcm, _pDelay);

    if (nResult < 0)
        throw Error(nResult, "snd_pcm_delay()");

    return nResult;
}

snd_pcm_state_t alsa::state(snd_pcm_t *_pPcm) {
    return snd_pcm_state(_pPcm);
}
