/*
    alsa.h

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

#ifndef LANSINK_ALSA_H_
#define LANSINK_ALSA_H_

#include <stdio.h>
#include <stdlib.h>
#include <alsa/input.h>
#include <alsa/output.h>
#include <alsa/global.h>
#include <alsa/conf.h>
#include <alsa/pcm.h>
#include <alsa/pcm_extplug.h>
#include <alsa/control.h>
#include <alsa/pcm_external.h>
#include <alsa/error.h>

#include <stdexcept>

#include "exception.h"

namespace alsa {

class Error : public RuntimeError {
public:
    template<typename... Args>
    Error(int _nError, const char *_strFormat, Args... _args) :
        RuntimeError((std::string(_strFormat) + ": %s").c_str(), _args..., snd_strerror(_nError)),
        m_nError(_nError)
    {
    }

    int get_error() const {
        return m_nError;
    }

private:
    int m_nError;
};

int open(snd_pcm_t **_ppPcm, const char *_strName, snd_pcm_stream_t _stream, int _nMode);
int close(snd_pcm_t *_pPcm);
int hw_params_malloc(snd_pcm_hw_params_t **_ppParams);
int hw_params_any(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams);
int hw_params_set_access(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams,
        snd_pcm_access_t _access);
int hw_params_set_format(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams,
        snd_pcm_format_t _format);
int hw_params_set_rate_near(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams,
        unsigned int *_puRate, int *_pnDir);
int hw_params_set_channels(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams,
        unsigned int _uChannels);
int hw_params_set_buffer_size_near(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams,
        snd_pcm_uframes_t *_puSize);
int hw_params_set_period_size_near(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams,
        snd_pcm_uframes_t *_puSize, int *_pnDir);
bool hw_params_can_pause(snd_pcm_hw_params_t *_pParams);
int hw_params(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams);
void hw_params_free(snd_pcm_hw_params_t *_pParams);
int prepare(snd_pcm_t *_pPcm);

int start(snd_pcm_t *_pPcm);
int drain(snd_pcm_t *_pPcm);
snd_pcm_sframes_t writei(snd_pcm_t *_pPcm, const void *_pBuffer,
        snd_pcm_uframes_t _cSize);
int recover(snd_pcm_t *_pPcm, int _nError, bool _bSilent);
int wait(snd_pcm_t *_pPcm, int _nTimeout);
snd_pcm_sframes_t avail_update(snd_pcm_t *_pPcm);
snd_pcm_sframes_t avail(snd_pcm_t *_pPcm);
int pause(snd_pcm_t *_pPcm, bool _bEnable);
int delay(snd_pcm_t *_pPcm, snd_pcm_sframes_t *_pDelay);
snd_pcm_state_t state(snd_pcm_t *_pPcm);

int ioplug_set_param_list(snd_pcm_ioplug_t *_pIO, int _nType,
        unsigned int _cNumList, const unsigned int *_pcList);
int ioplug_set_param_minmax(snd_pcm_ioplug_t *_pIO, int _nType,
        unsigned int _cMin, unsigned int _cMax);
int ioplug_create(snd_pcm_ioplug_t *_pIO, const char *_strName,
              snd_pcm_stream_t _stream, int _nMode);

int sw_params_current(snd_pcm_t *_pPcm, snd_pcm_sw_params_t *_pParams);
int sw_params_set_start_threshold(snd_pcm_t *_pPcm,
        snd_pcm_sw_params_t *_pParams, snd_pcm_uframes_t _cVal);
int sw_params_get_boundary(snd_pcm_sw_params_t *_pParams, snd_pcm_uframes_t *_pVal);
int sw_params(snd_pcm_t *_pPcm, snd_pcm_sw_params_t *_pParams);

unsigned int format_physical_width(snd_pcm_format_t _format);

std::string get_format_name(snd_pcm_format_t _format);
snd_pcm_format_t get_format(const std::string &_strName);
bool is_format(const std::string &_strName);

};

#endif /* LANSINK_ALSA_H_ */
