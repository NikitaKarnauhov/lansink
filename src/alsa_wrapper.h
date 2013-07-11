/*
 * alsa_wrapper.h
 *
 *  Created on: May 4, 2013
 *      Author: nikita.karnauhov@gmail.com
 */

#ifndef ALSA_WRAPPER_H_
#define ALSA_WRAPPER_H_

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

struct ALSA {
    class Error : public RuntimeError {
    public:
        template<typename... Args>
        Error(int _nError, const char *_strFormat, Args... _args) :
            RuntimeError((std::string(_strFormat) + ": %s").c_str(), _args..., snd_strerror(_nError)),
            m_nError(_nError)
        {
        }

        int getError() const {
            return m_nError;
        }

    private:
        int m_nError;
    };

    static int open(snd_pcm_t **_ppPcm, const char *_strName, snd_pcm_stream_t _stream, int _nMode);
    static int close(snd_pcm_t *_pPcm);
    static int hw_params_malloc(snd_pcm_hw_params_t **_ppParams);
    static int hw_params_any(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams);
    static int hw_params_set_access(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams,
            snd_pcm_access_t _access);
    static int hw_params_set_format(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams,
            snd_pcm_format_t _format);
    static int hw_params_set_rate_near(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams,
            unsigned int *_puRate, int *_pnDir);
    static int hw_params_set_channels(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams,
            unsigned int _uChannels);
    static int hw_params_set_buffer_size_near(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams,
            snd_pcm_uframes_t *_puSize);
    static int hw_params_set_period_size_near(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams,
            snd_pcm_uframes_t *_puSize, int *_pnDir);
    static int hw_params(snd_pcm_t *_pPcm, snd_pcm_hw_params_t *_pParams);
    static void hw_params_free(snd_pcm_hw_params_t *_pParams);
    static int prepare(snd_pcm_t *_pPcm);

    static int drain(snd_pcm_t *_pPcm);
    static snd_pcm_sframes_t writei(snd_pcm_t *_pPcm, const void *_pBuffer,
            snd_pcm_uframes_t _cSize);
    static int recover(snd_pcm_t *_pPcm, int _nError, bool _bSilent);
    static int wait(snd_pcm_t *_pPcm, int _nTimeout);
    static snd_pcm_sframes_t avail_update(snd_pcm_t *_pPcm);

    static unsigned int format_physical_width(snd_pcm_format_t _format);
};

#endif /* ALSA_WRAPPER_H_ */
