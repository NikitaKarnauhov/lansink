/*
    alsa_sink.cpp

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

#include "sink.h"

#include <alsa/asoundlib.h>

#include "alsa.h"
#include "settings.h"

class ALSASink : public Sink {
public:
    ALSASink(Log &_log);

    virtual ~ALSASink();

    virtual bool is_prepared() const {
        return m_pPcm != nullptr;
    }

    virtual void init(size_t _cChannels, const size_t _cRate, const std::string &_strFormat);
    virtual bool prepare();
    virtual void pause(bool _bEnable);

    virtual unsigned long get_buffer_size() const {
        return m_cBufferSize;
    }

    virtual unsigned long get_period_size() const {
        return m_cPeriodSize;
    }

    virtual unsigned int get_rate() const {
        return m_cRate;
    }

    virtual unsigned int get_frame_bytes() const {
        return m_cFrameBytes;
    }

    virtual bool can_be_paused() const {
        return m_bCanBePaused;
    }

    virtual long get_delay();
    virtual long get_avail(bool _bSync);
    virtual void recover(int _nError);

    virtual void report_state() {
        m_pLog->debug("State: %d", alsa::state(m_pPcm));
    }

    virtual bool is_buffering() const {
        return alsa::state(m_pPcm) == SND_PCM_STATE_PREPARED;
    }

    virtual bool is_running() const {
        return alsa::state(m_pPcm) == SND_PCM_STATE_RUNNING;
    }

    virtual bool is_paused() const {
        return alsa::state(m_pPcm) == SND_PCM_STATE_PAUSED;
    }

    virtual bool is_underrun() const {
        return alsa::state(m_pPcm) == SND_PCM_STATE_XRUN;
    }

    virtual void start();
    virtual void close();
    virtual long write(const void *_pBuffer, unsigned long _cFrames);

private:
    snd_pcm_t *m_pPcm;
    snd_pcm_uframes_t m_cBufferSize;
    snd_pcm_uframes_t m_cPeriodSize;
    size_t m_cChannelCount;
    unsigned int m_cRate;
    snd_pcm_format_t m_format;
    size_t m_cFrameBytes;
    Log *m_pLog;
    bool m_bReady;
    bool m_bCanBePaused;
    snd_pcm_sframes_t m_nBufferedFrames;

    void _handle(alsa::Error &_e) const __attribute__((noreturn));
};

ALSASink::ALSASink(Log &_log) :
    m_pPcm(nullptr), m_cBufferSize(2048), m_cPeriodSize(512), m_cChannelCount(0),
    m_cRate(0), m_format(SND_PCM_FORMAT_UNKNOWN), m_cFrameBytes(0), m_pLog(&_log),
    m_bReady(false), m_bCanBePaused(false), m_nBufferedFrames(0)
{
}

ALSASink::~ALSASink() {
    try {
        if (m_pPcm) {
            alsa::drain(m_pPcm);
            alsa::close(m_pPcm);
        }

        m_pPcm = nullptr;
    } catch (alsa::Error &e) {
        _handle(e);
    }
}

void ALSASink::init(size_t _cChannels, const size_t _cRate, const std::string &_strFormat) {
    m_cChannelCount = _cChannels;
    m_cRate = _cRate;
    m_format = alsa::get_format(_strFormat);
}

bool ALSASink::prepare() {
    try {
        snd_pcm_hw_params_t *pParams;

        if (m_pPcm == nullptr) {
            alsa::open(&m_pPcm, g_settings.strALSADevice.c_str(), SND_PCM_STREAM_PLAYBACK, 0);
            alsa::hw_params_malloc(&pParams);
            alsa::hw_params_any(m_pPcm, pParams);
            alsa::hw_params_set_access(m_pPcm, pParams, SND_PCM_ACCESS_RW_INTERLEAVED);
            alsa::hw_params_set_format(m_pPcm, pParams, m_format);
            alsa::hw_params_set_rate_near(m_pPcm, pParams, &m_cRate, 0);
            alsa::hw_params_set_channels(m_pPcm, pParams, m_cChannelCount);
            alsa::hw_params_set_buffer_size_near(m_pPcm, pParams, &m_cBufferSize);
            alsa::hw_params_set_period_size_near(m_pPcm, pParams, &m_cPeriodSize, NULL);
            alsa::hw_params(m_pPcm, pParams);
            m_bCanBePaused = alsa::hw_params_can_pause(pParams);
            alsa::hw_params_free(pParams);

            snd_pcm_sw_params_t *pSWParams;
            snd_pcm_uframes_t cBoundary;

            snd_pcm_sw_params_alloca(&pSWParams);
            alsa::sw_params_current(m_pPcm, pSWParams);
            alsa::sw_params_get_boundary(pSWParams, &cBoundary);
            alsa::sw_params_set_start_threshold(m_pPcm, pSWParams, cBoundary);
            alsa::sw_params(m_pPcm, pSWParams);

            m_pLog->info("Opened ALSA device \"%s\"", g_settings.strALSADevice.c_str());
        }

        alsa::prepare(m_pPcm);

        m_cFrameBytes = alsa::format_physical_width(m_format)*m_cChannelCount/8;
        m_nBufferedFrames = 0;
        m_bReady = true;
    } catch (std::exception &e) {
        m_pLog->error(e.what());
        m_bReady = false;

        if (m_pPcm) {
            alsa::close(m_pPcm);
            m_pPcm = nullptr;
        }

        return false;
    }

    return true;
}

void ALSASink::pause(bool _bEnable) {
    try {
        alsa::pause(m_pPcm, _bEnable);

        if (_bEnable)
            m_nBufferedFrames = get_delay();
    } catch (alsa::Error &e) {
        _handle(e);
    }
}

long ALSASink::get_delay() {
    snd_pcm_sframes_t nDelay = m_cBufferSize;

    if (is_buffering() || is_running())
        try {
            alsa::delay(m_pPcm, &nDelay);
            m_nBufferedFrames = nDelay;
        } catch (alsa::Error &e) {
            if (e.get_error() == -EIO) {
                m_pLog->warning(e.what());
                nDelay = std::min<snd_pcm_sframes_t>(m_cBufferSize, m_nBufferedFrames);
            } else
                _handle(e);
        }
    else if (is_paused())
        nDelay = std::min<snd_pcm_sframes_t>(m_cBufferSize, m_nBufferedFrames);

    return nDelay;
}

long ALSASink::get_avail(bool _bSync) {
    snd_pcm_sframes_t nFrames = 0;

    if (is_buffering() || is_running())
        try {
            nFrames = _bSync ? alsa::avail(m_pPcm) : alsa::avail_update(m_pPcm);
        } catch (alsa::Error &e) {
            if (e.get_error() == -EIO) {
                m_pLog->warning(e.what());
                nFrames = std::max<snd_pcm_sframes_t>(0, get_buffer_size() - m_nBufferedFrames);
            } else
                _handle(e);
        }
    else if (is_paused())
        nFrames = std::max<snd_pcm_sframes_t>(0, get_buffer_size() - m_nBufferedFrames);

    return nFrames;
}

void ALSASink::recover(int _nError) {
    try {
        alsa::recover(m_pPcm, _nError, true);
        m_nBufferedFrames = 0;
    } catch (alsa::Error &e) {
        _handle(e);
    }
}

void ALSASink::start() {
    try {
        alsa::start(m_pPcm);
    } catch (alsa::Error &e) {
        _handle(e);
    }
}

void ALSASink::close() {
    try {
        alsa::close(m_pPcm);
        m_pPcm = nullptr;
    } catch (alsa::Error &e) {
        _handle(e);
    }
}

long ALSASink::write(const void *_pBuffer, unsigned long _cFrames) {
    try {
        const snd_pcm_sframes_t nFramesWritten = alsa::writei(m_pPcm, _pBuffer, _cFrames);
        m_nBufferedFrames += nFramesWritten;
        return nFramesWritten;
    } catch (alsa::Error &e) {
        _handle(e);
    }
}

void ALSASink::_handle(alsa::Error &_e) const {
    if (_e.get_error() == -EPIPE)
        throw Error(Error::seUnderrun, _e.get_error(), _e.what());

    // Cannot recover from EIO: someone tells us device was unplugged.
    if (_e.get_error() == -EIO)
        throw Error(Error::seFatal, _e.get_error(), _e.what());

    throw Error(Error::seNormal, _e.get_error(), _e.what());
}

Sink *create_alsa_sink(Log &_log) {
    return new ALSASink(_log);
}
