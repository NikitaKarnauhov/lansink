/*
    sender.h

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

#ifndef LANSINK_SENDER_H_
#define LANSINK_SENDER_H_

#include <string>
#include <vector>

#include <alsa/input.h>
#include <alsa/output.h>
#include <alsa/global.h>
#include <alsa/conf.h>
#include <alsa/pcm.h>
#include <alsa/pcm_extplug.h>
#include <alsa/control.h>
#include <alsa/pcm_external.h>

#include "log.h"
#include "lansink.pb.h"

class Sender : public snd_pcm_ioplug_t {
public:
    std::string strHost;
    long nPort;
    long nMTU;
    long nSendPeriod;

    std::vector<unsigned int> rateValues, channelValues;
    std::vector<std::string> formats;

    mutable Log log;

    enum Status {
        usRunning = 0x40,
        usStopped = 0x01,
        usStopping = 0x02 | usRunning,
        usPaused = 0x04 | usRunning,
        usUnderrun = 0x08 | usRunning
    };

    Sender();
    ~Sender();

    Status get_status() const;
    void start();
    void stop();
    snd_pcm_sframes_t get_buffer_pointer() const;
    snd_pcm_sframes_t transfer(const char *_pData, size_t _cOffset, size_t _cSize);
    void prepare();
    void drain();
    void pause();
    void unpause();
    snd_pcm_sframes_t get_delay() const;
    void connect();
    const std::vector<unsigned int> &get_format_values();
    unsigned int get_channel_count() const;
    snd_pcm_uframes_t get_buffer_size() const;
    snd_pcm_uframes_t get_period_size() const;
    size_t get_bytes_per_frame() const;
    snd_pcm_format_t get_format() const;
    unsigned int get_rate() const;

private:
    class Impl;
    Impl *m_pImpl;
};

#endif /* LANSINK_SENDER_H_ */
