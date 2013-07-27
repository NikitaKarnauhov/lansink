/*
 * unap.h
 *
 *  Created on: Apr 2, 2013
 *      Author: nikita.karnauhov@gmail.com
 */

#ifndef UNAP_H_
#define UNAP_H_

#include <functional>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <list>
#include <mutex>
#include <random>

#include <alsa/input.h>
#include <alsa/output.h>
#include <alsa/global.h>
#include <alsa/conf.h>
#include <alsa/pcm.h>
#include <alsa/pcm_extplug.h>
#include <alsa/control.h>
#include <alsa/pcm_external.h>

#include "log.h"
#include "unap.pb.h"

class UNAP : public snd_pcm_ioplug_t {
public:
    std::string strHost;
    long nPort;
    long nMTU;

    std::vector<unsigned int> rateValues, channelValues;
    std::vector<std::string> formats;

    mutable Log log;

    enum Status {
        usRunning = 0x40,
        usStopped = 0x01,
        usStopping = 0x02 | usRunning,
        usPaused = 0x04 | usRunning
    };

    UNAP();
    ~UNAP();

    Status get_status() const;
    void start();
    void stop();
    snd_pcm_sframes_t get_buffer_pointer() const;
    snd_pcm_sframes_t transfer(const char *_pData, size_t _cOffset, size_t _cSize);
    void prepare();
    void drain();
    void pause();
    void unpause();
    snd_pcm_sframes_t get_unplayed_frames() const;
    void connect();
    const std::vector<unsigned int> &get_format_values();
    unsigned int get_channel_count() const;
    snd_pcm_uframes_t get_buffer_size() const;
    snd_pcm_uframes_t get_period_size() const;
    size_t get_bytes_per_frame() const;
    snd_pcm_format_t get_format() const;
    unsigned int get_rate() const;

private:
    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::duration<int, std::milli> Duration;
    typedef std::chrono::time_point<Clock> TimePoint;

    std::vector<unsigned int> m_formatValues;
    std::string m_strFormat;
    size_t m_cBitsPerSample;
    size_t m_cChannels;
    std::thread m_worker;
    mutable std::mutex m_mutex;
    int m_nSockWorker;
    TimePoint m_startTime;
    snd_pcm_sframes_t m_nLastFrames;
    snd_pcm_sframes_t m_nAvail;
    snd_pcm_sframes_t m_nPointer;
    std::list<std::pair<char *, char *> > m_queue;
    std::unique_ptr<char[]> m_pBuffer;
    Status m_status;
    bool m_bPrepared;
    int m_nSocket;
    static unsigned int s_cSeed;
    static std::default_random_engine s_randomEngine;
    uint64_t m_cStreamId;

    void _reset(bool _bResetStreamParams);
    std::function<void(void)> _make_worker();
    void _init_descriptors();
    snd_pcm_sframes_t _estimate_frames() const;
    void _prepare_packet(unap::Packet &_packet, unap::Packet_Kind _kind, uint64_t _nTimestamp);
    void _send_buffer(const void *_pBuf, size_t _cSize);
    void _send_packet(const unap::Packet &_packet);
    void _send_data();
    void _send_pause();
    void _send_unpause();
    void _send_stop();
};

#endif /* UNAP_H_ */
