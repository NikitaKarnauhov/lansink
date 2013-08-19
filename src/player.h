/*
    player.h

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

#ifndef LANSINK_PLAYER_H_
#define LANSINK_PLAYER_H_

#include <chrono>
#include <lansink.pb.h>
#include "log.h"

class Player {
public:
    typedef std::chrono::high_resolution_clock::time_point TimePoint;

    ~Player();

    static Player *get(lansink::Packet &_packet, Log &_log, TimePoint &_closeTime);
    static void remove(Player *_pPlayer);
    static bool remove_stopped(TimePoint &_closeTime);

    bool is_prepared() const;
    void init(lansink::Packet &_packet);
    void run();
    void play(lansink::Packet &_packet);
    uint64_t get_id() const;

private:
    class Impl;
    Impl *m_pImpl;

    Player(uint64_t _cStreamId, Log &_log);
};

#endif /* LANSINK_PLAYER_H_ */
