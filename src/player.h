/*
 * player.h
 *
 *  Created on: May 4, 2013
 *      Author: nikita.karnauhov@gmail.com
 */

#ifndef PLAYER_H_
#define PLAYER_H_

#include <unap.pb.h>
#include "log.h"

class Player {
public:
    ~Player();

    static Player *get(unap::Packet &_packet, Log &_log);
    static void remove(Player *_pPlayer);
    static void remove_stopped();

    bool is_prepared() const;
    void init(unap::Packet &_packet);
    void run();
    void play(unap::Packet &_packet);
    uint64_t get_id() const;

private:
    class Impl;
    Impl *m_pImpl;

    Player(uint64_t _cStreamId, Log &_log);
};

#endif /* PLAYER_H_ */
