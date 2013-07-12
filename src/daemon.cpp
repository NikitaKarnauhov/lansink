/*
 * daemon.cpp
 *
 *  Created on: Apr 7, 2013
 *      Author: nikita.karnauhov@gmail.com
 */

#include <string>
#include <stdexcept>
#include <memory>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <list>
#include <set>
#include <thread>
#include <mutex>
#include <algorithm>

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netdb.h>
#include <poll.h>
#include <arpa/inet.h>
#include <errno.h>
#include <string.h>
#include <getopt.h>
#include <signal.h>

#include <alsa/asoundlib.h>

#include <unap.pb.h>
#include "log.h"
#include "alsa_wrapper.h"
#include "player.h"
#include "settings.h"

// Signal flags.
static int g_nExitSignal = 0;

static
void _print_usage(std::ostream &_os) {
    _os << "UNAPd, network audio receiver.\n" <<
            "Usage: unapd [OPTION]...\n\n" <<
            "  -h, --help              Print this message and exit.\n" <<
            "  -v, --version           Display the version and exit.\n" <<
            "  -c, --config-path       Set configuration file location.\n" <<
            "  -l, --log-path          Set log file location.\n" <<
            "  -L, --log-level         Set log verbosity level (0 to 4, use 0 to disable logging).\n" <<
            "  -d, --daemon            Run as a daemon.\n" <<
            "  -n, --no-daemon         Don't run as a daemon, display log messages (default).\n" <<
            "  -p, --pid-path          PID file location.\n" <<
            "  -D, --alsa-device       Output ALSA device.\n" <<
            "" << std::flush;
}

static
void _print_version() {
    std::cout << "UNAPd, network audio receiver.\n" <<
            "Version 0.1" << std::endl;
}

static
void _parse_options(int _nArgs, char *const _pArgs[]) {
    static struct option options[] = {
            {"help", 0, 0, 'h'},
            {"version", 0, 0, 'v'},
            {"config-path", required_argument, 0, 'c'},

            // These options have equivalents in settings file.
            {"log-path", required_argument, 0, 'l'},
            {"log-level", required_argument, 0, 'L'},
            {"daemon", 0, 0, 'd'},
            {"no-daemon", 0, 0, 'n'},
            {"pid-path", 0, 0, 'p'},
            {"alsa-device", 0, 0, 'D'},
            {0, 0, 0, 0}
    };

    const char *strOptions = "hvc:l:L:dnp:D:";
    int nOption = 0;
    SettingsParser sp;
    std::map<std::string, std::string> kvs;

    // TODO: build std::map<int, std::string>

    // Handle --help, --version and --config-path options.
    while (true) {
        const int c = getopt_long(_nArgs, _pArgs, strOptions, options, &nOption);

        if (c == -1) {
            if (optind < _nArgs)
                throw RuntimeError("Redundant argument: %s", _pArgs[optind]);
            break;
        }

        switch (c) {
            case 'h':
                _print_usage(std::cout);
                exit(EXIT_SUCCESS);

            case 'v':
                _print_version();
                exit(EXIT_SUCCESS);

            case 'c':
                sp.parse_file(optarg);
                break;

            case '?':
                _print_usage(std::cerr);
                exit(EXIT_FAILURE);

            default:
                kvs[std::find_if(std::begin(options), std::end(options),
                        [c](struct option &_opt) {return _opt.val == c;})->name] = optarg;
                break;
        }
    }

    for (auto &kv : kvs)
        sp.parse_option(kv.first, kv.second);

    g_settings = sp.get();
}

static
void _main(Log &_log) {
    std::string strHost("127.0.0.1");
    std::string strPort("26751");

//    int m_nSocket = socket(PF_INET, SOCK_DGRAM, 0);
//
//    if (m_nSocket < 0)
//        throw std::runtime_error(strerror(errno));
//
//    struct sockaddr_in name;
//
//    name.sin_family = AF_INET;
//    name.sin_port = htons(nPort);
//
//    struct hostent *pHost = gethostbyname(strHost.c_str());
//
//    if (!pHost)
//        throw std::runtime_error(strerror(errno));
//
//    name.sin_addr = *(struct in_addr *)pHost->h_addr;


    struct addrinfo hints;
    struct addrinfo *serverinfo;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags = AI_PASSIVE;

    if (getaddrinfo(NULL, strPort.c_str(), &hints, &serverinfo) < 0)
        throw SystemError("getaddrinfo()");

    const int nSocket = socket(serverinfo->ai_family, serverinfo->ai_socktype, serverinfo->ai_protocol);

    if (bind(nSocket, serverinfo->ai_addr, serverinfo->ai_addrlen) < 0)
        throw SystemError("bind()");

    freeaddrinfo(serverinfo);

    struct pollfd fd{nSocket, POLLIN, 0};
    size_t cBufferSize = 1024*100; // Should be enough, right?
    auto pBuf = std::unique_ptr<char[]>(new char[cBufferSize]);
    std::list<unap::Packet> packets;
//    unap::Packet p;
//
//    player.init(p);
//    player.play(p);
//
//    return EXIT_SUCCESS;

    do {
        try {
            if (poll(&fd, 1, 1000) < 0)
                throw SystemError("poll()");

            if (fd.revents & POLLIN) {
                struct sockaddr sender;
                socklen_t sendsize = sizeof(sender);
                bzero(&sender, sizeof(sender));
                const int nPacketSize = recvfrom(fd.fd, pBuf.get(), cBufferSize, 0, &sender, &sendsize);
                char strSender[128];

                if (nPacketSize < 0)
                    throw SystemError("recvfrom()");

                inet_ntop(sender.sa_family, &((struct sockaddr_in &)sender).sin_addr,
                        strSender, sizeof(strSender));

                unap::Packet &packet = *packets.emplace(packets.end());

                if (!packet.ParseFromArray(pBuf.get(), nPacketSize)) {
                    _log.debug("Broken packet.");
                    continue;
                }

                Player *pPlayer = Player::get(packet, _log);

                if (!pPlayer->is_prepared()) {
                    pPlayer->init(packet);
                    pPlayer->run();
                }

                pPlayer->play(packet);
            }
        } catch (SystemError &se) {
            if (se.getError() != EINTR)
                throw;
        }

        if (g_nExitSignal != 0)
            _log.info("Got signal %d, exiting gracefully.", g_nExitSignal);
    } while (g_nExitSignal == 0);
}

static
bool _write_pid(Log &_log) {
    // Specify empty PID path to disable writing PID.
    if (g_settings.strPIDPath.empty())
        return false;

    std::ofstream os(g_settings.strPIDPath);

    if (!os.good())
        throw RuntimeError("Cannot write PID file: %s", g_settings.strPIDPath.c_str());

    os << getpid() << std::endl;
    _log.info("PID file written to %s", g_settings.strPIDPath.c_str());

    return true;
}

static
void _handle_signal(int _nSignal, siginfo_t *_pSigInfo, void *_pContext) {
    g_nExitSignal = _nSignal;
}

static
void _init_signals() {
    struct sigaction action = {};
    const int exitSignals[]{
            SIGTERM, SIGINT, SIGPIPE, SIGALRM, SIGUSR1, SIGUSR2, SIGPOLL,
            SIGPROF, SIGVTALRM
    };

    action.sa_sigaction = &_handle_signal;
    action.sa_flags = SA_SIGINFO;

    for (int nSignal : exitSignals)
        if (sigaction(nSignal, &action, NULL) < 0)
            throw SystemError("sigaction()");

}

int main(int _nArgs, char *const _pArgs[]) {
    Log log("");
    bool bPIDWritten = false;

    try {
        _parse_options(_nArgs, _pArgs);
        log.setLevel(LogLevel(g_settings.nLogLevel));

        if (!g_settings.strLogPath.empty()) {
            log.open(g_settings.strLogPath);
            log.info("Logging to %s", g_settings.strLogPath.c_str());
        }

        if (g_settings.bDaemon) {
            log.close("");

            if (daemon(true, false) != 0)
                throw SystemError("daemon()");
        }

        _init_signals();
        bPIDWritten = _write_pid(log);
        _main(log);
    } catch (std::exception &e) {
        log.log(llError, e.what());
        return EXIT_FAILURE;
    }

    if (bPIDWritten)
        unlink(g_settings.strPIDPath.c_str());

    return EXIT_SUCCESS;
}
