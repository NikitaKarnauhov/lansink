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
#include <unordered_map>

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
#include "alsa.h"
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
            "  -H, --host              Server host.\n" <<
            "  -P, --port              Server port.\n" <<
            "  -D, --alsa-device       Output ALSA device.\n" <<
            "      --recovery-timeout  Seconds to wait after for stream data to reappear.\n" <<
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
            {"pid-path", required_argument, 0, 'p'},
            {"host", required_argument, 0, 'H'},
            {"port", required_argument, 0, 'P'},
            {"alsa-device", required_argument, 0, 'D'},
            {"recovery-timeout", required_argument, 0, 500},
            {0, 0, 0, 0}
    };

    const char *strOptions = "hvc:l:L:dnp:H:P:D:";
    int nOption = 0;
    SettingsParser sp;
    std::map<std::string, std::string> kvs;
    std::unordered_map<int, std::string> longNames;

    for (auto &option : options)
        if (option.name)
            longNames[option.val] = option.name;

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

            case 'd':
                kvs["daemon"] = "true";
                break;

            case 'n':
                kvs["daemon"] = "false";
                break;

            case '?':
                _print_usage(std::cerr);
                exit(EXIT_FAILURE);

            default:
                if (!optarg)
                    throw LogicError("Option '%c' reuires argument", c);

                kvs[longNames[c]] = optarg;
                break;
        }
    }

    for (auto &kv : kvs)
        sp.parse_option(kv.first, kv.second);

    g_settings = sp.get();
}

static
std::string _in_addr_to_string(struct sockaddr *_pAddr) {
    char buf[INET6_ADDRSTRLEN];
    void *pInAddr = nullptr;

    if (_pAddr->sa_family == AF_INET)
        pInAddr = &(((struct sockaddr_in *) _pAddr)->sin_addr);
    else
        pInAddr = &(((struct sockaddr_in6 *) _pAddr)->sin6_addr);

    inet_ntop(_pAddr->sa_family, pInAddr, buf, sizeof(buf));

    return buf;
}

static
void _main(Log &_log) {
    std::string strPort = format(16, "%d", g_settings.nPort);
    struct addrinfo hints = {};
    struct addrinfo *pServerInfo = nullptr;

    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags = AI_PASSIVE;

    if (getaddrinfo(g_settings.strHost.empty() ? NULL : g_settings.strHost.c_str(),
            strPort.c_str(), &hints, &pServerInfo) < 0)
        throw SystemError("getaddrinfo()");

    const int nSocket = socket(pServerInfo->ai_family,
            pServerInfo->ai_socktype, pServerInfo->ai_protocol);

    if (bind(nSocket, pServerInfo->ai_addr, pServerInfo->ai_addrlen) < 0)
        throw SystemError("bind()");

    _log.info("Server listening on %s:%d",
            _in_addr_to_string(pServerInfo->ai_addr).c_str(), g_settings.nPort);

    freeaddrinfo(pServerInfo);

    struct pollfd fd{nSocket, POLLIN, 0};
    constexpr size_t cBufferSize = 1024*100; // Should be enough, right?
    auto pBuf = std::unique_ptr<char[]>(new char[cBufferSize]);
    std::list<unap::Packet> packets;

    do {
        try {
            if (poll(&fd, 1, 1000) < 0)
                throw SystemError("poll()");

            if (fd.revents & POLLIN) {
                struct sockaddr sender = {};
                socklen_t cSendSize = sizeof(sender);
                const int nPacketSize = recvfrom(
                        fd.fd, pBuf.get(), cBufferSize, 0, &sender, &cSendSize);

                if (nPacketSize < 0)
                    throw SystemError("recvfrom()");

                unap::Packet &packet = *packets.emplace(packets.end());

                if (!packet.ParseFromArray(pBuf.get(), nPacketSize)) {
                    _log.debug("Broken packet from %s", _in_addr_to_string(&sender).c_str());
                    continue;
                }

                Player *pPlayer = Player::get(packet, _log);

                if (!pPlayer->is_prepared()) {
                    // FIXME handle 'device or resource busy'.
                    _log.info("New connection from %s", _in_addr_to_string(&sender).c_str());
                    pPlayer->init(packet);
                    pPlayer->run();
                }

                pPlayer->play(packet);
            }
        } catch (SystemError &se) {
            if (se.get_error() != EINTR)
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

    log.setLevel(llError);

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
        log.error(e.what());
        return EXIT_FAILURE;
    }

    if (bPIDWritten)
        unlink(g_settings.strPIDPath.c_str());

    return EXIT_SUCCESS;
}
