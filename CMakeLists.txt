cmake_minimum_required(VERSION 2.8.5)
project(LANSink)

include(GNUInstallDirs)
include(FindPkgConfig)

pkg_check_modules(ALSA alsa)

if (ALSA_FOUND)
    include_directories(${ALSA_INCLUDE_DIRS})
endif(ALSA_FOUND)

pkg_check_modules(PROTOBUF protobuf)

if (PROTOBUF_FOUND)
    include_directories(${PROTOBUF_INCLUDE_DIRS})
endif(PROTOBUF_FOUND)

add_subdirectory(src)
