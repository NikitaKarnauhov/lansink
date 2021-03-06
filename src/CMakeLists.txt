cmake_minimum_required(VERSION 2.8.5)

set(CLIENT asound_module_pcm_lansink)
set(SERVER lansinkd)

add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/lansink.pb.cc
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS lansink.proto
    COMMAND protoc -I${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/lansink.proto
        --cpp_out=${CMAKE_CURRENT_BINARY_DIR}
)

set(CLIENT_SOURCES
    sender.cpp
    ioplug.cpp
    alsa.cpp
    utils.cpp
    log.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/lansink.pb.cc
)

add_definitions(
	-std=c++11
    -fmessage-length=0
    -Wall
    -DPIC
    -D_GLIBCXX_USE_NANOSLEEP
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--export-dynamic -Wl,--no-undefined")  

add_library(${CLIENT} SHARED ${CLIENT_SOURCES})
target_link_libraries(${CLIENT} dl ${ALSA_LIBRARIES} ${PROTOBUF_LIBRARIES})
install(TARGETS ${CLIENT} DESTINATION ${CMAKE_INSTALL_LIBDIR}/alsa-lib)

set(SERVER_SOURCES
    daemon.cpp
    log.cpp
    alsa.cpp
    alsa_sink.cpp
    player.cpp
    lexer.cpp
    settings.cpp
    utils.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/lansink.pb.cc
)

add_executable(${SERVER} ${SERVER_SOURCES})
target_link_libraries(${SERVER} ${ALSA_LIBRARIES} ${PROTOBUF_LIBRARIES})
install(TARGETS ${SERVER} DESTINATION ${CMAKE_INSTALL_BINDIR})
