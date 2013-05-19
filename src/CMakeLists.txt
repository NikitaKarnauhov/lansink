cmake_minimum_required(VERSION 2.8)

set(TARGET unap)

add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/unap.pb.cc
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS unap.proto
    COMMAND protoc -I${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/unap.proto
        --cpp_out=${CMAKE_CURRENT_BINARY_DIR}
)

set(CLIENT_SOURCES
    unap.cpp
    pcm_unap.cpp
    formats.cpp
    alsa_wrapper.cpp
    utils.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/unap.pb.cc
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
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -DPIC")


add_library(${TARGET} SHARED ${CLIENT_SOURCES})
target_link_libraries(${TARGET} dl ${ALSA_LIBRARIES} ${PROTOBUF_LIBRARIES})


set(SERVER_SOURCES
    daemon.cpp
    log.cpp
    alsa_wrapper.cpp
    player.cpp
    formats.cpp
    lexer.cpp
    settings.cpp
    utils.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/unap.pb.cc
)

add_executable(unapd ${SERVER_SOURCES})
target_link_libraries(unapd ${ALSA_LIBRARIES} ${PROTOBUF_LIBRARIES})
