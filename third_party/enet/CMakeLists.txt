cmake_minimum_required(VERSION 3.20)
project(enet C)

add_library(enet STATIC
    callbacks.c
    compress.c
    host.c
    list.c
    packet.c
    peer.c
    protocol.c
    win32.c
)

target_include_directories(enet PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if(WIN32)
    target_link_libraries(enet PUBLIC winmm ws2_32)
endif()
