cmake_minimum_required(VERSION 3.20)
project(enet C)

set(ENET_SOURCES
    callbacks.c
    compress.c
    host.c
    list.c
    packet.c
    peer.c
    protocol.c
)
if(WIN32)
    list(APPEND ENET_SOURCES win32.c)
else()
    # POSIX (Linux/macOS/Emscripten). win32.c compiles to an empty unit off
    # Windows but then the real implementation would be missing at link time.
    list(APPEND ENET_SOURCES unix.c)
endif()

add_library(enet STATIC ${ENET_SOURCES})

target_include_directories(enet PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if(WIN32)
    target_link_libraries(enet PUBLIC winmm ws2_32)
endif()
