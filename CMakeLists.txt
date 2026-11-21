cmake_minimum_required(VERSION 3.28)
project(ApexRushOnline CXX C)

# C++23: rush-engine / exstl need std::expected (and later named modules).
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_SCAN_FOR_MODULES ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND NOT EMSCRIPTEN)
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        add_compile_options(-O0)
    else()
        # Global -O3 + ThinLTO (parallel LTO, much cheaper than full LTO).
        # ThinLTO needs lld for the link step. Benchmarked faster than plain
        # -O3 and than the MSVC baseline on this codebase.
        add_compile_options(-O3 -flto=thin)
        add_link_options(-flto=thin -fuse-ld=lld)
    endif()
elseif(MSVC)
    add_compile_options(/W3 /permissive-)
endif()
add_compile_definitions(_CRT_SECURE_NO_WARNINGS)

# --- Third-party: raylib (render/input/window) -------------------------------
include(FetchContent)
# Reuse a seeded raylib source tree (copy of the patched FetchContent checkout
# under .deps/) across build trees and across build/ wipes — the configure-time
# GitHub clone was the "build hangs forever". Only the sources are shared; each
# build tree keeps its own raylib-build (native vs emscripten must not clash).
if(EXISTS "${CMAKE_SOURCE_DIR}/.deps/raylib-src/CMakeLists.txt")
    set(FETCHCONTENT_SOURCE_DIR_RAYLIB "${CMAKE_SOURCE_DIR}/.deps/raylib-src")
endif()
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
find_program(POWERSHELL_EXE NAMES powershell pwsh REQUIRED)
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
    GIT_SHALLOW TRUE
    PATCH_COMMAND ${POWERSHELL_EXE} -NoProfile -ExecutionPolicy Bypass
                  -File ${CMAKE_SOURCE_DIR}/cmake/patch_raylib.ps1 .
)
FetchContent_MakeAvailable(raylib)

# --- Third-party: qu3e (collision) / enet (networking) / nudge (alt physics) --
add_subdirectory(third_party/qu3e)
add_subdirectory(third_party/enet)

# --- Rush Engine (git submodule: render + physics + async runtime) ------------
add_subdirectory(rush-engine)

# nudge: C23 unity build of the engine core (see src/physics/nudge_unity.c).
# Compiled with the same flags upstream nudge uses for MSVC.
add_library(nudge STATIC src/physics/nudge_unity.c)
target_include_directories(nudge PUBLIC third_party/nudge)
set_target_properties(nudge PROPERTIES C_STANDARD 23 C_STANDARD_REQUIRED ON)
if(CMAKE_C_COMPILER_ID MATCHES "Clang")
    # -msse4.1: nudge's simd.h uses _mm_blendv_ps (clang enforces target
    # features on always_inline intrinsics, MSVC does not). Emscripten maps
    # the SSE intrinsics to wasm SIMD but wants -msimd128 alongside them.
    if(EMSCRIPTEN)
        target_compile_options(nudge PRIVATE -std=c23 -ffp-contract=off -msse4.1 -msimd128)
    else()
        target_compile_options(nudge PRIVATE -std=c23 -ffp-contract=off -msse4.1)
    endif()
elseif(MSVC)
    target_compile_options(nudge PRIVATE /std:clatest /experimental:c11atomics /fp:precise)
endif()

# --- Game ---------------------------------------------------------------------
add_executable(apex-rush
    src/main.cpp
    src/audio/MiniaudioAudio.cpp
    src/core/Config.cpp
    src/core/Paths.cpp
    src/physics/Qu3eWorld.cpp
    src/physics/NudgeWorld.cpp
    src/render/RaylibBackend.cpp
    src/render/NullBackend.cpp
    src/vehicle/TireModel.cpp
    src/vehicle/Vehicle.cpp
    src/game/MapData.cpp
    src/game/Track.cpp
    src/game/Game.cpp
)

target_include_directories(apex-rush PRIVATE src third_party/miniaudio)
target_link_libraries(apex-rush PRIVATE raylib qu3e enet nudge rush::engine)

# miniaudio is a ~4 MB single-header TU; the audio mixer needs no -O3 and the
# full optimization costs minutes of compile time.
set_property(SOURCE src/audio/MiniaudioAudio.cpp APPEND PROPERTY COMPILE_OPTIONS
             "$<$<CXX_COMPILER_ID:Clang>:-O1>" "$<$<CXX_COMPILER_ID:MSVC>:/O1>")

# --- Tests (ctest --preset release) ------------------------------------------
enable_testing()
add_test(NAME drive-test COMMAND apex-rush --drive-test)
set_tests_properties(drive-test PROPERTIES TIMEOUT 900)

if(WIN32)
    target_link_libraries(apex-rush PRIVATE winmm)
    # nudge's solver uses large stack frames (upstream builds with 8 MB too).
    # LINKER: prefix maps to -Wl, for the clang driver, bare flag for MSVC.
    target_link_options(apex-rush PRIVATE "LINKER:/STACK:8388608")
endif()

# --- Web (Emscripten) ---------------------------------------------------------
# The game loop is written in blocking form; ASYNCIFY lets it yield to the
# browser every frame (raylib sleeps inside WaitTime at the target FPS).
# Filesystem is in-memory MEMFS, so config.ini persists per tab session only.
if(EMSCRIPTEN)
    target_link_options(apex-rush PRIVATE
        -sASYNCIFY
        -sASYNCIFY_STACK_SIZE=65536
        -sALLOW_MEMORY_GROWTH=1
        -sFORCE_FILESYSTEM=1
        -sEXIT_RUNTIME=1
        --shell-file ${raylib_SOURCE_DIR}/src/minshell.html
        --preload-file ${CMAKE_SOURCE_DIR}/assets/maps@/assets/maps
    )
    # .html suffix makes emcc emit the shell page next to js/wasm.
    set_target_properties(apex-rush PROPERTIES SUFFIX ".html")
endif()
