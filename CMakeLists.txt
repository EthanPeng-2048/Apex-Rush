cmake_minimum_required(VERSION 3.28)
project(ApexRushOnline CXX C)

# C++23: rush-engine / exstl need std::expected (and named modules).
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

# --- Third-party: enet (networking) ------------------------------------------
add_subdirectory(third_party/enet)

# --- Rush Engine (git submodule: modules + physics + render) ------------------
# Provide raylib before the engine so the raylib backend compiles in.
set(RUSH_ENGINE_RAYLIB_TARGET raylib CACHE STRING "" FORCE)
add_subdirectory(rush-engine)

# --- Game ---------------------------------------------------------------------
add_executable(apex-rush
    src/main.cpp
    src/audio/MiniaudioAudio.cpp
    src/game/Track.cpp
    src/game/Game.cpp
)

target_include_directories(apex-rush PRIVATE src third_party/miniaudio)
target_link_libraries(apex-rush PRIVATE raylib enet rush::engine)

# miniaudio is a ~4 MB single-header TU; the audio mixer needs no -O3 and the
# full optimization costs minutes of compile time.
set_property(SOURCE src/audio/MiniaudioAudio.cpp APPEND PROPERTY COMPILE_OPTIONS
             "$<$<CXX_COMPILER_ID:Clang>:-O1>" "$<$<CXX_COMPILER_ID:MSVC>:/O1>")

# --- Developer tools (mapconv: CSV -> .rushmap) ------------------------------
# OFF by default; debug-tools preset turns this on. Release never builds tools.
option(APEX_BUILD_TOOLS "Build developer tools (mapconv)" OFF)
if(APEX_BUILD_TOOLS)
    add_subdirectory(tools/mapconv)
endif()

# --- Tests (ctest --preset release) ------------------------------------------
enable_testing()
add_test(NAME drive-test COMMAND apex-rush --drive-test)
set_tests_properties(drive-test PROPERTIES TIMEOUT 900)

if(WIN32)
    target_link_libraries(apex-rush PRIVATE winmm)
    # nudge's solver uses large stack frames (upstream builds with 8 MB too).
    target_link_options(apex-rush PRIVATE "LINKER:/STACK:8388608")
endif()

# --- Web (Emscripten) ---------------------------------------------------------
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
    set_target_properties(apex-rush PROPERTIES SUFFIX ".html")
endif()
