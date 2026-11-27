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
    # Target x86-64-v3 (AVX2 + FMA) on native desktop builds. Measured effect on
    # the collision kernel is nil (it is latency bound - see
    # docs/rpc-optimization.md section 4.3), but vehicle math, render transforms
    # and the audio mix do use these instructions and enabling them costs
    # nothing. Turn OFF when shipping to pre-2015 CPUs.
    # Default OFF: A/B on this machine showed x86-64-v3 is neutral on the
    # collision kernel and ~10% SLOWER on the suspension ray path (see
    # docs/rpc-optimization.md §4.3), so it is opt-in rather than default.
    option(APEX_NATIVE_ARCH "Compile for x86-64-v3 (AVX2/FMA)" OFF)
    if(APEX_NATIVE_ARCH AND CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
        add_compile_options(-march=x86-64-v3)
    endif()
elseif(MSVC)
    add_compile_options(/W3 /permissive-)
endif()
add_compile_definitions(_CRT_SECURE_NO_WARNINGS)

# --- Static CRT (portable build) ---------------------------------------------
# The default build links the runtime dynamically, so the exe needs
# MSVCP140.dll / VCRUNTIME140.dll from the VC++ redistributable - which is
# missing on freshly installed machines. APEX_STATIC_RUNTIME links libcmt /
# libcpmt instead, leaving only DLLs that Windows ships itself. Player-facing
# builds use it (preset release-static). It must cover every target, raylib and
# enet included: lld rejects mixed CRT flavors with LNK2038.
option(APEX_STATIC_RUNTIME "Link the C/C++ runtime statically (no VC++ redist on the target)" OFF)
if(APEX_STATIC_RUNTIME AND NOT EMSCRIPTEN AND CMAKE_CXX_COMPILER_ID MATCHES "Clang|MSVC")
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

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
    src/game/net/NetCars.cpp
    src/game/net/LanGame.cpp
    src/net/protocol/wire_codec.cpp
    src/net/identity.cpp
    src/net/roster.cpp
    src/net/transport.cpp
    src/net/discovery.cpp
    src/net/session.cpp
    src/net/net_selftest.cpp
)

target_include_directories(apex-rush PRIVATE src third_party/miniaudio rush-engine/src)  # rush-engine/src: TEMP probe
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

# L1 physics unit tests: pure rush-physics-core, no engine / no game.
add_executable(rpc-tests
    rush-engine/third_party/rush-physics-core/tests/test_compute_aabb.cpp
)
target_link_libraries(rpc-tests PRIVATE rush-physics-core)
add_test(NAME rpc-compute-aabb COMMAND rpc-tests)

# Pile-up stress harness: many multi-box car bodies spawned overlapping.
add_executable(rpc-pileup-tests
    rush-engine/third_party/rush-physics-core/tests/test_pileup.cpp
)
target_link_libraries(rpc-pileup-tests PRIVATE rush-physics-core)
add_test(NAME rpc-pileup COMMAND rpc-pileup-tests --cars 40 --seconds 2)

# Ray-index conformance: BVH vs static-grid+sweep vs brute force, plus injected
# factory (proves the rpcRayIndex seam). Fails on any closest-hit mismatch.
add_executable(rpc-ray-index-tests
    rush-engine/third_party/rush-physics-core/tests/test_ray_index.cpp
)
target_link_libraries(rpc-ray-index-tests PRIVATE rush-physics-core)
add_test(NAME rpc-ray-index COMMAND rpc-ray-index-tests)

if(WIN32)
    target_link_libraries(apex-rush PRIVATE winmm)
    # nudge's solver uses large stack frames (upstream builds with 8 MB too).
    target_link_options(apex-rush PRIVATE "LINKER:/STACK:8388608")
endif()

# --- Web (Emscripten) ---------------------------------------------------------
# APEX_PACK_SINGLE_HTML (web-single preset): -sSINGLE_FILE=1 embeds JS/WASM;
# post-build packs .data + FileSaver into dist/apex-rush-single.html.
option(APEX_PACK_SINGLE_HTML "After web link, pack offline single-file HTML" OFF)
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
    if(APEX_PACK_SINGLE_HTML)
        add_custom_command(TARGET apex-rush POST_BUILD
            COMMAND ${POWERSHELL_EXE} -NoProfile -ExecutionPolicy Bypass
                -File ${CMAKE_SOURCE_DIR}/tools/pack-single-html.ps1
                -BuildDir ${CMAKE_BINARY_DIR}
                -OutFile ${CMAKE_SOURCE_DIR}/dist/apex-rush-single.html
            COMMENT "Packing dist/apex-rush-single.html"
            VERBATIM
        )
    endif()
endif()
