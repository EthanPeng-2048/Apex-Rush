cmake_minimum_required(VERSION 3.28)
project(ApexRushOnline CXX C)

# C++23: rush-engine / exstl need std::expected (and named modules).
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_SCAN_FOR_MODULES ON)

set(APEX_GNU_LIKE OFF)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    set(APEX_GNU_LIKE ON)
endif()

# Is the target an x86-64 machine? (Gates every ISA flag below.)
set(APEX_X86_64 OFF)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64)$")
    set(APEX_X86_64 ON)
endif()

if(APEX_GNU_LIKE AND NOT EMSCRIPTEN)
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        add_compile_options(-O0)
    else()
        # Global -O3, plus ThinLTO (parallel LTO, much cheaper than full LTO).
        # Benchmarked faster than plain -O3 and than the MSVC baseline on this
        # codebase (docs/rpc-refactor.md).
        add_compile_options(-O3)
        # ThinLTO is only usable with a linker that implements it - lld. GNU ld
        # and gold both reject -flto=thin, so probe instead of assuming lld is
        # installed (it is not on a stock Ubuntu box). Override with
        # APEX_THIN_LTO=OFF on toolchains where lld misbehaves.
        option(APEX_THIN_LTO "Compile with -flto=thin (requires lld)" ON)
        if(APEX_THIN_LTO)
            find_program(APEX_LD_LLD NAMES ld.lld)
            if(NOT APEX_LD_LLD AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
                # Debian/Ubuntu often install only a versioned binary
                # (ld.lld-20) and no `lld`/`ld.lld` symlink. Ask clang which
                # one it would use, and pass that path explicitly.
                execute_process(
                    COMMAND ${CMAKE_CXX_COMPILER} -print-prog-name=ld.lld
                    OUTPUT_VARIABLE APEX_LD_LLD
                    OUTPUT_STRIP_TRAILING_WHITESPACE
                    ERROR_QUIET)
                if(NOT IS_ABSOLUTE "${APEX_LD_LLD}" OR NOT EXISTS "${APEX_LD_LLD}")
                    set(APEX_LD_LLD "")
                endif()
            endif()
            if(APEX_LD_LLD)
                add_compile_options(-flto=thin)
                add_link_options(-flto=thin)
                if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
                    # clang accepts a full path for -fuse-ld, gcc only a name.
                    # MSVC-ABI clang makes CMake emit -fuse-ld=lld itself; the
                    # duplicate is harmless (clang takes the last one and both
                    # resolve to lld), and this explicit form is what makes
                    # ThinLTO work when only a versioned ld.lld-<ver> exists.
                    add_link_options(-fuse-ld=${APEX_LD_LLD})
                else()
                    add_link_options(-fuse-ld=lld)
                endif()
            else()
                message(STATUS "APEX_THIN_LTO: lld not found - building without LTO "
                               "(install lld or pass -DAPEX_THIN_LTO=OFF)")
            endif()
        endif()
    endif()
elseif(MSVC)
    add_compile_options(/W3 /permissive-)
endif()
add_compile_definitions(_CRT_SECURE_NO_WARNINGS)

# --- ISA baseline / AVX2 opt-in ----------------------------------------------
# Default target is the x86-64 baseline (SSE2), so the binary runs on every
# x86-64 CPU including AVX2 machines.
#
# APEX_NATIVE_ARCH=ON raises the baseline to AVX2 + FMA (x86-64-v3 on
# clang/gcc, /arch:AVX2 on MSVC). It never uses -march=native, so the result
# still runs on any AVX2-capable CPU (Intel Haswell 2013+, AMD Excavator 2015+)
# rather than only on the build machine. Nothing in the tree requires an ISA
# above AVX2 (no AVX-512 anywhere), so v3 is the hard ceiling.
#
# Default OFF: A/B on the dev machine showed x86-64-v3 is neutral on the
# collision kernel and ~10% SLOWER on the suspension ray path (see
# docs/rpc-optimization.md §4.3), so it is opt-in rather than default. Note it
# also makes the binary refuse to start on pre-AVX2 CPUs.
option(APEX_NATIVE_ARCH "Compile for x86-64-v3 / AVX2+FMA (requires AVX2 CPU)" OFF)
if(APEX_NATIVE_ARCH)
    if(NOT APEX_X86_64)
        message(WARNING "APEX_NATIVE_ARCH is x86-64 only - ignored on "
                        "'${CMAKE_SYSTEM_PROCESSOR}'")
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
        add_compile_options(/arch:AVX2)
    elseif(APEX_GNU_LIKE AND NOT EMSCRIPTEN)
        add_compile_options(-march=x86-64-v3)
    else()
        message(WARNING "APEX_NATIVE_ARCH has no flag for "
                        "'${CMAKE_CXX_COMPILER_ID}' - ignored")
    endif()
endif()

# --- Static CRT (portable build) ---------------------------------------------
# The default build links the runtime dynamically, so the exe needs
# MSVCP140.dll / VCRUNTIME140.dll from the VC++ redistributable - which is
# missing on freshly installed machines. APEX_STATIC_RUNTIME links libcmt /
# libcpmt instead, leaving only DLLs that Windows ships itself. Player-facing
# builds use it (preset release-static). It must cover every target, raylib and
# enet included: lld rejects mixed CRT flavors with LNK2038.
option(APEX_STATIC_RUNTIME "Link the C/C++ runtime statically (no VC++ redist on the target)" OFF)
if(APEX_STATIC_RUNTIME AND WIN32 AND NOT EMSCRIPTEN AND CMAKE_CXX_COMPILER_ID MATCHES "Clang|MSVC")
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
# raylib bundles GLFW with both X11 and Wayland enabled on Linux, and the
# Wayland half needs pkg-config + libwayland-dev + wayland-protocols at
# configure time. X11 only keeps the dependency list short (Wayland sessions
# still work through XWayland); pass -DAPEX_GLFW_WAYLAND=ON for a native
# Wayland build.
if(UNIX AND NOT APPLE AND NOT EMSCRIPTEN)
    option(APEX_GLFW_WAYLAND "Build raylib's GLFW with the Wayland backend too" OFF)
    set(GLFW_BUILD_WAYLAND ${APEX_GLFW_WAYLAND} CACHE BOOL "" FORCE)
    set(GLFW_BUILD_X11 ON CACHE BOOL "" FORCE)
endif()
# The patch is a CMake script, not PowerShell: the configure step has to work
# on Linux too (pwsh is not part of a stock distro).
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
    GIT_SHALLOW TRUE
    PATCH_COMMAND ${CMAKE_COMMAND} -DAPEX_RAYLIB_DIR=<SOURCE_DIR>
                  -P ${CMAKE_SOURCE_DIR}/cmake/patch_raylib.cmake
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
    # iphlpapi: discovery reads the interface table to derive subnet broadcasts.
    target_link_libraries(apex-rush PRIVATE winmm iphlpapi)
    # nudge's solver uses large stack frames (upstream builds with 8 MB too).
    target_link_options(apex-rush PRIVATE "LINKER:/STACK:8388608")
elseif(NOT EMSCRIPTEN)
    # miniaudio dlopen()s its backends (ALSA/PulseAudio) and the C++ thread pool
    # needs pthread (glibc < 2.34 keeps libpthread separate).
    find_package(Threads REQUIRED)
    target_link_libraries(apex-rush PRIVATE Threads::Threads ${CMAKE_DL_LIBS} m)
    # Linux keeps the 8 MB main-thread stack by default, so nudge's large
    # solver frames need no extra flag here.
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
        # The packer is still a PowerShell script (it base64s a multi-MB .data
        # blob, which CMake cannot do cheaply). Only this optional step needs
        # pwsh; the native desktop build no longer depends on PowerShell at all.
        find_program(POWERSHELL_EXE NAMES pwsh powershell)
        if(NOT POWERSHELL_EXE)
            message(FATAL_ERROR
                "APEX_PACK_SINGLE_HTML needs PowerShell (pwsh). Either install "
                "it or build the web preset without single-file packing.")
        endif()
        # -ExecutionPolicy is Windows PowerShell only; pwsh on Linux rejects it.
        if(WIN32)
            set(APEX_PS_EXEC_POLICY -ExecutionPolicy Bypass)
        endif()
        add_custom_command(TARGET apex-rush POST_BUILD
            COMMAND ${POWERSHELL_EXE} -NoProfile ${APEX_PS_EXEC_POLICY}
                -File ${CMAKE_SOURCE_DIR}/tools/pack-single-html.ps1
                -BuildDir ${CMAKE_BINARY_DIR}
                -OutFile ${CMAKE_SOURCE_DIR}/dist/apex-rush-single.html
            COMMENT "Packing dist/apex-rush-single.html"
            VERBATIM
        )
    endif()
endif()
