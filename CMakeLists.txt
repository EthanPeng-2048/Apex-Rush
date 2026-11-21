cmake_minimum_required(VERSION 3.20)
project(ApexRushOnline CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    add_compile_options(/W3 /permissive-)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

# --- Third-party: raylib (render/input/window) -------------------------------
include(FetchContent)
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

# --- Third-party: qu3e (collision) / enet (networking) -----------------------
add_subdirectory(third_party/qu3e)
add_subdirectory(third_party/enet)

# --- Game ---------------------------------------------------------------------
add_executable(apex-rush
    src/main.cpp
    src/core/Config.cpp
    src/physics/Qu3eWorld.cpp
    src/render/RaylibBackend.cpp
    src/render/NullBackend.cpp
    src/vehicle/TireModel.cpp
    src/vehicle/Vehicle.cpp
    src/game/MapData.cpp
    src/game/Track.cpp
    src/game/Game.cpp
)

target_include_directories(apex-rush PRIVATE src)
target_link_libraries(apex-rush PRIVATE raylib qu3e enet)

if(MSVC)
    target_link_libraries(apex-rush PRIVATE winmm)
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
