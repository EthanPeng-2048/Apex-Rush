# Patches applied to the fetched raylib 5.5 sources. CMake-script port of the
# former patch_raylib.ps1 so the fetch works identically on Windows and Linux
# (no PowerShell dependency).
#
# 1) rcore_desktop_glfw.c: raylib aborts window init when the created window can
#    not be mapped to a monitor (locked/remote/headless sessions report zero
#    monitors). Skip centering instead of terminating GLFW.
# 2) rcore.c: InitWindow() ignored InitPlatform() failure and crashed in rlglInit
#    with no GL context. Bail out cleanly so the app can report the error.
#
# Usage: cmake -DAPEX_RAYLIB_DIR=<absolute raylib source root> -P patch_raylib.cmake
# FetchContent passes its own <SOURCE_DIR>, which is always absolute; a relative
# path would be resolved against whatever directory the script happens to run
# from, so reject it instead of patching the wrong tree.

cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED APEX_RAYLIB_DIR OR APEX_RAYLIB_DIR STREQUAL "")
    message(FATAL_ERROR "APEX_RAYLIB_DIR is required")
endif()
if(NOT IS_ABSOLUTE "${APEX_RAYLIB_DIR}")
    message(FATAL_ERROR "APEX_RAYLIB_DIR must be absolute, got '${APEX_RAYLIB_DIR}'")
endif()

# patch_file(<rel path> <old snippet> <new snippet>)
#
# Idempotent: no-op when <new> is already present (re-running over an
# already-patched tree, e.g. the seeded .deps/raylib-src). CRLF is normalized so
# patterns match regardless of checkout style. A missing file or snippet is a
# hard error: a silently unpatched raylib changes runtime behaviour (window init
# aborts on headless sessions) and is very hard to trace back here.
function(patch_file rel old new)
    set(path "${APEX_RAYLIB_DIR}/${rel}")
    if(NOT EXISTS "${path}")
        message(FATAL_ERROR "raylib source not found: ${path}")
    endif()

    file(READ "${path}" text)
    string(REPLACE "\r\n" "\n" text "${text}")
    string(REPLACE "\r\n" "\n" old "${old}")
    string(REPLACE "\r\n" "\n" new "${new}")

    string(FIND "${text}" "${new}" done_at)
    if(done_at GREATER -1)
        return()  # already patched
    endif()

    string(FIND "${text}" "${old}" at)
    if(at EQUAL -1)
        message(FATAL_ERROR "patch pattern not found in ${rel}")
    endif()

    string(REPLACE "${old}" "${new}" text "${text}")
    file(WRITE "${path}" "${text}")
    message(STATUS "patched ${rel}")
endfunction()

patch_file("src/platforms/rcore_desktop_glfw.c"
[==[
            // The monitor for the window-manager-created window can not be determined, so it can not be centered.
            glfwTerminate();
            TRACELOG(LOG_WARNING, "GLFW: Failed to determine Monitor to center Window");
            return -1;
]==]
[==[
            // The monitor for the window-manager-created window can not be determined, so it can not be centered.
            // Apex Rush patch: locked/headless sessions report zero monitors; skip centering instead of aborting.
            TRACELOG(LOG_WARNING, "GLFW: Failed to determine Monitor to center Window");
]==])

patch_file("src/platforms/rcore_desktop_glfw.c"
[==[
        glfwGetMonitorWorkarea(monitor, &monitorX, &monitorY, &monitorWidth, &monitorHeight);
]==]
[==[
        if (monitor) glfwGetMonitorWorkarea(monitor, &monitorX, &monitorY, &monitorWidth, &monitorHeight);
]==])

patch_file("src/platforms/rcore_desktop_glfw.c"
[==[
        SetWindowPosition(posX, posY);
]==]
[==[
        if (monitor) SetWindowPosition(posX, posY);
]==])

patch_file("src/rcore.c"
[==[
    InitPlatform();
]==]
[==[
    // Apex Rush patch: bail out cleanly when the platform init fails (e.g. no display)
    if (InitPlatform() != 0) return;
]==])

message(STATUS "raylib patch complete (${APEX_RAYLIB_DIR})")
