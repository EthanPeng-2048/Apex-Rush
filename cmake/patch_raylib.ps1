# Patches applied to the fetched raylib 5.5 sources.
# 1) rcore_desktop_glfw.c: raylib aborts window init when the created window can
#    not be mapped to a monitor (locked/remote sessions report zero monitors).
#    Skip centering instead of terminating GLFW.
# 2) rcore.c: InitWindow() ignored InitPlatform() failure and crashed in rlglInit
#    with no GL context. Bail out cleanly so the app can report the error.
# Usage (cwd = raylib source root): powershell -File patch_raylib.ps1 .
param([string]$SourceDir = ".")

function Patch-File([string]$RelPath, [string]$Old, [string]$New) {
    $f = Join-Path $SourceDir $RelPath
    if (-not (Test-Path $f)) { Write-Host "raylib source not found: $f"; return $false }
    $t = Get-Content -Raw $f
    # Normalize CRLF -> LF so here-string patterns match regardless of checkout style.
    $tLf = $t -replace "`r`n", "`n"
    $oldLf = $Old -replace "`r`n", "`n"
    $newLf = $New -replace "`r`n", "`n"
    if ($tLf.Contains($newLf)) { return $true }   # already patched
    if (-not $tLf.Contains($oldLf)) { Write-Host "pattern not found in $RelPath"; return $false }
    Set-Content -Path $f -Value ($tLf.Replace($oldLf, $newLf)) -NoNewline
    Write-Host "patched $RelPath"
    return $true
}

$ok = $true

$ok = (Patch-File "src/platforms/rcore_desktop_glfw.c" @'
            // The monitor for the window-manager-created window can not be determined, so it can not be centered.
            glfwTerminate();
            TRACELOG(LOG_WARNING, "GLFW: Failed to determine Monitor to center Window");
            return -1;
'@ @'
            // The monitor for the window-manager-created window can not be determined, so it can not be centered.
            // Apex Rush patch: locked/headless sessions report zero monitors; skip centering instead of aborting.
            TRACELOG(LOG_WARNING, "GLFW: Failed to determine Monitor to center Window");
'@) -and $ok

$ok = (Patch-File "src/platforms/rcore_desktop_glfw.c" @'
        glfwGetMonitorWorkarea(monitor, &monitorX, &monitorY, &monitorWidth, &monitorHeight);
'@ @'
        if (monitor) glfwGetMonitorWorkarea(monitor, &monitorX, &monitorY, &monitorWidth, &monitorHeight);
'@) -and $ok

$ok = (Patch-File "src/platforms/rcore_desktop_glfw.c" @'
        SetWindowPosition(posX, posY);
'@ @'
        if (monitor) SetWindowPosition(posX, posY);
'@) -and $ok

$ok = (Patch-File "src/rcore.c" @'
    InitPlatform();
'@ @'
    // Apex Rush patch: bail out cleanly when the platform init fails (e.g. no display)
    if (InitPlatform() != 0) return;
'@) -and $ok

if ($ok) { Write-Host "raylib patch complete"; exit 0 } else { exit 1 }
