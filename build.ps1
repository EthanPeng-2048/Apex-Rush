# Build script: activates VS 2026 developer shell, configures + builds with CMake.
param(
    [ValidateSet("Debug", "Release", "RelWithDebInfo")]
    [string]$Config = "RelWithDebInfo",
    [switch]$Clean,
    [switch]$ConfigureOnly
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $root

$vsRoot = "D:\Program Files\Microsoft Visual Studio\18\Community"
Import-Module "$vsRoot\Common7\Tools\Microsoft.VisualStudio.DevShell.dll"
Enter-VsDevShell -VsInstallPath $vsRoot -SkipAutomaticLocation -DevCmdArguments '-arch=x64 -host_arch=x64' | Out-Null

$buildDir = Join-Path $root "build"

if ($Clean -and (Test-Path $buildDir)) {
    Remove-Item $buildDir -Recurse -Force
}

if (-not (Test-Path (Join-Path $buildDir "CMakeCache.txt"))) {
    Write-Host "== Configuring ==" -ForegroundColor Cyan
    cmake -S $root -B $buildDir -G Ninja "-DCMAKE_BUILD_TYPE=$Config"
    if ($LASTEXITCODE -ne 0) { exit 1 }
}

if ($ConfigureOnly) { exit 0 }

Write-Host "== Building ($Config) ==" -ForegroundColor Cyan
cmake --build $buildDir --config $Config
if ($LASTEXITCODE -ne 0) { exit 1 }

Write-Host "== Done: $buildDir\apex-rush.exe ==" -ForegroundColor Green
