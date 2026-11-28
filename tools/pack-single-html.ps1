# pack-single-html.ps1 — combine Emscripten SINGLE_FILE html + .data (+ FileSaver) into one offline HTML.
# Does not modify game sources; reads build-web outputs and writes dist/.
param(
  [string]$BuildDir = (Join-Path $PSScriptRoot "..\build-web"),
  [string]$OutFile  = (Join-Path $PSScriptRoot "..\dist\apex-rush-single.html"),
  [string]$FileSaverPath = (Join-Path $PSScriptRoot "..\dist\FileSaver.min.js")
)
$ErrorActionPreference = "Stop"
$htmlPath = Join-Path $BuildDir "apex-rush.html"
$dataPath = Join-Path $BuildDir "apex-rush.data"
if (-not (Test-Path $htmlPath)) { throw "missing $htmlPath" }
if (-not (Test-Path $dataPath)) { throw "missing $dataPath" }

# dist/ is also where the FileSaver cache lands, so it has to exist before the
# first download — a clean checkout has no dist/ at all.
$outDir = Split-Path -Parent $OutFile
if ($outDir -and -not (Test-Path $outDir)) { New-Item -ItemType Directory -Force -Path $outDir | Out-Null }
$fsDir = Split-Path -Parent $FileSaverPath
if ($fsDir -and -not (Test-Path $fsDir)) { New-Item -ItemType Directory -Force -Path $fsDir | Out-Null }

$html = [System.IO.File]::ReadAllText($htmlPath)
$dataB64 = [Convert]::ToBase64String([System.IO.File]::ReadAllBytes($dataPath))

if (-not (Test-Path $FileSaverPath)) {
  $resp = Invoke-WebRequest -Uri "https://cdn.jsdelivr.net/gh/eligrey/FileSaver.js/dist/FileSaver.min.js" -UseBasicParsing
  [System.IO.File]::WriteAllText($FileSaverPath, $resp.Content, [System.Text.UTF8Encoding]::new($false))
}
$fsB64 = [Convert]::ToBase64String([System.IO.File]::ReadAllBytes($FileSaverPath))

$inject = @"
<script>
// apex-rush single-html packer: embed apex-rush.data (KVRec maps) + FileSaver
(function () {
  var dataB64 = "$dataB64";
  var fsB64 = "$fsB64";
  function b64ToBuf(b64) {
    var bin = atob(b64);
    var u8 = new Uint8Array(bin.length);
    for (var i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i);
    return u8.buffer;
  }
  function b64ToBin(b64) {
    var bin = atob(b64);
    var u8 = new Uint8Array(bin.length);
    for (var i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i);
    return u8;
  }
  try {
    var fsSrc = new TextDecoder().decode(b64ToBin(fsB64));
    (0, eval)(fsSrc);
  } catch (e) {}
  var dataBuf = null;
  function ensureBuf() {
    if (!dataBuf) dataBuf = b64ToBuf(dataB64);
    return dataBuf;
  }
  var g = (typeof globalThis !== "undefined") ? globalThis : window;
  g.Module = g.Module || {};
  g.Module.getPreloadedPackage = function (name, size) {
    if (name && name.indexOf("apex-rush.data") === -1) return null;
    return ensureBuf();
  };
})();
</script>
"@

$marker = '<script id=mainScript>'
$idx = $html.IndexOf($marker)
if ($idx -lt 0) {
  $marker = '<script id="mainScript">'
  $idx = $html.IndexOf($marker)
}
if ($idx -lt 0) { throw "mainScript marker not found; cannot inject data package" }

# strip previous packer block
$start = $html.IndexOf("<!--apex-pack-start-->")
$end = $html.IndexOf("<!--apex-pack-end-->")
if ($start -ge 0 -and $end -gt $start) {
  $html = $html.Substring(0, $start) + $html.Substring($end + "<!--apex-pack-end-->".Length)
  $idx = $html.IndexOf($marker)
  if ($idx -lt 0) {
    $marker = '<script id="mainScript">'
    $idx = $html.IndexOf($marker)
  }
  if ($idx -lt 0) { throw "mainScript marker not found after re-inject" }
}

$block = "<!--apex-pack-start-->" + $inject + "<!--apex-pack-end-->"
$html = $html.Substring(0, $idx) + $block + $html.Substring($idx)

# Neutralize CDN FileSaver script so offline never hits network (injected copy already ran).
$html = $html -replace '<script src=https://cdn\.jsdelivr\.net/gh/eligrey/FileSaver\.js/dist/FileSaver\.min\.js></script>', '<!-- FileSaver inlined -->'

$outDir = Split-Path -Parent $OutFile
if ($outDir -and -not (Test-Path $outDir)) { New-Item -ItemType Directory -Force -Path $outDir | Out-Null }
[System.IO.File]::WriteAllText($OutFile, $html, [System.Text.UTF8Encoding]::new($false))
$sz = (Get-Item $OutFile).Length
Write-Host "Wrote $OutFile ($sz bytes)"
