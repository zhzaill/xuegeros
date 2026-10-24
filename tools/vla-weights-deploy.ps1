# vla-weights-deploy.ps1 — one-command nomad.pth deployment pipeline.
#
# Usage (local machine, PowerShell or via scheduled watcher):
#   powershell -File tools\vla-weights-deploy.ps1 -Path C:\Users\zhzaill\Downloads\nomad.pth
#   powershell -File tools\vla-weights-deploy.ps1            # auto-search drop dirs
#
# Pipeline: validate file -> pscp to E:\vla\nomad\weights\ -> restart service ->
#           poll /health until weights.loaded -> assert missing_keys == 0 ->
#           run smoke test -> write marker tmp\vla-weights-deployed.json
# Idempotent: exits 0 immediately if the marker exists and health still loaded.
param([string]$Path = '')

$ErrorActionPreference = 'Stop'
$plink = 'C:\Program Files\PuTTY\plink.exe'
$pscp  = 'C:\Program Files\PuTTY\pscp.exe'
$fp    = 'SHA256:mi6ruODDjDlZBosOGrVIAJ4UFTIGPYRP11eH4cNLh+M'
$host_ = 'zhzaill@192.168.1.166'
$proj  = 'D:\11\Projects\xuegeros'
$marker = "$proj\tmp\vla-weights-deployed.json"
# Server password is NOT stored here. Set $env:XUEGEROS_SERVER_PW before use,
# or put a PuTTY session/hostkey-cached auth in place. This file is published.
$pw = $env:XUEGEROS_SERVER_PW
if (-not $pw) { "set XUEGEROS_SERVER_PW first (not committed anywhere)"; exit 1 }

function Remote([string]$script) {
    $b64 = [Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($script))
    & $plink -batch -ssh -hostkey $fp -pw $pw $host_ "powershell -NoProfile -EncodedCommand $b64"
}

# --- 0. already deployed? ----------------------------------------------------
if (Test-Path $marker) {
    $h = Remote 'Invoke-RestMethod http://127.0.0.1:18300/health -TimeoutSec 8' | ConvertFrom-Json
    if ($h.weights.loaded) { "already deployed (marker present, health loaded). Nothing to do."; exit 0 }
}

# --- 1. locate the file ------------------------------------------------------
if (-not $Path) {
    $drops = @(
        "$env:USERPROFILE\Downloads",
        'D:\11\Inbox\from-downloads',
        "$proj\tmp",
        "$env:USERPROFILE\Desktop"
    )
    $cand = foreach ($d in $drops) {
        Get-ChildItem $d -Filter *.pth -File -ErrorAction SilentlyContinue |
            Where-Object { $_.Name -match 'nomad' -and $_.Length -gt 1MB }
    }
    $f = $cand | Sort-Object LastWriteTime -Descending | Select-Object -First 1
    if (-not $f) { "no nomad.pth found in drop dirs"; exit 2 }
    $Path = $f.FullName
}
if (-not (Test-Path $Path)) { "file not found: $Path"; exit 2 }
$len = (Get-Item $Path).Length
if ($len -lt 1MB -or $len -gt 2GB) { "implausible size: $len bytes"; exit 2 }
$sha = (Get-FileHash $Path -Algorithm SHA256).Hash
"deploying $Path ($len bytes, sha256 $sha)"

# --- 2. upload ---------------------------------------------------------------
& $pscp -batch -hostkey $fp -pw $pw $Path "${host_}:E:/vla/nomad/weights/nomad.pth.incoming"
if ($LASTEXITCODE -ne 0) { "pscp failed"; exit 3 }

# --- 3. swap + restart + wait for health -------------------------------------
$out = Remote @'
[Console]::OutputEncoding=[Text.Encoding]::UTF8
$ErrorActionPreference='SilentlyContinue'
Move-Item E:\vla\nomad\weights\nomad.pth.incoming E:\vla\nomad\weights\nomad.pth -Force
powershell -NoProfile -ExecutionPolicy Bypass -File E:\vla\vla-ctl.ps1 stop
powershell -NoProfile -ExecutionPolicy Bypass -File E:\vla\vla-ctl.ps1 start
foreach ($i in 1..15) {
  Start-Sleep 2
  $h = & curl.exe -s --max-time 5 http://127.0.0.1:18300/health
  if ($h -match '"loaded":\s*true') { break }
}
$h
'@
$health = ($out | Where-Object { $_ -match '^\{' }) | ConvertFrom-Json
if (-not $health.weights.loaded) {
    "service did not load weights: " + ($out -join ' | ')
    exit 4
}
"w: loaded=true sha256=$($health.weights.sha256) size=$($health.weights.size_bytes) missing_keys=$($health.weights.missing_keys) unexpected_keys=$($health.weights.unexpected_keys)"
if ($health.weights.sha256 -ne $sha.ToLower()) { "SHA MISMATCH local vs server"; exit 5 }
if ([int]$health.weights.missing_keys -ne 0) { "missing_keys != 0 — checkpoint does not fit the model"; exit 6 }

# --- 4. smoke test ------------------------------------------------------------
$smoke = Remote 'E:\ComfyUI\venv\Scripts\python.exe E:\vla\nomad\vla_smoke_test.py 2>&1 | Select-Object -Last 5'
$smoke

# --- 5. marker ----------------------------------------------------------------
@{ path = $Path; size = $len; sha256 = $sha; deployed_at = (Get-Date -Format s) } |
    ConvertTo-Json | Out-File -Encoding utf8 $marker
"DEPLOYMENT COMPLETE"
