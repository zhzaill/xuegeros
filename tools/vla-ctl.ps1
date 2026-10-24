# vla-ctl.ps1 — start/stop/status for the XuegerOS NoMaD service (port 18300).
# Run on DESKTOP-6PVSJFV. Uses the ComfyUI venv interpreter read-only.
#
# NOTE: the service is started via Win32_Process.Create so it is parented
# OUTSIDE the caller's session. Processes started directly over the SSH
# session are torn down when plink disconnects (verified 2026-10-24: the
# service died silently right after the health check answered).
param(
    [Parameter(Position = 0)][ValidateSet('start', 'stop', 'status', 'log')]
    [string]$Action = 'status'
)

$ErrorActionPreference = 'SilentlyContinue'
$python  = 'E:\ComfyUI\venv\Scripts\python.exe'
$dir     = 'E:\vla\nomad'
$script  = "$dir\vla_server.py"
$stdout  = "$dir\service-out.log"
$procFile = "$dir\service.pid"

function Get-VlaProc {
    if (Test-Path $procFile) {
        $pid_ = (Get-Content $procFile -First 1) -as [int]
        if ($pid_) {
            $p = Get-Process -Id $pid_ -ErrorAction SilentlyContinue
            if ($p) { return $p }
            # pid file stale: locate by command line as fallback
            $wmi = Get-CimInstance Win32_Process -Filter "Name='python.exe'" |
                Where-Object { $_.CommandLine -like "*vla_server.py*" }
            if ($wmi) { return (Get-Process -Id $wmi[0].ProcessId) }
        }
    }
    return $null
}

switch ($Action) {
    'start' {
        $existing = Get-VlaProc
        if ($existing) { "already running pid $($existing.Id)"; exit 0 }
        if (-not (Test-Path $python)) { "python not found: $python"; exit 1 }
        if (-not (Test-Path $script)) { "script not found: $script"; exit 1 }

        $cmdLine = "cmd /c `"`"$python`" `"$script`" > `"$stdout`" 2>&1`""
        $result = ([wmiclass]'\\.\root\cimv2:Win32_Process').Create($cmdLine, $dir, $null)
        if ($result.ReturnValue -ne 0) { "Win32_Process.Create failed: $($result.ReturnValue)"; exit 1 }
        $result.ProcessId | Out-File -Encoding ascii $procFile

        $healthy = $false
        foreach ($i in 1..10) {
            Start-Sleep -Seconds 2
            $h = & curl.exe -s --max-time 5 "http://127.0.0.1:18300/health"
            if ($h -and $h -match '"ok":\s*true') { $healthy = $true; break }
        }
        if ($healthy) {
            $loaded = if ($h -match '"loaded":\s*(true|false)') { $Matches[1] } else { '?' }
            "started (launcher pid $($result.ProcessId)); service healthy; weights loaded: $loaded"
        } else {
            "launcher started (pid $($result.ProcessId)) but /health not answering; check $stdout"
        }
    }
    'stop' {
        $p = Get-VlaProc
        if ($p) {
            taskkill /PID $p.Id /T /F | Out-Null
            "stopped pid $($p.Id)"
        } else { "not running" }
        Remove-Item $procFile -Force
    }
    'status' {
        $p = Get-VlaProc
        if ($p) { "process: pid $($p.Id) running" } else { "process: not running" }
        $h = & curl.exe -s --max-time 5 "http://127.0.0.1:18300/health"
        if ($h) { $h } else { "health: no answer on 18300" }
    }
    'log' {
        Get-Content "$dir\vla_server.log" -Tail 40
    }
}
