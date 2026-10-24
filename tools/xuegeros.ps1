[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [ValidateSet(
        'help',
        'status',
        'start-vm',
        'ensure-network',
        'start-agent',
        'stop-agent',
        'agent-log',
        'start-bringup',
        'start-camera',
        'start-mapping',
        'start-nav2',
        'mapping-status',
        'battery',
        'check-forwarding',
        'snapshot'
    )]
    [string]$Action = 'help',

    [string]$CameraUrl = $env:XUEGEROS_CAMERA_URL,
    [switch]$Json
)

$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'

$cfg = [pscustomobject]@{
    ServerHost          = $env:XUEGEROS_SERVER_HOST
    ServerUser          = $env:XUEGEROS_SERVER_USER
    ServerKeyPath       = $env:XUEGEROS_SERVER_KEY
    # Pin Windows OpenSSH by default. Git Bash's MSYS ssh runs ProxyCommand via
    # sh -c, which eats the backslashes in unquoted key paths
    # (C:\Users\...\.ssh\id_loopros -> C:Users...sshid_loopros).
    SshPath             = $(if ($env:XUEGEROS_SSH_PATH) { $env:XUEGEROS_SSH_PATH } elseif (Test-Path 'C:\WINDOWS\System32\OpenSSH\ssh.exe') { 'C:\WINDOWS\System32\OpenSSH\ssh.exe' } else { 'ssh.exe' })
    VmrunPath           = $env:XUEGEROS_VMRUN_PATH
    VmxPath             = $env:XUEGEROS_VMX_PATH
    GuestIp             = $env:XUEGEROS_GUEST_IP
    GuestKeyPath        = $env:XUEGEROS_GUEST_KEY
    GuestRuntimeUser    = $env:XUEGEROS_GUEST_RUNTIME_USER
    GuestWorkspace      = $env:XUEGEROS_GUEST_WORKSPACE
    AgentBinary         = $env:XUEGEROS_AGENT_BINARY
    AgentLibraryPath    = $env:XUEGEROS_AGENT_LIBRARY_PATH
    AgentLog            = $env:XUEGEROS_AGENT_LOG
}

function Show-Help {
    @'
xuegeros.ps1 - Xuegeros VM, ROS2, camera, and mapping operations

Required environment variables:
  XUEGEROS_SERVER_HOST
  XUEGEROS_SERVER_USER
  XUEGEROS_SERVER_KEY
  XUEGEROS_VMRUN_PATH
  XUEGEROS_VMX_PATH
  XUEGEROS_GUEST_IP
  XUEGEROS_GUEST_KEY_PATH
  XUEGEROS_GUEST_RUNTIME_USER
  XUEGEROS_GUEST_WORKSPACE
  XUEGEROS_AGENT_BINARY
  XUEGEROS_AGENT_LIBRARY_PATH
  XUEGEROS_AGENT_LOG

Optional:
  XUEGEROS_SSH_PATH
  XUEGEROS_CAMERA_URL

This CLI is a maintenance and observation tool. It cannot publish /cmd_vel.
Physical motion belongs to the separate Tool Gateway.

Actions:
  help                 Show this help.
  status               Show VM, Agent, network, process, and log status.
  start-vm             Start the VM as a detached Windows process.
  ensure-network       Bring up ens33 and obtain DHCP if needed.
  start-agent          Start micro-ROS Agent on UDP 8888.
  stop-agent           Stop the micro-ROS Agent process.
  agent-log            Print the last 200 Agent log lines.
  start-bringup        Start xuegecar_bringup.
  start-camera         Start xuegecar_camera with the configured camera URL.
  start-mapping        Start slam_gmapping only.
  start-nav2           Start pure Navigation2 without Gmapping.
  mapping-status       Inspect ROS topics, /scan rate, /map, and TF.
  check-forwarding     Check host UDP listeners on 8888 and 8889.
  snapshot             Capture one JPEG frame from the HTTP MJPEG stream.

Examples:
  . .\config\xuegeros.env.ps1
  .\tools\xuegeros.ps1 status
  .\tools\xuegeros.ps1 start-vm
  .\tools\xuegeros.ps1 ensure-network
  .\tools\xuegeros.ps1 start-agent
  .\tools\xuegeros.ps1 start-camera
  .\tools\xuegeros.ps1 mapping-status
  .\tools\xuegeros.ps1 snapshot
'@
}

function Assert-RuntimeConfiguration {
    param([switch]$RequireGuestAccess)

    $required = [ordered]@{
        XUEGEROS_SERVER_HOST       = $cfg.ServerHost
        XUEGEROS_SERVER_USER       = $cfg.ServerUser
        XUEGEROS_SERVER_KEY        = $cfg.ServerKeyPath
        XUEGEROS_VMRUN_PATH        = $cfg.VmrunPath
        XUEGEROS_VMX_PATH          = $cfg.VmxPath
    }
    if ($RequireGuestAccess) {
        $required.XUEGEROS_GUEST_IP = $cfg.GuestIp
        $required.XUEGEROS_GUEST_KEY = $cfg.GuestKeyPath
        $required.XUEGEROS_GUEST_RUNTIME_USER = $cfg.GuestRuntimeUser
        $required.XUEGEROS_GUEST_WORKSPACE = $cfg.GuestWorkspace
        $required.XUEGEROS_AGENT_BINARY = $cfg.AgentBinary
        $required.XUEGEROS_AGENT_LIBRARY_PATH = $cfg.AgentLibraryPath
        $required.XUEGEROS_AGENT_LOG = $cfg.AgentLog
    }

    $missing = @($required.GetEnumerator() | Where-Object { -not $_.Value } | ForEach-Object { $_.Key })
    if ($missing.Count -gt 0) {
        throw "Missing configuration: $($missing -join ', '). Dot-source config\xuegeros.env.ps1 first."
    }

    if (-not (Test-Path -LiteralPath $cfg.ServerKeyPath)) {
        throw "Server SSH key not found: $($cfg.ServerKeyPath)"
    }
    if ($RequireGuestAccess -and -not (Test-Path -LiteralPath $cfg.GuestKeyPath)) {
        throw "Guest SSH key not found: $($cfg.GuestKeyPath)"
    }
}

function Expand-GuestTemplate {
    param([Parameter(Mandatory)][string]$Template)

    $result = $Template
    $result = $result.Replace('{{WORKSPACE}}', $cfg.GuestWorkspace)
    $result = $result.Replace('{{RUNTIME_USER}}', $cfg.GuestRuntimeUser)
    $result = $result.Replace('{{AGENT_BINARY}}', $cfg.AgentBinary)
    $result = $result.Replace('{{AGENT_LIBRARY_PATH}}', $cfg.AgentLibraryPath)
    $result = $result.Replace('{{AGENT_LOG}}', $cfg.AgentLog)
    return $result
}

function Invoke-RemotePowerShell {
    param(
        [Parameter(Mandatory)][string]$Script,
        [switch]$AllowNonZeroExit
    )

    $encoded = [Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($Script))
    $remoteCommand = "powershell -NoProfile -EncodedCommand $encoded"
    $sshArgs = @(
        '-n',
        '-o', 'BatchMode=yes',
        '-o', 'ConnectTimeout=8',
        '-o', 'StrictHostKeyChecking=yes',
        '-i', $cfg.ServerKeyPath,
        "$($cfg.ServerUser)@$($cfg.ServerHost)",
        $remoteCommand
    )

    $output = & $cfg.SshPath @sshArgs 2>&1
    $exitCode = $LASTEXITCODE
    if ($exitCode -ne 0 -and -not $AllowNonZeroExit) {
        throw "Remote command failed with exit code $exitCode`n$($output -join "`n")"
    }
    return ($output -join "`n").Trim()
}

function Expand-RemoteTemplate {
    param([Parameter(Mandatory)][string]$Template)

    $result = $Template
    $result = $result.Replace('{{VMRUN}}', $cfg.VmrunPath)
    $result = $result.Replace('{{VMX}}', $cfg.VmxPath)
    return $result
}

function ConvertFrom-RemoteBase64Json {
    param([Parameter(Mandatory)][string]$EncodedJson)

    $json = [Text.Encoding]::UTF8.GetString([Convert]::FromBase64String($EncodedJson.Trim()))
    return $json | ConvertFrom-Json
}

function Invoke-GuestBash {
    param([Parameter(Mandatory)][string]$Bash)

    $normalized = $Bash -replace "`r`n", "`n"
    $payload = "export LANG=C.UTF-8`nexport LC_ALL=C.UTF-8`nset +e`n$normalized`nexit 0`n"
    $payloadB64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($payload))

    $id = [guid]::NewGuid().ToString('N')
    $guestScript = "/tmp/xuegeros-$id.sh"
    $guestCommand = "base64 -d > $guestScript <<< '$payloadB64'; bash $guestScript"
    $proxyCommand = @(
        $cfg.SshPath,
        '-i', $cfg.ServerKeyPath,
        '-o', 'BatchMode=yes',
        '-o', 'StrictHostKeyChecking=yes',
        '-W', '%h:%p',
        "$($cfg.ServerUser)@$($cfg.ServerHost)"
    ) -join ' '
    $sshArgs = @(
        '-n',
        '-o', 'BatchMode=yes',
        '-o', 'ConnectTimeout=8',
        '-o', 'StrictHostKeyChecking=yes',
        '-o', "ProxyCommand=$proxyCommand",
        '-i', $cfg.GuestKeyPath,
        "root@$($cfg.GuestIp)",
        $guestCommand
    )
    $output = & $cfg.SshPath @sshArgs 2>&1
    $exitCode = $LASTEXITCODE
    if ($exitCode -ne 0) {
        throw "Guest command failed with exit code $exitCode`n$($output -join "`n")"
    }
    return ($output -join "`n").Trim()
}

function Get-VmStatus {
    $remote = Expand-RemoteTemplate @'
$ErrorActionPreference = 'SilentlyContinue'
$vmrun = '{{VMRUN}}'
$vmx = '{{VMX}}'
$result = [ordered]@{}
$vmList = @(& $vmrun list 2>&1)
$vmListText = ($vmList -join "`n")
$countMatch = [regex]::Match($vmListText, 'Total running VMs:\s*(\d+)')
$result.vmCount = if ($countMatch.Success) { [int]$countMatch.Groups[1].Value } else { $null }
$result.vmRunning = $result.vmCount -gt 0
$result.vmPath = $vmx
$result.toolsState = ((& $vmrun checkToolsState $vmx 2>&1) -join "`n").Trim()
$result.udpEndpoints = @(
    Get-NetUDPEndpoint -LocalPort 8888,8889 -ErrorAction SilentlyContinue |
        Select-Object LocalAddress, LocalPort, OwningProcess
)
$json = $result | ConvertTo-Json -Depth 6 -Compress
[Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($json))
'@
    $encodedJson = Invoke-RemotePowerShell -Script $remote
    return ConvertFrom-RemoteBase64Json -EncodedJson $encodedJson
}

function Start-VmDetached {
    $remote = Expand-RemoteTemplate @'
$ErrorActionPreference = 'Stop'
$vmrun = '{{VMRUN}}'
$vmx = '{{VMX}}'
$before = ((& $vmrun list 2>&1) -join "`n")
$countMatch = [regex]::Match($before, 'Total running VMs:\s*(\d+)')
$runningCount = if ($countMatch.Success) { [int]$countMatch.Groups[1].Value } else { 0 }
if ($runningCount -gt 0) {
    $started = $false
} else {
    $commandLine = '"' + $vmrun + '" -T ws start "' + $vmx + '" nogui'
    $created = Invoke-CimMethod -ClassName Win32_Process -MethodName Create -Arguments @{ CommandLine = $commandLine }
    if ($created.ReturnValue -ne 0) {
        throw "Win32_Process.Create failed with $($created.ReturnValue)"
    }
    $started = $true
    Start-Sleep -Seconds 25
}
$result = [ordered]@{
    started = $started
    vmRunning = $true
    vmPath = $vmx
    toolsState = ((& $vmrun checkToolsState $vmx 2>&1) -join "`n").Trim()
}
$json = $result | ConvertTo-Json -Compress
[Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($json))
'@
    $encodedJson = Invoke-RemotePowerShell -Script $remote
    return ConvertFrom-RemoteBase64Json -EncodedJson $encodedJson
}

function Get-GuestStatus {
    $script = Expand-GuestTemplate @'
echo "TIME"
date
echo "NETWORK"
ip -4 -o addr show ens33 2>&1 || true
echo "PROCESSES"
pgrep -af 'micro_ros_agent|slam_gmapping|gmapping|nav2|bringup|camsense|ldlidar|xuegecar_camera|http_video_publisher' || true
echo "UDP"
ss -lunp 2>&1 | grep -E '8888|8889' || true
echo "AGENT_LOG"
tail -n 20 {{AGENT_LOG}} 2>&1 | sed -r 's/\x1B\[[0-9;]*[mK]//g' || true
'@
    return Invoke-GuestBash -Bash $script
}

function Ensure-GuestNetwork {
    return Invoke-GuestBash @'
echo "BEFORE"
ip -4 -o addr show ens33 2>&1 || true
if ! ip -4 addr show ens33 2>/dev/null | grep -q 'inet '; then
    ip link set ens33 up
    dhclient ens33
fi
echo "AFTER"
ip -4 -o addr show ens33 2>&1 || true
ip route 2>&1 || true
ping -c 1 -W 2 192.168.253.2 2>&1 || true
'@
}

function Start-MicroRosAgent {
    $script = Expand-GuestTemplate @'
BIN='{{AGENT_BINARY}}'
AGENT_LOG='{{AGENT_LOG}}'
LIB='{{AGENT_LIBRARY_PATH}}'
mkdir -p "$(dirname "$AGENT_LOG")"
if pgrep -x micro_ros_agent >/dev/null; then
    echo "agent_already_running"
else
    runuser -u {{RUNTIME_USER}} -- nohup env LD_LIBRARY_PATH="$LIB" "$BIN" udp4 --port 8888 -v6 > "$AGENT_LOG" 2>&1 &
    sleep 4
fi
echo "PROCESSES"
pgrep -af micro_ros_agent || true
echo "UDP"
ss -lunp 2>&1 | grep 8888 || true
echo "LOG"
tail -n 30 "$AGENT_LOG" 2>&1 | sed -r 's/\x1B\[[0-9;]*[mK]//g' || true
'@
    return Invoke-GuestBash -Bash $script
}

function Stop-MicroRosAgent {
    return Invoke-GuestBash @'
pkill -x micro_ros_agent 2>/dev/null || true
sleep 1
echo "PROCESSES"
pgrep -af micro_ros_agent || true
echo "UDP"
ss -lunp 2>&1 | grep 8888 || true
'@
}

function Get-AgentLog {
    $script = Expand-GuestTemplate @'
tail -n 200 {{AGENT_LOG}} 2>&1 | sed -r 's/\x1B\[[0-9;]*[mK]//g' || true
'@
    return Invoke-GuestBash -Bash $script
}

function Start-GuestRosLaunch {
    param(
        [Parameter(Mandatory)][string]$Name,
        [Parameter(Mandatory)][string]$LaunchArguments
    )

    $launcherPath = "/tmp/xuegeros-$Name-launcher.sh"
    $logPath = "$($cfg.GuestWorkspace)/log/$Name.log"
    $launcherContent = @"
#!/bin/bash
source /opt/ros/humble/setup.bash
source $($cfg.GuestWorkspace)/install/setup.bash
cd $($cfg.GuestWorkspace)
exec ros2 launch $LaunchArguments
"@
    $launcherB64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes(($launcherContent -replace "`r`n", "`n")))

    $template = @'
mkdir -p '{{WORKSPACE}}/log'
if pgrep -f '[r]os2 launch {{LAUNCH_ARGUMENTS}}' >/dev/null; then
    echo "launch_already_running"
else
    base64 -d > '{{LAUNCHER_PATH}}' <<< '{{LAUNCHER_B64}}'
    chmod +x '{{LAUNCHER_PATH}}'
    chown {{RUNTIME_USER}}:{{RUNTIME_USER}} '{{LAUNCHER_PATH}}'
    runuser -u {{RUNTIME_USER}} -- nohup '{{LAUNCHER_PATH}}' > '{{LOG_PATH}}' 2>&1 &
    sleep 5
fi
echo "PROCESSES"
pgrep -af '{{LAUNCH_ARGUMENTS}}' || true
echo "LOG"
tail -n 60 '{{LOG_PATH}}' 2>&1 || true
'@
    $script = Expand-GuestTemplate($template)
    $script = $script.Replace('{{LAUNCH_ARGUMENTS}}', $LaunchArguments)
    $script = $script.Replace('{{LAUNCHER_PATH}}', $launcherPath)
    $script = $script.Replace('{{LAUNCHER_B64}}', $launcherB64)
    $script = $script.Replace('{{LOG_PATH}}', $logPath)

    return Invoke-GuestBash -Bash $script
}

function Get-MappingStatus {
    $checkerPath = "/tmp/xuegeros-mapping-status-$([guid]::NewGuid().ToString('N')).sh"
    $checkerContent = @"
#!/bin/bash
source /opt/ros/humble/setup.bash
source $($cfg.GuestWorkspace)/install/setup.bash
cd $($cfg.GuestWorkspace)
echo "NODES"
timeout 10 ros2 node list 2>&1 || true
echo "PROCESSES"
pgrep -af 'slam_gmapping|gmapping' || true
echo "TOPICS"
timeout 10 ros2 topic list 2>&1 || true
echo "SCAN_INFO"
timeout 8 ros2 topic info -v /scan 2>&1 || true
echo "SCAN_HZ"
timeout 8 ros2 topic hz /scan --window 20 2>&1 || true
echo "MAP_INFO"
timeout 8 ros2 topic info -v /map 2>&1 || true
echo "MAP_WIDTH"
timeout 8 ros2 topic echo /map --once --field info.width 2>&1 || true
echo "TF_INFO"
timeout 8 ros2 topic info -v /tf 2>&1 || true
"@
    $checkerB64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes(($checkerContent -replace "`r`n", "`n")))
    $template = @'
base64 -d > '{{CHECKER_PATH}}' <<< '{{CHECKER_B64}}'
chmod +x '{{CHECKER_PATH}}'
chown {{RUNTIME_USER}}:{{RUNTIME_USER}} '{{CHECKER_PATH}}'
runuser -u {{RUNTIME_USER}} -- bash '{{CHECKER_PATH}}'
'@
    $script = Expand-GuestTemplate($template)
    $script = $script.Replace('{{CHECKER_PATH}}', $checkerPath)
    $script = $script.Replace('{{CHECKER_B64}}', $checkerB64)
    $raw = Invoke-GuestBash -Bash $script
    [pscustomobject]@{
        observed_at = (Get-Date).ToUniversalTime().ToString('o')
        scan_present = ($raw -match '/scan' -and $raw -notmatch "Unknown topic '/scan'")
        scan_rate_observed = ($raw -match 'average rate:')
        map_present = ($raw -match '/map' -and $raw -notmatch "Unknown topic '/map'")
        tf_present = ($raw -match '/tf' -and $raw -notmatch "Unknown topic '/tf'")
        gmapping_running = ($raw -match 'slam_gmapping|gmapping')
        raw = $raw
    }
}

function Get-BatteryStatus {
    # /battery_state is published BEST_EFFORT, so a default RELIABLE subscriber
    # receives nothing at all. Subscribe explicitly with BEST_EFFORT.
    $probe = @'
import json, sys, time
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import BatteryState
except Exception as exc:
    print("XUEGEROS_BATTERY " + json.dumps({"ok": False, "error": "import_failed"}))
    sys.exit(0)
rclpy.init()
node = Node("xuegeros_battery_probe")
qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                 history=HistoryPolicy.KEEP_LAST)
seen = {}
node.create_subscription(BatteryState, "/battery_state",
                         lambda m: seen.update(v=float(m.voltage), p=float(m.percentage)), qos)
deadline = time.time() + 8.0
while time.time() < deadline and "v" not in seen:
    rclpy.spin_once(node, timeout_sec=0.2)
rclpy.shutdown()
if "v" in seen:
    print("XUEGEROS_BATTERY " + json.dumps({"ok": True, "voltage": round(seen["v"], 3),
                                            "percentage": round(seen["p"], 4)}))
else:
    print("XUEGEROS_BATTERY " + json.dumps({"ok": False, "error": "no_battery_message"}))
'@
    $probeB64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes(($probe -replace "`r`n", "`n")))
    $path = "/tmp/xuegeros-battery-$([guid]::NewGuid().ToString('N')).py"
    $template = @'
base64 -d > '{{PATH}}' <<< '{{B64}}'
chown {{RUNTIME_USER}}:{{RUNTIME_USER}} '{{PATH}}'
runuser -u {{RUNTIME_USER}} -- bash -c 'source /opt/ros/humble/setup.bash && python3 {{PATH}}'
rm -f '{{PATH}}'
'@
    $script = Expand-GuestTemplate($template)
    $script = $script.Replace('{{PATH}}', $path)
    $script = $script.Replace('{{B64}}', $probeB64)
    $raw = Invoke-GuestBash -Bash $script
    $parsed = $null
    foreach ($line in ($raw -split "`n")) {
        $trimmed = $line.Trim()
        if ($trimmed.StartsWith('XUEGEROS_BATTERY ')) {
            $json = $trimmed.Substring('XUEGEROS_BATTERY '.Length)
            try { $parsed = $json | ConvertFrom-Json } catch { $parsed = $null }
        }
    }
    if ($null -ne $parsed -and $parsed.ok) {
        return [pscustomobject]@{
            observed_at = (Get-Date).ToUniversalTime().ToString('o')
            available   = $true
            voltage     = $parsed.voltage
            percentage  = $parsed.percentage
        }
    }
    return [pscustomobject]@{
        observed_at = (Get-Date).ToUniversalTime().ToString('o')
        available   = $false
        error       = $(if ($parsed) { $parsed.error } else { 'probe_failed' })
    }
}

function Get-ForwardingStatus {
    $remote = Expand-RemoteTemplate @'
$ErrorActionPreference = 'SilentlyContinue'
$result = [ordered]@{}
$result.udpEndpoints = @(
    Get-NetUDPEndpoint -LocalPort 8888,8889 |
        Select-Object LocalAddress, LocalPort, OwningProcess
)
$result.firewallRule = Get-NetFirewallRule -DisplayName 'Codex micro-ROS Agent UDP' |
    Select-Object DisplayName, Enabled, Direction, Action, Profile
$json = $result | ConvertTo-Json -Depth 6 -Compress
[Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($json))
'@
    $encodedJson = Invoke-RemotePowerShell -Script $remote
    return ConvertFrom-RemoteBase64Json -EncodedJson $encodedJson
}

function Save-CameraSnapshot {
    $uri = [uri]$CameraUrl
    if ($uri.Scheme -notin @('http', 'https')) {
        throw "Unsupported camera URL: $CameraUrl"
    }

    $tmpDir = Join-Path $PSScriptRoot '..\tmp'
    New-Item -ItemType Directory -Force -Path $tmpDir | Out-Null
    $stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
    $streamPath = Join-Path $tmpDir "camera-stream-$stamp.mjpeg"
    $framePath = Join-Path $tmpDir "camera-snapshot-$stamp.jpg"

    & curl.exe --max-time 4 -sS $CameraUrl -o $streamPath 2>$null
    if (-not (Test-Path -LiteralPath $streamPath)) {
        throw "Camera stream did not produce data: $CameraUrl"
    }

    $bytes = [IO.File]::ReadAllBytes($streamPath)
    $start = -1
    for ($i = 0; $i -lt $bytes.Length - 2; $i++) {
        if ($bytes[$i] -eq 0xFF -and $bytes[$i + 1] -eq 0xD8 -and $bytes[$i + 2] -eq 0xFF) {
            $start = $i
            break
        }
    }

    $end = -1
    if ($start -ge 0) {
        for ($i = $start + 2; $i -lt $bytes.Length - 1; $i++) {
            if ($bytes[$i] -eq 0xFF -and $bytes[$i + 1] -eq 0xD9) {
                $end = $i + 1
                break
            }
        }
    }

    if ($start -lt 0 -or $end -le $start) {
        throw "No complete JPEG frame found; received $($bytes.Length) bytes."
    }

    $frame = New-Object byte[] ($end - $start + 1)
    [Array]::Copy($bytes, $start, $frame, 0, $frame.Length)
    [IO.File]::WriteAllBytes($framePath, $frame)
    Remove-Item -LiteralPath $streamPath -Force
    return [pscustomobject]@{
        cameraUrl = $CameraUrl
        snapshot = (Resolve-Path -LiteralPath $framePath).Path
        bytes = $frame.Length
    }
}

try {
    if ($Action -eq 'help') {
        Show-Help
        return
    }

    if ($Action -ne 'snapshot') {
        $guestActions = @(
            'status',
            'ensure-network',
            'start-agent',
            'stop-agent',
            'agent-log',
            'start-bringup',
            'start-camera',
            'start-mapping',
            'start-nav2',
            'mapping-status',
            'battery'
        )
        Assert-RuntimeConfiguration -RequireGuestAccess:($Action -in $guestActions)
    }

    $result = switch ($Action) {
        'status' {
            $vm = Get-VmStatus
            $guest = Get-GuestStatus
            [pscustomobject]@{
                host = $vm
                guest = $guest
            }
        }
        'start-vm' { Start-VmDetached }
        'ensure-network' { Ensure-GuestNetwork }
        'start-agent' { Start-MicroRosAgent }
        'stop-agent' { Stop-MicroRosAgent }
        'agent-log' { Get-AgentLog }
        'start-bringup' {
            Start-GuestRosLaunch -Name 'bringup' -LaunchArguments 'xuegecar_bringup xuegecar_bringup.launch.py'
        }
        'start-camera' {
            Start-GuestRosLaunch -Name 'camera' -LaunchArguments "xuegecar_camera http_video_publisher.launch.py url:=$CameraUrl"
        }
        'start-mapping' {
            Start-GuestRosLaunch -Name 'mapping' -LaunchArguments 'slam_gmapping slam_gmapping.launch.py'
        }
        'start-nav2' {
            Start-GuestRosLaunch -Name 'nav2' -LaunchArguments 'xuegecar_navigation2 navigation2.launch.py'
        }
        'mapping-status' { Get-MappingStatus }
        'battery' { Get-BatteryStatus }
        'check-forwarding' { Get-ForwardingStatus }
        'snapshot' { Save-CameraSnapshot }
    }

    if ($Json) {
        $result | ConvertTo-Json -Depth 10
    } else {
        if ($result -is [string]) {
            $result
        } elseif ($result -is [pscustomobject] -and $Action -eq 'status') {
            '=== HOST ==='
            $result.host | Format-List
            '=== GUEST ==='
            $result.guest
        } elseif ($result -is [pscustomobject] -and $Action -eq 'mapping-status') {
            $result.raw
        } else {
            $result | Format-List
        }
    }
} catch {
    Write-Error $_.Exception.Message
    exit 1
}
