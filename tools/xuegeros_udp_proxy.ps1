[CmdletBinding()]
param(
    [string]$ListenAddress = $(if ($env:XUEGEROS_UDP_LISTEN_ADDRESS) { $env:XUEGEROS_UDP_LISTEN_ADDRESS } else { '0.0.0.0' }),
    [int]$ListenPort = $(if ($env:XUEGEROS_UDP_LISTEN_PORT) { [int]$env:XUEGEROS_UDP_LISTEN_PORT } else { 8888 }),
    [string]$UpstreamAddress = $(if ($env:XUEGEROS_UDP_UPSTREAM_ADDRESS) { $env:XUEGEROS_UDP_UPSTREAM_ADDRESS } else { '192.168.253.128' }),
    [int]$UpstreamPort = $(if ($env:XUEGEROS_UDP_UPSTREAM_PORT) { [int]$env:XUEGEROS_UDP_UPSTREAM_PORT } else { 8888 }),
    [string]$LogPath = $(if ($env:XUEGEROS_UDP_PROXY_LOG) { $env:XUEGEROS_UDP_PROXY_LOG } else { "$HOME\xuegeros-udp-proxy.log" })
)

# This relay carries micro-ROS UDP between the robot (ESP32) and the guest VM.
#
# It MUST NOT die on a transient socket error. A UDP socket reports an
# unreachable peer only asynchronously: when the ESP32 reboots, its Wi-Fi
# drops, or the guest VM closes its socket, the OS delivers an ICMP
# port-unreachable and .NET surfaces it on a LATER Receive() call as a
# SocketException (ConnectionReset / ConnectionRefused). The original version
# treated that as fatal:
#
#     $ErrorActionPreference = 'Stop' + Receive() in one try/catch -> exit
#
# so a single robot blip permanently killed the link. Observed 2026-10-23: the
# scheduled task ran, logged `client selected`, then died with
# `FATAL ... Receive ... SocketException` (LastTaskResult=1) and UDP 8888 was
# left unbound - the whole car looked "broken" while ROS itself was healthy.
#
# Fix: a reconnect loop. Any socket-level error disposes both sockets, rebinds,
# and retries with capped backoff, logging the reason. Only a bind failure that
# cannot be recovered (port genuinely held by another process) backs off
# progressively instead of spinning.

$ErrorActionPreference = 'Stop'

function Write-ProxyLog([string]$Message) {
    try {
        $line = "$(Get-Date -Format 'o') $Message"
        Add-Content -LiteralPath $LogPath -Value $line -Encoding utf8
    } catch {
        # A failing log must never take the relay down.
    }
}

function New-Listener {
    if ($ListenAddress -eq '0.0.0.0' -or $ListenAddress -eq '*') {
        # UdpClient(port) binds the IPv4 wildcard. Constructing an IPEndPoint
        # from 0.0.0.0 can resolve as IPv6 and fail on some runtimes.
        return [System.Net.Sockets.UdpClient]::new($ListenPort)
    }
    $endpoint = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Parse($ListenAddress), $ListenPort)
    return [System.Net.Sockets.UdpClient]::new($endpoint)
}

Write-ProxyLog "relay starting: ${ListenAddress}:${ListenPort} -> ${UpstreamAddress}:${UpstreamPort}"

$backoffSeconds = 1
$maxBackoffSeconds = 15

while ($true) {
    $listener = $null
    $upstream = $null
    try {
        $listener = New-Listener
        $upstream = [System.Net.Sockets.UdpClient]::new(0)
        $upstream.Connect($UpstreamAddress, $UpstreamPort)

        # Poll instead of blocking, so a dead peer can never wedge the loop and
        # the 30 s stats heartbeat keeps proving liveness.
        $listener.Client.ReceiveTimeout = 0
        $upstream.Client.ReceiveTimeout = 0

        $clientEndpoint = $null
        $toUpstreamPackets = 0
        $toClientPackets = 0
        $lastStatsAt = Get-Date
        $lastErrorAt = Get-Date

        Write-ProxyLog "relay listening on ${ListenAddress}:${ListenPort} -> ${UpstreamAddress}:${UpstreamPort}"
        $backoffSeconds = 1

        while ($true) {
            try {
                if ($listener.Available -gt 0) {
                    $remote = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Any, 0)
                    $data = $listener.Receive([ref]$remote)
                    if ($null -eq $clientEndpoint -or $clientEndpoint.ToString() -ne $remote.ToString()) {
                        Write-ProxyLog "client selected: $($remote.ToString())"
                    }
                    $clientEndpoint = $remote
                    [void]$upstream.Send($data, $data.Length)
                    $toUpstreamPackets++
                }

                if ($upstream.Available -gt 0) {
                    $upstreamEndpoint = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Any, 0)
                    $data = $upstream.Receive([ref]$upstreamEndpoint)
                    if ($null -ne $clientEndpoint) {
                        [void]$listener.Send($data, $data.Length, $clientEndpoint)
                        $toClientPackets++
                    }
                }
            } catch [System.Net.Sockets.SocketException] {
                # Transient by design: the peer went away. Rebind and continue,
                # but do not spin - a fast failure loop would flood the log.
                $ex = $_.Exception
                if ($ex.InnerException -is [System.Net.Sockets.SocketException]) { $ex = $ex.InnerException }
                if (((Get-Date) - $lastErrorAt).TotalSeconds -ge 5) {
                    Write-ProxyLog "socket error ($($ex.SocketErrorCode)); rebinding: $($ex.Message)"
                    $lastErrorAt = Get-Date
                }
                # Reset the peer so the next packet re-elects a client.
                $clientEndpoint = $null
                Start-Sleep -Milliseconds 200
                continue
            }

            if (((Get-Date) - $lastStatsAt).TotalSeconds -ge 30) {
                Write-ProxyLog "stats client=$($clientEndpoint) upstream=$toUpstreamPackets client=$toClientPackets"
                $lastStatsAt = Get-Date
            }

            Start-Sleep -Milliseconds 5
        }
    } catch {
        # Bind failure, or something outside the inner handler. Back off and
        # retry - the old script exited here and left the port unbound forever.
        $message = $_.Exception.Message
        if ($_.Exception.InnerException) { $message = $_.Exception.InnerException.Message }
        Write-ProxyLog "relay error, retrying in ${backoffSeconds}s: $message"
        Start-Sleep -Seconds $backoffSeconds
        $backoffSeconds = [Math]::Min($backoffSeconds * 2, $maxBackoffSeconds)
    } finally {
        if ($listener) {
            try { $listener.Dispose() } catch { }
        }
        if ($upstream) {
            try { $upstream.Dispose() } catch { }
        }
    }
}
