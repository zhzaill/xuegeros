# Run once in an Administrator PowerShell to allow the two simulation ports.
$ErrorActionPreference = 'Stop'
$name = 'XuegerOS LAN simulation'
$rule = Get-NetFirewallRule -DisplayName $name -ErrorAction SilentlyContinue
if (-not $rule) {
    New-NetFirewallRule -DisplayName $name -Direction Inbound -Action Allow `
        -Protocol TCP -LocalPort 18142,18242 -RemoteAddress LocalSubnet -Profile Any | Out-Null
} else {
    $rule | Set-NetFirewallRule -Enabled True -Action Allow
}
Write-Output 'XuegerOS simulation TCP 18142/18242 allowed from LocalSubnet.'
