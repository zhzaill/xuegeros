# Simulation LAN firewall helper

Entry: Administrator PowerShell, `powershell -ExecutionPolicy Bypass -File tools/enable_sim_lan.ps1`.
Creates/enables the named inbound TCP rule for ports 18142 and 18242 from
LocalSubnet on all Windows network profiles. Does not disable Windows Firewall,
touch robotd port 18202, install packages or issue vehicle commands.
Requires Windows NetSecurity cmdlets and elevation. This session's direct
New-NetFirewallRule call returned Access Denied; the helper needs an operator
with administrator rights. Do not claim cross-host access based on loopback tests.
Verify `Get-NetFirewallRule -DisplayName 'XuegerOS LAN simulation'` and HTTP plus
WS access from another LAN computer. Undo by removing that named rule.
