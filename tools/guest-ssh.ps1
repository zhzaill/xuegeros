# xuegeros guest ssh helper (dot-source this)
# Usage: GuestSsh 'shell script text'   (base64-encoded, quote-safe)
$script:GuestSshArgs = @(
  '-i','C:\Users\zhzaill\.ssh\dsh_vm_key',
  '-o','BatchMode=yes',
  '-o','StrictHostKeyChecking=no',
  '-o','ConnectTimeout=10',
  '-o','ProxyCommand=ssh.exe -i C:\Users\zhzaill\.ssh\id_loopros -o BatchMode=yes -o StrictHostKeyChecking=yes -W %h:%p zhzaill@192.168.1.166',
  'chuyun@192.168.253.128'
)

function Invoke-GuestScript {
  param(
    [Parameter(Mandatory=$true)][string]$Script,
    [switch]$AsRoot
  )
  $b64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($Script))
  $args = $script:GuestSshArgs.Clone()
  if ($AsRoot) { $args[-1] = 'root@192.168.253.128' }
  # avoid PowerShell mangling: pass the remote command as one array element
  $remote = "base64 -d <<< '$b64' | bash"
  & ssh.exe @args $remote 2>&1
}

function GuestSsh { param([Parameter(Mandatory=$true)][string]$Command) Invoke-GuestScript -Script $Command }
function GuestRootSsh { param([Parameter(Mandatory=$true)][string]$Command) Invoke-GuestScript -Script $Command -AsRoot }
