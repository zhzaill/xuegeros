$ErrorActionPreference = 'Stop'
$log = 'D:\11\Projects\xuegeros\tmp\tunnel-err.log'
$proxy = 'ssh.exe -i C:\Users\zhzaill\.ssh\id_loopros -o BatchMode=yes -o StrictHostKeyChecking=yes -W %h:%p zhzaill@192.168.1.166'
$a = @(
  '-i','C:\Users\zhzaill\.ssh\dsh_vm_key',
  '-o','BatchMode=yes',
  '-o','StrictHostKeyChecking=no',
  '-o','ExitOnForwardFailure=yes',
  '-o',("ProxyCommand=" + $proxy),
  '-N','-L','18200:127.0.0.1:18200',
  'chuyun@192.168.253.128'
)
& ssh.exe @a 2>> $log
