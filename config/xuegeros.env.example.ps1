# Copy this file outside source control, fill in local paths, then dot-source it:
#
#   . .\config\xuegeros.env.ps1
#
# Never commit the filled-in file.

$env:XUEGEROS_SERVER_HOST = '192.168.1.166'
$env:XUEGEROS_SERVER_USER = 'zhzaill'
$env:XUEGEROS_SERVER_KEY = "$HOME\.ssh\id_loopros"

$env:XUEGEROS_VMRUN_PATH = 'C:\Program Files (x86)\VMware\VMware Workstation\vmrun.exe'
$env:XUEGEROS_VMX_PATH = 'F:\Studio\LEAP_ROS\7.虚拟机镜像\LEAP_ROSV2.1\LEAP_ROSV2.1.vmx'

$env:XUEGEROS_GUEST_IP = '192.168.253.128'
$env:XUEGEROS_GUEST_KEY = "$HOME\.ssh\dsh_vm_key"
$env:XUEGEROS_GUEST_RUNTIME_USER = 'chuyun'
$env:XUEGEROS_GUEST_WORKSPACE = '/home/chuyun/xuegeros_ws'

$env:XUEGEROS_AGENT_BINARY = '/home/chuyun/microros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent'
$env:XUEGEROS_AGENT_LIBRARY_PATH = '/home/chuyun/microros_ws/install/micro_ros_agent/lib:/home/chuyun/microros_ws/install/micro_ros_msgs/lib:/opt/ros/humble/lib'
$env:XUEGEROS_AGENT_LOG = '/home/chuyun/microros_ws/log/micro_ros_agent.log'

# Current camera endpoint. The address is DHCP-assigned and may change.
$env:XUEGEROS_CAMERA_URL = 'http://192.168.1.189:81/'
