#!/usr/bin/env bash
# Install XuegerOS guest systemd units so the car stack survives a guest reboot.
#
# Run on the guest as root:  bash install_guest_systemd.sh
#
# Units installed (all run as chuyun, all After=network-online.target):
#   xuegeros-micro-ros-agent.service  micro-ROS UDP agent (port 8888)
#   xuegeros-bringup.service          xuegecar_bringup (odom/imu/scan/tf/battery)
#   xuegeros-cmd-bridge.service       HTTP motion bridge (127.0.0.1:18200 local,
#                                     bound 0.0.0.0 for the SSH tunnel)
#   xuegeros-safety.service           twist_mux + cmd_vel_watchdog chain
#   xuegeros-gmapping.service         slam_gmapping (/map)
#
# gmapping is deliberately its own unit: mapping is optional for driving and
# should not gate the safety chain.
#
# Idempotent: safe to re-run.
set -euo pipefail

UNIT_DIR=/etc/systemd/system
WS=/home/chuyun/xuegeros_ws
AGENT_BIN=/home/chuyun/microros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent
AGENT_LIB=/home/chuyun/microros_ws/install/micro_ros_agent/lib:/home/chuyun/microros_ws/install/micro_ros_msgs/lib:/opt/ros/humble/lib

if [ "$(id -u)" -ne 0 ]; then
  echo "run as root" >&2
  exit 1
fi

write_unit() {
  local name="$1"; shift
  cat > "$UNIT_DIR/$name"
  chmod 644 "$UNIT_DIR/$name"
  echo "wrote $UNIT_DIR/$name"
}

# --- micro-ROS agent --------------------------------------------------------
write_unit xuegeros-micro-ros-agent.service <<EOF
[Unit]
Description=XuegerOS micro-ROS agent (udp4 8888)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=chuyun
Group=chuyun
WorkingDirectory=/home/chuyun
Environment=LD_LIBRARY_PATH=$AGENT_LIB
ExecStart=$AGENT_BIN udp4 --port 8888 -v4
Restart=always
RestartSec=3
StandardOutput=append:/home/chuyun/microros_ws/log/micro_ros_agent.service.log
StandardError=append:/home/chuyun/microros_ws/log/micro_ros_agent.service.log

[Install]
WantedBy=multi-user.target
EOF

# --- bringup ----------------------------------------------------------------
write_unit xuegeros-bringup.service <<EOF
[Unit]
Description=XuegerOS xuegecar bringup (odom/imu/scan/tf/battery)
After=network-online.target xuegeros-micro-ros-agent.service
Wants=network-online.target
StartLimitIntervalSec=0

[Service]
Type=simple
User=chuyun
Group=chuyun
WorkingDirectory=$WS
Environment=HOME=/home/chuyun
Environment=ROS_DOMAIN_ID=0
ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source $WS/install/setup.bash && exec ros2 launch xuegecar_bringup xuegecar_bringup.launch.py'
Restart=always
RestartSec=5
StandardOutput=append:$WS/log/xuegeros-bringup.service.log
StandardError=append:$WS/log/xuegeros-bringup.service.log

[Install]
WantedBy=multi-user.target
EOF

# --- motion bridge ----------------------------------------------------------
# The token is read from run_cmd_bridge.sh so the secret lives in one file.
write_unit xuegeros-cmd-bridge.service <<EOF
[Unit]
Description=XuegerOS motion command bridge (HTTP 18200)
After=network-online.target xuegeros-bringup.service
Wants=network-online.target
StartLimitIntervalSec=0

[Service]
Type=simple
User=chuyun
Group=chuyun
WorkingDirectory=$WS
Environment=HOME=/home/chuyun
ExecStart=/usr/bin/env bash $WS/tools/run_cmd_bridge.sh
Restart=always
RestartSec=5
StandardOutput=append:$WS/log/xuegeros-cmd-bridge.service.log
StandardError=append:$WS/log/xuegeros-cmd-bridge.service.log

[Install]
WantedBy=multi-user.target
EOF

# --- safety chain -----------------------------------------------------------
write_unit xuegeros-safety.service <<EOF
[Unit]
Description=XuegerOS motion safety (twist_mux + cmd_vel watchdog)
After=network-online.target xuegeros-bringup.service
Wants=network-online.target
StartLimitIntervalSec=0

[Service]
Type=simple
User=chuyun
Group=chuyun
WorkingDirectory=$WS
Environment=HOME=/home/chuyun
ExecStart=/usr/bin/env bash $WS/tools/run_safety.sh
Restart=always
RestartSec=5
StandardOutput=append:$WS/log/xuegeros-safety.service.log
StandardError=append:$WS/log/xuegeros-safety.service.log

[Install]
WantedBy=multi-user.target
EOF

# --- gmapping ---------------------------------------------------------------
write_unit xuegeros-gmapping.service <<EOF
[Unit]
Description=XuegerOS slam_gmapping (/map)
After=network-online.target xuegeros-bringup.service
Wants=network-online.target
StartLimitIntervalSec=0

[Service]
Type=simple
User=chuyun
Group=chuyun
WorkingDirectory=$WS
Environment=HOME=/home/chuyun
Environment=ROS_DOMAIN_ID=0
ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source $WS/install/setup.bash && exec ros2 launch slam_gmapping slam_gmapping.launch.py'
Restart=always
RestartSec=5
StandardOutput=append:$WS/log/xuegeros-gmapping.service.log
StandardError=append:$WS/log/xuegeros-gmapping.service.log

[Install]
WantedBy=multi-user.target
EOF

mkdir -p "$WS/log"
chown -R chuyun:chuyun "$WS/log"

systemctl daemon-reload
echo "--- enabling ---"
systemctl enable xuegeros-micro-ros-agent.service \
                 xuegeros-bringup.service \
                 xuegeros-cmd-bridge.service \
                 xuegeros-safety.service \
                 xuegeros-gmapping.service

echo "--- done. Start with: ---"
echo "  systemctl start xuegeros-micro-ros-agent xuegeros-bringup xuegeros-cmd-bridge xuegeros-safety xuegeros-gmapping"
