# xuegeros_safety

This package is the ROS-side safety boundary for physical motion.

It is not deployed automatically. Build and inspect it before running:

```bash
cd /home/chuyun/xuegeros_ws
colcon build --packages-select xuegeros_safety
source install/setup.bash
ros2 launch xuegeros_safety xuegeros_safety.launch.py
```

Data path:

```text
/cmd_vel_teleop     priority 100
/cmd_vel_nav        priority 50
/cmd_vel_autonomy   priority 20
        |
        v
   twist_mux
        |
        v
/cmd_vel_safe_raw
        |
        v
xuegeros_cmd_vel_watchdog
        |
        v
      /cmd_vel
```

The watchdog:

- publishes zero when the last command is older than 200 ms
- publishes zero when no valid motion lease exists
- clamps all components to configured limits
- disarms through service `~/disarm`

Lease format on `/motion/lease`:

```json
{"lease_id": "short-lived-id", "expires_at": 1792650000.0}
```

Do not connect this package to a physical motor controller until:

- the micro-ROS client session is confirmed
- `/odom`, `/scan`, `/imu` are fresh
- only one `/scan` producer is active
- lifted-wheel tests pass
- the physical emergency stop is available
