# MCU command-timeout bench test (review 2026-11-02 H4) - NOT RUN YET

Question: when the host driver dies with a non-zero wheel command, how long does the MCU
firmware keep driving? mcu_node sends one SpeedData per `/cmd_vel` plus a 100 ms Heartbeat;
a `kill -9` stops both, and no host stop frame is sent. Nothing above the MCU can stop the
wheels then, so this time is the last line of defence.

## Setup (wheels up, blade off)
1. Mower on blocks, both drive wheels clear of the ground, blade disc removed or the cutter
   motor connector unplugged. E-stop reachable.
2. Stop the stack, start only the driver in the container:
   `ros2 run mower_mcu_driver mcu_node --ros-args -p cmd_vel_timeout:=5.0`
   (5 s so the host-side timeout cannot be what stops the wheels).
3. Second shell: `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"`.
4. Film the wheel and a terminal in the same frame (phone slow motion, >= 120 fps). Put a
   white tape mark on the tyre.

## Run (5 times)
1. Wheels turning steadily for >= 3 s (`ros2 topic echo /mcu/measured_speed` ~0.3 m/s).
2. In the terminal on camera: `pkill -9 -f mcu_node; date +%T.%N`.
3. Stop the `topic pub`. Wait 10 s. If the wheels still turn after 10 s, press the e-stop and
   record "no firmware timeout".

## Record per run
- t_kill (the `date` line, i.e. the frame where it appears), t_decel (first frame the mark
  slows), t_stop (mark stationary); stop time = t_stop - t_kill, and the wheel turns after
  the kill (x the measured tyre circumference = distance).
- Firmware versions (`/mower_sensor_info` cutter/chassis board versions).
- Whether the wheels coast or brake, and anything on the MCU LED.

## Acceptance
- PASS: every run stops within **1.0 s** of the kill (<= 0.3 m at 0.3 m/s, the mowing speed).
  Record the median in `mcu_node.py` "Known gaps" and `docs/crash_recovery.md` section 2.
- FAIL (> 1.0 s or no stop): the driver needs an external watchdog (the supervisor's
  `/mcu/driver_heartbeat` DOWN only stops the mission, not the wheels); raise it before the
  next unattended mow.
