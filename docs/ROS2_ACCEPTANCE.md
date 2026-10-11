# ROS2 / Ackermann / HAL acceptance

Validated on 2026-10-11 (Asia/Shanghai). **PASS_REAL_SIM** for the ROS safety-to-CARLA command/state loop; **PASS_MOCK** for separately injected stationary ROS state; **PENDING_HARDWARE** for the physical chassis and target-compute HIL.

中文摘要：已实际运行 Humble ROS2 Topic/Service/TF、共享 SafetySupervisor 和 CARLA 0.9.15 真实车辆物理；没有将 Python 类内消息模拟作为 ROS 验收。车辆定位使用 CARLA Ground Truth，不代表实现真实感知或目标底盘串口/CAN 驱动。

## Executed pipeline and contract

```text
Timestamped /falcon/command AckermannDriveStamped
    + /falcon/state Odometry
    -> shared control.safety.SafetySupervisor
    -> /falcon/safe_command AckermannDriveStamped
    -> HAL CarlaAdapter
    -> CARLA real VehicleControl / synchronous physics
    -> actual rear-axle state -> /falcon/state + TF2
```

The package lives in `ros2/falcon_ros`. Algorithm-level `VehicleState(x,y,psi,v)` and `VehicleCommand(delta,a)` remain unchanged. ROS state uses `map -> rear_axle`, right-handed x-forward/y-left, CCW yaw radians, metres and metres per second. Road-wheel steering radians and longitudinal acceleration m/s² occupy the Ackermann steering_angle/acceleration fields. This is an acceleration-mode HAL contract: this bridge does not interpret the Ackermann speed field as an independent velocity goal. A future physical driver must implement that contract explicitly or translate it to its validated actuator interface.

The CARLA bridge reuses the project adapter's handedness and rear-axle conversion. Tested mode was `normalized` VehicleControl with the adapter's explicit nominal gain, which is not a hardware calibration. CARLA state is target-level simulation truth. ROS wall timestamps are converted into monotonic age envelopes before invoking the shared supervisor. Native R550 motor, steering, serial and CAN formats remain unimplemented until actual vendor documentation and hardware exist.

## Reproduce in Ubuntu 22.04 / WSL2

Tested runtime: Ubuntu 22.04, ROS2 Humble, Python 3.10.12, CARLA server/API 0.9.15, WSL2 Linux 6.18.40.1, AMD Ryzen 5 9600X (12 logical CPUs). Raw system/ROS versions are in `ros2/evidence/environment.txt`.

```bash
# After configuring the official ROS2 Humble apt repository:
sudo apt-get install -y ros-humble-ros-base ros-humble-ackermann-msgs \
  ros-humble-tf2-ros python3-colcon-common-extensions python3-numpy
python3 -m pip install carla==0.9.15  # Needed only for real CARLA mode

# From the repository root in Ubuntu/WSL:
bash ros2/verify.sh                  # Build, real ROS mock-input faults, bag record/replay
bash ros2/build.sh
bash ros2/accept.sh                  # Individual real ROS acceptance with synthetic stationary state
bash ros2/accept.sh --carla --host <reachable-CARLA-server-IP>
```

The local real-server run used the reachable Windows host address `172.21.16.1`, TCP 2000, Town10HD_Opt. That address is environment-specific; replace it with the current server address. Real mode spawns and owns one Tesla Model 3 actor, runs synchronous 50 ms physics with no-rendering mode, destroys its actor and restores the original world settings. It must have exclusive CARLA tick ownership. It does not kill unrelated nodes or actors.

The builder uses `$HOME/falcon_acceptance_ws`, configurable with `FALCON_ROS_WS`, and refuses to overwrite unrelated packages. Automated acceptance uses localhost ROS domain 74. Build output and raw logs are retained under `ros2/evidence`. A passive observer replays the recorded bag after publishers and CARLA/HAL adapters are stopped, so old commands are not applied to a running actuator.

Manual use after building:

```bash
source /opt/ros/humble/setup.bash
source "$HOME/falcon_acceptance_ws/install/setup.bash"
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=74 ROS_LOCALHOST_ONLY=1
ros2 launch falcon_ros bringup.launch.py
# A second terminal with the same sourced environment can attach without ticking:
ros2 run falcon_ros carla_bridge.py --host <server-IP> --actor-id <existing-vehicle-ID>
# Review the recorded state/commands without applying them:
bash ros2/replay_latest.sh carla
```

## Interfaces and safety acceptance

| Interface | Implemented contract / measured behavior |
|---|---|
| `/falcon/state` | nav_msgs/Odometry, map/rear_axle, measured state for CARLA mode; explicitly synthetic for mock-input mode |
| `/falcon/command` | Timestamped Ackermann steering/acceleration requested command |
| `/falcon/safe_command` | Timestamped supervisor-limited command actually consumed by the CARLA HAL |
| `/falcon/manual_command` | Timestamped manual command; same limits and TTL checks |
| `/falcon/safety_status` | Active/manual/stopped state and intervention reason |
| `/falcon/control_period` | Measured safety-node timer period in seconds |
| `/falcon/software_estop` | SetBool; software stop latches; false does not reset |
| `/falcon/reset_safety` | Trigger; requires fresh stationary measured state and explicit reset |
| `/falcon/manual_override` | SetBool; requires a fresh manual-command stream before enabling |
| `/tf` | Actual map-to-rear_axle TF from the CARLA state |

`config/safety.yaml` supplies ROS parameters for the same SafetyConfig used outside ROS. Tested thresholds remain state/command TTL 200 ms, maximum speed 20 m/s, steering angle 0.5 rad, steering rate 0.8 rad/s, acceleration 3 m/s² and braking magnitude 6 m/s². Nonfinite or wrong-frame inputs latch a stop. Default geofence is ±10 km and must be configured for deployment. The CARLA receiver independently rejects stale safe commands and brakes on timeout.

Actual DDS/service fault checks passed: command timeout, state timeout, latched stop after valid commands resume, explicit stationary reset, refusal to reset stale state, angle/acceleration clamp, steering rate limiting, software estop and manual-command handshake/override. These are software/ROS checks; no physical hardware emergency-stop circuit has been tested.

## Latest measured evidence

Raw results are `ros2/evidence/mock_io/acceptance.json`, `ros2/evidence/carla/acceptance.json`, adjacent replay JSON, launch logs, actual rosbag SQLite files and `bag_info.txt`. This is a dedicated ROS transport/physics smoke regression, separate from the project's multi-scene planning/control benchmarks.

| Measurement | Real ROS with mock input | Real ROS + CARLA physics |
|---|---:|---:|
| Status | PASS_MOCK (real ROS) | PASS_REAL_SIM |
| ROS command observer samples | 304 | 802 |
| ROS message latency P50 / P95 / P99 | 0.950 / 2.230 / 3.079 ms | 0.641 / 0.950 / 1.350 ms |
| Actual safety timer period P50 / P95 / P99 | 10.009 / 10.509 / 11.393 ms | 9.993 / 10.630 / 10.778 ms |
| Actual safety timer jitter standard deviation | 0.413 ms | 0.348 ms |
| Subscriber interarrival jitter standard deviation | 1.072 ms | Separate from timer measurement |
| Forward displacement during 3 s requested acceleration | Not applicable | 5.009897 m |
| Speed after 4 s requested full braking | Not applicable | 0.0 m/s |
| Real rosbag replay safe-command messages observed | 347 | 838 |

The CARLA test uses zero requested steering, requested acceleration 2 m/s² and requested braking -6 m/s²; real acceleration follows the CARLA actuator mapping rather than an ideal mathematical integrator. Its latest run applied 100 Hz safe commands to an actual CARLA actor and verified measured displacement and stopped speed. No fabricated perception or vehicle model is used for that displacement.

Latency measures publication ROS stamp to observer callback on the same WSL host, including DDS/executor delay; it is not planner-to-actuator latency. NumPy percentiles use linear interpolation. Timer jitter uses the actual monotonic timer interval, not subscriber interarrival. These workstation numbers do not establish hard-real-time scheduling or Raspberry Pi/HIL timing.

GNU `time -v` logs record mock acceptance including playback at 8.29 s, 120% job CPU, maximum RSS 58,892 KiB. CARLA launch/acceptance/record/teardown (before separate playback) took 13.64 s, 88% job CPU, maximum RSS 97,120 KiB. CPU percentage is the Linux job metric and maximum RSS is a maximum process measurement, not summed memory of the entire ROS graph or Windows CARLA server. Source logs are `ros2/evidence/mock_runtime.txt` and `ros2/evidence/carla_runtime.txt`.

## Engineering failures and remaining boundary

A single-thread ROS timer performing blocking CARLA synchronous ticks could starve command callbacks. The initial test failed with zero motion and the safety timeout correctly intervened. Separate mutually exclusive callback groups and a two-thread executor now allow command delivery while ticking, with no-rendering physics and restoration of the previous world settings. The final test above passed without changing safety TTLs. A manual-override transition previously depended on whether a manual command arrived before the next safety tick; enabling now requires a fresh manual stream, preventing that race and preserving the latched timeout behavior.

The standalone ROS route executes shared SafetySupervisor and HAL with real CARLA; the full behavior/planner/MPC scenario regressions are separate project experiments. CARLA sensor-based estimation/perception is not established by this ground-truth test. The R550 transport, actual steering mechanism, encoder/IMU calibration, independent physical estop and target-platform timing remain **PENDING_HARDWARE**. The software is **Hardware-Ready / Pending Physical Validation**, and HIL-ready without a measured target-HIL or real-vehicle claim.
