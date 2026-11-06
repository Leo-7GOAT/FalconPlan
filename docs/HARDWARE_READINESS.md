# Hardware readiness

R550 实车已经在树莓派 4B + ROS2 Humble 上闭环。协议、几何、限幅、相机占栅和 `/cmd_vel` ↔ `/odom` 桥见 [VEHICLE_ONBOARD.md](VEHICLE_ONBOARD.md) 与 [onboard/](../onboard/)。

`hal/base.py`：`get_state` / `apply_command` / `step`。`VehicleState(x,y,psi,v)` 后轴、右手系、米、逆时针弧度。`VehicleCommand(delta,a)` 为前轮转角与纵向加速度。时间戳必须同一单调钟；仿真 / mock / 回放 / 实车来源要写清。

Safety 在下发前检查时间戳。超时、围栏、超速、倒车、碰撞与软件急停会锁 STOPPED。这是软件请求，不是物理急停证书。断线时 STM32 不会自己停，人必须在电源开关旁。

仿真侧 CARLA 桥与 mock 仍在 `ros2/falcon_ros/`。厂家官方驱动包没有替换现有桥。
