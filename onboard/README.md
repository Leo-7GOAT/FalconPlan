# R550 实车 ROS2 节点

树莓派 Ubuntu 22.04 + ROS2 Humble。两个节点同一进程、话题通信：

| 节点 | 订阅 | 发布 |
|---|---|---|
| `lattice_planner` | `/odom`，`/voltage` | `/cmd_vel` |
| `chassis_bridge` | `/cmd_vel` | `/odom`，`/voltage` |

`chassis_bridge` 是唯一写串口的进程。规划器只发 `geometry_msgs/Twist`。

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH=/home/ubuntu/.venvs/falcon-control/lib/python3.10/site-packages:$PYTHONPATH
python3 onboard/r550_ros2_demo.py
```

起点：缝左边 15 cm，车头顺缝，尾对齐胶条 0。盒子在缝上约 1 m。退出发 6 s 零速。人须在电源开关旁。
