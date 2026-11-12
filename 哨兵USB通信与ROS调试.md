# 哨兵 USB 通信与 ROS 调试

## 1. 当前链路

```text
Jetson ROS 2 /cmd_vel
        |
standard_robot_pp_ros2
        |
USB CDC /dev/ttyACM0
        |
STM32 robot_comm.c
        |
现有底盘速度输入与电机控制
```

STM32 是 USB Device，Jetson 是 USB Host。USB CDC 的线路参数不受 YAML 中
`baud_rate` 实际限制，115200 仅为兼容串口驱动配置。

## 2. 已实现的数据包

| 方向 | ID | 内容 |
|---|---:|---|
| Jetson 到下位机 | `0x01` | 底盘 `vx/vy/wz`、云台、发射和视觉状态 |
| 下位机到 Jetson | `0x02` | IMU yaw/pitch/roll 与三轴角速度 |
| 下位机到 Jetson | `0x03` | 机器人机构类型与底盘/云台/发射在线状态 |
| 下位机到 Jetson | `0x0C` | yaw/pitch 关节角 |
| 下位机到 Jetson | `0x08` | 底盘实测 `vx/vy/wz` |

所有帧均为小端序，结构为：

```text
0x5A | len | id | CRC8 | timestamp | data | CRC16
```

## 3. 安全行为

- 下位机只接受 CRC8、CRC16 和长度均正确的 `0x01` 指令。
- 指令时间戳为 0 时只作为心跳，不接管底盘。
- 有效指令超过 `200 ms` 未刷新时自动失效。
- ROS 节点超过 `cmd_vel_timeout_ms`（默认 `300 ms`）未收到 `/cmd_vel` 时，
  发送时间戳 0 和零速度。
- USB 在 `200 ms` 内收到过有效时间戳速度命令，才视为上位机控制有效；时间戳为 0 的心跳不算控制。
- 蜂鸣器只在遥控器和上位机有效控制同时失效时报告控制丢失，单独失去遥控器不会响。
- DBUS 丢失导致的云台无力不会阻止 Jetson 底盘控制；主动云台无力、运动禁止和校准条件仍会阻止底盘。
- 下位机退出 Jetson 模式后，恢复原有遥控和无力保护逻辑。
- 云台现有的底盘停止条件仍然拥有最终优先级。

## 4. Jetson 构建

工作空间：

```bash
cd ~/pb2025_sentry_ws-main
source /opt/ros/humble/setup.bash
rosdep install -r \
  --from-paths src/interfaces/pb_rm_interfaces \
  src/interfaces/auto_aim_interfaces src/standard_robot_pp_ros2 \
  --ignore-src --rosdistro humble -y
colcon build --symlink-install --packages-select \
  pb_rm_interfaces auto_aim_interfaces standard_robot_pp_ros2
source install/setup.bash
```

如果还需要机器人 TF 树，再编译并启动 `pb2025_robot_description`，运行时增加
`use_robot_description:=True`。

## 5. 串口权限

首次使用执行：

```bash
cd ~/pb2025_sentry_ws-main/src/standard_robot_pp_ros2
sudo ./script/create_udev_rules.sh
```

随后重新登录，确认：

```bash
ls -l /dev/ttyACM0
groups
```

## 6. 启动与视觉调试

仅启动 USB 通信：

```bash
ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py \
  params_file:=$HOME/pb2025_sentry_ws-main/src/pb2025_sentry_bringup/params/node_params.yaml \
  use_robot_description:=False \
  use_respawn:=False
```

视觉组主要查看：

```bash
ros2 topic hz /serial/imu
ros2 topic hz /serial/gimbal_joint_state
ros2 topic hz /serial/robot_motion
ros2 topic echo /serial/robot_state_info
```

低速测试底盘速度，确认底盘架空并保持急停手段：

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

停止发布后应在约 300 ms 内退出 Jetson 控制。不要用长时间保持速度命令代替
速度闭环调试。

## 7. 下位机源码

- `application/robot_comm.c`：USB 环形缓冲、协议组包/解析、状态上报和超时。
- `application/usb_task.c`：1 ms 通信任务。
- `Src/usbd_cdc_if.c`：CDC 中断只把接收字节送入缓冲。
- `application/chassis_behaviour.c`：新增 Jetson 速度输入模式。
- `application/chassis_task.c`：允许无 DBUS 时由 USB 指令启动，并暴露只读反馈。

现有 PID、麦轮运动学、功率控制和运动参数没有调整。
