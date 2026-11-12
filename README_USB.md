# 哨兵 USB 通信使用说明

这份说明只讲“怎么用”，不展开通信原理。

目标链路：

```text
Jetson 发速度指令
    -> USB
    -> STM32
    -> 原有底盘控制程序
```

同时，STM32 会通过 USB 把 IMU、云台关节角和底盘速度发给 Jetson，供视觉组调试。

## 1. 两个目录分别是什么

下位机 STM32 工程：

```text
D:\zzy\rm\步兵\2026哨兵 改进
```

Jetson ROS 2 工作空间：

```text
D:\zzy\rm\步兵\pb2025_sentry_ws-main
```

本次已经补好 ROS 串口包，正常情况下不需要再自己写通信代码。

## 2. 第一次使用

### 2.1 检查下位机固件

先确认已经编译并烧录当前版本的固件。

打开：

```text
D:\zzy\rm\步兵\2026哨兵 改进\MDK-ARM\standard_tpye_c.uvprojx
```

点击 `Rebuild`，然后下载到 STM32。

注意：原工程使用 ARM Compiler 5。如果 Keil 提示：

```text
Default Compiler Version 5 which is not available
```

需要先安装 ARM Compiler 5，或者先在原工程中确认已有可用的 AC5 编译器。不要直接改成
AC6，因为原工程里有 AC5 专用写法。

### 2.2 Jetson 接 USB

用 USB 数据线连接 STM32 开发板和 Jetson。

连接后检查：

```bash
lsusb
ls /dev/ttyACM*
```

正常应该能看到：

```text
/dev/ttyACM0
```

如果没有设备名，先换一根能传数据的 USB 线，再检查供电和接口。

### 2.3 配置串口权限

以下命令只需要执行一次：

```bash
cd ~/pb2025_sentry_ws-main/src/standard_robot_pp_ros2
sudo ./script/create_udev_rules.sh
```

执行后重新登录 Jetson，再检查：

```bash
groups
```

输出中应该有 `dialout`。

## 3. 编译 ROS 通信程序

进入 Jetson 上的工作空间：

```bash
cd ~/pb2025_sentry_ws-main
source /opt/ros/humble/setup.bash
```

安装缺少的 ROS 依赖：

```bash
rosdep install -r \
  --from-paths src/interfaces/pb_rm_interfaces \
  src/interfaces/auto_aim_interfaces src/standard_robot_pp_ros2 \
  --ignore-src --rosdistro humble -y
```

编译通信需要的包：

```bash
colcon build --symlink-install --packages-select \
  pb_rm_interfaces auto_aim_interfaces standard_robot_pp_ros2
```

加载编译结果：

```bash
source install/setup.bash
```

## 4. 启动 USB 通信

只测试上下位机通信，不需要启动导航和视觉：

```bash
ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py \
  params_file:=$HOME/pb2025_sentry_ws-main/src/pb2025_sentry_bringup/params/node_params.yaml \
  use_robot_description:=False \
  use_respawn:=False
```

启动后不要马上发布速度。先看下位机有没有把数据传上来。

### 控制掉线报警规则

- 遥控器在线：`DBUS_TOE` 没有报错。
- 上位机控制有效：STM32 在 200 ms 内收到过带有效时间戳的 USB 速度指令。
- 只有遥控器和上位机有效控制同时失效时，控制丢失才会让蜂鸣器报警。
- 如果只是遥控器掉线、但 Jetson 正在持续发布 `/cmd_vel`，不会因为 `DBUS_TOE` 响蜂鸣器。
- ROS 节点只发心跳、但没有有效 `/cmd_vel`，不算上位机正在控制。
- 电机、IMU 等其他硬件故障仍按原逻辑报警。
- 遥控器离线导致云台进入无力时，不再阻止 Jetson 控制底盘；主动让云台无力或云台校准仍会阻止底盘。

## 5. 检查下位机数据

另开一个 Jetson 终端，执行：

```bash
source ~/pb2025_sentry_ws-main/install/setup.bash
ros2 topic list | grep serial
```

应该看到这些话题：

```text
/serial/imu
/serial/gimbal_joint_state
/serial/robot_motion
/serial/robot_state_info
```

检查频率：

```bash
ros2 topic hz /serial/imu
ros2 topic hz /serial/gimbal_joint_state
ros2 topic hz /serial/robot_motion
```

正常情况下应持续有数据，大约几十赫兹。

查看具体内容：

```bash
ros2 topic echo /serial/imu
ros2 topic echo /serial/gimbal_joint_state
ros2 topic echo /serial/robot_motion
```

## 6. 测试控制下位机

先确认底盘架空，并有随时断电或急停的措施。

发布一个很小的前进速度：

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

说明：

- `linear.x`：前后速度。
- `linear.y`：左右速度。
- `angular.z`：旋转速度。
- `-r 20`：每秒发送 20 次。

停止测试时，在发布速度的终端按：

```text
Ctrl+C
```

停止后约 `300 ms`，ROS 会停止刷新有效指令，下位机会自动退出 Jetson 控制模式。

## 7. 视觉组只需要做什么

视觉组不需要直接操作 USB 协议，只需要：

1. 启动 `standard_robot_pp_ros2`。
2. 订阅 `/serial/imu` 获取 IMU。
3. 订阅 `/serial/gimbal_joint_state` 获取云台关节角。
4. 订阅 `/serial/robot_motion` 获取底盘实测速度。
5. 需要测试移动时，向 `/cmd_vel` 发布 `geometry_msgs/msg/Twist`。

如果只做视觉识别，不需要向 `/cmd_vel` 发布任何内容，避免误控制底盘。

## 8. 常见问题

### 没有 `/dev/ttyACM0`

依次检查：

```bash
lsusb
ls /dev/ttyACM*
```

然后重新插拔 USB、换数据线，或重新执行 udev 配置。

### 有串口设备，但没有任何 `/serial/*` 话题

通常有三种原因：

1. STM32 没有烧录本次固件。
2. STM32 程序没有正常运行。
3. ROS 启动参数中的串口名不是实际的 `/dev/ttyACM0`。

### 有话题但没有数据

检查：

```bash
ros2 topic hz /serial/imu
```

如果一直是 `no new messages`，优先检查 STM32 固件和 USB 线。

### 发布 `/cmd_vel` 后底盘不动

依次检查：

1. 底盘电机是否在线。
2. 云台是否触发了底盘停止条件。
3. 发布速度是否在持续刷新，不能只发一次。
4. 速度是否超过当前底盘限幅。
5. 看 `/serial/robot_motion` 是否已有实际速度反馈。

### 停止 `/cmd_vel` 后还在动

先切断驱动电源。正常逻辑是：

- ROS 超过 `300 ms` 未收到 `/cmd_vel`，发送无效心跳。
- STM32 超过 `200 ms` 未收到有效控制包，退出 Jetson 模式。

如果仍然持续运动，不要继续调试，先检查固件是否真的是本次版本。

## 9. 本次修改了哪些文件

STM32：

```text
application/robot_comm.c
application/robot_comm.h
application/usb_task.c
application/chassis_task.c
application/chassis_task.h
application/chassis_behaviour.c
application/chassis_behaviour.h
Src/usbd_cdc_if.c
Src/freertos.c
MDK-ARM/standard_tpye_c.uvprojx
```

Jetson：

```text
src/standard_robot_pp_ros2
src/interfaces/pb_rm_interfaces
src/interfaces/auto_aim_interfaces
src/pb2025_robot_description
```

没有修改 PID 参数、麦轮运动学参数、功率参数和云台控制参数。

更详细的协议字段和包结构见：

```text
D:\zzy\rm\步兵\2026哨兵 改进\哨兵USB通信与ROS调试.md
```
