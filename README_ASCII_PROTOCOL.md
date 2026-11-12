# USB ASCII 通信

当前固件通过 USB CDC 发送 ASCII 遥测，并接收 ASCII 云台控制命令。

## 遥测

每 100 ms 发送一行，例如：

```text
T=12345 IMU yaw=+0.12 pitch=-2.34 roll=+1.23 yaw_rate=+0.40 pitch_rate=-0.10 roll_rate=+0.05 | JOINT pitch=+151.74 yaw=+87.80 | MOTION vx=+0.000 vy=+0.000 wz=+0.000 | STATE chassis=ERR gimbal=ERR shoot=ERR
```

行结束符为 `\r\n`。

## 云台控制

发送：

```text
G <Yaw整数角度> <Pitch整数角度>\r\n
```

例如：

```text
G 93 152
```

停止云台命令：

```text
S
```

必须在命令后附加 `\r\n` 或 `\n`。云台命令超过 500 ms 没有刷新时自动失效。

## 相关开关

`application/robot_comm.c`：

```c
#define ROBOT_COMM_TEXT_TELEMETRY       1U
#define ROBOT_COMM_ASCII_COMMAND_INPUT  1U
```
