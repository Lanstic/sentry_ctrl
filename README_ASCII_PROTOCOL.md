# USB ASCII 通信

当前固件通过 USB CDC 发送 ASCII 遥测，并接收带 CRC16 的 ASCII 云台控制帧。

## 遥测

每 100 ms 发送一行，例如：

```text
T=12345 IMU yaw=+0.12 pitch=-2.34 roll=+1.23 yaw_rate=+0.40 pitch_rate=-0.10 roll_rate=+0.05 | JOINT pitch=+151.74 yaw=+87.80 | MOTION vx=+0.000 vy=+0.000 wz=+0.000 | STATE chassis=ERR gimbal=ERR shoot=ERR
```

行结束符为 `\r\n`。

## 云台控制

发送新协议帧：

```text
SP,mode:1,yaw:0.120000,yaw_vel:0.030000,yaw_acc:0.500000,pitch:-0.080000,pitch_vel:0.010000,pitch_acc:0.200000*338C\r\n
```

字段含义：

| 字段 | 含义 | 单位 |
| --- | --- | --- |
| `mode` | `0` 不控制，`1` 控制云台，`2` 控制云台并携带开火请求，`3` 大符模式 | 无 |
| `yaw` / `pitch` | 目标角度 | rad |
| `yaw_vel` / `pitch_vel` | 目标角速度，当前固件先接收保存 | rad/s |
| `yaw_acc` / `pitch_acc` | 目标角加速度，当前固件先接收保存 | rad/s² |

CRC 输入是从 `S` 到 `*` 前的全部 ASCII 字节，不包含 `*`、CRC 字符和 `\r\n`。CRC16 算法使用 `CRC16_INIT = 0xFFFF` 和工程现有的 `get_CRC16_check_sum()`，传输为 4 位十六进制文本。字段名、字段顺序和大小写必须完全匹配。

只有 CRC、字段格式、模式范围和数值范围全部通过后，固件才更新云台目标。`mode:0` 会清除当前上位机云台控制；有效帧超过 500 ms 未刷新也会自动失效。当前 `gimbal_task` 使用 yaw/pitch 角度闭环，速度、加速度和开火请求先保存在通信结构中，未改变原有发射控制逻辑。

为兼容旧调试工具，固件仍接受：

```text
G <Yaw整数角度> <Pitch整数角度>\r\n
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
