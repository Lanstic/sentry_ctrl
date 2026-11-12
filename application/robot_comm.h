#ifndef ROBOT_COMM_H
#define ROBOT_COMM_H

#include "struct_typedef.h"

#define ROBOT_COMM_CMD_TIMEOUT_MS   200U
#define ROBOT_COMM_GIMBAL_CMD_TIMEOUT_MS 500U
#define ROBOT_COMM_USB_TIMEOUT_MS   300U
#define ROBOT_COMM_TELEMETRY_MS     5U

typedef struct
{
    fp32 vx;
    fp32 vy;
    fp32 wz;
    uint32_t timestamp;
} robot_chassis_cmd_t;

typedef struct
{
    fp32 yaw;
    fp32 pitch;
    uint32_t timestamp;
} robot_gimbal_cmd_t;

extern void robot_comm_init(void);
extern void robot_comm_usb_rx(const uint8_t *data, uint32_t len);
extern void robot_comm_update(void);
extern bool_t robot_comm_get_chassis_cmd(robot_chassis_cmd_t *cmd);
extern bool_t robot_comm_get_gimbal_cmd(robot_gimbal_cmd_t *cmd);
extern bool_t robot_comm_is_control_active(void);
extern bool_t robot_comm_is_usb_online(void);

#endif
