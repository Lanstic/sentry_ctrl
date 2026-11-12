#include "robot_comm.h"

#include <math.h>
#include <string.h>

#include "CRC8_CRC16.h"
#include "cmsis_os.h"
#include "main.h"
#include "usbd_cdc_if.h"

#include "CAN_receive.h"
#include "INS_task.h"
#include "chassis_task.h"
#include "detect_task.h"
#include "gimbal_task.h"

#define ROBOT_COMM_SOF                  0x5AU
#define ROBOT_COMM_CMD_ID               0x01U
#define ROBOT_COMM_IMU_ID               0x02U
#define ROBOT_COMM_ROBOT_INFO_ID        0x03U
#define ROBOT_COMM_JOINT_STATE_ID       0x0CU
#define ROBOT_COMM_ROBOT_MOTION_ID      0x08U

#define ROBOT_COMM_CMD_PACKET_SIZE      49U
#define ROBOT_COMM_CMD_DATA_SIZE        43U
#define ROBOT_COMM_RX_RING_SIZE         1024U
#define ROBOT_COMM_PARSE_BUF_SIZE       128U
#define ROBOT_COMM_MAX_DATA_SIZE        80U
#define ROBOT_COMM_TX_BUSY_RETRY_COUNT  2U

#pragma pack(push, 1)

typedef struct
{
    uint8_t sof;
    uint8_t len;
    uint8_t id;
    uint8_t crc;
} robot_comm_header_t;

typedef struct
{
    robot_comm_header_t header;
    uint32_t timestamp;
    struct
    {
        float yaw;
        float pitch;
        float roll;
        float yaw_vel;
        float pitch_vel;
        float roll_vel;
    } data;
    uint16_t checksum;
} robot_comm_imu_packet_t;

typedef struct
{
    robot_comm_header_t header;
    uint32_t timestamp;
    struct
    {
        uint16_t type;
        uint8_t state;
    } data;
    uint16_t checksum;
} robot_comm_robot_info_packet_t;

typedef struct
{
    robot_comm_header_t header;
    uint32_t timestamp;
    struct
    {
        float vx;
        float vy;
        float wz;
    } data;
    uint16_t checksum;
} robot_comm_motion_packet_t;

typedef struct
{
    robot_comm_header_t header;
    uint32_t timestamp;
    struct
    {
        float pitch;
        float yaw;
    } data;
    uint16_t checksum;
} robot_comm_joint_packet_t;

#pragma pack(pop)

#define ROBOT_COMM_STATIC_ASSERT(name, condition) typedef char name[(condition) ? 1 : -1]

ROBOT_COMM_STATIC_ASSERT(robot_comm_imu_size_check, sizeof(robot_comm_imu_packet_t) == 34U);
ROBOT_COMM_STATIC_ASSERT(robot_comm_info_size_check, sizeof(robot_comm_robot_info_packet_t) == 13U);
ROBOT_COMM_STATIC_ASSERT(robot_comm_motion_size_check, sizeof(robot_comm_motion_packet_t) == 22U);
ROBOT_COMM_STATIC_ASSERT(robot_comm_joint_size_check, sizeof(robot_comm_joint_packet_t) == 18U);

static uint8_t rx_ring[ROBOT_COMM_RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint32_t rx_overflow_count;

static uint8_t parse_buf[ROBOT_COMM_PARSE_BUF_SIZE];
static uint16_t parse_len;

static robot_chassis_cmd_t latest_cmd;
static uint32_t latest_cmd_tick;
static uint8_t latest_cmd_valid;
static volatile uint32_t latest_usb_frame_tick;

static robot_comm_imu_packet_t imu_packet __ALIGNED(4);
static robot_comm_robot_info_packet_t robot_info_packet __ALIGNED(4);
static robot_comm_motion_packet_t motion_packet __ALIGNED(4);
static robot_comm_joint_packet_t joint_packet __ALIGNED(4);

static uint32_t last_telemetry_tick;
static uint8_t telemetry_index;

volatile uint32_t robot_comm_tx_ok_count;
volatile uint32_t robot_comm_tx_busy_count;
volatile uint32_t robot_comm_tx_error_count;

static void send_packet(void *packet, uint16_t size);
static void parse_received_bytes(void);
static uint8_t rx_pop(uint8_t *data);
static void parse_command(const uint8_t *frame, uint16_t size);
static void send_imu(void);
static void send_robot_info(void);
static void send_motion(void);
static void send_joint_state(void);
static void prepare_header(robot_comm_header_t *header, uint8_t id, uint16_t size);
static uint8_t command_value_valid(fp32 value, fp32 limit);

void robot_comm_init(void)
{
    memset(rx_ring, 0, sizeof(rx_ring));
    memset(parse_buf, 0, sizeof(parse_buf));

    rx_head = 0;
    rx_tail = 0;
    rx_overflow_count = 0;
    parse_len = 0;

    memset(&latest_cmd, 0, sizeof(latest_cmd));
    latest_cmd_tick = 0;
    latest_cmd_valid = 0;
    latest_usb_frame_tick = 0;
    last_telemetry_tick = HAL_GetTick();
    telemetry_index = 0U;
    robot_comm_tx_ok_count = 0U;
    robot_comm_tx_busy_count = 0U;
    robot_comm_tx_error_count = 0U;

    memset(&imu_packet, 0, sizeof(imu_packet));
    memset(&robot_info_packet, 0, sizeof(robot_info_packet));
    memset(&motion_packet, 0, sizeof(motion_packet));
    memset(&joint_packet, 0, sizeof(joint_packet));

    prepare_header(&imu_packet.header, ROBOT_COMM_IMU_ID, sizeof(imu_packet));
    prepare_header(&robot_info_packet.header, ROBOT_COMM_ROBOT_INFO_ID,
                   sizeof(robot_info_packet));
    prepare_header(&motion_packet.header, ROBOT_COMM_ROBOT_MOTION_ID,
                   sizeof(motion_packet));
    prepare_header(&joint_packet.header, ROBOT_COMM_JOINT_STATE_ID,
                   sizeof(joint_packet));
}

void robot_comm_usb_rx(const uint8_t *data, uint32_t len)
{
    uint16_t head;
    uint16_t tail;
    uint16_t used;
    uint16_t free_size;
    uint32_t i;

    if (data == NULL || len == 0U)
    {
        return;
    }

    head = rx_head;
    tail = rx_tail;
    used = (uint16_t)(head - tail);
    free_size = (uint16_t)(ROBOT_COMM_RX_RING_SIZE - used);

    if (len > free_size)
    {
        rx_overflow_count++;
        return;
    }

    for (i = 0U; i < len; i++)
    {
        rx_ring[head] = data[i];
        head++;
        if (head >= ROBOT_COMM_RX_RING_SIZE)
        {
            head = 0U;
        }
    }

    __DMB();
    rx_head = head;
}

void robot_comm_update(void)
{
    uint32_t now;

    parse_received_bytes();

    now = HAL_GetTick();
    if ((uint32_t)(now - last_telemetry_tick) >= ROBOT_COMM_TELEMETRY_MS)
    {
        last_telemetry_tick = now;
        switch (telemetry_index)
        {
            case 0U:
                send_imu();
                break;
            case 1U:
                send_robot_info();
                break;
            case 2U:
                send_motion();
                break;
            default:
                send_joint_state();
                break;
        }
        telemetry_index = (uint8_t)((telemetry_index + 1U) & 0x03U);
    }
}

bool_t robot_comm_get_chassis_cmd(robot_chassis_cmd_t *cmd)
{
    uint32_t now;
    bool_t valid;
    uint32_t primask;

    if (cmd == NULL)
    {
        return 0;
    }

    now = HAL_GetTick();
    primask = __get_PRIMASK();
    __disable_irq();
    valid = latest_cmd_valid &&
            ((uint32_t)(now - latest_cmd_tick) <= ROBOT_COMM_CMD_TIMEOUT_MS);
    if (valid)
    {
        *cmd = latest_cmd;
    }
    else
    {
        latest_cmd_valid = 0;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }

    return valid;
}

bool_t robot_comm_is_control_active(void)
{
    robot_chassis_cmd_t cmd;
    return robot_comm_get_chassis_cmd(&cmd);
}

bool_t robot_comm_is_usb_online(void)
{
    uint32_t last_frame = latest_usb_frame_tick;
    return (last_frame != 0U) &&
           ((uint32_t)(HAL_GetTick() - last_frame) <= ROBOT_COMM_USB_TIMEOUT_MS);
}

static uint8_t rx_pop(uint8_t *data)
{
    uint16_t tail = rx_tail;

    if (tail == rx_head)
    {
        return 0;
    }

    *data = rx_ring[tail];
    tail++;
    if (tail >= ROBOT_COMM_RX_RING_SIZE)
    {
        tail = 0U;
    }
    __DMB();
    rx_tail = tail;
    return 1;
}

static void parse_received_bytes(void)
{
    uint8_t byte;
    uint16_t total_size;
    uint16_t i;

    while (rx_pop(&byte))
    {
        if (parse_len == 0U && byte != ROBOT_COMM_SOF)
        {
            continue;
        }

        if (parse_len >= ROBOT_COMM_PARSE_BUF_SIZE)
        {
            parse_len = 0;
            continue;
        }

        parse_buf[parse_len++] = byte;

        if (parse_len >= 2U &&
            (parse_buf[1] == 0U || parse_buf[1] > ROBOT_COMM_MAX_DATA_SIZE))
        {
            parse_len = 0;
            continue;
        }

        if (parse_len >= 4U &&
            !verify_CRC8_check_sum(parse_buf, sizeof(robot_comm_header_t)))
        {
            for (i = 1U; i < parse_len; i++)
            {
                if (parse_buf[i] == ROBOT_COMM_SOF)
                {
                    memmove(parse_buf, &parse_buf[i], parse_len - i);
                    parse_len = (uint16_t)(parse_len - i);
                    break;
                }
            }
            if (i == parse_len)
            {
                parse_len = 0;
            }
            continue;
        }

        if (parse_len < 4U)
        {
            continue;
        }

        total_size = (uint16_t)(4U + parse_buf[1] + 2U);
        if (parse_len < total_size)
        {
            continue;
        }

        if (verify_CRC16_check_sum(parse_buf, total_size))
        {
            latest_usb_frame_tick = HAL_GetTick();
            if (parse_buf[2] == ROBOT_COMM_CMD_ID &&
                parse_buf[1] == ROBOT_COMM_CMD_DATA_SIZE &&
                total_size == ROBOT_COMM_CMD_PACKET_SIZE)
            {
                parse_command(parse_buf, total_size);
            }
        }

        parse_len = 0;
    }
}

static void parse_command(const uint8_t *frame, uint16_t size)
{
    fp32 vx;
    fp32 vy;
    fp32 wz;
    uint32_t timestamp;
    uint32_t primask;

    if (size != ROBOT_COMM_CMD_PACKET_SIZE)
    {
        return;
    }

    memcpy(&timestamp, &frame[4], sizeof(timestamp));
    memcpy(&vx, &frame[8], sizeof(vx));
    memcpy(&vy, &frame[12], sizeof(vy));
    memcpy(&wz, &frame[16], sizeof(wz));

    if (timestamp == 0U ||
        !command_value_valid(vx, 5.0f) ||
        !command_value_valid(vy, 5.0f) ||
        !command_value_valid(wz, 20.0f))
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    latest_cmd.vx = vx;
    latest_cmd.vy = vy;
    latest_cmd.wz = wz;
    latest_cmd.timestamp = timestamp;
    latest_cmd_tick = HAL_GetTick();
    latest_cmd_valid = 1;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static uint8_t command_value_valid(fp32 value, fp32 limit)
{
    return (value == value) && (fabsf(value) <= limit);
}

static void prepare_header(robot_comm_header_t *header, uint8_t id, uint16_t size)
{
    header->sof = ROBOT_COMM_SOF;
    header->len = (uint8_t)(size - 6U);
    header->id = id;
    append_CRC8_check_sum((unsigned char *)header, sizeof(*header));
}

static void send_packet(void *packet, uint16_t size)
{
    uint8_t result;
    uint8_t retry;

    if (packet == NULL)
    {
        return;
    }

    append_CRC16_check_sum((uint8_t *)packet, size);
    result = CDC_Transmit_FS((uint8_t *)packet, size);
    for (retry = 0U; (result == USBD_BUSY) &&
                     (retry < ROBOT_COMM_TX_BUSY_RETRY_COUNT); retry++)
    {
        robot_comm_tx_busy_count++;
        osDelay(1U);
        result = CDC_Transmit_FS((uint8_t *)packet, size);
    }

    if (result == USBD_OK)
    {
        robot_comm_tx_ok_count++;
    }
    else
    {
        robot_comm_tx_error_count++;
    }
}

static void send_imu(void)
{
    const fp32 *angle = get_INS_angle_point();
    const fp32 *gyro = get_gyro_data_point();

    imu_packet.timestamp = HAL_GetTick();
    imu_packet.data.yaw = angle[INS_YAW_ADDRESS_OFFSET];
    imu_packet.data.pitch = angle[INS_PITCH_ADDRESS_OFFSET];
    imu_packet.data.roll = angle[INS_ROLL_ADDRESS_OFFSET];
    imu_packet.data.yaw_vel = gyro[INS_YAW_ADDRESS_OFFSET];
    imu_packet.data.pitch_vel = gyro[INS_PITCH_ADDRESS_OFFSET];
    imu_packet.data.roll_vel = gyro[INS_ROLL_ADDRESS_OFFSET];
    send_packet(&imu_packet, sizeof(imu_packet));
}

static void send_robot_info(void)
{
    uint16_t type = 0U;
    uint8_t state = 0U;

    type |= (uint16_t)1U;       /* mecanum chassis */
    type |= (uint16_t)(1U << 3);
    type |= (uint16_t)(1U << 6);

    if (!toe_is_error(CHASSIS_MOTOR1_TOE) &&
        !toe_is_error(CHASSIS_MOTOR2_TOE) &&
        !toe_is_error(CHASSIS_MOTOR3_TOE) &&
        !toe_is_error(CHASSIS_MOTOR4_TOE))
    {
        state |= 0x01U;
    }
    if (!toe_is_error(YAW_GIMBAL_MOTOR_TOE) &&
        !toe_is_error(PITCH_GIMBAL_MOTOR_TOE))
    {
        state |= 0x02U;
    }
    if (!toe_is_error(TRIGGER_MOTOR_TOE))
    {
        state |= 0x04U;
    }

    robot_info_packet.timestamp = HAL_GetTick();
    robot_info_packet.data.type = type;
    robot_info_packet.data.state = state;
    send_packet(&robot_info_packet, sizeof(robot_info_packet));
}

static void send_motion(void)
{
    const chassis_move_t *chassis = get_chassis_move_point();

    motion_packet.timestamp = HAL_GetTick();
    motion_packet.data.vx = chassis->vx;
    motion_packet.data.vy = chassis->vy;
    motion_packet.data.wz = chassis->wz;
    send_packet(&motion_packet, sizeof(motion_packet));
}

static void send_joint_state(void)
{
    const gimbal_motor_t *pitch = get_pitch_motor_point();
    const gimbal_motor_t *yaw = get_yaw_motor_point();

    joint_packet.timestamp = HAL_GetTick();
    joint_packet.data.pitch = pitch->relative_angle;
    joint_packet.data.yaw = yaw->relative_angle;
    send_packet(&joint_packet, sizeof(joint_packet));
}
