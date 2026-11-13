#include "robot_comm.h"

#include <math.h>
#include <stdio.h>
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
#define ROBOT_COMM_GIMBAL_CMD_ID        0x21U
#define ROBOT_COMM_IMU_ID               0x02U
#define ROBOT_COMM_ROBOT_INFO_ID        0x03U
#define ROBOT_COMM_JOINT_STATE_ID       0x0CU
#define ROBOT_COMM_ROBOT_MOTION_ID      0x08U

#define ROBOT_COMM_CMD_PACKET_SIZE      49U
#define ROBOT_COMM_CMD_DATA_SIZE        43U
#define ROBOT_COMM_GIMBAL_CMD_PACKET_SIZE 18U
#define ROBOT_COMM_GIMBAL_CMD_DATA_SIZE   12U
#define ROBOT_COMM_RX_RING_SIZE         1024U
#define ROBOT_COMM_PARSE_BUF_SIZE       192U
#define ROBOT_COMM_MAX_DATA_SIZE        80U
#define ROBOT_COMM_TX_BUSY_RETRY_COUNT  2U

#define ROBOT_COMM_TEXT_TELEMETRY       0U
#define ROBOT_COMM_ASCII_COMMAND_INPUT  1U
#define ROBOT_COMM_GIMBAL_STATE_TELEMETRY 1U
#define ROBOT_COMM_GIMBAL_STATE_TELEMETRY_MS 10U
#define ROBOT_COMM_TEXT_BUFFER_SIZE     320U
#define ROBOT_COMM_RAD_TO_DEG           57.2957795f
#define ROBOT_COMM_ASCII_COMMAND_BUFFER_SIZE 192U
#define ROBOT_COMM_ASCII_CRC16_INIT     0xFFFFU
#define ROBOT_COMM_GIMBAL_ANGLE_LIMIT   3.1415927f
#define ROBOT_COMM_GIMBAL_VEL_LIMIT     100.0f
#define ROBOT_COMM_GIMBAL_ACC_LIMIT     1000.0f

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

typedef struct
{
    uint8_t head[2];
    uint8_t mode;
    fp32 q[4];
    fp32 yaw;
    fp32 yaw_vel;
    fp32 pitch;
    fp32 pitch_vel;
    fp32 bullet_speed;
    uint16_t bullet_count;
    uint16_t crc16;
} robot_comm_gimbal_state_packet_t;

#pragma pack(pop)

#define ROBOT_COMM_STATIC_ASSERT(name, condition) typedef char name[(condition) ? 1 : -1]

ROBOT_COMM_STATIC_ASSERT(robot_comm_imu_size_check, sizeof(robot_comm_imu_packet_t) == 34U);
ROBOT_COMM_STATIC_ASSERT(robot_comm_info_size_check, sizeof(robot_comm_robot_info_packet_t) == 13U);
ROBOT_COMM_STATIC_ASSERT(robot_comm_motion_size_check, sizeof(robot_comm_motion_packet_t) == 22U);
ROBOT_COMM_STATIC_ASSERT(robot_comm_joint_size_check, sizeof(robot_comm_joint_packet_t) == 18U);
ROBOT_COMM_STATIC_ASSERT(robot_comm_gimbal_state_size_check, sizeof(robot_comm_gimbal_state_packet_t) == 43U);

static uint8_t rx_ring[ROBOT_COMM_RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint32_t rx_overflow_count;

static uint8_t parse_buf[ROBOT_COMM_PARSE_BUF_SIZE];
static uint16_t parse_len;
#if ROBOT_COMM_ASCII_COMMAND_INPUT
static char ascii_cmd_buf[ROBOT_COMM_ASCII_COMMAND_BUFFER_SIZE];
static uint16_t ascii_cmd_len;
#endif

static robot_chassis_cmd_t latest_cmd;
static uint32_t latest_cmd_tick;
static uint8_t latest_cmd_valid;
static robot_gimbal_cmd_t latest_gimbal_cmd;
static uint32_t latest_gimbal_cmd_tick;
static uint8_t latest_gimbal_cmd_valid;
static volatile uint32_t latest_usb_frame_tick;

static robot_comm_imu_packet_t imu_packet __ALIGNED(4);
static robot_comm_robot_info_packet_t robot_info_packet __ALIGNED(4);
static robot_comm_motion_packet_t motion_packet __ALIGNED(4);
static robot_comm_joint_packet_t joint_packet __ALIGNED(4);
static robot_comm_gimbal_state_packet_t gimbal_state_packet __ALIGNED(4);
static uint8_t telemetry_mode;

#if ROBOT_COMM_TEXT_TELEMETRY
static char text_telemetry_buffers[2][ROBOT_COMM_TEXT_BUFFER_SIZE] __ALIGNED(4);
static uint8_t text_telemetry_buffer_index;
#endif

static uint32_t last_telemetry_tick;
#if !ROBOT_COMM_TEXT_TELEMETRY
static uint8_t telemetry_index;
#endif

volatile uint32_t robot_comm_tx_ok_count;
volatile uint32_t robot_comm_tx_busy_count;
volatile uint32_t robot_comm_tx_error_count;

#if !ROBOT_COMM_TEXT_TELEMETRY
static void send_packet(void *packet, uint16_t size);
#endif
static void parse_received_bytes(void);
#if ROBOT_COMM_ASCII_COMMAND_INPUT
static void parse_ascii_gimbal_command(const char *command);
static uint8_t parse_ascii_sp_gimbal_command(const char *command);
static uint8_t ascii_hex_value(char value, uint8_t *result);
#endif
static uint8_t rx_pop(uint8_t *data);
static void parse_command(const uint8_t *frame, uint16_t size);
static void parse_gimbal_command(const uint8_t *frame, uint16_t size);
#if !ROBOT_COMM_TEXT_TELEMETRY
static void send_imu(void);
static void send_robot_info(void);
static void send_motion(void);
static void send_joint_state(void);
#endif
#if ROBOT_COMM_GIMBAL_STATE_TELEMETRY
static void send_gimbal_state(void);
#endif
static void prepare_header(robot_comm_header_t *header, uint8_t id, uint16_t size);
static uint8_t command_value_valid(fp32 value, fp32 limit);
#if ROBOT_COMM_TEXT_TELEMETRY
static void send_text_telemetry(void);
static void format_fixed(char *buffer, uint16_t size, fp32 value, uint8_t decimals);
#endif

void robot_comm_init(void)
{
    memset(rx_ring, 0, sizeof(rx_ring));
    memset(parse_buf, 0, sizeof(parse_buf));

    rx_head = 0;
    rx_tail = 0;
    rx_overflow_count = 0;
    parse_len = 0;
#if ROBOT_COMM_ASCII_COMMAND_INPUT
    memset(ascii_cmd_buf, 0, sizeof(ascii_cmd_buf));
    ascii_cmd_len = 0;
#endif

    memset(&latest_cmd, 0, sizeof(latest_cmd));
    latest_cmd_tick = 0;
    latest_cmd_valid = 0;
    memset(&latest_gimbal_cmd, 0, sizeof(latest_gimbal_cmd));
    latest_gimbal_cmd_tick = 0;
    latest_gimbal_cmd_valid = 0;
    latest_usb_frame_tick = 0;
    last_telemetry_tick = HAL_GetTick();
#if !ROBOT_COMM_TEXT_TELEMETRY
    telemetry_index = 0U;
#endif
    robot_comm_tx_ok_count = 0U;
    robot_comm_tx_busy_count = 0U;
    robot_comm_tx_error_count = 0U;

    memset(&imu_packet, 0, sizeof(imu_packet));
    memset(&robot_info_packet, 0, sizeof(robot_info_packet));
    memset(&motion_packet, 0, sizeof(motion_packet));
    memset(&joint_packet, 0, sizeof(joint_packet));
    memset(&gimbal_state_packet, 0, sizeof(gimbal_state_packet));
    gimbal_state_packet.head[0] = 'S';
    gimbal_state_packet.head[1] = 'P';
    telemetry_mode = 1U;

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
#if ROBOT_COMM_GIMBAL_STATE_TELEMETRY
    if ((uint32_t)(now - last_telemetry_tick) >= ROBOT_COMM_GIMBAL_STATE_TELEMETRY_MS)
    {
        last_telemetry_tick = now;
        send_gimbal_state();
    }
#elif ROBOT_COMM_TEXT_TELEMETRY
    if ((uint32_t)(now - last_telemetry_tick) >= 100U)
    {
        last_telemetry_tick = now;
        send_text_telemetry();
    }
#else
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
#endif
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

bool_t robot_comm_get_gimbal_cmd(robot_gimbal_cmd_t *cmd)
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
    valid = latest_gimbal_cmd_valid &&
            ((uint32_t)(now - latest_gimbal_cmd_tick) <=
             ROBOT_COMM_GIMBAL_CMD_TIMEOUT_MS);
    if (valid)
    {
        *cmd = latest_gimbal_cmd;
    }
    else
    {
        latest_gimbal_cmd_valid = 0;
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
#if ROBOT_COMM_ASCII_COMMAND_INPUT
        if (parse_len == 0U && byte != ROBOT_COMM_SOF)
        {
            if (byte == '\r')
            {
                continue;
            }
            if (byte == '\n')
            {
                if (ascii_cmd_len > 0U)
                {
                    ascii_cmd_buf[ascii_cmd_len] = '\0';
                    parse_ascii_gimbal_command(ascii_cmd_buf);
                    ascii_cmd_len = 0U;
                }
                continue;
            }
            if (ascii_cmd_len < (sizeof(ascii_cmd_buf) - 1U))
            {
                ascii_cmd_buf[ascii_cmd_len++] = (char)byte;
            }
            else
            {
                ascii_cmd_len = 0U;
            }
            continue;
        }

        if (byte == ROBOT_COMM_SOF)
        {
            ascii_cmd_len = 0U;
        }
#endif

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
            else if (parse_buf[2] == ROBOT_COMM_GIMBAL_CMD_ID &&
                     parse_buf[1] == ROBOT_COMM_GIMBAL_CMD_DATA_SIZE &&
                     total_size == ROBOT_COMM_GIMBAL_CMD_PACKET_SIZE)
            {
                parse_gimbal_command(parse_buf, total_size);
            }
        }

        parse_len = 0;
    }
}

#if ROBOT_COMM_ASCII_COMMAND_INPUT
static void parse_ascii_gimbal_command(const char *command)
{
    long yaw_deg;
    long pitch_deg;
    char trailing;
    uint32_t primask;

    if (command == NULL || command[0] == '\0')
    {
        return;
    }

    while (*command == ' ' || *command == '\t')
    {
        command++;
    }

    if (command[0] == 'S' && command[1] == 'P' && command[2] == ',')
    {
        (void)parse_ascii_sp_gimbal_command(command);
        return;
    }

    if (command[0] == 'S' || command[0] == 's')
    {
        primask = __get_PRIMASK();
        __disable_irq();
        latest_gimbal_cmd.mode = 0U;
        latest_gimbal_cmd_valid = 0U;
        telemetry_mode = 0U;
        if (primask == 0U)
        {
            __enable_irq();
        }
        return;
    }

    if (command[0] != 'G' && command[0] != 'g')
    {
        return;
    }

    if (sscanf(&command[1], "%ld %ld %c", &yaw_deg, &pitch_deg, &trailing) != 2)
    {
        return;
    }
    if (yaw_deg < -180L || yaw_deg > 180L ||
        pitch_deg < -180L || pitch_deg > 180L)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    latest_gimbal_cmd.mode = 1U;
    telemetry_mode = 1U;
    latest_gimbal_cmd.yaw = (fp32)yaw_deg * 0.01745329252f;
    latest_gimbal_cmd.yaw_vel = 0.0f;
    latest_gimbal_cmd.yaw_acc = 0.0f;
    latest_gimbal_cmd.pitch = (fp32)pitch_deg * 0.01745329252f;
    latest_gimbal_cmd.pitch_vel = 0.0f;
    latest_gimbal_cmd.pitch_acc = 0.0f;
    latest_gimbal_cmd.timestamp = HAL_GetTick();
    if (latest_gimbal_cmd.timestamp == 0U)
    {
        latest_gimbal_cmd.timestamp = 1U;
    }
    latest_gimbal_cmd_tick = HAL_GetTick();
    latest_gimbal_cmd_valid = 1U;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static uint8_t parse_ascii_sp_gimbal_command(const char *command)
{
    /* The CRC covers the named-field payload before '*', excluding CR/LF. */
    const char *star;
    uint16_t expected_crc;
    uint16_t calculated_crc;
    uint16_t payload_len;
    uint16_t crc_text_len;
    uint8_t crc_nibble;
    unsigned int mode;
    fp32 yaw;
    fp32 yaw_vel;
    fp32 yaw_acc;
    fp32 pitch;
    fp32 pitch_vel;
    fp32 pitch_acc;
    int parsed_len;
    uint16_t i;
    uint32_t now;
    uint32_t primask;

    if (command == NULL || command[0] != 'S' || command[1] != 'P' ||
        command[2] != ',')
    {
        return 0U;
    }

    star = strchr(command, '*');
    if (star == NULL || star <= command || star[1] == '\0')
    {
        return 0U;
    }

    payload_len = (uint16_t)(star - command);
    crc_text_len = (uint16_t)strlen(star + 1);
    if (payload_len >= ROBOT_COMM_ASCII_COMMAND_BUFFER_SIZE ||
        crc_text_len != 4U)
    {
        return 0U;
    }

    expected_crc = 0U;
    for (i = 0U; i < 4U; i++)
    {
        if (!ascii_hex_value(star[1U + i], &crc_nibble))
        {
            return 0U;
        }
        expected_crc = (uint16_t)((expected_crc << 4) | crc_nibble);
    }

    calculated_crc = get_CRC16_check_sum(
        (uint8_t *)command, payload_len, ROBOT_COMM_ASCII_CRC16_INIT);
    if (calculated_crc != expected_crc)
    {
        return 0U;
    }

    parsed_len = 0;
    if (sscanf(command,
               "SP,mode:%u,yaw:%f,yaw_vel:%f,yaw_acc:%f,pitch:%f,pitch_vel:%f,pitch_acc:%f%n",
               &mode, &yaw, &yaw_vel, &yaw_acc, &pitch, &pitch_vel,
               &pitch_acc, &parsed_len) != 7 ||
        parsed_len != (int)payload_len || mode > 3U ||
        !command_value_valid(yaw, ROBOT_COMM_GIMBAL_ANGLE_LIMIT) ||
        !command_value_valid(yaw_vel, ROBOT_COMM_GIMBAL_VEL_LIMIT) ||
        !command_value_valid(yaw_acc, ROBOT_COMM_GIMBAL_ACC_LIMIT) ||
        !command_value_valid(pitch, ROBOT_COMM_GIMBAL_ANGLE_LIMIT) ||
        !command_value_valid(pitch_vel, ROBOT_COMM_GIMBAL_VEL_LIMIT) ||
        !command_value_valid(pitch_acc, ROBOT_COMM_GIMBAL_ACC_LIMIT))
    {
        return 0U;
    }

    now = HAL_GetTick();
    if (now == 0U)
    {
        now = 1U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    latest_gimbal_cmd.mode = (uint8_t)mode;
    telemetry_mode = (uint8_t)mode;
    latest_gimbal_cmd.yaw = yaw;
    latest_gimbal_cmd.yaw_vel = yaw_vel;
    latest_gimbal_cmd.yaw_acc = yaw_acc;
    latest_gimbal_cmd.pitch = pitch;
    latest_gimbal_cmd.pitch_vel = pitch_vel;
    latest_gimbal_cmd.pitch_acc = pitch_acc;
    latest_gimbal_cmd.timestamp = now;
    latest_gimbal_cmd_tick = now;
    latest_gimbal_cmd_valid = (mode != 0U) ? 1U : 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return 1U;
}

static uint8_t ascii_hex_value(char value, uint8_t *result)
{
    if (result == NULL)
    {
        return 0U;
    }

    if (value >= '0' && value <= '9')
    {
        *result = (uint8_t)(value - '0');
    }
    else if (value >= 'A' && value <= 'F')
    {
        *result = (uint8_t)(value - 'A' + 10);
    }
    else if (value >= 'a' && value <= 'f')
    {
        *result = (uint8_t)(value - 'a' + 10);
    }
    else
    {
        return 0U;
    }

    return 1U;
}
#endif

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

static void parse_gimbal_command(const uint8_t *frame, uint16_t size)
{
    fp32 yaw;
    fp32 pitch;
    uint32_t timestamp;
    uint32_t primask;

    if (size != ROBOT_COMM_GIMBAL_CMD_PACKET_SIZE)
    {
        return;
    }

    memcpy(&timestamp, &frame[4], sizeof(timestamp));
    memcpy(&yaw, &frame[8], sizeof(yaw));
    memcpy(&pitch, &frame[12], sizeof(pitch));

    if (timestamp == 0U ||
        !command_value_valid(yaw, 3.1415927f) ||
        !command_value_valid(pitch, 3.1415927f))
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    latest_gimbal_cmd.mode = 1U;
    telemetry_mode = 1U;
    latest_gimbal_cmd.yaw = yaw;
    latest_gimbal_cmd.yaw_vel = 0.0f;
    latest_gimbal_cmd.yaw_acc = 0.0f;
    latest_gimbal_cmd.pitch = pitch;
    latest_gimbal_cmd.pitch_vel = 0.0f;
    latest_gimbal_cmd.pitch_acc = 0.0f;
    latest_gimbal_cmd.timestamp = timestamp;
    latest_gimbal_cmd_tick = HAL_GetTick();
    latest_gimbal_cmd_valid = 1;
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

#if !ROBOT_COMM_TEXT_TELEMETRY
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
#endif

#if !ROBOT_COMM_TEXT_TELEMETRY
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
#endif

#if ROBOT_COMM_GIMBAL_STATE_TELEMETRY
static void send_gimbal_state(void)
{
    const fp32 *quat = get_INS_quat_point();
    const gimbal_motor_t *pitch = get_pitch_motor_point();
    const gimbal_motor_t *yaw = get_yaw_motor_point();
    uint32_t now = HAL_GetTick();
    uint8_t result;
    uint8_t retry;

    /* Advertise auto-aim while idle so the vision process can complete its
     * first state-frame handshake. The gimbal task still remains inactive
     * until a valid CSV command is received. */
    if (latest_gimbal_cmd_valid &&
        (uint32_t)(now - latest_gimbal_cmd_tick) <= ROBOT_COMM_GIMBAL_CMD_TIMEOUT_MS)
    {
        gimbal_state_packet.mode = telemetry_mode;
    }
    else
    {
        gimbal_state_packet.mode = 1U;
    }

    memcpy(gimbal_state_packet.q, quat, sizeof(gimbal_state_packet.q));
    gimbal_state_packet.yaw = yaw->relative_angle;
    gimbal_state_packet.yaw_vel = yaw->motor_gyro;
    gimbal_state_packet.pitch = pitch->relative_angle;
    gimbal_state_packet.pitch_vel = pitch->motor_gyro;
    gimbal_state_packet.bullet_speed = 0.0f;
    gimbal_state_packet.bullet_count = 0U;
    append_CRC16_check_sum((uint8_t *)&gimbal_state_packet,
                           sizeof(gimbal_state_packet));

    result = CDC_Transmit_FS((uint8_t *)&gimbal_state_packet,
                             sizeof(gimbal_state_packet));
    for (retry = 0U; (result == USBD_BUSY) &&
                     (retry < ROBOT_COMM_TX_BUSY_RETRY_COUNT); retry++)
    {
        robot_comm_tx_busy_count++;
        osDelay(1U);
        result = CDC_Transmit_FS((uint8_t *)&gimbal_state_packet,
                                 sizeof(gimbal_state_packet));
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
#endif

#if ROBOT_COMM_TEXT_TELEMETRY
static void format_fixed(char *buffer, uint16_t size, fp32 value, uint8_t decimals)
{
    int32_t factor;
    int32_t scaled;
    uint32_t magnitude;
    uint32_t whole;
    uint32_t fraction;

    if (value != value)
    {
        (void)snprintf(buffer, size, "nan");
        return;
    }

    factor = (decimals == 2U) ? 100 : 1000;
    value *= (fp32)factor;
    if (value > 2147483000.0f || value < -2147483000.0f)
    {
        (void)snprintf(buffer, size, "ovf");
        return;
    }

    scaled = (int32_t)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
    magnitude = (scaled < 0) ? (uint32_t)(-scaled) : (uint32_t)scaled;
    whole = magnitude / (uint32_t)factor;
    fraction = magnitude % (uint32_t)factor;

    if (decimals == 2U)
    {
        (void)snprintf(buffer, size, "%s%lu.%02lu",
                       (scaled < 0) ? "-" : "+",
                       (unsigned long)whole,
                       (unsigned long)fraction);
    }
    else
    {
        (void)snprintf(buffer, size, "%s%lu.%03lu",
                       (scaled < 0) ? "-" : "+",
                       (unsigned long)whole,
                       (unsigned long)fraction);
    }
}

static void send_text_telemetry(void)
{
    const fp32 *angle = get_INS_angle_point();
    const fp32 *gyro = get_gyro_data_point();
    const chassis_move_t *chassis = get_chassis_move_point();
    const gimbal_motor_t *pitch_motor = get_pitch_motor_point();
    const gimbal_motor_t *yaw_motor = get_yaw_motor_point();
    /*
     * This function is called from one task only. Keep the formatting
     * scratch space out of the FreeRTOS task stack.
     */
    static char yaw[16];
    static char pitch[16];
    static char roll[16];
    static char yaw_rate[16];
    static char pitch_rate[16];
    static char roll_rate[16];
    static char joint_pitch[16];
    static char joint_yaw[16];
    static char vx[16];
    static char vy[16];
    static char wz[16];
    char *text;
    int length;
    uint8_t result;
    uint8_t chassis_ok;
    uint8_t gimbal_ok;
    uint8_t shoot_ok;

    format_fixed(yaw, sizeof(yaw),
                 angle[INS_YAW_ADDRESS_OFFSET] * ROBOT_COMM_RAD_TO_DEG, 2U);
    format_fixed(pitch, sizeof(pitch),
                 angle[INS_PITCH_ADDRESS_OFFSET] * ROBOT_COMM_RAD_TO_DEG, 2U);
    format_fixed(roll, sizeof(roll),
                 angle[INS_ROLL_ADDRESS_OFFSET] * ROBOT_COMM_RAD_TO_DEG, 2U);
    format_fixed(yaw_rate, sizeof(yaw_rate),
                 gyro[INS_YAW_ADDRESS_OFFSET] * ROBOT_COMM_RAD_TO_DEG, 2U);
    format_fixed(pitch_rate, sizeof(pitch_rate),
                 gyro[INS_PITCH_ADDRESS_OFFSET] * ROBOT_COMM_RAD_TO_DEG, 2U);
    format_fixed(roll_rate, sizeof(roll_rate),
                 gyro[INS_ROLL_ADDRESS_OFFSET] * ROBOT_COMM_RAD_TO_DEG, 2U);
    format_fixed(joint_pitch, sizeof(joint_pitch),
                 pitch_motor->relative_angle * ROBOT_COMM_RAD_TO_DEG, 2U);
    format_fixed(joint_yaw, sizeof(joint_yaw),
                 yaw_motor->relative_angle * ROBOT_COMM_RAD_TO_DEG, 2U);
    format_fixed(vx, sizeof(vx), chassis->vx, 3U);
    format_fixed(vy, sizeof(vy), chassis->vy, 3U);
    format_fixed(wz, sizeof(wz), chassis->wz, 3U);

    chassis_ok = (!toe_is_error(CHASSIS_MOTOR1_TOE) &&
                  !toe_is_error(CHASSIS_MOTOR2_TOE) &&
                  !toe_is_error(CHASSIS_MOTOR3_TOE) &&
                  !toe_is_error(CHASSIS_MOTOR4_TOE)) ? 1U : 0U;
    gimbal_ok = (!toe_is_error(YAW_GIMBAL_MOTOR_TOE) &&
                 !toe_is_error(PITCH_GIMBAL_MOTOR_TOE)) ? 1U : 0U;
    shoot_ok = (!toe_is_error(TRIGGER_MOTOR_TOE)) ? 1U : 0U;

    text = text_telemetry_buffers[text_telemetry_buffer_index];
    text_telemetry_buffer_index ^= 1U;

    length = snprintf(
        text,
        ROBOT_COMM_TEXT_BUFFER_SIZE,
        "T=%lu IMU yaw=%s pitch=%s roll=%s "
        "yaw_rate=%s pitch_rate=%s roll_rate=%s | "
        "JOINT pitch=%s yaw=%s | "
        "MOTION vx=%s vy=%s wz=%s | "
        "STATE chassis=%s gimbal=%s shoot=%s\r\n",
        (unsigned long)HAL_GetTick(),
        yaw,
        pitch,
        roll,
        yaw_rate,
        pitch_rate,
        roll_rate,
        joint_pitch,
        joint_yaw,
        vx,
        vy,
        wz,
        chassis_ok ? "OK" : "ERR",
        gimbal_ok ? "OK" : "ERR",
        shoot_ok ? "OK" : "ERR");

    if (length <= 0)
    {
        robot_comm_tx_error_count++;
        return;
    }
    if (length >= (int)ROBOT_COMM_TEXT_BUFFER_SIZE)
    {
        length = (int)ROBOT_COMM_TEXT_BUFFER_SIZE - 1;
    }

    result = CDC_Transmit_FS((uint8_t *)text, (uint16_t)length);
    if (result == USBD_OK)
    {
        robot_comm_tx_ok_count++;
    }
    else if (result == USBD_BUSY)
    {
        robot_comm_tx_busy_count++;
    }
    else
    {
        robot_comm_tx_error_count++;
    }
}
#endif
