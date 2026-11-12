/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             041¡è7041¡è7041¡è7010009041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_task.h"
#include "chassis_behaviour.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"
#include "robot_comm.h"
#include <math.h>

/* Spin exit only. Chassis yaw: rad/s, positive CCW. Joint q = gimbal - chassis. */
#define CHASSIS_ALIGN_ENABLE          1
#define CHASSIS_ALIGN_MAX_WZ          9.5f    /* rad/s; does not change auto-spin */
#define CHASSIS_ALIGN_ACCEL           30.0f   /* rad/s^2, command slew limit */
#define CHASSIS_ALIGN_BRAKE           18.0f   /* rad/s^2, below the command slew limit */
#define CHASSIS_ALIGN_LAG             0.10f   /* s, measured-speed stopping reserve */
#define CHASSIS_ALIGN_MARGIN          0.05f   /* rad, entry stopping reserve */
#define CHASSIS_ALIGN_TOL             0.03f   /* rad, about 1.7 degrees */
#define CHASSIS_ALIGN_SETTLE_WZ       0.20f   /* rad/s, command AND feedback */
#define CHASSIS_ALIGN_REVERSE_WZ      0.30f   /* rad/s, residual correction only */
#define CHASSIS_ALIGN_DWELL_MS        100U
#define CHASSIS_ALIGN_DT             (CHASSIS_CONTROL_TIME_MS * 0.001f)
#define CHASSIS_ALIGN_TWO_PI         6.2831853071795864769f

extern uint8_t chassis_last_frame_in_auto_spin;

enum { ALIGN_OFF = 0, ALIGN_INIT, ALIGN_DIRECTED, ALIGN_FOLLOW };
static struct
{
    fp32 direction;      /* S from the last auto-spin wz_set, +/-1 or 0 */
    fp32 remaining;      /* rad along S; continuous, NOT wrapped each cycle */
    fp32 last_joint;     /* rad, wrapped encoder joint angle */
    fp32 last_target;    /* rad, wrapped live target including SWING */
    fp32 command;        /* rad/s, last output of this limiter */
    uint16_t settled_ms;
    uint8_t state;
} chassis_align;
static uint8_t chassis_angle_pid_seed;

/* Clearing alone would still produce Kd * e on the first PID_calc(). */
static fp32 chassis_angle_step(pid_type_def *pid, fp32 error)
{
    if (chassis_angle_pid_seed)
    {
        PID_clear(pid);
        pid->error[0] = pid->error[1] = pid->error[2] = error;
        chassis_angle_pid_seed = 0;
    }
    return PID_calc(pid, 0.0f, error);
}

static fp32 chassis_follow_yaw_command(chassis_move_t *move, fp32 target)
{
    fp32 joint = move->chassis_yaw_motor->relative_angle;
    fp32 error = rad_format(target - joint);
    fp32 request;
    fp32 limit;
    fp32 distance;
    fp32 speed;
    fp32 brake_step = CHASSIS_ALIGN_BRAKE * CHASSIS_ALIGN_DT;
    fp32 step = CHASSIS_ALIGN_ACCEL * CHASSIS_ALIGN_DT;

    if (CHASSIS_ALIGN_ENABLE &&
        (toe_is_error(DBUS_TOE) || toe_is_error(YAW_GIMBAL_MOTOR_TOE) ||
         toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) ||
         toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE)))
    {
        /* Do not integrate stale feedback. Recovery starts from zero command. */
        chassis_align.state = ALIGN_INIT;
        chassis_align.command = 0.0f;
        chassis_align.settled_ms = 0;
        chassis_angle_pid_seed = 1;
        PID_clear(&move->chassis_angle_pid);
        return 0.0f;
    }

    if (chassis_align.state == ALIGN_INIT)
    {
        chassis_align.direction = (chassis_align.command > 0.0f) ? 1.0f :
                                  ((chassis_align.command < 0.0f) ? -1.0f : 0.0f);
        chassis_align.remaining = -chassis_align.direction * error;
        if (chassis_align.remaining < 0.0f)
        {
            chassis_align.remaining += CHASSIS_ALIGN_TWO_PI;
        }
        speed = fmaxf(fabsf(chassis_align.command),
                      chassis_align.direction * move->wz);
        distance = speed * speed / (2.0f * CHASSIS_ALIGN_BRAKE) +
                   speed * (CHASSIS_ALIGN_DT + CHASSIS_ALIGN_LAG) + CHASSIS_ALIGN_MARGIN;
        if (chassis_align.direction != 0.0f && chassis_align.remaining < distance)
        {
            /* Skip an alignment point that cannot be reached without hard braking. */
            chassis_align.remaining += CHASSIS_ALIGN_TWO_PI *
                ceilf((distance - chassis_align.remaining) / CHASSIS_ALIGN_TWO_PI);
        }
        chassis_align.last_joint = joint;
        chassis_align.last_target = target;
        chassis_align.settled_ms = 0;
        chassis_align.state = (chassis_align.direction == 0.0f) ? ALIGN_FOLLOW : ALIGN_DIRECTED;
        chassis_angle_pid_seed = 1;
        if (chassis_align.state == ALIGN_DIRECTED)
        {
            error = -chassis_align.direction * chassis_align.remaining;
        }
        (void)chassis_angle_step(&move->chassis_angle_pid, error);
        return chassis_align.command; /* Exact continuity on the switching frame. */
    }

    if (chassis_align.state == ALIGN_DIRECTED)
    {
        /* q_dot = gimbal_yaw_rate - chassis_wz; positive wz reduces q. */
        chassis_align.remaining += chassis_align.direction *
            (rad_format(joint - chassis_align.last_joint) -
             rad_format(target - chassis_align.last_target));
        chassis_align.last_joint = joint;
        chassis_align.last_target = target;
        if (chassis_align.remaining < -CHASSIS_ALIGN_TOL)
        {
            /* 修复：错过捕获窗口时，立即切到 ALIGN_FOLLOW 让 PID 选最短路径。
             * 旧逻辑加 2π 让底盘继续原方向转，会导致"反转一大圈"。 */
            chassis_align.state = ALIGN_FOLLOW;
            chassis_align.direction = 0.0f;
            chassis_align.command = 0.0f;
            chassis_angle_pid_seed = 1;
            return 0.0f;  /* 本帧输出 0，下一帧由 PID 接管 */
        }
        error = -chassis_align.direction * chassis_align.remaining;
        request = -chassis_angle_step(&move->chassis_angle_pid, error);
        distance = fabsf(chassis_align.remaining);
        /* v*dt + v^2/(2*a) <= distance, with the existing PID for the final approach. */
        speed = fmaxf(0.0f, ((chassis_align.remaining >= 0.0f) ?
                            chassis_align.direction : -chassis_align.direction) * move->wz);
        limit = fmaxf(0.0f, distance - CHASSIS_ALIGN_LAG * speed);
        limit = sqrtf(brake_step * brake_step + 2.0f * CHASSIS_ALIGN_BRAKE * limit) - brake_step;
        limit = fp32_constrain(limit, 0.0f, CHASSIS_ALIGN_MAX_WZ);
        request = chassis_align.direction * request;
        if (chassis_align.remaining >= 0.0f)
        {
            request = fp32_constrain(request, 0.0f, limit);
        }
        else
        {
            /* Reverse correction only inside tolerance and after measured braking. */
            limit = fminf(limit, CHASSIS_ALIGN_REVERSE_WZ);
            if (fabsf(chassis_align.command) > CHASSIS_ALIGN_SETTLE_WZ ||
                fabsf(move->wz) > CHASSIS_ALIGN_SETTLE_WZ)
            {
                limit = 0.0f;
            }
            request = fp32_constrain(request, -limit, 0.0f);
        }
        request *= chassis_align.direction;

        if (distance <= CHASSIS_ALIGN_TOL &&
            fabsf(chassis_align.command) <= CHASSIS_ALIGN_SETTLE_WZ &&
            fabsf(move->wz) <= CHASSIS_ALIGN_SETTLE_WZ)
        {
            chassis_align.settled_ms += CHASSIS_CONTROL_TIME_MS;
            if (chassis_align.settled_ms >= CHASSIS_ALIGN_DWELL_MS)
            {
                chassis_align.state = ALIGN_FOLLOW;
                chassis_angle_pid_seed = 1;
            }
        }
        else
        {
            chassis_align.settled_ms = 0;
        }
    }
    else
    {
        /* Avoid a derivative pulse when the shortest error crosses +/-pi. */
        if (fabsf(error - move->chassis_angle_pid.error[0]) > CHASSIS_ALIGN_TWO_PI * 0.5f)
        {
            chassis_angle_pid_seed = 1;
        }
        request = -chassis_angle_step(&move->chassis_angle_pid, error);
    }

    if (chassis_align.state != ALIGN_OFF)
    {
        chassis_align.command += fp32_constrain(request - chassis_align.command, -step, step);
        return chassis_align.command; /* Keep slew limiting after PID handover. */
    }
    return request;
}

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }


/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          041¡è7041¡è7010093041¡è7041¡è7"chassis_move"041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7pid041¡è7041¡è7010093041¡è7041¡è7041¡è7041¡è7 010041041¡è7041¡è7041¡è7041¡è7010028041¡è7041¡è7041¡è7010093041¡è7041¡è7041¡è7041¡è73508041¡è7041¡è7041¡è7010099041¡è7041¡è7010028041¡è7041¡è7041¡è7010093041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010086041¡è7041¡è7041¡è7041¡è7041¡è7010093041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010054010048041¡è7010028041¡è7041¡è7041¡è7010093041¡è7041¡è7
  * @param[out]     chassis_move_init:"chassis_move"041¡è7041¡è7041¡è7041¡è7010028041¡è7041¡è7.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init);

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          041¡è7041¡è7041¡è7010011041¡è7041¡è7010009041¡è7041¡è7041¡è7010050010094041¡è7041¡è7041¡è7041¡è7010048041¡è7041¡è7'chassis_behaviour_mode_set'041¡è7041¡è7041¡è7041¡è7041¡è7¡ì03010063041¡è7
  * @param[out]     chassis_move_mode:"chassis_move"041¡è7041¡è7041¡è7041¡è7010028041¡è7041¡è7.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
  * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
  * @param[out]     chassis_move_transit: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          041¡è7041¡è7041¡è7041¡è7010050010094041¡è7010063024038041¡è7041¡è7¡ì03041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010048041¡è7010063024038041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010009041¡è7041¡è7041¡è7yaw041¡è7010048041¡è7041¡è70305010025010¨¢8041¡è7010009041¡è7010042041¡è7010042041¡è7041¡è7041¡è7041¡è7yaw041¡è7010048041¡è7
  * @param[out]     chassis_move_transit:"chassis_move"041¡è7041¡è7041¡è7041¡è7010028041¡è7041¡è7.
  * @retval         none
  */
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
/**
  * @brief          chassis some measure data updata, such as motor speed, euler angle041¡è7041¡è7 robot speed
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          041¡è7041¡è7041¡è7010096041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7011066041¡è7041¡è7010045041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7011018010¡ì2041¡è7010029041¡è7041¡è7041¡è7010048010¡ì2041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7011018041¡è7
  * @param[out]     chassis_move_update:"chassis_move"041¡è7041¡è7041¡è7041¡è7010028041¡è7041¡è7.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
/**
  * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
  *                 
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          
  * @param[out]     chassis_move_update:"chassis_move"041¡è7041¡è7041¡è7041¡è7010028041¡è7041¡è7.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sentto motor
  * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          041¡è7041¡è7041¡è7041¡è7010097041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7011073041¡è7041¡è7041¡è7041¡è70305010025041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010025041¡è7041¡è7041¡è7041¡è7041¡è7¡ì¨¢041¡è7041¡è7041¡è7
  * @param[out]     chassis_move_control_loop:"chassis_move"041¡è7041¡è7041¡è7041¡è7010028041¡è7041¡è7.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif



//041¡è7041¡è7041¡è7041¡è7041¡è7010036041¡è7041¡è7041¡è7041¡è7041¡è7
chassis_move_t chassis_move;

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7002034041¡è7041¡è7 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 041¡è7041¡è7
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{
    //wait a time 
    //041¡è7041¡è7041¡è7041¡è7010065041¡è7041¡è7010082041¡è7041¡è7
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //chassis init
    //041¡è7041¡è7041¡è7010097041¡è7010093041¡è7041¡è7
    chassis_init(&chassis_move);
    //make sure all chassis motor is online,
    //041¡è7¡ì01010043041¡è7041¡è7010099041¡è7041¡è7041¡è7010049041¡è7041¡è7041¡è7041¡è7041¡è7
    while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE))
    {
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }

    while (toe_is_error(DBUS_TOE) && !robot_comm_is_control_active())
    {
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }

    while (1)
    {
        //set chassis control mode
        //041¡è7041¡è7041¡è7010011041¡è7041¡è7010009041¡è7041¡è7041¡è7010050010094
        chassis_set_mode(&chassis_move);
        //when mode changes, some data save
        //010050010094041¡è7¡ì06041¡è7041¡è7041¡è7041¡è7011¡è9041¡è7041¡è7041¡è7
        chassis_mode_change_control_transit(&chassis_move);
        //chassis data update
        //041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7011066041¡è7041¡è7041¡è7
        chassis_feedback_update(&chassis_move);
        //set chassis control set-point 
        //041¡è7041¡è7041¡è7010009041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7
        chassis_set_contorl(&chassis_move);
        //chassis control pid calculate
        //041¡è7041¡è7041¡è7010009041¡è7041¡è7041¡è7PID041¡è7041¡è7041¡è7041¡è7
        chassis_control_loop(&chassis_move);

        //make sure  one motor is online at least, so that the control CAN message can be received
        //010082041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010065041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7011¡ã3041¡è7 041¡è7041¡è7041¡è7041¡è7CAN041¡è7041¡è7041¡è7010086041¡è7041¡è7041¡è7041¡è7010¨¦3041¡è7041¡è7041¡è7041¡è7010081041¡è7
        if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) && toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE)))
        {
            //when remote control is offline, chassis motor should receive zero current. 
            //041¡è7041¡è7010041041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7011051041¡è7010082041¡è7080093041¡è7041¡è7010066041¡è7041¡è7041¡è7041¡è7010099041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7.
            if (toe_is_error(DBUS_TOE) && !robot_comm_is_control_active())
            {
                CAN_cmd_chassis(0, 0, 0, 0);
            }
            else
            {
                //send control current
                //041¡è7041¡è7041¡è7010073041¡è7041¡è7010091041¡è7041¡è7041¡è7
                CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                                chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
            }
        }
        //os delay
        //010043010061041¡è7041¡è7010082
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

const chassis_move_t *get_chassis_move_point(void)
{
    return &chassis_move;
}

/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          041¡è7041¡è7010093041¡è7041¡è7"chassis_move"041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7pid041¡è7041¡è7010093041¡è7041¡è7041¡è7041¡è7 010041041¡è7041¡è7041¡è7041¡è7010028041¡è7041¡è7041¡è7010093041¡è7041¡è7041¡è7041¡è73508041¡è7041¡è7041¡è7010099041¡è7041¡è7010028041¡è7041¡è7041¡è7010093041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010086041¡è7041¡è7041¡è7041¡è7041¡è7010093041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010054010048041¡è7010028041¡è7041¡è7041¡è7010093041¡è7041¡è7
  * @param[out]     chassis_move_init:"chassis_move"041¡è7041¡è7041¡è7041¡è7010028041¡è7041¡è7.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    //chassis motor speed PID
    //041¡è7041¡è7041¡è7041¡è7041¡è7011018010086041¡è7pid010025
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    
    //chassis angle PID
    //041¡è7041¡è7041¡è7010007010048041¡è7pid010025
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    uint8_t i;

    //in beginning041¡è7041¡è7 chassis mode is raw 
    //041¡è7041¡è7041¡è7010009041¡è7041¡è7041¡è7010¨¹8010090010092010¨¨9010093
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    //get remote control point
    //041¡è7041¡è7010¡ì0010041041¡è7041¡è7041¡è7041¡è7010028041¡è7041¡è7
    chassis_move_init->chassis_RC = get_remote_control_point();
    //get gyro sensor euler angle point
    //041¡è7041¡è7010¡ì0041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010090041¡è7041¡è7010028041¡è7041¡è7
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    //get gimbal motor data point
    //041¡è7041¡è7010¡ì0041¡è7041¡è7010086041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010028041¡è7041¡è7
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
    
    //get chassis motor data point,  initialize motor speed PID
    //041¡è7041¡è7010¡ì0041¡è7041¡è7041¡è7010099041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010028041¡è7030023041¡è7041¡è7010093041¡è7041¡è7PID 
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    //initialize angle PID
    //041¡è7041¡è7010093041¡è7041¡è7041¡è7010048041¡è7PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    
    //first order low-pass filter  replace ramp function
    //041¡è7041¡è7010065041¡è7041¡è7041¡è7010032041¡è7041¡è7041¡è7041¡è7041¡è7¡ì06041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    //max and min speed
    //041¡è7041¡è7041¡è7 041¡è7041¡è7¡ì06041¡è7011018041¡è7
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //update data
    //041¡è7041¡è7041¡è7041¡è7010065041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7
    chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          041¡è7041¡è7041¡è7010011041¡è7041¡è7010009041¡è7041¡è7041¡è7010050010094041¡è7041¡è7041¡è7041¡è7010048041¡è7041¡è7'chassis_behaviour_mode_set'041¡è7041¡è7041¡è7041¡è7041¡è7¡ì03010063041¡è7
  * @param[out]     chassis_move_mode:"chassis_move"041¡è7041¡è7041¡è7041¡è7010028041¡è7041¡è7.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    //in file "chassis_behaviour.c"
    chassis_behaviour_mode_set(chassis_move_mode);
}

/**
  * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
  * @param[out]     chassis_move_transit: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          041¡è7041¡è7041¡è7041¡è7010050010094041¡è7010063024038041¡è7041¡è7¡ì03041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010048041¡è7010063024038041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010009041¡è7041¡è7041¡è7yaw041¡è7010048041¡è7041¡è70305010025010¨¢8041¡è7010009041¡è7010042041¡è7010042041¡è7041¡è7041¡è7041¡è7yaw041¡è7010048041¡è7
  * @param[out]     chassis_move_transit:"chassis_move"041¡è7041¡è7041¡è7041¡è7010028041¡è7041¡è7.
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    PID_clear(&chassis_move_transit->chassis_angle_pid);
    chassis_angle_pid_seed = 1;
    chassis_align.state = ALIGN_OFF;
    if (CHASSIS_ALIGN_ENABLE && chassis_last_frame_in_auto_spin &&
        chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_align.command = chassis_move_transit->wz_set;
        chassis_align.state = ALIGN_INIT;
    }

    //change to follow gimbal angle mode
    //041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010086010050010094
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
    //change to follow chassis yaw angle
    //041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010007010048041¡è7010050010094
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    //change to no follow angle
    //041¡è7041¡è7041¡è7031038041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010086010050010094
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @brief          chassis some measure data updata, such as motor speed, euler angle041¡è7041¡è7 robot speed
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          041¡è7041¡è7041¡è7010096041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7011066041¡è7041¡è7010045041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7011018010¡ì2041¡è7010029041¡è7041¡è7041¡è7010048010¡ì2041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7011018041¡è7
  * @param[out]     chassis_move_update:"chassis_move"041¡è7041¡è7041¡è7041¡è7010028041¡è7041¡è7.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //update motor speed, accel is differential of speed PID
        //041¡è7041¡è7041¡è7010058041¡è7041¡è7041¡è7011018010¡ì2041¡è7041¡è7041¡è7041¡è7011018041¡è7041¡è7041¡è7041¡è7011018010080041¡è7PID010091041¡è7041¡è7
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }

    //calculate vertical speed, horizontal speed ,rotation speed, left hand rule 
    //041¡è7041¡è7041¡è7010058041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7011018041¡è7 x041¡è7041¡è7 010099041¡è7041¡è7041¡è7011018041¡è7y041¡è7041¡è7041¡è7041¡è7010048041¡è7011018041¡è7wz041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010043010092041¡è7041¡è7041¡è7041¡è7010043
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

    //calculate chassis euler angle, if chassis add a new gyro sensor,please change this code
    //041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010090041¡è7010048041¡è7, 041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7018049041¡è7010024041¡è7041¡è7041¡è7
    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - chassis_move_update->chassis_yaw_motor->relative_angle);
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);
    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
}
/**
  * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
  *                 
  * @param[out]     vx_set: vertical speed set-point
  * @param[out]     vy_set: horizontal speed set-point
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          041¡è7041¡è7041¡è7041¡è7010041041¡è7041¡è7041¡è7041¡è7010050041¡è7041¡è7010025041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010068041¡è7041¡è7041¡è7041¡è7011018041¡è7
  *                 
  * @param[out]     vx_set: 041¡è7041¡è7041¡è7041¡è7041¡è7011018041¡è7010028041¡è7041¡è7
  * @param[out]     vy_set: 041¡è7041¡è7041¡è7041¡è7041¡è7011018041¡è7010028041¡è7041¡è7
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" 041¡è7041¡è7041¡è7041¡è7010028041¡è7041¡è7
  * @retval         none
  */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    
    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    //deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
    //041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010073041¡è7041¡è7041¡è7010092010041041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7011008041¡è7041¡è7011078041¡è7041¡è7041¡è7 010039041¡è7041¡è7041¡è7041¡è7041¡è7¡ì07024038041¡è7041¡è7010025041¡è7041¡è70100920
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

    //keyboard set speed set-point
    //041¡è7041¡è7041¡è7010009041¡è7041¡è7041¡è7
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
    }

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
    }

    //first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
    //010065041¡è7010¨¹9041¡è7010050041¡è7010032041¡è7041¡è7041¡è7041¡è7041¡è7¡ì06041¡è7041¡è7041¡è7041¡è7010092041¡è7041¡è7041¡è7041¡è7041¡è7011018041¡è7041¡è7041¡è7041¡è7041¡è7
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
    //stop command, need not slow change, set zero derectly
    //010045010029041¡è7010032010010041¡è7041¡è7041¡è7041¡è7041¡è7010048041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7011099041¡è7010021041¡è7010010041¡è7041¡è7011017041¡è7041¡è7041¡è7
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}
/**
  * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          041¡è7041¡è7041¡è7010011041¡è7041¡è7010009041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010025, 041¡è7041¡è7041¡è7010036041¡è7041¡è7041¡è7041¡è7041¡è7010025041¡è7041¡è7010050041¡è7041¡è7chassis_behaviour_control_set041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010011041¡è7
  * @param[out]     chassis_move_update:"chassis_move"041¡è7041¡è7041¡è7041¡è7010028041¡è7041¡è7.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }


    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    //get three control set-point, 041¡è7041¡è7010¡ì0041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010025
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

    //follow gimbal mode
    //041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010086010050010094
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        //rotate chassis direction, make sure vertial direction follow gimbal 
        //041¡è7041¡è7010048041¡è7041¡è7041¡è7010091041¡è7041¡è7041¡è7041¡è7011018010082041¡è7041¡è7080087041¡è7010008010042041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010086041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010036041¡è7010099041¡è7041¡è7
        sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        //set control relative angle  set-point
        //041¡è7041¡è7041¡è7010017041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010086041¡è7010048041¡è7
        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
        //calculate ratation speed
        //041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010048PID041¡è7041¡è7041¡è7011018041¡è7
        chassis_move_control->wz_set = chassis_follow_yaw_command(chassis_move_control, chassis_move_control->chassis_relative_angle_set);
        //speed limit
        //041¡è7011018041¡è7041¡è7011009041¡è7
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        fp32 delat_angle = 0.0f;
        //set chassis yaw angle set-point
        //041¡è7041¡è7041¡è7010011041¡è7041¡è7010009041¡è7041¡è7010091010075010048041¡è7
        chassis_move_control->chassis_yaw_set = rad_format(angle_set);
        delat_angle = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
        //calculate rotation speed
        //041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010048041¡è7010075041¡è7041¡è7011018041¡è7
        chassis_move_control->wz_set = chassis_angle_step(&chassis_move_control->chassis_angle_pid, delat_angle);
        //speed limit
        //041¡è7011018041¡è7041¡è7011009041¡è7
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        //"angle_set" is rotation speed set-point
        //angle_set 0805 ¡Á08¡Á090902090607010004
        chassis_move_control->wz_set = angle_set;

        // ========== 00020503050208010701010700060605060309080505090604040900¡Á03¡À¨º0308¡Á090303 ==========
        // 08010701080400¡§06030908 yaw 050505¡§0807¡Á080107000606050508+ 08¡Á030006030908 yaw 0505 0404090003¨¤090805050906
        // 080400¡§03¨¤0908070308¡Á0300080205050906 = 080400¡§0603090805050906 - 08¡Á03000603090805050906
        fp32 gimbal_absolute_yaw = chassis_move_control->chassis_yaw_motor->absolute_angle;  // 080400¡§06030908 yaw (010700060605)
        fp32 chassis_absolute_yaw = chassis_move_control->chassis_yaw;                      // 08¡Á030006030908 yaw (010700060605)
        fp32 relative_yaw = rad_format(gimbal_absolute_yaw - chassis_absolute_yaw);          // 03¨¤090805050906

        fp32 sin_yaw = arm_sin_f32(-relative_yaw);
        fp32 cos_yaw = arm_cos_f32(-relative_yaw);
        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        // ========== 0002050305¨¢0803 ============================================

        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        //in raw mode, set-point is sent to CAN bus
        //041¡è7041¡è7010¨¨9010093010050010094041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010025041¡è7010049041¡è7041¡è7010063041¡è7CAN041¡è7041¡è7041¡è7041¡è7
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    }
}

/**
  * @brief          four mecanum wheels speed is calculated by three param. 
  * @param[in]      vx_set: vertial speed
  * @param[in]      vy_set: horizontal speed
  * @param[in]      wz_set: rotation speed
  * @param[out]     wheel_speed: four mecanum wheels speed
  * @retval         none
  */
/**
  * @brief          041¡è7010070041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7011018041¡è7041¡è7041¡è7010050041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7
  * @param[in]      vx_set: 041¡è7041¡è7041¡è7041¡è7041¡è7011018041¡è7
  * @param[in]      vy_set: 041¡è7041¡è7041¡è7041¡è7041¡è7011018041¡è7
  * @param[in]      wz_set: 041¡è7041¡è7010048041¡è7011018041¡è7
  * @param[out]     wheel_speed: 041¡è7010070041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7011018041¡è7
  * @retval         none
  */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    //because the gimbal is in front of chassis, when chassis rotates, wheel 0 and wheel 1 should be slower and wheel 2 and wheel 3 should be faster
    //041¡è7041¡è7010048041¡è7041¡è7010082041¡è7041¡è7 041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010086041¡è7041¡è7010042041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010042041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7 0 041¡è7041¡è71 041¡è7041¡è7010048041¡è7041¡è7041¡è7011018010¡§6041¡è7041¡è7041¡è7041¡è7041¡è7 041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7 2,3 041¡è7041¡è7010048041¡è7041¡è7041¡è7011018010¡§6041¡è7041¡è7
//    wheel_speed[0] = -vx_set - vy_set + (0.05f - 1.0f) * 0.25f * wz_set;
//    wheel_speed[1] = vx_set - vy_set + (0.05f - 1.0f) * 0.22f * wz_set;
//    wheel_speed[2] = vx_set + vy_set + (-0.05f - 1.0f) * 0.22f * wz_set;
//    wheel_speed[3] = -vx_set + vy_set + (-0.05f - 1.0f) * 0.25f * wz_set;
		wheel_speed[0] = -vx_set - vy_set -  MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = vx_set - vy_set -  MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = vx_set + vy_set - MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = -vx_set + vy_set -  MOTOR_DISTANCE_TO_CENTER * wz_set;
}


/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sentto motor
  * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          041¡è7041¡è7041¡è7041¡è7010097041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7011073041¡è7041¡è7041¡è7041¡è70305010025041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010025041¡è7041¡è7041¡è7041¡è7041¡è7¡ì¨¢041¡è7041¡è7041¡è7
  * @param[out]     chassis_move_control_loop:"chassis_move"041¡è7041¡è7041¡è7041¡è7010028041¡è7041¡è7.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;

    //mecanum wheel speed calculation
    //041¡è7041¡è7041¡è7041¡è7041¡è7010036041¡è7041¡è7010¡Â3041¡è7
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
        }
        //in raw mode, derectly return
        //raw041¡è7041¡è7041¡è7041¡è7010021041¡è7010005041¡è7041¡è7041¡è7
        return;
    }

    //calculate the max speed in four wheels, limit the max speed
    //041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7010013041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7011018010¡ì2041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7041¡è7011018041¡è7
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }

    //calculate pid
    //041¡è7041¡è7041¡è7041¡è7pid
    for (i = 0; i < 4; i++)
    {
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }


    //041¡è7041¡è7041¡è7010096041¡è7041¡è7041¡è7
    chassis_power_control(chassis_move_control_loop);


    //041¡è7041¡è7010025041¡è7041¡è7041¡è7041¡è7010025
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
}
