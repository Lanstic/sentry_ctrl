/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      ???????.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ???
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "shoot.h"
#include "main.h"

#include "cmsis_os.h"

#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "detect_task.h"
#include "pid.h"

#define shoot_fric1_on(pwm) fric1_on((pwm)) //?????1pwm????
#define shoot_fric2_on(pwm) fric2_on((pwm)) //?????2pwm????
#define shoot_fric_off()    fric_off()      //????????????

#define shoot_laser_on()    laser_on()      //??????????
#define shoot_laser_off()   laser_off()     //?????????
//???????IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)




/**
  * @brief          ?????????????????????????????????????????1??????1?????????????????????????3min?????????????
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);
/**
  * @brief          ??????????
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          ??????????
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);

/**
  * @brief          ????????????????????????????????
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);



shoot_control_t shoot_control;          //???????


/**
  * @brief          ???????????????PID????????????????
  * @param[in]      void
  * @retval         ?????
  */
void shoot_init(void)
{

    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    shoot_control.shoot_mode = SHOOT_STOP;
    //????????
    shoot_control.shoot_rc = get_remote_control_point();
    //??????
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
    //?????PID
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    //????????
    shoot_feedback_update();
    ramp_init(&shoot_control.fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN, FRIC_OFF);
    ramp_init(&shoot_control.fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN, FRIC_OFF);
    shoot_control.fric_pwm1 = FRIC_OFF;
    shoot_control.fric_pwm2 = FRIC_OFF;
    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.set_angle = shoot_control.angle;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
    shoot_control.trigger_motor_was_offline = 0;
    shoot_control.fric_need_restart = 0;
}

/**
  * @brief          ??????
  * @param[in]      void
  * @retval         ????can?????
  */
int16_t shoot_control_loop(void)
{

    shoot_set_mode();        //????????
    shoot_feedback_update(); //????????

    // ========== 检测发射机构断电重启 ==========
    // 通过拨弹轮CAN在线状态判断发射机构是否重新上电
    uint8_t trigger_is_offline = toe_is_error(TRIGGER_MOTOR_TOE);
    if (shoot_control.trigger_motor_was_offline && !trigger_is_offline)
    {
        // 拨弹轮从离线变为在线，说明发射机构重新上电了
        // 需要：1.重置摩擦轮 ramp，让摩擦轮重新启动
        //      2.强制停止发射，等待右拨杆重新操作
        shoot_control.fric_need_restart = 1;
        shoot_control.shoot_mode = SHOOT_STOP;  // 强制停止，等待重新启动
    }
    shoot_control.trigger_motor_was_offline = trigger_is_offline;

    // 如果需要重启摩擦轮，重置 ramp 到初始状态
    if (shoot_control.fric_need_restart)
    {
        ramp_init(&shoot_control.fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN, FRIC_OFF);
        ramp_init(&shoot_control.fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN, FRIC_OFF);
        shoot_control.fric_need_restart = 0;
    }


    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        //??????????????
        shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
    {
        //??????????????
        shoot_control.speed_set = 0.0f;
    }
    else if(shoot_control.shoot_mode ==SHOOT_READY_BULLET)
    {
        shoot_control.trigger_speed_set = 0.0f;
        shoot_control.speed_set = 0.0f;
        shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY)
    {
        //??????????????
         shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_BULLET)
    {
        shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control();
    }
    else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
    {
        trigger_motor_turn_back();
    }
    else if(shoot_control.shoot_mode == SHOOT_DONE)
    {
        shoot_control.speed_set = 0.0f;
    }

    if(shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_laser_off();
        shoot_control.given_current = 0;
        //?????????????????????????????????????????????????
        ramp_calc(&shoot_control.fric1_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
        ramp_calc(&shoot_control.fric2_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
    }
    else
    {
        shoot_laser_on(); //??????
        //??????????PID
        PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
        shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
        if(shoot_control.shoot_mode < SHOOT_READY_BULLET)
        {
            shoot_control.given_current = 0;
        }
        //?????????????????????????????????????????????????
        ramp_calc(&shoot_control.fric1_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
        ramp_calc(&shoot_control.fric2_ramp, SHOOT_FRIC_PWM_ADD_VALUE);

    }

    shoot_control.fric_pwm1 = (uint16_t)(shoot_control.fric1_ramp.out);
    shoot_control.fric_pwm2 = (uint16_t)(shoot_control.fric2_ramp.out);
    shoot_fric1_on(shoot_control.fric_pwm1);
    shoot_fric2_on(shoot_control.fric_pwm2);
    return shoot_control.given_current;
}

/**
  * @brief          ???????????
  *                 ?????????
  *                 - ????????????????????
  *                 - ?????????????????????????
  *                 - ?????????
  *                 ??????????????????????
  *                 - Q???????????
  *                 - E??????????
  *                 - ????????????????
  *                 - ????????????????
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void)
{
    static uint8_t keyboard_control_fric = 0;  //键盘控制摩擦轮标志

    //获取遥控器拨杆状态
    int8_t rc_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
    int8_t rc_s_left = shoot_control.shoot_rc->rc.s[0];   //左拨杆

    //=================== Q/E键控制摩擦轮 ===================
    //Q键开启摩擦轮
    if (shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD)
    {
        keyboard_control_fric = 1;
        shoot_control.shoot_mode = SHOOT_READY_FRIC;
    }
    //E键关闭摩擦轮
    else if (shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD)
    {
        keyboard_control_fric = 1;
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    //如果左拨杆不在下位，重置键盘控制标志
    if (keyboard_control_fric && !switch_is_down(rc_s_left))
    {
        keyboard_control_fric = 0;
    }

    //=================== 遥控器模式控制（键盘控制未激活时） ===================
    if (keyboard_control_fric == 0)
    {
        //右拨杆下 = 停止
        if (switch_is_down(rc_s))
        {
            shoot_control.shoot_mode = SHOOT_STOP;
        }
        //右拨杆中/上 = 摩擦轮启动
        else if (switch_is_mid(rc_s) || switch_is_up(rc_s))
        {
            shoot_control.shoot_mode = SHOOT_READY_FRIC;
        }
    }

    //=================== 摩擦轮启动后自动进入拨弹轮准备状态 ===================
    //当摩擦轮PWM达到1500以上时，认为摩擦轮已准备好
    if(shoot_control.shoot_mode == SHOOT_READY_FRIC &&
       shoot_control.fric1_ramp.out >= 1500 &&
       shoot_control.fric2_ramp.out >= 1500)
    {
        shoot_control.shoot_mode = SHOOT_READY_BULLET;
    }

    //=================== 拨弹轮控制 ===================
    //支持两种方式控制拨弹：鼠标左键 或 遥控器右拨杆上拨
    //重要：必须等摩擦轮准备就绪后才能拨弹
    if(shoot_control.shoot_mode >= SHOOT_READY_BULLET)
    {
        // 检查摩擦轮是否准备就绪（PWM达到1500以上，认为摩擦轮已启动）
        uint8_t fric_ready = (shoot_control.fric1_ramp.out >= 1500 &&
                              shoot_control.fric2_ramp.out >= 1500);

        // 鼠标右键按下且摩擦轮就绪，拨弹轮反转
        if (fric_ready && shoot_control.press_r)
        {
            shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
            shoot_control.trigger_speed_set = -4.0f;  // 反转速度
        }
        // 鼠标左键按下 或 右拨杆上拨，且摩擦轮就绪，拨弹轮转动（连发模式）
        else if (fric_ready && (shoot_control.press_l || switch_is_up(rc_s)))
        {
            shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
            shoot_control.trigger_speed_set = CONTINUE_TRIGGER_SPEED;  // 正转速度
        }
        // 停止拨弹
        else if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            shoot_control.shoot_mode = SHOOT_READY_BULLET;
            shoot_control.block_time = 0;
            shoot_control.reverse_time = 0;
            shoot_control.trigger_speed_set = CONTINUE_TRIGGER_SPEED;  // 重置为正数
        }
    }

    //=================== 热量限制 ===================
    get_shoot_heat0_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);
    if(!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.heat_limit))
    {
        if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            shoot_control.shoot_mode = SHOOT_READY_BULLET;
        }
    }

    //=================== 云台停止命令 ===================
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
        keyboard_control_fric = 0;
    }
}
/**
  * @brief          ??????????
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //?????????????????
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //?????????
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;

    //??????????? ??????????????? ???????? 36???????????????????????????????????????????
    if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        shoot_control.ecd_count--;
    }
    else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        shoot_control.ecd_count++;
    }

    if (shoot_control.ecd_count == FULL_COUNT)
    {
        shoot_control.ecd_count = -(FULL_COUNT - 1);
    }
    else if (shoot_control.ecd_count == -FULL_COUNT)
    {
        shoot_control.ecd_count = FULL_COUNT - 1;
    }

    //???????????
    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
    //???????
    shoot_control.key = BUTTEN_TRIG_PIN;
    //?????
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
    //???????
    if (shoot_control.press_l)
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME)
        {
            shoot_control.press_l_time++;
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }

    if (shoot_control.press_r)
    {
        if (shoot_control.press_r_time < PRESS_LONG_TIME)
        {
            shoot_control.press_r_time++;
        }
    }
    else
    {
        shoot_control.press_r_time = 0;
    }

    //???????????????
    if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {

        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time++;
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }

}

static void trigger_motor_turn_back(void)
{
    // 如果是手动设置的反转速度（负值），直接使用，不应用卡弹检测逻辑
    if (shoot_control.trigger_speed_set < 0)
    {
        shoot_control.speed_set = shoot_control.trigger_speed_set;
        return;
    }

    if( shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.speed_set = shoot_control.trigger_speed_set;
    }
    else
    {
        shoot_control.speed_set = -shoot_control.trigger_speed_set;
    }

    if(fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;
    }
    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;
    }
    else
    {
        shoot_control.block_time = 0;
    }
}

/**
  * @brief          ????????????????????????????????
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{

    //??????? 1/4PI????
    if (shoot_control.move_flag == 0)
    {
        shoot_control.set_angle = rad_format(shoot_control.angle + PI_TEN);
        shoot_control.move_flag = 1;
    }
    if(shoot_control.key == SWITCH_TRIGGER_OFF)
    {

        shoot_control.shoot_mode = SHOOT_DONE;
    }
    //??????????
    if (rad_format(shoot_control.set_angle - shoot_control.angle) > 0.05f)
    {
        //?????????????????
        shoot_control.trigger_speed_set = TRIGGER_SPEED;
        trigger_motor_turn_back();
    }
    else
    {
        shoot_control.move_flag = 0;
    }
}

