  /**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      according to remote control, change the chassis behaviour.
  *             ???????????????????????????
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================
    add a chassis behaviour mode
    1. in chassis_behaviour.h , add a new behaviour name in chassis_behaviour
    erum
    {  
        ...
        ...
        CHASSIS_XXX_XXX, // new add
    }chassis_behaviour_e,
    2. implement new function. chassis_xxx_xxx_control(fp32 *vx, fp32 *vy, fp32 *wz, chassis_move_t * chassis )
        "vx, vy, wz" param is chassis movement contorl input. 
        first param: 'vx' usually means  vertical speed,
            positive value means forward speed, negative value means backward speed.
        second param: 'vy' usually means horizotal speed,
            positive value means letf speed, negative value means right speed
        third param: 'wz' can be rotation speed set or angle set, 

        in this new function, you can assign speed to "vx","vy",and "wz",as your wish
    3.  in "chassis_behaviour_mode_set" function, add new logical judgement to assign CHASSIS_XXX_XXX to  "chassis_behaviour_mode" variable,
        and in the last of the function, add "else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)" 
        choose a chassis control mode.
        four mode:
        CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW : 'vx' and 'vy' are speed control, 'wz' is angle set to control relative angle
            between chassis and gimbal. you can name third param to 'xxx_angle_set' other than 'wz'
        CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW : 'vx' and 'vy' are speed control, 'wz' is angle set to control absolute angle calculated by gyro
            you can name third param to 'xxx_angle_set.
        CHASSIS_VECTOR_NO_FOLLOW_YAW : 'vx' and 'vy' are speed control, 'wz' is rotation speed control.
        CHASSIS_VECTOR_RAW : will use 'vx' 'vy' and 'wz'  to linearly calculate four wheel current set, 
            current set will be derectly sent to can bus.
    4. in the last of "chassis_behaviour_control_set" function, add
        else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)
        {
            chassis_xxx_xxx_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
        }

        
    ???????????????????
    1.???????chassis_behaviour.h??????? ?????????????????? chassis_behaviour_e
    erum
    {  
        ...
        ...
        CHASSIS_XXX_XXX, // ???????
    }chassis_behaviour_e,

    2. ???????????? chassis_xxx_xxx_control(fp32 *vx, fp32 *vy, fp32 *wz, chassis_move_t * chassis )
        "vx,vy,wz" ??????????????????????
        ?????????: 'vx' ??????????????,??? ????? ??? ????
        ?????????: 'vy' ?????????????,??? ????, ??? ????
        ??????????: 'wz' ???????????????????????
        ???????????, ????? "vx","vy",and "wz" ??????????????
    3.  ??"chassis_behaviour_mode_set"????????????????????????????chassis_behaviour_mode?????CHASSIS_XXX_XXX
        ????????????"else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)" ,?????????????????
        4??:
        CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW : 'vx' and 'vy'????????? 'wz'??????? ??????????????
        ???????????"xxx_angle_set"??????'wz'
        CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW : 'vx' and 'vy'????????? 'wz'??????? ??????????????????????
        ???????????"xxx_angle_set"
        CHASSIS_VECTOR_NO_FOLLOW_YAW : 'vx' and 'vy'????????? 'wz'???????????
        CHASSIS_VECTOR_RAW : ???'vx' 'vy' and 'wz'????????????????????????????????????can ??????
    4.  ??"chassis_behaviour_control_set" ?????????????
        else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)
        {
            chassis_xxx_xxx_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
        }
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "arm_math.h"

#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "robot_comm.h"

//软启动相关：记录上一帧是否在小陀螺模式（用于检测上升沿）
uint8_t chassis_last_frame_in_auto_spin = 0;

// 陀螺仪离线检测相关（用于检测底盘CAN断电重启）
// 初始化为1，确保系统启动时电机第一次上线也会触发软启动
static uint8_t chassis_motor1_was_offline = 1;

// 跟随云台旋转软启动相关
static uint8_t chassis_last_frame_in_follow_gimbal = 0;

/**
  * @brief          when chassis behaviour mode is CHASSIS_ZERO_FORCE, the function is called
  *                 and chassis control mode is raw. The raw chassis control mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all speed zero.
  * @param[out]     vx_can_set: vx speed value, it will be sent to CAN bus derectly.
  * @param[out]     vy_can_set: vy speed value, it will be sent to CAN bus derectly.
  * @param[out]     wz_can_set: wz rotate speed value, it will be sent to CAN bus derectly.
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ????????????????????????????raw?????????????????can????????????????????0
  * @author         RM
  * @param[in]      vx_set???????? ????????????can??????
  * @param[in]      vy_set???????? ????????????can??????
  * @param[in]      wz_set???????? ????????????can??????
  * @param[in]      chassis_move_rc_to_vector????????
  * @retval         ?????
  */
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);


/**
  * @brief          when chassis behaviour mode is CHASSIS_NO_MOVE, chassis control mode is speed control mode.
  *                 chassis does not follow gimbal, and the function will set all speed zero to make chassis no move
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: wz rotate speed value, positive value means counterclockwise , negative value means clockwise.
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ????????????????????????????????????
  * @author         RM
  * @param[in]      vx_set????????,??? ??????? ??? ???????
  * @param[in]      vy_set????????,??? ???????? ??? ???????
  * @param[in]      wz_set???????????????????????????????
  * @param[in]      chassis_move_rc_to_vector????????
  * @retval         ?????
  */
static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          when chassis behaviour mode is CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW, chassis control mode is speed control mode.
  *                 chassis will follow gimbal, chassis rotation speed is calculated from the angle difference.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     angle_set: control angle difference between chassis and gimbal
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ????????????????????????????????????????????????????????????????????????
  * @author         RM
  * @param[in]      vx_set????????,??? ??????? ??? ???????
  * @param[in]      vy_set????????,??? ???????? ??? ???????
  * @param[in]      angle_set?????????????????????
  * @param[in]      chassis_move_rc_to_vector????????
  * @retval         ?????
  */
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          when chassis behaviour mode is CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, chassis control mode is speed control mode.
  *                 chassis will follow chassis yaw, chassis rotation speed is calculated from the angle difference between set angle and chassis yaw.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     angle_set: control angle[-PI, PI]
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ??????????yaw?????????????????????????????????????????????????????????????
  * @author         RM
  * @param[in]      vx_set????????,??? ??????? ??? ???????
  * @param[in]      vy_set????????,??? ???????? ??? ???????
  * @param[in]      angle_set?????????yaw?????? -PI??PI
  * @param[in]      chassis_move_rc_to_vector????????
  * @retval         ?????
  */
static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          when chassis behaviour mode is CHASSIS_NO_FOLLOW_YAW, chassis control mode is speed control mode.
  *                 chassis will no follow angle, chassis rotation speed is set by wz_set.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ??????????????????????????????????????????????????????????
  * @author         RM
  * @param[in]      vx_set????????,??? ??????? ??? ???????
  * @param[in]      vy_set????????,??? ???????? ??? ???????
  * @param[in]      wz_set???????????????,??? ????????????? ???????
  * @param[in]      chassis_move_rc_to_vector????????
  * @retval         ?????
  */
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          when chassis behaviour mode is CHASSIS_AUTO_SPIN, chassis will spin automatically
 *                 while gimbal keeps global angle. chassis rotation speed is constant.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     wz_set: wz rotate speed, positive value means counterclockwise, negative value means clockwise
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
static void chassis_auto_spin_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

static void chassis_jetson_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);



/**
  * @brief          when chassis behaviour mode is CHASSIS_OPEN, chassis control mode is raw control mode.
  *                 set value will be sent to can bus.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ???????????????????????????raw??????????????????????can??????
  * @param[in]      vx_set????????,??? ??????? ??? ???????
  * @param[in]      vy_set???????????? ???????? ??? ???????
  * @param[in]      wz_set ??????? ??? ????????????? ???????
  * @param[in]      chassis_move_rc_to_vector????????
  * @retval         none
  */

static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);






//highlight, the variable chassis behaviour mode
//????????????????????
chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;

// ========== 统一位移软启动参数 ==========
#define MOVE_SOFT_START_TIME_MS 80    // 位移软启动时间(ms)
#define MOVE_SOFT_START_POWER 2.0f     // 位移软启动幂次
#define MOVE_SOFT_MIN_FACTOR 0.4f      // 启动时最小系数

// 统一位移软启动变量
static uint32_t normal_mode_move_soft_start_tick = 0;
static fp32 normal_mode_last_vx = 0.0f;
static fp32 normal_mode_last_vy = 0.0f;


/**
  * @brief          logical judgement to assign "chassis_behaviour_mode" variable to which mode
  * @param[in]      chassis_move_mode: chassis data
  * @retval         none
  */
/**
  * @brief          ??????????????"chassis_behaviour_mode"????????
  * @param[in]      chassis_move_mode: ????????
  * @retval         none
  */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    //软启动相关：在模式切换前保存当前状态（上一帧是否在小陀螺模式）
    chassis_last_frame_in_auto_spin = (chassis_behaviour_mode == CHASSIS_AUTO_SPIN) ? 1 : 0;

    if (toe_is_lost(DBUS_TOE) &&
        robot_comm_is_control_active() &&
        !gimbal_cmd_to_chassis_stop_allow_jetson())
    {
        chassis_behaviour_mode = CHASSIS_JETSON;
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
        return;
    }

    //点动触发：记录上一帧Ctrl+按键状态，用于检测上升沿
    static uint8_t last_ctrl_z_pressed = 0;
    static uint8_t last_ctrl_x_pressed = 0;
    static uint8_t last_ctrl_c_pressed = 0;

    //检测键盘Ctrl+Z/X/C切换底盘模式(与云台同步控制)
    //条件：左右拨杆都为下（都处于下拨位置）时才允许键盘控制
    //Ctrl+Z = 底盘自动旋转模式 (CHASSIS_AUTO_SPIN)
    //Ctrl+X = 底盘跟随云台模式 (CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    //Ctrl+C = 底盘无力模式 (CHASSIS_ZERO_FORCE)
    uint8_t left_switch_down = switch_is_down(chassis_move_mode->chassis_RC->rc.s[0]);  //左拨杆
    uint8_t right_switch_down = switch_is_down(chassis_move_mode->chassis_RC->rc.s[1]); //右拨杆
    
    //当左右拨杆都为下时，允许键盘介入控制底盘
    if (left_switch_down && right_switch_down)
    {
        //获取当前Ctrl+Z/X/C按键状态
        uint8_t ctrl_z_now = (chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_Z) && 
                              (chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_CTRL);
        uint8_t ctrl_x_now = (chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_X) && 
                              (chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_CTRL);
        uint8_t ctrl_c_now = (chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_C) && 
                              (chassis_move_mode->chassis_RC->key.v & KEY_PRESSED_OFFSET_CTRL);
        
        //点动触发：检测上升沿（从无按键变为有按键）
        if (ctrl_z_now && !last_ctrl_z_pressed)
        {
            chassis_behaviour_mode = CHASSIS_AUTO_SPIN;  //Ctrl+Z: 自动旋转模式
        }
        else if (ctrl_x_now && !last_ctrl_x_pressed)
        {
            chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;  //Ctrl+X: 跟随云台模式
        }
        else if (ctrl_c_now && !last_ctrl_c_pressed)
        {
            chassis_behaviour_mode = CHASSIS_ZERO_FORCE;  //Ctrl+C: 无力模式
        }
        //其他情况保持当前模式不变
        
        //更新上一帧状态
        last_ctrl_z_pressed = ctrl_z_now;
        last_ctrl_x_pressed = ctrl_x_now;
        last_ctrl_c_pressed = ctrl_c_now;
    }
    else
    {
        //非左右拨杆都为下时，重置点动状态
        last_ctrl_z_pressed = 0;
        last_ctrl_x_pressed = 0;
        last_ctrl_c_pressed = 0;
        
        //使用拨杆控制
        if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
        {
            chassis_behaviour_mode = CHASSIS_NO_MOVE;
        }
        else if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
        {
            chassis_behaviour_mode = CHASSIS_AUTO_SPIN;
        }
        else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
        {
            chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
        }
    }

    //云台状态决定底盘是否停止（进入无力模式）
    if (gimbal_cmd_to_chassis_stop())
    {
        chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    }

    //add your own logic to enter the new mode
    //???????????????????????


    //accord to beheviour mode, choose chassis control mode
    //????????????????????????
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW; 
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; 
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW; 
    }
    else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_AUTO_SPIN)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
}


/**
  * @brief          set control set-point. three movement param, according to difference control mode,
  *                 will control corresponding movement.in the function, usually call different control function.
  * @param[out]     vx_set, usually controls vertical speed.
  * @param[out]     vy_set, usually controls horizotal speed.
  * @param[out]     wz_set, usually controls rotation speed.
  * @param[in]      chassis_move_rc_to_vector,  has all data of chassis
  * @retval         none
  */
/**
  * @brief          ?????????.???????????????????????????????????.??????????????????????????????.
  * @param[out]     vx_set, ??????????????.
  * @param[out]     vy_set, ?????????????.
  * @param[out]     wz_set, ?????????????.
  * @param[in]      chassis_move_rc_to_vector,  ???????????????.
  * @retval         none
  */

void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_no_move_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
        chassis_engineer_follow_chassis_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {
        chassis_open_set_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_JETSON)
    {
        chassis_jetson_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_AUTO_SPIN)
    {
        chassis_auto_spin_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }

    // ========== 统一位移软启动（适用于所有模式） ==========
    // 检测WASD按下/松开，开始/结束位移软启动（vx和vy共用一个计时器）
    uint8_t has_vx_input = (fabsf(*vx_set) > 0.01f);
    uint8_t has_vy_input = (fabsf(*vy_set) > 0.01f);
    uint8_t has_any_input = has_vx_input || has_vy_input;

    if (has_any_input)
    {
        uint8_t was_no_input = (fabsf(normal_mode_last_vx) < 0.01f) && (fabsf(normal_mode_last_vy) < 0.01f);
        if (was_no_input)
        {
            normal_mode_move_soft_start_tick = osKernelSysTick();
        }
    }

    // 计算位移软启动系数
    fp32 move_soft_factor = 1.0f;
    if (has_any_input)
    {
        uint32_t move_elapsed_ms = 0;
        uint32_t current_tick = osKernelSysTick();
        if (current_tick >= normal_mode_move_soft_start_tick)
        {
            move_elapsed_ms = (current_tick - normal_mode_move_soft_start_tick) * 1000 / configTICK_RATE_HZ;
        }

        if (move_elapsed_ms < MOVE_SOFT_START_TIME_MS)
        {
            fp32 progress = (fp32)move_elapsed_ms / MOVE_SOFT_START_TIME_MS;
            move_soft_factor = MOVE_SOFT_MIN_FACTOR + (1.0f - MOVE_SOFT_MIN_FACTOR) * powf(progress, MOVE_SOFT_START_POWER);
        }
    }

    // 应用位移软启动系数
    *vx_set *= move_soft_factor;
    *vy_set *= move_soft_factor;

    // 保存上次的输入值
    normal_mode_last_vx = *vx_set;
    normal_mode_last_vy = *vy_set;
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_ZERO_FORCE, the function is called
  *                 and chassis control mode is raw. The raw chassis chontrol mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all speed zero.
  * @param[out]     vx_can_set: vx speed value, it will be sent to CAN bus derectly.
  * @param[out]     vy_can_set: vy speed value, it will be sent to CAN bus derectly.
  * @param[out]     wz_can_set: wz rotate speed value, it will be sent to CAN bus derectly.
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ????????????????????????????raw?????????????????can????????????????????0
  * @author         RM
  * @param[in]      vx_set???????? ????????????can??????
  * @param[in]      vy_set???????? ????????????can??????
  * @param[in]      wz_set???????? ????????????can??????
  * @param[in]      chassis_move_rc_to_vector????????
  * @retval         ?????
  */

static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_NO_MOVE, chassis control mode is speed control mode.
  *                 chassis does not follow gimbal, and the function will set all speed zero to make chassis no move
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: wz rotate speed value, positive value means counterclockwise , negative value means clockwise.
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ????????????????????????????????????
  * @author         RM
  * @param[in]      vx_set????????,??? ??????? ??? ???????
  * @param[in]      vy_set????????,??? ???????? ??? ???????
  * @param[in]      wz_set???????????????????????????????
  * @param[in]      chassis_move_rc_to_vector????????
  * @retval         ?????
  */

static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW, chassis control mode is speed control mode.
  *                 chassis will follow gimbal, chassis rotation speed is calculated from the angle difference.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     angle_set: control angle difference between chassis and gimbal
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ????????????????????????????????????????????????????????????????????????
  * @author         RM
  * @param[in]      vx_set????????,??? ??????? ??? ???????
  * @param[in]      vy_set????????,??? ???????? ??? ???????
  * @param[in]      angle_set?????????????????????
  * @param[in]      chassis_move_rc_to_vector????????
  * @retval         ?????
  */

static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    // 更新上一帧状态（在函数开头，在其他逻辑之前）
    chassis_last_frame_in_follow_gimbal = (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW) ? 1 : 0;

    //channel value and keyboard value change to speed set-point, in general
    //????????????????????? ???? ???????????????
    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    // ========== 跟随云台旋转软启动 ==========
    // 用于解决从小陀螺切换到跟随云台模式时，由于云台朝向与底盘朝向差异大，
    // 导致底盘瞬间反转跟随，功率暴涨的问题
    #define FOLLOW_GIMBAL_ROTATE_SOFT_START_TIME_MS 300  // 旋转软启动时间(ms)
    #define FOLLOW_GIMBAL_ROTATE_SOFT_START_POWER 2.0f   // 幂次

    static uint32_t follow_gimbal_rotate_soft_start_tick = 0;
    static uint8_t first_frame_in_follow_gimbal = 0;

    uint8_t current_is_follow_gimbal = (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW) ? 1 : 0;

    // 检测刚进入跟随云台模式（此时chassis_last_frame_in_follow_gimbal还是上一帧的值）
    if (current_is_follow_gimbal && !chassis_last_frame_in_follow_gimbal)
    {
        follow_gimbal_rotate_soft_start_tick = osKernelSysTick();
        first_frame_in_follow_gimbal = 1;
    }
    else
    {
        first_frame_in_follow_gimbal = 0;
    }

    fp32 rotate_factor = 1.0f;
    if (first_frame_in_follow_gimbal || (current_is_follow_gimbal && follow_gimbal_rotate_soft_start_tick != 0))
    {
        uint32_t current_tick = osKernelSysTick();
        uint32_t elapsed_ms = 0;

        if (current_tick >= follow_gimbal_rotate_soft_start_tick)
        {
            elapsed_ms = (current_tick - follow_gimbal_rotate_soft_start_tick) * 1000 / configTICK_RATE_HZ;
        }

        if (elapsed_ms < FOLLOW_GIMBAL_ROTATE_SOFT_START_TIME_MS)
        {
            fp32 progress = (fp32)elapsed_ms / FOLLOW_GIMBAL_ROTATE_SOFT_START_TIME_MS;
            rotate_factor = powf(progress, FOLLOW_GIMBAL_ROTATE_SOFT_START_POWER);
        }
        else
        {
            rotate_factor = 1.0f;
            follow_gimbal_rotate_soft_start_tick = 0;  // 软启动完成
        }
    }

    //swing angle is generated by sin function, swing_time is the input time of sin
    //???????????sin?????????swing_time ??sin???????????
    static fp32 swing_time = 0.0f;
    
    static fp32 swing_angle = 0.0f;
    //max_angle is the max angle that chassis will ratate
    //max_angle ??sin????????
    static fp32 max_angle = SWING_NO_MOVE_ANGLE;
    //swing_time  plus the add_time in one control cycle
    //swing_time ???????????????????? add_time
    static fp32 const add_time = PI * 0.5f * configTICK_RATE_HZ / CHASSIS_CONTROL_TIME_MS;
    
    static uint8_t swing_flag = 0;

    //judge if swing
    //???????????
    if (chassis_move_rc_to_vector->chassis_RC->key.v & SWING_KEY)
    {
        if (swing_flag == 0)
        {
            swing_flag = 1;
            swing_time = 0.0f;
        }
    }
    else
    {
        swing_flag = 0;
    }

    //judge if keyboard is controlling the chassis, if yes, reduce the max_angle
    //???????????????????????????????????????????????
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY ||
        chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        max_angle = SWING_MOVE_ANGLE;
    }
    else
    {
        max_angle = SWING_NO_MOVE_ANGLE;
    }
    
    if (swing_flag)
    {
        swing_angle = max_angle * arm_sin_f32(swing_time);
        swing_time += add_time;
    }
    else
    {
        swing_angle = 0.0f;
    }
    //swing_time  range [0, 2*PI]
    //sin??????????2pi
    if (swing_time > 2 * PI)
    {
        swing_time -= 2 * PI;
    }


    *angle_set = swing_angle * rotate_factor;
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, chassis control mode is speed control mode.
  *                 chassis will follow chassis yaw, chassis rotation speed is calculated from the angle difference between set angle and chassis yaw.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     angle_set: control angle[-PI, PI]
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ??????????yaw?????????????????????????????????????????????????????????????
  * @author         RM
  * @param[in]      vx_set????????,??? ??????? ??? ???????
  * @param[in]      vy_set????????,??? ???????? ??? ???????
  * @param[in]      angle_set?????????yaw?????? -PI??PI
  * @param[in]      chassis_move_rc_to_vector????????
  * @retval         ?????
  */

static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    *angle_set = rad_format(chassis_move_rc_to_vector->chassis_yaw_set - CHASSIS_ANGLE_Z_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL]);
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_NO_FOLLOW_YAW, chassis control mode is speed control mode.
  *                 chassis will no follow angle, chassis rotation speed is set by wz_set.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ??????????????????????????????????????????????????????????
  * @author         RM
  * @param[in]      vx_set????????,??? ??????? ??? ???????
  * @param[in]      vy_set????????,??? ???????? ??? ???????
  * @param[in]      wz_set???????????????,??? ????????????? ???????
  * @param[in]      chassis_move_rc_to_vector????????
  * @retval         ?????
  */

static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *wz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
}

/**
 * @brief          when chassis behaviour mode is CHASSIS_AUTO_SPIN, chassis will spin automatically
 *                 while gimbal keeps global angle. chassis rotation speed is constant.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     wz_set: wz rotate speed, positive value means counterclockwise, negative value means clockwise
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
static void chassis_auto_spin_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    // 自动旋转速度目标值
    #define AUTO_SPIN_SPEED_TARGET 9.5f
    // 软启动时间（毫秒），可根据实际情况调整
    #define AUTO_SPIN_SOFT_START_TIME_MS 1200
    // 幂次，值越大曲线越陡（开始越慢，后面越快）
    #define AUTO_SPIN_SOFT_START_POWER 3.0f

    // 位移软启动参数（使用统一宏定义，已在chassis_behaviour_control_set末尾统一处理）

    // 软启动相关变量（自转）
    static uint32_t soft_start_begin_tick = 0;
    static uint32_t soft_start_enter_count = 0;
    static uint8_t first_frame_in_auto_spin = 0;  // 标记是否是小陀螺模式的第一帧

    uint8_t current_is_auto_spin = (chassis_behaviour_mode == CHASSIS_AUTO_SPIN) ? 1 : 0;

    // ========== 底盘电机离线检测：断电重启后重新软启动 ==========
    uint8_t motor1_is_offline = toe_is_error(CHASSIS_MOTOR1_TOE);
    if (chassis_motor1_was_offline && !motor1_is_offline)
    {
        // 底盘电机从离线变为在线，说明底盘CAN重新上电了，需要重新软启动
        soft_start_begin_tick = osKernelSysTick();
        // 重置统一位移软启动计时器
        normal_mode_move_soft_start_tick = osKernelSysTick();
        normal_mode_last_vx = 0.0f;
        normal_mode_last_vy = 0.0f;
    }
    chassis_motor1_was_offline = motor1_is_offline;

    // 检测刚进入小陀螺模式（使用全局变量，确保在所有模式下都能正确跟踪状态）
    if (current_is_auto_spin && !chassis_last_frame_in_auto_spin)
    {
        // 每次进入都重置自转软启动
        soft_start_begin_tick = osKernelSysTick();
        // 重置统一位移软启动计时器
        normal_mode_move_soft_start_tick = osKernelSysTick();
        normal_mode_last_vx = 0.0f;
        normal_mode_last_vy = 0.0f;
        first_frame_in_auto_spin = 1;  // 标记为第一帧
        soft_start_enter_count++;
    }
    else
    {
        first_frame_in_auto_spin = 0;
    }

    // 自动旋转速度范围 [1.5, 8.0] rad/s（软启动后生效）
    #define AUTO_SPIN_SPEED_MAX 9.5f
    #define AUTO_SPIN_SPEED_MIN 8.5f
    #define AUTO_SPIN_DECAY_POWER 0.4f  // 幂次，越大曲线后半段越陡

    // 计算软启动进度
    fp32 spin_speed = 0.0f;
    uint8_t in_soft_start = 0;

    if (current_is_auto_spin)
    {
        uint32_t current_tick = osKernelSysTick();
        uint32_t elapsed_ms = 0;

        if (current_tick >= soft_start_begin_tick)
        {
            elapsed_ms = (current_tick - soft_start_begin_tick) * 1000 / configTICK_RATE_HZ;
        }

        if (elapsed_ms < AUTO_SPIN_SOFT_START_TIME_MS)
        {
            // 软启动中
            in_soft_start = 1;
            fp32 progress = (fp32)elapsed_ms / AUTO_SPIN_SOFT_START_TIME_MS;
            spin_speed = AUTO_SPIN_SPEED_TARGET * powf(progress, AUTO_SPIN_SOFT_START_POWER);
        }
        else
        {
            // 软启动完成
            spin_speed = AUTO_SPIN_SPEED_TARGET;
        }
    }

    // 获取遥控器位移需求
    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    // 注意：位移软启动已在 chassis_behaviour_control_set 末尾统一处理

    // ========== 自转软启动（与位移独立） ==========
    if (in_soft_start)
    {
        // 软启动期间旋转速度使用软启动曲线
        *wz_set = spin_speed;
    }
    else
    {
        // 软启动完成后：使用原有逻辑计算自转速度

        // 计算位移速度的大小
        fp32 move_speed = sqrtf((*vx_set) * (*vx_set) + (*vy_set) * (*vy_set));

        // 使用幂函数衰减计算自转速度
        // 无位移时 factor = 1.0，位移越大 factor 越小
        fp32 factor = 1.0f;
        if (move_speed > 0.0f)
        {
            // 归一化位移速度 (0~1)
            fp32 ratio = move_speed / AUTO_SPIN_SPEED_MAX;
            if (ratio > 1.0f)
                ratio = 1.0f;

            // 幂函数衰减：ratio 越大，factor 越小
            // (1 - ratio)^power 实现开始衰减慢，后面衰减快的效果
            factor = AUTO_SPIN_SPEED_MIN / AUTO_SPIN_SPEED_MAX + (1.0f - AUTO_SPIN_SPEED_MIN / AUTO_SPIN_SPEED_MAX) * powf(1.0f - ratio, AUTO_SPIN_DECAY_POWER);
        }

        // 计算最终自转速度
        *wz_set = AUTO_SPIN_SPEED_MAX * factor;
    }
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_OPEN, chassis control mode is raw control mode.
  *                 set value will be sent to can bus.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ???????????????????????????raw??????????????????????can??????
  * @param[in]      vx_set????????,??? ??????? ??? ???????
  * @param[in]      vy_set???????????? ???????? ??? ???????
  * @param[in]      wz_set ??????? ??? ????????????? ???????
  * @param[in]      chassis_move_rc_to_vector????????
  * @retval         none
  */

static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *vy_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *wz_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    return;
}

static void chassis_jetson_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    robot_chassis_cmd_t cmd;

    (void)chassis_move_rc_to_vector;

    if (vx_set == NULL || vy_set == NULL || wz_set == NULL)
    {
        return;
    }

    if (!robot_comm_get_chassis_cmd(&cmd))
    {
        *vx_set = 0.0f;
        *vy_set = 0.0f;
        *wz_set = 0.0f;
        return;
    }

    *vx_set = cmd.vx;
    *vy_set = cmd.vy;
    *wz_set = cmd.wz;
}
