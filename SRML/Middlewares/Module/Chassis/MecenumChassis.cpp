/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file MecenumChasis.cpp
 * @author lrc
 * @brief 底盘控制
 * @date 2022-11-04
 * @version 1.0
 * @par Change Log:
 * <table>
 * <tr><th>Date <th>Version <th>Author <th>Description
 * <tr><td>2022-11-04 <td> 1.0 <td>lrc <td>Creator
 * </table>
 *
 ==============================================================================
 ##### How to use this driver #####
 ==============================================================================
 @note
    -# 新建"myChassis"对象，构造时设置初始底盘参数，选择是否进行力矩优化和姿态保护

    -# 在电机接收任务调用Update_MotorData()更新电机数据
       在mpu读取任务调用Update_CurrentAttitude()更新陀螺仪三轴角度数据

    -# 调用Switch_Mode()切换底盘状态
       调用Set_Target()设置指令目标

    -# 按照配置的运行频率调用 Chassis_Control().

    -# 通过Set_xx()设置参数、目标
       使用Update_xx()更新一些当前值
 @warning
    - 在整定底盘控制器参数时，请注意通过调节运动加速度大小，使任何一个电机输出不会
      长时间被输出限幅作用（大于电机最大输出时进入饱和区，使底盘运动性能恶化）
 ******************************************************************************
 * @attention
 *
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version Number, write dowm your name and the date, the most
* important is make sure the users will have clear and definite understanding
 * through your new brief.
 *
 * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
 * All rights reserved.</center></h2>
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "srml_config.h"

#if USE_SRML_MECENUM_CHASSIS

#include "MecenumChassis.h"
/* Exported macros -----------------------------------------------------------*/
template <typename T>
inline T myabs(T x) { return x < 0 ? -x : x; } // 自定义绝对值
/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

/**
 * @brief 底盘总控制
 *
 * @return E_Chassis_Mode
 */
E_Chassis_Mode Chassis_ClassDef::Control()
{
    // 位置环的速度输出,无位置环时使用外部速度指令
    position_control();

    // 使用外部速度指令时启动输出力矩优化
    torque_optimization();

    // 姿态保护，在优化的基础上叠加输出，并联结构
    attitude_control();

    // 最终控制输出计算
    speed_control();

    return Chassis_Mode;
}

/**
 * @brief  失控保护，清零pid，将电机输出置零
 * @param
 * @return NULL
 */
void Chassis_ClassDef::lostControlSave()
{
    if (Chassis_Mode != E_Chassis_Mode::Halt)
    {
        Chassis_Mode = E_Chassis_Mode::Halt;

        for (register uint8_t i = 0; i < 4; i++)
        {
            WheelSpeedLoop[i] = myPID(); // 重置所有轮子PID

            Wheel[i].Out = 0; // 输出置零
        }

        sendMotorData();

        Position_X = myPID(); // 重置其他pid
        Position_Y = myPID();
        Position_Z = myPID();
        TurnOverSave = myPID();

        setAllPidParam(); // 设置pid参数
    }
}

/**
 * @brief 设置目标指令
 *
 * @param target_X
 * @param target_Y
 * @param target_Z
 */
void Chassis_ClassDef::setTarget(float target_X, float target_Y, float target_Z)
{
    if (Chassis_Mode == E_Chassis_Mode::Speed)
    {
        Command_Velocity.x_speed = std_lib::constrain(target_X, -1.0f, 1.0f) * wheel_max_speed * speed_scale;
        Command_Velocity.y_speed = std_lib::constrain(target_Y, -1.0f, 1.0f) * wheel_max_speed * speed_scale;
        Command_Velocity.z_speed = std_lib::constrain(target_Z, -1.0f, 1.0f) * wheel_max_speed * speed_scale;
    }
    else if (Chassis_Mode == E_Chassis_Mode::Position)
    {
        Command_Pos.x = target_X;
        Command_Pos.y = target_Y;
        Command_Pos.yaw = target_Z;
    }
    else
    {
    }
}

/**
 * @brief 设置所有模式运动速度档位（不同档位速度最大值不同）
 *
 * @param targetGear
 */
void Chassis_ClassDef::setSpeedGear(E_Chassis_SpeedGears targetGear)
{
    Chassis_Mode = E_Chassis_Mode::Speed;
    Speed_Gear = targetGear;
    switch (Speed_Gear)
    {
    case E_Chassis_SpeedGears::SLOW_GEAR:
        speed_scale = lowspeed_scale;
        break;
    case E_Chassis_SpeedGears::NORMAL_GEAR:
        speed_scale = normalspeed_scale;
        break;
    case E_Chassis_SpeedGears::FAST_GEAR:
        speed_scale = highspeed_scale;
        break;
    default:
        break;
    }
}

// static float debug_kp = 10;
/**
 * @brief 所有pid初始化
 *
 */
void Chassis_ClassDef::setAllPidParam()
{
    for (register uint8_t i = 0; i < 4; i++)
        WheelSpeedLoop[i].SetPIDParam(10, 0, 0, 0, 16384);
    Position_X.SetPIDParam(0, 0, 0, 0, 0);
    Position_Y.SetPIDParam(0, 0, 0, 0, 0);
    Position_Z.SetPIDParam(0, 0, 0, 0, 0);
    TurnOverSave.SetPIDParam(0, 0, 0, 0, 0);
}
/**
 * @brief 设置速度三档系数
 *
 * @param slow
 * @param normal
 * @param fast
 */
void Chassis_ClassDef::setSpeedParam(float slow, float normal, float fast)
{
    lowspeed_scale = std_lib::constrain(slow, 0.0f, 1.0f);
    normalspeed_scale = std_lib::constrain(normal, 0.0f, 1.0f);
    highspeed_scale = std_lib::constrain(fast, 0.0f, 1.0f);
}
/**
 * @brief 设置启动、常规运动、刹车状态的最大加速度
 *
 * @param launch
 * @param normal
 * @param brake
 */
void Chassis_ClassDef::setAccelerationParam(int16_t launch, int16_t normal, uint16_t brake)
{
    launch = std_lib::constrain((int16_t)launch, (int16_t)-32765, (int16_t)32765);
    normal = std_lib::constrain((int16_t)normal, (int16_t)-32765, (int16_t)32765);
    brake = std_lib::constrain((int16_t)normal, (int16_t)0, (int16_t)65535);

    max_launch_acceleration = myabs(launch);
    max_normal_acceleration = myabs(normal);
    max_brake_acceleration = myabs(brake);
}
/**
 * @brief 设置是否开启力矩优化
 *
 * @param flag 1：开启
 *             0：关闭
 */
void Chassis_ClassDef::setTorqueOptimizeFlag(bool flag)
{
    torqueOptimize_flag = flag;
}
/**
 * @brief 设置是否开启姿态控制
 *
 * @param flag 1：开启
 *             0：关闭
 */
void Chassis_ClassDef::setAttitudeControlFlag(bool flag)
{
    attitudeControl_flag = flag;
}

/**
 * @brief 更新零位姿
 *
 */
void Chassis_ClassDef::updateZeroPose()
{
    Zero_Pos.x = Current_Pos.x;
    Zero_Pos.y = Current_Pos.y;
    Zero_Pos.roll = Current_Pos.roll;
    Zero_Pos.yaw = Current_Pos.yaw;
    Zero_Pos.pitch = Current_Pos.pitch;
}
/**
 * @brief 更新电机数据
 *
 * @param CAN_RxMsg
 */
void Chassis_ClassDef::updateMotorData(CAN_COB CAN_RxMsg)
{
    for (register uint8_t i = 0; i < 4; i++)
    {
        if (Wheel[i].CheckID(CAN_RxMsg.ID)) // 匹配电机对应的包
        {
            Wheel[i].update(CAN_RxMsg.Data);                 // 更新电机数据
            WheelSpeedLoop[i].Current = Wheel[i].getSpeed(); // 更新电机当前值
        }
    }
}
/**
 * @brief 更新位置坐标
 *
 * @param current_x
 * @param current_y
 */
void Chassis_ClassDef::updateCurrentPosition(float current_x, float current_y)
{
    Current_Pos.x = current_x;
    Current_Pos.y = current_y;
}
/**
 * @brief 更新姿态角
 *
 * @param roll
 * @param pitch
 * @param yaw
 */
void Chassis_ClassDef::updateCurrentAttitude(float roll, float pitch, float yaw)
{
    Current_Pos.roll = roll;
    Current_Pos.pitch = pitch;
    Current_Pos.yaw = yaw;
}

/**
 * @brief 获取底盘当前状态
 *
 * @return E_Chassis_Mode
 */
E_Chassis_Mode Chassis_ClassDef::getMode()
{
    return Chassis_Mode;
}
/**
 * @brief 切换底盘状态
 *
 * @param target_mode
 */
void Chassis_ClassDef::switchMode(E_Chassis_Mode target_mode)
{
    Chassis_Mode = target_mode;
}

/**
 * @brief 位置控制，由位姿指令解算出速度指令
 *
 * @return  1: 执行
 *          0: 跳过
 */
bool Chassis_ClassDef::position_control()
{
    if (Chassis_Mode != E_Chassis_Mode::Position)
        return 0; // 非位置控制模式

    static S_Chassis_GlobalPos Target_Pos, Relative_Pos;

    Target_Pos = Command_Pos;

    Relative_Pos = Current_Pos - Zero_Pos; // 相对位姿=当前位姿-零位姿，以陀螺仪数据

    Position_X.Target = Target_Pos.x;
    Position_Y.Target = Target_Pos.y;
    Position_Z.Target = Target_Pos.yaw;

    Position_X.Current = Relative_Pos.x;
    Position_Y.Current = Relative_Pos.y;
    Position_Z.Current = Relative_Pos.yaw;

    Target_Velocity.x_speed = Position_X.Adjust(); // 位置环计算
    Target_Velocity.y_speed = Position_Y.Adjust();
    Target_Velocity.z_speed = Position_Z.Adjust();

    return 1;
}

/**
 * @brief 启动和刹车时进行力矩优化
 *
 * @return true 执行
 * @return false 跳过
 */
bool Chassis_ClassDef::torque_optimization()
{
    static S_Chassis_Velocity Last_Target;
    static float dspeed, max_acceleration;

    if (Chassis_Mode != E_Chassis_Mode::Speed)
        return 0; // 非速度模式无需力矩优化

    if (torqueOptimize_flag == 0)
        return 0; // 没有开启力矩优化

    /* X Axis */
    if (myabs(Command_Velocity.x_speed) > myabs(Last_Target.x_speed))
    {
        // accelerate
        if (myabs(Command_Velocity.x_speed) <= launch_speed)
        {
            max_acceleration = max_launch_acceleration;
        }
        else
        {
            max_acceleration = max_normal_acceleration;
        }
    }
    else // decelerate
    {
        max_acceleration = max_brake_acceleration;
    }

    dspeed = max_acceleration * dt;

    if (myabs(Command_Velocity.x_speed - Last_Target.x_speed) >= dspeed)
    {
        if (Command_Velocity.x_speed >= Last_Target.x_speed)
        {
            Last_Target.x_speed += dspeed;
        }
        else
        {
            Last_Target.x_speed -= dspeed;
        }
    }
    else
    {
        Last_Target.x_speed = Command_Velocity.x_speed;
    }

    /* Y Axis */
    if (myabs(Command_Velocity.y_speed) > myabs(Last_Target.y_speed))
    {
        // accelerate
        if (myabs(Command_Velocity.y_speed) <= launch_speed)
        {
            max_acceleration = max_launch_acceleration;
        }
        else
        {
            max_acceleration = max_normal_acceleration;
        }
    }
    else // decelerate
    {
        max_acceleration = max_brake_acceleration;
    }

    dspeed = max_acceleration * dt;

    if (myabs(Command_Velocity.y_speed - Last_Target.y_speed) >= dspeed)
    {
        if (Command_Velocity.y_speed >= Last_Target.y_speed)
        {
            Last_Target.y_speed += dspeed;
        }
        else
        {
            Last_Target.y_speed -= dspeed;
        }
    }
    else
    {
        Last_Target.y_speed = Command_Velocity.y_speed;
    }

    /* Z don't limit acceleration */
    Last_Target.z_speed = Command_Velocity.z_speed;

    Target_Velocity = Last_Target;

    return 1;
}

/**
 * @brief 姿态控制，防翻车
 *
 * @return true 执行
 * @return false 跳过
 */
bool Chassis_ClassDef::attitude_control()
{
    if (attitudeControl_flag == 0)
        return 0; // 没有开启姿态控制
    if (Chassis_Mode != E_Chassis_Mode::Speed)
        return 0; // 非速度模式

    static S_Chassis_Velocity Attitude_Compensation;

    if (myabs(Current_Pos.pitch) > 30.0f) // 简单写的，还未验证
        Attitude_Compensation.y_speed = Target_Velocity.y_speed * Current_Pos.pitch / 90.0f;
    if (myabs(Current_Pos.roll) > 30.0f) // 简单写的，还未验证
        Attitude_Compensation.x_speed = Target_Velocity.x_speed * Current_Pos.roll / 90.0f;

    Target_Velocity = Target_Velocity + Attitude_Compensation; // 并联结构

    return 1;
}

/**
 * @brief 底盘运动速度控制与优化，从目标速度解算出每个轮子的输出
 *
 * @return true
 * @return false
 */

bool Chassis_ClassDef::speed_control()
{
    static int16_t temp_rpm[6] = {0}; // 缓存电机输出值
    static float scale = 0;           // 等比例缩小系数
    float max_rpm = 0;                // 找出转速最快的轮子

    if (Chassis_Mode == E_Chassis_Mode::Halt)
    {
        Target_Velocity.x_speed = 0; // 置零
        Target_Velocity.y_speed = 0;
        Target_Velocity.z_speed = 0;
    }

    /*麦轮底盘运动解算*/
    resolve(temp_rpm);

    /*速度优化
      当解算后的四个电机最高目标转速超过限定值，将所有电机转速进行等比列缩小，
      使得最高转速总是不超过限定值且不引起底盘运动崎变*/
    for (register uint8_t i = 0; i < 4; i++) // 取电机转速最大值
        max_rpm = max_rpm < myabs(temp_rpm[i]) ? myabs(temp_rpm[i]) : max_rpm;

    if (max_rpm >= wheel_max_speed) // 最大值超过，则等比例缩小
        scale = (float)wheel_max_speed / (float)max_rpm;
    else
        scale = 1.0f;

    for (register uint8_t i = 0; i < 4; i++) // PID计算出最终输出
    {
        WheelSpeedLoop[i].Target = (int16_t)((float)temp_rpm[i] * scale);
        WheelSpeedLoop[i].Current = Wheel[i].getSpeed();
        Wheel[i].Out = WheelSpeedLoop[i].Adjust();
    }

    return 1;
}

/**
 * @brief 麦轮底盘运动解算
 *
 * @param int16_t *wheelOut
 */
void Chassis_ClassDef::resolve(int16_t *wheelOut)
{
    wheelOut[LF] = Target_Velocity.x_speed + Target_Velocity.y_speed + Target_Velocity.z_speed;
    wheelOut[RF] = Target_Velocity.x_speed - Target_Velocity.y_speed + Target_Velocity.z_speed;
    wheelOut[RB] = -Target_Velocity.x_speed - Target_Velocity.y_speed + Target_Velocity.z_speed;
    wheelOut[LB] = -Target_Velocity.x_speed + Target_Velocity.y_speed + Target_Velocity.z_speed;
}

/**
 * @brief 发送指令给电机
 *
 */
void Chassis_ClassDef::sendMotorData()
{
    static Motor_CAN_COB CAN1_pack;
    extern QueueHandle_t CAN1_TxPort;

    CAN1_pack = motor_dji::MotorMsgPack(CAN1_pack, Wheel[LF], Wheel[RF], Wheel[RB], Wheel[LB]);

    //	 xQueueSend(CAN1_TxPort, &CAN1_pack.Id1ff, 0);
    xQueueSend(CAN1_TxPort, &CAN1_pack.Id200, 0);
}

#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB**************************/
