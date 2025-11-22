/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file    MecenumChassis.h
 * @author  lrc
 * @brief   Header file
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

#ifndef _myChasis_H_
#define _myChasis_H_

#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/
#include "SRML.h" //使用srml库（电机库）
#include "srml_std_lib.h"

#include "FreeRTOS.h" //使用freertos队列向任务发送信息
#include "queue.h"
/* Private macros ------------------------------------------------------------*/

/************枚举量*************/
enum E_Chasis_WheelType // 麦轮底盘轮子
{
  LF = 0U,
  RF,
  RB,
  LB
};

enum class E_Chassis_Mode // 底盘模式
{
  Speed = 0U, // 速度模式
  Position,   // 位置模式
  Halt        // 暂停模式
};

enum class E_Chassis_SpeedGears // 速度挡位
{
  SLOW_GEAR = 0U,
  NORMAL_GEAR,
  FAST_GEAR
};

/* Private type --------------------------------------------------------------*/

/************结构体类型*************/
typedef struct S_Chassis_GlobalPos // 全局位姿，包含整车位置坐标与三轴姿态信息
{
  float x, y, roll, pitch, yaw;
  /***************************重载运算符***************************/
  S_Chassis_GlobalPos &operator=(const S_Chassis_GlobalPos &globalPos)
  {
    if (&globalPos != this)
    {
      x = globalPos.x;
      y = globalPos.y;
      pitch = globalPos.pitch;
      roll = globalPos.roll;
      yaw = globalPos.yaw;
    }
    return *this;
  }

  S_Chassis_GlobalPos &operator-(const S_Chassis_GlobalPos &globalPos)
  {
    if (&globalPos != this)
    {
      x -= globalPos.x;
      y -= globalPos.y;
      pitch -= globalPos.pitch;
      roll -= globalPos.roll;
      yaw -= globalPos.yaw;
    }
    return *this;
  }

  S_Chassis_GlobalPos &operator+(const S_Chassis_GlobalPos &globalPos)
  {
    if (&globalPos != this)
    {
      x += globalPos.x;
      y += globalPos.y;
      pitch += globalPos.pitch;
      roll += globalPos.roll;
      yaw += globalPos.yaw;
    }
    return *this;
  }
} S_Chassis_GlobalPos;

typedef struct S_Chassis_Velocity // 速度矢量
{
  float x_speed;
  float y_speed;
  float z_speed;
  /***************************重载运算符***************************/
  S_Chassis_Velocity &operator=(S_Chassis_Velocity &velocity)
  {
    x_speed = velocity.x_speed;
    y_speed = velocity.y_speed;
    z_speed = velocity.z_speed;
    return *this;
  }

  S_Chassis_Velocity &operator+(S_Chassis_Velocity &velocity)
  {
    x_speed += velocity.x_speed;
    y_speed += velocity.y_speed;
    z_speed += velocity.z_speed;
    return *this;
  }

  S_Chassis_Velocity &operator-(S_Chassis_Velocity &velocity)
  {
    x_speed -= velocity.x_speed;
    y_speed -= velocity.y_speed;
    z_speed -= velocity.z_speed;
    return *this;
  }
} S_Chassis_Velocity;

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

class Chassis_ClassDef
{
public:

  /* Wheel Define */
  Motor_C620 Wheel[4]{1, 2, 3, 4}; // LF RF RB LB

  Chassis_ClassDef(float Max_Motor_Output = 16384,                  // 电机最大输出，默认c620最大输出
                   int16_t Wheel_Max_Speed = 8000,                  // 麦轮最大转速
                   bool torque_optimize_flag = 1,                   // 是否开启力矩优化，默认开
                   bool attitude_control_flag = 0,                  // 是否开启姿态控制，默认关
                   float Dt = 0.001f)                               // 步长，默认1ms
  {
    Chassis_Mode = E_Chassis_Mode::Speed;         // 默认速度模式
    Speed_Gear = E_Chassis_SpeedGears::FAST_GEAR; // 默认高速
    max_motor_output = Max_Motor_Output;          // 电机最高输出
    wheel_max_speed = Wheel_Max_Speed;            // 轮子最大转速
    torqueOptimize_flag = torque_optimize_flag;   // 力矩优化标志位
    attitudeControl_flag = attitude_control_flag; // 姿态控制标志位
    launch_speed = wheel_max_speed / 5.0f;        // 启动速度
    dt = Dt;
    max_launch_acceleration = 5555; // 随便给的，待测
    max_normal_acceleration = 8888;
    max_brake_acceleration = 11111;

    lowspeed_scale = 0.2;
    normalspeed_scale = 0.6;
    highspeed_scale = 1;
    setAllPidParam(); // 在构造函数里初始化所有pid
  };

  /* 控制主函数 */
  E_Chassis_Mode Control();

  /* 失控保护 */
  void lostControlSave();

  /*
      设置目标值：
      1.速度模式 --> 设置三轴运动速度(X,Y,Rotate)：范围-1.0~1.0
      2.位置模式 --> 设置目标位姿（X,Y,Rotate）
    */
  void setTarget(float, float, float);

  /* 设置所有模式运动速度档位（不同档位速度最大值不同）*/
  void setSpeedGear(E_Chassis_SpeedGears);

  /* 设置底盘运动参数 */
  void setAllPidParam();
  void setSpeedParam(float slow, float normal, float fast);
  void setAccelerationParam(int16_t launch, int16_t normal, uint16_t brake);
  void setTorqueOptimizeFlag(bool flag);
  void setAttitudeControlFlag(bool flag);

  /* 更新普通位置模式下的零位姿点为当前位姿点 */
  void updateZeroPose();

  /* 更新电机数据 */
  void updateMotorData(CAN_COB CAN_RxMsg);

  /* 更新当前位姿（X,Y,Roll,Pitch,Yaw）*/
  void updateCurrentPosition(float _x, float _y);
  void updateCurrentAttitude(float _pitch, float _roll, float _yaw);

  /* 切换底盘运行状态 */
  E_Chassis_Mode getMode();
  void switchMode(E_Chassis_Mode target_mode);

  /* 向电机发送数据 */
  void sendMotorData();

private:
  E_Chassis_Mode Chassis_Mode;     // 底盘模式（四种）
  E_Chassis_SpeedGears Speed_Gear; // 速度挡位（三种）

  /* Gloabal Pose Define */
  S_Chassis_GlobalPos Command_Pos; // 指令位姿
  S_Chassis_GlobalPos Zero_Pos;    // 零位姿
  S_Chassis_GlobalPos Current_Pos; // 当前位姿

  /* Velocity Vector Define */
  S_Chassis_Velocity Command_Velocity; // 指令速度矢量
  S_Chassis_Velocity Target_Velocity;  // 最终目标速度矢量

  /* Time interval Define for speed resolve */
  float dt;

  /* Different Speed Gear Scale Define */
  float lowspeed_scale;
  float normalspeed_scale;
  float highspeed_scale;
  float speed_scale;

  /* Flag Define */
  bool torqueOptimize_flag;
  bool attitudeControl_flag;

  /* Limit Out Define */
  int16_t wheel_max_speed;
  int16_t max_motor_output;

  /* Acceleration Define for different stages(Unit: rpm/s) */
  int16_t max_launch_acceleration;
  int16_t max_normal_acceleration;
  uint16_t max_brake_acceleration;
  int16_t launch_speed;

  /* Wheel PID Define */
  myPID WheelSpeedLoop[4];

  /* Pose PID Define */
  myPID Position_X;
  myPID Position_Y;
  myPID Position_Z;
  myPID TurnOverSave; // 防翻车保护

  /* Macanum Motion Resolve */
  void resolve(int16_t *wheelOut); // 麦轮解算

  /* Private Controller */
  bool speed_control();       // 速度控制优化
  bool position_control();    // 位置控制优化
  bool torque_optimization(); // 力矩优化启动和刹车
  bool attitude_control();    // 姿态控制，防翻车
};

/* Exported function declarations --------------------------------------------*/
#endif

#ifdef __cplusplus
extern "C"
{
#endif
  /* Exported macros -----------------------------------------------------------*/
  /* Exported types ------------------------------------------------------------*/
  /* Exported function declarations --------------------------------------------*/
}
#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
