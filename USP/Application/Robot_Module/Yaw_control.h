#pragma once

#ifdef __cplusplus

#include "SRML.h"
#include "robot_types.h"

//仿照global_data.h中的状态枚举定义方式
#if enum_X_Macros_disable
enum yaw_control_state_e
{
    YAW_MANUAL_AIM = 0,
    YAW_VISION_AIM,
    YAW_CORRECT_AIM,
    YAW_CALIBRATING,
    YAW_DISABLE_MOTOR 
};
#else
//1. 定义 X-列表：
#define YAW_CONTROL_STATE_LIST(X) \
    X(YAW_MANUAL_AIM)         \
    X(YAW_VISION_AIM)         \
    X(YAW_CORRECT_AIM)        \
    X(YAW_CALIBRATING)    \
    X(YAW_DISABLE_MOTOR)
//自动生成枚举定义
enum yaw_control_state_e {
    #define AS_ENUM(name) name,
    YAW_CONTROL_STATE_LIST(AS_ENUM)
    #undef AS_ENUM
};

//自动生成转换函数：
inline const char* Yaw_Control_State_To_Str(yaw_control_state_e state) {
    switch(state) {
        #define AS_CASE(name) case name: return #name;
        YAW_CONTROL_STATE_LIST(AS_CASE)
        #undef AS_CASE
        default: return "UNKNOWN";
    }
}

//2. 在日志中使用：
//LOG_INFO("State changed to: %s", Yaw_Control_State_To_Str(Robot.Status.yaw_control_state));
//增加新状态时，只需在 YAW_CONTROL_STATE_LIST 中添加一行，枚举和字符串会自动同步。
#endif

class Missle_YawController_Classdef
{
private:
  float MAX_YAW_ANGLE, MIN_YAW_ANGLE;
public:
    // yaw轴初始化标志，0未初始化，1初始化中，2初始化完成
    uint8_t Yaw_Init_flag = 0;
    // pid对象
    myPID PID_Yaw_Angle ,PID_Yaw_Speed ,PID_Yaw_Vision;
    //电机模式
    Control_Mode_e mode_YAW;
    //yaw_target
    float yaw_target = 0;
    //电机抽象对象
    abstractMotor<Motor_GM6020> YawMotor;

    // 构造函数
    Missle_YawController_Classdef(uint8_t _ID_YAW);
    // 校准函数
    void calibration();
    // 更新yaw轴目标角度，带限幅，虽然说任务里是直接修改target的，也可封装，我觉得没必要。
    void update(float _yaw_target);
    // 根据电机模式进行pid计算
    void adjust();
    // 失能函数
    void disable();
    // 输出电机速度
    void yaw_out_motor_speed();
    //yaw轴子状态机,包含状态如下,同时会对行程电机进行控制
    /*
        YAW_MANUAL_AIM:手动瞄准
        YAW_VISION_AIM:视觉瞄准
        YAW_CORRECT_AIM调参板瞄准
        YAW_DISABLE_MOTOR:失能电机
        yaw_calibrating:校准模式
    */
    
    void yaw_state_machine(yaw_control_state_e *yaw_state,float RC_X,float RC_Y);
    // 判断yaw轴是否到达目标角度
    bool isMotorAngleReached(float threshold);
    bool is_Yaw_pid_Vision_stable(float threshold);
    // 判断yaw轴是否初始化完成
    inline bool is_Yaw_Init() { return (Yaw_Init_flag == 2); }
    //校准细化
    inline bool is_Yaw_L_calibrated() { return (Yaw_Init_flag >= 1); }
    inline bool is_Yaw_R_calibrated() { return (Yaw_Init_flag == 2); }
};

#endif