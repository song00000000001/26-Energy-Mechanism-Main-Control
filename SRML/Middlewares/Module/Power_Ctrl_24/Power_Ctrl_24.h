/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    Power_Ctrl_24.h
  * @author  余俊晖 2460857175@QQ.COM
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
#pragma once

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include "srml_std_lib.h"
#include "Middlewares/Algorithm/Filters/filters.h"
#include "Middlewares/Algorithm/PID/PID.h"
/* Private macros ------------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
struct power_model_param_t
{
    float k1;
    float k2;
    float k3;
};

namespace PowerCtrl
{
    const power_model_param_t M3508_PARAMS = {0.001543f, 0.13075412f, 1.79296E-07f};
    const power_model_param_t M6020_PARAMS = {0.05916479f, 6.03461764, 1.41747E-05f};
    
    /**
     * @brief 轮、舵功率分配函数，库内默认实现为__weak弱函数，使用者可在库外自定义实现
     * 
     * @param total_limit_power 总功率限制
     * @param wheel_power_limit 函数内被修改为分配后的（轮）功率
     * @param steer_power_limit 函数内被修改为分配后的（舵）功率
     */
    void distribute_steer_power(float total_limit_power, float &wheel_power_limit, float &steer_power_limit);

    /**
     * @brief 将6020的电压(±30000)转化为电流(±16384)
     * 
     * @param volatage  6020的电压(±30000)
     * @param current_speed 电机当前速度(RPM)
     */
    float m6020_volatage_to_current(float volatage, float current_speed);

    float get_motor_power(const power_model_param_t &MODEL_PARAM, float torque_current, float speed);

    float get_motor_current_limit(const power_model_param_t &MODEL_PARAM, float limit_power, float current_speed);
}


/* Exported function declarations --------------------------------------------*/

/**
 * @brief 功率控制单元
 * 
 * @tparam MOTOR_NUM 电机数量
 */
template<const uint8_t MOTOR_NUM>
class Power_Ctrl_Unit_Classdef
{
private:
    const power_model_param_t& MODEL_PARAM;
    /* data */
    float total_limit_power; // 总功率限制
    float total_current_power; // 估算的当前功率
    float remain_power; // 可用于分配的功率
    float predict_power = 0;
    float current_power[MOTOR_NUM]; // 按当前PID输出下，四个电机的功率
    float stable_power[MOTOR_NUM]; // 轮子稳定在当前速度的稳态功率
    float limit_power[MOTOR_NUM]; // 分配给每个电机的限制功率

    float target_I[MOTOR_NUM];
    float limit_I[MOTOR_NUM]; // 分配给每个电机的限制相电流或力矩，单位为A或者Nm

    // 差速分配
    float total_speed_err = 0;
    float speed_err[MOTOR_NUM];
    float target_speed[MOTOR_NUM];  // 每个电机的目标速度
    float current_speed[MOTOR_NUM]; // 每个电机的当前速度

    float k = 0, b = 0;
    inline float get_stable_Power(float speed)
    {
        return k * speed + b;
    }

public:
    float motorOut_to_A = 1;    // 将adjust传入的电机输出的单位转化为A的系数
    float motorOut[MOTOR_NUM];  // 最终给电机的目标输出，单位与adjust函数中传入的_target_Out相同

    /**
     * @brief 构造函数
     * 
     * @param _MODEL_PARAM 电机功率模型参数
     * @param _k 
     * @param _b 
     */
    Power_Ctrl_Unit_Classdef(const power_model_param_t &_MODEL_PARAM, float _k = 0, float _b = 0)
        : MODEL_PARAM(_MODEL_PARAM), k(_k), b(_b){}
    /**
     * @brief 计算电机限制相电流
     * 
     * @param _total_limit_power 当前目标功率
     * @param _target_speed 电机目标速度，单位请使用功率模型拟合用的单位
     * @param _current_speed 电机当前速度，单位请使用功率模型拟合用的单位
     * @param _target_Out PID计算出的电机目标电流，单位任意，但要记得修改motorOut_to_A
     * @return float 若维持当前的目标相电流的功率 < 限制功率Out_to_A，则返回多出来的功率数值，否则返回0
     */
    float adjust(float _total_limit_power, const float _target_speed[MOTOR_NUM], const float _current_speed[MOTOR_NUM], const float _target_Out[MOTOR_NUM]);

    template <uint8_t _MOTOR_NUM>
    friend class Power_Ctrl_NormalChassis_Classdef;

    template <uint8_t WHEEL_MOTOR_NUM, uint8_t STEER_MOTOR_NUM>
    friend class Power_Ctrl_SteerChassis_Classdef;
};

/**
 * @brief 普通轮式底盘功控模块（非舵轮）
 * 
 * @tparam MOTOR_NUM 电机数量，默认为4
 */
template<uint8_t MOTOR_NUM = 4>
class Power_Ctrl_NormalChassis_Classdef 
{
private:
    MeanFilter<50> filter;
    float power_limit_adjusted; // 调整后，实际传入给功控单元的功率
    Power_Ctrl_Unit_Classdef<MOTOR_NUM> ctrl_unit;

public:
    myPID powerLoop, capVolatageLoop;

    float &motorOut_to_A = ctrl_unit.motorOut_to_A;     // 将adjust传入的电机输出的单位转化为A的系数
    const float *const motorOut = ctrl_unit.motorOut;   // 最终给电机的目标输出，单位与adjust函数中传入的_target_Out相同

    /**
     * @brief 构造函数
     * 
     * @param _MODEL_PARAM 电机功率模型参数，默认为3508
     * @param _k 
     * @param _b 
     */
    Power_Ctrl_NormalChassis_Classdef(const power_model_param_t &_MODEL_PARAM = PowerCtrl::M3508_PARAMS, float _k = 0, float _b = 0) 
        : ctrl_unit(_MODEL_PARAM, _k, _b){}

    /**
     * @brief 更新功率信息
     * 
     * @param target_power 目标功率
     * @param current_power 当前功率
     * @param current_capVoltage 当前电容电压，单位为V
     * @param En_capVolatageLoop 电容环启用标志位，非0即启用电容环
     */
    void update_power_data(float target_power, float current_power, float current_capVoltage, uint8_t En_capVolatageLoop)
    {
        if(En_capVolatageLoop)
        {
            // 电容环目标值只升不降，不会为了充电而降低底盘实际功率
           if(current_capVoltage > capVolatageLoop.Target)
           {
               capVolatageLoop.Target = current_capVoltage;
           }
        }
        else
        {
            capVolatageLoop.Target = current_capVoltage;
            capVolatageLoop.clean_intergral();
        }
        capVolatageLoop.Current = current_capVoltage;
        capVolatageLoop.Adjust();

        powerLoop.Target = target_power;
        powerLoop.Current = current_power;
        powerLoop.Adjust();
        if(ctrl_unit.total_current_power < target_power * 0.8f)
        {
            powerLoop.clean_intergral();
            powerLoop.Out = 0;
        }

        power_limit_adjusted = target_power + powerLoop.Out + capVolatageLoop.Out;
    }

    float cmp;  // 模型误差补偿值
    /**
     * @brief 模型误差补偿
     * 
     * @param target_power 目标功率
     * @param simple_power 数字电源采样的负载功率
     * @param static_power 底盘静态功率
     * @param over_power_flag 超功率标志位 
     */
    void model_err_cmp(float target_power, float simple_power, float static_power, uint8_t over_power_flag)
    {
        float predict_power = ctrl_unit.predict_power;
        float err = simple_power - (predict_power + static_power);
        cmp = filter.f(err);

        power_limit_adjusted = target_power - static_power - cmp;
    }

    /**
     * @brief 计算电机限制相电流
     * 
     * @param _target_speed 电机目标速度，单位请使用功率模型拟合用的单位
     * @param _current_speed 电机当前速度，单位请使用功率模型拟合用的单位
     * @param _target_Out PID计算出的电机目标电流，单位任意，但要记得修改motorOut_to_A
     */
    void adjust(const float _target_speed[MOTOR_NUM], const float _current_speed[MOTOR_NUM], const float _target_Out[MOTOR_NUM])
    {
        ctrl_unit.adjust(power_limit_adjusted, _target_speed, _current_speed, _target_Out);
    }
};   

/**
 * @brief 舵轮底盘功控模块
 * 
 * @tparam WHEEL_MOTOR_NUM      （轮）电机数量，默认为4
 * @tparam STEER_MOTOR_NUM      （舵）电机数量，默认与轮电机数量相同
 */
template<uint8_t WHEEL_MOTOR_NUM = 4, uint8_t STEER_MOTOR_NUM = WHEEL_MOTOR_NUM>
class Power_Ctrl_SteerChassis_Classdef 
{
private:
    MeanFilter<50> filter;
    float power_limit_adjusted; // 调整后，实际传入给功控单元的功率
    Power_Ctrl_Unit_Classdef<WHEEL_MOTOR_NUM> wheel_ctrl_unit;
    Power_Ctrl_Unit_Classdef<STEER_MOTOR_NUM> steer_ctrl_unit;

public:
    myPID powerLoop, capVolatageLoop;

    float &wheel_motorOut_to_A = wheel_ctrl_unit.motorOut_to_A; // 将adjust传入的（轮）电机输出的单位转化为A的系数
    float &steer_motorOut_to_A = steer_ctrl_unit.motorOut_to_A; // 将adjust传入的（舵）电机输出的单位转化为A的系数

    const float *const wheel_motorOut = wheel_ctrl_unit.motorOut; // 最终给（轮）电机的目标输出，单位与adjust函数中传入的wheel_target_Out相同
    const float *const steer_motorOut = steer_ctrl_unit.motorOut; // 最终给（舵）电机的目标输出，单位与adjust函数中传入的steer_target_Out相同

    /**
     * @brief 构造函数
     * 
     * @param WHEEL_MOTOR_PARAM （轮）电机功率模型参数，默认为3508
     * @param STEER_MOTOR_PARAM （舵）电机功率模型参数，默认为6020
     */
    Power_Ctrl_SteerChassis_Classdef(const power_model_param_t& WHEEL_MOTOR_PARAM = PowerCtrl::M3508_PARAMS, const power_model_param_t& STEER_MOTOR_PARAM = PowerCtrl::M6020_PARAMS) 
    : wheel_ctrl_unit(WHEEL_MOTOR_PARAM), steer_ctrl_unit(STEER_MOTOR_PARAM){}

    /**
     * @brief 更新功率信息
     * 
     * @param target_power 目标功率
     * @param current_power 当前功率
     * @param current_capVoltage 当前电容电压，单位为V
     * @param En_capVolatageLoop 电容环启用标志位，非0即启用电容环
     */
    void update_power_data(float target_power, float current_power, float current_capVoltage, uint8_t En_capVolatageLoop)
    {
        if(En_capVolatageLoop)
        {
            // 电容环目标值只升不降，不会为了充电而降低底盘实际功率
           if(current_capVoltage > capVolatageLoop.Target)
           {
               capVolatageLoop.Target = current_capVoltage;
           }
        }
        else
        {
            capVolatageLoop.Target = current_capVoltage;
            capVolatageLoop.clean_intergral();
        }
        capVolatageLoop.Current = current_capVoltage;
        capVolatageLoop.Adjust();

        powerLoop.Target = target_power;
        powerLoop.Current = current_power;
        powerLoop.Adjust();

        float total_current_power = wheel_ctrl_unit.total_current_power + steer_ctrl_unit.total_current_power;
        if(total_current_power < target_power * 0.8f)
        {
            powerLoop.clean_intergral();
            powerLoop.Out = 0;
        }

        power_limit_adjusted = target_power + powerLoop.Out + capVolatageLoop.Out;
    }

    float cmp;  // 模型误差补偿值
    /**
     * @brief 模型误差补偿
     * 
     * @param target_power 目标功率
     * @param simple_power 数字电源采样的负载功率
     * @param static_power 底盘静态功率
     * @param over_power_flag 超功率标志位 
     */
    void model_err_cmp(float target_power, float simple_power, float static_power, uint8_t over_power_flag)
    {
        float predict_power = wheel_ctrl_unit.predict_power + steer_ctrl_unit.predict_power;
        float err = simple_power - (predict_power + static_power);
        cmp = filter.f(err);

        power_limit_adjusted = target_power - static_power - cmp;
    }

    /**
     * @brief 计算电机限制相电流
     * 
     * @param wheel_target_speed  （轮）电机目标速度，单位请使用功率模型拟合用的单位
     * @param wheel_current_speed  （轮）电机当前速度，单位请使用功率模型拟合用的单位
     * @param wheel_target_Out  （轮）PID计算出的电机目标电流，单位任意，但要记得修改wheel_motorOut_to_A
     * @param steer_target_speed （舵）电机目标速度，单位请使用功率模型拟合用的单位
     * @param steer_current_speed （舵）电机当前速度，单位请使用功率模型拟合用的单位
     * @param steer_target_Out （舵）PID计算出的电机目标电流，单位任意，但要记得修改steer_motorOut_to_A
     */
    void adjust(const float wheel_target_speed[WHEEL_MOTOR_NUM], const float wheel_current_speed[WHEEL_MOTOR_NUM], const float wheel_target_Out[WHEEL_MOTOR_NUM],
                const float steer_target_speed[STEER_MOTOR_NUM], const float steer_current_speed[STEER_MOTOR_NUM], const float steer_target_Out[STEER_MOTOR_NUM])
    {
        float wheel_power_limit, steer_power_limit;
        PowerCtrl::distribute_steer_power(power_limit_adjusted, wheel_power_limit, steer_power_limit);

        float steer_remain_power = steer_ctrl_unit.adjust(steer_power_limit, steer_target_speed, steer_current_speed, steer_target_Out);
        wheel_ctrl_unit.adjust(wheel_power_limit + steer_remain_power, wheel_target_speed, wheel_current_speed, wheel_target_Out);
    }
};   

template<const uint8_t MOTOR_NUM>
float Power_Ctrl_Unit_Classdef<MOTOR_NUM>::adjust(float _total_limit_power, const float _target_speed[MOTOR_NUM], const float _current_speed[MOTOR_NUM], const float _target_Out[MOTOR_NUM])
{
    total_limit_power = _total_limit_power;
    total_current_power = 0;
    total_speed_err = 0;
    remain_power = total_limit_power; // 可用于分配的功率
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        target_I[i] = _target_Out[i] * motorOut_to_A;
        target_speed[i] = _target_speed[i];
        current_speed[i] = _current_speed[i];
        current_power[i] = PowerCtrl::get_motor_power(MODEL_PARAM, target_I[i], current_speed[i]); // "单个电机"维持当前PID输出将使用的功率
        total_current_power += current_power[i];  // 电机维持当前PID输出将使用的"总功率"

        // 加速时err应为正，刹车时应为负
        if(target_I[i] > 0)
        {
            speed_err[i] = target_speed[i] - current_speed[i]; 
        }
        else if(target_I[i] < 0)
        {
            speed_err[i] = -1 * (target_speed[i] - current_speed[i]);
        }
        else
        {
            speed_err[i] = fabsf(target_speed[i] - current_speed[i]); 
        }
        total_speed_err += speed_err[i];

        stable_power[i] = get_stable_Power(current_speed[i]); // 计算电机"维持当前速度"的 稳态功率
        // 若电机维持PID输出的功率 < 稳态功率，不使用稳态功率计算可分配功率
        if(current_power[i] < stable_power[i])
        {
            remain_power -= current_power[i]; // 计算可用于差速分配的功率
        }
        else
        {
            remain_power -= stable_power[i]; // 计算可用于差速分配的功率
        }

    }

    // 若维持原有PID输出不变的估算功率 < 限制功率，则不加限制
    if(total_current_power < total_limit_power)
    {
        for (int i = 0; i < MOTOR_NUM; i++)
        {
            limit_I[i] = fabsf(target_I[i]);
            motorOut[i] = _target_Out[i];
        }
        predict_power = total_current_power;
        return total_limit_power - total_current_power;
    }

    predict_power = 0;
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        limit_power[i] = stable_power[i] + (speed_err[i] / total_speed_err) * remain_power; // 差速分配
        limit_I[i] = PowerCtrl::get_motor_current_limit(MODEL_PARAM, limit_power[i], fabsf(current_speed[i])); // 得到限制相电流
        if(limit_I[i] < 0)
            limit_I[i] = 0;
        motorOut[i] = std_lib::constrain(target_I[i], -limit_I[i], limit_I[i]) / motorOut_to_A;
        predict_power += PowerCtrl::get_motor_power(MODEL_PARAM, motorOut[i] * motorOut_to_A, current_speed[i]);
    }
    return 0;
}

#endif /* __cplusplus */

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/