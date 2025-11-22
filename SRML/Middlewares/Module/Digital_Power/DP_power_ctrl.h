/**
******************************************************************************
* Copyright (c) 2022 - ~, SCUT-RobotLab Development Team
* @file    DP_power_ctrl.h
* @author  易宇杰 17379904155
*          张至睿 18042844330
*          余俊晖 13826150939
* @brief   Header file of DP_power_ctrl.
* @date    2022.1.14
* @version 1.0
* @par Change Log：
* <table>
* <tr><th>Date        	<th>Version  <th>Author    		<th>Description
* <tr><td>2022.1.14   	<td> 1.0     <td>易宇杰       <td>Creater
* <tr><td>2022.11.23   	<td> 2.0     <td>张至睿       <td>完善了功率计算逻辑
                                                          修复部分bug
                                                          合并新功控和旧功控电容充电
                                                          完善封装
  <tr><td>2023.5.2   	<td> 3.0     <td>余俊晖       <td>针对数字电源进行功控库的适配
* </table>
*
==============================================================================
                        How to use this Module
==============================================================================
@note
        1. 创建对象输入3508电机数量、6020电机数量、步兵种类
        2. 加载功控控制器：功率环控制器、电容电压环控制器
        3. 更新数据：底盘功率数据，电机速度数据
        4. 决定是否启用电容电压环（超功率则不启动，非超功率模式启动）
        5. 功率控制结算
        6. 控制量输出
@attention

@example
        //1. 创建对象输入3508电机数量、6020电机数量、步兵种类
        DP_Power_Ctrl_Classdef DP_Power_Ctrl(WHEEL_NUM, STEER_NUM, STEER_CHASSIS);

        //2. 加载功控控制器：功率环控制器、电容电压环控制器
        DP_Power_Ctrl.Load_PowerController(PowerOutController);
        DP_Power_Ctrl.Load_CapVolController(CapVolController);

        //3. 更新数据：底盘功率数据，电机速度数据
        DP_Power_Ctrl.Update_Chassis_Data(digital_Power.power.pow_motor, MaxPower_Chassis, digital_Power.unit_DPW_data.Vcap);
        //第二个参数为期望的底盘使用功率, 不是裁判系统限制功率！！！
        //第三个参数请传入以“V"为单位的电容电压，数字电源的DPW中电容电压是以mV为单位的，unit_DPW_data中的电容电压才是以V为单位的
        DP_Power_Ctrl.Update_Moto_Data(Movement_Data.Output.Wheel_TargetSpeed,      //3508的目标速度
                                       Movement_Data.Output.Wheel_CurrentSpeed,     //3508的当前速度
                                       SteeringSpeed_target,                        //6020的目标速度
                                       Movement_Data.Output.Steer_CurrentSpeed);    //6020的当前速度
        //除了舵轮，其他步兵不需要输入6020的速度参数

        //4. 决定是否启用电容电压环（超功率则不启动，非超功率模式启动）
        if(超功率)
        {
            DP_Power_Ctrl.En_volcaploop = 0;
        }
        else
        {
            DP_Power_Ctrl.En_volcaploop = 1;
        }

        //5. 功率控制结算
        DP_Power_Ctrl.Power_Ctrl_Adjust();  //计算函数

        //6. 控制量输出
        DP_Power_Ctrl.Get_Data().limit_current_3508[i]；//3508的限制下发电流值
        DP_Power_Ctrl.Get_Data().limit_current_6020[j]; //6020的限制下发电流值
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
#ifndef _DP_POWER_CTRL_H_
#define _DP_POWER_CTRL_H_

#if __cplusplus

/* Includes ------------------------------------------------------------------*/
#include "Middlewares/Algorithm/filters/filters.h"
#include "stdint.h"
#include <stddef.h> /* 使用NULL */

/* Private define ------------------------------------------------------------*/
#define SW_6020_CURRENT_SCALE 504   // 舵轮速度电流斜率
#define SW_6020_CURRENT_RESIDUE 0.5 // 舵轮速度电流截距
#define SW_3508_CURRENT_SCALE 1440  // 舵轮速度电流斜率
#define SW_3508_CURRENT_RESIDUE 0.5 // 舵轮速度电流截距

#define MC_3508_CURRENT_SCALE 0.025 // 麦轮速度电流截距
#define MC_3508_CURRENT_RESIDUE 475 // 麦轮速度电流截距

/* Private variables ---------------------------------------------------------*/
enum DP_Chassis_Type_Enumdef
{
    MECANUM_CHASSIS = 0, /* 麦轮底盘*/
    STEER_CHASSIS = 1    /* 舵轮底盘 */
};

struct DP_Data_Out_Structdef
{
    float limit_current_3508[4]; // 3508限幅电流值
    float limit_current_6020[4]; // 6020限幅电流值
} ;            // 输出数据结构体

/* Exported types ------------------------------------------------------------*/
class DP_Power_Ctrl_Classdef
{
private:
    /*底盘功率*/
    float chassis_power_target; // 底盘功率目标值
    float power_out_fact;       // 数字电源读取的实际输出功率
    float power_in_theory;      // 功率目标加上功率环获得的理论可支配功率
    float Vcap;                 // 电容电压
    /*功率环*/
    float feed_forward_total = 0;                                                              // 总功率前馈
    float power_loop_out = 0;                                                                  // 获取功率环总输出
    float cap_loop_out = 0;                                                                    // 获取电容环总输出
    float feed_forward_wheel[4] = {0};                                                         // 轮功率前馈
    float feed_forward_STEER[4] = {0};                                                         // 舵功率前馈
    float speed_to_current(float _speed, float _scale, float _residue);                        // 由速度得稳态电流(拟合直线关系)
    float cal_feed_forward_total(float power, float _scale, float _residue);                   // 根据目标功率得到前馈总输出
    void distribute_wheel(float *speed_target, float *now_speed);                              // 由速度-稳态电流关系，把前馈分给四个轮
    void distribute_STEER(float *speed_target, float *now_speed);                              // 由速度-稳态电流关系，把前馈分给四个舵
    void wheel_phase_current_trans(float *_target_speed, float *_speed, float *_feed_forward); // 舵前馈功率转相电流
    void STEER_phase_current_trans(float *_target_speed, float *_speed, float *_feed_forward); // 轮前馈功率转相电流
    /*电机*/
    uint8_t num_3508;
    uint8_t num_6020;
    float speed_target_3508[4] = {0};
    float speed_current_3508[4] = {0};
    float speed_target_6020[4] = {0};
    float speed_current_6020[4] = {0};
    float current_scale_6020; // 速度转稳态电流关系变量
    float current_residue_6020;
    float current_scale_3508;
    float current_residue_3508;
    /* 控制器 */
    float (*power_loop_controller)(const float current, const float target) = NULL;
    float (*cap_loop_controller)(const float current, const float target) = NULL;
    /*输出*/
    DP_Data_Out_Structdef output; // 输出数据
public:
    DP_Chassis_Type_Enumdef chassis_type;
    uint8_t En_volcaploop = 1;
    DP_Power_Ctrl_Classdef(uint8_t _num_3508, uint8_t _num_6020, DP_Chassis_Type_Enumdef _chassis_type);
    /*接口*/
    // 加载功率输出环控制器
    void Load_CapVolController(float (*pFunc)(const float current, const float target));
    void Load_PowerController(float (*pFunc)(const float current, const float target));
    // 更新数据
    void Update_Chassis_Data(float _power_out_fact, float _chassis_power_target, float _Vcap);                                                                                    // 更新底盘功率数据
    void Update_Moto_Data(float *_speed_target_3508 = NULL, float *_speed_current_3508 = NULL, float *_speed_target_6020 = NULL, float *_speed_current_6020 = NULL); // 更新电机数据
    // 功率控制结算
    void Power_Ctrl_Adjust(); // 功率环结算
    // 输出数据
    const DP_Data_Out_Structdef &Get_Data() { return output; } // 返回一个联合数据类型的常量引用（不能被外部更改）
};
#endif /* __cplusplus */

#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB**************************/
