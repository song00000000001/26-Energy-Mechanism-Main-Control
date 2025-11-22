/**
  ******************************************************************************
  * Copyright (c) 2022 - ~, SCUT-RobotLab Development Team
  * @file    DP_power_ctrl.cpp
  * @author  易宇杰 17379904155
  *          张至睿 18042844330
  *          余俊晖 13826150939
  * @brief   Power control for chassis in Robomaster.
  * @date    2022.1.14
  * @version 1.0
  * @par Change Log：
  * <table>
  * <tr><th>Date        	<th>Version  <th>Author    		<th>Description
  * <tr><td>2022.1.14   	<td> 1.0     <td>易宇杰       <td>Creater
  *	<tr><td>2022.11.23   	<td> 2.0     <td>张至睿       <td>完善了功率计算逻辑
                                                              修复部分bug
                                                              合并新功控和旧功控电容充电
                                                              完善封装
    <tr><td>2023.5.2   	    <td> 3.0     <td>余俊晖       <td>针对数字电源进行功控库的适配
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
            DP_Power_Ctrl.Update_Moto_Data(Movement_Data.Output.Wheel_TargetSpeed,   //3508的目标速度
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
  * <h2><center>&copy; Copyright (c) 2022 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "srml_config.h"

#if USE_SRML_DIGITAL_POWER || USE_SRML_DIGITAL_POWER_V2

#include "DP_power_ctrl.h"
#include "math.h"

/* Function prototypes -------------------------------------------------------*/
/**
 * @brief  构造函数
 * @note
 * @param   3508数量、6020数量、步兵底盘类型
 * @return
 * @retval  None
 */
DP_Power_Ctrl_Classdef::DP_Power_Ctrl_Classdef(uint8_t _num_3508, uint8_t _num_6020, DP_Chassis_Type_Enumdef _chassis_type)
{
    chassis_type = _chassis_type;
    num_3508 = _num_3508;
    num_6020 = _num_6020;
    if (chassis_type == MECANUM_CHASSIS)
    {
        current_scale_6020 = 0;
        current_residue_6020 = 0;
        current_scale_3508 = MC_3508_CURRENT_SCALE;
        current_residue_3508 = MC_3508_CURRENT_RESIDUE;
    }
    else if (chassis_type == STEER_CHASSIS)
    {
        current_scale_6020 = SW_6020_CURRENT_SCALE;
        current_residue_6020 = SW_6020_CURRENT_RESIDUE;
        current_scale_3508 = SW_3508_CURRENT_SCALE;
        current_residue_3508 = SW_3508_CURRENT_RESIDUE;
    }
}

/**
 * @brief  更新底盘功率数据
 * @note
 * @param   实际输出功率、底盘功率目标值
 * @return
 * @retval  None
 */
void DP_Power_Ctrl_Classdef::Update_Chassis_Data(float _power_out_fact, float _chassis_power_target, float _Vcap)
{
    static MeanFilter<100> power_motor_filter;
    power_out_fact = power_motor_filter.f(_power_out_fact);
    chassis_power_target = _chassis_power_target;
    Vcap = _Vcap;
}

/**
 * @brief  更新电机数据
 * @note
 * @param   更新电机的速度目标值和当前值
 * @return
 * @retval  None
 */
void DP_Power_Ctrl_Classdef::Update_Moto_Data(float *_speed_target_3508, float *_speed_current_3508, float *_speed_target_6020, float *_speed_current_6020)
{
    if (_speed_target_3508 != NULL && _speed_current_3508 != NULL)
    {
        for (int i = 0; i < num_3508; i++)
        {
            speed_target_3508[i] = _speed_target_3508[i];
            speed_current_3508[i] = _speed_current_3508[i];
        }
    }

    if (chassis_type == STEER_CHASSIS)
    {
        if (_speed_target_6020 != NULL && _speed_current_6020 != NULL)
        {
            for (int j = 0; j < num_6020; j++)
            {
                speed_target_6020[j] = _speed_target_6020[j];
                speed_current_6020[j] = _speed_current_6020[j];
            }
        }
    }
}

/**
 * @brief  加载外部控制器
 * @note
 * @param
 * @return
 * @retval  None
 */
void DP_Power_Ctrl_Classdef::Load_PowerController(float (*pFunc)(const float current, const float target))
{
    power_loop_controller = pFunc;
}
void DP_Power_Ctrl_Classdef::Load_CapVolController(float (*pFunc)(const float current, const float target))
{
    cap_loop_controller = pFunc;
}
/************************************************输出功率控制********************************************************/

/**
 * @brief  功率输出总控制函数
 * @note
 * @param
 * @return
 * @retval  None
 */
void DP_Power_Ctrl_Classdef::Power_Ctrl_Adjust(void)
{// 电压环
    if (En_volcaploop == true)
    {
        if (cap_loop_controller != NULL)
        {
            cap_loop_out = cap_loop_controller(Vcap, 29);
        }
        else
        {
            cap_loop_out = 0;
        }
    }
    else
    {
        cap_loop_out = 0;
    }
    // 跑功率环
    if (power_loop_controller != NULL)
    {
        power_loop_out = power_loop_controller(power_out_fact, chassis_power_target + cap_loop_out);
    }
    

    power_in_theory = chassis_power_target + power_loop_out + cap_loop_out;
    cal_feed_forward_total(power_in_theory, 0.11, -0.5); // 得到总前馈电流

    if (chassis_type == STEER_CHASSIS) // 舵轮优先分配舵功率
    {
        distribute_STEER(speed_target_6020, speed_current_6020);
        STEER_phase_current_trans(speed_target_6020, speed_current_6020, feed_forward_STEER);
    }

    distribute_wheel(speed_target_3508, speed_current_3508);                              // 差速分配
    wheel_phase_current_trans(speed_target_3508, speed_current_3508, feed_forward_wheel); // 转为相电流
}

/**
 * @brief  功率前馈
 * @note   由当前功率得到总前馈输出
 * @param
 * @return
 * @retval  None
 */
float DP_Power_Ctrl_Classdef::cal_feed_forward_total(float power, float _scale, float _residue)
{
    feed_forward_total = (_scale * power + _residue) * 819.2f; // （斜率*功率+截距）*比例

    if (feed_forward_total < 0)
    {
        feed_forward_total = 0; // 前馈不应该小于0
    }
    return feed_forward_total;
}

/**
 * @brief  求速度对应的稳态电流
 * @note
 * @param
 * @return
 * @retval  None
 */
float DP_Power_Ctrl_Classdef::speed_to_current(float _speed, float _scale, float _residue)
{
    float current;
    if (_speed == 0)
    {
        current = 0;
    }
    else if (_speed > 0)
    {
        current = (_speed / _scale + _residue) * 819.2f;
    }
    else if (_speed < 0)
    {
        current = (_speed / _scale - _residue) * 819.2f;
    }

    if (current > 16384)
        current = 16384;
    else if (current < -16384)
        current = -16384; // 电调电流限幅

    return current;
}

/**
 * @brief  分配舵前馈输出 麦轮无视
 * @note    舵轮可根据6020的功率需求对power_scale进行修改配置
 * @param
 * @return
 * @retval  None
 */
void DP_Power_Ctrl_Classdef::distribute_STEER(float *speed_target, float *now_speed)
{
    float STEER_error[4] = {0};
    float power_scale = 0.3;
    if (chassis_power_target <= 50)
    {
        power_scale = 0.3;
    }
    else if (chassis_power_target <= 60 && chassis_power_target > 50)
    {
        power_scale = 0.3;
    }
    else if (chassis_power_target <= 70 && chassis_power_target > 60)
    {
        power_scale = 0.3;
    }
    else if (chassis_power_target <= 80 && chassis_power_target > 70)
    {
        power_scale = 0.3;
    }
    else if (chassis_power_target <= 100 && chassis_power_target > 80)
    {
        power_scale = 0.3;
    }
    else if (chassis_power_target <= 120 && chassis_power_target > 100)
    {
        power_scale = 0.3;
    }
    else if (chassis_power_target <= 180 && chassis_power_target > 120)
    {
        power_scale = 0.3;
    }
    else if (chassis_power_target <= 300 && chassis_power_target > 180)
    {
        power_scale = 0.3;
    }

    float error_total = 0;
    float feed_STEER_total = 0;              // 舵总功率前馈
    for (int num = 0; num < num_6020; num++) // 6020的循环功率解算
    {
        STEER_error[num] = speed_to_current(speed_target[num], current_scale_6020, current_residue_6020) - speed_to_current(now_speed[num], current_scale_6020, current_residue_6020); // 求出目标功率和实际功率前馈的误差
        feed_STEER_total += fabsf(speed_to_current(speed_target[num], current_scale_6020, current_residue_6020));                                                                      // 舵电机的前馈总输出
        error_total += fabsf(STEER_error[num]);
    }

    for (int i = 0; i < num_6020; i++)
    {
        if (feed_STEER_total >= feed_forward_total * power_scale) // 按照功率系数分配功率给舵电机，剩下的给轮电机
        {
            feed_forward_STEER[i] = (STEER_error[i] / error_total) * (power_scale * feed_forward_total) + 10;
        }
        else if (feed_STEER_total < power_scale * feed_forward_total)
        {
            if (fabsf(speed_target[i] - now_speed[i]) > 10)
            {
                feed_forward_STEER[i] = (STEER_error[i] / error_total) * (feed_STEER_total) + 10;
            }
            else if (fabsf(speed_target[i] - now_speed[i]) <= 5)
            {
                if (fabsf(speed_target[i]) > 50)
                {
                    feed_forward_STEER[i] = speed_to_current(speed_target[i], current_scale_6020, current_residue_6020);
                }
                else if (fabsf(speed_target[i]) < 50)
                {
                    feed_forward_STEER[i] = 81.9;
                }
            }
        }
        else
        {
        }
        if (speed_target[i] == 0)
            feed_forward_STEER[i] = 0;
        else
        {
        }
    }
}

/**
 * @brief  差速分配轮前馈输出
 * @note
 * @param   轮目标速度，轮当前速度
 * @return
 * @retval  None
 */
void DP_Power_Ctrl_Classdef::distribute_wheel(float *speed_target, float *now_speed)
{

    float STEER_forward_out = 0;
    float error[4];
    float back_error[4] = {0};
    float back_error_total = 0;
    float error_total = 0;

    float feed_wheel_total = 0; // 轮总功率前馈
    // 识别差速
    /*识别加减速，将减速部分的前馈回收到总前馈，加速部分按照差速分配*/
    for (int i = 0; i < num_3508; i++)
    {
        if ((speed_target[i] > now_speed[i]) && (speed_target[i] * now_speed[i] >= 0) && (now_speed[i] >= 0)) // 正向加速,current>0
        {
            error[i] = fabsf(speed_to_current(speed_target[i], current_scale_3508, current_residue_3508) - speed_to_current(now_speed[i], current_scale_3508, current_residue_3508)); // 先得到当前剩余功率
        }
        else if ((speed_target[i] > now_speed[i]) && (speed_target[i] * now_speed[i] < 0) && (now_speed[i] < 0)) // 正向加速,current<0
        {
            error[i] = fabsf(speed_to_current(speed_target[i], current_scale_3508, current_residue_3508));
            back_error[i] = fabsf(speed_to_current(now_speed[i], current_scale_3508, current_residue_3508)); // 反向功率
        }
        else if ((speed_target[i] < now_speed[i]) && (speed_target[i] * now_speed[i] < 0) && (now_speed[i] > 0)) // 反向加速,current>0
        {
            error[i] = fabsf(speed_to_current(speed_target[i], current_scale_3508, current_residue_3508));
            back_error[i] = fabsf(speed_to_current(now_speed[i], current_scale_3508, current_residue_3508));
        }
        else if ((speed_target[i] < now_speed[i]) && (speed_target[i] * now_speed[i] >= 0) && (now_speed[i] <= 0)) // 反向加速,current<0
        {
            error[i] = fabsf(speed_to_current(speed_target[i], current_scale_3508, current_residue_3508) - speed_to_current(now_speed[i], current_scale_3508, current_residue_3508));
        }
        else if ((speed_target[i] < now_speed[i]) && (speed_target[i] * now_speed[i] >= 0) && (now_speed[i] > 0)) // 正向减速,current>0
        {
            back_error[i] = fabsf(speed_to_current(speed_target[i], current_scale_3508, current_residue_3508) - speed_to_current(now_speed[i], current_scale_3508, current_residue_3508));
        }
        else if ((speed_target[i] > now_speed[i]) && (speed_target[i] * now_speed[i] >= 0) && (now_speed[i] < 0)) // 反向减速,current<0
        {
            back_error[i] = fabsf(speed_to_current(speed_target[i], current_scale_3508, current_residue_3508) - speed_to_current(now_speed[i], current_scale_3508, current_residue_3508));
        }
        error_total += fabsf(error[i]); /*差速的差之和*/

        back_error_total += fabsf(back_error[i]);                                                            /*减速部分要回收的前馈*/
        feed_wheel_total += fabsf(speed_to_current(now_speed[i], current_scale_3508, current_residue_3508)); /*目前已经分配的前馈*/
    }
    // 分配前馈

    if (chassis_type == MECANUM_CHASSIS)
    {
        for (int i = 0; i < num_3508; i++)
        {
            if (fabsf(speed_target[i] - now_speed[i]) > 50)
            {
                if ((speed_target[i] > now_speed[i]) && (speed_target[i] * now_speed[i] >= 0) && (now_speed[i] >= 0)) // 正向加速,current>0
                {
                    feed_forward_wheel[i] = (error[i] / error_total) * (feed_forward_total + back_error_total - feed_wheel_total) + fabsf(speed_to_current(now_speed[i], current_scale_3508, current_residue_3508));
                }
                else if ((speed_target[i] > now_speed[i]) && (speed_target[i] * now_speed[i] < 0) && (now_speed[i] < 0)) // 正向加速,current<0
                {
                    feed_forward_wheel[i] = (error[i] / error_total) * (feed_forward_total + back_error_total - feed_wheel_total);
                }
                else if ((speed_target[i] < now_speed[i]) && (speed_target[i] * now_speed[i] < 0) && (now_speed[i] > 0)) // 反向加速,current>0
                {
                    feed_forward_wheel[i] = (error[i] / error_total) * (feed_forward_total + back_error_total - feed_wheel_total);
                }
                else if ((speed_target[i] < now_speed[i]) && (speed_target[i] * now_speed[i] >= 0) && (now_speed[i] <= 0)) // 反向加速,current<0
                {
                    feed_forward_wheel[i] = (error[i] / error_total) * (feed_forward_total + back_error_total - feed_wheel_total) + fabsf(speed_to_current(now_speed[i], current_scale_3508, current_residue_3508));
                }
                else if ((speed_target[i] < now_speed[i]) && (speed_target[i] * now_speed[i] >= 0) && (now_speed[i] > 0)) // 正向减速,current>0
                {
                    feed_forward_wheel[i] = fabsf(speed_to_current(speed_target[i], current_scale_3508, current_residue_3508));
                }
                else if ((speed_target[i] > now_speed[i]) && (speed_target[i] * now_speed[i] >= 0) && (now_speed[i] < 0)) // 反向减速,current<0
                {
                    feed_forward_wheel[i] = fabsf(speed_to_current(speed_target[i], current_scale_3508, current_residue_3508));
                }
            }
            else if (fabsf(speed_target[i] - now_speed[i]) <= 50)
            {
                feed_forward_wheel[i] = fabsf(speed_to_current(speed_target[i], current_scale_3508, current_residue_3508));
            }
            else
            {
            }
        }
    }
    else if (chassis_type == STEER_CHASSIS) // 舵轮车，需要先分配舵的前馈再分配轮的前馈。
    {
        for (int j = 0; j < num_6020; j++)
        {
            STEER_forward_out += fabsf(feed_forward_STEER[j]);
        }

        for (int i = 0; i < num_3508; i++)
        {
            if (fabsf(speed_target[i] - now_speed[i]) > 50)
            {
                if ((speed_target[i] > now_speed[i]) && (speed_target[i] * now_speed[i] >= 0) && (now_speed[i] >= 0)) // 正向加速,c>0
                {
                    feed_forward_wheel[i] = (error[i] / error_total) * (feed_forward_total + back_error_total - feed_wheel_total - STEER_forward_out) + fabsf(speed_to_current(now_speed[i], current_scale_3508, current_residue_3508));
                }
                else if ((speed_target[i] > now_speed[i]) && (speed_target[i] * now_speed[i] < 0) && (now_speed[i] < 0)) // 正向加速,c<0
                {
                    feed_forward_wheel[i] = (error[i] / error_total) * (feed_forward_total + back_error_total - feed_wheel_total - STEER_forward_out);
                }
                else if ((speed_target[i] < now_speed[i]) && (speed_target[i] * now_speed[i] < 0) && (now_speed[i] > 0)) // 反向加速,c>0
                {
                    feed_forward_wheel[i] = (error[i] / error_total) * (feed_forward_total + back_error_total - feed_wheel_total - STEER_forward_out);
                }
                else if ((speed_target[i] < now_speed[i]) && (speed_target[i] * now_speed[i] >= 0) && (now_speed[i] <= 0)) // 反向加速,c<0
                {
                    feed_forward_wheel[i] = (error[i] / error_total) * (feed_forward_total + back_error_total - feed_wheel_total - STEER_forward_out) + fabsf(speed_to_current(now_speed[i], current_scale_3508, current_residue_3508));
                }
                else if ((speed_target[i] < now_speed[i]) && (speed_target[i] * now_speed[i] >= 0) && (now_speed[i] > 0)) // 正向减速,c>0
                {
                    feed_forward_wheel[i] = fabsf(speed_to_current(speed_target[i], current_scale_3508, current_residue_3508));
                }
                else if ((speed_target[i] > now_speed[i]) && (speed_target[i] * now_speed[i] >= 0) && (now_speed[i] < 0)) // 反向减速,c<0
                {
                    feed_forward_wheel[i] = fabsf(speed_to_current(speed_target[i], current_scale_3508, current_residue_3508));
                }
            }
            else if (fabsf(speed_target[i] - now_speed[i]) <= 50)
            {
                feed_forward_wheel[i] = fabsf(speed_to_current(speed_target[i], current_scale_3508, current_residue_3508));
            }
            if (error_total <= 10)
            {
                feed_forward_wheel[i] = fabsf(speed_to_current(now_speed[i], current_scale_3508, current_residue_3508));
            }
            else
            {
            }
        }
    }
    // else if (chassis_type == BALANCE)
    // {
    //     for (int i = 0; i < num_3508; i++)
    //     {
    //         if (fabsf(speed_target[i] - now_speed[i]) > 50)
    //         {
    //             if ((speed_target[i] > now_speed[i]) && (speed_target[i] * now_speed[i] >= 0) && (now_speed[i] >= 0)) //正向加速,current>0
    //             {
    //                 feed_forward_wheel[i] = (error[i] / error_total) * (feed_forward_total + back_error_total - feed_wheel_total) + fabsf(speed_to_current(now_speed[i], current_scale_3508, current_residue_3508));
    //             }
    //             else if ((speed_target[i] > now_speed[i]) && (speed_target[i] * now_speed[i] < 0) && (now_speed[i] < 0)) //正向加速,current<0
    //             {
    //                 feed_forward_wheel[i] = (error[i] / error_total) * (feed_forward_total + back_error_total - feed_wheel_total);
    //             }
    //             else if ((speed_target[i] < now_speed[i]) && (speed_target[i] * now_speed[i] < 0) && (now_speed[i] > 0)) //反向加速,current>0
    //             {
    //                 feed_forward_wheel[i] = (error[i] / error_total) * (feed_forward_total + back_error_total - feed_wheel_total);
    //             }
    //             else if ((speed_target[i] < now_speed[i]) && (speed_target[i] * now_speed[i] >= 0) && (now_speed[i] <= 0)) //反向加速,current<0
    //             {
    //                 feed_forward_wheel[i] = (error[i] / error_total) * (feed_forward_total + back_error_total - feed_wheel_total) + fabsf(speed_to_current(now_speed[i], current_scale_3508, current_residue_3508));
    //             }
    //             else if ((speed_target[i] < now_speed[i]) && (speed_target[i] * now_speed[i] >= 0) && (now_speed[i] > 0)) //正向减速,current>0
    //             {
    //                 feed_forward_wheel[i] = fabsf(speed_to_current(speed_target[i], current_scale_3508, current_residue_3508));
    //             }
    //             else if ((speed_target[i] > now_speed[i]) && (speed_target[i] * now_speed[i] >= 0) && (now_speed[i] < 0)) //反向减速,current<0
    //             {
    //                 feed_forward_wheel[i] = fabsf(speed_to_current(speed_target[i], current_scale_3508, current_residue_3508));
    //             }
    //         }
    //         else if (fabsf(speed_target[i] - now_speed[i]) <= 50)
    //         {
    //             feed_forward_wheel[i] = fabsf(speed_to_current(speed_target[i], current_scale_3508, current_residue_3508));
    //         }
    //         else
    //         {
    //         }
    //     }
    // }
}

/**
 * @brief  3508实际电流转化为相电流输出
 * @note
 * @param
 * @return
 * @retval  None
 */
void DP_Power_Ctrl_Classdef::wheel_phase_current_trans(float *_target_speed, float *_speed, float *_feed_forward)
{
    float power[4];
    float wheel_phase_current[4] = {0}; // 轮输出相电流
    for (int i = 0; i < num_3508; i++)
    {
        power[i] = fabsf(_feed_forward[i] / 16384 * 20 * 24);
        wheel_phase_current[i] = fabsf(power[i] / 1.0f * 110000.0f / fabsf(_speed[i]));
        if (wheel_phase_current[i] > 14000)
            wheel_phase_current[i] = 14000;
        if (fabsf(_target_speed[i]) < 10)
        {
            wheel_phase_current[i] = 4000;
        }
        output.limit_current_3508[i] = wheel_phase_current[i];
    }
}
/**
 * @brief   6020实际电流转化为相电流输出
 * @note
 * @param
 * @return
 * @retval  None
 */
void DP_Power_Ctrl_Classdef::STEER_phase_current_trans(float *_target_speed, float *_speed, float *_feed_forward)
{
    float power[4] = {0};
    float STEER_speed[4] = {0};
    float STEER_phase_current[4] = {0}; // 舵输出相电流
    for (int i = 0; i < num_6020; i++)
    {
        STEER_speed[i] = _speed[i];
        power[i] = fabsf(_feed_forward[i] / 16384 * 20 * 24);

        if (_target_speed[i] * _speed[i] >= 0)
        {
            if (fabsf(STEER_speed[i]) < 3)
            {
                STEER_speed[i] = 0;
            }

            STEER_phase_current[i] = (power[i] * power[i] * 130.0f * 130.0f) / (fabsf(STEER_speed[i]) + 10);
        }
        else if (_target_speed[i] * _speed[i] < 0)
        {
            STEER_phase_current[i] = 20000;
        }
        else
        {
        }

        if (STEER_phase_current[i] > 20000)
        {

            STEER_phase_current[i] = 20000;
        }
        else if (STEER_phase_current[i] < -20000)
        {
            STEER_phase_current[i] = -20000;
        }
        else
        {
        }
        if (fabsf(_speed[i]) < 20)
        {
            STEER_phase_current[i] = 15000;
        }
        else
        {
        }
        output.limit_current_6020[i] = STEER_phase_current[i];
    }
}
#endif /* USE_SRML_DIGITAL_POWER */
/************************ COPYRIGHT(C) SCUT-ROBOTLAB**************************/
