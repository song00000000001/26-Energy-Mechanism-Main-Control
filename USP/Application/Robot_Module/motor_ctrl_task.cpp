#include "internal.h"
#include "global_data.h"
#include "robot_config.h"

// 1/3*PI 约1.047 rad/s, 1转/6s
#define se_motor_speed 1/(3.0f) * PI
//a=0.78~1.045, w=1.884~2.000
//spd=(a*sin(w*t)+2.09-a),约0.0873~0.125 rad/s, 5.24~7.46 rad/60s, 0.87~1.25转/60s
#define be_motor_speed_w_max 2.000f
#define be_motor_speed_w_min 1.884f
#define be_motor_speed_a_max 1.045f
#define be_motor_speed_a_min 0.780f
#define be_motor_speed_ba  2.090f

#define mymotor_id 0x202 // 电机ID
#define motor_speed_max 3.0f // 电机最大速度，单位 rad/s
#define motor_reduction_ratio 29.0f / 43.0f //链条传动减速比。电机侧/实际侧=29/43，函数会除以这个值，即实际发送=设置目标/(29/43)

void task_motor_ctrl(void *arg)
{
    TickType_t xLastWakeTime_t;
    xLastWakeTime_t = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2);

    abstractMotor<Motor_DM_classdef> mymotor(mymotor_id); // 创建一个抽象电机实例，绑定ID为mymotor_id的达妙电机类对象
    vTaskDelay(1000); // 稍微延时一下，确保电机相关的CAN发送队列已经创建并且系统稳定后再初始化电机，避免在系统刚启动时就发送CAN消息可能导致的问题
	mymotor.init(CAN1_TxPort);
    mymotor.speed_unit_convert = motor_reduction_ratio; // 设置速度单位转换，考虑减速比的影响，实际发送给电机的速度=设置的目标速度/减速比
    
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime_t, xFrequency);

        // 实时更新时间戳
        uint32_t time_clock = xTaskGetTickCount();
  
        // 状态机处理
        switch(g_SystemState.SysMode)
        {
            // 正常是恒定转速1/(3*pi) rad/s, 约
            case wait_start: // 等待开始
            case success: // 通关成功
            case successed: // 已经通关
            case small_energy: // 小能量机关
                g_SystemState.TargetSpeed = se_motor_speed; 
                break;
                
            case big_energy: // 大能量机关
                {
                    g_TargetCtrl.BigEnergy_A = std_lib::constrain(g_TargetCtrl.BigEnergy_A, be_motor_speed_a_min, be_motor_speed_a_max);
                    g_TargetCtrl.BigEnergy_W = std_lib::constrain(g_TargetCtrl.BigEnergy_W, be_motor_speed_w_min, be_motor_speed_w_max);
                    float param_a = g_TargetCtrl.BigEnergy_A;
                    float param_w = g_TargetCtrl.BigEnergy_W;

                    // 计算大符速度
                    g_SystemState.TargetSpeed = (param_a * sin(param_w * time_clock/1000.0f) + be_motor_speed_ba - param_a);
                }
                break;

            case idle: // 停止/待机                
            default:
                g_SystemState.TargetSpeed = 0;
                break;
            case test_mode: // 测试模式
                //无,直接在debug改
                break;
        }  

        // 速度控制
        g_SystemState.TargetSpeed = std_lib::constrain(g_SystemState.TargetSpeed, -motor_speed_max, motor_speed_max); // 限幅
        // 发送给电机
        /**
         * 达妙电机的速度模式控制接口，直接设置目标速度和kd参数，电机会根据内部算法计算出合适的控制量（电流/扭矩）来实现目标速度。
         * 内部实现会调用motor_dm的control函数，传入位置=0（不使用位置控制），速度=目标速度，kp=0（不使用位置控制），kd=用户设置的参数，torque=0（不使用力矩控制）。
         * control函数会根据这些参数计算出最终的控制量，并通过之前绑定的CAN发送给电机。
         */
        mymotor.setMotorSpeed(g_SystemState.TargetSpeed);

        #ifdef INCLUDE_uxTaskGetStackHighWaterMark
        StackWaterMark_Get(motor_ctrl);
        #endif
    }
}
