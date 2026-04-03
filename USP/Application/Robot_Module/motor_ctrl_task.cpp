#include "internal.h"
#include "global_data.h"
#include "robot_config.h"


void task_motor_ctrl(void *arg)
{
    TickType_t xLastWakeTime_t;
    xLastWakeTime_t = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2);

    CAN_COB Tx_Buff = {};
        abstractMotor<Motor_DM_classdef> mymotor(0x202); // 创建一个抽象电机实例，绑定ID为0x202的达妙电机类对象
    vTaskDelay(100); // 稍微延时一下，确保电机相关的CAN发送队列已经创建并且系统稳定后再初始化电机，避免在系统刚启动时就发送CAN消息可能导致的问题
    mymotor.init(CAN1_TxPort); // 绑定CAN1发送队列并使能电机
	vTaskDelay(100);	
	mymotor.init(CAN1_TxPort);
    mymotor.speed_unit_convert = 29.0f / 43.0f; //电机侧/实际侧=29/43，函数会除以这个值，即实际发送=设置目标/(29/43)
    
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
                g_SystemState.TargetSpeed = 1/(3.0f * PI) ; // 约 0.1061 rad/s, 6.28 rad/60s, 1转/60s
                break;
                
            case big_energy: // 大能量机关
                {
                    float param_a = g_TargetCtrl.BigEnergy_A;
                    float param_w = g_TargetCtrl.BigEnergy_W;
                    
                    // 参数限幅 (保持原有逻辑)
                    if(param_a > 1.045f) param_a = 1.045f;
                    else if(param_a < 0.780f) param_a = 0.780f;
                    
                    if(param_w > 2.000f) param_w = 2.000f;
                    else if(param_w < 1.884f) param_w = 1.884f;

                    // 计算大符速度
                    //g_SystemState.TargetSpeed = (param_a * sin(param_w * time_clock/1000.0f) + 2.090f - param_a) * motor_reduction_ratio_t * 4 * 60 / 6.28f;

                    g_SystemState.TargetSpeed = 1/(3.0f * PI) ;//在单位确定之前，先使用恒定转速，方便测试和调试。后续可以根据实际情况调整为上述计算方式。
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
        
        //以下方式为达妙电机速度模式的直接控制方式，未使用 PID 调节.
        // if(g_SystemState.SysMode == idle){
        //     g_SystemState.TargetSpeed = 0;
        // }
        // else{
        //     if(mymotor.){ // 电机未使能，强制使能
        //         motor_ctrl.mymotor.startMotor(); // 使能电机
        //     }
        //     else if(motor_ctrl.dm_motor_recdata.state!=1){ // 电机状态异常，强制使能
        //         motor_ctrl.mymotor.ClearError(); // 清除错误
        //         motor_ctrl.mymotor.startMotor(); // 使能电机
        //     }
        // }
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
