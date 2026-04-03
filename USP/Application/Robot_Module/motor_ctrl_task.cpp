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

#define rand_float_0_1 ((float)rand() / (float)RAND_MAX)

#define motor_speed_max 3.0f // 电机最大速度，单位 rad/s
#define motor_reduction_ratio 29.0f / 43.0f //链条传动减速比。电机侧/实际侧=29/43，函数会除以这个值，即实际发送=设置目标/(29/43)

void task_motor_ctrl(void *arg)
{
    TickType_t xLastWakeTime_t;
    xLastWakeTime_t = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);

    static EnergySystemMode_t last_mode = idle;
    static float locked_big_energy_a = (be_motor_speed_a_min + be_motor_speed_a_max) * 0.5f;
    static float locked_big_energy_w = (be_motor_speed_w_min + be_motor_speed_w_max) * 0.5f;
    Debugger.debug_dm_motor_speed_k=1.0f; // 初始速度参数缩放因子
    /**
     * 测试
     * target_speed=0时,get_speed=1380
     * target_speed=1.047时,get_speed=1466
     * get_speed与target_speed近似线性关系，且斜率约为(1466-1380)/1.047=83.3
     * 
     */

    vTaskDelay(1000); // 稍微延时一下，确保电机相关的CAN发送队列已经创建并且系统稳定后再初始化电机，避免在系统刚启动时就发送CAN消息可能导致的问题
	mymotor.init(CAN1_TxPort);
    mymotor.speed_unit_convert = motor_reduction_ratio; // 设置速度单位转换，考虑减速比的影响，实际发送给电机的速度=设置的目标速度/减速比
    
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime_t, xFrequency);

        // 实时更新时间戳
        uint32_t time_clock = xTaskGetTickCount();

        // 每次进入大符模式时，重新随机参数并在本轮大符期间锁定不变
        if (g_SystemState.SysMode != last_mode) {
            if (g_SystemState.SysMode == big_energy) {
                locked_big_energy_a = be_motor_speed_a_min + rand_float_0_1 * (be_motor_speed_a_max - be_motor_speed_a_min);
                locked_big_energy_w = be_motor_speed_w_min + rand_float_0_1 * (be_motor_speed_w_max - be_motor_speed_w_min);

                // 同步到全局参数，便于调试/上位机查看
                g_TargetCtrl.BigEnergy_A = locked_big_energy_a;
                g_TargetCtrl.BigEnergy_W = locked_big_energy_w;
            }
            last_mode = g_SystemState.SysMode;
        }
  
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
                    float param_a = locked_big_energy_a;
                    float param_w = locked_big_energy_w;

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
