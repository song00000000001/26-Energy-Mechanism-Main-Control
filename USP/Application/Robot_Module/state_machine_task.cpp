#include "internal.h"
#include "global_data.h"
#include "motor_ctrl_driver.h"
#include "remote_ctrl_driver.h"
#include "robot_config.h"
#include "ws2812_ctrl_driver.h"

void state_machine_reset();

uint8_t debug_test_light_effect[5] = {0}; // 调试用灯效选择

//状态机任务
void task_state_machine(void *arg)
{
    // 任务频率控制
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    uint32_t main_task_now = xTaskGetTickCount();

    Motor_CAN_COB Tx_Buff = {};

    Debugger={
        .enable_debug_mode=debug_idle,
    };
     // --- 初始化部分 ---
    // 启动 ADC DMA (只需要启动一次)
    static uint32_t adcSeed = 0;
    adc_start(&adcSeed);
    
    // 稍微延时一下，确保 DMA 已经搬运了至少一次数据到 adcSeed
    // (因为 DMA 启动到完成第一次传输需要一点点时间，虽然极短)
    vTaskDelay(5); 

    // 设置随机数种子 (只需要执行一次)
    // 结合 ADC 悬空值和当前时间戳，保证每次上电的随机序列都不同
    srand(adcSeed + xTaskGetTickCount()); 

    g_SystemState.SysMode=small_energy; //默认小能量机关模式
    g_SystemState.BE_StateData.BE_Group = 0;
    g_SystemState.BE_StateData.BE_State = BE_GENERATE_TARGET;
    g_SystemState.CurrentHitID = 0;
    g_SystemState.SE_StateData.SE_Group = 0; // 小能量轮数
    g_SystemState.SE_StateData.SE_State = SE_GENERATE_TARGET; // 小能量状态机
    g_TargetCtrl={
        .target_mode = tar_test_mode,        // 默认停止/待机
        .TargetColor = color_red,      // 默认红色
        .BigEnergy_A = 0.9125f,        // 默认大符参数 (0.780 + 1.045)/2
        .BigEnergy_W = 1.942f          // 默认大符参数 (1.884 + 2.000)/2
    };
    g_TargetCtrl.UpperCtrlBool.upperctrl_timeout_reset_enable = true; // 默认启用超时重置
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        main_task_now = xTaskGetTickCount();
        
        debug_simulate_hit_f(); // 调试函数，模拟击打事件
        
        //模式切换
        switch (g_TargetCtrl.target_mode)
        {
        case tar_stop: // 停止/待机
            g_SystemState.SysMode=idle;
            break;
        case tar_start: // 激活
            g_SystemState.SysMode=wait_start;
            break;
        case tar_small_energy_signle: // 小能量机关
        case tar_small_energy_continue: // 连续小能量机关
            g_SystemState.SysMode = small_energy; // 切换到小能量机关
            break;
        case tar_big_energy_single: // 大能量机关
        case tar_big_energy_continue: // 连续大能量机关
            g_SystemState.SysMode = big_energy; // 切换到大能量机关
            break;
        case tar_test_mode:
            g_SystemState.SysMode = test_mode;
            break;
        case tar_success:
            g_SystemState.SysMode = success;
            break;
        case tar_successed:
            g_SystemState.SysMode = successed;
        default:
            break;
        }

        //由于模式切换变成立即覆盖,可能存在小符运行到一半被打断出现异常,
        //因此在每次模式切换时重置状态机,确保状态机从初始状态开始运行
        static EnergySystemMode_t last_mode = idle;
        if(g_SystemState.SysMode != last_mode) {
            state_machine_reset();
            last_mode = g_SystemState.SysMode;
        }
        // 模式判断
        switch (g_SystemState.SysMode)
        {
        case idle:
        {
            state_machine_reset();
            if(g_TargetCtrl.target_mode == tar_stop)
                g_SystemState.SysMode=idle;
            else
                g_SystemState.SysMode=wait_start;   
        }
        break;
        case wait_start:
        {
            //state_machine_reset();
            g_SystemState.TargetSpeed = 0.1f; // 初始目标速度
        }
        break;
        case small_energy:
        {
            small_energy_logic();
        }
        break;
        case big_energy:
        {
            big_energy_logic();
        }
        break;
        case success:
        {
            //优化成非阻塞版本，避免在闪烁过程中任务被占用无法响应其他事件
            //用简易的状态机实现闪烁逻辑
            static int8_t flash_count = 7; // 记录当前闪烁次数
            static TickType_t last_flash_time = 0;
            TickType_t now = xTaskGetTickCount();
            static int8_t flash_state = 1; 

            // 切换状态
            if(flash_count == 0) {
                // 闪烁完成，重置状态
                flash_count = 7;//为下一次使用重置闪烁次数
                g_TargetCtrl.target_mode = tar_successed;
				break;
            }
            else if(now - last_flash_time > 200) { // 每500ms切换一次状态
                flash_state=-flash_state; // 状态在-1和1之间切换
                flash_count--;
                last_flash_time = now;
            }

            switch (flash_state)
            {
            case -1: // 全灭
                all_off_effect();
                break;
            case 1: // 全亮
                all_on_effect();
                break;
            default:
                break;
            }

        }
        break;
        case successed:
        {
            // 永久全亮，保持在成功状态
            all_on_effect();

        }
		break;
        case test_mode:
        {
            // 测试模式，保留。
            //g_SystemState.TargetSpeed = 0;
            //test_light_effect(debug_test_light_effect);
        }
        break;
        default:
            //state_machine_reset();
        break;
        }
                
        //记录任务剩余栈空间
        #ifdef INCLUDE_uxTaskGetStackHighWaterMark
        //Stack_Remain.task_state_machine_stack_remain = uxTaskGetStackHighWaterMark(NULL);
        #endif
    }
}

void state_machine_reset(){
    g_SystemState.BE_StateData.BE_Group = 0;
    g_SystemState.BE_StateData.BE_State = BE_GENERATE_TARGET;
    g_SystemState.BE_StateData.BE_Targets[0] = 0;
    g_SystemState.BE_StateData.BE_Targets[1] = 0;
    g_SystemState.CurrentHitID = 0;
    g_SystemState.SE_StateData.SE_Group = 0; // 重置小能量轮数
    g_SystemState.SE_StateData.SE_State = SE_GENERATE_TARGET; // 重置小能量状态机
    all_off_effect(); // 熄灭所有装甲板
    //R_light(color_off); // 熄灭R标灯
    //g_TargetCtrl.TargetColor = color_off; // 重置目标颜色
}

void debug_simulate_hit_f() {
    if (Debugger.Debug_simulate_hit) {
        Debugger.Debug_simulate_hit = false; // 重置模拟击打标志
        if(g_SystemState.SysMode == small_energy && g_SystemState.SE_StateData.SE_State == SE_WAIT_HIT) {
            g_SystemState.CurrentHitID = g_SystemState.SE_StateData.SE_TargetID; // 模拟击中目标
        }
        else if(g_SystemState.SysMode == big_energy && (g_SystemState.BE_StateData.BE_State == BE_WAIT_HIT_1 || g_SystemState.BE_StateData.BE_State == BE_WAIT_HIT_2)) {
            if(g_SystemState.BE_StateData.BE_State == BE_WAIT_HIT_1) {
                g_SystemState.CurrentHitID = g_SystemState.BE_StateData.BE_Targets[0]; // 模拟击中第一个目标
            }
            else {
                g_SystemState.CurrentHitID = g_SystemState.BE_StateData.BE_Targets[1]; // 模拟击中第二个目标
            }
        }
        g_SystemState.CurrentHitScores = rand() % 10 + 1; // 模拟得分为1-10之间的随机数
    }
}

//增加统一的跳过超时逻辑的函数，便于大小符状态机调用，避免重复代码
bool is_check_timeout_enable(void) {
    return (g_TargetCtrl.UpperCtrlBool.upperctrl_timeout_reset_enable); 
}

bool is_lock_state(void) {
    return (g_TargetCtrl.UpperCtrlBool.upperctrl_lock_state_enable);
}