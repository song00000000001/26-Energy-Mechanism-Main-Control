#pragma once

#include "internal.h"
#include "remote_ctrl_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum{
    debug_idle = 0,
    debug_mtvofa_monitor,
    debug_mpuvoda_monitor
}Debug_Mode_e;

//调试数据结构体
typedef struct {
    // 标志位
    Debug_Mode_e enable_debug_mode; // 在watch窗口改为true以进入调试模式(配合遥控器)
    bool Debug_simulate_hit;// 模拟击打（进入调试模式后，在watch窗口改为true以模拟一次击打事件，配合遥控器）
    uint8_t debug_arm_light_effect; // 调试用灯效选择（进入调试模式后，在watch窗口修改以切换不同灯效，配合遥控器）
    float debug_dm_motor_speed_k;//速度参数缩放因子,获取的速度有点奇怪,单位未知
} Debug_Data_t;
extern Debug_Data_t Debugger;  

#ifdef INCLUDE_uxTaskGetStackHighWaterMark
//定义栈剩余空间记录结构体
typedef struct 
{
    uint16_t FS_I6X_stack_remain;
    uint16_t motor_ctrl_stack_remain;
    uint16_t state_machine_stack_remain;
    uint16_t armer_ctrl_stack_remain;
    uint16_t uart_comm_stack_remain;
    uint16_t debug_send_stack_remain;
    uint16_t vision_comm_stack_remain;
    uint16_t can_comm_stack_remain;

}stack_remain_t;
extern stack_remain_t Stack_Remain;

//把栈水位获取封装成宏
#define StackWaterMark_Get(x)  \
    do{ \
        Stack_Remain.x##_stack_remain = uxTaskGetStackHighWaterMark(NULL); \
    }while(0)

#endif

//system state enum
typedef enum{
    idle = 0,
    wait_start,
    small_energy,
    big_energy,
    success,
    successed,
    test_mode
}EnergySystemMode_t;

//big energy state enum
typedef enum{
    BE_GENERATE_TARGET = 0,
    BE_WAIT_HIT_1,
    BE_WAIT_HIT_2,
    BE_END
}BigEnergyState_t;

//small energy state enum
typedef enum{
    SE_GENERATE_TARGET = 0,
    SE_WAIT_HIT,
    SE_END
}SmallEnergyState_t;

//颜色枚举
typedef enum 
{
    color_off = 0,
    color_red,
    color_blue
}light_color_enum;

typedef enum{
    tar_stop = 0,
    tar_start,
    tar_small_energy_signle,
    tar_big_energy_single,
    tar_small_energy_continue,
    tar_big_energy_continue,
    tar_test_mode,
    tar_success,
    tar_successed
}EnergyTargetMode_t;


/**
 * 新增上位机控制包，由于当前上位机只有按钮控件，打算用bool型的变量用于切换状态。
 * 目前的按键保留：
 * 停止(a5 00)，小符(a5 02)，大符(a5 03)，连续小符(a5 04)，连续大符(a5 05)
 * 颜色切换:a0 00
 * 电机使能切换:a0 01
 * 大小符取消超时重置切换:a0 02
 * 模拟击打：ff 01
 */
typedef struct{
    bool upperctrl_color_toggle; // 上位机控制颜色切换
    bool upperctrl_motor_enable; // 上位机控制电机使能切换
    bool upperctrl_timeout_reset_enable; // 上位机控制超时重置切换
    bool upperctrl_lock_state_enable; // 上位机控制锁定状态切换
} UpperCtrlBool_t;

typedef struct {
    EnergyTargetMode_t target_mode;    // 0:停止/待机, 1:激活, 2. 小能量机关, 3:大能量机关 ,4: 连续小能量机关,5: 连续大能量机关
    light_color_enum  TargetColor;   // 0:Red, 1:Blue
    float    BigEnergy_A;   // 大符正弦 A
    float    BigEnergy_W;   // 大符正弦 Omega
    UpperCtrlBool_t UpperCtrlBool; // 上位机控制的布尔变量
}target_ctrl_t;
extern target_ctrl_t g_TargetCtrl;

//小能量机关状态结构体
typedef struct{
    // --- 小能量机关状态变量  ---
    SmallEnergyState_t  SE_State;      // 状态机: 0:生成, 1:等待击打, 2:结束
    uint8_t  SE_TargetID; // 当前目标ID (1-5)
    uint8_t  SE_Group;      // 当前轮数 (0，1-5)
    uint8_t  SE_TargetID_GROUP[5];   // 当前目标序列组 
    uint32_t SE_StateTimer; // 状态计时器
    uint8_t  SE_Scores;
}SmallEnergyStateData_t;

//大能量机关状态结构体
typedef struct{
    // --- 大能量机关状态变量  ---
    uint8_t  BE_Group;      // 当前轮数 (0，1-5)
    uint8_t  BE_Targets[2]; // 当前两个目标ID (1-5)
    BigEnergyState_t  BE_State;      // 状态机: 0:生成, 1:等待首击, 2:等待连击, 3:结束
    uint32_t BE_StateTimer; // 状态计时器
    uint8_t  BE_Scores;
    uint8_t  BE_ActivedArms; // 当前激活的装甲板数量 (1-10)
}BigEnergyStateData_t;

// 全局状态结构体 - 用于Debug控制和系统状态管理
typedef struct {
    EnergySystemMode_t  SysMode;       // 0:停止/待机, 1:小能量机关, 2:大能量机关
    SmallEnergyStateData_t SE_StateData; // 小能量机关状态数据
    BigEnergyStateData_t BE_StateData; // 大能量机关状态数据
    uint8_t  CurrentHitID;  // 当前被击中的ID (反馈)
    uint8_t  CurrentHitScores; // 当前得分 (根据击打情况计算，供上位机显示)
    float    TargetSpeed;   // 当前计算目标速度 (供上位机显示)
    float    dm_motor_kd; // 达妙电机速度模式的kd参数，供上位机显示和调整
    float    dm_motor_torque; // 达妙电机当前力矩，供上位机显示
} EnergySystemState_t;

extern EnergySystemState_t g_SystemState;

// 通信包结构体 (4字节定长)
#pragma pack(1)
typedef struct {
    uint8_t header;       // 固定为 0xA5
    uint8_t cmd;          // 见 FanCmdType
    uint8_t color;        // 0:Blue, 1:Red (与 Current_color 保持一致)
    //uint8_t target_id;    // 目标装甲板ID (虽然物理线路已区分，但保留此字段用于校验或显示)
    uint8_t current_stage;// 新增：当前击打阶段/组数 (0-5)，用于大符阶梯式亮灯效果
} FanPacket_t;
#pragma pack()

#pragma pack(1)
typedef struct {
    uint8_t ctrl_header;
    uint8_t ctrl_content;
} UpperCtrlPacket_t;
#pragma pack()
extern UpperCtrlPacket_t upper_ctrl_packet;

extern Motor_DM_classdef mymotor; // 声明全局电机实例

void small_energy_logic();
void big_energy_logic();
void debug_simulate_hit_f() ;//调试函数，模拟一次一定正确且环数随机的击打事件

void all_off_effect();
void all_on_effect();
void se_select_effect(uint8_t arm_id);
void se_hit_effect(uint8_t arm_id);
void be_select_effect(uint8_t arm_id);
void be_stage_effect(uint8_t arm_id);
void be_hit_effect(uint8_t arm_id);
void test_light_effect(uint8_t effect[5]);
bool is_check_timeout_enable(void);
bool is_lock_state(void);
#ifdef __cplusplus
}
#endif