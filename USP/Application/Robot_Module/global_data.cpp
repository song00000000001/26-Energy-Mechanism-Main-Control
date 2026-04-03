#include "global_data.h"
#include "robot_config.h"
#include "internal.h"

Debug_Data_t Debugger;//调试数据结构体实例化

#ifdef INCLUDE_uxTaskGetStackHighWaterMark
stack_remain_t Stack_Remain={255};
#endif

UpperCtrlPacket_t upper_ctrl_packet;// 上位机控制包实例

//全局变量定义
// 系统状态实例
EnergySystemState_t g_SystemState ;
target_ctrl_t g_TargetCtrl;