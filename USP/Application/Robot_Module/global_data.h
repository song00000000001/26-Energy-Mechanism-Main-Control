#pragma once

#include "stdint.h"
#include "launchest.h"
#include "Yaw_control.h"

// 飞镖数据存储相关定义
//最大飞镖数据池大小?待确认
#define MAX_DART_DATAPOOL_SIZE 16

//飞镖数据结构体,用于选择目标是基地还是前哨站
typedef enum __DartAimEnumdef
{
  Outpost = 0,
  Base = 1
}DartAimEnumdef;

//飞镖数据结构体,用于存储飞镖目标数据
typedef struct __DartDataStructdef
{
  double Ignitergoal[2]; //扳机目标位置
  double YawCorrectionAngle[2];//偏航修正角
}DartDataStructdef;

/**
 * @brief 发射主控任务
 * @parma None
 * @return None
 */
enum Missle_State_t
{
	DEINIT,
	WAIT_ACT,
	PULL,
	BACK,
	WAIT_SHOOT
};

enum
{
  R = 0,
  L = 1
};

// 定义一个本地结构体，只保存我们需要的数据
struct DR16_Snapshot_t {
    LinkageStatus_Typedef Status;
    SW_Status_Typedef S1;
    SW_Status_Typedef S2;
    float RY_Norm;
    float RX_Norm;
};

/* 状态机枚举定义建议 (在 internal.h) */
enum LaunchState_e {
    STATE_OFFLINE,      // 离线/失能
    STATE_DEINIT,       // 正在初始化/归位
    STATE_WAIT_CMD,     // 等待发射指令 (滑块在缓冲区，随时可发)
    STATE_PULLING,      // 正在下拉蓄力
    STATE_LATCHING,     // 到底部，等待挂机稳定 (User code: PULL delay)
    STATE_RETURNING,    // 正在回升到缓冲区
    STATE_FIRING,       // 正在执行发射动作
    STATE_RELOADING,    // (预留) 正在执行自动装填动作
    STATE_COOLDOWN      // 发射后冷却
};

extern DartDataStructdef DartsData[MAX_DART_DATAPOOL_SIZE]; // 飞镖数据
extern uint8_t DartDataSlot[5];                             // 发射数据选择
extern DartAimEnumdef HitTarget;                            // 打击目标
extern uint8_t ParamSendPack[9];
extern uint8_t CurrentCnt; // 当前发数
extern float Yaw_Angle[2]; // 默认前哨站和基地角度


// 调试/通信用变量
extern Missle_State_t state; // 发射状态

// 声明全局机构对象
extern Launch_Classdef Launch; // 发射类
extern Missle_YawController_Classdef Yaw; // yaw控制类
extern float yaw_target , yaw_goal , igniter_target_pos , igniter_goal_pos;

extern int status;
extern int cnt;
extern int goal;
extern int open;
extern float visionangle;
extern myPID anglepid;
extern myPID speedpid;
extern bool vision_aim_state;
extern float vision_base_angle;
extern float storage_base_angle;


/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

