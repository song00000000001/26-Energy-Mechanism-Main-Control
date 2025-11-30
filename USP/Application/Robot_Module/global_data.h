#pragma once

#ifdef __cplusplus
extern "C"{
#endif

#include "stdint.h"

// 声明全局机构对象


// 声明任务句柄


// 调试/通信用变量

#define INIT_SPEED_DELIVER            6000
#define INIT_SPEED_IGNITER            -4500
#define INIT_SPEED_YAW                -500


#define SW_DELIVER_L_Pin              GPIO_PIN_4
#define SW_DELIVER_L_GPIO_Port        GPIOC
#define SW_DELIVER_R_Pin              GPIO_PIN_5
#define SW_DELIVER_R_GPIO_Port        GPIOA
#define SW_IGNITER_Pin                GPIO_PIN_14
#define SW_IGNITER_GPIO_Port          GPIOC

#define SW_YAW_R_Pin                  GPIO_PIN_6
#define SW_YAW_R_GPIO_Port            GPIOA
#define SW_YAW_L_Pin                  GPIO_PIN_7
#define SW_YAW_L_GPIO_Port            GPIOA


#define POLARITY_DELIVER_L            -1
#define POLARITY_DELIVER_R            1
#define POLARITY_IGNITER              1
#define POLARITY_YAW                  1

#define ID_DELIVER_L                  4
#define ID_DELIVER_R                  3
#define ID_IGNITER                    1
#define ID_YAW                        2

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



extern DartDataStructdef DartsData[MAX_DART_DATAPOOL_SIZE]; // 飞镖数据
extern uint8_t DartDataSlot[5];                             // 发射数据选择
extern DartAimEnumdef HitTarget;                            // 打击目标
extern uint8_t ParamSendPack[9];
extern uint8_t CurrentCnt; // 当前发数
extern float Yaw_Angle[2]; // 默认前哨站和基地角度

#ifdef  __cplusplus
}
#endif



/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

