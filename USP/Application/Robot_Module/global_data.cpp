#include "global_data.h"
#include "robot_config.h"
#include "internal.h"

DartDataStructdef DartsData[MAX_DART_DATAPOOL_SIZE]; //发射数据
uint8_t DartDataSlot[5]={0,1,2,3,4};
DartAimEnumdef HitTarget;
uint8_t ParamSendPack[9];
uint8_t CurrentCnt=1;
float Yaw_Angle[2];
float _YawCorrectionAngle;//yaw轴修正角
Motor_GM6020 loadermotor[1]{5};//装填电机
Launch_Classdef Launch(ID_DELIVER_R, ID_DELIVER_L, ID_IGNITER); //发射类
Missle_YawController_Classdef Yaw(ID_YAW);//yaw控制类
VisionRecvData_t vision_recv_pack;//电视接收包
VisionSendData_t vision_send_pack;//电视发送包
uint32_t vision_last_recv_time = 0;

Missle_State_t state = DEINIT;
float yaw_target = 0, yaw_goal = 0, igniter_target_pos = 0, igniter_goal_pos = 0;

int status = 1;						            // 装填状态（1~4分别对应4发飞镖）
int cnt = 0;									// 控制status在每次装填后只加一
int goal;										// 装填电机的目标值
int open = 0;									// 此值为0时装填镖体，为1时准备发射
float visionangle;								// 视觉目标角
myPID anglepid;									// 装填电机的编码器环
myPID speedpid;									// 装填电机的速度环
bool vision_aim_state = 0;				// 视觉瞄准状态
float vision_base_angle, storage_base_angle; // 视觉基准角、原基准角暂存
