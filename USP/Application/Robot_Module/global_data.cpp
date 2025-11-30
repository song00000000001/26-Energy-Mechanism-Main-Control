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