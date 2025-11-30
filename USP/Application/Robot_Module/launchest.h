#pragma once

/* Includes ------------------------------------------------------------------*/
#include "SRML.h"
/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
class Launch_Classdef
{
private:
    GPIO_PinState (*READ_DELIVERSWITCH[2])(void);
 
    /* PID */

public:
	 float TargetAngle_Deliver = -200;
   float goal;
		myPID PID_Deliver_Speed[2];
    myPID PID_Deliver_Angle[2];
    myPID PID_Deliver_Diff;
    myPID PID_Igniter_Speed;
    myPID PID_Igniter_Angle;
    Launch_Classdef(uint8_t _ID_DELIVER_R, uint8_t _ID_DELIVER_L, uint8_t _ID_IGNITER_R);
    /* DjiMotor */
    abstractMotor<Motor_C620> DeliverMotor[2];
    abstractMotor<Motor_C610> IgniterMotor;
    /* 复位执行函数 */
    void Deliver_Init();
    void Igniter_Init();
    /*传送带下拉函数*/
    void Deliver_Pull();
    void Deliver_Back();
    /* 丝杆移动函数*/
    void Igniter_Pos_Set(float _target_pos);
    /* 扳机控制函数*/
    void Igniter_On();
    void Igniter_Off();
    /*电机打包函数*/
    void Deliver_test();
    /* 各种标志位*/
    bool Pull_Ready_flag = false;
    bool Igniter_Init_flag = false;
    bool Deliver_Init_flag[2] = {false, false};

    inline bool is_Deliver_Init() { return Deliver_Init_flag[0] & Deliver_Init_flag[1]; }
    void adjust();
    void disable();
};
/* Exported macros -----------------------------------------------------------*/
bool Is_Launching();
/* Exported types ------------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/




/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/