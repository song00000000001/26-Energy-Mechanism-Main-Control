/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    MultiTurnRecorder.h
  * @author  余俊晖 2460857175@QQ.COM
  * @brief   多圈角度记录器（用于电机、单圈编码器等设备）
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */
#pragma once

#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/
#include "srml_std_lib.h"
/* Private macros ------------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

class MultiTurnRecorderClassdef
{
public:
    inline void ClearRoundCnt() { round_cnt = 0; }
protected:
    void setAngleOffset(float _angle_offset)
    {
        angle_is_init = 1;
        last_singleTurnAngle = angle_offset = _angle_offset;
    }

    float MultiTurnRecord(float SingleTurnAngle)
    {
        if (angle_is_init)
        {
            if (SingleTurnAngle - last_singleTurnAngle > 180)
                this->round_cnt--;
            else if (SingleTurnAngle - last_singleTurnAngle < -180)
                this->round_cnt++;
        }
        else
        {
            angle_offset = SingleTurnAngle;
            angle_is_init = true;
        }
        this->last_singleTurnAngle = SingleTurnAngle;
        return round_cnt * 360 + SingleTurnAngle - angle_offset;
    }
private:
    int32_t round_cnt = 0;
    bool angle_is_init = 0;
    float last_singleTurnAngle = 0, angle_offset = 0;
};

#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
