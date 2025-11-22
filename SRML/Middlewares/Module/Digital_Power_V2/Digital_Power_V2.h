/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file    Digital_Power_V2.h
 * @author  余俊晖 2460857175@QQ.COM
 * @brief   Header file
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
#include "srml_std_lib.h"

#ifdef __cplusplus
extern "C"
{
#endif
    /* Includes ------------------------------------------------------------------*/

    /* Private macros ------------------------------------------------------------*/

    /* Private type --------------------------------------------------------------*/

    /* Exported macros -----------------------------------------------------------*/

    /* Exported types ------------------------------------------------------------*/
#pragma pack(1)
    typedef struct __SendFromDigitalPowerPack_S
    {
        float pow_In;
        float pow_motor;
        float pow_Charge;
        uint16_t Vcap;
        uint16_t fcs;
    }SendFromDigitalPowerPack_S;
#pragma pack()

#pragma pack(1)
    typedef struct __SendToDigitalPowerPack_S
    {
        float PowerLimit;  // 功率限制
        float PowerBuffer; // 当前缓冲能量
        uint16_t HP;      // 当前血量
        uint16_t fcs;           // CRC16
    }SendToDigitalPowerPack_S;
#pragma pack()

    /* Exported function declarations --------------------------------------------*/
    uint8_t RecFromDigitalPower(SendFromDigitalPowerPack_S *ptr, const void *RecDataPtr, uint8_t RecLength);
#ifdef __cplusplus
}
#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/