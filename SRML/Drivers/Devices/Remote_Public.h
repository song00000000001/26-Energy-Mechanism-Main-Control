/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    Remote_Public.h
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

#ifdef __cplusplus
extern "C"{
#endif
/* Includes ------------------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
enum SW_Status_Typedef
{
  SW_NONE = 0,
  SW_UP = 1,
  SW_MID = 3,
  SW_DOWN = 2,
};

/**
  @brief 连接状态
*/
enum LinkageStatus_Typedef
{
  LOST = 0U,
  ESTABLISHED,
};

/**
  @brief SBUS数据包内容
*/
#pragma pack(1)
struct SBUS_DataPack_Typedef
{
  uint8_t HEAD;
  uint64_t ch1 : 11;
  uint64_t ch2 : 11;
  uint64_t ch3 : 11;
  uint64_t ch4 : 11;
  uint64_t ch5 : 11;
  uint64_t ch6 : 11;
  uint64_t ch7 : 11;
  uint64_t ch8 : 11;
  uint64_t ch9 : 11;
  uint64_t ch10 : 11;
  uint64_t ch11 : 11;
  uint64_t ch12 : 11;
  uint64_t ch13 : 11;
  uint64_t ch14 : 11;
  uint64_t ch15 : 11;
  uint64_t ch16 : 11;
  uint8_t END;
};
#pragma pack()

/* Exported function declarations --------------------------------------------*/

#ifdef  __cplusplus
}
#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
