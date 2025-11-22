/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file
 * @author
 * @brief   Code for FS-I6X driver in embedded software system.
 ******************************************************************************
 * @attention
 *
 * if you had modified this file, please make sure your code does not have any
 * bugs, update the version Number, write dowm your name and the date. The most
 * important thing is make sure the users will have clear and definite under-
 * standing through your new brief.
 *
 * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
 * All rights reserved.</center></h2>
 ******************************************************************************
 */
#ifndef _FS_I6X_H_
#define _FS_I6X_H_

#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/
#include "srml_std_lib.h"
#include "Drivers/Devices/Remote_Public.h"
/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class FS_I6X_Classdef
{
private:
  static const uint16_t LOST_THRESHOLD = 100;
  uint32_t last_recv_time;             /*<! 上一次在线检测时间*/
  LinkageStatus_Typedef Status;        /*<! 连接状态 */
  SBUS_DataPack_Typedef DataPack = {}; /*<! 数据包*/
  float RX_Norm, RY_Norm, LX_Norm, LY_Norm;
  float VRA_Norm, VRB_Norm;
  SW_Status_Typedef SWA, SWB, SWC, SWD;

  SW_Status_Typedef SW_Process(uint16_t ch_val);

public:
  float DeadZone = 0.05f; // 摇杆死区

  /* 数据接收、处理函数 */
  inline void DataCapture(void *addr) { memcpy(&DataPack, addr, 25); }
  void DataProcess();

  /* 连接状态相关操作 */
  void Check_Link(uint32_t current_check_time);
  LinkageStatus_Typedef GetStatus() { return Status; }

  /* 获取遥控信息 */
  float Get_RX_Norm(void) { return RX_Norm; } // 摇杆值（归一化）
  float Get_RY_Norm(void) { return RY_Norm; } // 摇杆值（归一化）
  float Get_LX_Norm(void) { return LX_Norm; } // 摇杆值（归一化）
  float Get_LY_Norm(void) { return LY_Norm; } // 摇杆值（归一化）
  float Get_VRA_Norm(void) { return VRA_Norm; } // 旋钮值（归一化）
  float Get_VRB_Norm(void) { return VRB_Norm; } // 旋钮值（归一化）
  SW_Status_Typedef Get_SWA(void) { return SWA; } // 拨杆状态
  SW_Status_Typedef Get_SWB(void) { return SWB; } // 拨杆状态
  SW_Status_Typedef Get_SWC(void) { return SWC; } // 拨杆状态
  SW_Status_Typedef Get_SWD(void) { return SWD; } // 拨杆状态
  const SBUS_DataPack_Typedef &Get_Raw_SBUS_Data() { return DataPack; } // SBUS原始数据

  
};
/* Exported function declarations --------------------------------------------*/

#endif // ! __cplusplus

#endif // ! _FS_I6X_H_
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
