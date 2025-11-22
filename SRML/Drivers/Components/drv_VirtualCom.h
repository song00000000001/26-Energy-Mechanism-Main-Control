/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    drv_VirtualCom.h
  * @author  CuiQQ 860241578@qq.com
  * @brief   Code for VirtualCom driver in STM32 series MCU, supported packaged:
  *          - STM32Cube_FW_F4_V1.24.0.
  *          - STM32Cube_FW_F1_V1.8.0.
  *          - STM32Cube_FW_H7_V1.5.0.
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
#ifndef _DRV_VIRTUAL_COM_H
#define _DRV_VIRTUAL_COM_H
/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

#ifdef  __cplusplus
extern "C"{
#endif

/* Exported types ------------------------------------------------------------*/
typedef void (*VirtualComRecCallbackType)(uint8_t*, uint16_t);
typedef void (*VirtualComTransCallbackType)(uint8_t epnum);	

/* Exported function declarations ---------------------------------------------*/
void VirtualComUserInit(uint8_t* _RxBuf, VirtualComRecCallbackType _pVirtualComRecCpltCallback ,\
																				 VirtualComTransCallbackType _pVirtualComTrsmCpltCallback);
uint8_t VirtualComTransmitData(uint8_t* _TxBuf, uint32_t _Len);
	
#ifdef  __cplusplus
}
#endif

#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
