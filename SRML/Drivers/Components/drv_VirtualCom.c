/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    drv_VirtualCom.c
  * @author  CuiQQ
  * @brief   
  * @date    2022-04-06
  * @version 1.0
  * @par Change Log：Null
  *
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
		@note:
			- #使用VirtualComUserInit()函数,传入接收回调,发送回调和接收存放地址来进行该模块的初始化.
			- #需要发送USB数据时，调用VirtualComTransmitData()函数进行发送，底层自动调用DMA.
			- #在初始化时设定的回调函数中可获取数据长度和数据包.
    
    @warning:
			- #使用此库前先确保工程中有CubeMX生成的USB底层库.
			- #USB底层的初始化并不是在此文件函数进行，而是在USB库对应代码中实现.
			- #目前仅支持USB通信中的虚拟串口(VirtualCOM)通信,其它通信如HID,DFU暂时需求不大,不作开发
			- #更多使用讲解信息请前往VirtualCOM范例仓库了解 https://git.scutbot.cn/Embedded/VirtualCom.git
			- #范例仓库中可查看CUBE MX配置，范例工程，还有 Readme.md 和 虚拟串口介绍.md 等详细教程
			
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
	
/* Includes ------------------------------------------------------------------*/
#include "srml_config.h"

#if USE_SRML_VIRTUAL_COM

/* Includes ------------------------------------------------------------------*/
#include "drv_VirtualCom.h"

/* Private function declarations ---------------------------------------------*/
static int8_t Rewrite_CDC_Init_HS(void);
static int8_t Rewrite_CDC_Receive_HS(uint8_t* Recv_Data, uint32_t* ReceiveLen);
static int8_t Rewrite_CDC_TransmitCplt_HS(uint8_t* Buf, uint32_t* Len, uint8_t epnum);

/* Extern variables ----------------------------------------------------------*/
extern USBD_HandleTypeDef hUsbDeviceHS;

/* Private variables ---------------------------------------------------------*/
VirtualComRecCallbackType pVirtualComRecCpltCallback;
VirtualComTransCallbackType pVirtualComTrsmCpltCallback;
uint8_t* pUserRxBuf;

/**
* @brief 初始化虚拟串口
* @note	 注册发送和接收的回调函数，
* @param RecCpltRegisterFun 接收完成回调函数，若不使用则写入NULL
* @param TransCpltRegisterFun 发送完成回调函数，若不使用则写入NULL
* @retval NULL
*/
void VirtualComUserInit(uint8_t* _RxBuf, VirtualComRecCallbackType _pVirtualComRecCpltCallback ,\
																				 VirtualComTransCallbackType _pVirtualComTrsmCpltCallback)
{
	
	pUserRxBuf = _RxBuf;
	USBD_Interface_fops_HS.Receive = Rewrite_CDC_Receive_HS;
	USBD_Interface_fops_HS.TransmitCplt = Rewrite_CDC_TransmitCplt_HS;
	USBD_Interface_fops_HS.Init = Rewrite_CDC_Init_HS;
	
	pVirtualComRecCpltCallback = _pVirtualComRecCpltCallback;
	pVirtualComTrsmCpltCallback = _pVirtualComTrsmCpltCallback;
}

/**
* @brief 重写CDC初始化完成后的接口函数,用于给USBD_Interface_fops_HS.Init赋值
* @retval NULL
*/
static int8_t Rewrite_CDC_Init_HS(void)
{
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, pUserRxBuf);
  return (USBD_OK);
}

/**
* @brief 重写CDC发送回调函数,并导向用户回调函数,用于给USBD_Interface_fops_HS.TransmitCplt赋值
* @param epnum USB端点号(如果不使用多个端点一般不用管)
* @retval NULL
*/
static int8_t Rewrite_CDC_TransmitCplt_HS(uint8_t* Buf, uint32_t* Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
		
	if(pVirtualComTrsmCpltCallback != NULL)
		pVirtualComTrsmCpltCallback(epnum);
	
	return result;
}
/**
* @brief 重写CDC发送回调函数,并导向用户回调函数,用于给USBD_Interface_fops_HS.TransmitCplt赋值,
* 			 虚拟串口接收到数据后将在USB库底层调用这个函数
* @param _Len 接收到的数据长度
* @param _Buf 接收到的数组
*/
static int8_t Rewrite_CDC_Receive_HS(uint8_t* Recv_Data, uint32_t* ReceiveLen)
{
	int8_t result;
	
	if(pVirtualComRecCpltCallback != NULL)
		pVirtualComRecCpltCallback(Recv_Data, *ReceiveLen);
	
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, pUserRxBuf);
  result = USBD_CDC_ReceivePacket(&hUsbDeviceHS);
	
	return result;
}

/**
* @brief 虚拟串口发送数据，Cube中打开了DMA将自动使用DMA发送
* @param _Len 发送数据的长度
* @param _Buf 待发送的数组
* @retval 是否发送成功，成功则返回 USBD_OK
*/
uint8_t VirtualComTransmitData(uint8_t* _TxBuf, uint32_t _Len)
{
  uint8_t result;
	
	if ((_TxBuf == NULL) || (_Len == 0U))
	{
		return USBD_FAIL;
	}
	
	USBD_CDC_HandleTypeDef* hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData;
	
  if (hcdc->TxState != 0)
	{
    return USBD_BUSY;
  }
	
	/* 设置发送数组 */
  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, _TxBuf, _Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceHS);
	
	return result;
}
#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
