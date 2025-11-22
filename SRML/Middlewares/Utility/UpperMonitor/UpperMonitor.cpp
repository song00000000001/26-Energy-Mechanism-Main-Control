/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    UpperMonitor.c
  * @author  LiangHong Lin(林亮洪)
  * @brief   Code for Upper monitor supported by Mr.Lin in STM32F4.
  * @date    Unkown.
  * @version 1.0
  * @par Change Log：
  * <table
  * <tr><th>Date        <th>Version  <th>Author    		  <th>Description
  * <tr><td>2019-06-12  <td> 1.0     <td>LiangHong Lin  <td>Creator
  * </table>2019-11-06  <td> 1.1     <td>Mentos Seetoo  <td>Add return valie for
  *                                                         `RecHandle()`
  * </table>2023-10-10  <td> 2.0 	 <td>yjh 			<td>Updater
  * </table>2023-10-17  <td> 2.1 	 <td>lrc 			<td>Modify the instructions for use
  ==============================================================================
					  ##### How to use this driver #####
  ==============================================================================
	@note
	  -# 配置好串口与相应DMA
      -# 调用UpperMonitor::setDatas(uint8_t firstindex, T... dataArg),传入起始通道编号(0~9)和需要传输的数据
      -# 调用UpperMonitor::send(uint8_t uast_id)发送数据，传参为串口id号
	  -# 调用UpperMonitor::bind_Modified_Var(const uint8_t index, T *_ptr)将序号与使用上位机修改的变量绑定
	@warning
	  -# 

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
#if USE_SRML_UPPER_MONITOR

#include "UpperMonitor.h"
#include "Drivers/Components/drv_uart.h"
/* Private define ------------------------------------------------------------*/
using namespace UpperMonitor;
#warning("Please pay attention to the baud rate of the serial port (UART/USART) to avoid data loss. ")
/* Private type --------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
namespace UpperMonitor
{
	__SRAM Type_change_t SendDataPool[Max_Data_Num + 2]; // 传输数据共用体
	__CCM TypedPointer_t TypedPointer[20];
}
/* Private function declarations ---------------------------------------------*/
static float PARAMETER_Change_float(uint8_t *PARAMETER);
static void PARAMETER_MODIFICATION(uint8_t *PARAMETER);
static uint32_t RecHandle(uint8_t *data_buf, uint16_t length);
/* function prototypes -------------------------------------------------------*/
/**
 * @brief 接收初始换函数
 * 
 * @param uast_id 接收串口ID
 */
void UpperMonitor::init(uint8_t uast_id){
	Usart_Rx_Callback_Register(uast_id, RecHandle);
}

/**
 * @brief 发送数据函数
 * 
 * @param uast_id 接收串口ID
 */
void UpperMonitor::send(uint8_t uast_id)
{
	__CCM static USART_COB Usart_TxCOB = {1, sizeof(SendDataPool) - 5, (uint8_t *)SendDataPool + 3};
	Usart_TxCOB.port_num = uast_id;

	SendDataPool[0].change_u8[3] = 0xfd;						// 发送数据头
	SendDataPool[Max_Data_Num + 1].change_u8[0] = Max_Data_Num; // 数据尾
	SendDataPool[Max_Data_Num + 1].change_u8[1] = 0xfe;			// 校验位

	SRML_UART_Transmit_DMA(&Usart_TxCOB);
}

/**
* @brief  串口接收解析函数
* @param  data_buf：接收到的数据指针
		  length  ：数据长度
* @return No meaning.
*/
static uint32_t RecHandle(uint8_t *data_buf, uint16_t length)
{
	__CCM static uint8_t USART_receive[5] = {0};	  // 串口接收缓存数组
	__CCM static uint8_t USART_Interrupt_flag = 0xff; // 串口中断标志位

	for (int i = 0; i < length; i++)
	{
		switch (USART_Interrupt_flag)
		{
		case 0xff:							 // USART0_Interrupt_flag==0xff时为等待模式，等待指令头输入
			if (data_buf[i] == 0xf0)		 // 指令头，识别上位机发送了修改指令
				USART_Interrupt_flag = 0xf0; // 下一个指令将进入模式选择模式
			break;
		case 0xf0:					 // 进入模式选择
			if (data_buf[i] == 0x00) // 修改参数
			{
				USART_Interrupt_flag = 0x00; // 进入参数修改模式
			}
			break;

		case 0x00: // 参数修改模式
			memcpy(USART_receive, &data_buf[i], 5);
			PARAMETER_MODIFICATION(USART_receive);
			USART_Interrupt_flag = 0xff; // 回到等待模式
			i += 4;
			break;

		default:
			USART_Interrupt_flag = 0xff; // 回到等待模式
			break;
		}
	}
	return 0;
}

/**
 * @brief  上位机参数转变成浮点数函数
 * @param  PARAMETER：指令数组指针，用于读取指令
 * @return None.
 */
static float PARAMETER_Change_float(uint8_t *PARAMETER)
{
	union Type_change_t Sent_data_temp; // 传输数据共用体
	// 转换成共用体数据类型
	Sent_data_temp.change_u8[0] = PARAMETER[3];
	Sent_data_temp.change_u8[1] = PARAMETER[2];
	Sent_data_temp.change_u8[2] = PARAMETER[1];
	Sent_data_temp.change_u8[3] = PARAMETER[0];

	return Sent_data_temp.change_float; // 返回共用体转化后的数据
}

/**
 * @brief  上位机参数修改函数（要调的参数）
 * @param  PARAMETER：指令数组指针，用于读取指令
 * @return None.
 */
static void PARAMETER_MODIFICATION(uint8_t *PARAMETER)
{
	if (PARAMETER[0] < 0 || PARAMETER[0] > Max_Receive_ID)
		return;

	auto *_type = TypedPointer[PARAMETER[0]].type;
	void *_ptr = TypedPointer[PARAMETER[0]].ptr;

	if(_type == nullptr || _ptr == nullptr)
		return;

	float value = PARAMETER_Change_float(PARAMETER + 1);

	if (*_type == typeid(bool *))
	{
		using t = bool;
		*(t *)_ptr = (t)value;
	}
	else if (*_type == typeid(int8_t *))
	{
		using t = int8_t;
		*(t *)_ptr = (t)value;
	}
	else if (*_type == typeid(uint8_t *))
	{
		using t = uint8_t;
		*(t *)_ptr = (t)value;
	}
	else if (*_type == typeid(int16_t *))
	{
		using t = int16_t;
		*(t *)_ptr = (t)value;
	}
	else if (*_type == typeid(uint16_t *))
	{
		using t = uint16_t;
		*(t *)_ptr = (t)value;
	}
	else if (*_type == typeid(int32_t *))
	{
		using t = int32_t;
		*(t *)_ptr = (t)value;
	}
	else if (*_type == typeid(uint32_t *))
	{
		using t = uint32_t;
		*(t *)_ptr = (t)value;
	}
	else if (*_type == typeid(int64_t *))
	{
		using t = int64_t;
		*(t *)_ptr = (t)value;
	}
	else if (*_type == typeid(uint64_t *))
	{
		using t = uint64_t;
		*(t *)_ptr = (t)value;
	}
	else if (*_type == typeid(float *))
	{
		using t = float;
		*(t *)_ptr = (t)value;
	}
	else if (*_type == typeid(double *))
	{
		using t = double;
		*(t *)_ptr = (t)value;
	}
	else
		return;
}
#endif /* USE_SRML_UPEER_MONITOR */
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
