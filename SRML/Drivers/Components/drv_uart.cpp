/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    drv_uart.c
  * @author  LWJ 851756890@qq.com
  * @brief   Code for UART driver in STM32 series MCU, supported packaged:
  *          - STM32Cube_FW_F4_V1.24.0.
  *          - STM32Cube_FW_F1_V1.8.0.
  *          - STM32Cube_FW_H7_V1.5.0.
  * @date    2019-06-12
  * @version 1.2
  * @par Change Log：
  * <table>
  * <tr><th>Date        <th>Version  <th>Author         <th>Description
  * <tr><td>2019-06-12  <td> 1.0     <td>charlie        <td>Creator
  * <tr><td>2019-10-28  <td> 1.1     <td>LWJ            <td>Remove the precompiled macro \n
  *                                                         Remove receive buffer \n
  *                                                         Add user specific buffer.
  * <tr><td>2019-11-11  <td> 1.2     <td>Mentos Seetoo  <td>Add callback regist to init fun. \n
  *                                                         Add transmit function.
  * <tr><td>2023-09-21  <td> 2.0     <td>余俊晖  		<td>增强隔离性，除了Uart_Init外均不使用HAL库串口句柄
  * 														在SRML_UART_Transmit_DMA中添加对于发送的地址不在SRAM中的相应机制
  * 														适配不使用DMA时的串口中断接收，不使用DMA时没有接收数据长度检测，认为接收数据长度 = 缓冲区长度
  * 														精简代码
  * </table>
  *
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================
    @note
      -# 调用`Uart_Init()`，传入串口对象句柄地址，缓冲数组首地址，数组长度，回调，中断类型 \n
	  	 或者调用Uart_Init_DMA()和Uart_Init_Normal()
         函数初始化串口。
      -# 如果在初始化的时候没有设置回调函数，在初始化后可单独用`Usart_Rx_Callback_Register()` \n
         设置串口接收中断回调函数的指针。
      -# 在`stm32f4xx_it.c`对应的串口中断里面加入`Uart_Receive_Handler()`，注意 \n
      -# 在需要用到发送的部分调用`SRML_UART_Transmit_DMA()`函数。
    
    @warning
      -# 用户需要自己定义缓存数组并初始化数组，数组类型为uint8_t。
      -# 添加预编译宏`USE_FULL_ASSERT`可以启用断言检查。
		
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

#include "srml_config.h"

#if USE_SRML_UART

/* Includes ------------------------------------------------------------------*/
#include "queue"
#include "string"
#include "Drivers/Components/drv_timer.h"
#include "Drivers/Components/drv_uart.h"

using std::queue;
/* Private define ------------------------------------------------------------*/
//为了统一使用UARTx，因此将USARTx定义为UARTx
#define UART1 USART1
#define UART2 USART2

#if	defined(USART10)
	#define UART3 USART3
	#define UART6 USART6
	#define UART10 USART10
#elif defined(USART6)
	#define UART3 USART3
	#define UART6 USART6
#elif defined(USART3)
	#define UART3 USART3
#endif

//将各个串口的Instance放到统一数组内，方便Uart_Init内使用for循环遍历
#if defined(UART10)
	#define UART_NUM 10
	__CCM static const USART_TypeDef* UART_Instances[UART_NUM] = {UART1, UART2, UART3, UART4, UART5, UART6, UART7, UART8, UART9, UART10};
#elif defined(UART9)
	#define UART_NUM 9
	__CCM static const USART_TypeDef* UART_Instances[UART_NUM] = {UART1, UART2, UART3, UART4, UART5, UART6, UART7, UART8, UART9};
#elif defined(UART8)
	#define UART_NUM 8
	__CCM static const USART_TypeDef* UART_Instances[UART_NUM] = {UART1, UART2, UART3, UART4, UART5, UART6, UART7, UART8};
#elif defined(UART7)
	#define UART_NUM 7
	__CCM static const USART_TypeDef* UART_Instances[UART_NUM] = {UART1, UART2, UART3, UART4, UART5, UART6, UART7};
#elif defined(UART6)
	#define UART_NUM 6
	__CCM static const USART_TypeDef* UART_Instances[UART_NUM] = {UART1, UART2, UART3, UART4, UART5, UART6};
#elif defined(UART5)
	#define UART_NUM 5
	__CCM static const USART_TypeDef* UART_Instances[UART_NUM] = {UART1, UART2, UART3, UART4, UART5};
#elif defined(UART4)
	#define UART_NUM 4
	__CCM static const USART_TypeDef* UART_Instances[UART_NUM] = {UART1, UART2, UART3, UART4};
#elif defined(UART3)
	#define UART_NUM 3
	__CCM static const USART_TypeDef* UART_Instances[UART_NUM] = {UART1, UART2, UART3};
#else
	#define UART_NUM 2
	__CCM static const USART_TypeDef* UART_Instances[UART_NUM] = {UART1, UART2};
#endif

#if  !SRML_UARTBUFF_MALLOC	//不使用malloc创建串口缓冲区时，在此创建静态数组
const uint16_t Uart_Buff_Len[10] = {	UART1_RX_BUFFER_SIZE,
										UART2_RX_BUFFER_SIZE, 
										UART3_RX_BUFFER_SIZE, 
										UART4_RX_BUFFER_SIZE, 
										UART5_RX_BUFFER_SIZE,
										UART6_RX_BUFFER_SIZE,
										UART7_RX_BUFFER_SIZE,
										UART8_RX_BUFFER_SIZE,
										UART9_RX_BUFFER_SIZE,
										UART10_RX_BUFFER_SIZE};


__SRAM uint8_t Uart1_Rx_Buff[UART1_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart1 */
__SRAM uint8_t Uart2_Rx_Buff[UART2_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart2 */

#if defined(UART3)
	__SRAM uint8_t Uart3_Rx_Buff[UART3_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart3 */
#endif

#if defined(UART4)
	__SRAM uint8_t Uart4_Rx_Buff[UART4_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart4 */
#endif

#if defined(UART5)
	__SRAM uint8_t Uart5_Rx_Buff[UART5_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart5 */
#endif

#if defined(UART6)
	__SRAM uint8_t Uart6_Rx_Buff[UART6_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart6 */
#endif

#if defined(UART7)
	__SRAM uint8_t Uart7_Rx_Buff[UART7_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart7 */
#endif

#if defined(UART8)
	__SRAM uint8_t Uart8_Rx_Buff[UART8_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart8 */
#endif

#if defined(UART9)
	__SRAM uint8_t Uart9_Rx_Buff[UART9_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart9 */
#endif

#if defined(UART10)
	__SRAM uint8_t Uart10_Rx_Buff[UART10_RX_BUFFER_SIZE];     /*!< Receive buffer for Uart10 */
#endif

//将各个串口的缓冲区地址放到统一数组内，方便Uart_Init内使用for循环遍历
#if defined(UART10)
	__CCM uint8_t *Uart_Buff_Addr[UART_NUM] = {Uart1_Rx_Buff, Uart2_Rx_Buff, Uart3_Rx_Buff, Uart4_Rx_Buff, Uart5_Rx_Buff, Uart6_Rx_Buff, Uart7_Rx_Buff, Uart8_Rx_Buff, Uart9_Rx_Buff, Uart10_Rx_Buff};
#elif defined(UART9)
	__CCM uint8_t *Uart_Buff_Addr[UART_NUM] = {Uart1_Rx_Buff, Uart2_Rx_Buff, Uart3_Rx_Buff, Uart4_Rx_Buff, Uart5_Rx_Buff, Uart6_Rx_Buff, Uart7_Rx_Buff, Uart8_Rx_Buff, Uart9_Rx_Buff};
#elif defined(UART8)
	__CCM uint8_t *Uart_Buff_Addr[UART_NUM] = {Uart1_Rx_Buff, Uart2_Rx_Buff, Uart3_Rx_Buff, Uart4_Rx_Buff, Uart5_Rx_Buff, Uart6_Rx_Buff, Uart7_Rx_Buff, Uart8_Rx_Buff};
#elif defined(UART7)
	__CCM uint8_t *Uart_Buff_Addr[UART_NUM] = {Uart1_Rx_Buff, Uart2_Rx_Buff, Uart3_Rx_Buff, Uart4_Rx_Buff, Uart5_Rx_Buff, Uart6_Rx_Buff, Uart7_Rx_Buff};
#elif defined(UART6)
	__CCM uint8_t *Uart_Buff_Addr[UART_NUM] = {Uart1_Rx_Buff, Uart2_Rx_Buff, Uart3_Rx_Buff, Uart4_Rx_Buff, Uart5_Rx_Buff, Uart6_Rx_Buff};
#elif defined(UART5)
	__CCM uint8_t *Uart_Buff_Addr[UART_NUM] = {Uart1_Rx_Buff, Uart2_Rx_Buff, Uart3_Rx_Buff, Uart4_Rx_Buff, Uart5_Rx_Buff};
#elif defined(UART4)
	__CCM uint8_t *Uart_Buff_Addr[UART_NUM] = {Uart1_Rx_Buff, Uart2_Rx_Buff, Uart3_Rx_Buff, Uart4_Rx_Buff};
#elif defined(UART3)
	__CCM uint8_t *Uart_Buff_Addr[UART_NUM] = {Uart1_Rx_Buff, Uart2_Rx_Buff, Uart3_Rx_Buff};
#else
	__CCM uint8_t *Uart_Buff_Addr[UART_NUM] = {Uart1_Rx_Buff, Uart2_Rx_Buff};
#endif

#endif /* !SRML_UARTBUFF_MALLOC */

/* Private type --------------------------------------------------------------*/
/**
 * @brief Contain uart control info.
 */
struct usart_manage_obj_t
{
	UART_HandleTypeDef *uart_h = nullptr;
	uint16_t rx_buffer_size = 0;
	uint8_t *rx_buffer = nullptr;
	User_Uart_Callback call_back_f = nullptr;
	UART_Receive_Type receive_type = UART_RECV_DMA;
} ;

/* Private variables ---------------------------------------------------------*/
__CCM usart_manage_obj_t usart_manage_obj[UART_NUM] = {};

/* Private function declarations ---------------------------------------------*/
static void Uart_IDLE_RxCallback(usart_manage_obj_t *m_obj);
static inline void Error_Handler(void);

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 串口初始化
 * 
 * @param huart 串口句柄
 * @param buff_addr 缓冲区地址，若为NULL，将会使用串口库的动态/静态缓冲区
 * @param length 缓冲区长度，若已使用“Uart_Set_Receive_Buffer”制定了缓冲区，则该参数无效
 * @param fun 外部回调函数
 * @param receive_type 中断类型，DMA接收中断or普通接收中断
 */
void Uart_Init(UART_HandleTypeDef *huart, uint8_t* buff_addr, uint32_t length, User_Uart_Callback fun, UART_Receive_Type receive_type)
{
	if (huart == NULL)
	{
		Error_Handler();
		return;
	}
	else {}

	for (int i = 0; i < UART_NUM; i++)
	{
		if (huart->Instance == UART_Instances[i])
		{
			if(buff_addr == NULL)
			{
			#if SRML_UARTBUFF_MALLOC
				usart_manage_obj[i].rx_buffer = (uint8_t*)malloc(length);
				usart_manage_obj[i].rx_buffer_size = length;
			#else
				usart_manage_obj[i].rx_buffer = Uart_Buff_Addr[i];
				usart_manage_obj[i].rx_buffer_size = Uart_Buff_Len[i];
			#endif
			}
			else
			{
				usart_manage_obj[i].rx_buffer = buff_addr;
				usart_manage_obj[i].rx_buffer_size = length;
			}
			
			usart_manage_obj[i].uart_h = huart;
			usart_manage_obj[i].call_back_f = fun;
			usart_manage_obj[i].receive_type = receive_type;

			__HAL_UART_CLEAR_IDLEFLAG(huart);
			__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

			if(receive_type == UART_RECV_DMA)
			{
				HAL_UART_Receive_DMA(huart, usart_manage_obj[i].rx_buffer, usart_manage_obj[i].rx_buffer_size);
			}
			else
				HAL_UART_Receive_IT(huart, usart_manage_obj[i].rx_buffer, usart_manage_obj[i].rx_buffer_size);
			return;
		}
	}
	
	Error_Handler();
	return;
}

/**
 * @brief 设置串口缓冲区
 * 
 * @param usart_index 串口号
 * @param buf_addr 缓冲区地址
 * @param len 缓冲区长度
 */
void Uart_Set_Receive_Buffer(uint8_t usart_index, void* buf_addr, uint16_t len)
{
	usart_manage_obj[usart_index - 1].rx_buffer = (uint8_t *)buf_addr;
	usart_manage_obj[usart_index - 1].rx_buffer_size = len;
		
	HAL_UART_AbortReceive(usart_manage_obj[usart_index - 1].uart_h);
	HAL_UART_Receive_DMA(usart_manage_obj[usart_index - 1].uart_h, usart_manage_obj[usart_index - 1].rx_buffer, len);
}

/**
 * @brief   重定向用户回调函数
 * @param   m_obj: serial port handle
 * @param   fun: user callback function
 * @retval  None
 */
void Usart_Rx_Callback_Register(uint8_t usart_index, User_Uart_Callback fun)
{
  /* Check the parameters */
	assert_param(fun != NULL);
	assert_param(usart_index <= UART_NUM && usart_index >= 0);

	usart_manage_obj[usart_index - 1].call_back_f = fun;
	return;
}

/**
 * @brief   串口接收中断处理函数
 * @param   usart_index: 触发中断的串口号，比如串口1则传参数“1”
 * @retval  None
 */
void Uart_Receive_Handler(uint8_t usart_index)
{
	usart_manage_obj_t *m_obj = &usart_manage_obj[usart_index - 1];
	if(m_obj->uart_h == NULL)
		Error_Handler();
	if(__HAL_UART_GET_FLAG(m_obj->uart_h, UART_FLAG_IDLE) != RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(m_obj->uart_h);
		Uart_IDLE_RxCallback(m_obj);
	}
}

__CCM static uint32_t DMA_IFCR_TOTAL_FLAGs[4] = {
	(DMA_FLAG_TCIF0_4 | DMA_FLAG_HTIF0_4 | DMA_FLAG_DMEIF0_4 | DMA_FLAG_FEIF0_4 | DMA_FLAG_TEIF0_4),
	(DMA_FLAG_TCIF1_5 | DMA_FLAG_HTIF1_5 | DMA_FLAG_DMEIF1_5 | DMA_FLAG_FEIF1_5 | DMA_FLAG_TEIF1_5),
	(DMA_FLAG_TCIF2_6 | DMA_FLAG_HTIF2_6 | DMA_FLAG_DMEIF2_6 | DMA_FLAG_FEIF2_6 | DMA_FLAG_TEIF2_6),
	(DMA_FLAG_TCIF3_7 | DMA_FLAG_HTIF3_7 | DMA_FLAG_DMEIF3_7 | DMA_FLAG_FEIF3_7 | DMA_FLAG_TEIF3_7)};

__CCM static const DMA_Stream_TypeDef* DMA_Streams[4][4] = {
	{DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3},
	{DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7},
	{DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3},
	{DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7}};

__CCM static volatile uint32_t *DMA_IFCRs[4] = {&(DMA1->LIFCR), &(DMA1->HIFCR), &(DMA2->LIFCR), &(DMA2->HIFCR)};

inline void ENSURE_DMA_ENABLE(DMA_HandleTypeDef* hdma)
{
	uint8_t i, j;
	for (i = 0; i < 4; i++)
	{
		if (hdma->Instance <= DMA_Streams[i][3])
		{
			for (j = 0; j < 4; j++)
			{
				if (hdma->Instance == DMA_Streams[i][j])
				{
					break;
				}
			}
			break;
		}
	}
	*DMA_IFCRs[i] = DMA_IFCR_TOTAL_FLAGs[j]; // 清除DMA各种中断标志位
	__HAL_DMA_ENABLE(hdma);
}

/**
 * @brief   串口空闲中断处理函数
 * @note    call in uart_receive_handler() function
 * @param   uart IRQHandler id
 * @retval  None
 */
static void Uart_IDLE_RxCallback(usart_manage_obj_t *m_obj)
{
  /* Check the parameters */
	assert_param(m_obj != NULL);
	
  /* Private variables */
	static uint16_t usart_rx_num;

  /* clear DMA transfer complete flag */
  	if(m_obj->receive_type == UART_RECV_DMA)
	{	
		HAL_UART_AbortReceive(m_obj->uart_h);
		usart_rx_num = m_obj->rx_buffer_size - __HAL_DMA_GET_COUNTER(m_obj->uart_h->hdmarx);
	}
	else
	{
		usart_rx_num = m_obj->rx_buffer_size;
	};
	/* handle received data in idle interrupt */
	if (m_obj->call_back_f != NULL)
	{
	#ifdef USE_CACHE
		// 将Cache内数据无效化，若SCB_InvalidateDCache执行频繁的时候，会进硬件中断，原因请前往飞书嵌入式软件知识库
		SCB_CleanInvalidateDCache_by_Addr((uint32_t*)m_obj->rx_buffer, usart_rx_num); 
	#endif
		m_obj->call_back_f(m_obj->rx_buffer, usart_rx_num);
	}
	if(m_obj->receive_type == UART_RECV_DMA)
	{
		HAL_UART_Receive_DMA(m_obj->uart_h, m_obj->rx_buffer, m_obj->rx_buffer_size);
		if((m_obj->uart_h->hdmarx->Instance->CR & DMA_SxCR_EN) == 0)
			ENSURE_DMA_ENABLE(m_obj->uart_h->hdmarx);
	}
	else
		HAL_UART_Receive_IT(m_obj->uart_h, m_obj->rx_buffer, m_obj->rx_buffer_size);
}

HAL_StatusTypeDef SRML_UART_Transmit(uint8_t port_num, const uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	if(usart_manage_obj[port_num - 1].uart_h == nullptr)
		Error_Handler();
	return SRML_UART_Transmit(usart_manage_obj[port_num - 1].uart_h, pData, Size, Timeout);
}
HAL_StatusTypeDef SRML_UART_Transmit_IT(uint8_t port_num, const uint8_t *pData, uint16_t Size)
{
	if(usart_manage_obj[port_num - 1].uart_h == nullptr)
		Error_Handler();
	return SRML_UART_Transmit_IT(usart_manage_obj[port_num - 1].uart_h, pData, Size);
}
HAL_StatusTypeDef SRML_UART_Transmit_DMA(uint8_t port_num, const uint8_t *pData, uint16_t Size)
{
	if(usart_manage_obj[port_num - 1].uart_h == nullptr)
		Error_Handler();
	return SRML_UART_Transmit_DMA(usart_manage_obj[port_num - 1].uart_h, pData, Size);
}


HAL_StatusTypeDef SRML_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	return HAL_UART_Transmit(huart, pData, Size, Timeout);
}

HAL_StatusTypeDef SRML_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
{
	return HAL_UART_Transmit_IT(huart, pData, Size);
}

__CCM static void *buffer[UART_NUM] = {};
__CCM static uint16_t buffer_size[UART_NUM] = {};

/**
 * @brief SRML串口DMA发送函数，当发送的数据不在SRAM中时，有相应机制确保其可以正常发送
 * 
 * @param huart 串口句柄
 * @param pData 数据地址
 * @param Size 数据长度
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef SRML_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
{
	HAL_StatusTypeDef return_ans;
	if (__IS_ADDR_AT_SRAM(pData)) // 检测数据是否在SRAM中
	{
#ifdef USE_CACHE
		SCB_CleanDCache_by_Addr((uint32_t *)pData, Size); // 将Cache数据同步到RAM中
#endif
		return_ans = HAL_UART_Transmit_DMA(huart, pData, Size);
	}
	else
	{
		if (huart->gState != HAL_UART_STATE_READY) // 检测串口是否发送完毕
		{
			return HAL_BUSY;
		}

		uint8_t port_num;
		// 判断串口号
		for (port_num = 0; port_num < UART_NUM; port_num++)
		{
			if (huart->Instance == UART_Instances[port_num])
			{
				break;
			}
		}
		// 若当前需要发送的长度 < 缓冲区长度，且二者长度差小于一定值（即空间浪费小于一定值），则不需要free和重新malloc
		if(Size > buffer_size[port_num] || (buffer_size[port_num] - Size) > 64)
		{
			// 进入这一步，说明上一次已经发完，可以free了
			if (buffer[port_num] != nullptr) 
			{
				free(buffer[port_num]);
				buffer[port_num] = nullptr;
			}
			buffer[port_num] = malloc(Size);
			buffer_size[port_num] = Size;
		}
		memcpy(buffer[port_num], pData, Size); // 复制DTCM数据到SRAM中

#ifdef USE_CACHE
		SCB_CleanDCache_by_Addr((uint32_t *)buffer[port_num], Size);
#endif
		return_ans = HAL_UART_Transmit_DMA(huart, (const uint8_t *)buffer[port_num], Size);
	}
	return return_ans;
}
  
/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler(void)
{
  /* Normally the program would never run here. */
  while(1){}
}

#endif /* USE_SRML_UART */

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
