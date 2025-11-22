/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file    drv_uart.h
 * @author  LWJ 851756890@qq.com
 * @brief   Code for UART driver in STM32 series MCU, supported packaged:
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
#ifndef __DRV_UART_H__
#define __DRV_UART_H__

/* Includes ------------------------------------------------------------------*/
#include "srml_config.h"
#include "srml_std_lib.h"

#ifndef SRML_UARTBUFF_MALLOC
  #define SRML_UARTBUFF_MALLOC 1
#endif

#ifdef __cplusplus
extern "C"
{
#endif
  /* Private macros ------------------------------------------------------------*/
  /* Private type --------------------------------------------------------------*/
  /**
   * @brief 用户串口回调函数类型
   */
  typedef uint32_t (*User_Uart_Callback)(uint8_t *buf, uint16_t len);

  /**
   * @brief 串口接收类型enum，使用DMA or 不用DMA（比如DMA资源不够的情况）
   */
  typedef enum 
  {
    UART_RECV_DMA = 0,
    UART_RECV_IT
  }UART_Receive_Type;
  
  /**
   * @brief USART message data type (Communication Object).
   */
  typedef struct
  {
    uint8_t port_num;
    uint16_t len;
    uint8_t *address;
  } USART_COB;

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
HAL_StatusTypeDef SRML_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef SRML_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef SRML_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size);

#ifndef __cplusplus
  void Uart_Init(UART_HandleTypeDef *huart, uint8_t* buff_addr, uint32_t length, User_Uart_Callback fun, UART_Receive_Type receive_type);
  #if SRML_UARTBUFF_MALLOC
    #define Uart_Init_DMA(huart, buf, length, fun) Uart_Init(huart, buf, length, fun, UART_RECV_DMA);
    #define Uart_Init_Normal_IT(huart, buf, length, fun) Uart_Init(huart, buf, length, fun, UART_RECV_IT);
  #else
    #define Uart_Init_DMA(huart, fun) Uart_Init(huart, NULL, 0, fun, UART_RECV_DMA);
    #define Uart_Init_Normal_IT(huart, fun) Uart_Init(huart, NULL, 0, fun, UART_RECV_IT);
  #endif
#else /* __cplusplus */
  void Uart_Init(UART_HandleTypeDef *huart, uint8_t* buff_addr, uint32_t length, User_Uart_Callback fun, UART_Receive_Type receive_type = UART_RECV_DMA);
#endif /* __cplusplus */

/**
 * @brief 设置串口缓冲区
 * 
 * @param usart_index 串口号
 * @param buf_addr 缓冲区地址
 * @param len 缓冲区长度
 */
  void Uart_Set_Receive_Buffer(uint8_t usart_index, void* buf_addr, uint16_t len);

  /**
   * @brief 用户回调函数重定向API
   *
   * @param usart_index 串口ID号
   * @param fun 需要重定向的函数指针
   */
  void Usart_Rx_Callback_Register(uint8_t usart_index, User_Uart_Callback fun);

/**
 * @brief 串口接收处理函数，需要手动添加到串口的IRQ_Hander中
 * 
 * @param usart_index 触发中断的串口ID号
 */
  void Uart_Receive_Handler(uint8_t usart_index);


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
  inline void Uart_Init(UART_HandleTypeDef *huart, uint32_t length, User_Uart_Callback fun, UART_Receive_Type receive_type = UART_RECV_DMA)
  {
    Uart_Init(huart, NULL, length, fun, receive_type);
  }
  
  #if SRML_UARTBUFF_MALLOC
    inline void Uart_Init_DMA(UART_HandleTypeDef *huart, uint32_t length, User_Uart_Callback fun)
    {
      Uart_Init(huart, NULL, length, fun, UART_RECV_DMA);
    }
    inline void Uart_Init_Normal_IT(UART_HandleTypeDef *huart, uint32_t length, User_Uart_Callback fun)
    {
      Uart_Init(huart, NULL, length, fun, UART_RECV_IT);
    }
  #else /* SRML_UARTBUFF_MALLOC */
    inline void Uart_Init_DMA(UART_HandleTypeDef *huart, User_Uart_Callback fun)
    {
      Uart_Init(huart, NULL, 0, fun, UART_RECV_DMA);
    }
    inline void Uart_Init_Normal_IT(UART_HandleTypeDef *huart, User_Uart_Callback fun)
    {
      Uart_Init(huart, NULL, 0, fun, UART_RECV_IT);
    }
  #endif /* SRML_UARTBUFF_MALLOC */

  inline void Uart_Init_DMA(UART_HandleTypeDef *huart, uint8_t* buff_addr, uint32_t length, User_Uart_Callback fun)
  {
    Uart_Init(huart, buff_addr, length, fun, UART_RECV_DMA);
  }

  inline void Uart_Init_Normal_IT(UART_HandleTypeDef *huart, uint8_t* buff_addr, uint32_t length, User_Uart_Callback fun)
  {
    Uart_Init(huart, buff_addr, length, fun, UART_RECV_IT);
  }

  HAL_StatusTypeDef SRML_UART_Transmit(uint8_t port_num, const uint8_t *pData, uint16_t Size, uint32_t Timeout);
  HAL_StatusTypeDef SRML_UART_Transmit_IT(uint8_t port_num, const uint8_t *pData, uint16_t Size);
  HAL_StatusTypeDef SRML_UART_Transmit_DMA(uint8_t port_num, const uint8_t *pData, uint16_t Size);

  inline HAL_StatusTypeDef SRML_UART_Transmit(USART_COB *TxPack, uint32_t Timeout)
  {
    return SRML_UART_Transmit(TxPack->port_num, TxPack->address, TxPack->len, Timeout);
  }
  inline HAL_StatusTypeDef SRML_UART_Transmit_IT(USART_COB *TxPack)
  {
    return SRML_UART_Transmit_IT(TxPack->port_num, TxPack->address, TxPack->len);
  }
  inline HAL_StatusTypeDef SRML_UART_Transmit_DMA(USART_COB *TxPack)
  {
    return SRML_UART_Transmit_DMA(TxPack->port_num, TxPack->address, TxPack->len);
  }
#endif /* __cplusplus */

#endif /* __DRV_UART_H__ */

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
