/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    shit_mountain.cpp
  * @author  M3chD09 rinngo17@foxmail.com
  * @brief   Library full of shit.
  * @date    2021-04-05
  * @version 1.0
  *
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

/* Includes ------------------------------------------------------------------*/
#include "srml_std_lib.h"

using namespace std_lib;
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/

/**
  * @brief  Get the number of the specified input port pin
  * @param  GPIO_Pin specifies the port number to get.
  *         This parameter can be GPIO_PIN_x where x can be (0..15).
  * @retval The x of GPIO_PIN_x
  */
uint16_t std_lib::get_gpio_pin_num(uint16_t GPIO_Pin)
{
  uint16_t x = 0;
  while (GPIO_Pin >>= 1)x++;
  return x;
}

uint32_t std_lib::float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;    
    return (uint32_t) ((x-offset)*((float)((1<<bits)-1))/span);
}
/*整数转浮点数*/
float std_lib::uint_to_float(int x_int, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @brief 8位CRC校验
 * 
 * @param _data 数据指针
 * @param length 数据长度
 * @param polynomial 校验码
 * @return uint8_t 校验和
 */
uint8_t std_lib::CRC8(const void* _data, uint16_t length, uint8_t polynomial)
{
    uint8_t crc = 0xFF;
    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= ((uint8_t*)_data)[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x1)
            {
                crc = (crc >> 1) ^ polynomial;
            }
            else
            {
                crc = crc >> 1;
            }
        }
    }

    return crc;
}

/**
 * @brief 16位CRC校验
 * 
 * @param _data 数据指针
 * @param length 数据长度
 * @param polynomial 校验码
 * @return uint16_t 校验和
 */
uint16_t std_lib::CRC16(const void* _data, uint16_t length, uint16_t polynomial)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= ((uint8_t*)_data)[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x1)
            {
                crc = (crc >> 1) ^ polynomial;
            }
            else
            {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

/**
 * @brief 32位CRC校验
 * 
 * @param _data 数据指针
 * @param length 数据长度
 * @param polynomial 校验码
 * @return uint32_t 校验和
 */
uint32_t std_lib::CRC32(const void* _data, uint16_t length, uint32_t polynomial)
{
    uint32_t crc = 0xFFFFFFFF;

    for (uint16_t i = 0; i < length; ++i)
    {
        crc ^= ((uint8_t*)_data)[i];
        for (uint8_t j = 0; j < 8; ++j)
        {
            if (crc & 0x1)
            {
                crc = (crc >> 1) ^ polynomial;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/**
 * @brief 死区处理，常用于消除零点附近的微小误差
 * 
 * @param num 要处理的数
 * @param DZ_min 死区范围
 * @param DZ_max 死区范围
 * @param DZ_num 落在死区内时返回的值
 * @return float 处理后的结果
 */
float std_lib::DeadZone_Process(float num,float DZ_min, float DZ_max, float DZ_num)
{
    //若在死区内则返回死区值
    if(num > DZ_min && num < DZ_max)
    {
        return DZ_num;
    }
    else
        return num;
}
