/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file    UpperMonitor.c
 * @author  LiangHong Lin(林亮洪)
 * @brief   Code for Upper monitor supported by Mr.Lin in STM32F4.
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
#ifndef _UPPER_MONITOR_H_
#define _UPPER_MONITOR_H_
#include "srml_std_lib.h"

#ifdef __cplusplus
#include <typeinfo>

namespace UpperMonitor
{
  struct TypedPointer_t
  {
    const std::type_info *type = nullptr;
    void *ptr = nullptr;
  };

  union Type_change_t // 数据传输共用体结构
  {
    uint8_t change_u8[4]; // 8位无符号整型【1个字节】
    float change_float;   // 32位浮点型数据【4个字节】
  };

  const uint8_t Max_Data_Num = 9;
  const uint8_t Max_Receive_ID = 0x14;

  extern TypedPointer_t TypedPointer[];
  extern Type_change_t SendDataPool[];

  void init(uint8_t uast_id);
  void send(uint8_t uast_id);

  template <typename T>
  void bind_Modified_Var(const uint8_t index, T *_ptr);

  template <typename T>
  void setDatas(uint8_t firstindex, T data);

  template <typename T, typename... Ts>
  void setDatas(uint8_t firstindex, T data, Ts... dataArgs);
}

/**
 * @brief 将序号与使用上位机修改的变量绑定
 *
 * @tparam T 指针类型
 * @param index 序号
 * @param _ptr 变量地址
 */
template <typename T>
void UpperMonitor::bind_Modified_Var(const uint8_t index, T *_ptr)
{
  if (index < 0 || index > Max_Receive_ID)
    return;

  TypedPointer[index].type = &typeid(_ptr);
  TypedPointer[index].ptr = _ptr;
}

/**
 * @brief
 * @warning firstindex最小可以从0开始
 * @tparam T  可变类型，传入变量的类型是可变的，可以传入多个不同类型的变量
 * @param firstindex 传入的数据中，第一个数据放在第几条曲线
 * @param dataArg 需要发送的数据，为可变参数，可以传入的参数数量是不固定
 */
template <typename T>
void UpperMonitor::setDatas(uint8_t firstindex, T data)
{
  if (firstindex < 0 || firstindex >= Max_Data_Num)
    return;
  SendDataPool[1 + firstindex].change_float = data;
}

template <typename T, typename... Ts>
void UpperMonitor::setDatas(uint8_t firstindex, T data, Ts... dataArgs)
{
  static_assert(sizeof...(dataArgs) < Max_Data_Num, "The sent variable exceeded Max_Data_Num of UpperMonitor");

  setDatas(firstindex, data);
  firstindex++;
  setDatas(firstindex, dataArgs...);
}
#endif /* __cplusplus */

#endif /* _UPPER_MONITOR_H_ */
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
