/**
 ******************************************************************************
 * Copyright (c) 2023 - ~, SCUT-RobotLab Development Team
 * @file drv_memory.cpp
 * @author 余俊晖 (2460857175@qq.com)
 * @brief 利用FreeRTOS的heap4动态内存管理重写malloc、free、new、delete
 *        malloc与free的声明位于"stdlib.h"内，因此不用建一个头文件写函数声明
 *        new与delete不用写声明也会全局替换编译器原生的实现
 * 
 *        如果申请大小为size的内存空间，heap4自己本身需要额外使用8字节空间
 *        则需要的空间为size + 8，申请的数量越多，heap4使用的额外空间就越多，
 *        而且heap4的内存分配是以8的整数倍的对齐的，
 *        因此当(size + 8) % 8 != 0，实际heap占用会补齐到8的倍数
 *        比如申请大小为1的空间，1+8=9，则实际占用空间为16字节
 *        因此相比于碎片式的new和delete，将数据集中new一大段连续内存，其对于heap的利用效率会更高
 * @version 1.0
 * @date 2023-09-02
 *
 ******************************************************************************
 * @attention
 *
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version Number, write dowm your name and the date, the most
 * important is make sure the users will have clear and definite understanding
 * through your new brief.
 *
 * <h2><center>&copy; Copyright (c) 2023 - ~, SCUT-RobotLab Development Team.
 * All rights reserved.</center></h2>
 ******************************************************************************
 */
#include "FreeRTOS.h"
#include "stdlib.h"

void* std::malloc(size_t size)
{
    return pvPortMalloc(size);
}

void std::free(void* ptr)
{
    vPortFree(ptr);
}

void* operator new(size_t size)
{
  return pvPortMalloc(size);
}

void operator delete(void* ptr)
{
  vPortFree(ptr);
}

void* operator new[](size_t size) 
{
  return pvPortMalloc(size);
}

void operator delete[](void* ptr)
{
    vPortFree(ptr); 
}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
