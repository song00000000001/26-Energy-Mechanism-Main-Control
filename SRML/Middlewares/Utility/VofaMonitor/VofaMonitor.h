/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file    VofaMonitor.h
 * @author  ZengXilang chenmoshaoalen@126.com
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
#include "srml_config.h"
#include "srml_std_lib.h"

#ifndef VOFAMONITOR_MAX_DATA_NUM
#define VOFAMONITOR_MAX_DATA_NUM 10
#endif

#ifdef __cplusplus

namespace VofaMonitor
{
	union Type_change_t
	{
		uint8_t change_u8[4]; // 8位无符号整型【1个字节】
		float change_float;	  // 32位浮点型数据【4个字节】
	};
	const uint8_t Max_Data_Num = VOFAMONITOR_MAX_DATA_NUM;
	extern Type_change_t SendDataPool[]; // 传输数据池
	extern uint8_t DataNum;			   // 记录当前需要传输的数据量

	void send(uint8_t uart_id);

	/**
	 * @brief
	 * @warning firstindex最小可以从0开始
	 * @tparam T
	 * @param firstindex 传入的数据中，第一个数据放在第几条曲线
	 * @param dataArg 需要发送的数据
	 */
	template <typename T>
	void setDatas(uint8_t index, T data)
	{
		if (index >= Max_Data_Num)
			return;
		SendDataPool[index].change_float = data;
		DataNum = index > DataNum ? index : DataNum;
	}

	template <typename T, typename... Ts>
	void setDatas(uint8_t firstindex, T data, Ts... dataArgs)
	{
		static_assert(sizeof...(dataArgs) < Max_Data_Num, "The sent variable exceeded Max_Data_Num of VofaMonitor");

		setDatas(firstindex, data);
		firstindex++;
		setDatas(firstindex, dataArgs...);
	}
}

#endif /* __cplusplus */

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
