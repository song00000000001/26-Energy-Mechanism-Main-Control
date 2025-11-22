/**
 ******************************************************************************
 * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
 * @file    LPMS_BE2.h
 * @author  ZhiRui Zhang 2231625449@qq.com
 * @brief   Header file of LPMS-BE2.
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
#ifndef _LPMS_BE2_H_
#define _LPMS_BE2_H_

/* Includes ------------------------------------------------------------------*/
#include "srml_std_lib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
/* Private macros ------------------------------------------------------------*/
#ifdef __cplusplus

/* 参数设置宏定义 */
// 数据类型
namespace LPMS_BE2
{
	enum DataConfig_e
	{
		DATA_16BIT = 2,
		DATA_32BIT = 4
	};

	// 数据输出单位
	enum UnitConfig_e
	{
		DEGREE = 0,
		RADIAN = 1
	};

#pragma pack(1) // 结构体按一个字节对齐
	struct RecInfo_s
	{
		uint8_t head;		// 包头
		uint8_t SensorID_L; // 低位传感器ID
		uint8_t SensorID_H; // 高位传感器ID
		uint8_t InsID_L;	// 低位指令号
		uint8_t InsID_H;	// 高位指令号
		uint8_t Lenth_L;	// 低位数据长度
		uint8_t Lenth_H;	// 高位数据长度
		/*LRC校验*/
		uint8_t LRC_L;
		uint8_t LRC_H;
		/*包尾*/
		uint8_t pack_tail_L;
		uint8_t pack_tail_H;
	};
#pragma pack()

	/*统一单位数据包*/
	struct Unity_Data_s
	{
		/*时间戳*/
		uint32_t timestamp;
		/*原始的加速度计数据 (g)*/
		float rawAccX;
		float rawAccY;
		float rawAccZ;
		/*校准后的加速度计数据 (g)*/
		float calAccX;
		float calAccY;
		float calAccZ;
		/*原始的陀螺仪数据 (dps (默认) / rad/s)*/
		float rawGyroX;
		float rawGyroY;
		float rawGyroZ;
		/*静止偏差校准后的陀螺仪数据 (dps (默认) / rad/s)*/
		float caliGyroX;
		float caliGyroY;
		float caliGyroZ;
		/*坐标轴校准后的陀螺仪数据 (dps (默认) / rad/s)*/
		float aligGyroX;
		float aligGyroY;
		float aligGyroZ;
		/*角速度(rad/s)*/
		float angularSpeedX;
		float angularSpeedY;
		float angularSpeedZ;
		/*四元数 (归一化单位*/
		float Quaternion1;
		float Quaternion2;
		float Quaternion3;
		float Quaternion4;
		/*欧拉角数据 (degree (默认) / r)*/
		float Euler_Roll;
		float Euler_Pitch;
		float Euler_Yaw;
		/*线性加速度(g)*/
		float linearAccX;
		float linearAccY;
		float linearAccZ;
		/*温度*/
		float temperature;
	};
};
/* Private type --------------------------------------------------------------*/
/*使能列表结构体*/
struct LpmsEnable_Structdef
{
	uint8_t RAW_ACC : 1;
	uint8_t CAL_ACC : 1;
	uint8_t RAW_GYRO : 1;
	uint8_t CALI_GYRO : 1;
	uint8_t ALIG_GYRO : 1;
	uint8_t ANGULAR_SPEED : 1;
	uint8_t QUETERNION : 1;
	uint8_t EULER : 1;
	uint8_t LINEAR_ACC : 1;
	uint8_t TEMPERATURE : 1;
	uint8_t REVERSE : 6;
};

class LPMS_BE2_Lite_Classdef
{
public:
	LPMS_BE2_Lite_Classdef(uint8_t _sensor_id, LPMS_BE2::DataConfig_e _data_type, LPMS_BE2::UnitConfig_e _degrad_unit)
		: sensor_id(_sensor_id), data_type(_data_type), degrad_unit(_degrad_unit){}

	void Init(LpmsEnable_Structdef _list); // 初始化函数
	uint8_t Update(uint8_t *data);		   // 读取原始数据函数
	uint32_t Link_Check();
	const LPMS_BE2::Unity_Data_s &get_data() { return unity_data; } // 返回一个联合数据类型的常量引用（不能被外部更改）
protected:
	bool is_init = false; // 陀螺仪是否成功初始化
	bool is_reply = false; // 是否收到应答信号
	uint8_t sensor_id;	   // 传感器id号
	uint32_t link_count = 0;
	const LPMS_BE2::DataConfig_e data_type;	  // 接收数据类型
	const LPMS_BE2::UnitConfig_e degrad_unit; // 输出数据单位

	LpmsEnable_Structdef enable_list;
	LPMS_BE2::RecInfo_s rec_info;
	LPMS_BE2::Unity_Data_s unity_data;

	void Data_Update_32(uint8_t *_data);
	void Data_Update_16(uint8_t *_data);
};

// /*设置初始化指令发送间隔时间*/
// #define COMMAND_DELAY 100 // 目前测得较稳定指令间隔(ms)
// // 陀螺仪量程范围
// #define GYR_RANGE_125DPS 125
// #define GYR_RANGE_250DPS 250
// #define GYR_RANGE_500DPS 500
// #define GYR_RANGE_1000DPS 1000
// #define GYR_RANGE_2000DPS 2000
// // 加速度量程范围
// #define ACC_RANGE_2G 2
// #define ACC_RANGE_4G 4
// #define ACC_RANGE_8G 8
// #define ACC_RANGE_16G 16
// // 滤波模式
// #define GYR_ONLY 0
// #define KALMAN 1
// #define DCM 3
// // 使能自动校准
// #define AUTO_CALI_ENABLE 1
// #define AUTO_CALI_DISABLE 0
// // 调整坐标系模式
// #define OBJECT 0
// #define HEADING 1
// #define ALIGMENT 2
// // 设置数据流频率
// #define FREQ_5HZ 5
// #define FREQ_10HZ 10
// #define FREQ_50HZ 50
// #define FREQ_100HZ 10
// #define FREQ_250HZ 250
// #define FREQ6_500HZ 500
// /* 发送指令标准格式包*/
// #pragma pack(1)
// typedef struct _CommandPack_Structdef
// {
// 	uint8_t head = 0x3A;
// 	uint8_t SensorID_L;		 // 低位传感器ID
// 	uint8_t SensorID_H;		 // 高位传感器ID
// 	uint8_t Command_L;		 // 低位指令
// 	uint8_t Command_H;		 // 高位指令
// 	uint8_t Length_L = 0x00; // 低位长度
// 	uint8_t Length_H = 0x00; // 高位长度
// 	/*数据*/
// 	uint8_t data1 = 0;
// 	uint8_t data2 = 0;
// 	uint8_t data3 = 0;
// 	uint8_t data4 = 0;
// 	/*LRC校验*/
// 	uint8_t LRC_L;
// 	uint8_t LRC_H;
// 	/*包尾*/
// 	uint8_t pack_tail_L = 0x0D;
// 	uint8_t pack_tail_H = 0x0A;
// } CommandPack_Structdef;
// #pragma pack()
// class LPMS_BE2_Classdef : public LPMS_BE2_Lite_Classdef
// {
// public:
// 	QueueHandle_t *USART_TxPort = nullptr;
// 	LPMS_BE2_Classdef(uint8_t _sensor_id, LPMS_BE2::DataConfig_e _data_type, LPMS_BE2::UnitConfig_e _degrad_unit)
// 		: LPMS_BE2_Lite_Classdef(_sensor_id, _data_type, _degrad_unit)
// 	{
// 	}
// 	void Transmit_Init(QueueHandle_t *_USART_TxPort, uint8_t _usart_num)
// 	{
// 		USART_TxPort = (_USART_TxPort);
// 		usart_num = (_usart_num);
// 	}
// 	void Data_Type_Config(uint8_t _data_type);												 // 陀螺仪数据输出类型设置函数
// 	void Stream_Freq_Config(uint16_t _stream_freq);											 // 陀螺仪数据流频率设置函数
// 	void Data_Range_Config(uint16_t _gyr_range, uint8_t _acc_range, uint8_t _degrad_unit);	 // 陀螺仪数据量程设置函数
// 	void Filter_Mode_Config(uint8_t _filter_mode, uint8_t _auto_cali, uint8_t _offset_mode); // 陀螺仪滤波修正设置函数
// protected:
// 	uint8_t usart_num;						 // 使用串口串口号
// 	uint16_t stream_freq = FREQ6_500HZ;		 // 数据流频率
// 	uint8_t orientation_offset = ALIGMENT;	 // 坐标系调整模式
// 	uint8_t acc_range = ACC_RANGE_4G;		 // 加速度量程
// 	uint16_t gyr_range = GYR_RANGE_2000DPS;	 // 陀螺仪量程
// 	uint8_t filter_mode = KALMAN;			 // 滤波模式
// 	uint8_t offset_mode = ALIGMENT;			 // 调整传感器坐标系模式
// 	uint32_t baudrate = 230400;				 // 波特率
// 	uint8_t is_auto_cali = AUTO_CALI_ENABLE; // 是否开启自动校准
// 	uint8_t send_data_large[15];			 // 发送指令数据包
// 	uint8_t send_data_small[11];
// 	CommandPack_Structdef command;
// 	// 收发数据函数
// 	void LPMS_BE2_Send_Command(CommandPack_Structdef *command_pack, uint8_t usart_port_num);				 // 发送普通指令函数
// 	void LPMS_BE2_Command_Pack(CommandPack_Structdef *_command_pack, uint16_t _command, uint32_t _data = 0); // 指令发送打包函数
// 	// 陀螺仪初始化配置函数
// 	void LPMS_BE2_Data_Type(uint32_t _data_type);
// 	void LPMS_BE2_Data_Range(uint32_t _gyr_range, uint32_t _acc_range, uint32_t _degrad_unit);
// 	void LPMS_BE2_Filter_Mode(uint32_t _filter_mode, uint32_t _auto_cali, uint32_t _offset_mode);
// 	// 陀螺仪指令发送函数
// 	void set_imu_transmit_data();
// 	void goto_command_mode();
// 	void goto_stream_mode();
// 	void write_registers();
// 	void restore_factory_value();
// 	void set_imu_id(uint32_t id);
// 	void reset_orientation_offset();
// 	void start_gyr_calibration();
// 	uint8_t set_stream_freq(uint32_t freq);
// 	uint8_t set_degrad_output(uint32_t unit);
// 	uint8_t set_orientation_offset(uint32_t mode);
// 	uint8_t set_acc_range(uint32_t range);
// 	uint8_t set_gyr_range(uint32_t range);
// 	uint8_t set_enable_gyr_autocalibration(uint32_t is_enable);
// 	uint8_t set_filter_mode(uint32_t mode);
// 	uint8_t set_uart_baudrate(uint32_t baud);
// 	uint8_t set_lpbus_data_precision(uint32_t unit);
// };
#endif /* __cplusplus */

#endif /* _LPMS_BE2_H_ */

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
