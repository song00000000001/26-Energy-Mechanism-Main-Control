#ifndef _HT04_H_
#define _HT04_H_

/* Includes ------------------------------------------------------------------*/
#include "srml_std_lib.h"
#include "Drivers/Components/drv_can.h"
#include "Middlewares/Algorithm/MultiTurnRecorder/MultiTurnRecorder.h"
#include "FreeRTOS.h"
#include "queue.h"

#ifdef __cplusplus

class MotorHT04Classdef : public MultiTurnRecorderClassdef
{
private:
	uint16_t link_count = 0;
	void MotorCmdPack(uint8_t cmd);
	/*接收参数*/
	struct RecData_Structdef_
	{
		float position; // 当前位置,(rad)
		float velocity; // 当前速度,(rad/s)
		float torque;	// 当前力矩,(N*m)
		float angle;	// 多圈角度,(°)
	} Rec_Data;

	/* can发送数据体 */
	CAN_COB TxPack = {Can_STDID, 0, 8, {}};
	QueueHandle_t Tx_Handle;

public:
	const uint8_t ID; // 电机id号
	MotorHT04Classdef(uint8_t _id) : ID(_id) { TxPack.ID = ID; }
	inline void bindCanQueueHandle(QueueHandle_t _sendQueue) { Tx_Handle = _sendQueue; }

	bool update(uint32_t _unuse_id, uint8_t can_rx_data[]);
	inline const RecData_Structdef_ &getRecData() { return Rec_Data; }

	void control(float position, float velocity, float kp, float kd, float torque);
	inline void setTorque(float torque) { control(0, 0, 0, 0, torque); }											 // 力矩控制
	inline void setPosition(float _position, float _kp, float _kd) { control(_position, 0, _kp, _kd, 0); }			 // 内置位置控制
	inline void setSpeed(float _velocity, float _kd, float torque = 0) { control(0, _velocity, 0, _kd, torque); } // 内置速度控制

	void startMotor();	  // 电机进入控制模式
	void stopMotor();	  // 电机进入RESET模式
	void cmd_zero_position(); // 设置电机零点

	uint16_t getLinkCount() { return link_count; }
	void checkLink()
	{
		link_count++;
		if (link_count > 1000)
		{
			link_count = 1000;
		}
	}
};
#endif

#endif
