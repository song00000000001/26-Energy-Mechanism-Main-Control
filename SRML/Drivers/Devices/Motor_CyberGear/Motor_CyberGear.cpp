#include "srml_config.h"

#if USE_SRML_CYBER_GEAR
#include "Motor_CyberGear.h"

using namespace CyberGear;
using std_lib::float_to_uint;
using std_lib::uint_to_float;

/**
 * @brief 运控模式电机控制指令（通信类型1）
 *
 * @param position  目标位置(rad)
 * @param velocity  目标速度(rad/s)
 * @param kp        位置环增益
 * @param kd        速度环增益
 * @param torque    附加力矩(N*m)
 */
void Motor_CyberGear_Classdef::control(float position, float velocity, float kp, float kd, float torque)
{
  if (DataPack.mode_state == RESET_MODE)
  {
    startMotor();
  }
  /* 限制输入的参数在定义的范围内 */
  LIMIT_MIN_MAX(position, P_MIN, P_MAX);
  LIMIT_MIN_MAX(velocity, V_MIN, V_MAX);
  LIMIT_MIN_MAX(kp, KP_MIN, KP_MAX);
  LIMIT_MIN_MAX(kd, KD_MIN, KD_MAX);
  LIMIT_MIN_MAX(torque, T_MIN, T_MAX);

  EXTID.mode = CONTROL_MOTOR;
  EXTID.data = float_to_uint(torque, T_MIN, T_MAX, 16);

  uint16_t pos_tmp = float_to_uint(position, P_MIN, P_MAX, 16);
  uint16_t vel_tmp = float_to_uint(velocity, V_MIN, V_MAX, 16);
  uint16_t kp_tmp = float_to_uint(kp, KP_MIN, KP_MAX, 16);
  uint16_t kd_tmp = float_to_uint(kd, KD_MIN, KD_MAX, 16);

  TxPack.Data[0] = pos_tmp >> 8;
  TxPack.Data[1] = pos_tmp;
  TxPack.Data[2] = vel_tmp >> 8;
  TxPack.Data[3] = vel_tmp;
  TxPack.Data[4] = kp_tmp >> 8;
  TxPack.Data[5] = kp_tmp;
  TxPack.Data[6] = kd_tmp >> 8;
  TxPack.Data[7] = kd_tmp;

  TxPack.ID = *((uint32_t *)&(EXTID));
  xQueueSend(Tx_Handle, &TxPack, 0);
};

/**
 * @brief 电机数据接收处理函数
 *
 * @param canRecID CAN接收ID
 * @param data CAN接收数据
 * @return true ID检测通过，数据成功更新
 * @return false ID不匹配
 */
bool Motor_CyberGear_Classdef::update(uint32_t canRecID, uint8_t data[8])
{
  DataPack = *(DataPack_Typedef *)&(canRecID);

  if (DataPack.motor_id != EXTID.motor_id)
    return 0;

  if (DataPack.mode_state == MOTOR_FEEDBACK)
  {
    update_angle(data);
    int32_t v_int = (data[2] << 8 | data[3]);
    int32_t t_int = (data[4] << 8 | data[5]);
    Rec_Data.velocity = uint_to_float(v_int, V_MIN, V_MAX, 16);
    Rec_Data.torque = uint_to_float(t_int, T_MIN, T_MAX, 16);
    Rec_Data.temperature = (data[6] << 8 | data[7]) / 10.0f;
  }
  else if (DataPack.mode_state == MOTOR_ERROR)
  {
    while (1)
    {
      stopMotor();
    }
  }

  return 1;
}

/**
 * @brief 设置编码器offset
 *
 * @param offset
 */
void Motor_CyberGear_Classdef::setEncoderOffset(uint16_t offset)
{
  this->encoder_offset = offset;
  this->last_encoder = offset;
  this->encoder_is_init = true;
}

/**
 * @brief 发送指令
 * 
 * @param comType 通信类型
 * @param data 附带数据
 */
void Motor_CyberGear_Classdef::sendCommand(CyberGear::MOTOR_COM_Enumdef comType, uint16_t data)
{
  EXTID.mode = comType;
  EXTID.data = data;
  TxPack.ID = *((uint32_t *)&(EXTID));
  xQueueSend(Tx_Handle, &TxPack, 0);
}

void Motor_CyberGear_Classdef::update_angle(uint8_t can_rx_data[])
{
  encoder = (uint16_t)(can_rx_data[0] << 8) | can_rx_data[1];
  uint16_t pos_temp = ((uint16_t)(encoder - encoder_offset)) % (2 << 14);
  Rec_Data.position = std_lib::uint_to_float(pos_temp, 0, 2 * PI, 14);

  if (encoder_is_init)
  {
    if (this->encoder - this->last_encoder > encoder_max / 2)
      this->round_cnt--;
    else if (this->encoder - this->last_encoder < -encoder_max / 2)
      this->round_cnt++;
  }
  else
  {
    encoder_offset = encoder;
    encoder_is_init = true;
  }
  this->last_encoder = this->encoder;
  int32_t total_encoder = round_cnt * encoder_max + encoder - encoder_offset;
  /* 该电机转四圈编码器才跑满一个65535 */
  Rec_Data.angle = total_encoder * 360.f * 4.f / 65535.f;
}
#endif /* USE_SRML_CYBER_GEAR */
