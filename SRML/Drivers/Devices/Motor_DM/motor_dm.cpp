#include "srml_config.h"

#if USE_SRML_MOTOR_DM
#include "motor_dm.h"

void Motor_DM_classdef::setEncoderOffset(uint16_t offset)
{
    this->encoder_offset = offset;
    this->last_encoder = offset;
    this->encoder_is_init = true;
}

void Motor_DM_classdef::startMotor()
{
    TxPack.Data[0] = 0xFF;
    TxPack.Data[1] = 0xFF;
    TxPack.Data[2] = 0xFF;
    TxPack.Data[3] = 0xFF;
    TxPack.Data[4] = 0xFF;
    TxPack.Data[5] = 0xFF;
    TxPack.Data[6] = 0xFF;
    TxPack.Data[7] = 0xFC;
    xQueueSend(Tx_Handle, &TxPack, 0);
}

/* 发送失能帧给电机 */
void Motor_DM_classdef::stopMotor()
{
    TxPack.Data[0] = 0xFF;
    TxPack.Data[1] = 0xFF;
    TxPack.Data[2] = 0xFF;
    TxPack.Data[3] = 0xFF;
    TxPack.Data[4] = 0xFF;
    TxPack.Data[5] = 0xFF;
    TxPack.Data[6] = 0xFF;
    TxPack.Data[7] = 0xFD;
    xQueueSend(Tx_Handle, &TxPack, 0);
}

/**
 * @brief 清除错误标志位
 * @note 新DM电机第一次需要该帧清除出厂状态。
 */
void Motor_DM_classdef::ClearError()
{
    TxPack.Data[0] = 0xFF;
    TxPack.Data[1] = 0xFF;
    TxPack.Data[2] = 0xFF;
    TxPack.Data[3] = 0xFF;
    TxPack.Data[4] = 0xFF;
    TxPack.Data[5] = 0xFF;
    TxPack.Data[6] = 0xFF;
    TxPack.Data[7] = 0xFB;
    xQueueSend(Tx_Handle, &TxPack, 0);
}

/**
 * @brief
 * @param  电机目标位置
 * @param  电机目标速度
 * @param  kp（kp *（角度目标 - 当前角度））
 * @param  kd（kd *（速度目标 - 当前速度））
 */
/* 电机控制帧，一般只要对最后一个形参（力矩）赋值 */
void Motor_DM_classdef::control(float position, float velocity, float kp, float kd, float torque)
{
    /* 限制输入的参数在定义的范围内 */
    LIMIT_MIN_MAX(position, P_MIN, P_MAX);
    LIMIT_MIN_MAX(velocity, V_MIN, V_MAX);
    LIMIT_MIN_MAX(kp, KP_MIN, KP_MAX);
    LIMIT_MIN_MAX(kd, KD_MIN, KD_MAX);
    LIMIT_MIN_MAX(torque, T_MIN, T_MAX);

    uint16_t pos_tmp = std_lib::float_to_uint(position, P_MIN, P_MAX, 16);
    uint16_t vel_tmp = std_lib::float_to_uint(velocity, V_MIN, V_MAX, 12);
    uint16_t kp_tmp = std_lib::float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint16_t kd_tmp = std_lib::float_to_uint(kd, KD_MIN, KD_MAX, 12);
    uint16_t tor_tmp = std_lib::float_to_uint(torque, T_MIN, T_MAX, 12);

    TxPack.Data[0] = (pos_tmp >> 8);
    TxPack.Data[1] = pos_tmp;
    TxPack.Data[2] = (vel_tmp >> 4);
    TxPack.Data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    TxPack.Data[4] = kp_tmp;
    TxPack.Data[5] = (kd_tmp >> 4);
    TxPack.Data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    TxPack.Data[7] = tor_tmp;
    xQueueSend(Tx_Handle, &TxPack, 0);
}

/**
 * @brief 电机数据接收处理函数
 *
 * @param canRecID CAN接收ID
 * @param data CAN接收数据
 * @return true ID检测通过，数据成功更新
 * @return false ID不匹配
 */
bool Motor_DM_classdef::update(uint32_t _unuse_id, uint8_t data[8])
{
    UNUSED(_unuse_id);
    if ((this->ID) != (data[0] & 0x0F))
        //return false;

    update_angle(data);

    state = (data[0]) >> 4;
    int32_t v_int = (data[3] << 4) | (data[4] >> 4);
    int32_t t_int = ((data[4] & 0xF) << 8) | data[5];
    //Rec_Data.velocity = std_lib::uint_to_float(v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
    //Rec_Data.torque = std_lib::uint_to_float(t_int, T_MIN, T_MAX, 12);   // (-10.0,10.0)

    Rec_Data.T_mos = (float)(data[6]);
    Rec_Data.T_rotor = (float)(data[7]);

    return true;
}

void Motor_DM_classdef::update_angle(uint8_t can_rx_data[])
{
    encoder = (uint16_t)(can_rx_data[1] << 8) | can_rx_data[2];
    uint16_t pos_temp = ((uint16_t)(encoder - encoder_offset)) % (2 << 14);
    //Rec_Data.position = std_lib::uint_to_float(pos_temp, 0, 2 * PI, 14);

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
#endif
