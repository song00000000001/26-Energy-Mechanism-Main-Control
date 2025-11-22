#include "srml_config.h"

#if USE_SRML_HT04

#include "HT04.h"

// 电机默认设定参数
#define P_MIN -95.5f // Radians
#define P_MAX 95.5f
#define V_MIN -45.0f // Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f // N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN -35.0f
#define T_MAX 35.0f

#define CMD_MOTOR_MODE 0xFC    // 电机进入控制模式
#define CMD_RESET_MODE 0xFD    // 进入零点设置模式
#define CMD_ZERO_POSITION 0xFE // 设置零点

using namespace std_lib;
/*
 * @brief 	can通信接收解包函数
 * @param	can_rx_data[]			 can通信数据包
 * @return 	NULL
 */
bool MotorHT04Classdef::update(uint32_t _unuse_id, uint8_t can_rx_data[])
{
    UNUSED(_unuse_id);
    if (can_rx_data[0] != ID)
        return 0;

    link_count = 0;
    Rec_Data.position = uint_to_float((can_rx_data[1] << 8) + can_rx_data[2], P_MIN, P_MAX, 16);
    Rec_Data.velocity = uint_to_float((can_rx_data[3] << 4) + (can_rx_data[4] >> 4), V_MIN, V_MAX, 12);
    Rec_Data.torque = uint_to_float(((can_rx_data[4] & 0x0F) << 8) + (can_rx_data[5]), T_MIN, T_MAX, 12);
    Rec_Data.angle = MultiTurnRecord(Rec_Data.position * 180 / PI);
    return 1;
}

/*
 * @brief 	写入控制参数
 * @param	float position
 * @param   float velocity
 * @param   float kp
 * @param   float kd
 * @param   float torque
 * @return 	NULL
 */
void MotorHT04Classdef::control(float position, float velocity, float kp, float kd, float torque)
{
    /* 限制输入的参数在定义的范围内 */
    LIMIT_MIN_MAX(position, P_MIN, P_MAX);
    LIMIT_MIN_MAX(velocity, V_MIN, V_MAX);
    LIMIT_MIN_MAX(kp, KP_MIN, KP_MAX);
    LIMIT_MIN_MAX(kd, KD_MIN, KD_MAX);
    LIMIT_MIN_MAX(torque, T_MIN, T_MAX);

    /* 根据协议，对float参数进行转换 */
    uint32_t pos_tmp = float_to_uint(position, P_MIN, P_MAX, 16); // 将f_p转换为一个16位整数，转换规则见该函数的定义
    uint32_t vel_tmp = float_to_uint(velocity, V_MIN, V_MAX, 12); // 将f_v转换为一个12位整数，虽然v是16位数，但高四位全为0，所以f_v=0000xxxx xxxxxxxx
    uint32_t kp_tmp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint32_t kd_tmp = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    uint32_t tor_tmp = float_to_uint(torque, T_MIN, T_MAX, 12);

    /* 根据传输协议，把数据转换为CAN命令数据字段 */
    TxPack.Data[0] = pos_tmp >> 8;                           // p的高8位
    TxPack.Data[1] = pos_tmp & 0xFF;                         // p的低8位
    TxPack.Data[2] = vel_tmp >> 4;                           // 从上一步操作可知v的有效值仅为12位，该操作是将v有效值中的高8位赋给该字节
    TxPack.Data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8); // 该字节高4位是v有效值的低4位；该字节低4位是kp有效值的高4位。
    TxPack.Data[4] = kp_tmp & 0xFF;                          // kp有效值的低8位。
    TxPack.Data[5] = kd_tmp >> 4;                            // 从上一步操作可知kd的有效值仅为12位，该操作是将kd有效值中的高8位赋给该字节
    TxPack.Data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8); // 该字节高4位是kd有效值的低4位；该字节低4位是t有效值的高4位。
    TxPack.Data[7] = tor_tmp & 0xff;                         // t有效值的低8位。

    xQueueSend(Tx_Handle, &TxPack, 0);
}

/*
 * @brief 	发送特定指令
 * @param	cmd			 电机特定指令
 * @return 	uint8_t buf[8]
 */
void MotorHT04Classdef::MotorCmdPack(uint8_t cmd)
{
    static uint8_t buff[7] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    memcpy(TxPack.Data, buff, 7);
    switch (cmd)
    {
    case CMD_MOTOR_MODE:
        TxPack.Data[7] = CMD_MOTOR_MODE;
        break;

    case CMD_RESET_MODE:
        TxPack.Data[7] = CMD_RESET_MODE;
        break;

    case CMD_ZERO_POSITION:
        TxPack.Data[7] = CMD_ZERO_POSITION;
        break;

    default:
        return; /* 直接退出函数 */
    }
}
void MotorHT04Classdef::startMotor()
{
    MotorCmdPack(CMD_MOTOR_MODE);
    xQueueSend(Tx_Handle, &TxPack, 0);
}
void MotorHT04Classdef::stopMotor()
{
    MotorCmdPack(CMD_RESET_MODE);
    xQueueSend(Tx_Handle, &TxPack, 0);
}
void MotorHT04Classdef::cmd_zero_position()
{
    MotorCmdPack(CMD_ZERO_POSITION);
    xQueueSend(Tx_Handle, &TxPack, 0);
}

#endif /* USE_SRML_HT04 */
