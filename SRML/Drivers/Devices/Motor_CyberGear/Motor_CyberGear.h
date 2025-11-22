#ifndef _CYBER_GEAR_H_
#define _CYBER_GEAR_H_

#ifdef __cplusplus
#include "srml_std_lib.h"
#include "Drivers/Components/drv_can.h"
#include "FreeRTOS.h"
#include "queue.h"

namespace CyberGear
{
    typedef enum
    {
        GET_ID = 0,
        CONTROL_MOTOR = 1,
        MOTOR_FEEDBACK = 2,
        START_MOTOR = 3,
        STOP_MOTOR = 4,
        SET_ZERO_POSITION = 6,
        CHANGE_ID = 7,
        GET_PARAMETER = 17,
        CHANGE_PARAMETER = 18,
        MOTOR_ERROR = 21
    } MOTOR_COM_Enumdef;

    typedef enum
    {
        OK = 0,                  // 无故障
        BAT_LOW_ERR = 1,         // 欠压故障
        OVER_CURRENT_ERR = 2,    // 过流
        OVER_TEMP_ERR = 3,       // 过温
        MAGNETIC_ERR = 4,        // 磁编码故障
        HALL_ERR_ERR = 5,        // HALL编码故障
        NO_CALIBRATION_ERR = 6   // 未标定
    } MOTOR_ERROR_STATE_Enumdef; // 电机状态（故障信息）

    typedef enum
    {
        CONTROL_MODE = 0,      // 运控模式
        LOCATION_MODE = 1,     // 位置模式
        SPEED_MODE = 2,        // 速度模式
        CURRENT_MODE = 3       // 电流模式
    } MOTOR_CTRL_MODE_Enumdef; // 电机运行模式

    typedef enum
    {
        IQ_REF = 0X7006,        // 电流模式Iq指令
        SPD_REF = 0X700A,       // 转速模式转速指令
        LIMIT_TORQUE = 0X700B,  // 转矩限制
        CUR_KP = 0X7010,        // 电流的 Kp
        CUR_KI = 0X7011,        // 电流的 Ki
        CUR_FILT_GAIN = 0X7014, // 电流滤波系数filt_gain
        LOC_REF = 0X7016,       // 位置模式角度指令
        LIMIT_SPD = 0X7017,     // 位置模式速度设置
        LIMIT_CUR = 0X7018      // 速度位置模式电流设置
    } MOTOR_PARAM_ID_Enumdef;   // 电机功能码

    typedef enum
    {
        RESET_MODE = 0,   // Reset模式[复位]
        CALI_MODE = 1,    // Cali 模式[标定]
        RUN_MODE = 2      // Motor模式[运行]
    } MOTOR_MODE_Enumdef; // 电机模式状态

#pragma pack(1)
    typedef struct
    {
        uint32_t motor_id : 8; // 只占8位
        uint32_t data : 16;
        uint32_t mode : 5;
        uint32_t res : 3;
    } EXT_ID_t; // 32位扩展ID解析结构体

    typedef struct
    {
        uint32_t master_can_id : 8;
        uint32_t motor_id : 8;
        uint32_t under_voltage_fault : 1;
        uint32_t over_current_fault : 1;
        uint32_t over_temperature_fault : 1;
        uint32_t magnetic_encoding_fault : 1;
        uint32_t HALL_encoding_failure : 1;
        uint32_t unmarked : 1;
        uint32_t mode_state : 2;
        uint32_t communication_type : 5;
        uint32_t res : 3;
    } DataPack_Typedef; // 小米电机解码内容对齐
#pragma pack()

};

class Motor_CyberGear_Classdef
{
private:
    /* 数据池 */
    CyberGear::EXT_ID_t EXTID; // CAN拓展帧32位ID部分
    CyberGear::DataPack_Typedef DataPack;

    const int32_t encoder_max = 65536; /* 码盘值最大值 */
    bool encoder_is_init = false;
    int32_t round_cnt = 0;
    uint16_t encoder, encoder_offset = 0, last_encoder = 0;

    /*接收参数*/
    struct RecData_Structdef_
    {
        float angle;       // 多圈角度(°)
        float position;    // 单圈位置(rad)
        float velocity;    // 当前速度(rad/s)
        float torque;      // 当前力矩(N*m)
        float temperature; //(℃)
    } Rec_Data;

    QueueHandle_t Tx_Handle;
    CAN_COB TxPack = {Can_EXTID, 0, 8, {}};

public:
    Motor_CyberGear_Classdef(uint8_t _id)
    {
        EXTID.res = 0;
        EXTID.motor_id = _id;
    }
    /**
     * @brief 绑定CAN发送队列
     *
     * @param _sendQueue 被绑定的CAN发送队列
     */
    inline void bindCanQueueHandle(QueueHandle_t _sendQueue)
    {
        Tx_Handle = _sendQueue;
        startMotor();
        setZeroPos();
    }

    void control(float position, float velocity, float kp, float kd, float torque);
    inline void setTorque(float torque) { control(0, 0, 0, 0, torque); }                                          // 力矩控制
    inline void setPosition(float _position, float _kp, float _kd = 0) { control(_position, 0, _kp, _kd, 0); }    // 内置位置控制
    inline void setSpeed(float _velocity, float _kd, float torque = 0) { control(0, _velocity, 0, _kd, torque); } // 内置速度控制

    bool update(uint32_t canRecID, uint8_t data[8]);
    inline uint8_t getComID() { return EXTID.motor_id; }
    inline const RecData_Structdef_ &getRecData() { return Rec_Data; }

    void sendCommand(CyberGear::MOTOR_COM_Enumdef comType, uint16_t data = 0);
    inline void startMotor() { sendCommand(CyberGear::START_MOTOR); }
    inline void stopMotor() { sendCommand(CyberGear::STOP_MOTOR); }
    inline void setZeroPos() { sendCommand(CyberGear::SET_ZERO_POSITION); }
    inline void setMotorID(uint8_t _id)
    {
        stopMotor();
        sendCommand(CyberGear::CHANGE_ID, ((uint16_t)_id) << 8);
        EXTID.motor_id = _id;
    }

    /* 设置编码器offset */
    void setEncoderOffset(uint16_t offset);

private:
    void update_angle(uint8_t can_rx_data[]);
    /* 需要的常量定义 */
    const float P_MIN = -4 * PI;
    const float P_MAX = 4 * PI;
    const float V_MIN = -30.0f;
    const float V_MAX = 30.0f;
    const float KP_MIN = 0.0f;
    const float KP_MAX = 500.0f;
    const float KD_MIN = 0.0f;
    const float KD_MAX = 5.0f;
    const float T_MIN = -12.0f;
    const float T_MAX = 12.0f;
};

#endif  /* __cplusplus */

#endif  /* _CYBER_GEAR_H_ */