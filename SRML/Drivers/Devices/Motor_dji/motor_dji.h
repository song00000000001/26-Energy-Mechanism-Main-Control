/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    motor_dji.h
  * @author  BigeYoung 851756890@qq.com & M3chD09
  * @brief   Code for Motor driver, Including almost every motor model that used
  *			     in robomaster(Since robomaster 2019).
  * @date    2019-06-26
  * @version 1.2
  *
  ==============================================================================
                          How to use this library
  ==============================================================================
    @see
      - To view more details about how to use this library, please visit: \n
        https://www.scut-robotlab.cn/git/Embedded/motor.git
    @warning
      - Standard C++11 required!
      - SRML Package(drv_can) required!
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

/* Includes ------------------------------------------------------------------*/
#include "srml_std_lib.h"
#include "Drivers/Components/drv_can.h"
#include "Middlewares/Algorithm/PID/PID.h"
#include "Middlewares/Algorithm/MultiTurnRecorder/MultiTurnRecorder.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "Drivers/Devices/Motor_DM/motor_dm.h"
#ifdef __cplusplus
/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
struct Motor_CAN_COB
{
    CAN_COB Id200 = {Can_STDID, 0x200, 8, {0}};
    CAN_COB Id1ff = {Can_STDID, 0x1ff, 8, {0}};
    CAN_COB Id2ff = {Can_STDID, 0x2ff, 8, {0}};
    CAN_COB Id3fe = {Can_STDID, 0x3fe, 8, {0}};
};
/* Exported variables --------------------------------------------------------*/
/* Exported function declarations --------------------------------------------*/

/**
 * @brief 匿名命名空间，存放几个外部不需要使用的基类
 *
 */
namespace
{
    /**
     * @brief 电机速度部分
     *
     */
    class MotorSpeed
    {
    public:
        float getSpeed() const { return this->speed; }

    protected:
        int16_t speed = 0;
        void update_speed(uint8_t can_rx_data[])
        {
            this->speed = (int16_t)(can_rx_data[2] << 8 | can_rx_data[3]);
        }
    };

    /**
     * @brief 电机电流部分
     *
     */
    class MotorCurrent
    {
    public:
        int16_t getTorqueCurrent() const { return TorqueCurrent; }

    protected:
        int16_t TorqueCurrent;
        void update_current(uint8_t can_rx_data[])
        {
            this->TorqueCurrent = (int16_t)(can_rx_data[4] << 8 | can_rx_data[5]);
        }
    };

    /**
     * @brief 电机角度部分
     *
     */
    class MotorAngle : public MultiTurnRecorderClassdef
    {
    public:
        float getAngle() const { return angle; }
        uint16_t getEncoder() const { return encoder; }

        inline void setEncoderOffset(uint16_t offset)
        {
            setAngleOffset(offset / ENCODER_ANGLE_RATIO());
        }

    protected:
        uint16_t encoder = 0;
        float angle = 0;

        int16_t ENCODER_MAX() const { return 8192; }
        float ENCODER_ANGLE_RATIO() const { return 8192.0f / 360.0f; }

        void update_angle(uint8_t can_rx_data[])
        {
            encoder = (uint16_t)(can_rx_data[0] << 8 | can_rx_data[1]);
            angle = MultiTurnRecord(encoder / ENCODER_ANGLE_RATIO());
        }
    };

    /**
     * @brief 电机通信部分
     *
     */
    class MotorComm
    {
    public:
        const uint8_t ID = 0;
        float Out = 0; /*!< Output ampere value that sent to ESC */

        MotorComm(uint8_t id) : ID(id) {}
        ~MotorComm() {}

        uint32_t getRecID() { return this->REC_ID_INIT() + (uint32_t)ID; }
        virtual bool update(uint32_t canRecID, uint8_t data[8]) = 0;

        virtual uint32_t REC_ID_INIT() = 0;
        virtual uint32_t SEND_ID_LOW() = 0;
        virtual uint32_t SEND_ID_HIGH() = 0;
        virtual float MAX_CURRENT() = 0;
    };
};

/**
  @brief  Motor that used ESC C610.
*/
class Motor_C610 : public MotorComm, public MotorAngle, public MotorSpeed, public MotorCurrent
{
public:
    Motor_C610(uint8_t id) : MotorComm(id) {}

    virtual bool update(uint32_t canRecID, uint8_t data[8]) override
    {
        if (canRecID != getRecID())
            return 0;
        this->update_angle(data);
        this->update_speed(data);
        this->update_current(data);
        return 1;
    }

    virtual uint32_t REC_ID_INIT() override { return 0x200; }
    virtual uint32_t SEND_ID_LOW() override { return 0x200; }
    virtual uint32_t SEND_ID_HIGH() override { return 0x1ff; }
    virtual float MAX_CURRENT() override { return 10000; }
};

/**
  @brief  Motor that used ESC C620.
*/
class Motor_C620 : public MotorComm, public MotorAngle, public MotorSpeed, public MotorCurrent
{
public:
    Motor_C620(uint8_t id) : MotorComm(id) {}

    uint8_t getTempature() const { return temperature; }

    virtual bool update(uint32_t canRecID, uint8_t data[8]) override
    {
        if (canRecID != getRecID())
            return 0;
        this->update_angle(data);
        this->update_speed(data);
        this->update_current(data);
        temperature = data[6];
        return 1;
    }

    virtual uint32_t REC_ID_INIT() override { return 0x200; }
    virtual uint32_t SEND_ID_LOW() override { return 0x200; }
    virtual uint32_t SEND_ID_HIGH() override { return 0x1ff; }
    virtual float MAX_CURRENT() override { return 16384; }

private:
    uint8_t temperature = 0;
};

/**
  @brief  GM6020.
*/
class Motor_GM6020 : public MotorComm, public MotorAngle, public MotorSpeed, public MotorCurrent
{
public:
    /**
     * @brief 电机类构造函数
     *
     * @param id 电机id号，即指示灯每秒闪烁频率
     */
    Motor_GM6020(uint8_t id) : MotorComm(id) {}

    virtual bool update(uint32_t canRecID, uint8_t data[8]) override
    {
        if (canRecID != getRecID())
            return 0;
        this->update_angle(data);
        this->update_speed(data);
        this->update_current(data);
        temperature = data[6];
        return 1;
    }
    virtual uint32_t REC_ID_INIT() override { return 0x204; }
    virtual uint32_t SEND_ID_LOW() override { return 0x1ff; }
    virtual uint32_t SEND_ID_HIGH() override { return 0x2ff; }
    virtual float MAX_CURRENT() override { return 30000; }

    uint8_t getTempature() const { return temperature; }

    float currentLoop_Kp = 0.4f;
    float EMFCompensationScale = 68.9f; // 反电动势补偿系数
    float R = 1.15f;                    // 广义相电阻
    void setCurrentOut(float target_current)
    {
        this->Out = currentLoop_Kp * (target_current - this->TorqueCurrent) + (EMFCompensationScale * this->speed) + target_current * R;
    }

private:
    uint8_t temperature = 0;
};

/**
  @brief  Motor that used ESC 820R.
*/
class Motor_820R : public MotorComm, public MotorAngle, public MotorSpeed
{
public:
    Motor_820R(uint8_t id) : MotorComm(id) {}
    virtual bool update(uint32_t canRecID, uint8_t data[8]) override
    {
        if (canRecID != getRecID())
            return 0;
        this->update_angle(data);
        this->update_speed(data);
        return 1;
    }

    virtual uint32_t REC_ID_INIT() override { return 0x200; }
    virtual uint32_t SEND_ID_LOW() override { return 0x200; }
    virtual uint32_t SEND_ID_HIGH() override { return 0x1ff; }
    virtual float MAX_CURRENT() override { return 16384; }
};
/**
  @brief  GM3510.
*/
class Motor_GM3510 : public MotorComm, public MotorAngle, public MotorCurrent
{
public:
    Motor_GM3510(uint8_t id) : MotorComm(id) {}
    virtual bool update(uint32_t canRecID, uint8_t data[8]) override
    {
        if (canRecID != getRecID())
            return 0;
        this->update_angle(data);
        this->update_current(data);
        return 1;
    }

    virtual uint32_t REC_ID_INIT() override { return 0x204; }
    virtual uint32_t SEND_ID_LOW() override { return 0x1ff; }
    virtual uint32_t SEND_ID_HIGH() override { return 0; }
    virtual float MAX_CURRENT() override { return 29000; }

protected:
    void update_current(uint8_t can_rx_data[])
    {
        this->TorqueCurrent = (int16_t)(can_rx_data[2] << 8 | can_rx_data[3]);
    }
};
/**
  @brief  Motor 6623.
*/
class Motor_6623 : public MotorComm, public MotorAngle, public MotorCurrent
{
public:
    Motor_6623(uint8_t id) : MotorComm(id) {}
    virtual bool update(uint32_t canRecID, uint8_t data[8]) override
    {
        if (canRecID != getRecID())
            return 0;
        this->update_angle(data);
        this->update_current(data);
        return 1;
    }

    virtual uint32_t REC_ID_INIT() override { return 0x204; }
    virtual uint32_t SEND_ID_LOW() override { return 0x1ff; }
    virtual uint32_t SEND_ID_HIGH() override { return 0x2ff; }
    virtual float MAX_CURRENT() override { return 5000; }

protected:
    void update_current(uint8_t can_rx_data[])
    {
        this->TorqueCurrent = (int16_t)(can_rx_data[2] << 8 | can_rx_data[3]);
    }
};

namespace motor_dji
{
    /**
      Send CAN communication object.
    */
    template <class MotorType, int N>
    void MotorMsgSend(QueueHandle_t CanQueueHander, MotorType (&motors)[N])
    {
        uint8_t TxDataLow[8] = {0};
        uint8_t TxDataHigh[8] = {0};
        int16_t constrain_Out = 0;
        bool low = false;
        bool high = false;
        for (int i = 0; i < N; i++)
        {
            motors[i].Out = std_lib::constrain(motors[i].Out, -motors[0].MAX_CURRENT(), motors[0].MAX_CURRENT());
            constrain_Out = motors[i].Out;
            if (motors[i].ID <= 4 && motors[i].ID > 0)
            {
                low = true;
                TxDataLow[motors[i].ID * 2 - 2] = (constrain_Out >> 8) & 0xff;
                TxDataLow[motors[i].ID * 2 - 1] = constrain_Out & 0xff;
            }
            else if (motors[i].ID <= 8 && motors[i].ID > 4)
            {
                high = true;
                TxDataHigh[motors[i].ID * 2 - 10] = (constrain_Out >> 8) & 0xff;
                TxDataHigh[motors[i].ID * 2 - 9] = constrain_Out & 0xff;
            }
        }

        static CAN_COB txPack = {Can_STDID, 0, 8};
        if (low)
        {
            txPack.ID = motors[0].SEND_ID_LOW();
            memcpy(txPack.Data, TxDataLow, 8);
            xQueueSend(CanQueueHander, &txPack, 0);
        }

        if (high)
        {
            txPack.ID = motors[0].SEND_ID_HIGH();
            memcpy(txPack.Data, TxDataHigh, 8);
            xQueueSend(CanQueueHander, &txPack, 0);
        }
    }

    template <class MotorType>
    void MotorMsgSend(QueueHandle_t CanQueueHander, MotorType &motor);

    template <class MotorType>
    Motor_CAN_COB &MotorMsgPack(Motor_CAN_COB &motor_msg, MotorType &motor);

    template <class MotorType, int N>
    Motor_CAN_COB &MotorMsgPack(Motor_CAN_COB &motor_msg, MotorType (&motors)[N]);

    template <class MotorType, class... MotorTypes>
    Motor_CAN_COB &MotorMsgPack(Motor_CAN_COB &motor_msg, MotorType &motor, MotorTypes &...motors);
}

/**
 * @brief 单个dji电机can发送函数
 * @tparam MotorType
 * @param CanQueueHander 将电机can包打包进消息队列
 * @param motor dji电机类
 */
template <class MotorType>
void motor_dji::MotorMsgSend(QueueHandle_t CanQueueHander, MotorType &motor)
{
    MotorType motor_arr[1] = {motor};
    motor_dji::MotorMsgSend(CanQueueHander, motor_arr);
}

/**
 * @brief 将电机数据整合入电机can通信数据包
 * @param &motor_msg	电机can通信数据包
 * @param (&motors)[N]	电机类(数组）
 * @return motor_msg	电机can通信数据包
 */
template <class MotorType, int N>
Motor_CAN_COB &motor_dji::MotorMsgPack(Motor_CAN_COB &motor_msg, MotorType (&motors)[N])
{
    for (int i = 0; i < N; i++)
    {
        motor_msg = motor_dji::MotorMsgPack(motor_msg, motors[i]);
    }
    return motor_msg;
}

/**
 * @brief 将电机数据整合入电机can通信数据包
 * @param &motor_msg    电机can通信数据包
 * @param &motor		电机类
 * @param &...motors	电机类（可变参数）
 * @return motor_msg	电机can通信数据包
 */
template <class MotorType, class... MotorTypes>
Motor_CAN_COB &motor_dji::MotorMsgPack(Motor_CAN_COB &motor_msg, MotorType &motor, MotorTypes &...motors)
{
    motor_msg = motor_dji::MotorMsgPack(motor_msg, motor);
    return motor_dji::MotorMsgPack(motor_msg, motors...);
}

/**
 * @brief 将电机数据整合入电机can通信数据包
 * @param &motor_msg	电机can通信数据包
 * @param &motor		电机类
 * @return motor_msg	电机can通信数据包
 */
template <class MotorType>
Motor_CAN_COB &motor_dji::MotorMsgPack(Motor_CAN_COB &motor_msg, MotorType &motor)
{
    motor.Out = std_lib::constrain(motor.Out, -motor.MAX_CURRENT(), motor.MAX_CURRENT());
    int16_t constrain_Out = motor.Out;
    if (motor.ID <= 4 && motor.ID > 0)
    {
        if (motor.SEND_ID_LOW() == motor_msg.Id200.ID)
        {
            motor_msg.Id200.Data[motor.ID * 2 - 2] = (constrain_Out >> 8) & 0xff;
            motor_msg.Id200.Data[motor.ID * 2 - 1] = constrain_Out & 0xff;
        }
        else if (motor.SEND_ID_LOW() == motor_msg.Id1ff.ID)
        {
            motor_msg.Id1ff.Data[motor.ID * 2 - 2] = (constrain_Out >> 8) & 0xff;
            motor_msg.Id1ff.Data[motor.ID * 2 - 1] = constrain_Out & 0xff;
        }
    }
    else if (motor.ID <= 8 && motor.ID > 4)
    {
        if (motor.SEND_ID_HIGH() == motor_msg.Id1ff.ID)
        {
            motor_msg.Id1ff.Data[motor.ID * 2 - 10] = (constrain_Out >> 8) & 0xff;
            motor_msg.Id1ff.Data[motor.ID * 2 - 9] = constrain_Out & 0xff;
        }
        else if (motor.SEND_ID_HIGH() == motor_msg.Id2ff.ID)
        {
            motor_msg.Id2ff.Data[motor.ID * 2 - 10] = (constrain_Out >> 8) & 0xff;
            motor_msg.Id2ff.Data[motor.ID * 2 - 9] = constrain_Out & 0xff;
        }
    }
    return motor_msg;
}

//添加对达妙电机的支持
template <>
inline Motor_CAN_COB &motor_dji::MotorMsgPack(Motor_CAN_COB &motor_msg, Motor_DM_classdef &motor)
{
    // 电机ID=2，对应一拖四控制帧的第 3,4 字节
    int16_t constrain_Out = (int16_t)std_lib::constrain(motor.Out, -10000.0f, 10000.0f);
    /*todo
    song
    根据实际量程调整
    */ 
    
    // 映射逻辑：ID 1->Data[0-1], ID 2->Data[2-3], ID 3->Data[4-5], ID 4->Data[6-7]
    if (motor.ID >= 1 && motor.ID <= 4) {
        motor_msg.Id200.Data[motor.ID * 2 - 2] = (constrain_Out >> 8) & 0xff;
        motor_msg.Id200.Data[motor.ID * 2 - 1] = constrain_Out & 0xff;
    }
    return motor_msg;
}

#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
