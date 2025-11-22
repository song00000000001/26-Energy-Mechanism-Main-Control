/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file    MF9025_v2.h
 * @author  余俊晖 (2460857175@qq.com)
 * @brief   Header file for Motor-MF9025V2 driver
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
#ifndef _MF9025_V2_H
#define _MF9025_V2_H

/* Includes ------------------------------------------------------------------*/
#include "srml_std_lib.h"
#include "Drivers/Components/drv_can.h"
#include "Middlewares/Algorithm/MultiTurnRecorder/MultiTurnRecorder.h"
#include "FreeRTOS.h"
#include "queue.h"

#ifdef __cplusplus
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/

class MotorMF9025v2Classdef : MultiTurnRecorderClassdef
{
private:
    static constexpr float RATIO_RECEIVE_VOLTAGE = 0.1f;
    static constexpr float RATIO_RECEIVE_ANGLE = 0.01f;
    static constexpr float RATIO_ENCODER_TO_ANGLE = 360 / 65536.f;
    static constexpr float RATIO_CURRENT_TO_A = 16.5f / 2048.f;
    //16T是0.32f
    static constexpr float TORQUE_CONSTANT = 0.32f; // 扭矩常数
    // 35T是0.81f
    // static constexpr float TORQUE_CONSTANT = 0.81f;
    struct DataPoolStructdef
    {
        int32_t accel = NULL;          // 从电机返回来的加速度
        uint16_t encoder = NULL;       // 从电机返回来的编码器值（减去offset后的）
        uint16_t encoderRaw = NULL;    // 从电机返回来的编码器值（没有减去offset）
        uint16_t encoderOffset = NULL; // 电机返回来的编码器offset
        float speed = NULL;            // 转速，单位为RPM
        float singleAngle = NULL;      // 单圈角度，单位为°
        float totalAngle = NULL;       // 多圈总角度，单位为°
        float totalAngleLocal = 0;     // 本地计算的多圈总角度
        int32_t round_cnt = 0;         // 本地计算的已转圈数

        int8_t iqPidKp = NULL;    // 从电机返回来的力矩环kp
        int8_t iqPidKi = NULL;    // 从电机返回来的力矩环ki
        int8_t speedPidKp = NULL; // 从电机返回来的速度环kp
        int8_t speedPidKi = NULL; // 从电机返回来的速度环ki
        int8_t anglePidKp = NULL; // 从电机返回来的角度环kp
        int8_t anglePidKi = NULL; // 从电机返回来的角度环ki

        float current = NULL;      // 单位为A(安培)
        int8_t temperature = NULL; // 单位为℃
        float voltage = NULL;      // 从电机返回来的电压，单位为V
        float iA = NULL;           // A相电流，单位为A(安培)
        float iB = NULL;           // B相电流，单位为A(安培)
        float iC = NULL;           // C相电流，单位为A(安培)
        uint8_t errorState = NULL; // 错误状态

        uint8_t motorId = NULL;       // 电机id
        int16_t targetCurrent = NULL; // 多电机模式下发送的期望电流
    } dataPool;
    float last_angle = 0;
    QueueHandle_t sendQueue = NULL;
    CAN_COB canSendStruct;

public:
    MotorMF9025v2Classdef(uint8_t _id);
    MotorMF9025v2Classdef(bool id1, bool id2, bool id3);
    void init(QueueHandle_t _sendQueue);
    //  使用方法：变量名.getData().变量，例如：motor.getData().totalAngleLocal;
    //  此方法可以保障外部无法修改内部变量，但可以获取
    //  "const DataPoolStructdef &" 含义为“返回DataPoolStructdef类型的常量引用”
    const DataPoolStructdef &getData() { return dataPool; }

    //  此函数用于修改内部变量，是否使用有使用者决定,例如motor.setData().iqPidKp = 1;
    //  因为该电机类中的变量基本用于存储电机返回来的数据，故理论上应该只被读取，不被外部修改，故先注释掉，使用者决定是否使用
    //  DataPoolStructdef & setData() { return dataPool; }

    void setEncoderOffset(uint16_t offset); // 设置软件计算的编码器Offset

    uint16_t getComId() const { return REC_ID_INIT() + dataPool.motorId; }
    int16_t MAX_CURRENT() const { return 2048; }
    bool update(uint32_t canRecID, uint8_t can_rx_data[]); // 电机返回值更新函数

    void setTatgetCurrent(int16_t _current); //  设置目标电流，多电机控制模式下使用
    /*---读取函数---*/
    void readPid();                    // 读取pid参数
    void readAccel();                  // 读取角加速度
    void readEncoder();                // 读取编码器值
    void readTotalAngle();             // 读取多圈角度
    void readSingleAngle();            // 读取单圈绝对角度
    void readMotorState1_errorState(); // 读取电机状态和错误标志
    void readMotorState2();
    void readMotorState3();
    /*---写入函数---*/
    // 将pid参数写入RAM中
    void writePidToRAM(int8_t _iqKp, int8_t _iqKi, int8_t _angleKp, int8_t _angleKi, int8_t _speedKp, int8_t _speedKi);
    // 将pid参数写入ROM中
    void writePidToROM(int8_t _iqKp, int8_t _iqKi, int8_t _angleKp, int8_t _angleKi, int8_t _speedKp, int8_t _speedKi);
    void writeEncoderOffset(uint16_t _offset); // 设置编码器值offset
    void writeNowEncoderAsOffset();            // 设置当前编码器值为offset
    void writeAccelToRAM(int32_t _accel);      // 写入加速度到RAM
    /*---控制函数---*/
    void cleanErrorState();                                              // 清除电机错误标志命令，需要先让电机恢复正常
    void closeMotor();                                                   // 关闭电机，清除运行状态和接收过的控制指令
    void stopMotor();                                                    // 停止电机，不清除运行状态和接收过的控制指令
    void startMotor();                                                   // 停止电机后，恢复电机运行
    void iqCloseControl(float _targetTorque);                            // 闭环力矩控制
    void iqCloseControl_Current(int16_t _targetCurrent);                 // 闭环力矩控制，输入目标电流值，主要用于速度环在MCU上运算时，向电机下发控制量，但是封库时测试用起来效果并不佳
    void speedControl(float _targetRPM);                                 // 速度闭环控制
    void angleTotalControl_1(float _targetAngle);                        // 多圈角度闭环控制模式1
    void angleTotalControl_2(float _targetAngle, float _speedLimit);     // 多圈角度闭环控制模式2
    void angleSingleControl_1(float _targetAngle);                       // 单圈角度闭环控制模式1
    void angleSingleControl_2(float _targetAngle, float _speedLimit);    // 单圈角度闭环控制模式2
    void angleIncrementControl_1(float _targetAngle);                    // 角度增量闭环控制模式1
    void angleIncrementControl_2(float _targetAngle, float _speedLimit); // 角度增量闭环控制模式2

private:
    uint16_t REC_ID_INIT() const { return 0x140; }
    void updatePid(uint8_t can_rx_data[]);
    void updateEncoder(uint8_t can_rx_data[]);
    void updateEncoderOffset(uint8_t can_rx_data[]);
    void updateSingleAngle(uint8_t can_rx_data[]);
    void updateTotalAngle(uint8_t can_rx_data[]);
    void updateAccel(uint8_t can_rx_data[]);
    void updateMotorState1(uint8_t can_rx_data[]);
    void updateMotorState2(uint8_t can_rx_data[]);
    void updateMotorState3(uint8_t can_rx_data[]);

    inline void sendData()
    {
        if (sendQueue != NULL)
            xQueueSend(sendQueue, &canSendStruct, 0);
    };
    bool angle_offset_state = false;
};

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported function declarations --------------------------------------------*/

CAN_COB &MotorMsgPack(CAN_COB &motor_msg, MotorMF9025v2Classdef &motor);

/*
 * @brief 	将电机数据整合入电机can通信数据包
 * @param		&motor_msg			电机can通信数据包
 * @param 	&motor					电机类
 * @param		&...motors			电机类（可变参数）
 * @return 	motor_msg				电机can通信数据包
 */
template <class... MotorTypes>
CAN_COB &MotorMsgPack(CAN_COB &motor_msg, MotorMF9025v2Classdef &motor, MotorTypes &...motors)
{
    motor_msg = MotorMsgPack(motor_msg, motor);
    return MotorMsgPack(motor_msg, motors...);
}
#endif

#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB**************************/
