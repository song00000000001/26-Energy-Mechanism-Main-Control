/**
 ******************************************************************************
 * @file   System_DataPool.cpp
 * @brief  All used resources are contained in this file.
 ******************************************************************************
 * @note
 *  - User can define datas including variables ,structs ,and arrays in
 *    this file, which are used in deffrient tasks or services.
 **/
#include "internal.h"

/* RTOS Resources ------------------------------------------------------------*/
/* Queues */
QueueHandle_t USART_TxPort; //	串口发送队列
QueueHandle_t USART_RxPort; // 串口接收队列
QueueHandle_t Param_RxPort;  //调参板接收队列  
QueueHandle_t CAN1_TxPort;  //	can1 发送队列
QueueHandle_t CAN1_RxPort;  //	can1 接收队列
QueueHandle_t CAN2_TxPort;  //	can2 发送队列
QueueHandle_t CAN2_RxPort;  //	can2 接收队列
#if USE_SRML_DR16
QueueHandle_t DR16_QueueHandle; //	dr16（串口） 接收队列
#endif
/* Semaphores */

/* Mutexes */
#if USE_SRML_DR16
SemaphoreHandle_t DR16_mutex; //	dr16互斥量
#endif

/* Notifications */

/* Other Resources -----------------------------------------------------------*/
#if USE_SRML_VIRTUAL_COM
uint8_t VirtualCom_Rx_Buff[VIRTUALCOM_RX_BUFFER_SIZE];
#endif

#if USE_SRML_MPU6050
__CCM mpu_rec_s mpu_receive; // mpu6050数据
#endif

#if USE_SRML_DR16
__CCM DR16_Classdef DR16; // 遥控器DR16类
#endif

#if USE_SRML_FS_I6X
__CCM FS_I6X_Classdef remote;
#endif // !USE_SRML_FS_I6X

#if USE_SRML_REFEREE
__SRAM referee_Classdef Referee;
#endif
#if USE_SRML_BMI088
/** TODO:  25赛季完全删除，停止对第二代解算库支持*/
// /** BMI088传感器驱动模块 */
// BMI088SensorClassdef BMI088 = BMI088SensorClassdef(&hspi1, BMI088_ACC_NSS_GPIO_Port, BMI088_ACC_NSS_Pin, BMI088_GYRO_NSS_GPIO_Port, BMI088_GYRO_NSS_Pin);
// /** BMI088解算模块 */
// MahonyFilterClassdef BMI088_AHRS;
#endif

DartDataStructdef DartsData[MAX_DART_DATAPOOL_SIZE]; //发射数据
uint8_t DartDataSlot[5]={0,1,2,3,4};
DartAimEnumdef HitTarget;
uint8_t ParamSendPack[9];
uint8_t CurrentCnt=1;
float Yaw_Angle[2];
float _YawCorrectionAngle;//yaw轴修正角
Motor_GM6020 loadermotor[1]{5};//装填电机
Launch_Classdef Launch(ID_DELIVER_R, ID_DELIVER_L, ID_IGNITER); //发射类
Missle_YawController_Classdef Yaw(ID_YAW);//yaw控制类
VisionRecvData_t vision_recv_pack;//电视接收包
VisionSendData_t vision_send_pack;//电视发送包
uint32_t vision_last_recv_time = 0;

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
