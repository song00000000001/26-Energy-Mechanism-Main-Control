#include "internal.h"
#include "global_data.h"


/**
 * @brief 发射主控任务
 * @parma None
 * @return None
 */

 void LaunchCtrl(void *arg)
 {
     Motor_CAN_COB Tx_Buff;
     Motor_CAN_COB Tx_Buff1;
     TickType_t xLastWakeTime_t;
     xLastWakeTime_t = xTaskGetTickCount();
     
     for (;;)
     {
         vTaskDelay(1);
         if (DR16.GetStatus() == DR16_ESTABLISHED)
         {
             if (DR16.GetS1() != SW_DOWN || vision_aim_state == 1) // 视觉调整yaw时不允许发射
             {
                 switch (state)
                 {
                 case DEINIT: // 初始化
                     Launch.Igniter_On();
                     Launch.Deliver_Init(); // 左右滑块
                     Launch.Igniter_Init(); // 扳机丝杆
                     if (Launch.Igniter_Init_flag == 1 && Launch.is_Deliver_Init() == 1)
                     {
                         state = WAIT_ACT;
                     }
                     break;
                 case WAIT_ACT: // 等待蓄力指令,并且丝杆移动到指定位置
                     Launch.Deliver_Init();
                     Launch.Igniter_On();
                     if (cnt == 0)
                     {
                         status++;
                         cnt = 1; // 使status只加一次
                     }
                     if (DR16.GetS1() == SW_MID || DR16.GetS1() == SW_DOWN) // 丝杆数据用调参板数据
                     {
                         igniter_goal_pos = DartsData[status - 1].Ignitergoal[HitTarget];
                         igniter_goal_pos = std_lib::constrain(igniter_goal_pos, 3.f, 260.f);
                         Launch.Igniter_Pos_Set(igniter_target_pos);
                     }
                     if (DR16.GetS2() == SW_DOWN && Launch.PID_Igniter_Angle.Error < 0.1) // 右边拨杆下拨且丝杆移动到位后，滑块拉下，扳机扣下
                     {
                         state = PULL;
                     }
                     break;
 
                 case PULL:
                     if (Launch.Pull_Ready_flag != 1)
                     {
                         Launch.Igniter_Off();
                         Launch.Deliver_Pull();
                     }
                     else
                     {
                         Launch.Igniter_Off(); // 扳机扣下
                         vTaskDelay(300);
                         state = BACK;
                     }
                     break;
                 case BACK:
                     Launch.Igniter_Off();
                     Launch.Deliver_Init();
                     open = 1; // 小风车强制切换到发射状态
                     goal = 2750;
                     if (Launch.is_Deliver_Init() == 1)
                     {
                         state = WAIT_SHOOT;
                     }
                     break;
                 case WAIT_SHOOT: // 进入待发射状态
                     Launch.Deliver_Init();
                     if (DR16.GetS2() == SW_UP) // 扳机释放，发射！
                     {
                         Launch.Igniter_On();
                         state = WAIT_ACT; // 回到等待指令状态，准备下一次发射
                         vTaskDelay(2000);
                         open = 0;
                         cnt = 0; // 更新装填状态，准备装填下一发
                     }
                     else
                     {
                         Launch.Igniter_Off();
                     }
                     break;
 
                 default:
                     break;
                 }
             }
         }
         /*丝杆电机进入调试模式，遥控可控制丝杆的前后*/
         if (state != PULL && DR16.GetS1() == SW_UP && Launch.Igniter_Init_flag == 1) // 左拨杆朝上，进入调试模式
         {
             igniter_target_pos += DR16.Get_RY_Norm() * 0.03f;
             igniter_target_pos = std_lib::constrain(igniter_target_pos, 0.f, 260.f);
             Launch.Igniter_Pos_Set(igniter_target_pos);
         }
         Launch.adjust();
         /*关控保护*/
         if (DR16.GetStatus() != DR16_ESTABLISHED)
         {
             Launch.disable();
             Yaw.disable();
             loadermotor[0].Out = 0;
         }
         /*切换目标*/
         if (DR16.Get_RY_Norm() < -0.9f)
         {
             if (DR16.Get_RX_Norm() < -0.9f)
             {
                 vTaskDelay(1000);
                 HitTarget = DR16.Get_RX_Norm() < -0.9f ? Outpost : HitTarget;
             }
             else if (DR16.Get_RX_Norm() > 0.9f)
             {
                 vTaskDelay(1000);
                 HitTarget = DR16.Get_RX_Norm() > 0.9f ? Base : HitTarget;
             }
         }
         /*打包数据发送*/
         MotorMsgPack(Tx_Buff, Launch.DeliverMotor[L], Launch.DeliverMotor[R], Launch.IgniterMotor);
         xQueueSend(CAN2_TxPort, &Tx_Buff.Id200, 0);
     }
 }