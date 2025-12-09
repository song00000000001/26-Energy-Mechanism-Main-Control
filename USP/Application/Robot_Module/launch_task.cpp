#include "internal.h"
#include "global_data.h"
#include "launcher_driver.h"
#include "robot_config.h"

/*摇杆逻辑
markdown:

| 摇杆  | 状态     | 动作                                                                 |     | 摇杆  | 状态         | 动作                             |
| --- | ------ | ------------------------------------------------------------------ | --- | --- | ---------- | ------------------------------ |
| s1  | up     | 电机失能（不修改状态）                                                        |     | s2  | up         | 手动控制yaw和igniter                |
|     | middle | 进自检，自检手动完成或者跳过自动进校准，校准完后进等待发射，（卡stay状态，后续的autofire状态会因此被重置到stay状态） |     |     | middle     | 使用修正值控制yaw和igniter（修正值由调参板设置）  |
|     | down   | 激活自动发射，（进autofireprep状态），跳过自检（自检就是手动按限位开关）                         |     |     | down       | 使用视觉控制，如果视觉失联，转而用修正值控制，直到视觉重连。 |
| LX  |        |                                                                    |     | RX  | left/right | 增量控制yaw                        |
| LY  |        |                                                                    |     | RY  | up/donw    | 增量控制igniter                    |
s1要是多个状态就可以在自检完后进idle空闲而不是直接开始校准（电机会直接开转），不然就要事先在自检前失能，检完后使能（middle)，然后打down自动发射。
todo:
1. 改发射条件,需要手动掰LY到底发射,并且向左向右选择连发模式2/4发。连发模式还没写。可以写在发射机里。
2.

date:2025/12/10/0:47
*/



/*发射主控任务*/
void LaunchCtrl(void *arg)
{
    //can发送的包
    Motor_CAN_COB Tx_Buff,Tx_Buff1;
 
    // 初始状态设为自检
    Robot.Status.current_state = SYS_CHECKING;
    // 初始自检标志位失能
	Robot.Flag.Check.limit_sw_ok=false;
    //跳过自检标志位失能,只会在s1下时生效,并且会在s1为中重置
    Robot.Cmd.skip_check=false;

    Debugger.enable_debug_mode=false;

    //校准速度初始化
    calibration_speed={
	.yaw_calibration_speed=-600,
	.deliver_calibration_speed=6000,
    .igniter_calibration_speed=-2000
    };

    // 任务频率控制
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
		


        if (DR16.GetStatus() != DR16_ESTABLISHED) {
            Robot.Status.current_state = SYS_OFFLINE;
        }
        else{
            // Debug 模式判定 (最高优先级的主动模式)
            // 只有当遥控器连接，且全局 Debug 标志位被置 1 时进入，并且校准完成。
            if (Debugger.enable_debug_mode&&Robot.Flag.Status.is_calibrated) {
                Robot.Status.current_state = SYS_DEBUG;
            }
            else {
                // 如果之前是 Debug，现在退出了，并且手动失能，则回 Checking 状态
                if (Robot.Status.current_state == SYS_DEBUG && !Robot.Cmd.sys_enable) {
                    // 安全起见重新自检
                    Launcher.check_progress=0; // 重置自检进度
                    Robot.Status.current_state = SYS_CHECKING;
                    //为了防止自己跳过自检（电机速度角度环状态只在校准后才会切换角度环，而debug中可能会改成速度环然后退出，那么后续就不会进入角度环模式，那就不行）
                    
                    Launcher.mode_deliver[1] = MODE_ANGLE;
                }
            }

            // 处理遥控器开关逻辑
            // 手动失能开关 S1向上
            if(DR16.GetS1()==SW_UP){
                Robot.Cmd.sys_enable=false;
            }
            else{
                Robot.Cmd.sys_enable=true;
            }
            //自动发射模式,跳过自检 S1向下
            if(DR16.GetS1()==SW_DOWN){
                Robot.Cmd.skip_check=true;
                Robot.Cmd.auto_mode=true;
            }
            //s1中则是自动发射关闭,会卡在待机状态,并且不会跳过自检
            if(DR16.GetS1()==SW_MID){
                Robot.Cmd.auto_mode=false;
                Robot.Cmd.skip_check=false;
            }
            //s2
            if(DR16.GetS2()==SW_UP){
                Robot.Cmd.manual_override=true;
                //Robot.Status.yaw_control_state = MANUAL_AIM; //切换到手动瞄准
                //这里无需切换,保持即可,不然会抢占校准状态,校准完后,主状态机自动切换到手动模式
            }
            else if(DR16.GetS2()==SW_MID){
                Robot.Cmd.manual_override=false;
                Robot.Status.yaw_control_state = CORRECT_AIM; //切换到修正值瞄准
            }
            else{
                Robot.Cmd.manual_override=false;
                Robot.Status.yaw_control_state = VISION_AIM; //切换到视觉瞄准
            }
            // 手动微调 igniter
            if (Robot.Cmd.manual_override) {
                // 手动微调逻辑
                // 读取当前角度 + 摇杆增量
                float new_igniter_pos = Launcher.IgniterMotor.getMotorTotalAngle()+ DR16.Get_LY_Norm() * 0.002f;
                //将计算结果传给驱动
                Launcher.target_igniter_angle=new_igniter_pos;
                
                Yawer.yaw_target -= DR16.Get_RX_Norm() * 0.002f;
                Yawer.yaw_target = std_lib::constrain(Yawer.yaw_target, -10.2f, 10.2f);
            }
        }
        /*在非校准状态如果发生碰撞限位的现象，则立即取消使能并且记录错误电机信息，
        并且重置为error状态，此时会将该电机的校准状态重置,  
        需要重新进行限位校准，此时如果拨右摇杆朝下则会反向旋转对应电机，
        拨左摇杆取消使能则进入check状态，
        */
		
        if(Robot.Status.current_state != SYS_CALIBRATING){
            if(SW_IGNITER_OFF||SW_YAW_L_OFF||SW_YAW_R_OFF||SW_DELIVER_L_OFF||SW_DELIVER_R_OFF){
                Robot.Status.current_state = SYS_ERROR; // 进入错误状态
            }
        } 
        
       
        switch (Robot.Status.current_state)
        {
		case SYS_ERROR:
		{
            // 恢复条件：手动失能
            if (DR16.GetStatus() == DR16_ESTABLISHED&& !Robot.Cmd.sys_enable) {
                
                Robot.Status.current_state = SYS_CHECKING;
            }
		}  
		break;
		
        case SYS_DEBUG:
        {

        }
        break;
			
        case SYS_OFFLINE:
        {
            // 恢复条件：遥控器重连
            if (DR16.GetStatus() == DR16_ESTABLISHED) {
                Robot.Status.current_state = SYS_CHECKING;
            }
        }
        break;
        case SYS_CHECKING:
        {
            //按键自检逻辑
            Launcher.key_check();
            // 3. 处理跳过逻辑
            if (Robot.Cmd.skip_check) {
                Launcher.check_progress = MASK_ALL_PASSED; // 强制全满
            }

            // 4. 更新全局标志位
            Robot.Flag.Check.limit_sw_ok = (Launcher.check_progress == MASK_ALL_PASSED);
   
            // 5个按键手动检查全部通过则进入校准状态,后续可以加入电机检查
            if (Robot.Flag.Check.limit_sw_ok) {
                Robot.Status.current_state = SYS_CALIBRATING;
                Launcher.start_calibration();
            }
        }
        break;
        
        case SYS_CALIBRATING:
            // 此状态下，Launcher.adjust() 内部正在跑归零逻辑，任务层只需要等待驱动层反馈 "已校准"
            //yaw控制考虑到是并行的，主状态机和子状态机采用状态位判断
            Robot.Status.yaw_control_state = YAW_CALIBRATING;
            if(Yawer.is_Yaw_Init()){
                Robot.Status.yaw_control_state = MANUAL_AIM; //校准完成后，进入手动模式
                Yawer.yaw_target=0;
            }
            // 1. 处理归零状态转换
            Launcher.check_calibration_logic();
            Robot.Flag.Status.is_calibrated=Yawer.is_Yaw_Init()&&Launcher.is_calibrated();
            // 2. 全部校准完毕后，切换到待机状态
            //校准完毕跳转待机状态
            if (Robot.Flag.Status.is_calibrated) {
                Robot.Status.current_state = SYS_STANDBY;
            }
            break;

        case SYS_STANDBY:
			 
            // 切换到自动模式
            if (Robot.Cmd.auto_mode) {
                Launcher.fire_state = FIRE_IDLE; // 重置发射子状态机
                Robot.Status.current_state = SYS_AUTO_PREP;
            }
            break;
            
        case SYS_AUTO_PREP:
            // --- 自动发射准备 ---
            // 确保机构归位到待发状态
			Launcher.target_igniter_angle=POS_BUFFER; // 回缓冲
            Launcher.target_igniter_angle=POS_IGNITER;  // 去瞄准默认高度(示例)
            
            // 检查是否到位
            if (Launcher.is_deliver_at_target() && Launcher.is_igniter_at_target()) 
            {  
                
                Robot.Status.current_state = SYS_AUTO_FIRE;
            }
            
            // 随时允许切回手动
            if (!Robot.Cmd.auto_mode) Robot.Status.current_state = SYS_STANDBY;
            break;
            
        case SYS_AUTO_FIRE:
            Launcher.Run_Firing_Sequence();
            // 随时允许切回手动 (Run_Firing_Sequence 内部也会处理打断复位)
            if (!Robot.Cmd.auto_mode) Robot.Status.current_state = SYS_STANDBY;
            break;
        }

        //yaw轴子状态机,包含状态如下
        /*
            manual_aim:手动瞄准
            vision_aim:视觉瞄准
            correct_aim:修正值瞄准
            disable_motor:失能电机
            yaw_calibrating:校准模式
        */
        Yawer.yaw_state_machine(Robot.Status.yaw_control_state);

        if (Robot.Status.current_state != SYS_OFFLINE && 
            Robot.Status.current_state != SYS_CHECKING&&
            Robot.Status.current_state != SYS_ERROR &&
            Robot.Cmd.sys_enable
        ) 
        {
            //计算 PID
            Launcher.adjust();
            Yawer.adjust();
            //输出电流
            Launcher.out_all_motor_speed();
            Yawer.yaw_out_motor_speed();
        }
        else
        {
            //停止电机
            Launcher.stop_all_motor();
            Yawer.disable();
        }
        
        MotorMsgPack(Tx_Buff1, Yawer.YawMotor);
        xQueueSend(CAN2_TxPort, &Tx_Buff1.Id1ff, 0);
        //R0L1
        MotorMsgPack(Tx_Buff, Launcher.DeliverMotor[1], Launcher.DeliverMotor[0], Launcher.IgniterMotor);
		xQueueSend(CAN1_TxPort, &Tx_Buff.Id200, 0);

    }
}


