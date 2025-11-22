/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file   : referee.h
 * @author : Lingzi_Xie 1357657340@qq.com
 * @brief  : Code for communicating with Referee system of Robomaster 2021.
 ******************************************************************************
 * @attention
 *
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version NO., write dowm your name and the date, the most
 * important is make sure the users will have clear and definite understanding
 * through your new brief.
 ******************************************************************************
 */
#ifndef __REFEREE_H__
#define __REFEREE_H__

#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "srml_std_lib.h"
#include "Drivers/Components/SRML_Timer.h"

#include "referee_ui.h"		// 存放UI所需要的信息
#include "referee_struct.h" // 存放各种struct的定义
/* Private define ------------------------------------------------------------*/

#define DRAWING_PACK 15
#define RF_TIMEOUT_TIME 40

/* Private variables ---------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Exported ------------------------------------------------------------------*/
/* 裁判系统状态；0：掉线，1：在线 */
enum RF_status_e
{
	RF_OFFLINE = 0U,
	RF_ONLINE
};

/* 枚举变量，用于车间通信数组访问 */
enum
{
	HERO = 0U,
	ENGINEER,
	INFANTRY_3,
	INFANTRY_4,
	INFANTRY_5,
	AERIAL,
	SENTRY,
	RADAR = 8U
};

enum
{
	Robot_Red = 0U,
	Robot_Blue
};

/* ----------------------------------各种数据类型对应的ID号--------------------------------------- */
typedef enum
{
	/* 裁判系统发过来数据帧ID类型，注意这里是CMD_ID */
	GameState_ID = 0x0001,				// 比赛状态数据 1Hz
	GameResult_ID = 0x0002,				// 比赛结果数据 结束后发送
	GameRobotHP_ID = 0x0003,			// 机器人血量数据 1Hz
	DartStatus_ID = 0x0004,				// 【05.06的协议中移除了】飞镖发射状态数据
	ICRA_DebuffStatus_ID = 0x0005,		// 人工智能挑战赛加成与惩罚状态 1Hz
	EventData_ID = 0x0101,				// 场地事件数据 1Hz
	SupplyProjectileAction_ID = 0x0102, // 补给站动作标识 动作后发送

	RefereeWarning_ID = 0x0104,		 // 裁判警告数据 警告后发送
	DartRemainingTime_ID = 0x0105,	 // 飞镖发射口倒计时 1Hz
	GameRobotState_ID = 0x0201,		 // 机器人状态 10Hz
	PowerHeatData_ID = 0x0202,		 // 功率热量数据 50Hz
	GameRobotPos_ID = 0x0203,		 // 机器人位置数据 10Hz
	BuffMusk_ID = 0x0204,			 // 机器人增益数据 状态改变后发送
	AerialRobotEnergy_ID = 0x0205,	 // 空中机器人能量 10Hz。只有空中机器人发送
	RobotHurt_ID = 0x0206,			 // 伤害数据 伤害发生后发送
	ShootData_ID = 0x0207,			 // 实时射击数据 发射后发送
	BulletRemaining_ID = 0x0208,	 // 子弹剩余发送数 地面机器人及空中 10Hz
	RFID_Status_ID = 0x0209,		 // RFID状态 1Hz
	ExtDartClientCmd_ID = 0x020A,	 // 飞镖机器人客户端发送指令 10Hz
	GroundRobotPosition_ID = 0x020B, // 地面机器人位置 哨兵机器人发送 1Hz
	RadarMarkProgress_ID = 0x020C,	 // 雷达标记进度数据，向雷达发送，以 1Hz 频率发送 常规链路

	SentryAutonomousDecision_ID = 0x020D, // 哨兵自主决策信息同步 1Hz
	RadarSync_ID = 0x020E,				  // 雷达自主决策信息同步 1Hz

	StudentInteractiveHeaderData_ID = 0x0301, // 机器人交互数据 30Hz上限
	CustomControllerData_ID = 0x0302,		  // 自定义控制器交互数据接口 30Hz 图传链路
	MiniMapInteractiveData_ID = 0x0303,		  // 小地图交互数据 触发时发送
	VT_Data_ID = 0x0304,					  // 键盘、鼠标数据，通过图传发送
	ClientMapCommand_ID = 0x0305,			  // 小地图接收信息
	CustomClientData_ID = 0x0306,			  // 模拟键鼠操作选手端
	SentryPath_ID = 0x0307,					  // 哨兵向空中机器人选手端发送路径坐标数据
	Custom_info_ID = 0x0308,				  // 己方机器人向己方任意选手端发送自定义的消息，该消息会在己方选手端特定位置显示

	/* 机器人交互数据段的ID类型，注意这里是数据段内容ID！ */
	RobotComData_ID = 0x0233, // 车间交互，该ID由各个队伍自定义
	CustomData_ID = 0xD180,	  // 自定义数据ID
	Drawing_Clean_ID = 0x0100,
	Drawing_1_ID = 0x0101,
	Drawing_2_ID = 0x0102,
	Drawing_5_ID = 0x0103,
	Drawing_7_ID = 0x0104,
	Drawing_Char_ID = 0x0110,

	SentryDecision_ID = 0x0120,
	RadarDecision_ID = 0x0121,
} RefereeSystemID_t;

/* 交互数据帧的通信类型，机器人间通信 or 自定义UI or 小地图通信 */
typedef enum
{
	CV_OtherRobot = 0U,
	UI_Client,
	MiniMap_Client
} receive_Type_e;

/* ----------------------------------裁判系统串口通信类--------------------------------------- */
class referee_Classdef
{
public:
	/* 机器人ID */
	RC_ID robot_client_ID;

	/* 裁判系统连接状态 */
	RF_status_e status;
	uint32_t LastPackReceiveTime; // 上一次收到包的时候
	/* 比赛接收数据 */
	ext_game_status_t GameState;				 // 比赛状态数据
	ext_game_result_t GameResult;				 // 比赛结果数据
	ext_game_robot_HP_t GameRobotHP;			 // 机器人血量数据
	ext_dart_status_t DartStatus;				 // 飞镖状态数据
	ext_event_data_t EventData;					 // 场地事件数据
	ext_supply_projectile_action_t SupplyAction; // 补给站动作数据
	ext_referee_warning_t RefereeWarning;		 // 裁判警告信息数据
	ext_dart_remaining_time_t DartRemainTime;	 // 飞镖发射倒计时数据
	ext_game_robot_status_t GameRobotState;		 // 机器人状态：当前血量、射速、底盘功率等
	ext_power_heat_data_t PowerHeatData;		 // 机器人功率、热量数据
	ext_game_robot_pos_t RobotPos;				 // 机器人位置数据
	ext_buff_t RobotBuff;						 // 机器人增益数据
	aerial_robot_energy_t AerialEnergy;			 // 空中机器人能量状态数据
	ext_robot_hurt_t RobotHurt;					 // 机器人伤害状态数据
	ext_shoot_data_t ShootData;					 // 实时射击信息数据
	ext_bullet_remaining_t BulletRemaining;		 // 剩余弹丸量数据
	ext_rfid_status_t RFID_Status;				 // RFID状态数据
	ext_dart_client_cmd_t DartClientCmd;		 // 飞镖机器人客户端指令数据
	ground_robot_position_t GroundRobotPosition; // 地面机器人位置数据
	radar_mark_data_t RadarMarkProgress;		 // 雷达标记进度
	ext_VT_command_t VT_Data;
	ext_client_map_command_t ClientMapCommand;
	custom_client_data_t CustomClientData;
	map_sentry_data_t SentryPath;
	map_custom_data_t custom_info;
	sentry_info_t SentryAutonomousDecision;
	radar_info_t RadarSync;
	/* 交互数据 */
	my_interactive_data_t robot_rec_data[9];				  // 存储本方其他机器人发送的信息，添加了雷达站
	custom_controller_interactive_data_t custom_control_data; // 自定义控制器数据段部分
	ext_mini_map_command_t mini_map_data;					  // 小地图下发信息
	ext_VT_command_t VT_command_data;						  // 键鼠信息，通过图传发送端接收
	ext_client_map_command_t radar_map_data;				  // 雷达站信息

	referee_Classdef() {} // 构造函数定义

	void Init(UART_HandleTypeDef *_huart, SystemTick_Fun getTick_fun);									// 初始化学生串口
	void unPackDataFromRF(uint8_t *data_buf, uint32_t length);											// 数据帧整帧解包
	void CV_ToOtherRobot(uint8_t target_id, uint8_t *data1, uint8_t length);							// 机器人之间车间通信
	void Radar_dataTransmit(uint8_t target_id, float position_x, float position_y, float toward_angle); // 雷达站信息发送
	void Send_Rebirth_Decision(uint16_t target_id, uint8_t *data1, uint16_t length);					// 哨兵发送决策复活和购买发弹量
	void Sentry_dataTransmit(uint8_t sender_id,
							 uint8_t intention,
							 uint16_t start_position_x,
							 uint16_t start_position_y,
							 int8_t *delta_x,
							 int8_t *delta_y);
	void custom_dataTransmit(uint8_t sender_id,
							 uint16_t receiver_id,
							 uint8_t *user_data);
	/**
	 * @brief 新增加的离线监测功能，需要在某个task中定时执行这个函数
	 *
	 * @param SystemTime 用xTaskGetTickCount把当前系统时间传进去
	 */
	inline void offlineDetection(uint32_t SystemTime)
	{
		status = (SystemTime - LastPackReceiveTime <= RF_TIMEOUT_TIME) ? RF_ONLINE : RF_OFFLINE;
	}
	/* 用于清除图像 */
	void clean_one_picture(uint8_t vlayer, uint8_t name[]);					  // 清除单个图形
	void clean_two_picture(uint8_t vlayer, uint8_t name1[], uint8_t name2[]); // 清除两张图形
	void clean_layer(uint8_t _layer);										  // 清除单个图层
	void clean_all();														  // 清除所有

	/* 组合图形用户接口实现【已测试】 */
 #ifdef USE_REFEREE_UI
	/* 字符串绘制 */
	void Draw_Char(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint8_t *char_name, uint8_t *data, uint16_t str_len, uint16_t str_size, colorType_e _color, drawOperate_e _operate_type);
	/* 通用UI标尺 */
	uint8_t UI_ruler(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t scale_step, uint16_t scale_long, uint16_t scale_short, colorType_e _color, drawOperate_e _operate_type);
	/* 通用摩擦轮状态 */
	void Draw_Fri_State(uint8_t fri_flag, uint16_t center_x, uint16_t center_y);

	/* 通用UI准星 */
	void UI_Collimator(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t line_length, colorType_e _color, drawOperate_e _operate_type);
	/* 通用敌方血条 */
	void HP_UI(uint8_t ID, uint16_t start_x, uint16_t start_y, uint16_t length, uint16_t width, colorType_e _color, uint8_t enable_cnt);

	/* 【图层0】视觉自瞄开启标志 */
	void Draw_Auto_Lock_Range(uint8_t auto_flag, uint8_t auto_mode, uint16_t center_x, uint16_t center_y, uint16_t length, uint16_t width, uint16_t line_width);
	void Draw_Auto_Lock(uint8_t auto_flag, uint16_t center_x, uint16_t center_y, uint16_t line_width, colorType_e _color);
	/* 【图层0】平衡步状态绘制 */
	void Draw_Balance_State(float pitch_angle, float yaw_angle, uint16_t start_x, uint16_t start_y, uint16_t length, colorType_e _color);
	/* 【自定义绘制】平衡步停车区域绘制 */
	void Draw_Balance_Stop_Erea(uint8_t _layer, uint8_t enable_cnt, float _current_speed, float _max_speed, float height, uint16_t scale);
	/*【自定义绘制】平衡步小陀螺撞墙提示 */
	void Draw_Rotation_Crash(uint8_t _layer, uint16_t center_x, uint16_t center_y, bool is_crash);
	/*【自定义绘制】并联五连杆轮足方位角绘制 */
	void Draw_WheelBipe_Posture(uint8_t _layer, uint8_t enable_cnt, uint16_t center_x, uint16_t center_y, float b1, float b2, float b3, float _f_angle, float _b_angle, float bipe_endx, float bipe_endy, float _scale, colorType_e _color);
	/* 【图层0】车界线绘制 */

	void SHU_Cap_Energy(float current_volt, float max_volt, float min_volt, uint8_t enable_cnt, uint16_t center_x, uint16_t center_y, uint8_t if_merge = 0, uint8_t *order = nullptr);
	void SHU_ruler(uint8_t enable_cnt, uint16_t center_x, uint16_t center_y);
	void SHU_vision_mode(uint8_t vision_mode, uint8_t if_vision, uint8_t enable_cnt, uint16_t center_x, uint16_t center_y, uint8_t if_merge = 0, uint8_t *order = nullptr);
	void SHU_bullet(int bullet_num, uint8_t enable_cnt, uint16_t center_x, uint16_t center_y, uint8_t if_merge = 0, uint8_t *order = nullptr, colorType_e color = PURPLE);
	void Pack_Data(uint8_t _index);

	void Draw_Robot_Limit(uint16_t height, uint16_t distance, uint16_t center_x, uint16_t line_width, colorType_e _color, drawOperate_e _operate_type);
	/* 【图层3】静态弹仓标志，不允许添加其他图形在该图层 */
	void Draw_Bullet(uint8_t bullet_flag, uint16_t center_x, uint16_t center_y, uint16_t line_width, colorType_e _color);
	/*【图层4】开弹仓时提示*/
	void Draw_BulletBay_Open(uint8_t bulletbay_open_flag, uint16_t center_x, uint16_t center_y, colorType_e _color);
	/* 【图层2】静态小陀螺标志 */
	void Draw_Spin(uint8_t spin_flag, uint16_t center_x, uint16_t center_y, uint16_t line_width, colorType_e _color);
	/* 【图层1】超级电容开启绘制 */
	void Draw_Boost(uint8_t boost_flag, uint16_t center_x, uint16_t center_y, uint16_t line_width, colorType_e _color);
	/* 【图层1】电容电压百分比绘制，并配合超级电容开启打印提示字符串 */
	void Draw_Cap_Energy(float current_volt, float max_volt, float min_volt, uint8_t enable_cnt, uint16_t center_x, uint16_t center_y);
	/* 【图层4】无弹丸时提示补弹 */
	void Draw_No_Bullet(uint8_t no_bullet_flag, uint16_t center_x, uint16_t center_y, colorType_e _color);
	/* 【图层5】枪口热量 */
	void Draw_CoolingHeat(uint16_t cooling_heat, uint16_t cooling_limit, uint16_t center_x, uint16_t center_y, uint16_t r, uint16_t line_width);

	/* ---------------------英雄机器人榴弹准星，自定义图层----------------------- */
	void Hero_UI_line(uint16_t startx, uint16_t starty, uint16_t endx, uint16_t endy, colorType_e _color);
	void Hero_UI_line_2(uint16_t startx, uint16_t starty, uint16_t endx, uint16_t endy, colorType_e _color);
	void Hero_UI_ruler(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t *line_distance, uint16_t *line_length, colorType_e *_color, drawOperate_e _operate_type);
	void Hero_PitchAng_Frame(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t size, colorType_e _color);
	void Hero_PitchAng_Update(uint16_t ang_cur, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t size, colorType_e _color);
	void Hero_YawAng_Frame(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t size, colorType_e _color);
	void Hero_YawAng_Update(uint8_t ang_cur, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t size, colorType_e _color);
	void Hero_Compensate_Update(uint8_t com_pitch, uint8_t com_yaw, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t size, colorType_e _color);
	void Hero_Fric_Speed_Update(uint16_t fric_spe, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t size, colorType_e _color);
	void Hero_PitCom_Update(uint8_t com_pitch, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t size, colorType_e _color);
	/* ---------------------空中机器人UI绘制，自定义图层----------------------- */
	// 空中机器人pitch轴标尺
	void Aerial_PitchRuler_Frame(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, uint16_t long_scale_length, uint16_t short_scale_length, colorType_e ruler_color, colorType_e current_color, colorType_e target_color);
	void Aerial_Pitch_Update(float pitch_angle, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, colorType_e tag_color);	 // 空中机器人pitch轴浮标
	void Aerial_Pitch_Target(float pitch_target, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, colorType_e target_color); // 空中机器人pitch轴目标浮标
	// 云台手剩余全局剩余弹量显示
	void Aerial_BulletRemain_Frame(uint8_t layer, uint16_t start_x, uint16_t start_y, uint16_t size);
	void Aerial_BulletRemain_Update(uint16_t _17mm, uint16_t _42mm, uint8_t layer, uint16_t start_x, uint16_t start_y, uint16_t size);

	/* ---------------------工程机器人UI绘制，自定义图层----------------------- */
	// 工程机器人抬升高度标尺
	void Engineer_HighthRuler_Frame(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, uint16_t long_scale_length, uint16_t short_scale_length, uint8_t ruler_tag, colorType_e ruler_color, colorType_e current_color);
	// 工程机器人当前抬升高度浮标
	void Engineer_Height_Update(float height, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, uint8_t ruler_tag, colorType_e tag_color);
	// 工程机器人当前抬升高度浮标（两个标尺的浮标合并为一个数据包发送，减少数据包传输延迟带来的动态效果不同步）
	void Engineer_HeightMul_Update(float *height, uint8_t _layer, uint16_t *center_x, uint16_t *center_y, uint16_t *total_length, colorType_e tag_color);
	// 工程机器人目标抬升高度浮标
	void Engineer_Target_Height(float target_height, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, uint8_t ruler_tag, colorType_e tag_color);
	// 工程机器人对准中间矿石直线
	void Engineer_UI_line(uint16_t center_x, uint16_t width, colorType_e _color);

	void Mine_Icon(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t mine_tag, colorType_e _color, drawOperate_e _operate_type); // 矿石图标
	void Rescue_Icon(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, colorType_e _color, drawOperate_e _operate_type);
	void Engineer_MineRe_Frame(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint8_t size);										 // 工程机器人当前存储矿石数UI框架
	void Engineer_MineRe_Update(uint8_t mine_num, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint8_t size);					 // 更新工程机器人存储矿石数
	void Engineer_MineMode_Update(uint8_t mode, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint8_t size, colorType_e _color);	 // 工程机器人自动/手动组状态
	void Engineer_RescueMode_Update(uint8_t mode, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint8_t size, colorType_e _color); // 工程机器人救援/刷卡状态
	void Engineer_AutoMode_Update(uint8_t status, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint8_t size);					 // 工程机器人自动组状态

	/* ----------------------哨兵机器人UI绘制，自定义图层----------------------- */
	void Sentry_PosRuler_Frame(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, colorType_e ruler_color, colorType_e current_color); // 哨兵机器人移动轨道绘制
	void Sentry_Pos_Update(float pos_percent, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t total_length, colorType_e tag_color);				 // 哨兵机器人当前位置浮标
	void Sentry_Patrol_Frame(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t size, uint8_t *keys, colorType_e patrol_color);						 // 哨兵机器人巡逻区域绘制
	void Sentry_Patrol_Update(uint8_t tag, uint8_t _layer, uint16_t center_x, uint16_t center_y, uint16_t size, colorType_e _color);							 // 哨兵机器人索敌状态指示
	void Sentry_Bullet_Frame(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t size, colorType_e _color);											 // 哨兵机器人发弹状态框架
	void Sentry_Bullet_Update(uint8_t tag, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t size, colorType_e _color);								 // 哨兵机器人发弹状态
	void Sentry_Mode_Frame(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t size, colorType_e _color);
	void Sentry_Mode_Update(uint8_t mode, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t size, colorType_e _color);
	void Sentry_Operation_Frame(uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t size, colorType_e _color);
	void Sentry_Operation_Update(uint8_t opeartion, uint8_t _layer, uint16_t start_x, uint16_t start_y, uint16_t size, colorType_e _color);

	/* ----------------------雷达站策略集显示，占用图层9------------------------
	策略集发生变化时，会占用带宽以绘制对应UI*/
	void Armor(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t armor_tag, colorType_e _color, drawOperate_e _operate_type);		 // 盾牌图标
	void Sword(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t sword_tag, colorType_e _color, drawOperate_e _operate_type);		 // 剑图标
	void Outpost_Icon(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t tag, colorType_e _color, drawOperate_e _operate_type);	 // 前哨站图标
	void Sentry_Icon(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t tag, colorType_e _color, drawOperate_e _operate_type);		 // 哨兵图标
	void Missle_Icon(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t tag, colorType_e _color, drawOperate_e _operate_type);		 // 飞镖图标
	void High_Land(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t tag, colorType_e _color, drawOperate_e _operate_type);		 // 高地图标
	void Windmill_Icon(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t tag, colorType_e _color, drawOperate_e _operate_type);	 // 神符图标
	void FlyingSlope_Icon(uint8_t _layer, uint16_t center_x, uint16_t center_y, uint8_t size, uint8_t tag, colorType_e _color, drawOperate_e _operate_type); // 飞坡图标
	void Radar_Strategy_Frame(uint16_t *frame_pos_x, uint16_t *frame_pos_y);																				 // 雷达站策略UI框架
	void Radar_CStrategy_Update(uint8_t protect, uint8_t attack, uint8_t comment_startegy, uint16_t *pos_x, uint16_t *pos_y);								 // 雷达站全局策略UI绘制
	void Radar_SStrategy_Update(uint16_t special_startegy, uint16_t *pos_x, uint16_t *pos_y);																 // 雷达站专用策略UI绘制
 #endif
private:
	/* 外部句柄 */
	UART_HandleTypeDef *refereeUart;
	SystemTick_Fun Get_SystemTick; /*<! Pointer of function to get system tick */

	uint8_t repeat_cnt = 0; // UI绘制重复发包

	/* 发送数据包缓存区，最大128字节 */
	uint8_t transmit_pack[128];
	/* UI绘制数据包 */
	uint8_t data_pack[DRAWING_PACK * 7] = {0};

	/* 客户端删除图形数据包 */
	ext_client_custom_graphic_delete_t cleaning;

	/* 计算我方机器人ID */
	void Calc_Robot_ID(uint8_t local_id);

	/* 解包过程用到的变量及函数 */
	uint8_t DataCheck(uint8_t **p);																			  // 绘画时用于拼接的空间
	unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8); // 获取数据包包头的CRC8校验和
	uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);					  // 获取数据包整包的CRC16校验和
	void RefereeHandle(uint8_t *data_buf);																	  // 更新对应ID的数据
	void RobotInteractiveHandle(robot_interactive_data_t *RobotInteractiveData_t);							  // 机器人间通信

	/* 发送时用到的 */
	void pack_send_robotData(uint16_t _data_cmd_id, uint16_t _receiver_ID, uint8_t *_data, uint16_t _data_len); // 0x301交互数据，包括UI绘制和车间通信
	void send_toReferee(uint16_t _com_id, uint16_t length, receive_Type_e _receive_type);

	/**
	具体图形的绘制：空操作、线、矩形、圆、椭圆、圆弧、浮点数、整数、字符；并指定绘制的图层
	*/
	graphic_data_struct_t *null_drawing(uint8_t _layer, uint8_t name[]);
	graphic_data_struct_t *line_drawing(uint8_t _layer, drawOperate_e _operate_type, uint16_t startx, uint16_t starty, uint16_t endx, uint16_t endy, uint16_t line_width, colorType_e vcolor, uint8_t name[]);
	graphic_data_struct_t *rectangle_drawing(uint8_t _layer, drawOperate_e _operate_type, uint16_t startx, uint16_t starty, uint16_t length, uint16_t width, uint16_t line_width, colorType_e vcolor, uint8_t name[]);
	graphic_data_struct_t *circle_drawing(uint8_t _layer, drawOperate_e _operate_type, uint16_t centrex, uint16_t centrey, uint16_t r, uint16_t line_width, colorType_e vcolor, uint8_t name[]);
	graphic_data_struct_t *oval_drawing(uint8_t _layer, drawOperate_e _operate_type, uint16_t centrex, uint16_t centrey, uint16_t minor_semi_axis, uint16_t major_semi_axis, uint16_t line_width, colorType_e vcolor, uint8_t name[]);
	graphic_data_struct_t *arc_drawing(uint8_t _layer, drawOperate_e _operate_type, uint16_t centrex, uint16_t centrey, uint16_t minor_semi_axis, uint16_t major_semi_axis, int16_t start_angle, int16_t end_angle, uint16_t line_width, colorType_e vcolor, uint8_t name[]);
	graphic_data_struct_t *float_drawing(uint8_t _layer, drawOperate_e _operate_type, uint16_t startx, uint16_t starty, uint16_t size, uint16_t width, colorType_e vcolor, float data, uint8_t name[]);
	graphic_data_struct_t *int_drawing(uint8_t _layer, drawOperate_e _operate_type, uint16_t startx, uint16_t starty, uint16_t size, uint16_t width, colorType_e vcolor, int32_t data, uint8_t name[]);
	graphic_data_struct_t *character_drawing(uint8_t _layer, drawOperate_e _operate_type, uint16_t startx, uint16_t starty, uint16_t size, uint8_t width, uint8_t *data, uint16_t str_len, colorType_e vcolor, uint8_t name[]);
};

 #endif

#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
