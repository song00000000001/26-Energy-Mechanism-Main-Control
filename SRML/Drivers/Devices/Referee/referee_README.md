## 一、模块功能
2021赛季的裁判系统库提供了如下功能：

**1.裁判系统数据接收与外部读取**  
裁判系统会把机器人的状态信息等数据通过串口发送给主控  

**2.机器人车间通信**  
每个机器人都可以通过发送一定格式的数据包给裁判系统，裁判系统根据数据包类型转发给队友的机器人

**3.自定义UI绘制**
可通过裁判系统在本机器人对应的操作手界面上绘制出基本图形和字母、数字等，辅助操作手捕捉必要赛场信息

## 二、使用方法  
**1. Init()初始化模块，指定接入裁判系统学生串口的串口句柄**

**2. 裁判系统数据接收与外部读取**
- 使用unPackDataFromRF()解包裁判系统串口数据
- 如果需要用到裁判系统提供的各种数据（具体有些什么数据请查看手册），读取相应结构体即可

**3. 机器人车间通信**
1. 发送端调用CV_ToOtherRobot()发送数据
2. 接收端使用库提供的宏访问结构体数组robot_rec_data[]，即可得到其他机器人发送的车间通信数据。如工程发送过来的数据在robot_rec_data[ENGINEER]
3. 一个例子就是21赛季的哨兵机器人，采用车间通信实现云台手控制机器人走位~
4. 21赛季的车间通信全队约定为八个字节，可通过修改宏`ROBOT_COM_PACK`改变字节数，但建议宜少不宜多（会占用带宽）

**4. 操作手界面UI**
- 该库已针对21赛季的操作手需求封装了大量的UI组合图形（例如英雄机器人的发弹下坠标尺），可以直接调用。
- 调用前请在`Keil->Option of Target->C/C++中添加宏定义`USE_REFEREE_UI`，将UI库参与编译。
- 关于如何使用这些UI库请移步至[此处](#附录：UI库使用示例（步兵、英雄） )

- 如果你想创建出新的UI，可以使用该库提供的基本图形绘制API如下即可：
```cpp
graphic_data_struct_t* null_drawing(uint8_t _layer,uint8_t name[]);
graphic_data_struct_t* line_drawing(uint8_t _layer,drawOperate_e _operate_type,uint16_t startx,uint16_t starty,uint16_t endx,uint16_t endy, uint16_t line_width, colorType_e vcolor,uint8_t name[]);
graphic_data_struct_t* rectangle_drawing(uint8_t _layer,drawOperate_e _operate_type, uint16_t startx,uint16_t starty,uint16_t length,uint16_t width, uint16_t line_width, colorType_e vcolor, uint8_t name[]);
graphic_data_struct_t* circle_drawing(uint8_t _layer,drawOperate_e _operate_type, uint16_t centrex,uint16_t centrey,uint16_t r, uint16_t line_width, colorType_e vcolor, uint8_t name[]);
graphic_data_struct_t* oval_drawing(uint8_t _layer,drawOperate_e _operate_type, uint16_t centrex,uint16_t centrey,uint16_t minor_semi_axis,uint16_t major_semi_axis, uint16_t line_width, colorType_e vcolor, uint8_t name[]);
graphic_data_struct_t* arc_drawing(uint8_t _layer,drawOperate_e _operate_type, uint16_t centrex,uint16_t centrey,uint16_t minor_semi_axis,uint16_t major_semi_axis,int16_t start_angle,int16_t end_angle,uint16_t line_width,colorType_e vcolor, uint8_t name[]);
graphic_data_struct_t* float_drawing(uint8_t _layer,drawOperate_e _operate_type, uint16_t startx,uint16_t starty,uint16_t size, uint16_t width,colorType_e vcolor, float data, uint8_t name[]);
graphic_data_struct_t* int_drawing(uint8_t _layer, drawOperate_e _operate_type, uint16_t startx,uint16_t starty,uint16_t size, uint16_t width, colorType_e vcolor, int32_t data,uint8_t name[]);
graphic_data_struct_t* character_drawing(uint8_t _layer,drawOperate_e _operate_type,uint16_t startx,uint16_t starty,uint16_t size, uint8_t width,uint8_t* data, uint16_t str_len, colorType_e vcolor, uint8_t name[]);		
```

## 三、注意事项们
**1. UI和车间通信在底层均用到了vTaskDelay控制发送速率，故不需要在UI绘制任务中再调用延时**

**2. 发数据给裁判系统时，务必注意等待上电稳定之后才发送，否则会有明显丢包** 

**3. 绘制自己的UI图形时，注意事项：**
- 屏幕坐标范围为左下角(0,0)~右上角(1920,1080)
- 每个图形都有自己的名字（三个字符）。名字相同的图形，只显示先发出去的那个，后面发的都无效
- 画图之后如果想要修改某个图形（例如直线延长），请通过MODIFY_PICTURE操作实现
- 要有图才能MODIFY_PICTURE操作，否则应ADD_PICTURE添加图片   
- 字符串的长度不应超过30  
- 每一个图形建议画多几次，有时候裁判系统接收不到  
- 每一个图形的名字要使用数组定义 

## **附录：UI库使用示例（步兵、英雄）**
- **电控预留好小陀螺、超级电容、弹仓对应的接口：uint8_t型状态标志，非0时表示开启**
- **创建UI绘制任务如下，对步兵：**

```
/**
* @brief    裁判系统可视化UI绘制任务
* @note     绘制车界线、电容电压（百分比）以及三种指示标志：小陀螺、超级电容、弹仓
* @return   None.
*/
void Device_Referee_UI(void *arg)
{
    static TickType_t _xPreviousWakeTime;

    //起始绘制时的绘制次数。若服务器丢包率较高，该值可适当给大
    static uint8_t enable_cnt = 20;             
 
    //下坠UI标尺的水平刻度线长度、距离、颜色；垂直线总长度由为各水平刻度线距离之和
 	uint16_t line_distance[6] = {30,30,30,30,30,50};
	uint16_t line_length[6] = {120,80,70,60,40,30};
    colorType_e ruler_color[7] = {WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE};     //最后一个为垂直线颜色

    //雷达站策略集坐标
    uint16_t cpos_x[4] = {100,160,220,260};						                        //全局UI推荐坐标
	uint16_t cpos_y[4] = {730,730,730,730};
	uint16_t frame_x[2] = {100,100};							
	uint16_t frame_y[2] = {800,670};

	uint16_t spos_x[3] = {100,200,80};							                        //专用UI推荐坐标
	uint16_t spos_y[3] = {610,610,560};	

    //图传稳定需要一段时间的延时
    vTaskDelay(500);
    referee.clean_all();

    vTaskDelay(2000);                        
 
    for(;;)
    { 
        //刚开始时多次绘制图形，确保能在动态UI刚开启时顺利绘制图形
        if(enable_cnt)
        {
            //车界线、下坠标尺绘制
            referee.Draw_Robot_Limit(180, 80, 961, 3, YELLOW, ADD_PICTURE);
            referee.Hero_UI_ruler(5, 961, 538, line_distance, line_length, ruler_color, ADD_PICTURE);	

            //绘制电容剩余能量
            referee.Draw_Cap_Energy(SourceManage.capObj.Voltage, 24, 17, enable_cnt, 420,800);			

            //雷达站策略集部分
			referee.Radar_Strategy_Frame(frame_x, frame_y);

            enable_cnt--;
        }
        else
        {
            referee.Draw_Cap_Energy(SourceManage.capObj.Voltage, 24, 17, enable_cnt, 420,800);	//绘制电容剩余能量
			referee.Draw_Boost((uint8_t)Chassis.Mode-1, 1600, 740, 10, PINK);					//绘制超级电容状态
			referee.Draw_Spin((uint8_t)Chassis.moveMode, 1400, 740, 10, BLUE);					//绘制小陀螺开启状态
			referee.Draw_Bullet(Chassis.bulletBayMode, 1800, 740, 8, GREEN);					//绘制弹仓开启状态
			referee.Draw_Auto_Lock(Chassis.pcVisionMode, 1400, 680, 8, WHITE);					//绘制自瞄开启状态
			referee.Draw_No_Bullet(Chassis.bulletNum, 861, 738, ORANGE);						//绘制空弹提示
			
			//Radar
			referee.Radar_CStrategy_Update(0, 0, referee.robot_rec_data[RADAR].data[0], cpos_x, cpos_y);		//雷达站通用策略集
			referee.Radar_SStrategy_Update(referee.robot_rec_data[RADAR].data[1], spos_x, spos_y);				//雷达站专用策略集
        }
    }
}
```

- **创建UI绘制任务如下，对英雄：**

```
/**
* @brief    裁判系统可视化UI绘制任务
* @note     绘制车界线、电容电压（百分比）以及三种指示标志：小陀螺、超级电容、弹仓
* @return   None.
*/
void Device_Referee_UI(void *arg)
{
    static TickType_t _xPreviousWakeTime;

    //起始绘制时的绘制次数。若服务器丢包率较高，该值可适当给大
    static uint8_t enable_cnt = 20;             
 
    //下坠UI标尺的水平刻度线长度、距离、颜色；垂直线总长度由为各水平刻度线距离之和
 	uint16_t line_distance[6] = {30,42,30,30,15,65};
	uint16_t line_length[6] = {0,0,60,48,60,60};
	colorType_e ruler_color[7] = {WHITE, WHITE, ORANGE, YELLOW, ORANGE, RED, RED};      //最后一个为垂直线颜色
	
    //雷达站策略集坐标
    uint16_t cpos_x[4] = {100,160,220,260};						                        //全局UI推荐坐标
	uint16_t cpos_y[4] = {730,730,730,730};
	uint16_t frame_x[2] = {100,100};							
	uint16_t frame_y[2] = {800,670};

	uint16_t spos_x[3] = {100,200,80};							                        //专用UI推荐坐标
	uint16_t spos_y[3] = {610,610,560};	

    //图传稳定需要一段时间的延时
    vTaskDelay(500);
    referee.clean_all();

    vTaskDelay(2000);                        
 
    for(;;)
    { 
        //刚开始时多次绘制图形，确保能在动态UI刚开启时顺利绘制图形
        if(enable_cnt)
        {
            //车界线、下坠标尺绘制
            referee.Draw_Robot_Limit(180, 80, 961, 3, YELLOW, ADD_PICTURE);
            referee.Hero_UI_ruler(5, 961, 538, line_distance, line_length, ruler_color, ADD_PICTURE);	

            //绘制电容剩余能量
            referee.Draw_Cap_Energy(SourceManage.capObj.Voltage, 24, 17, enable_cnt, 420,800);			

            //雷达站策略集部分
			referee.Radar_Strategy_Frame(frame_x, frame_y);

            enable_cnt--;
        }
        else
        {
            referee.Draw_Cap_Energy(SourceManage.capObj.Voltage, 24, 17, enable_cnt, 420,800);	//绘制电容剩余能量
			referee.Draw_Boost((uint8_t)Chassis.Mode-1, 1600, 740, 10, PINK);					//绘制超级电容状态
			referee.Draw_Spin((uint8_t)Chassis.moveMode, 1400, 740, 10, BLUE);					//绘制小陀螺开启状态
			referee.Draw_Bullet(Chassis.bulletBayMode, 1800, 740, 8, GREEN);					//绘制弹仓开启状态
			referee.Draw_Auto_Lock(Chassis.pcVisionMode, 1400, 680, 8, WHITE);					//绘制自瞄开启状态
			referee.Draw_No_Bullet(Chassis.bulletNum, 861, 738, ORANGE);						//绘制空弹提示
			
			//Radar
			referee.Radar_CStrategy_Update(0, 0, referee.robot_rec_data[RADAR].data[0], cpos_x, cpos_y);		//雷达站通用策略集
			referee.Radar_SStrategy_Update(referee.robot_rec_data[RADAR].data[1], spos_x, spos_y);				//雷达站专用策略集
        }
    }
}
```

- **关于enable_cnt，电控可通过新增按键，在任务运行时对该变量重新赋予非0值，达到重置UI绘制的目的**
- **另外，车界线和三种指示标志的位置已预留出位置接口，可根据操作手习惯自行调整显示位置**
