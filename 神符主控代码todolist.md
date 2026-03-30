## 增加测试模式提交问题
1. 降低delay到1s
```
case success:
        {
            // 全部点亮
            Ctrl_All_Armors(FAN_CMD_SUCCESS, g_TargetCtrl.TargetColor, 5);
            vTaskDelay(3000); 
            g_TargetCtrl.target_mode = tar_stop; // 结束，回到待机
        }
```
2. 在测试模式里加入对5个灯条的控制代码，直接用一个测试结构体
```
 case test_mode:
        {
            // 测试模式下不跑大小符逻辑，不重置灯效
            g_SystemState.TargetSpeed = 0;
        }
        break;
```
3. 删掉0xEE的控制包解析。
4. 删除FanFeedbackProcess的测试代码。
5. 排查id问题和灯效id问题，排查模拟击打问题
6. 排查灯板控制的调用关系。等会把灯板控制集中在一个函数里统一更新。其他的任务只修改一个数组即可。
## feature分支问题
1. FanCmdType修改掉，和分控统一名称和内容。
2. SE的打错，重置else分支放错地方了。
3. SE的灯板控制逻辑混乱。重新查target id的生成路径。
4. success的命令记得改。vtaskdelay阻塞式延时都尽量优化掉。
5. SendFanPacket里面加了1ms延时，好像能增加稳定性？但是是发送到队列，一般不需要延时。需要测试队列动态占用情况再做决定。can发送应该没那么慢。可能电机也会往Can2发，需要排查一下。
## main分支问题
1. Ctrl_All_Armors的索引统一一下。
2. reset后加延时。（可能需要，看reset效果如何）
3. 取消se reset和be reset的重复串口打印。只打激活时的一次。
4. SE的updateSEArmorLight会覆盖控制其他灯臂。索引记得改。
5. R标任务挪到状态机里吧。只需要在颜色变化时输出就行。
6. 增大can发送队列的深度。检查是发的指针还是结构体。把发送消息优化到全局位置，防止把局部变量的地址发给can了。
7. 删除无关r标代码。
8. hit_feedback_to_uart前后记得加延时，防止连续发送造成数据错乱。