## SRML: ~~SCUT Robotlab Middlewares Layer Library~~ 
## Shit Rubbish Mountain Library
<Font><font size="5">**机器人实验室嵌入式软件中间件层库**</Font>

![](https://img.shields.io/badge/当前版本-V3.0-blue.svg)

[TOC]

# Summary 简介 

SRML旨在将已有的代码解耦，模块化后封装成一个服务于组织内所有软件项目的中间件层，形成实验室内部软件开发标准接口。今后机器人的电控软件业务逻辑（跟比赛项目中机器人的具体功能流程逻辑相关的代码）的开发过程中统一基于这个接口，以解决实验室代码混乱的问题。同时机器人软件开发者需要维护、升级已有的模块，而无需为了同一个功能重复造轮子，提高软件开发效率和质量。如果你认为自己对某个模块有更好的代码方案，或者发现了现有模块不再满足当前使用需求，请仔细阅读完原有代码后向实验室其他队员收集需求意见然后再进行重构。重构时请务必谨慎考虑模块接口的设置！
- __功能强大__
  - 本库包含实验室所有稳定的底层代码，通信协议，设备驱动，功能模块以及终端调试系统，可提供完整的嵌入式控制系统开发工具链。
- __灵活方便__
  - 内核可裁剪（通过预编译宏决定参与编译的模块），使用时只需要包含一个头文件即可。
- __维护方便__
  - 本仓库将严格按照代码规范编写，各个功能模块由模块负责人专人维护，建立清晰的依赖关系。
  - 所有代码需要严格测试后才能发布。
  - 按照Git使用规范管理本仓库。

设计思路详情请参考 Confluence：[《通用软件系统搭建分析》](https://bbs.scutbot.cn/x/k4BPAQ), [《软件架构篇》](https://bbs.scutbot.cn/x/GoDKAQ)，[《如何构建一个机器人栈》](https://bbs.scutbot.cn/x/OYDKAQ)。

## Improvement 改进方向

后续可以参考软件设计模式中的发布订阅模型，典型的可以学习**ROS2**，**PX4**，**Apollo**，**DJI Robomaster2020 AI开源机器人**等已有产品的架构，鼓励有兴趣的同学去思考根据我们的应用场景来改进甚至重构我们的软件架构。其中参与RM开源机器人开发的嵌入式工程师是我们的师兄[17-电控-张波](https://bbs.scutbot.cn/x/IIDKAQ),有兴趣可以多上空间点赞评论催更。

## How to Deploy 配置方法
- __获取 SRML 库__
  - 参见：[添加子模块的远程仓库](#添加子模块的远程仓库)
- __添加 SRML 库到崭新的 Stm32 工程中__
  - 包含`SRML.h`
  - 将`srml_config_template.h`拷贝到用户代码目录，并重命名为`srml_config.h`
  - 在`srml_config.h`中定义各模块是否参与编译
  - __单个STM32CubeIDE工程__
    - 将SRML目录加入源代码位置和包含路径
    - 编译
  - [__MDK工程__](https://git.scutbot.cn/Embedded/20_Project_Template.git)
    - 按照文件目录结构将库文件添加到工程中
    - 编译

## How to Develop 维护方法
- __Git仓库管理__
  - 添加子模块的远程仓库

  ```bash
  # 以下两种方法选一种即可
  $ git submodule add https://git.scutbot.cn/Embedded/SRML.git # http方法
  $ git submodule add ssh://git@git.scutbot.cn:23232/Embedded/SRML.git # ssh方法
  ```

  - clone已有的工程时拉取SRML的代码
  ```bash
  $ git submodule update --init --recursive
  ```

  - 更新 SRML 库

  ```bash
  $ cd SRML/
  $ git pull origin master
  ```

  - 推送更改到 SRML 库
  
  ```bash
  $ cd SRML/
  $ git push origin dev
  ```

# 各功能模块详解

## 底层外设驱动模块

### 	drv_uart模块

[‬⁣⁣‬⁣⁣⁢‌﻿﻿⁡⁤⁤⁣‌⁤‬⁤⁡⁢‍‍‍‍⁡⁡⁣﻿‬⁢⁣⁣‬⁢﻿‌﻿‌⁤⁢⁡‍drv_uart的使用与设计思路 - 飞书云文档 (feishu.cn)](https://scutrobotlab.feishu.cn/wiki/MhbawEBNoiKBeLkDTaRcA9wfnyf)

### 	drv_can模块

[‍‌﻿‍‍⁢‬﻿‬⁢‬⁢‌⁤‍﻿‍⁤⁡‍‍‌⁤⁡⁢‍‌‍﻿⁤⁣⁡⁢‌⁤⁣‌‍⁣⁣drv_can的使用与设计思路 - 飞书云文档 (feishu.cn)](https://scutrobotlab.feishu.cn/wiki/WJmdwVPmoiBKPQkyNRcch3efnj3)

### 	drv_timer模块

[‍‌﻿﻿⁢⁡⁤‍‬⁡‍⁡﻿‍‌‬⁣‌﻿⁤⁡⁣⁤⁤⁢‬‌⁢‌⁤⁢⁤⁢‌﻿‌⁡⁢‍⁡⁣‌‬drv_timer的使用与设计思路 - 飞书云文档 (feishu.cn)](https://scutrobotlab.feishu.cn/wiki/XyFowizthiaAQekrWYCcMj5Fnuf)

### 	drv_memory模块

[单片机动态内存管理的实现 - 飞书云文档 (feishu.cn)](https://scutrobotlab.feishu.cn/wiki/Nbnmwg5wOiwa06kikxcctPvtn7f)

### 	drv_VirtualCom模块

[‌﻿‌‌‬﻿‬⁤⁢‍⁤‍⁤‌⁢﻿‍⁤⁢⁡‌‍⁢‬⁢‬‍⁢⁤⁣⁤‍‍‍‬⁡⁡⁢⁡⁢‬‍drv_VirtualCom的使用与设计思路 - 飞书云文档 (feishu.cn)](https://scutrobotlab.feishu.cn/wiki/WTtVwh7nSi3HnKkdq74cCum2nbe)



## 设备模块

### 	DR16模块

[遥控器DR16模块](https://scutrobotlab.feishu.cn/wiki/RgFZwgyl7iIABhksqYbc55HWnSh) 

### 富斯I6X模块

[富斯I6X模块 - 飞书云文档 (feishu.cn)](https://scutrobotlab.feishu.cn/wiki/SpqQwYYBmiawypk8De6crmivnGb)

### 	裁判系统模块

未填坑



## 电机模块

### 	大疆电机

[‍⁣⁤‍﻿⁡‍‍⁤⁣‍‬⁣‌⁢⁡⁢‍‍⁢‬⁢‍⁣‬‬⁢‬﻿⁡⁣﻿⁢⁣⁤⁤﻿‬‍﻿⁤MotorDJI模块 - 飞书云文档 (feishu.cn)](https://scutrobotlab.feishu.cn/wiki/OUw3w0J7gijkesk9eGYc57S1nHb)

### 	翎控MF9025v2电机

[‬﻿‍⁢⁢⁡⁣⁤‍‌‬⁢‬⁤⁡﻿⁣⁣⁢⁣⁢⁡⁤⁤‌﻿⁢⁤⁢‬‍‬⁣‍‍⁢⁢MF9025v2电机模块 - 飞书云文档 (feishu.cn)](https://scutrobotlab.feishu.cn/wiki/SNXNwT2QfirngZkramoc8cpCn6e)

### 	海泰HT04电机

未填坑



## IMU模块

### 	MPU6050模块

[MPU6050模块 - 飞书云文档 (feishu.cn)](https://scutrobotlab.feishu.cn/wiki/KtJWw8tH5iYBi2kbwwOcZpZPnRg)

### 	阿路比LPMS BE2模块

[‍﻿﻿‌‌﻿⁢⁢⁣⁤‌⁢⁣‬⁣⁣‍⁢⁡⁢⁡⁢⁣⁡‬‌⁢‍⁤⁡⁡⁣⁣⁡﻿⁣⁡‌⁡⁢阿路比LPMS BE2模块 - 飞书云文档 (feishu.cn)](https://scutrobotlab.feishu.cn/wiki/Mar0w4nzNiavuwkjqKWcY6gznwb)



## 抽象库

### 	抽象IMU库

未填坑

### 	抽象电机库

未填坑



## 算法模块

### 	PID模块

[PID算法模块 - 飞书云文档 (feishu.cn)](https://scutrobotlab.feishu.cn/wiki/G7awweNkxiigJCktP6scOSKKnSh)

### 	滤波器模块

[滤波模块 - 飞书云文档 (feishu.cn)](https://scutrobotlab.feishu.cn/wiki/GxrNw7Nc5iVkDzko9QpclYzQnre)

### 卡尔曼滤波器

[卡尔曼滤波器 - 飞书云文档 (feishu.cn)](https://scutrobotlab.feishu.cn/wiki/Tns2wqF8oiMGKIkoT89cQabbnEc)

### 	微分计算器

[微分计算器模块 - 飞书云文档 (feishu.cn)](https://scutrobotlab.feishu.cn/wiki/AQvnwbOjpiapWckFqPJcZ7N3nLf)

### 矩阵库

[轻量级矩阵库 - 飞书云文档 (feishu.cn)](https://scutrobotlab.feishu.cn/wiki/QhXnwJUpRiIKI5kSaNecfvhunng)



## 功控相关

### 	数字电源模块

[数字电源模块使用方法](./Middlewares/Module/Digital_Power/README.md)

### 	数字电源的功控模块

[数字电源的功控块使用方法](./Middlewares/Module/Digital_Power/数字电源功控库适配更新.md)



##  调试相关

### Easylog日志库

[‍‬‌⁡‌﻿‍‌⁢‬⁣‍﻿⁡‍‌‬﻿‬‌‍⁤‬⁤‍‌‬⁤‌‌⁣⁢⁢⁢‬Easylog使用教程 - 飞书云文档 (feishu.cn)](https://scutrobotlab.feishu.cn/wiki/UDaCwQh55i6WApkMgBycmVqWn2g)
