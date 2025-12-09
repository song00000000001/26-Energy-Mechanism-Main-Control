#include "robot_config.h"

//由于在cpp调用c总会出现警告，所以把任务中会调用c的语句都集中在该c文件中
//为了方便修改参数，底层调用的函数都写成宏定义，放在robot_config.h中
//这里只写动作组合任务
