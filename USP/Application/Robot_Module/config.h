#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "stdbool.h"
#ifdef __cplusplus
	#include "stdint.h"
	extern "C" {
#endif

void System_Device_Init(void);
void System_Task_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* _CONFIG_H_ */