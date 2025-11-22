#ifndef _DART_DATA_H_
#define _DART_DATA_H_
#include "stdint.h"
#define MAX_DART_DATAPOOL_SIZE 16
typedef enum __DartAimEnumdef
{
  Outpost = 0,
  Base = 1
}DartAimEnumdef;
typedef struct __DartDataStructdef
{
  double Ignitergoal[2];
  double YawCorrectionAngle[2];
}DartDataStructdef;
typedef enum __ControlStateEnumdef
{
  stanby,
  launch
}ControlStateEnumdef;
#define RCSwitchEnumdef SW_Status_Typedef
#define RC_Up   SW_UP
#define RC_Mid  SW_MID
#define RC_Down SW_DOWN

#endif
