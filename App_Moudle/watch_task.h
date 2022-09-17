#ifndef _WATCH_TASK_H_
#define _WATCH_TASK_H_
#include "can.h"
#include "stm32f4xx_hal.h"
#include "canbus_task.h"
#define CAN_TIME 1000
typedef enum
{
	CAN_PREPARE,
	CAN_NORMAL,
	CAN_ERROR,
}can_state_en;

typedef struct
{
	unsigned int CM1;
	unsigned int CM2;
	unsigned int CM3;
	unsigned int CM4;
	unsigned int PITCH;
	unsigned int YAW;
	unsigned int SUPPLY;
	unsigned int FrictionL;
	unsigned int FrictionR;
	unsigned int REMOTE;
	
}Tick_Controller_t;

typedef enum
{
	eCM1 = 0,
	eCM2,
	eCM3,
	eCM4,
	ePITCH,
	eYAW,
	eSUPPLY,
	eFrictionL,
	eFrictionR
}motor_online_en;

extern Tick_Controller_t Tick_Controller;
extern char Is_Remote_Online;
extern volatile int can_count ;

void Watch_Task(void);
void CAN_Watch(void);
extern void SetCanState(can_state_en state);
extern can_state_en GetCanState(void);


#endif

