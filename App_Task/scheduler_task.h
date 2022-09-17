#ifndef __SCHEDULER_TASK_H
#define __SCHEDULER_TASK_H
#include "stm32f4xx_hal.h"
#include "bsp_imu.h"
#include "canbus_task.h"
#include "bsp_can.h"
#include "gimbal_control.h"
#include "vofaplus.h"
//#include "shoot_control.h"
typedef struct{
	void (*task_fuc)(void);
	uint16_t rate_hz;
	uint16_t interval_tick;
	uint32_t last_run;
}sched_task_t;
void Loop_1000Hz(void);
void Loop_250Hz(void);
void Loop_100Hz(void);
void Loop_50Hz(void);
void Loop_10Hz(void);
void Loop_5Hz(void);
void Loop_3Hz(void);
void Loop_1Hz(void);

void Scheduler_Setup(void);
void Scheduler_Run(void);

void Vofaplus_Sendware(void);
extern volatile int time_tick_1ms;
//void vofaplus_task(void const *pvParameters);
#endif


