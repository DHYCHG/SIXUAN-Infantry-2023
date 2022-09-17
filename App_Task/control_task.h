#ifndef _CONTROL_TASK_H
#define _CONTROL_TASK_H
#include "stm32f4xx_hal.h"
#include "my_ramp.h"
#include "pid.h"
#include "gimbal_control.h"
#include "shoot_control.h"
#include "chassis_control.h"
#include "remote_task.h"
#include  "watch_task.h"
//#include "cmsis_os.h"

#define CONTROL_TASK_DELAY_TICK 1  
typedef enum
{
    PREPARE_STATE,     		//上电后初始化状态4S左右
    NORMAL_STATE,			//无输入状态
    STOP_STATE,        	//停止运动状态
    CALI_STATE,    			//校准状态
}WorkState_e;
extern float Pitch_Start_Encode_Angle;
//extern float Pitch2_Start_Encode_Angle;
extern float Yaw_Start_Encode_Angle;
extern void SetWorkMode(WorkState_e state);
extern WorkState_e GetWorkState(void);
void WorkStateFSM(void);
void control_task_init(void);
extern volatile int time_tick_1ms;
#endif

