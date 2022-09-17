#include "scheduler_task.h"
#include "pm01_api.h"
int dt_time[3]={0};
int watch_ad=0;
void Loop_1000Hz() 
{
	dt_time[1] = dt_time[0];
	dt_time[0] = HAL_GetTick();
	time_tick_1ms++;
	WorkStateFSM();
	Watch_Task();
	CAN_Watch();

	mpu_get_data();
	imu_ahrs_update();
	imu_attitude_update();
	
	Gimbal_Control();
	ALL_Motor_Out();
	dt_time[2] = dt_time[0] - dt_time[1];
}

void Loop_250Hz()
{
	Shoot_Control();
	Chassis_Control();
	//Chassis_Power_Limit();
}

void Loop_100Hz()
{
		Ammunition_supply_Control();	
		Shoot_Power_Control();
	
}

void Loop_50Hz()
{
	Jundge_Analysis(&unpack_data);
	Vofaplus_Sendware();
}

void Loop_10Hz()
{
	
}

void Loop_5Hz()
{
	Draw_Dynamicr_CrossHair_1(&Draw_CH, 0x0301, 0x0104, 4,0x168);
} 

void Loop_3Hz()
{
	//Draw_Dynamicr_CrossHair_1(&Draw_CH, 0x0301, 0x0104, 104,0x168);
//		
}

void Loop_1Hz()
{
//	Draw_Dynamicr_Char_2(&Draw_Char,0x0301,0x0110 ,103,0x167);
//	Draw_Static_Char_1(&Draw_Char,0x0301,0x0104 ,3,0x103);/*交互数据，四图，蓝3收发*/
//	Draw_Static_CrossHair_1(&Draw_CH,0x0301,0x0104 ,3,0x103);	
//		   Robot=jundge_game_robot_status_t.date.robot_id;
		watch_ad++;
		if(watch_ad%3==0)	Draw_Static_CrossHair_1(&Draw_CH,0x0301,0x0104 ,4,0x104);	
		else if(watch_ad%3==1)Draw_Static_Char(&Draw_Char, 0x301, 0x0110, 4, 0x104);
		else if(watch_ad%3==2)Draw_Static_Char_1(&Draw_Char, 0x301, 0x0110, 4, 0x104);
	
	
}
static sched_task_t Sched_Task[]={
	{Loop_1000Hz,1000,0,0},			
	{Loop_250Hz ,250 ,0,0},
	{Loop_100Hz ,100 ,0,0},
	{Loop_50Hz ,50 ,0,0},
	{Loop_10Hz ,10 ,0,0},
	{Loop_5Hz ,5 ,0,0},
	{Loop_3Hz ,3 ,0,0},
	{Loop_1Hz	,1,0,0},
	
};
#define TASK_NUM (sizeof(Sched_Task)/sizeof(sched_task_t))
void Scheduler_Setup()
{
	for(int index=0;index<TASK_NUM;index++)
	{
		Sched_Task[index].interval_tick = 1000/Sched_Task[index].rate_hz;
		if(Sched_Task[index].interval_tick<1)
			Sched_Task[index].interval_tick = 1;
	}
}

void Scheduler_Run()
{
	uint32_t tnow,diff;
	for(int index=0;index<TASK_NUM;index++)
	{
		tnow = HAL_GetTick();
		diff = (tnow - Sched_Task[index].last_run);
		if(Sched_Task[index].interval_tick <= diff)
		{
			Sched_Task[index].last_run = tnow;
			Sched_Task[index].task_fuc();			
		}
	}
	
}




