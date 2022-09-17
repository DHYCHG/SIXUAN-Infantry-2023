#ifndef _SHOOT_CONTROL_H
#define _SHOOT_CONTROL_H
#include "stm32f4xx_hal.h"
#include "control_task.h"
//#include "tim.h"
#define RadioFrequency 1000
#define PWM1 TIM1->CCR1
#define PWM2 TIM1->CCR2
//#define Double_Loop_PID
#define SetFrictionWheelSpeed(x) \
	PWM1=x+1000;                          \
	PWM2=x+1000;

typedef enum
{
	STOP_Supply=0,
	Supply=1,
//	Just_Supply=2,
	error_Supply=3,
	
}supply_state_e;
typedef struct
{
	char Shoot;
	char Fill;
	char Error;
}supply_flag_t;
extern int Shoot_Speed;
extern float m;
extern float shoot_speed;// 180∂‘”¶15
extern int Shoot_Frequency;
//extern int supply_flag;
//extern int rotate_time;
extern int Count_Time;
extern supply_state_e Supply_State;
void Shoot_Control(void);
//void supply_error(void);
void Shoot_Power_Control(void);
void Ammunition_supply_Control(void);
extern float last_angle;
extern int Last_Shoot_Frequency;
extern int Count_time;
extern volatile int Little_angle;
extern supply_flag_t Supply_Flag;
extern myrampGen_t Friction_Ramp;
extern pid_t FrictionSpeedPID_L;
extern pid_t FrictionSpeedPID_R;
extern pid_t PID_Shoot_Speed;
extern pid_t PID_Shoot_Position;
#endif



