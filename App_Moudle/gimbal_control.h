#ifndef _GIMBAL_CONTROL_H
#define _GIMBAL_CONTROL_H
#include "stm32f4xx_hal.h"
#include "remote_task.h"
#include "bsp_imu.h"
#include "my_ramp.h"
#include "canbus_task.h"
#include "pid.h"
//extern myrampGen_t Friction_Ramp;
extern myrampGen_t Pitch_ramp;
extern myrampGen_t Yaw_ramp;

extern pid_t PID_Pitch_Position;
extern pid_t PID_Pitch_Speed;
extern pid_t PID_Yaw_Position;
extern pid_t PID_Yaw_Speed;
extern int	LED_T_1;
extern int imu_flag;
extern int reset_gimbal_angle;
/*gyro=1,encoder=2,vision=3*/

float Get_Yaw(void);
void Gimbal_Control(void);

extern void Servo_Control(int state);

#endif



