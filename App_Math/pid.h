#ifndef _PID_H
#define _PID_H
#include "stm32f4xx_hal.h"
#define PID_PARAM_DEFAULT \
{\
  0,\
  0,\
  0,\
  0,\
  0,\
  {0,0,0},\
  0,\
  0,\
  0,\
  0,\
  0,\
  0,\
}

enum
{
	LLAST,
	LAST,
	NOW,
	POSITION_PID,
	DELTA_PID,
};
typedef struct pid_t
{
	float p;
	float i;
	float d;
	
	float set;
	float get;
	float err[3];
	
	float pout;
	float iout;
	float dout;
	float out;
	
	float input_max_err;
	float output_deadhand;
	
	uint32_t pid_mode;
	uint32_t max_out;
	uint32_t integral_limit;
	void (*f_param_init)(struct pid_t *pid,
		uint32_t pid_mode,
		uint32_t max_output,
		uint32_t intel_limit,
		float p,
		float i,
		float d);
	void (*f_pid_reset)(struct pid_t *pid,float p,float i,float d);
	float (*f_calc)(struct pid_t *pid);
}pid_t;

void PID_struct_init(pid_t *pid,
	uint32_t mode,
	uint32_t maxout,
	uint32_t intergral_limit,
	float kp,
	float ki,
	float kd);
void abs_limit(float *a,float ABS_MAX);
float pid_calc(pid_t *pid,float get,float set);
void Pid_Out_Clear( pid_t*   pid);
#endif

