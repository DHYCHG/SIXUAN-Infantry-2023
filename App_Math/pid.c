/*
 * @Name: pid
 * @Description: pid的初始化和功能函数
 * @Author: source
 * @Copyright: SixuanRobomasterLab
 */
#include "pid.h"
#include "math.h"
/**
 * @Name: abs_limit
 * @Description: 输入值的绝对值的最大值限定函数
 * @Param: a输入值,ABS_MAX最大值
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void abs_limit(float *a,float ABS_MAX)
{
	if(*a>ABS_MAX)
		*a=ABS_MAX;
	if(*a<-ABS_MAX)
		*a=-ABS_MAX;
}
/**
 * @Name:pid_param_init 
 * @Description: pid使用时初始化函数
 * @Param: pid的各种参数
 * @Return: void
 * @Author: source
 * @Warning: 只在本文件内有效
 */
static void pid_param_init(pid_t *pid,
	uint32_t pid_mode,
	uint32_t max_output,
	uint32_t intel_limit,
	float kp,
	float ki,
	float kd)
{
	pid->pid_mode = pid_mode;
	pid->max_out = max_output;
	pid->integral_limit = intel_limit;
	
	pid->p = kp;
	pid->i = ki;
	pid->d = kd;
}
/**
 * @Name: pid_reset
 * @Description: 代码运行时修改PID参数
 * @Param: 所用的pid以及它的参数
 * @Return: void
 * @Author: source
 * @Warning: void
 */
static void pid_reset(pid_t *pid,float kp,float ki,float kd)
{
	pid->p = kp;
	pid->i = ki;
	pid->d = kd;
	
	pid->pout = 0;
	pid->iout = 0;
	pid->dout = 0;
}
/**
 * @Name: pid_calc
 * @Description: 计算增量PID和位置PID
 * @Param:所设PID和它的期望以及反馈
 * @Return: void
 * @Author: source
 * @Warning: void
 */
float pid_calc(pid_t *pid,float get,float set)
{
	pid->set = set;
	pid->get = get;
	pid->err[NOW] = set - get;
	if((pid->input_max_err!=0)&&fabs(pid->err[NOW])>pid->input_max_err)
		return 0;
	if(pid->pid_mode == POSITION_PID)
	{
		pid->pout = pid->p * pid->err[NOW];
		pid->iout += pid->i * pid->err[NOW];
		pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);
		
		abs_limit(&pid->iout,pid->integral_limit);
		pid->out = pid->pout + pid->iout + pid->dout;
		abs_limit(&pid->out,pid->max_out);
	}
	if(pid->pid_mode == DELTA_PID)
	{
		pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
		pid->iout = pid->i * pid->err[NOW];
		pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
	
		pid->out = pid->pout + pid->iout + pid->dout;
		abs_limit(&pid->out,pid->max_out);	
	}
	pid->err[LAST] = pid->err[NOW];
	pid->err[LLAST] = pid->err[LAST];
	if(pid->output_deadhand != 0&&fabs(pid->out)<pid->output_deadhand)
		return 0;
	else 
		return pid->out;
}
/**
 * @Name: PID_Calc
 * @Description: new计算增量PID和位置PID
 * @Param:所设PID和它的期望以及反馈
 * @Return: void
 * @Author: source
 * @Warning: void
 */
float PID_Calc(pid_t *pid)
{
	pid->err[NOW] = pid->set - pid->get;
	if((pid->input_max_err!=0)&&fabs(pid->err[NOW])>pid->input_max_err)
		return 0;
	
	pid->pout = pid->p * pid->err[NOW];
	pid->iout += pid->i * pid->err[NOW];
	pid->dout = pid->d * (pid->err[NOW] -  pid->err[LAST]);
	
	abs_limit(&pid->iout,pid->integral_limit);
	pid->out = pid->pout + pid->iout + pid->dout;
	abs_limit(&pid->out,pid->max_out);
	
	if(pid->output_deadhand !=0&&fabs(pid->out)<pid->output_deadhand)
		return 0;
	else 
		return pid->out;
}
/**
 * @Name: PID_struct_init
 * @Description: 初始化PID参数
 * @Param: 所设pid,类型,最大输出,最大积分,P,i,d
 * @Return: void
 * @Author: source
 * @Warning: 最大输出为最大积分的俩倍
 */
void PID_struct_init(pid_t *pid,
	uint32_t mode,
	uint32_t maxout,
	uint32_t intergral_limit,
	float kp,
	float ki,
	float kd)
{
	pid->f_param_init = pid_param_init;
	pid->f_pid_reset = pid_reset;
	pid->f_calc = PID_Calc;
	pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
	pid->f_pid_reset(pid,kp,ki,kd);
}
/**
 * @Name: Pid_Out_Clear
 * @Description: 清零PID参数
 * @Param: 所设pid
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void Pid_Out_Clear( pid_t*   pid)
{
	pid->set = 0;
	pid->get = 0;
	pid->err[NOW] = pid->err[LAST] = pid->err[LLAST] = 0;
	pid->pout = 0;
	pid->iout = 0;
	pid->dout = 0;
	pid->out = 0;
}


