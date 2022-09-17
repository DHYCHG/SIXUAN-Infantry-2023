/*
 * @Name: pid
 * @Description: pid�ĳ�ʼ���͹��ܺ���
 * @Author: source
 * @Copyright: SixuanRobomasterLab
 */
#include "pid.h"
#include "math.h"
/**
 * @Name: abs_limit
 * @Description: ����ֵ�ľ���ֵ�����ֵ�޶�����
 * @Param: a����ֵ,ABS_MAX���ֵ
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
 * @Description: pidʹ��ʱ��ʼ������
 * @Param: pid�ĸ��ֲ���
 * @Return: void
 * @Author: source
 * @Warning: ֻ�ڱ��ļ�����Ч
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
 * @Description: ��������ʱ�޸�PID����
 * @Param: ���õ�pid�Լ����Ĳ���
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
 * @Description: ��������PID��λ��PID
 * @Param:����PID�����������Լ�����
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
 * @Description: new��������PID��λ��PID
 * @Param:����PID�����������Լ�����
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
 * @Description: ��ʼ��PID����
 * @Param: ����pid,����,������,������,P,i,d
 * @Return: void
 * @Author: source
 * @Warning: ������Ϊ�����ֵ�����
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
 * @Description: ����PID����
 * @Param: ����pid
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


