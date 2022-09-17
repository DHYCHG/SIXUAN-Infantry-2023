#include "gimbal_control.h"
#include "tim.h"
#include "chassis_control.h"
pid_t PID_Pitch_Position = PID_PARAM_DEFAULT;
pid_t PID_Pitch_Speed = PID_PARAM_DEFAULT;
pid_t PID_Yaw_Position = PID_PARAM_DEFAULT;
pid_t PID_Yaw_Speed = PID_PARAM_DEFAULT;
int LED_T_1=13;
float gimbal_pid_change_1=0,gimbal_pid_change_2=0,gimbal_pid_change_3=0;//陀螺仪、编码器、视觉模式PID
/**
 * @Name: Servo_Control
 * @Description: 弹舱盖控制
 * @Param: state 1开，0关
 * @Return: none
 * @Author: source
 * @Warning: void
 */
void Servo_Control(int state)
{
	if(state)
	{
		TIM1->CCR1=7500;
		GimbalRef.pitch_angle_dynamic_ref = 10; // 保持Pitch水平
	}
	else
		TIM1->CCR1=2000;
}	
myrampGen_t Pitch_ramp={0};
myrampGen_t Yaw_ramp={0};

int imu_flag=1;
int ignore_time=0;

/**
 * @Name: Get_Yaw
 * @Description: 获得及处理陀螺仪yaw轴角度值,将不连续的值转换为连续的值,详情见函数注释
 * @Param: void
 * @Return: 一个连续的yaw轴的角度值
 * @Author: source
 * @Warning: void
 */
float Get_Yaw()
{
	static int temp_yaw_count = 0;
	static float temp_yaw = 0;
	static float temp_last_yaw = 0;
	
  if(GetGimbalAndChassisSportState()==Emergency_mode)
	{ //直接读取解码后的角度值
	 temp_yaw = YawEncoder.ecd_angle; 
		return temp_yaw;
	}
	
  else{
		temp_last_yaw = temp_yaw;
		temp_yaw = imu.yaw; 							 //获得陀螺仪yaw轴角度值
		if(temp_yaw - temp_last_yaw > 330) //检测yaw的值是否突变,即从-180变至180(圈数减一),或从180变至-180时(圈数加一)
			temp_yaw_count--;
		if(temp_yaw - temp_last_yaw < -330) 
			temp_yaw_count++;
		return (temp_yaw+temp_yaw_count*360);
	}
}

/**
 * @Name: Pitch_Control
 * @Description: 枪管俯仰角(pitch)控制函数,双环PID
 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void Pitch_Control()
{		
		float pitch_set_angle;
		GimbalAngleLimit();
   if(GetGimbalAndChassisSportState()==Emergency_mode)
	 {//应急模式
		 
		PID_Pitch_Position.set=Pitch_Start_Encode_Angle+GimbalRef.pitch_angle_dynamic_ref;/* 期望值= 它的编码器角度+pitch云台动态角度参量*/
		PID_Pitch_Position.get=PitchEncoder.ecd_angle;/*反馈值= pitch云台编码器角度*/
		PID_Pitch_Position.f_calc(&PID_Pitch_Position);/*计算pitch云台的 位置环PID*/
		 
		PID_Pitch_Speed.set=PID_Pitch_Position.out;/*期望值为 “pitch云台输出的角度PID增量”*/
		PID_Pitch_Speed.get=PitchEncoder.filter_rate*5;/*反馈值= imu（陀螺仪 y轴）获取的数据*/
		PID_Pitch_Speed.f_calc(&PID_Pitch_Speed);/*计算pitch云台 速度环PID*/
	 }
		
		else {
			PID_Pitch_Position.set = Pitch_Start_Encode_Angle + GimbalRef.pitch_angle_dynamic_ref;
			PID_Pitch_Position.get = PitchEncoder.ecd_angle;
			PID_Pitch_Position.f_calc(&PID_Pitch_Position);

			PID_Pitch_Speed.set = PID_Pitch_Position.out;
			PID_Pitch_Speed.get = -imu.wy * 57.3f;
			PID_Pitch_Speed.f_calc(&PID_Pitch_Speed);
		}
	
//	last_aim = Auto_Aim;
}

/**
 * @Name: Yaw_Control
 * @Description: 云台旋转控制（yaw轴）函数，双环PID控制
 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: void
 */
float Imu_wz,a;
float last_pget, pget, b;
float Gimbal_Yaw_Offset=0;
void Yaw_Control()
{
   if(GetGimbalAndChassisSportState()==Emergency_mode)
	 {
//编码器控制
		 PID_Yaw_Position.set=Yaw_Start_Encode_Angle;
		 PID_Yaw_Position.get=YawEncoder.ecd_angle;
		 PID_Yaw_Position.f_calc(&PID_Yaw_Position);/*计算yaw云台 位置环PID*/
		/*yaw云台的 速度环PID */
		PID_Yaw_Speed.set=PID_Yaw_Position.out;/*期望值= “yaw云台输出的 位置环PID的负数”*/
		PID_Yaw_Speed.get= YawEncoder.filter_rate*5;/*反馈值= 编码器获取的数据*/
		PID_Yaw_Speed.f_calc(&PID_Yaw_Speed);/*计算Yaw云台 速度环PID*/
	 }
	else
	{
//	陀螺仪控制
	//Vision_Clear();
		PID_Yaw_Position.set = GimbalRef.yaw_angle_dynamic_ref;
		PID_Yaw_Position.get = (Get_Yaw()-Gimbal_Yaw_Offset);
		PID_Yaw_Position.f_calc(&PID_Yaw_Position);
		
		PID_Yaw_Speed.set = -PID_Yaw_Position.out;
		PID_Yaw_Speed.get = imu.wz * 57.3f;
		PID_Yaw_Speed.f_calc(&PID_Yaw_Speed);
	}
}

int reset_gimbal_angle=0;
void Gimbal_Control()
{
	if(GetWorkState()==STOP_STATE)
	{
		Pid_Out_Clear(&PID_Yaw_Position);
		Pid_Out_Clear(&PID_Yaw_Speed);
		Pid_Out_Clear(&PID_Pitch_Position);
		Pid_Out_Clear(&PID_Pitch_Speed);
		
		ramp_clear(&Pitch_ramp);
		ramp_clear(&Yaw_ramp);		
	}
	if(GetWorkState()==PREPARE_STATE)
	{
		if((time_tick_1ms==2980)&&(GetInputMode()==KEY_MOUSE_INPUT))	
		{	 
			Mpu_Device_Init();
		}
		Gimbal_Yaw_Offset = Get_Yaw();
		GimbalRef.yaw_angle_dynamic_ref=0;
	}
	if(GetWorkState()==NORMAL_STATE)
	{
		Pitch_Control();
		Yaw_Control();
	}
//	if(can_count>1000&&can_count<2000){
//			/*有小陀螺的步兵*/
////		if(PitchEncoder.raw_value<1200)
////			PitchEncoder.round_cnt=0;
////		else if(PitchEncoder.raw_value>7000)
////			PitchEncoder.round_cnt=-1;
//		if(PitchEncoder.raw_value<3000)
//			PitchEncoder.round_cnt=0;
//		else if(PitchEncoder.raw_value>=3000)
//			PitchEncoder.round_cnt=0;
//		/*无小陀螺的步兵*/
//		if(YawEncoder.raw_value<=3000)
//			YawEncoder.round_cnt=0;
//		else if(YawEncoder.raw_value>3000)
//			YawEncoder.round_cnt=-1;
//	}
}



