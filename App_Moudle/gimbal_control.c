#include "gimbal_control.h"
#include "tim.h"
#include "chassis_control.h"
pid_t PID_Pitch_Position = PID_PARAM_DEFAULT;
pid_t PID_Pitch_Speed = PID_PARAM_DEFAULT;
pid_t PID_Yaw_Position = PID_PARAM_DEFAULT;
pid_t PID_Yaw_Speed = PID_PARAM_DEFAULT;
int LED_T_1=13;
float gimbal_pid_change_1=0,gimbal_pid_change_2=0,gimbal_pid_change_3=0;//�����ǡ����������Ӿ�ģʽPID
/**
 * @Name: Servo_Control
 * @Description: ���ոǿ���
 * @Param: state 1����0��
 * @Return: none
 * @Author: source
 * @Warning: void
 */
void Servo_Control(int state)
{
	if(state)
	{
		TIM1->CCR1=7500;
		GimbalRef.pitch_angle_dynamic_ref = 10; // ����Pitchˮƽ
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
 * @Description: ��ü�����������yaw��Ƕ�ֵ,����������ֵת��Ϊ������ֵ,���������ע��
 * @Param: void
 * @Return: һ��������yaw��ĽǶ�ֵ
 * @Author: source
 * @Warning: void
 */
float Get_Yaw()
{
	static int temp_yaw_count = 0;
	static float temp_yaw = 0;
	static float temp_last_yaw = 0;
	
  if(GetGimbalAndChassisSportState()==Emergency_mode)
	{ //ֱ�Ӷ�ȡ�����ĽǶ�ֵ
	 temp_yaw = YawEncoder.ecd_angle; 
		return temp_yaw;
	}
	
  else{
		temp_last_yaw = temp_yaw;
		temp_yaw = imu.yaw; 							 //���������yaw��Ƕ�ֵ
		if(temp_yaw - temp_last_yaw > 330) //���yaw��ֵ�Ƿ�ͻ��,����-180����180(Ȧ����һ),���180����-180ʱ(Ȧ����һ)
			temp_yaw_count--;
		if(temp_yaw - temp_last_yaw < -330) 
			temp_yaw_count++;
		return (temp_yaw+temp_yaw_count*360);
	}
}

/**
 * @Name: Pitch_Control
 * @Description: ǹ�ܸ�����(pitch)���ƺ���,˫��PID
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
	 {//Ӧ��ģʽ
		 
		PID_Pitch_Position.set=Pitch_Start_Encode_Angle+GimbalRef.pitch_angle_dynamic_ref;/* ����ֵ= ���ı������Ƕ�+pitch��̨��̬�ǶȲ���*/
		PID_Pitch_Position.get=PitchEncoder.ecd_angle;/*����ֵ= pitch��̨�������Ƕ�*/
		PID_Pitch_Position.f_calc(&PID_Pitch_Position);/*����pitch��̨�� λ�û�PID*/
		 
		PID_Pitch_Speed.set=PID_Pitch_Position.out;/*����ֵΪ ��pitch��̨����ĽǶ�PID������*/
		PID_Pitch_Speed.get=PitchEncoder.filter_rate*5;/*����ֵ= imu�������� y�ᣩ��ȡ������*/
		PID_Pitch_Speed.f_calc(&PID_Pitch_Speed);/*����pitch��̨ �ٶȻ�PID*/
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
 * @Description: ��̨��ת���ƣ�yaw�ᣩ������˫��PID����
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
//����������
		 PID_Yaw_Position.set=Yaw_Start_Encode_Angle;
		 PID_Yaw_Position.get=YawEncoder.ecd_angle;
		 PID_Yaw_Position.f_calc(&PID_Yaw_Position);/*����yaw��̨ λ�û�PID*/
		/*yaw��̨�� �ٶȻ�PID */
		PID_Yaw_Speed.set=PID_Yaw_Position.out;/*����ֵ= ��yaw��̨����� λ�û�PID�ĸ�����*/
		PID_Yaw_Speed.get= YawEncoder.filter_rate*5;/*����ֵ= ��������ȡ������*/
		PID_Yaw_Speed.f_calc(&PID_Yaw_Speed);/*����Yaw��̨ �ٶȻ�PID*/
	 }
	else
	{
//	�����ǿ���
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
//			/*��С���ݵĲ���*/
////		if(PitchEncoder.raw_value<1200)
////			PitchEncoder.round_cnt=0;
////		else if(PitchEncoder.raw_value>7000)
////			PitchEncoder.round_cnt=-1;
//		if(PitchEncoder.raw_value<3000)
//			PitchEncoder.round_cnt=0;
//		else if(PitchEncoder.raw_value>=3000)
//			PitchEncoder.round_cnt=0;
//		/*��С���ݵĲ���*/
//		if(YawEncoder.raw_value<=3000)
//			YawEncoder.round_cnt=0;
//		else if(YawEncoder.raw_value>3000)
//			YawEncoder.round_cnt=-1;
//	}
}



