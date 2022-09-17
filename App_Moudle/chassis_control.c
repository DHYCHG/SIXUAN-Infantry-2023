#include "chassis_control.h"
#include <stdlib.h>


pid_t PID_Chassis_Rotate_Speed=PID_PARAM_DEFAULT;
pid_t PID_Chassis_Motor1_Speed=PID_PARAM_DEFAULT;
pid_t PID_Chassis_Motor2_Speed=PID_PARAM_DEFAULT;
pid_t PID_Chassis_Motor3_Speed=PID_PARAM_DEFAULT;
pid_t PID_Chassis_Motor4_Speed=PID_PARAM_DEFAULT;
myrampGen_t Rotate_Ramp = {0};
float Chassis_Rotate_Offset = 0;
Chassis_SportState_e chassisState = SPORT_STATE;

float chassis_rote_mote=120;
/**
 * @Name: SetChassisSportState
 * @Description: 设置底盘运动模式
 * @Param: 底盘运动模式,为枚举类型
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void SetChassisSportState(Chassis_SportState_e state)
{
	chassisState = state;
}
/**
 * @Name: GetChassisSportState
 * @Description: 获得底盘运动状态
 * @Param: void
 * @Return: 底盘运动状态
 * @Author: source
 * @Warning: void
 */
Chassis_SportState_e GetChassisSportState()
{
	return chassisState;
}


/**
 * @Name: Chassis_Power_Limit
 * @Description: 新底盘功率限制
 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: void
 */
float CHATS=150;//底盘偏转速度
float CHATY=1;//底盘输出限制
float Klimit=1;
float Plimit=0;

float Watch_X=0;
float Watch_Y=0;
float Watch_disage=0;
float Watch_Power_Max;
float Watch_Power;
float Watch_Buffer;
double Chassis_pidout;

double Scaling1=0,Scaling2=0,Scaling3=0,Scaling4=0;
static double Scaling1,Scaling2,Scaling3,Scaling4;
static uint16_t  Power_Max;
static float Power,Power_Buffer;
//	static float  Power_Maybe = 0.0f;
//	static float Power_Bufferlimit = 0.0f;
//	static double Chassis_pidout=0.0f;
 float Chassis_pidout_max;
myrampGen_t ROTATION_ramp={0};


void get_chassis_power_and_buffer(float *power, float *buffer,uint16_t *powmax)
{
	*power = referee_receive_data.jundge_power_heat_data.date.chassis_power;
	*buffer = (float)referee_receive_data.jundge_power_heat_data.date.chassis_power_buffer;
	*powmax = referee_receive_data.jundge_robot_state.data.chassis_power_limit;
}

void Chassis_VAL_LIMIT(int X)
{
		VAL_LIMIT(PID_Chassis_Motor1_Speed.out,-X,X);		
		VAL_LIMIT(PID_Chassis_Motor2_Speed.out,-X,X);		
		VAL_LIMIT(PID_Chassis_Motor3_Speed.out,-X,X);
		VAL_LIMIT(PID_Chassis_Motor4_Speed.out,-X,X);	
}

void Chassis_Power_Limit(void)
{	
	//819.2/A
	Watch_Power_Max=Klimit;	Watch_Power=Power;	Watch_Buffer=Power_Buffer;
	get_chassis_power_and_buffer(&Power, &Power_Buffer, &Power_Max);

	Chassis_pidout_max=65536;//32768，40，960
//	PID_Chassis_Motor1_Speed.f_calc(&PID_Chassis_Motor1_Speed);
//	PID_Chassis_Motor2_Speed.f_calc(&PID_Chassis_Motor2_Speed);	
//	PID_Chassis_Motor3_Speed.f_calc(&PID_Chassis_Motor3_Speed);	
//	PID_Chassis_Motor4_Speed.f_calc(&PID_Chassis_Motor4_Speed);	

	if(Power>960)	Chassis_VAL_LIMIT(4096);//5*4*24;
	else{
//	else if(Power_Buffer<=55){
		Chassis_pidout=(
						fabs(PID_Chassis_Motor1_Speed.set-PID_Chassis_Motor1_Speed.get)+
						fabs(PID_Chassis_Motor2_Speed.set-PID_Chassis_Motor2_Speed.get)+
						fabs(PID_Chassis_Motor3_Speed.set-PID_Chassis_Motor3_Speed.get)+
						fabs(PID_Chassis_Motor4_Speed.set-PID_Chassis_Motor4_Speed.get));

		Scaling1=(PID_Chassis_Motor1_Speed.set-PID_Chassis_Motor1_Speed.get)/Chassis_pidout;	
		Scaling2=(PID_Chassis_Motor2_Speed.set-PID_Chassis_Motor2_Speed.get)/Chassis_pidout;
		Scaling3=(PID_Chassis_Motor3_Speed.set-PID_Chassis_Motor3_Speed.get)/Chassis_pidout;	
		Scaling4=(PID_Chassis_Motor4_Speed.set-PID_Chassis_Motor4_Speed.get)/Chassis_pidout;
		Klimit=Chassis_pidout/1500;
		VAL_LIMIT(Klimit,-1,1);

		if(Power_Buffer<50&&Power_Buffer>=40)				Plimit=0.9;//15
		else if(Power_Buffer<40&&Power_Buffer>=35)	Plimit=0.75;
		else if(Power_Buffer<35&&Power_Buffer>=30)	Plimit=0.5;
		else if(Power_Buffer<30&&Power_Buffer>=20)	Plimit=0.25;
		else if(Power_Buffer<20&&Power_Buffer>=10)	Plimit=0.125;
		else if(Power_Buffer<10&&Power_Buffer>=0)		Plimit=0.05;
		else if(Power_Buffer==60)										Plimit=1;
		
		PID_Chassis_Motor1_Speed.out=Scaling1*Chassis_pidout_max*Klimit*Plimit;
		PID_Chassis_Motor2_Speed.out=Scaling2*Chassis_pidout_max*Klimit*Plimit;
		PID_Chassis_Motor3_Speed.out=Scaling3*Chassis_pidout_max*Klimit*Plimit;
		PID_Chassis_Motor4_Speed.out=Scaling4*Chassis_pidout_max*Klimit*Plimit;/*同比缩放电流*/

//		//PID_Chassis_Motor4_Speed.out = PID_Chassis_Motor1_Speed.out*(float)(Power_Buffer+20)/Power_Max*Chassis_pidout_max;//同比例放缩电流值
	}	
}

/**
 * @Name: Chassis_Init
 * @Description: 底盘相关数据初始化		
 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void Chassis_Init()
{
	ChassisSpeedRef.forward_back_ref = 0;
	ChassisSpeedRef.left_right_ref = 0;
	ChassisSpeedRef.rotate_ref = 0;
	Pid_Out_Clear(&PID_Chassis_Rotate_Speed);
	Pid_Out_Clear(&PID_Chassis_Motor1_Speed);
	Pid_Out_Clear(&PID_Chassis_Motor2_Speed);
	Pid_Out_Clear(&PID_Chassis_Motor3_Speed);
	Pid_Out_Clear(&PID_Chassis_Motor4_Speed);	
	SetChassisSportState(SPORT_STATE);
}

float rotate_power_scale = 0.003;
float rotate_count;
float rotation_speed=20;
float dis_angle=0;//static float dis_angle;
float X_MOVE_Ref = 0.0f;
float Y_MOVE_Ref = 0.0f;
int rotation_m = 150;
int rotation_n = 150;
int rancount = 0;
/**
 * @Name: Chassis_control
 * @Description: 底盘控制程序,在不同模式及运动状态下底盘以不同形式运动(未写完,待续)
 * 				
 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void Chassis_Control()
{
	if(GetWorkState()==PREPARE_STATE||GetWorkState()==STOP_STATE||GetWorkState()==CALI_STATE)
	{
		ChassisSpeedRef.forward_back_ref = 0;
		ChassisSpeedRef.left_right_ref = 0;
		ChassisSpeedRef.rotate_ref = 0;
		rotate_count = 0;
		
		Pid_Out_Clear(&PID_Chassis_Motor1_Speed);
		Pid_Out_Clear(&PID_Chassis_Motor2_Speed);
		Pid_Out_Clear(&PID_Chassis_Motor3_Speed);
		Pid_Out_Clear(&PID_Chassis_Motor4_Speed);
		SetChassisSportState(SPORT_STATE);
	}
	else
	{
		switch(GetGimbalAndChassisSportState())
		{
			case CHASSIS_FOLLOW_YAW:{//用yaw轴			
				switch(GetChassisSportState())
				{
					case SPORT_STATE:{
						rotate_count = 0;
						if((YawEncoder.ecd_angle > Yaw_Start_Encode_Angle + 360)||(YawEncoder.ecd_angle < Yaw_Start_Encode_Angle - 360))
						{
							YawEncoder.round_cnt = 0;//0
						
						}
						dis_angle = -(YawEncoder.ecd_angle - Yaw_Start_Encode_Angle);
						if(dis_angle<-180)
						{
							YawEncoder.round_cnt = -1;
						}
						
						PID_Chassis_Rotate_Speed.set = Yaw_Start_Encode_Angle;
						PID_Chassis_Rotate_Speed.get = YawEncoder.ecd_angle;
						ChassisSpeedRef.rotate_ref = PID_Chassis_Rotate_Speed.f_calc(&PID_Chassis_Rotate_Speed);
						//ChassisSpeedRef.rotate_ref*=1.25;
						VAL_LIMIT(ChassisSpeedRef.rotate_ref,-CHATS,CHATS);//令地盘旋转速度限制
						dis_angle = -(YawEncoder.ecd_angle - Yaw_Start_Encode_Angle);
						/*小陀螺如果无法以变云台坐标系正常移动，则disage值取反（正负符号）即可*/
						X_MOVE_Ref=(float)((cos(dis_angle/57.3f)*ChassisSpeedRef.forward_back_ref)-(sin(dis_angle/57.3f)*ChassisSpeedRef.left_right_ref));	
						Y_MOVE_Ref=(float)((sin(dis_angle/57.3f)*ChassisSpeedRef.forward_back_ref)+(cos(dis_angle/57.3f)*ChassisSpeedRef.left_right_ref));
					}break;
					case ROTATION_STATE: {
						rotation_speed = rand()%(rotation_n-rotation_m+1)+rotation_m;
						rotate_count = rotation_speed*(fabsf(ramp_calc(&Rotate_Ramp,0)));//0.5
						ChassisSpeedRef.rotate_ref = rotate_count;
						/*万向移动解算*/
						dis_angle = -(YawEncoder.ecd_angle - Yaw_Start_Encode_Angle);
						/*小陀螺如果无法以变云台坐标系正常移动，则disage值取反（正负符号）即可*/
						X_MOVE_Ref=(float)((cos(dis_angle/57.3f)*ChassisSpeedRef.forward_back_ref)-(sin(dis_angle/57.3f)*ChassisSpeedRef.left_right_ref));	
						Y_MOVE_Ref=(float)((sin(dis_angle/57.3f)*ChassisSpeedRef.forward_back_ref)+(cos(dis_angle/57.3f)*ChassisSpeedRef.left_right_ref));
					}break;
				}
			}break;
			
			case CHASSIS_NO_FOLLOW_YAW:{//不用yaw
				rotate_count = 0;
				dis_angle = -(YawEncoder.ecd_angle - Yaw_Start_Encode_Angle);
				/*小陀螺如果无法以变云台坐标系正常移动，则disage值取反（正负符号）即可*/
				X_MOVE_Ref=(float)((cos(dis_angle/57.3f)*ChassisSpeedRef.forward_back_ref)-(sin(dis_angle/57.3f)*ChassisSpeedRef.left_right_ref));	
				Y_MOVE_Ref=(float)((sin(dis_angle/57.3f)*ChassisSpeedRef.forward_back_ref)+(cos(dis_angle/57.3f)*ChassisSpeedRef.left_right_ref));
			};break;
			
			case Emergency_mode:{//应急模式，固定yaw轴，灵动pitch轴
				rotate_count = 0;
				VAL_LIMIT(ChassisSpeedRef.rotate_ref,-CHATS,CHATS);//令地盘旋转速度限制
				
				X_MOVE_Ref = ChassisSpeedRef.forward_back_ref; 
				Y_MOVE_Ref = ChassisSpeedRef.left_right_ref;
				ChassisSpeedRef.rotate_ref = GimbalRef.yaw_angle_dynamic_ref;	
			}	
		}
		/*
			2***头***1
			*				 *
			*				 *
			*				 *
			*				 *
			*				 *
			3********4
		*/
		PID_Chassis_Motor1_Speed.set = -X_MOVE_Ref + Y_MOVE_Ref + ChassisSpeedRef.rotate_ref;
		PID_Chassis_Motor2_Speed.set =	X_MOVE_Ref + Y_MOVE_Ref + ChassisSpeedRef.rotate_ref;
		PID_Chassis_Motor3_Speed.set =  X_MOVE_Ref - Y_MOVE_Ref + ChassisSpeedRef.rotate_ref;
		PID_Chassis_Motor4_Speed.set = -X_MOVE_Ref - Y_MOVE_Ref + ChassisSpeedRef.rotate_ref;

		PID_Chassis_Motor1_Speed.get = CM1Encoder.filter_rate;
		PID_Chassis_Motor2_Speed.get = CM2Encoder.filter_rate;
		PID_Chassis_Motor3_Speed.get = CM3Encoder.filter_rate;
		PID_Chassis_Motor4_Speed.get = CM4Encoder.filter_rate;

		PID_Chassis_Motor1_Speed.f_calc(&PID_Chassis_Motor1_Speed);
		PID_Chassis_Motor2_Speed.f_calc(&PID_Chassis_Motor2_Speed);	
		PID_Chassis_Motor3_Speed.f_calc(&PID_Chassis_Motor3_Speed);	
		PID_Chassis_Motor4_Speed.f_calc(&PID_Chassis_Motor4_Speed);		
	}
}
 
