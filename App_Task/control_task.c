#include "control_task.h"
#include "bsp_buzzer.h"
#include <math.h>

WorkState_e last_workState = STOP_STATE;
WorkState_e workState = STOP_STATE;
GimbalAndChassis_SportState_e gimbalandchassisState = CHASSIS_FOLLOW_YAW;
volatile int time_tick_1ms=0;
float Yaw_Start_Encode_Angle = 0 , Pitch_Start_Encode_Angle =0;

extern pid_t PID_IMU_Tmp;

/**
 * @Name: SetWorkMode
 * @Description: 设置工作模式
 * @Param: 工作模式,为枚举类型
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void SetWorkMode(WorkState_e state)
{
	workState = state;
}
/**
 * @Name: GetWorkState
 * @Description: 获得工作状态
 * @Param: void
 * @Return: 工作状态
 * @Author: source
 * @Warning: void
 */
WorkState_e GetWorkState()
{
	return workState;
}

/**
 * @Name: SetGimbalAndChassisSportState
 * @Description: 设置云台与底盘之间的运动模式
 * @Param: 云台与底盘之间的运动模式,为枚举类型
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void SetGimbalSportState(GimbalAndChassis_SportState_e state)
{
	gimbalandchassisState = state;
}

/**
 * @Name: GetGimbalAndChassisSportState
 * @Description: 获得云台与底盘之间的运动状态
 * @Param: void
 * @Return: 云台与底盘之间的运动状态
 * @Author: source
 * @Warning: void
 */
GimbalAndChassis_SportState_e GetGimbalAndChassisSportState()
{
	return gimbalandchassisState;
}

uint16_t Prepare_Time_MS = 3000;    //prapare time in ms
extern uint8_t FastMode;
/**
* @Name: WorkStateFSM
* @Description: 工作模式状态机,负责四种工作模式的切换
* @Param: void
* @Return: void
* @Author: source
* @Warning: void
*/
void WorkStateFSM()
{
	(FastMode)?(Prepare_Time_MS = 0):(Prepare_Time_MS = 3000);
	switch(workState)
	{
		case PREPARE_STATE: {
			if(GetInputMode()==STOP || Is_Remote_Online==0)
				workState = STOP_STATE;
			else if(time_tick_1ms>Prepare_Time_MS)
					workState = NORMAL_STATE;/*四秒上电*/
		};break;
		case NORMAL_STATE:{
			if(GetInputMode()==STOP || Is_Remote_Online==0)
				workState = STOP_STATE;
		};break;
		case STOP_STATE:{
			if(GetInputMode()!=STOP && Is_Remote_Online==1)/*非停止模式，并且上电校准过*/
			{/*模式重新初始化*/
				workState = PREPARE_STATE;
				time_tick_1ms = 0;
			}
		};break;
		default:
		{
			
		}
	}
}

int16_t Pitch_Cali_Out = 8000;
float PitchScale = 0.47f;
float low_angle_limit = 0.0f, high_angle_limit = 0.0f;
extern uint8_t FastMode;
/**
* @Name: Pitch_Calibration
* @Description: Pitch电机初始角设置
* @Param: void
* @Return: void
* @Author: DHY 2022-09-17
* @Warning: void
*/
void Pitch_Calibration()
{
	float low_motor_angle = 0.0f, high_motor_angle = 0.0f;
	int16_t pitch_out;
	int count = 300, i = 0;
	
	if(!FastMode)
		buzzer_on(1, 60000);
	
	SetCanState(CAN_NORMAL);
	HAL_Delay(10);
	
	//获取云台最低角度
	pitch_out = Pitch_Cali_Out;
	i = 0;
	while(i<count)
	{
		Pitch_Motor_Out(pitch_out);
		HAL_Delay(1);
		i++;
	}
	low_motor_angle = PitchEncoder.ecd_angle;
	
	//获取云台最高角度
	pitch_out = -Pitch_Cali_Out;
	i=0;
	while(i<count)
	{
		Pitch_Motor_Out(pitch_out);
		HAL_Delay(1);
		i++;
	}
	high_motor_angle = PitchEncoder.ecd_angle;
	
	Pitch_Motor_Stop();
	SetCanState(CAN_PREPARE);
	
	Pitch_Start_Encode_Angle = (low_motor_angle + high_motor_angle)*PitchScale;
	//while(1);
	low_angle_limit = low_motor_angle - Pitch_Start_Encode_Angle;
	high_angle_limit = high_motor_angle - Pitch_Start_Encode_Angle;
	
	if(!FastMode)
	{
		//校准结束提示音
		buzzer_on(1, 30000);
		MPU_DELAY(50);
		buzzer_off();
		MPU_DELAY(50);
		
		buzzer_on(1, 30000);
		MPU_DELAY(50);
		buzzer_off();
		MPU_DELAY(50);
		
		buzzer_on(1, 30000);
		MPU_DELAY(50);
		buzzer_off();
		MPU_DELAY(50);
	}

}

void control_task_init()
{
	time_tick_1ms = 0;
	//设置工作模式
	SetWorkMode(STOP_STATE);
	//斜坡初始化
	my_ramp_init(&Rotate_Ramp,800,mode0to1);
	my_ramp_init(&Friction_Ramp,1000,mode0to1);
	my_ramp_init(&Pitch_ramp,1000,mode0to1);
	my_ramp_init(&Yaw_ramp,800,mode0to1);
		
	Pitch_Calibration();
	//云台给定角度初始化
	Yaw_Start_Encode_Angle=91.0f;
	//Pitch_Start_Encode_Angle=-208.0f;
	//Pitch2_Start_Encode_Angle=141.416;
		//激光初始化，未测试
//	HAL_GPIO_WritePin(LASER_CTRL_GPIO_Port, LASER_CTRL_Pin, GPIO_PIN_SET);

	/************************参数调整*****************************************P*******I*******D****/
	/*陀螺仪*/
	PID_struct_init(&PID_Pitch_Position      	, POSITION_PID,	5000	, 500		,	20  	,	0  		, 0);
	PID_struct_init(&PID_Pitch_Speed         	, POSITION_PID,	20000 , 3000	, 200		,	0  		, 0);
	PID_struct_init(&PID_Yaw_Position        	, POSITION_PID,	6000	, 6000	, 20 		, 0  		, 0);
  PID_struct_init(&PID_Yaw_Speed           	, POSITION_PID,	30000	, 3000	, 200		, 0			, 0);
	PID_struct_init(&PID_Chassis_Motor1_Speed	, POSITION_PID, 15000	, 2000	, 100		, 0			, 0);
	PID_struct_init(&PID_Chassis_Motor2_Speed	, POSITION_PID, 15000	, 2000	, 100		, 0			, 0);
	PID_struct_init(&PID_Chassis_Motor3_Speed	, POSITION_PID, 15000	, 2000	, 100		, 0			, 0);
	PID_struct_init(&PID_Chassis_Motor4_Speed	, POSITION_PID, 15000	, 2000	, 100		, 0			, 0);
	PID_struct_init(&PID_Chassis_Rotate_Speed	, POSITION_PID, 300  	, 20  	,	5			,	0 		, 0);
  PID_struct_init(&PID_Shoot_Speed         	, POSITION_PID, 10000	, 2000	,	450		,	0  		, 0);
	PID_struct_init(&FrictionSpeedPID_R				,	POSITION_PID,	10000	,	1000	,	0			,	0			, 0);
	PID_struct_init(&FrictionSpeedPID_L				,	POSITION_PID,	10000	,	1000	,	0			,	0			, 0);
	PID_struct_init(&PID_IMU_Tmp							,	POSITION_PID,	4500	,	4400	,	1600	,	0.002	, 0);
	
}


