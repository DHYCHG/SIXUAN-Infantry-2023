#include "remote_task.h"
#include "shoot_control.h"
#include "tim.h"

#define DEADZONE 5

RC_Ctrl_t RC_CtrlData;
InputMode_e InputMode;
Gimbal_Ref_t GimbalRef;
myrampGen_t W_Ramp={0};
myrampGen_t S_Ramp={0};
myrampGen_t KeyA_Ramp={0};
myrampGen_t KeyD_Ramp={0};
myrampGen_t KeyQ_Ramp={0};
myrampGen_t KeyE_Ramp={0};
ChassisSpeed_Ref_t ChassisSpeedRef;
int servo_flag;
// FrictionWheelState_e Friction_Wheel_State = FRICTION_WHEEL_OFF;
/**
 * @Name:IsRemoteBeingAction 
 * @Description: ������ľ���ֵ�Ƿ��������
 * @Param: void
 * @Return: �Ǿͷ���1���Ƿ���0
 * @Author: source
 * @Warning: void
 */
uint8_t IsRemoteBeingAction(void)
{
	return (fabs(ChassisSpeedRef.forward_back_ref)>=10 || fabs(ChassisSpeedRef.left_right_ref)>=10 || fabs(GimbalRef.yaw_speed_ref)>=10 || fabs(GimbalRef.pitch_speed_ref)>=10);
}

/**
 * @Name: SetInputMode
 * @Description:ң������������ģʽ 
 * @Param: ң��������
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void SetInputMode(Remote *rc)
{
	switch(rc->s2)
	{
		case 2: InputMode = REMOTE_INPUT; break;
		case 3: InputMode = STOP; break;
		case 1: InputMode = KEY_MOUSE_INPUT; break;
	}

}
/**
 * @Name:RemoteShootControl
 * @Description: ң�����������
 * @Param: ң������ֵ
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void RemoteShootControl(RemoteSwitch_t *sw, uint8_t val) 
{


}
/**
 * @Name:MouseShootControl 
 * @Description: ���������
 * @Param: ������
 * @Return: void
 * @Author: source
 * @Warning: void
 */	 
void MouseShootControl(Mouse *mouse)
{

	if(mouse->press_l == 1)
	{
		Supply_Flag.Shoot = 1;
		Shoot_Frequency=7;
	}
	
	else if(mouse->press_r == 1)
	{
		Supply_Flag.Shoot = 1;
		Shoot_Frequency=12;
	}
	
	else
	{
		Shoot_Frequency=0;
	}
}



/**
 * @Name:RemoteDataPrcess 
 * @Description: ң����,����,�����Ϣ��ȡ
 * @Param: *pData
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void RemoteDataPrcess(uint8_t *pData)
{
	Tick_Controller.REMOTE = HAL_GetTick();
	RC_CtrlData.rc.ch0 = ((int16_t)(pData[0] | pData[1]<<8)) & 0x7FF;
	RC_CtrlData.rc.ch1 = (int16_t)(pData[1]>>3 | pData[2]<<5) & 0x7FF;
	RC_CtrlData.rc.ch2 = (int16_t)(pData[2]>>6 | pData[3]<<2|pData[4]<<10) & 0x7FF;
	RC_CtrlData.rc.ch3 = (int16_t)(pData[4]>>1 | pData[5]<<7) & 0x7FF;	
	RC_CtrlData.rc.ch4 = (pData[16] | (pData[17] << 8)) & 0x07FF; 
	RC_CtrlData.rc.s1 = (pData[5]>>4 & 0x000C) >> 2;
	RC_CtrlData.rc.s2 = pData[5]>>4 & 0x0003;
	
  RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
  RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
  RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);    

  RC_CtrlData.mouse.press_l = pData[12];
  RC_CtrlData.mouse.press_r = pData[13];
  RC_CtrlData.key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);	
	SetInputMode(&RC_CtrlData.rc);
	switch(GetInputMode())
	{
		case REMOTE_INPUT: RemoteControlProcess(&RC_CtrlData.rc); break;
		case STOP: break;
		case KEY_MOUSE_INPUT: MouseKeyControlProcess(&RC_CtrlData.mouse,&RC_CtrlData.key); break;
	}
	
}
InputMode_e GetInputMode()
{
	return InputMode;
}


/**
  * @brief          ң�����������жϣ���Ϊң�����Ĳ�������λ��ʱ�򣬲�һ��Ϊ0��
  * @param          �����ң����ֵ
  * @param          ��������������ң����ֵ
  * @param          ����ֵ
  */
int16_t rc_deadband_limit(int16_t input, int16_t dealine)        
{        
	int16_t output;	
	if ((input) > (dealine) || (input) < -(dealine)) 
	{                                                
			(output) = (input);                          
	}                                                
	else                                             
	{                                                
			(output) = 0;                                
	} 
	
	return output;
}

/**
 * @Name: RemoteControlProcess
 * @Description: ң��������ģʽ����
 * @Param: ң��������
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void RemoteControlProcess(Remote *rc)
{
	int16_t input_tmp = 0; //ҡ��ֵ��ʱ����
		//���̴���
	if(GetWorkState()!=NORMAL_STATE)
	{
		ChassisSpeedRef.forward_back_ref =0;
		ChassisSpeedRef.left_right_ref =0;
	}
	if(GetWorkState() == NORMAL_STATE)
	{
		input_tmp = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
		ChassisSpeedRef.forward_back_ref = rc_deadband_limit(input_tmp, DEADZONE)*0.0015151515 * CHASSIS_MAX_SPEED;
		
		input_tmp = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
		ChassisSpeedRef.left_right_ref = rc_deadband_limit(input_tmp, DEADZONE)*0.0015151515	* CHASSIS_MAX_SPEED; 
		
		input_tmp = (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
		GimbalRef.pitch_angle_dynamic_ref += rc_deadband_limit(input_tmp, DEADZONE)*0.0015151515 * PITCH_MAX_SPEED/71.43;//71.43hz
		GimbalAngleLimit();	
		
		if(GetGimbalAndChassisSportState()==CHASSIS_FOLLOW_YAW)
		{
			input_tmp = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
			GimbalRef.yaw_angle_dynamic_ref  += rc_deadband_limit(input_tmp, DEADZONE)*0.0015151515 * YAW_MAX_SPEED/71.43; 			
		}
		else
		{
			input_tmp = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
			GimbalRef.yaw_angle_dynamic_ref  += rc_deadband_limit(input_tmp, DEADZONE)*0.0015151515 * YAW_MAX_SPEED/71.43;   
			//ChassisSpeedRef.rotate_ref = 0;
			
			input_tmp = (rc->ch4 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
			ChassisSpeedRef.rotate_ref = -rc_deadband_limit(input_tmp, DEADZONE)*0.0015151515 * CHASSIS_ROTATION_MAX_SPEED; 
		}
	}

	if(rc->s1==1)
	{	
		shoot_speed = 0;//140/3M
		Supply_Flag.Shoot=0;
		SetGimbalSportState(CHASSIS_NO_FOLLOW_YAW);
		SetChassisSportState(SPORT_STATE);
	}
	else if(rc->s1==3)
	{
		SetGimbalSportState(CHASSIS_FOLLOW_YAW);
		SetChassisSportState(SPORT_STATE);
    Supply_Flag.Shoot=0; 
		shoot_speed = 0;//140/3M
	}
	else if(rc->s1==2)
	{
		SetGimbalSportState(CHASSIS_FOLLOW_YAW);
		SetChassisSportState(ROTATION_STATE);
		Supply_Flag.Shoot=0;
		Shoot_Frequency=0;	
	}		
}
/**
 * @Name: MouseKeyControlProcess
 * @Description: �����̿���ģʽ����
 * @Param: ���Ĳ���,���̵Ĳ���
 * @Return: void
 * @Author: source
 * @Warning: void
 */
int simpel_intt=0;
float Q=0.0f,W=0.0f,E=0.0f;
void MouseKeyControlProcess(Mouse *mouse, Key *key)
{
	//static int servo_flag;
	static uint16_t forward_back_speed = 0;
	static uint16_t left_right_speed = 0;
	if(GetWorkState()!=PREPARE_STATE)
	{
		if(simpel_intt<1)
		{//��ʼ��ģʽ
			SetGimbalSportState(CHASSIS_FOLLOW_YAW);
			SetChassisSportState(SPORT_STATE);
			simpel_intt++;
		}
					
		if(key->v & KEY_SHIFT)
		{
			forward_back_speed =  HIGH_FORWARD_BACK_SPEED;
			left_right_speed = HIGH_LEFT_RIGHT_SPEED;

		}
		else
		{
			forward_back_speed =  NORMAL_FORWARD_BACK_SPEED;
			left_right_speed = NORMAL_LEFT_RIGHT_SPEED;
		}
	
		Supply_Flag.Shoot = 0;
		MouseShootControl(mouse);
		//speed mode: normal speed/high speed
		if(key->v & KEY_SHIFT){
			forward_back_speed =  HIGH_FORWARD_BACK_SPEED;
			left_right_speed = HIGH_LEFT_RIGHT_SPEED;
		}
		else {
			forward_back_speed =  NORMAL_FORWARD_BACK_SPEED;
			left_right_speed = NORMAL_LEFT_RIGHT_SPEED;
		}
		
		//��������Щ��λ,onenote����
		//movement process
		if(key->v &KEY_W)
			ChassisSpeedRef.forward_back_ref = forward_back_speed*ramp_calc(&W_Ramp,0);//��б��}
		else if(key->v&KEY_S)  
			ChassisSpeedRef.forward_back_ref = -forward_back_speed*ramp_calc(&S_Ramp,0);
		else{
			ChassisSpeedRef.forward_back_ref = 0;
			ramp_clear(&W_Ramp);
			ramp_clear(&S_Ramp);
		}
		
		if(key->v & KEY_D){
			ChassisSpeedRef.left_right_ref =left_right_speed*ramp_calc(&KeyD_Ramp,0);//��б��
		}
		else if(key->v & KEY_A){
			ChassisSpeedRef.left_right_ref = -left_right_speed*ramp_calc(&KeyA_Ramp,0);//��б��
		}
		else	
		{
			ChassisSpeedRef.left_right_ref = 0;
			ramp_clear(&KeyD_Ramp);
			ramp_clear(&KeyA_Ramp);
			//��б������
		}
	
		//Q�����˶� EС����
		if(key->v & KEY_R) {
			SetGimbalSportState(CHASSIS_FOLLOW_YAW);
			SetChassisSportState(SPORT_STATE);
		}
		else if(key->v & KEY_F){
			SetGimbalSportState(CHASSIS_FOLLOW_YAW);
			SetChassisSportState(ROTATION_STATE);
			//chassis_rote_mote=160;
		}
		else if(key->v & KEY_G)
		{
			SetGimbalSportState(CHASSIS_NO_FOLLOW_YAW);
			SetChassisSportState(SPORT_STATE);
		}
	  else if(key->v & KEY_B){
			SetWorkMode(PREPARE_STATE);
			time_tick_1ms = 2900;
			reset_gimbal_angle++;
		}
		//���ոǼ�λ
		// C����V��
		if((key->v & KEY_C)){
			Servo_Control(ON);//F lage_UI_Mode_right1=0;
			shoot_speed = 0;
			Supply_Flag.Shoot = 0;
		}
		if(key->v & KEY_V){
			Servo_Control(OFF);
			shoot_speed = 182;
		}

		if(key->v & KEY_E){
			if(GetGimbalAndChassisSportState()!=Emergency_mode)
			shoot_speed = 0;
			else	
			GimbalRef.yaw_angle_dynamic_ref=100*ramp_calc(&KeyE_Ramp,0);
		}
		else if(key->v & KEY_Q){
			if(GetGimbalAndChassisSportState()!=Emergency_mode)
			shoot_speed = 202;
			else	
			GimbalRef.yaw_angle_dynamic_ref=-100*ramp_calc(&KeyQ_Ramp,0);
		}
		else{
			if(GetGimbalAndChassisSportState()==Emergency_mode)
			{
				GimbalRef.yaw_angle_dynamic_ref = 0;
				ramp_clear(&KeyE_Ramp);
				ramp_clear(&KeyQ_Ramp);
			}
		}
		if(key->v & KEY_Z){
			shoot_speed = 168;//15
			Supply_Flag.Shoot = 1;
			Shoot_Frequency=-18;
		}
		else if(key->v & KEY_X){
			shoot_speed = 286;//18
		}
		if(shoot_speed==286)	Flage_UI_Mode_right2=1;
		else 									Flage_UI_Mode_right2=0;
		/*���*/
//		VAL_LIMIT(mouse->x, -150, 150); 
//		VAL_LIMIT(mouse->y, -150, 150); 	
		GimbalRef.pitch_angle_dynamic_ref += mouse->y* MOUSE_TO_PITCH_ANGLE_INC_FACT*10;
		GimbalRef.yaw_angle_dynamic_ref   += mouse->x* MOUSE_TO_YAW_ANGLE_INC_FACT*10;
		GimbalAngleLimit();	
	}
}


//supply_state_e GetShootState()
//{
//	return Supply_State;
//}

//void SetShootState(supply_state_e v)
//{
//	Supply_State = v;
//}

//FrictionWheelState_e GetFrictionState()
//{
//	return Friction_Wheel_State;
//}

//void SetFrictionState(FrictionWheelState_e v)
//{
//	Friction_Wheel_State = v;
//}
/**
 * @Name:Gimbalanglelimit 
 * @Description: pitch�ḩ�����������
 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: void
 */
extern float low_angle_limit, high_angle_limit;
void GimbalAngleLimit()
{
	VAL_LIMIT(GimbalRef.pitch_angle_dynamic_ref,high_angle_limit,low_angle_limit);//�Զ�У׼ֵ
}

/**
 * @Name: Remote_Task_Init
 * @Description: ң�������ݳ�ʼ����б�º�����ʼ��
 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void Remote_Task_Init()
{
	//б�³�ʼ��
//	my_ramp_init(&KeyW_Ramp,100,mode0to1);
//	my_ramp_init(&KeyS_Ramp,100,mode0to1);

	my_ramp_init(&W_Ramp,50,mode0to1);
	my_ramp_init(&S_Ramp,50,mode0to1);
	my_ramp_init(&KeyA_Ramp,50,mode0to1);
	my_ramp_init(&KeyD_Ramp,50,mode0to1);
	my_ramp_init(&KeyQ_Ramp,50,mode0to1);
	my_ramp_init(&KeyE_Ramp,50,mode0to1);
	//��̨���̸���ֵ��ʼ��
	GimbalRef.pitch_angle_dynamic_ref = 0.0f;
	GimbalRef.yaw_angle_dynamic_ref = 0.0f;
	ChassisSpeedRef.forward_back_ref = 0.0f;
	ChassisSpeedRef.left_right_ref = 0.0f;
	ChassisSpeedRef.rotate_ref = 0.0f;
	
}


