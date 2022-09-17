#include "shoot_control.h"
myrampGen_t Friction_Ramp = {0};
supply_state_e Supply_State = STOP_Supply;
supply_flag_t Supply_Flag;
pid_t PID_Shoot_Speed={0};
pid_t PID_Shoot_Position={0};
pid_t FrictionSpeedPID_L={0};
pid_t FrictionSpeedPID_R={0};
/**
 * @Name: Get_Fill_State
 * @Description: ��ȡ��λ����״̬����Ϊ��������������
 * @Param: void
 * @Return: ����״̬��0Ϊ������1Ϊ��,Ϊö������
 * @Author: source
 * @Warning: void
 */
unsigned char Get_Fill_State()
{	
	return HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_0);
}

/**
 * @Name: SetSupplyState
 * @Description:���ù���ģʽ
 * @Param: ����ģʽ��Ϊö������
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void SetSupplyState(supply_state_e state)
{
	Supply_State = state;
}
/**
 * @Name:GetSupplyState 
 * @Description: ��ȡ����ģʽ
 * @Param: void
 * @Return: ����ģʽ,Ϊö������
 * @Author: source
 * @Warning: void
 */
supply_state_e GetSupplyState()
{
	return Supply_State;
}
/**
 * @Name: FSM_for_supply
 * @Description: ������ʽ״̬�������𹩵�����ģʽ���л�
 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: supply����ģʽ�л����ڴ˺�����ɣ������������ط��л�ģʽ
 */
void FSM_for_supply()
{
	switch(GetSupplyState())
	{
		case Supply:{ 
			if(Supply_Flag.Shoot==0)
				SetSupplyState(STOP_Supply);
		}break;
		case STOP_Supply: {
			if(Supply_Flag.Shoot == 1)
				SetSupplyState(Supply);
		}break;
		case error_Supply:{
			if(Supply_Flag.Error == 0)
				SetSupplyState(Supply);
		}break;
	}
}

/**
 * @Name: Ammunition_supply_Control
 * @Description: ��������

 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: void
 */

int Shoot_Frequency=-5; //��ÿ��
int Last_Shoot_Frequency=10;
int eer_bigger_than_200_tick_times;
int Count_time;
void Ammunition_supply_Control()
{
	static unsigned long int Temp_Tick = 0;
	static supply_state_e Temp_Last_Record;
	
	FSM_for_supply();
	if(Temp_Last_Record != GetSupplyState())
	{
		Pid_Out_Clear(&PID_Shoot_Speed);
		Temp_Tick = 0;
	}
	Temp_Last_Record = GetSupplyState();
	if(GetSupplyState() == Supply)
{
	pid_calc(&PID_Shoot_Speed, ShootEncoder.filter_rate,-Shoot_Frequency);
}
	else if(GetSupplyState() == STOP_Supply)
	{
		//Shoot_Frequency=ShootEncoder.ecd_angle;
		Pid_Out_Clear(&PID_Shoot_Speed);
	}
	else if(GetSupplyState() == error_Supply)
	{
//		Temp_Tick++;
//		if(Temp_Tick<=40)//400ms
//		pid_calc(&PID_Shoot_Speed,ShootEncoder.filter_rate,Shoot_Frequency);
//		else 
//		Supply_Flag.Error = 0;
		
	}

	}


/**
 * @Name: Shoot_Control
 * @Description:Ħ����ת�ٿ���
 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: void
 */
float shoot_speed=0;// 180��Ӧ15   80��10
float m=2;
void Shoot_Control()//ͬʱ��m��shoot_speed��ֵ������Ħ�����ٶ�
{
	if(GetWorkState() == NORMAL_STATE)////������״̬
	{
		FrictionSpeedPID_R.set=shoot_speed;
		FrictionSpeedPID_R.get=FrictionEncoder_R.filter_rate;
		FrictionSpeedPID_R.f_calc(&FrictionSpeedPID_R);
		
		FrictionSpeedPID_L.set=-shoot_speed;
		FrictionSpeedPID_L.get=FrictionEncoder_L.filter_rate;
		FrictionSpeedPID_L.f_calc(&FrictionSpeedPID_L);

	}//pid_calc(&PID_Shoot_Speed, ShootEncoder.filter_rate,-Shoot_Frequency);
	else 
	{
		Pid_Out_Clear(&FrictionSpeedPID_L);
		Pid_Out_Clear(&FrictionSpeedPID_R);
		ramp_clear(&Friction_Ramp);
	}

}

////          *rt_heat ----------------- ǹ��ʵʱ����
////          *cooling_rate ------------ ǹ��ÿ����ȴֵ
////          *heat_limit -------------- ǹ����������
////          *rt_bullet_speed --------- ��������
////          *bullet_freq ------------- ������Ƶ
////          *bullet_speed_limit ------ ������������
///**
// * @Name: Shoot_Power_Control
// * @Description:ǹ�����������Լ�Ħ�����ٶȿ��ƣ���ֹ���٣�
// * @Param: void
// * @Return: void
// * @Author: dhy
// * @Warning: void
/**
 * @Name: Shoot_Power_Control
 * @Description:ǹ�����������Լ�Ħ�����ٶȿ��ƣ���ֹ���٣�
 * @Param: void
 * @Return: void
 * @Author: dhy
 * @Warning: void
*/
uint16_t rt_heat=0, cooling_rate, heat_limit=0;//ʵʱ������ÿ����ȴ����������
float rt_bullet_speed=0, bullet_freq=0;//���٣���Ƶ
uint16_t bullet_speed_limit=0;//��������
float last_bullet_speed=0;

float shot_warning_heat=0;
float shot_warning_speed=0;
float heat_warning_scale = 0.75;//0.65
float bullet_warning_scale = 0.75;

void Shoot_Power_Control()
{
	Get_Referee_Shoot_Power_Information(&rt_heat, &cooling_rate, &heat_limit, &rt_bullet_speed, &bullet_freq, &bullet_speed_limit);
	
	//ǹ����������
	shot_warning_heat = 0.82*heat_limit;

	if(rt_heat >= shot_warning_heat)
	{
		PID_Shoot_Speed.out = 0;
	}
	
//	if(bullet_speed_limit==15)/**/
//		shoot_speed=180;
//	else if(bullet_speed_limit==18)
//		shoot_speed=240;
//	else if(bullet_speed_limit==30)
//		shoot_speed=420;
}



