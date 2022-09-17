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
 * @Description: 获取限位开关状态，作为弹链充满的依据
 * @Param: void
 * @Return: 弹链状态，0为不满，1为满,为枚举类型
 * @Author: source
 * @Warning: void
 */
unsigned char Get_Fill_State()
{	
	return HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_0);
}

/**
 * @Name: SetSupplyState
 * @Description:设置供弹模式
 * @Param: 供弹模式，为枚举类型
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
 * @Description: 获取供弹模式
 * @Param: void
 * @Return: 供弹模式,为枚举类型
 * @Author: source
 * @Warning: void
 */
supply_state_e GetSupplyState()
{
	return Supply_State;
}
/**
 * @Name: FSM_for_supply
 * @Description: 供弹方式状态机，负责供弹四种模式的切换
 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: supply所有模式切换均在此函数完成，请勿在其他地方切换模式
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
 * @Description: 供弹任务

 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: void
 */

int Shoot_Frequency=-5; //颗每秒
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
 * @Description:摩擦轮转速控制
 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: void
 */
float shoot_speed=0;// 180对应15   80对10
float m=2;
void Shoot_Control()//同时改m，shoot_speed的值，调节摩擦轮速度
{
	if(GetWorkState() == NORMAL_STATE)////无输入状态
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

////          *rt_heat ----------------- 枪口实时热量
////          *cooling_rate ------------ 枪口每秒冷却值
////          *heat_limit -------------- 枪口热量上限
////          *rt_bullet_speed --------- 弹丸射速
////          *bullet_freq ------------- 弹丸射频
////          *bullet_speed_limit ------ 弹丸射速限制
///**
// * @Name: Shoot_Power_Control
// * @Description:枪口热量限制以及摩擦轮速度控制（防止超速）
// * @Param: void
// * @Return: void
// * @Author: dhy
// * @Warning: void
/**
 * @Name: Shoot_Power_Control
 * @Description:枪口热量限制以及摩擦轮速度控制（防止超速）
 * @Param: void
 * @Return: void
 * @Author: dhy
 * @Warning: void
*/
uint16_t rt_heat=0, cooling_rate, heat_limit=0;//实时热量，每秒冷却，热量上限
float rt_bullet_speed=0, bullet_freq=0;//弹速，射频
uint16_t bullet_speed_limit=0;//射速上限
float last_bullet_speed=0;

float shot_warning_heat=0;
float shot_warning_speed=0;
float heat_warning_scale = 0.75;//0.65
float bullet_warning_scale = 0.75;

void Shoot_Power_Control()
{
	Get_Referee_Shoot_Power_Information(&rt_heat, &cooling_rate, &heat_limit, &rt_bullet_speed, &bullet_freq, &bullet_speed_limit);
	
	//枪口热量限制
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



