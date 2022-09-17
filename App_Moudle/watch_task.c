#include  "watch_task.h"
#include "bsp_buzzer.h"
#include  "main.h"
#define REMOTE_TIMEOUT 	1000 	//ms
#define MOTER_TIMEOUT 	1000 	//ms
Tick_Controller_t Tick_Controller;
char Is_Remote_Online=0;
char Is_Motor_Online[9] = {0};
volatile int can_count = 0;

/**
 * @Name: Remote_Watch
 * @Description: 实时监测遥控器通信任务
 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void Remote_Watch()
{
	if((HAL_GetTick()-Tick_Controller.REMOTE) > REMOTE_TIMEOUT)
	{
		Is_Remote_Online=0;
		//TIM12->CCR1=1000;
	}
	else
	{
		Is_Remote_Online=1;
		//TIM12->CCR1=0;
	}	
}

/**
 * @Name: Motor_Watch
 * @Description: 实时监测电机通信任务
 * @Param: void
 * @Return: void
 * @Author: DHY 2022-09-16
 * @Warning: void
 */
void Motor_Watch()
{
	unsigned int now_tick = HAL_GetTick();
	
	((now_tick-Tick_Controller.CM1) > MOTER_TIMEOUT)?(Is_Motor_Online[eCM1] = 0):(Is_Motor_Online[eCM1] = 1);
	((now_tick-Tick_Controller.CM2) > MOTER_TIMEOUT)?(Is_Motor_Online[eCM2] = 0):(Is_Motor_Online[eCM2] = 1);
	((now_tick-Tick_Controller.CM3) > MOTER_TIMEOUT)?(Is_Motor_Online[eCM3] = 0):(Is_Motor_Online[eCM3] = 1);
	((now_tick-Tick_Controller.CM4) > MOTER_TIMEOUT)?(Is_Motor_Online[eCM4] = 0):(Is_Motor_Online[eCM4] = 1);
	
	((now_tick-Tick_Controller.PITCH) > MOTER_TIMEOUT)?(Is_Motor_Online[ePITCH] = 0):(Is_Motor_Online[ePITCH] = 1);
	((now_tick-Tick_Controller.YAW) > MOTER_TIMEOUT)?(Is_Motor_Online[eYAW] = 0):(Is_Motor_Online[eYAW] = 1);
	
	((now_tick-Tick_Controller.SUPPLY) > MOTER_TIMEOUT)?(Is_Motor_Online[eSUPPLY] = 0):(Is_Motor_Online[eSUPPLY] = 1);
	((now_tick-Tick_Controller.FrictionL) > MOTER_TIMEOUT)?(Is_Motor_Online[eFrictionL] = 0):(Is_Motor_Online[eFrictionL] = 1);
	((now_tick-Tick_Controller.FrictionR) > MOTER_TIMEOUT)?(Is_Motor_Online[eFrictionR] = 0):(Is_Motor_Online[eFrictionR] = 1);
}

/**
  * @brief          使得蜂鸣器响
	* @param[in]      sound:音调 num:响声次数
  * @retval         none
  */
static void buzzer_warn_error(uint16_t sound, uint8_t num)
{
    static uint8_t show_num = 0;
    static uint8_t stop_num = 100;
    if(show_num == 0 && stop_num == 0)
    {
        show_num = num;
        stop_num = 100;
    }
    else if(show_num == 0)
    {
        stop_num--;
        buzzer_off();
    }
    else
    {
        static uint8_t tick = 0;
        tick++;
        if(tick < 50)
        {
            buzzer_off();
        }
        else if(tick < 100)
        {
            buzzer_on(1, sound);
        }
        else
        {
            tick = 0;
            show_num--;
        }
    }
}

/**
 * @Name: Buzzer_Control
 * @Description: 蜂鸣器报警控制
 * @Param: void
 * @Return: void
 * @Author: DHY 2022-09-16
 * @Warning: void
 */
void Buzzer_Control()
{
	if(Is_Motor_Online[eCM1]==0 || Is_Motor_Online[eCM2]==0 || Is_Motor_Online[eCM3]==0 || Is_Motor_Online[eCM4]==0)
	{
		buzzer_warn_error(10000, 10);
	}
	else
	{
		buzzer_off();
		if(Is_Motor_Online[eYAW]==0 || Is_Motor_Online[ePITCH]==0)
		{
			buzzer_warn_error(40000, 10);
		}
		else
		{
			buzzer_off();
			if(Is_Motor_Online[eSUPPLY]==0 || Is_Motor_Online[eFrictionL]==0 || Is_Motor_Online[eFrictionR]==0)
			{
				//buzzer_warn_error(60000, 10);//太吵啦~ 先注释掉了
			}
			else
			{
				buzzer_off();
			}
		}
	}
}

void Watch_Task()
{
	Remote_Watch();
	Motor_Watch();
	Buzzer_Control();
}


