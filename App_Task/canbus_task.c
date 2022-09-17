#include "canbus_task.h"
#include "gimbal_control.h"
#include "pm01_api.h"

volatile Encoder_t CM1Encoder={0.0,0,0,0,0,0,0,0,0,0};
volatile Encoder_t CM2Encoder={0,0,0,0,0,0,0,0,0,0};
volatile Encoder_t CM3Encoder={0,0,0,0,0,0,0,0,0,0};
volatile Encoder_t CM4Encoder={0,0,0,0,0,0,0,0,0,0};
volatile Encoder_t YawEncoder={0,0,0,0,0,0,0,0,0,0};
volatile Encoder_t PitchEncoder={0,0,0,0,0,0,0,0,0,0};
volatile Encoder_t ShootEncoder={0,0,0,0,0,0,0,0,0,0};
volatile Encoder_t FrictionEncoder_R= {0,0,0,0,0,0,0,0,0};
volatile Encoder_t FrictionEncoder_L= {0,0,0,0,0,0,0,0,0};
volatile int tick_time=0;
//static uint32_t can_count = 0;
can_state_en Can_State;

void SetCanState(can_state_en state)
{
	Can_State = state;
}
can_state_en GetCanState()
{
	return Can_State;
}


void Encoder_Process_Bias(volatile Encoder_t *e,uint8_t rx_data[])
{
	e->init_raw_value = (rx_data[0]<<8|rx_data[1]);
	e->raw_value = e->init_raw_value;
	e->last_raw_value = e->init_raw_value;
}
/**
 * @Name: Encoder_Process_For_3508
 * @Description: 电机编码器值的处理
 * @Param: 编码器的值,can发送的信息
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void Encoder_Process_For_3508(volatile Encoder_t *e,uint8_t rx_data[])
{
	
	e->raw_value = (rx_data[0]<<8|rx_data[1]);
	e->speed_rpm = (uint16_t)(rx_data[2]<<8|rx_data[3]);
	e->current = (uint16_t)(rx_data[4]<<8|rx_data[5]);
	e->ecd_diff = e->raw_value - e->last_raw_value;
	if(e->ecd_diff >= 3000)
		e->round_cnt--;
	if(e->ecd_diff <= -3000)
		e->round_cnt++;
	e->ecd_angle = (float)(e->raw_value-e->init_raw_value)*360/8192+e->round_cnt*360; //角度 °
	e->rate_buff[e->buf_cnt++] = e->speed_rpm;
	if(e->buf_cnt>=RATE_BUF_SIZE)
		e->buf_cnt=0;
	int32_t temp_sum;
	for(int i =0;i<RATE_BUF_SIZE;i++)
	{
		temp_sum += e->rate_buff[i];
	}
	e->filter_rate = (float)temp_sum/RATE_BUF_SIZE*MECANUM_WHEEL_CIRCUNFERENCE/60/19; //cm/s
	e->last_raw_value = e->raw_value;
}
/**
 * @Name: Encoder_Process_For_6623
 * @Description: 电机编码器值的处理
 * @Param: 编码器的值,can发送的信息
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void Encoder_Process_For_6623(volatile Encoder_t *e,uint8_t rx_data[])
{
	
	e->raw_value = (rx_data[0]<<8|rx_data[1]);
	e->speed_rpm = (uint16_t)(rx_data[2]<<8|rx_data[3]);
	e->current = (uint16_t)(rx_data[4]<<8|rx_data[5]);
	e->ecd_diff = e->raw_value - e->last_raw_value;
	if(e->ecd_diff >= 2500)
		e->round_cnt--;
	if(e->ecd_diff <= -3000)
		e->round_cnt++;
	e->ecd_angle = (float)(e->raw_value-e->init_raw_value)*360/8192+e->round_cnt*360; //角度 °
	e->rate_buff[e->buf_cnt++] = e->speed_rpm;
	if(e->buf_cnt>=RATE_BUF_SIZE)
		e->buf_cnt=0;
	int32_t temp_sum ;
	for(int i =0;i<RATE_BUF_SIZE;i++)
	{
		temp_sum += e->rate_buff[i];
	}
	e->filter_rate = (float)temp_sum/RATE_BUF_SIZE*360; //°/m
	e->last_raw_value = e->raw_value;
}
/**
 * @Name: Encoder_Process_For_6020
 * @Description: 电机编码器值的处理
 * @Param: 编码器的值,can发送的信息
 * @Return: void
 * @Author: source
 * @Warning: void
 */
	
void Encoder_Process_For_6020(volatile Encoder_t *e,uint8_t *rx_data)
{
	e->raw_value = (rx_data[0]<<8|rx_data[1]);
	e->speed_rpm = (uint16_t)(rx_data[2]<<8|rx_data[3]);
	e->current = (uint16_t)(rx_data[4]<<8|rx_data[5]);
	e->ecd_diff = e->raw_value - e->last_raw_value;
	if(e->ecd_diff >= 3000)
		e->round_cnt--;
	if(e->ecd_diff <= -3000)
		e->round_cnt++;
	e->ecd_angle = (float)(e->raw_value-e->init_raw_value)*360/8192+e->round_cnt*360; //角度 °
	e->rate_buff[e->buf_cnt++] = e->speed_rpm;
	if(e->buf_cnt>=RATE_BUF_SIZE)
		e->buf_cnt=0;
	int32_t temp_sum;
	for(int i =0;i<RATE_BUF_SIZE;i++)
	{
		temp_sum += e->rate_buff[i];
	}
	e->filter_rate = (float)temp_sum/RATE_BUF_SIZE; //°/m
	temp_sum = 0;
	e->last_raw_value = e->raw_value;
}
/**
 * @Name: Encoder_Process_For_2006
 * @Description: 电机编码器值的处理
 * @Param: 编码器的值,can发送的信息
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void Encoder_Process_For_2006(volatile Encoder_t *e,uint8_t rx_data[])
{
	e->raw_value = (rx_data[0]<<8|rx_data[1]);
	e->speed_rpm = (uint16_t)(rx_data[2]<<8|rx_data[3]);
	e->current = (uint16_t)(rx_data[4]<<8|rx_data[5]);
	e->ecd_diff = e->raw_value - e->last_raw_value;
	if(e->ecd_diff >= 3000)
		e->round_cnt--;
	if(e->ecd_diff <= -3000)
		e->round_cnt++;
	e->ecd_angle = (float)(e->raw_value-e->init_raw_value)*REDUCTION_RATIO_TO_2006/8192+e->round_cnt*REDUCTION_RATIO_TO_2006; //角度 °
	e->rate_buff[e->buf_cnt++] = e->speed_rpm;
	if(e->buf_cnt>=RATE_BUF_SIZE)
		e->buf_cnt=0;
	int32_t temp_sum;
	for(int i =0;i<RATE_BUF_SIZE;i++)
	{
		temp_sum += e->rate_buff[i];
	}
	e->filter_rate = (float)temp_sum/RATE_BUF_SIZE*8/36/60; // 发/秒
	e->last_raw_value = e->raw_value;
}
/**
 * @Name: HAL_CAN_RxFifo0MsgPendingCallback
 * @Description:  This function process the can message representing the encoder data received from the CAN1 bus.
 * @Param: 使用的CAN
 * @Return: void
 * @Author: source
 * @Warning: 该函数由HAL库函数提供的回调函数，已在CAN接收中断函数中
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(__HAL_CAN_GET_IT_SOURCE(hcan,CAN_IT_RX_FIFO0_MSG_PENDING))
	{
		__HAL_CAN_CLEAR_FLAG(hcan,CAN_FLAG_FF0);
		CAN_RxHeaderTypeDef rx_msg;
		uint8_t rx_data[8];
		can_count++;		
		HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_msg,rx_data);
		if(GetCanState() == CAN_NORMAL)
		{
			switch(rx_msg.StdId)
			{
				case CAN_3508_M1_ID: Encoder_Process_For_3508(&CM1Encoder,rx_data); Tick_Controller.CM1 = HAL_GetTick();break;
				case CAN_3508_M2_ID: Encoder_Process_For_3508(&CM2Encoder,rx_data); Tick_Controller.CM2 = HAL_GetTick();break;
				case CAN_3508_M3_ID: Encoder_Process_For_3508(&CM3Encoder,rx_data); Tick_Controller.CM3 = HAL_GetTick();break;
				case CAN_3508_M4_ID: Encoder_Process_For_3508(&CM4Encoder,rx_data); Tick_Controller.CM4 = HAL_GetTick();break;
		
				case CAN_SHOOT_M2006_ID : Encoder_Process_For_2006(&ShootEncoder,rx_data); Tick_Controller.SUPPLY = HAL_GetTick();break;
				
				case CAN_FrictionSpeed_L_ID : Encoder_Process_For_3508(&FrictionEncoder_L,rx_data); Tick_Controller.FrictionL = HAL_GetTick();break;
				case CAN_FrictionSpeed_R_ID : Encoder_Process_For_3508(&FrictionEncoder_R,rx_data); Tick_Controller.FrictionR = HAL_GetTick();break;
				
				case CAN_YAW_GM6020_ID : Encoder_Process_For_6020(&YawEncoder,rx_data); Tick_Controller.YAW = HAL_GetTick();break;				
				case CAN_PITCH_GM6020_ID : Encoder_Process_For_6020(&PitchEncoder,rx_data); Tick_Controller.PITCH = HAL_GetTick();break;
			}
		}
		if(GetWorkState() == CAN_PREPARE)
		{
			switch(rx_msg.StdId)
				
			{
				case CAN_3508_M1_ID: Encoder_Process_Bias(&CM1Encoder,rx_data);break;
				case CAN_3508_M2_ID: Encoder_Process_Bias(&CM2Encoder,rx_data);break;
				case CAN_3508_M3_ID: Encoder_Process_Bias(&CM3Encoder,rx_data);break;
				case CAN_3508_M4_ID: Encoder_Process_Bias(&CM4Encoder,rx_data);break;
			}
		}
	}
}
/**
 * @Name: Set_CM_Speed
 * @Description:    给电调板发送指令,ID号为0x1FF,只用两个电调板,数据回传ID为0x205和0x206
	                 cyq:更改为发送三个电调的指令。 
 * @Param: 不同ID电流的期望值
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void Set_CM_Speed(CAN_HandleTypeDef *hcan, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
	uint8_t Data[8];
	uint32_t TxMailBox;
	CAN_TxHeaderTypeDef CAN_Tx_Message;
	CAN_Tx_Message.StdId = 0x200;
	CAN_Tx_Message.IDE = CAN_ID_STD;
	CAN_Tx_Message.RTR = CAN_RTR_DATA;
	CAN_Tx_Message.DLC = 0x08;
	
	Data[0] = (uint8_t)(cm1_iq>>8);
	Data[1] = (uint8_t)(cm1_iq);
	Data[2] = (uint8_t)(cm2_iq>>8);
	Data[3] = (uint8_t)(cm2_iq);
	Data[4] = (uint8_t)(cm3_iq>>8);
	Data[5] = (uint8_t)(cm3_iq);	
	Data[6] = (uint8_t)(cm4_iq>>8);
	Data[7] = (uint8_t)(cm4_iq);

	HAL_CAN_AddTxMessage(hcan,&CAN_Tx_Message,Data,&TxMailBox);
}

void Set_Gimbal_Current(CAN_HandleTypeDef *hcan, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq, int16_t shoot_iq)
{
	uint8_t Data[8];
	uint32_t TxMailBox;
	CAN_TxHeaderTypeDef CAN_Tx_Message;
	CAN_Tx_Message.StdId = 0x1FF;
	CAN_Tx_Message.IDE = CAN_ID_STD;
	CAN_Tx_Message.RTR = CAN_RTR_DATA;
	CAN_Tx_Message.DLC = 0x08;
	
	Data[0] = 0;//(uint8_t)(gimbal_pitch_iq>>8);//改ID要改这个.
	Data[1] = 0;//(uint8_t)(gimbal_pitch_iq);
	Data[2] = (uint8_t)(gimbal_yaw_iq>>8);//(uint8_t)(gimbal_pitch_iq>>8);
	Data[3] = (uint8_t)(gimbal_yaw_iq);//(uint8_t)(gimbal_pitch_iq);
	Data[4] = (uint8_t)(shoot_iq>>8);
	Data[5] = (uint8_t)(shoot_iq);	
	Data[6] = 0x00;
	Data[7] = 0x00;
	
	HAL_CAN_AddTxMessage(hcan,&CAN_Tx_Message,Data,&TxMailBox);	
	
	CAN_Tx_Message.StdId = 0x2FF;
	
	
	Data[0] = (uint8_t)(gimbal_pitch_iq>>8);//改ID要改这个.
	Data[1] = (uint8_t)(gimbal_pitch_iq);
	Data[2] = 0x00;//(uint8_t)(gimbal_pitch_iq>>8);
	Data[3] = 0x00;//(uint8_t)(gimbal_pitch_iq);
	Data[4] = 0x00;
	Data[5] = 0x00;	
	Data[6] = 0x00;
	Data[7] = 0x00;
	
	HAL_CAN_AddTxMessage(hcan,&CAN_Tx_Message,Data,&TxMailBox);	
}

void Set_CAN2_Current(CAN_HandleTypeDef *hcan,int16_t friction_l_iq,int16_t friction_r_iq){
	uint8_t Data[8];
	uint32_t TxMailBox;
	CAN_TxHeaderTypeDef CAN_Tx_Message;
	CAN_Tx_Message.StdId = 0x1FF;
	CAN_Tx_Message.IDE = CAN_ID_STD;
	CAN_Tx_Message.RTR = CAN_RTR_DATA;
	CAN_Tx_Message.DLC = 0x08;
	Data[0] = (uint8_t)(friction_r_iq>>8);
	Data[1] = (uint8_t)(friction_r_iq);
	Data[2] = 0x00;
	Data[3] = 0x00;
	Data[4] = 0x00;
	Data[5] = 0x00;
	Data[6] = (uint8_t)(friction_l_iq>>8);
	Data[7] = (uint8_t)(friction_l_iq);
	HAL_CAN_AddTxMessage(hcan,&CAN_Tx_Message,Data,&TxMailBox);	
}
	
void CAN_Watch()
{
	if(GetCanState() == CAN_PREPARE)
	{
		if(can_count>1000)
			SetCanState(CAN_NORMAL);
	}
	else if(GetCanState()==CAN_NORMAL)
	{
		
	}
	else if(GetCanState()==CAN_ERROR)
	{
	//	HAL_CAN_ResetError(hcan1);

	}
}

void SoftReset(void)//软件复位
{
    __set_FAULTMASK(1); // 关闭所有中断
    NVIC_SystemReset(); // 复位
}

/**
 * @Name: ALL_Motor_Out
 * @Description: 所有电机的输出函数，将此函数屏蔽则会完全无力
 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void ALL_Motor_Out()
{
	if(GetWorkState() == STOP_STATE)
	{
		Set_CM_Speed(&hcan1,0,0,0,0);
		Set_Gimbal_Current(&hcan1,0,0,0);
		Set_CAN2_Current(&hcan2,0,0);			
	}
	else if(GetWorkState() ==PREPARE_STATE)
	{
		Set_CM_Speed(&hcan1,0,0,0,0);
		Set_Gimbal_Current(&hcan1,0,0,0);
		Set_CAN2_Current(&hcan2,0,0);		
	}
	else
	{	
		Chassis_VAL_LIMIT(15000);		
		Set_CM_Speed(&hcan1,	
							PID_Chassis_Motor1_Speed.out,
							PID_Chassis_Motor2_Speed.out, 
							PID_Chassis_Motor3_Speed.out,
							PID_Chassis_Motor4_Speed.out);
		Set_Gimbal_Current(&hcan1,PID_Yaw_Speed.out,PID_Pitch_Speed.out,PID_Shoot_Speed.out);
		Set_CAN2_Current(&hcan2,FrictionSpeedPID_L.out,FrictionSpeedPID_R.out);
	}
}

/**
 * @Name: Pitch_Motor_Out
 * @Description: 单独Pitch电机的输出函数,Pitch自动校准用
 * @Param: void
 * @Return: void
 * @Author: DHY 2022-09-17
 * @Warning: void
 */
void Pitch_Motor_Out(int16_t pitchout)
{
	Set_Gimbal_Current(&hcan1,0,pitchout,0);
}

/**
 * @Name: Pitch_Motor_Stop
 * @Description: 单独Pitch电机的输出清除,Pitch自动校准用
 * @Param: void
 * @Return: void
 * @Author: DHY 2022-09-17
 * @Warning: void
 */
void Pitch_Motor_Stop()
{
	Set_Gimbal_Current(&hcan1,0,0,0);
}

