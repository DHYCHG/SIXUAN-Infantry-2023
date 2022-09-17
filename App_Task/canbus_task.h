#ifndef _CANBUS_TASK_H
#define _CANBUS_TASK_H
#include "stm32f4xx_hal.h"
#include "can.h"

#define CAN_3508_M1_ID 0x201
#define CAN_3508_M2_ID 0x202
#define CAN_3508_M3_ID 0x203
#define CAN_3508_M4_ID 0x204

#define CAN_FrictionSpeed_L_ID 0x208
#define CAN_FrictionSpeed_R_ID 0x205

#define CAN_SHOOT_M2006_ID 0x207//can1

#define CAN_YAW_GM6020_ID 0x206 //can1
#define CAN_PITCH_GM6020_ID 0x209//can1


#define RATE_BUF_SIZE 6

#define MECANUM_WHEEL_CIRCUNFERENCE	47.9093522675f
#define REDUCTION_RATIO_TO_2006 10 
//typedef enum
//{
//	CAN_PREPARE,
//	CAN_NORMAL,
//	CAN_ERROR,
//}can_state_en;

typedef struct{
	int16_t init_raw_value;
	int16_t raw_value;
	int16_t last_raw_value;
	int16_t ecd_diff;
  int16_t speed_rpm;
	int16_t round_cnt;
	uint8_t buf_cnt;
	int16_t temp_cnt;
	int16_t rate_buff[RATE_BUF_SIZE];
	int16_t current;
	float ecd_angle;
	float filter_rate;
}Encoder_t;
extern volatile Encoder_t CM1Encoder;
extern volatile Encoder_t CM2Encoder;
extern volatile Encoder_t CM3Encoder;
extern volatile Encoder_t CM4Encoder;
extern volatile Encoder_t YawEncoder;
extern volatile Encoder_t PitchEncoder;
extern volatile Encoder_t ShootEncoder;
extern volatile Encoder_t FrictionEncoder_R;
extern volatile Encoder_t FrictionEncoder_L;
void Encoder_Process_For_3508(volatile Encoder_t *e,uint8_t rx_data[]);
void Encoder_Process_For_6020(volatile Encoder_t *e,uint8_t *rx_data);
void Encoder_Process_For_2006(volatile Encoder_t *e,uint8_t rx_data[]);
void Set_CM_Speed(CAN_HandleTypeDef *hcan, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
void Set_Gimbal_Current(CAN_HandleTypeDef *hcan, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq, int16_t shoot_iq);
void CAN_Watch(void);
void ALL_Motor_Out(void);
void Pitch_Motor_Stop(void);
void Pitch_Motor_Out(int16_t pitchout);
void can_task_init(void);
void SoftReset(void);
#endif

