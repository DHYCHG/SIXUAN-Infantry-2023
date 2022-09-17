#ifndef JUNDEG_H
#define JUNDEG_H
#include "stm32f4xx_hal.h"
#include "my_queue.h"
#include "mf_crc.h"
#include "string.h"
#include "usart.h"
#define FRAME_HEADER_LEN 5
#define REF_CRC16_SIZE 2
#define REF_HAEDER_CMDID_CRC16_SIZE (FRAME_HEADER_LEN+REF_CRC16_SIZE+2)
#define REF_HAEDER_CMDID_SIZE (FRAME_HEADER_LEN+sizeof(uint16_t))

/** 
  * @brief  frame header structure definition
  */

typedef __packed struct
{
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
} Frame_Header_t;

/** 
  * @brief  frame header structure definition
  */
typedef __packed struct
{
 uint8_t game_type : 4;
 uint8_t game_progress : 4;
 uint16_t stage_remain_time;
} ext_game_state_t;
typedef union
{
	uint8_t save[3];/* data */
	ext_game_state_t date;
}jundge_game_state_u;
/** 
  * @brief  frame header structure definition
  */

typedef __packed struct
{
 uint8_t winner;
} ext_game_result_t;
/** 
  * @brief  frame header structure definition
  */

typedef __packed struct
{
 uint16_t robot_legion;
} ext_game_robot_survivors_t;
/** 
  * @brief  frame header structure definition
  */

typedef __packed struct
{
 uint32_t event_type;
} ext_event_data_t;
/** 
  * @brief  frame header structure definition
  */

typedef __packed struct
{
 uint8_t supply_projectile_id;
 uint8_t supply_robot_id;
 uint8_t supply_projectile_step;
 uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

//¾É°æ²ÃÅÐÏµÍ³
//typedef __packed struct
//{
// uint8_t robot_id;
// uint8_t robot_level;
// uint16_t remain_HP;
// uint16_t max_HP;
// uint16_t shooter_id1_17mm_cooling_rate;
// uint16_t shooter_id1_17mm_cooling_limit;
// uint16_t shooter_id2_17mm_cooling_rate;
// uint16_t shooter_id2_17mm_cooling_limit;
// uint8_t mains_power_gimbal_output : 1;
// uint8_t mains_power_chassis_output : 1;
// uint8_t mains_power_shooter_output : 1;
//} ext_game_robot_state_t;
//typedef __packed union
//{
//	uint8_t save[14];
//	ext_game_robot_state_t data;
//}jundge_robot_state_u;

/** 
  * @brief  frame header structure definition
  */
typedef __packed struct
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t remain_HP;
	uint16_t max_HP;
	uint16_t shooter_id1_17mm_cooling_rate;
	uint16_t shooter_id1_17mm_cooling_limit;
	uint16_t shooter_id1_17mm_speed_limit;
	uint16_t shooter_id2_17mm_cooling_rate;
	uint16_t shooter_id2_17mm_cooling_limit;
	uint16_t shooter_id2_17mm_speed_limit;
	uint16_t shooter_id1_42mm_cooling_rate;
	uint16_t shooter_id1_42mm_cooling_limit;
	uint16_t shooter_id1_42mm_speed_limit;
	uint16_t chassis_power_limit;
	uint8_t mains_power_gimbal_output : 1;
	uint8_t mains_power_chassis_output : 1;
	uint8_t mains_power_shooter_output : 1;
}ext_game_robot_state_t;	

typedef __packed union
{
	uint8_t save[29];
	ext_game_robot_state_t data;
}jundge_robot_state_u;

/** 
  * @brief  frame header structure definition
  */

typedef __packed struct
{
 uint16_t chassis_volt;
 uint16_t chassis_current;
 float chassis_power;
 uint16_t chassis_power_buffer;
 uint16_t shooter_id1_17mm_cooling_heat;
 uint16_t shooter_id2_17mm_cooling_heat;
	uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

typedef  __packed union
{
	uint8_t save[16];/* data */
	ext_power_heat_data_t date;
}jundge_power_heat_data_u;

/** 
  * @brief  frame header structure definition
  */
typedef __packed struct{
 float x;
 float y;
 float z;
 float yaw;
} ext_game_robot_pos_t;
typedef  __packed union
{
	uint8_t save[16];/* data */
	ext_game_robot_pos_t date;
}jundge_robot_pos_data_u;
/** 
  * @brief  frame header structure definition
  */
typedef __packed struct
{
 uint8_t power_rune_buff;
}ext_buff_musk_t;
/** 
  * @brief  frame header structure definition
  */
typedef __packed struct
{
 uint8_t energy_point;
 uint8_t attack_time;
} aerial_robot_energy_t;
/** 
  * @brief  frame header structure definition
  */
typedef __packed struct
{
 uint8_t armor_id : 4;
 uint8_t hurt_type : 4;
} ext_robot_hurt_t;
/** 
  * @brief  frame header structure definition
  */
#pragma pack(1)
typedef __packed struct
{
	uint8_t bullet_type;
	uint8_t shooter_id;
	uint8_t bullet_freq;
	float bullet_speed;
	
} ext_shoot_data_t;

typedef union
{
	uint8_t save[7];/* data */
	ext_shoot_data_t date;
}jundge_shoot_data_u;
#pragma pack()
/** 
  * @brief  frame header structure definition
  */
typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
}ext_student_interactive_header_data_t;
/** 
  * @brief  frame header structure definition
  */
typedef __packed struct
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye:3;
	uint32_t graphic_tpye:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t radius:10;
	uint32_t end_x:11;
	uint32_t end_y:11;
}graphic_data_struct_t;
typedef __packed struct
{
	float data1;
	float data2;
	float data3;
	uint8_t makes;
}client_custom_data_t;

typedef __packed struct
{
	Frame_Header_t txFrame_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Client_Custom_ID;
	graphic_data_struct_t Graphic_Data[7];
	uint16_t CRC16;
}ext_draw_crosshair_t;

typedef __packed struct
{
	Frame_Header_t txFrame_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Client_Custom_ID;
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
	uint16_t CRC16;
}ex_draw_char;//UI_×Ö·û

typedef struct
{
	Frame_Header_t txFrame_Header;
	uint16_t CMD_ID;
	ext_student_interactive_header_data_t Client_Custom_ID;
	client_custom_data_t  Custom_Data;
	uint16_t CRC16;
}ex_sendcustom_t;

typedef struct{
	uint8_t data[256];
	uint16_t cmd_id;
	uint16_t data_len;
}unpack_data_t;

typedef struct{
	jundge_game_state_u jundge_game_state;    //0x0001
	jundge_robot_state_u jundge_robot_state;  //0x0201
	jundge_power_heat_data_u jundge_power_heat_data;  //0x0202
	jundge_robot_pos_data_u jundge_robot_pos_data; //0x0203
	jundge_shoot_data_u jundge_shoot_data; //0x0207
}referee_receive_data_t;

typedef enum{
	GAME_STATE_CMD_ID = 0x0001,
	GAME_ROBOT_BLOOD_CMD_ID = 0x0003,
	GAME_AREA_EVENT_CMD_ID = 0x0101,
	GAME_ROBOT_STATE_CMD_ID = 0x0201,
	GAME_POWER_HEART_CMD_ID = 0x0202,
	GAME_ROBOT_POS_CMD_ID = 0x0203,
	GAME_ROBOT_BUFF_CMD_ID = 0x0204,
	GAME_ROBOT_SHOOT_CMD_ID = 0x207,
}game_cmd_id_e;

extern referee_receive_data_t referee_receive_data;
extern ex_sendcustom_t SendCustom;
extern ex_draw_char Draw_Char;
extern ext_draw_crosshair_t Draw_CH;
extern ext_power_heat_data_t Judge_extPowerHeatData;
extern QUEUE8_t Queue_Date;
extern uint8_t My_Queue[256];
extern unpack_data_t unpack_data;
extern jundge_power_heat_data_u jundge_power_heat_data_t ;
void Jundge_Analysis(unpack_data_t *d);
void Draw_CrossHair(ext_draw_crosshair_t *draw,uint16_t cmd_id,uint16_t data_id,uint16_t tx_id,uint16_t rx_id);
void referee_data_solve(referee_receive_data_t *rec,uint16_t cmd_id,uint16_t data_len,uint8_t *pData);
void Get_Referee_Chassis_Power(float *chassis_power,float *chassis_power_buffer);
//void Get_Referee_Shoot_Power_Information(uint16_t *heat,uint16_t *cooling_rate,uint16_t *cooling_limit,float *shot_rate,float *shot_freq,uint8_t *shot_type);
float Get_Referee_Chassis_Power_Limit(void);
void Get_Referee_Shoot_Power_Information(uint16_t *rt_heat, uint16_t *cooling_rate, uint16_t *heat_limit, float *rt_bullet_speed, float *bullet_freq, uint16_t *bullet_speed_limit);

extern float dis_angle;
extern int Flage_UI_Mode_right1,Flage_UI_Mode_right2;
void Draw_CrossHair(ext_draw_crosshair_t *draw,uint16_t cmd_id,uint16_t data_id,uint16_t tx_id,uint16_t rx_id);
void Draw_Static_CrossHair_1(ext_draw_crosshair_t *draw,uint16_t cmd_id,uint16_t data_id,uint16_t tx_id,uint16_t rx_id);
void Draw_Dynamicr_CrossHair_1(ext_draw_crosshair_t *draw,uint16_t cmd_id,uint16_t data_id,uint16_t tx_id,uint16_t rx_id);
void Draw_Static_Char(ex_draw_char *draw,uint16_t cmd_id,uint16_t data_id,uint16_t tx_id,uint16_t rx_id);

void Draw_Static_Char_1(ex_draw_char *draw,uint16_t cmd_id,uint16_t data_id,uint16_t tx_id,uint16_t rx_id);
void Draw_Dynamicr_CrossHair_1(ext_draw_crosshair_t *draw,uint16_t cmd_id,uint16_t data_id,uint16_t tx_id,uint16_t rx_id);
void Draw_Dynamicr_Char_2(ex_draw_char *draw,uint16_t cmd_id,uint16_t data_id,uint16_t tx_id,uint16_t rx_id);
#endif



