#ifndef _REMOTE_TASK_H
#define _REMOTE_TASK_H
#include "stm32f4xx_hal.h"
#include "main.h"
#include "shoot_control.h"
#include "watch_task.h"
#include "math.h"
#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\


#define REMOTE_CONTROLLER_STICK_OFFSET      1024u   
#define CHASSIS_MAX_SPEED 400//cm/s//300
#define STICK_TO_CHASSIS_SPEED_REF_FACT     1.5f 
#define PITCH_MAX_SPEED 50//度/s
#define YAW_MAX_SPEED 120//度/s
#define CHASSIS_ROTATION_MAX_SPEED 150
#define RMOTE_VALUE_UNITIZATION 0.0015151515151515f

//鼠标参数
#define MOUSE_TO_PITCH_ANGLE_INC_FACT 		0.025f
#define MOUSE_TO_YAW_ANGLE_INC_FACT 		0.025f

#define NORMAL_FORWARD_BACK_SPEED 			200   //300
#define NORMAL_LEFT_RIGHT_SPEED   			200
#define HIGH_FORWARD_BACK_SPEED 			380   //660
#define HIGH_LEFT_RIGHT_SPEED   			380   //660

#define FRICTION_RAMP_TICK_COUNT			100
#define MOUSE_LR_RAMP_TICK_COUNT			50
#define MOUSR_FB_RAMP_TICK_COUNT			60

#define KEY_W 0x01
#define KEY_S 0x02
#define KEY_A 0x04
#define KEY_D 0x08
#define KEY_SHIFT 0x10
#define KEY_CTRL 0x20
#define KEY_Q 0x40
#define KEY_E 0x80
#define KEY_R 0x100
#define KEY_F 0x200
#define KEY_G 0x400
#define KEY_Z 0x800
#define KEY_X 0x1000
#define KEY_C 0x2000
#define KEY_V 0x4000
#define KEY_B 0x8000


#define REMOTE_SWITCH_VALUE_UP         		0x01u  
#define REMOTE_SWITCH_VALUE_DOWN			0x02u
#define REMOTE_SWITCH_VALUE_CENTRAL			0x03u

#define REMOTE_SWITCH_CHANGE_1TO3      (uint8_t)((REMOTE_SWITCH_VALUE_UP << 2) | REMOTE_SWITCH_VALUE_CENTRAL)   
#define REMOTE_SWITCH_CHANGE_2TO3      (uint8_t)((REMOTE_SWITCH_VALUE_DOWN << 2) | REMOTE_SWITCH_VALUE_CENTRAL)  
#define REMOTE_SWITCH_CHANGE_3TO1      (uint8_t)((REMOTE_SWITCH_VALUE_CENTRAL << 2) | REMOTE_SWITCH_VALUE_UP)
#define REMOTE_SWITCH_CHANGE_3TO2      (uint8_t)((REMOTE_SWITCH_VALUE_CENTRAL << 2) | REMOTE_SWITCH_VALUE_DOWN)

#define REMOTE_SWITCH_CHANGE_1TO3TO2   (uint8_t)((REMOTE_SWITCH_VALUE_UP << 4) |\
                                                 (REMOTE_SWITCH_VALUE_CENTRAL << 2) |\
                                                 (REMOTE_SWITCH_VALUE_DOWN))   

#define REMOTE_SWITCH_CHANGE_2TO3TO1   (uint8_t)((REMOTE_SWITCH_VALUE_DOWN << 4) |\
                                                 (REMOTE_SWITCH_VALUE_CENTRAL << 2) |\
                                                 (REMOTE_SWITCH_VALUE_UP)) 
#define REMOTE_SWITCH_VALUE_BUF_DEEP   16u
#define ON 1
#define OFF 0
void RemoteDataPrcess(uint8_t *pData);
typedef __packed struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	int8_t s1;
	int8_t s2;
}Remote;
typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
}Mouse;	
typedef	__packed struct
{
	uint16_t v;
}Key;
typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
}RC_Ctrl_t;
typedef enum
{
	REMOTE_INPUT = 1,
	STOP,
	KEY_MOUSE_INPUT,
}InputMode_e;
//摩擦轮状态枚举
typedef enum
{
	FRICTION_WHEEL_OFF = 0,
	FRICTION_WHEEL_START_TURNNING = 1,
	FRICTION_WHEEL_ON = 2,
}FrictionWheelState_e;

//底盘远程数据处理
typedef __packed struct
{
    int16_t forward_back_ref;
    int16_t left_right_ref;
    int16_t rotate_ref;
}ChassisSpeed_Ref_t;
//云台远程数据处理
typedef struct
{
    float pitch_angle_dynamic_ref;
    float yaw_angle_dynamic_ref;
    float pitch_angle_static_ref;
    float yaw_angle_static_ref;
    float pitch_speed_ref;
    float yaw_speed_ref;
		float shoot_angle_dynamic_ref;
}Gimbal_Ref_t;
//检测开关的动作
typedef struct RemoteSwitch_t
{
	 uint8_t switch_value_raw;            // 电流开关值
	 uint8_t switch_value1;				  //  最后一个值<<2值
	 uint8_t switch_value2;				  //
	 uint8_t switch_long_value; 		  //不切换时保持静止
	 uint8_t switch_value_buf[REMOTE_SWITCH_VALUE_BUF_DEEP]; 
	 uint8_t buf_index;
	 uint8_t buf_last_index;
	 uint8_t buf_end_index;
}RemoteSwitch_t;
extern Gimbal_Ref_t GimbalRef;
extern ChassisSpeed_Ref_t ChassisSpeedRef;
InputMode_e GetInputMode(void);
void RemoteControlProcess(Remote *rc);
void MouseKeyControlProcess(Mouse *mouse,Key *key);
void GimbalAngleLimit(void);
void Remote_Task_Init(void);
uint8_t IsRemoteBeingAction(void);
void MouseShootControl(Mouse *mouse);
void RemoteShootControl(RemoteSwitch_t *sw, uint8_t val) ;
#endif


