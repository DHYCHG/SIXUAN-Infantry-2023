#ifndef _DRV_ICM20602_H
#define _DRV_ICM20602_H
#include "stm32f4xx_hal.h"
#include "drv_icm20602.h"
#define ICM20602_CS(X)			(X==0)?HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET):HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET) //MPU6500Ƭѡ�ź�
typedef struct __MPU6050_RAW_Data__
{
    short Accel_X;  //�Ĵ���ԭ��X����ٶȱ�ʾֵ
    short Accel_Y;  //�Ĵ���ԭ��Y����ٶȱ�ʾֵ
    short Accel_Z;  //�Ĵ���ԭ��Z����ٶȱ�ʾֵ
    short Temp;     //�Ĵ���ԭ���¶ȱ�ʾֵ
    short Gyro_X;   //�Ĵ���ԭ��X�������Ǳ�ʾֵ
    short Gyro_Y;   //�Ĵ���ԭ��Y�������Ǳ�ʾֵ
    short Gyro_Z;   //�Ĵ���ԭ��Z�������Ǳ�ʾֵ
	  short Mag_X;   //�Ĵ���ԭ��X�������Ǳ�ʾֵ
    short Mag_Y;   //�Ĵ���ԭ��Y�������Ǳ�ʾֵ
    short Mag_Z;   //�Ĵ���ԭ��Z�������Ǳ�ʾֵ
	
}MPU6050_RAW_DATA;

typedef struct __MPU6050_REAL_Data__
{
    float Accel_X;  //ת����ʵ�ʵ�X����ٶȣ�
    float Accel_Y;  //ת����ʵ�ʵ�Y����ٶȣ�
    float Accel_Z;  //ת����ʵ�ʵ�Z����ٶȣ�
    float Temp;     //ת����ʵ�ʵ��¶ȣ���λΪ���϶�
    float Gyro_X;   //ת����ʵ�ʵ�X��Ǽ��ٶȣ�
    float Gyro_Y;   //ת����ʵ�ʵ�Y��Ǽ��ٶȣ�
    float Gyro_Z;   //ת����ʵ�ʵ�Z��Ǽ��ٶ�
	  float Mag_X;   //ת����ʵ�ʵ�X��Ǽ��ٶȣ�
    float Mag_Y;   //ת����ʵ�ʵ�Y��Ǽ��ٶȣ�
    float Mag_Z;   //ת����ʵ�ʵ�Z��Ǽ��ٶ�
	
}MPU6050_REAL_DATA;

//define the eluer angle
typedef struct AHRS
{
	float pitch;
	float roll;
	float yaw;
	
}AHRS;
extern AHRS ahrs;

//the max and min data of the mag
typedef __packed struct
{
	int16_t MaxMagX;
	int16_t MaxMagY;
	int16_t MaxMagZ;
	int16_t MinMagX;
	int16_t MinMagY;
	int16_t MinMagZ;
}MagMaxMinData_t;
extern MagMaxMinData_t MagMaxMinData;

#define GYRO_CALI_FLAG 	  		(1<<0)
#define HMC5883_CALI_FLAG 		(1<<1)
#define ENCODER_CALI_FLAG 		(1<<2)
#define ALL_SENSOR_CALI_FLAG	(GYRO_CALI_FLAG|HMC5883_CALI_FLAG|ENCODER_CALI_FLAG)



void SPI2_Init(void);
void IST8310_getRaw(int16_t *x,int16_t *y,int16_t *z);
void IST8310_mgetValues(volatile float *arry);
void IST8310_getlastValues(int16_t *hx,int16_t *hy, int16_t *hz);
uint8_t IST8310_Init(void);
uint8_t ICM20602_Write_Reg(uint8_t reg,uint8_t value);
uint8_t ICM20602_Read_Reg(uint8_t reg);
uint8_t SPI2_Read_Write_Byte(uint8_t TxData);//SPI���߶�дһ���ֽ�
#endif


