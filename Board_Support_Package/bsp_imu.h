/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_imu.h
 * @brief      this file contains the common defines and functions prototypes for 
 *             the bsp_imu.c driver
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-30-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#ifndef __MPU_H__
#define __MPU_H__
#include "drv_icm20602.h"
#include "mytype.h"
#define MPU_DELAY(x) HAL_Delay(x)

#define NEWEST_IMU_CODE

#define GYRO_PN2000_RAD 0.00106518f
#define ACC_PN8G_TO_CMSS 0.239257f
#define ACC_PN3G_TO_CMSS 0.897244f

typedef struct
{
	float center_pos_cm[3];
	float gryo_rad_value[3];
	float gryo_old_rad_value[3];
	float center_gryo_rad_acc[3];
	float center_liner_acc[3];
}_center_t;

typedef enum
{
	x=0,
  y,
  z
}_axis_en;

typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;

	int16_t mx;
	int16_t my;
	int16_t mz;

	int16_t temp;

	int16_t gx;
	int16_t gy;
	int16_t gz;
	
	int16_t ax_offset;
	int16_t ay_offset;
	int16_t az_offset;

	int16_t gx_offset;
	int16_t gy_offset;
	int16_t gz_offset;
	int16_t acc_val_ref[3];
	int16_t gryo_val_ref[3];
	int16_t accel_filter[3];
	int16_t gryo_filter[3];
	int16_t acc_val[3];
	int16_t gryo_val[3];
	int16_t gryo_deg[3];
	int16_t gryo_deg_lpf[3];
} mpu_data_t;

typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;

	int16_t mx;
	int16_t my;
	int16_t mz;

	float temp;

	float wx; /*!< omiga, +- 2000dps => +-32768  so gx/16.384/57.3 =	rad/s */
	float wy;
	float wz;

	float vx;
	float vy;
	float vz;

	float rol;
	float pit;
	float yaw;
} imu_t;

extern mpu_data_t mpu_data;
extern imu_t      imu;

uint8_t   Mpu_Device_Init(void);
void init_quaternion(void);
void mpu_get_data(void);
void imu_ahrs_update(void);
void imu_attitude_update(void);
void mpu_offset_call(void);

#endif


