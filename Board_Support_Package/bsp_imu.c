#include "drv_icm20602.h"
#include "drv_icm20602_reg.h"
#include <math.h>
#include "bsp_imu.h"
#include "bsp_buzzer.h"
#include "ist8310_reg.h" 
#include "string.h"
#include "pid.h"
#include "..\BMI088\BMI088driver.h"
#include "..\IST8310\ist8310driver.h"
#define MPU_NSS_LOW HAL_GPIO_WritePin		(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET)
#define MPU_NSS_HIGH HAL_GPIO_WritePin	(GPIOB,GPIO_PIN_12,GPIO_PIN_SET)

#define Kp 2.0f                                              /* 
                                                              * proportional gain governs rate of 
                                                              * convergence to accelerometer/magnetometer 
																															*/
#define Ki 0.001f                                             /* 
                                                              * integral gain governs rate of 
                                                              * convergence of gyroscope biases 
																															*/
volatile float        q0 = 1.0f;
volatile float        q1 = 0.0f;
volatile float        q2 = 0.0f;
volatile float        q3 = 0.0f;
volatile float        exInt, eyInt, ezInt;                   /* error integral */
static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;  
volatile uint32_t     last_update, now_update;               /* Sampling cycle count, ubit ms */
static uint8_t        tx, rx;

_axis_en              axis;
_center_t             center_pos;
uint8_t               mpu_buff[14];                          /* buffer to save imu raw data */
uint8_t               ist_buff[6];                           /* buffer to save IST8310 raw data */
mpu_data_t            mpu_data;
imu_t                 imu={0};
float 								mag[3];
float									hisimu[10][3];//保存10个历史姿态角

pid_t PID_IMU_Tmp=PID_PARAM_DEFAULT;

void add_hisimu(imu_t imu)
{
	float imutmp[3];
	int i = 0;
	
	imutmp[0] = imu.rol;
	imutmp[1] = imu.pit;
	imutmp[2] = imu.yaw;
	
	for(i = 9; i > 0; i--)
	{
		hisimu[i][0] = hisimu[i-1][0];
		hisimu[i][1] = hisimu[i-1][1];
		hisimu[i][2] = hisimu[i-1][2];
	}
	
	hisimu[0][0] = imutmp[0];
	hisimu[0][1] = imutmp[1];
	hisimu[0][2] = imutmp[2];
	
}

/**
  * @brief  fast inverse square-root, to calculate 1/Sqrt(x)
  * @param  x: the number need to be calculated
  * @retval 1/Sqrt(x)
  * @usage  call in imu_ahrs_update() function
  */
float inv_sqrt(float x) 
{
	float halfx = 0.5f * x;
	float y     = x;
	long  i     = *(long*)&y;
	
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	
	return y;
}

/**
  * @brief  write a byte of data to specified register
  * @param  reg:  the address of register to be written
  *         data: data to be written
  * @retval 
  * @usage  call in ist_reg_write_by_mpu(),         
  *                 ist_reg_read_by_mpu(), 
  *                 mpu_master_i2c_auto_read_config(), 
  *                 ist8310_init(), 
  *                 mpu_set_gyro_fsr(),             
  *                 mpu_set_accel_fsr(), 
  *                 mpu_device_init() function
  */
uint8_t mpu_write_byte(uint8_t const reg, uint8_t const data)
{
    MPU_NSS_LOW;
    
    SPI2_Read_Write_Byte(reg);
    
    SPI2_Read_Write_Byte(data);
    MPU_NSS_HIGH;
    return 0;
}

/**
  * @brief  read a byte of data from specified register
  * @param  reg: the address of register to be read
  * @retval 
  * @usage  call in ist_reg_read_by_mpu(),         
  *                 mpu_device_init() function
  */
uint8_t mpu_read_byte(uint8_t const reg)
{
    MPU_NSS_LOW;
    tx = reg | 0x80;
    SPI2_Read_Write_Byte(tx);
    rx=SPI2_Read_Write_Byte(tx);
    MPU_NSS_HIGH;
    return rx;
}

/**
  * @brief  read bytes of data from specified register
  * @param  reg: address from where data is to be written
  * @retval 
  * @usage  call in ist8310_get_data(),         
  *                 mpu_get_data(), 
  *                 mpu_offset_call() function
  */

uint8_t mpu_read_bytes(uint8_t const regAddr, uint8_t* pData, uint8_t len)
{
    MPU_NSS_LOW;
   SPI2_Read_Write_Byte(regAddr|0x80); 				//浠庡姞閫熷害璁＄殑瀵勫瓨鍣ㄥ紑濮嬭繘琛岃?诲彇闄€铻轰华鍜屽姞閫熷害璁＄殑鍊?//鍙戦€佽?诲懡浠?+瀵勫瓨鍣ㄥ彿
	
	for(int i	=	0;i	<	len;i++)														//涓€鍏辫?诲彇14瀛楄妭鐨勬暟鎹?
	{
		*(pData+i)	=	SPI2_Read_Write_Byte(0xff);	//杈撳叆0xff,鍥犱负slave涓嶈瘑鍒?
	}	
    MPU_NSS_HIGH;
    return 0;
}
	static void icm20602_writeBit(u8 reg, u8 data, u8 bitNum) 
{
    u8 b;
   mpu_read_bytes(reg, &b, 1 );
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	mpu_write_byte(reg, b);
}

/**
	* @brief  write IST8310 register through MPU6500's I2C master
  * @param  addr: the address to be written of IST8310's register
  *         data: data to be written
	* @retval   
  * @usage  call in ist8310_init() function
	*/
static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data)
{
    /* turn off slave 1 at first */
    mpu_write_byte(MPU_RA_I2C_SLV1_CTRL, 0x00);
    MPU_DELAY(2);
    mpu_write_byte(MPU_RA_I2C_SLV1_REG, addr);
    MPU_DELAY(2);
    mpu_write_byte(MPU_RA_I2C_SLV1_DO, data);
    MPU_DELAY(2);
    /* turn on slave 1 with one byte transmitting */
    mpu_write_byte(MPU_RA_I2C_SLV1_CTRL, 0x80 | 0x01);
    /* wait longer to ensure the data is transmitted from slave 1 */
    MPU_DELAY(10);
}

/**
	* @brief  write IST8310 register through MPU6500's I2C Master
	* @param  addr: the address to be read of IST8310's register
	* @retval 
  * @usage  call in ist8310_init() function
	*/
static uint8_t ist_reg_read_by_mpu(uint8_t addr)
{
    uint8_t retval;
    mpu_write_byte(MPU_RA_I2C_SLV4_REG, addr);
    MPU_DELAY(10);
    mpu_write_byte(MPU_RA_I2C_SLV4_CTRL, 0x80);
    MPU_DELAY(10);
    retval = mpu_read_byte(MPU_RA_I2C_SLV4_DI);
    /* turn off slave4 after read */
    mpu_write_byte(MPU_RA_I2C_SLV4_CTRL, 0x00);
    MPU_DELAY(10);
    return retval;
}

/**
	* @brief    initialize the MPU6500 I2C Slave 0 for I2C reading.
* @param    device_address: slave device address, Address[6:0]
	* @retval   void
	* @note     
	*/
static void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
    /* 
	   * configure the device address of the IST8310 
     * use slave1, auto transmit single measure mode 
	   */
    mpu_write_byte(MPU_RA_I2C_SLV1_ADDR , device_address);
    MPU_DELAY(2);
    mpu_write_byte(MPU_RA_I2C_SLV1_REG, IST8310_R_CONFA);
    MPU_DELAY(2);
    mpu_write_byte(MPU_RA_I2C_SLV1_DO, IST8310_ODR_MODE);
    MPU_DELAY(2);

    /* use slave0,auto read data */
    mpu_write_byte(MPU_RA_I2C_SLV0_ADDR, 0x80 | device_address);
    MPU_DELAY(2);
    mpu_write_byte(MPU_RA_I2C_SLV0_REG, reg_base_addr);
    MPU_DELAY(2);

    /* every eight mpu6500 internal samples one i2c master read */
    mpu_write_byte(MPU_RA_I2C_SLV4_CTRL, 0x03);
    MPU_DELAY(2);
    /* enable slave 0 and 1 access delay */
    mpu_write_byte(MPU_RA_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    MPU_DELAY(2);
    /* enable slave 1 auto transmit */
    mpu_write_byte(MPU_RA_I2C_SLV1_CTRL, 0x80 | 0x01);
		/* Wait 6ms (minimum waiting time for 16 times internal average setup) */
    MPU_DELAY(6); 
    /* enable slave 0 with data_num bytes reading */
    mpu_write_byte(MPU_RA_I2C_SLV0_CTRL, 0x80 | data_num);
    MPU_DELAY(2);
}

/**
	* @brief  get the data of IST8310
  * @param  buff: the buffer to save the data of IST8310
	* @retval 
  * @usage  call in mpu_get_data() function
	*/
void ist8310_get_data(uint8_t* buff)
{
    mpu_read_bytes(MPU_RA_EXT_SENS_DATA_00, buff, 6); 
}
/**
* @brief  sensor position 
* @param  according to the sensor position and the center point position
* @retval 
* @usage  call in Mpu_Device_Init() function
*/
void center_pos_set()
{
	center_pos.center_pos_cm[x]=0.0f;
	center_pos.center_pos_cm[y]=0.0f;
	center_pos.center_pos_cm[z]=0.0f;
}
int16_t gyro[3] = {0,0,0}, accel[3] = {0,0,0}, temp = 0;

void imu_pwm_set(uint16_t pwm)
{
    TIM10->CCR1 = (pwm);

}

float set_temp = 40.0f;
uint8_t use_mag = 0;
int8_t bmi088_orientation[3][3] = {	
																		{1, 0, 0},//x
																		{0, 1, 0},//y
																		{0, 0, 1} //z
																	};

int8_t imt8310_orientation[3][3] = {	
																		{1, 0, 0},//x
																		{0, 1, 0},//y
																		{0, 0, 1} //z
																	};

	/**
	* @brief  get the data of imu
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void mpu_get_data()
{
		int i,hz=1000;//hz:运行周期
		uint16_t tempPWM;
		BMI088_readraw(gyro, accel, &temp);
	
		mpu_data.ax   = accel[0]-mpu_data.ax_offset;
    mpu_data.ay   = accel[1]-mpu_data.ay_offset;
    mpu_data.az   = accel[2]-mpu_data.az_offset;
    mpu_data.temp = temp;

    mpu_data.gx = (gyro[0] - mpu_data.gx_offset);
    mpu_data.gy = (gyro[1] - mpu_data.gy_offset);
    mpu_data.gz = (gyro[2] - mpu_data.gz_offset);
		
		/*  传感器旋转角度 */
		for(i = 0; i < 3; i++)
		{
			mpu_data.acc_val_ref[i] = mpu_data.ax * bmi088_orientation[i][0] + mpu_data.ay * bmi088_orientation[i][1] + mpu_data.az * bmi088_orientation[i][2];
			mpu_data.gryo_val_ref[i] = mpu_data.gx * bmi088_orientation[i][0] + mpu_data.gy * bmi088_orientation[i][1] + mpu_data.gz * bmi088_orientation[i][2];
		}
	
		/*  软件低通滤波 */
		for(i=0;i<3;i++)
		{
			mpu_data.accel_filter[i+x]+=0.1f*(mpu_data.acc_val_ref[i]-mpu_data.acc_val[i+x]);//0.75
			mpu_data.gryo_filter[i+x]+=0.75f*(mpu_data.gryo_val_ref[i]-mpu_data.gryo_val[i+x]);
		}
		/* 旋转加速度补偿 */
		for(i=0;i<3;i++)
		{
			center_pos.gryo_old_rad_value[i]=center_pos.gryo_rad_value[i];
			center_pos.gryo_rad_value[i]=mpu_data.gryo_filter[i]*GYRO_PN2000_RAD;
			center_pos.center_gryo_rad_acc[i]=(center_pos.gryo_rad_value[i]-center_pos.gryo_old_rad_value[i])*hz;
		}
		center_pos.center_liner_acc[x]=center_pos.center_gryo_rad_acc[y]*center_pos.center_pos_cm[z]+center_pos.center_gryo_rad_acc[z]*center_pos.center_pos_cm[y];
		center_pos.center_liner_acc[y]=center_pos.center_gryo_rad_acc[x]*center_pos.center_pos_cm[z]+center_pos.center_gryo_rad_acc[z]*center_pos.center_pos_cm[x];
		center_pos.center_liner_acc[z]=center_pos.center_gryo_rad_acc[y]*center_pos.center_pos_cm[x]+center_pos.center_gryo_rad_acc[x]*center_pos.center_pos_cm[y];
		/* 赋值 */
		for(i=0;i<3;i++)
		{
			mpu_data.acc_val[i] = mpu_data.accel_filter[i]-center_pos.center_liner_acc[i];
			mpu_data.gryo_val[i] = mpu_data.gryo_filter[i];
		}

    imu.temp = mpu_data.temp* BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;;
		/* +-3g -> cm/ss */
		imu.ax   = mpu_data.acc_val[x]*ACC_PN3G_TO_CMSS;
		imu.ay   = mpu_data.acc_val[y]*ACC_PN3G_TO_CMSS;
		imu.az   = mpu_data.acc_val[z]*ACC_PN3G_TO_CMSS;
	  /* 2000dps -> rad/s */
		for(int i=0;i<3;i++)
		{
			// 2000dps -> °/s
			mpu_data.gryo_deg[i]= mpu_data.gryo_val[i]*0.061036f;
			mpu_data.gryo_deg_lpf[i]+=0.75f*(mpu_data.gryo_deg[i]-mpu_data.gryo_deg_lpf[i]);
		}
		imu.wx =mpu_data.gryo_deg_lpf[x]*0.01745f;
		imu.wy=mpu_data.gryo_deg_lpf[y]*0.01745f;
		imu.wz	=mpu_data.gryo_deg_lpf[z]*0.01745f;
		
		if(use_mag)
		{
			ist8310_read_mag(mag);
			imu.mx = mag[0];
			imu.my = mag[1];
			imu.mz = mag[2];
		}
		else
		{
			imu.mx = 0;
			imu.my = 0;
			imu.mz = 0;
		}
		
		add_hisimu(imu);
		
		//恒温PID计算
		pid_calc(&PID_IMU_Tmp, imu.temp, set_temp);
		if (PID_IMU_Tmp.out < 0.0f)
		{
				PID_IMU_Tmp.out = 0.0f;
		}
		tempPWM = (uint16_t)PID_IMU_Tmp.out;
		imu_pwm_set(tempPWM);
}


/**
	* @brief  set imu 6500 gyroscope measure range
  * @param  fsr: range(0,卤250dps;1,卤500dps;2,卤1000dps;3,卤2000dps)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t mpu_set_gyro_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU_RA_GYRO_CONFIG, fsr << 3);
}


/**
	* @brief  set imu 6050/6500 accelerate measure range
  * @param  fsr: range(0,卤2g;1,卤4g;2,卤8g;3,卤16g)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t mpu_set_accel_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU_RA_ACCEL_CONFIG, fsr << 3); 
}

uint8_t id;
uint8_t FastMode = 1;
/**
	* @brief  initialize imu mpu6500 and magnet meter ist3810
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
uint8_t Mpu_Device_Init(void)
{
	uint8_t tmp;
	MPU_DELAY(100);
	
	
	id                               = mpu_read_byte(MPU_RA_WHO_AM_I);
	
	uint8_t i                        = 0;
	icm20602_writeBit ( MPUREG_INT_PIN_CFG, ICM_INTMODE_ACTIVEHIGH, ICM_INTCFG_INT_LEVEL_BIT );
	icm20602_writeBit ( MPUREG_INT_PIN_CFG, ICM_INTDRV_PUSHPULL , ICM_INTCFG_INT_OPEN_BIT);
	icm20602_writeBit ( MPUREG_INT_PIN_CFG, ICM_INTLATCH_50USPULSE, ICM_INTCFG_LATCH_INT_EN_BIT);//MPU6050_INTLATCH_WAITCLEAR );
	icm20602_writeBit ( MPUREG_INT_PIN_CFG, ICM_INTCLEAR_ANYREAD ,ICM_INTCFG_INT_RD_CLEAR_BIT);

	icm20602_writeBit ( MPUREG_INT_ENABLE,1,ICM_INTERRUPT_DATA_RDY_BIT );
	uint8_t ICM20602_Init_Data[12][2] = {{ MPU_RA_PWR_MGMT_1, 0x80 },     /* Reset Device */ 
	                                     { MPU_RA_PWR_MGMT_1, 0x01 },     /* Clock Source - Gyro-Z */ 
																			 { MPU_RA_DMP_CFG_1,  0x40 },																		 
																			 { MPU_RA_PWR_MGMT_2, 0x00 },    /* Enable Acc & Gyro */ 
																			 { MPU_RA_SMPLRT_DIV, 0x00 },
																		   { MPU_RA_CONFIG, 0x03 },         /* LPF 41Hz */   //0
																			 { MPU_RA_GYRO_CONFIG, 0x18 },    /* +-2000dps */ 
																			 { MPU_RA_ACCEL_CONFIG, 0x10 },   /* +-8G */ 
																			 { MPU_RA_FF_THR, 0x04 }, /* enable LowPassFilter  Set Acc LPF */ 
																			 { MPU_RA_USER_CTRL, 0x01 },/* Enable AUX */ 
	                                     { MPU_RA_FF_DUR, 0x00 },
																			 { MPU_RA_FIFO_EN, 0x00 },};    
	for (i = 0; i < 12; i++)
	{
		mpu_write_byte(ICM20602_Init_Data[i][0], ICM20602_Init_Data[i][1]);
		MPU_DELAY(1);
	}
	mpu_read_bytes (MPUREG_WHOAMI, &tmp,1 );
	id = mpu_read_byte(MPU_RA_WHO_AM_I);
	center_pos_set();
	mpu_set_gyro_fsr(3); 		
	mpu_set_accel_fsr(2);
	
//	ist8310_init();
	
	//使用蜂鸣器提醒即将开始校准陀螺仪
	
	if(FastMode)//快速启动模式
	{
		//直接校准
		mpu_offset_call();
	}
	else
	{
		//蜂鸣器提醒后校准
		buzzer_on(1, 30000);
		MPU_DELAY(1000);
		buzzer_off();
		MPU_DELAY(50);
		
		buzzer_on(1, 30000);
		MPU_DELAY(50);
		buzzer_off();
		MPU_DELAY(50);
		
		buzzer_on(1, 40000);
		MPU_DELAY(50);
		buzzer_off();
		MPU_DELAY(50);
		
		buzzer_on(1, 40000);
		MPU_DELAY(50);
		buzzer_off();
		MPU_DELAY(50);
		
		//开始校准陀螺仪
		buzzer_on(1, 60000);
		mpu_offset_call();
		MPU_DELAY(1000);
		buzzer_off();
		
		//校准结束提示音
		buzzer_on(1, 30000);
		MPU_DELAY(50);
		buzzer_off();
		MPU_DELAY(50);
		
		buzzer_on(1, 30000);
		MPU_DELAY(50);
		buzzer_off();
		MPU_DELAY(50);
	}
	return 0;
}

/**
	* @brief  get the offset data of MPU6500
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void mpu_offset_call(void)
{
	int i;
	for (i=0; i<100;i++)
	{
		//mpu_read_bytes(MPU_RA_ACCEL_XOUT_H, mpu_buff, 14);
		BMI088_readraw(gyro, accel, &temp);

//		mpu_data.ax_offset += mpu_buff[0] << 8 | mpu_buff[1];
//		mpu_data.ay_offset += mpu_buff[2] << 8 | mpu_buff[3];
//		mpu_data.az_offset += (mpu_buff[4] << 8 | mpu_buff[5])-4096;
//	
//		mpu_data.gx_offset += mpu_buff[8]  << 8 | mpu_buff[9];
//		mpu_data.gy_offset += mpu_buff[10] << 8 | mpu_buff[11];
//		mpu_data.gz_offset += mpu_buff[12] << 8 | mpu_buff[13];
//		mpu_data.ax_offset += accel[0];
//		mpu_data.ay_offset += accel[1];
//		mpu_data.az_offset += accel[2];//-4096;
		
		mpu_data.ax_offset = 0;
		mpu_data.ay_offset = 0;
		mpu_data.az_offset = 0;//-4096;
	
		mpu_data.gx_offset += gyro[0];
		mpu_data.gy_offset += gyro[1];
		mpu_data.gz_offset += gyro[2];
	 
		MPU_DELAY(5);
	}
	mpu_data.ax_offset=mpu_data.ax_offset / 100;
	mpu_data.ay_offset=mpu_data.ay_offset / 100;
	mpu_data.az_offset=mpu_data.az_offset / 100;
	mpu_data.gx_offset=mpu_data.gx_offset / 100;
	mpu_data.gy_offset=mpu_data.gy_offset / 100;
	mpu_data.gz_offset=mpu_data.gz_offset / 100;
}



/**
	* @brief  Initialize quaternion
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void init_quaternion(void)
{
	int16_t hx, hy;//hz;
	
	hx = imu.mx;
	hy = imu.my;
	//hz = imu.mz;
	
	#ifdef BOARD_DOWN
	if (hx < 0 && hy < 0) 
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = -0.005;
			q1 = -0.199;
			q2 = 0.979;
			q3 = -0.0089;
		}
		else
		{
			q0 = -0.008;
			q1 = -0.555;
			q2 = 0.83;
			q3 = -0.002;
		}
		
	}
	else if (hx < 0 && hy > 0)
	{
		if (fabs(hx / hy)>=1)   
		{
			q0 = 0.005;
			q1 = -0.199;
			q2 = -0.978;
			q3 = 0.012;
		}
		else
		{
			q0 = 0.005;
			q1 = -0.553;
			q2 = -0.83;
			q3 = -0.0023;
		}
		
	}
	else if (hx > 0 && hy > 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.0012;
			q1 = -0.978;
			q2 = -0.199;
			q3 = -0.005;
		}
		else
		{
			q0 = 0.0023;
			q1 = -0.83;
			q2 = -0.553;
			q3 = 0.0023;
		}
		
	}
	else if (hx > 0 && hy < 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.0025;
			q1 = 0.978;
			q2 = -0.199;
			q3 = 0.008;			
		}
		else
		{
			q0 = 0.0025;
			q1 = 0.83;
			q2 = -0.56;
			q3 = 0.0045;
		}		
	}
	#else
		if (hx < 0 && hy < 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.195;
			q1 = -0.015;
			q2 = 0.0043;
			q3 = 0.979;
		}
		else
		{
			q0 = 0.555;
			q1 = -0.015;
			q2 = 0.006;
			q3 = 0.829;
		}
		
	}
	else if (hx < 0 && hy > 0)
	{
		if(fabs(hx / hy) >= 1)
		{
			q0 = -0.193;
			q1 = -0.009;
			q2 = -0.006;
			q3 = 0.979;
		}
		else
		{
			q0 = -0.552;
			q1 = -0.0048;
			q2 = -0.0115;
			q3 = 0.8313;
		}
		
	}
	else if (hx > 0 && hy > 0)
	{
		if(fabs(hx / hy) >= 1)
		{
			q0 = -0.9785;
			q1 = 0.008;
			q2 = -0.02;
			q3 = 0.195;
		}
		else
		{
			q0 = -0.9828;
			q1 = 0.002;
			q2 = -0.0167;
			q3 = 0.5557;
		}
		
	}
	else if (hx > 0 && hy < 0)
	{
		if(fabs(hx / hy) >= 1)
		{
			q0 = -0.979;
			q1 = 0.0116;
			q2 = -0.0167;
			q3 = -0.195;			
		}
		else
		{
			q0 = -0.83;
			q1 = 0.014;
			q2 = -0.012;
			q3 = -0.556;
		}		
	}
	#endif
}

/**
	* @brief  update imu AHRS
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void imu_ahrs_update(void) 
{
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez, halfT;
	float tempq0,tempq1,tempq2,tempq3;

	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;   
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;   

	gx = imu.wx;
	gy = imu.wy;
	gz = imu.wz;
	ax = imu.ax;
	ay = imu.ay;
	az = imu.az;
	mx = imu.mx;
	my = imu.my;
	mz = imu.mz;

//	now_update  = GetTick(); //us
//	halfT       = ((float)(now_update - last_update) / 2000000.0f);
//	last_update = now_update;
 halfT=1.0f/2000.0f;

	/* Fast inverse square-root */
	norm = inv_sqrt(ax*ax + ay*ay + az*az);       
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	
	#ifdef IST8310
		norm = inv_sqrt(mx*mx + my*my + mz*mz);          
		mx = mx * norm;
		my = my * norm;
		mz = mz * norm; 
	#else
		mx = 0;
		my = 0;
		mz = 0;		
	#endif
	/* compute reference direction of flux */
	hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
	hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
	hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
	bx = sqrt((hx*hx) + (hy*hy));
	bz = hz; 
	
	/* estimated direction of gravity and flux (v and w) */
	vx = 2.0f*(q1q3 - q0q2);
	vy = 2.0f*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
	wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
	wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
	
	/* 
	 * error is sum of cross product between reference direction 
	 * of fields and direction measured by sensors 
	 */
	ex = (ay*vz - az*vy) + (my*wz - mz*wy);
	ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
	ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

	/* PI */
	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		exInt = exInt + ex * Ki * halfT;
		eyInt = eyInt + ey * Ki * halfT;	
		ezInt = ezInt + ez * Ki * halfT;
		
		gx = gx + Kp*ex + exInt;
		gy = gy + Kp*ey + eyInt;
		gz = gz + Kp*ez + ezInt;
	}
	
	tempq0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
	tempq1 = q1 + (q0*gx + q2*gz - q3*gy) * halfT;
	tempq2 = q2 + (q0*gy - q1*gz + q3*gx) * halfT;
	tempq3 = q3 + (q0*gz + q1*gy - q2*gx) * halfT;  

	/* normalise quaternion */
	norm = inv_sqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
	q0 = tempq0 * norm;
	q1 = tempq1 * norm;
	q2 = tempq2 * norm;
	q3 = tempq3 * norm;
}

/**
	* @brief  update imu attitude
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void imu_attitude_update(void)
{
	/* yaw    -pi----pi */
	imu.yaw = -atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1)* 57.3; 
	/* pitch  -pi/2----pi/2 */
	imu.pit = -asin(-2*q1*q3 + 2*q0*q2)* 57.3;   
	/* roll   -pi----pi  */	
	imu.rol =  atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)* 57.3;
}

