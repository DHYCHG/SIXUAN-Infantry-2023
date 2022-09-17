/*
 * @Name: lolt.c
 * @Description: volt+��λ���ײ����,����Volt_Sendware����DATA�ṹ��+VOLT�����幩����
 * @Author: source
 * @Copyright: SixuanRobomasterLab
 */
#include "vofaplus.h"
#include "stdio.h"
volt_un volt;
extern float									hisimu[10][3];//����10����ʷ��̬��
extern RC_Ctrl_t RC_CtrlData;
/**
 * @Name: Volt_Sendware
 * @Description:���� Э��     ��scheduler_task�ļ����е���Volt_Sendware());����
 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void Vofaplus_Sendware()
{
	int i;
	volt.Volt_Data.Data[0] = (float)imu.rol;
	volt.Volt_Data.Data[1] = (float)imu.pit;
	volt.Volt_Data.Data[2] = (float)imu.yaw;
	volt.Volt_Data.Data[3] = (float)imu.temp;
	
	for(i = 4; i < 14; i++)
		volt.Volt_Data.Data[i] = (float)hisimu[i-4][2];
	
	volt.Volt_Data.Data[14] = (float)RC_CtrlData.mouse.x;
	volt.Volt_Data.Data[15] = (float)RC_CtrlData.mouse.y;
	volt.Volt_Data.Data[16] = (float)RC_CtrlData.mouse.z;
	
	volt.Volt_Data.End=0x7f800000;		
	HAL_UART_Transmit(&huart6,volt.Sent,sizeof(volt.Sent),HAL_MAX_DELAY);
}
