#include "bsp_can.h"


void CAN1_Filter_Init()
{
	CAN_FilterTypeDef fcan1;

	fcan1.FilterMode = CAN_FILTERMODE_IDMASK;
	fcan1.FilterScale = CAN_FILTERSCALE_32BIT;
	fcan1.FilterIdHigh = 0x0000;
	fcan1.FilterIdLow = 0x0000;
	fcan1.FilterMaskIdHigh = 0x0000;
	fcan1.FilterMaskIdLow = 0x0000;		
	fcan1.FilterBank = 0;
	fcan1.FilterFIFOAssignment = CAN_RX_FIFO0;
	fcan1.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan1,&fcan1);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_TX_MAILBOX_EMPTY);
	
}
void CAN2_Filter_Init()
{
	CAN_FilterTypeDef fcan2;
	fcan2.FilterMode = CAN_FILTERMODE_IDMASK;
	fcan2.FilterScale = CAN_FILTERSCALE_32BIT;
	fcan2.FilterIdHigh = 0x0000;
	fcan2.FilterIdLow = 0x0000;
	fcan2.FilterMaskIdHigh = 0x0000;
	fcan2.FilterMaskIdLow = 0x0000;		
	fcan2.FilterBank = 14;
	fcan2.FilterFIFOAssignment = CAN_RX_FIFO0;
	fcan2.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan2,&fcan2);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_TX_MAILBOX_EMPTY);

}
