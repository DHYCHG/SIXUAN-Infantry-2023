#include "bsp_spi.h"
uint8_t SPI2_Read_Write_Byte(uint8_t TxData)
{
	uint8_t rxdata ;
	HAL_SPI_TransmitReceive(&hspi2,&TxData,&rxdata,1,100);
	return rxdata;
}


