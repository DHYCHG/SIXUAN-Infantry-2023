#ifndef __BSP_USART_H
#define __BSP_USART_H
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "remote_task.h"
#define BSP_USART3_DMA_RX_BUF_LEN 30u//Ò£¿ØÆ÷
#define BSP_USART6_RX_BUF_LEN 256//²ÃÅÐÏµÍ³
#define BSP_USART1_DMA_RX_BUF_LEN 14u//ÊÓ¾õ
#define RC_FRAME_LENGTH 18u
extern uint8_t Receive_Data[2][BSP_USART6_RX_BUF_LEN];
typedef struct
{
	char Start0;
	char Start1; 
  short	Delay_Error;
	float X;
	float Y;
	char Flag;
	char Stop;
}AUTO_AIM_DATA_t;
typedef  union
{
	char Buff[14];
	AUTO_AIM_DATA_t Auto_data;
}SPACE_e;
extern float X_Axis;
extern float Y_Axis;

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

//extern DMA_HandleTypeDef hdma_usart1_tx;
//extern DMA_HandleTypeDef hdma_usart1_rx;
//extern DMA_HandleTypeDef hdma_usart3_rx;
//extern DMA_HandleTypeDef hdma_usart6_rx;
//extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

void USART1_DMA_RX_Init(void);
void USART3_RX_DMA_Init(void);
void USART6_DMA_RX_Init(void);
#endif 


