#include "bsp_usart.h"
#include "usart.h"
/**
 * @Name: USART3_RX_DMA_Init
 * @Description: 遥控器接收使用到IO口的初始化
 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: 这是遥控器接收IO口的初始化
 */
uint8_t _USART3_DMA_RX_BUF[2][BSP_USART3_DMA_RX_BUF_LEN];
void USART3_RX_DMA_Init()
{
	SET_BIT(huart3.Instance->CR3,USART_CR3_DMAR);//使能串口1的DMA接收
	__HAL_DMA_DISABLE(&hdma_usart3_rx);
	while(hdma_usart3_rx.Instance->CR&DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(&hdma_usart3_rx);
	}
	hdma_usart3_rx.Instance->M0AR=(uint32_t)&_USART3_DMA_RX_BUF[0][0];
	hdma_usart3_rx.Instance->M1AR=(uint32_t)&_USART3_DMA_RX_BUF[1][0];
	hdma_usart3_rx.Instance->PAR=(uint32_t)&USART3->DR;
	__HAL_DMA_SET_COUNTER(&hdma_usart3_rx,BSP_USART3_DMA_RX_BUF_LEN);
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
	__HAL_DMA_ENABLE(&hdma_usart3_rx);

	HAL_NVIC_SetPriority(USART3_IRQn,0,0);
	HAL_NVIC_EnableIRQ(USART3_IRQn);
	
}

void USART3_IRQHandler(void)
{
	static uint16_t this_time_rx_len;
	if(__HAL_UART_GET_IT_SOURCE(&huart3,UART_IT_IDLE)!=RESET)
	{
		__HAL_UART_CLEAR_PEFLAG(&huart3);
		if((hdma_usart3_rx.Instance->CR&DMA_SxCR_CT)==RESET)
		{
			__HAL_DMA_DISABLE(&hdma_usart3_rx);	
			this_time_rx_len = BSP_USART3_DMA_RX_BUF_LEN - hdma_usart3_rx.Instance->NDTR;
			__HAL_DMA_SET_COUNTER(&hdma_usart3_rx,BSP_USART3_DMA_RX_BUF_LEN);
			hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT; 
			__HAL_DMA_ENABLE(&hdma_usart3_rx);
			__HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx,DMA_HISR_TCIF7);
			if(this_time_rx_len==RC_FRAME_LENGTH)
				RemoteDataPrcess(_USART3_DMA_RX_BUF[0]);
		}
		else
		{
			__HAL_DMA_DISABLE(&hdma_usart3_rx);
			this_time_rx_len = BSP_USART3_DMA_RX_BUF_LEN - hdma_usart3_rx.Instance->NDTR;
			__HAL_DMA_SET_COUNTER(&hdma_usart3_rx,BSP_USART3_DMA_RX_BUF_LEN);
			hdma_usart3_rx.Instance->CR &= ~DMA_SxCR_CT;
			__HAL_DMA_ENABLE(&hdma_usart3_rx);
			__HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx,DMA_HISR_TCIF7);
			if(this_time_rx_len==RC_FRAME_LENGTH)
				RemoteDataPrcess(_USART3_DMA_RX_BUF[1]);
		}
	}
}
/**
 * @Name: USART1_DMA_RX_Init
 * @Description:串口1初始化
	 此串口为视觉所用
 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: void
 */
SPACE_e VISUAL_DATA[2];
void USART1_DMA_RX_Init()
{
	SET_BIT(huart1.Instance->CR3,USART_CR3_DMAR);
	__HAL_DMA_DISABLE(&hdma_usart1_rx);
	while(hdma_usart1_rx.Instance->CR&DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(&hdma_usart1_rx);
	}
	hdma_usart1_rx.Instance->M0AR = (uint32_t)&VISUAL_DATA[0];
	hdma_usart1_rx.Instance->M1AR = (uint32_t)&VISUAL_DATA[1];
	hdma_usart1_rx.Instance->PAR = (uint32_t)&USART1->DR;
	__HAL_DMA_SET_COUNTER(&hdma_usart1_rx,sizeof(VISUAL_DATA)/2);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	__HAL_DMA_ENABLE(&hdma_usart1_rx);
	
	HAL_NVIC_SetPriority(USART1_IRQn,2,0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
 * @Name: USART1_IRQHandler
 * @Description: 视觉的接收函数及数据处理
 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: void
 */
float X_Axis=0;
float Y_Axis=0;

int DataError_Times=0;
void USART1_IRQHandler()
{
	static uint16_t this_time_rx_len;
	if(__HAL_UART_GET_IT_SOURCE(&huart1,UART_IT_IDLE) != RESET)
	{
		__HAL_UART_CLEAR_PEFLAG(&huart1);
		if((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
		{
			__HAL_DMA_DISABLE(&hdma_usart1_rx);
			this_time_rx_len = BSP_USART1_DMA_RX_BUF_LEN - hdma_usart1_rx.Instance->NDTR;
			__HAL_DMA_SET_COUNTER(&hdma_usart1_rx,BSP_USART1_DMA_RX_BUF_LEN);
			hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
			__HAL_DMA_ENABLE(&hdma_usart1_rx);
			__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_HISR_TCIF7);
			if(VISUAL_DATA[0].Auto_data.Start0==3&&VISUAL_DATA[0].Auto_data.Start1==252)
			{
//				X_Axis = VISUAL_DATA[0].Auto_data.X;
//				Y_Axis=VISUAL_DATA[0].Auto_data.Y;
			}
			else
				DataError_Times++;
		}
		else 
		{
			__HAL_DMA_DISABLE(&hdma_usart1_rx);
			this_time_rx_len = BSP_USART1_DMA_RX_BUF_LEN - hdma_usart1_rx.Instance->NDTR;
			__HAL_DMA_SET_COUNTER(&hdma_usart1_rx,BSP_USART1_DMA_RX_BUF_LEN);
			hdma_usart1_rx.Instance->CR &= ~DMA_SxCR_CT;
			__HAL_DMA_ENABLE(&hdma_usart1_rx);
			__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_HISR_TCIF7);
			if(VISUAL_DATA[1].Auto_data.Start0==3&&VISUAL_DATA[1].Auto_data.Start1==252)
			{
				X_Axis = VISUAL_DATA[1].Auto_data.X;
				Y_Axis=VISUAL_DATA[1].Auto_data.Y;
			}
			else 
				DataError_Times++;
		}
	}
}

/**
 * @Name: USART3_DMA_RX_Init
 * @Description:串口3
				USART3----PD8-----TX
				USART3----PD9------RX
				BaudRate------ 115200
	 此串口为裁判系统通信所用，此串口对应A型板USART3接口
 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: void
 */
uint8_t Receive_Data[2][BSP_USART6_RX_BUF_LEN];
void USART6_DMA_RX_Init()
{
	SET_BIT(huart6.Instance->CR3,USART_CR3_DMAR);
	SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);
	__HAL_DMA_DISABLE(&hdma_usart6_rx);
	while(hdma_usart6_rx.Instance->CR&DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(&hdma_usart6_rx);
	}
	hdma_usart6_rx.Instance->M0AR = (uint32_t)&Receive_Data[1][0];
	hdma_usart6_rx.Instance->M1AR = (uint32_t)&Receive_Data[1][0];
	hdma_usart6_rx.Instance->PAR = (uint32_t)&USART6->DR;
	__HAL_DMA_SET_COUNTER(&hdma_usart6_rx,BSP_USART6_RX_BUF_LEN);
	__HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE);//闲时中断
	__HAL_DMA_ENABLE(&hdma_usart6_rx);
	
	    //DMA发送
    __HAL_DMA_DISABLE(&hdma_usart6_tx);
    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }
    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);
	HAL_NVIC_SetPriority(USART6_IRQn,1,0);
	HAL_NVIC_EnableIRQ(USART6_IRQn);
}

/**
 * @Name: USART3_IRQHandler
 * @Description: 串口3中断函数，裁判系统数据接收处理
 * @Param: void
 * @Return: void
 * @Author: source
 * @Warning: void
 */
void USART6_IRQHandler()
{
	HAL_UART_IRQHandler(&huart6);
	static uint16_t this_time_rx_len;
	if(__HAL_UART_GET_IT_SOURCE(&huart6,UART_IT_IDLE) != RESET)
	{
		__HAL_UART_CLEAR_PEFLAG(&huart6);
		if((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
		{
			__HAL_DMA_DISABLE(&hdma_usart6_rx);
			this_time_rx_len = BSP_USART6_RX_BUF_LEN - hdma_usart6_rx.Instance->NDTR;
			__HAL_DMA_SET_COUNTER(&hdma_usart6_rx,BSP_USART6_RX_BUF_LEN);
			hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
			__HAL_DMA_ENABLE(&hdma_usart6_rx);
			__HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx,DMA_HISR_TCIF7);
			QUEUE_PacketIn(&Queue_Date,(uint8_t*)Receive_Data[0],this_time_rx_len);
		}
		else 
		{
			__HAL_DMA_DISABLE(&hdma_usart6_rx);
			this_time_rx_len = BSP_USART6_RX_BUF_LEN - hdma_usart6_rx.Instance->NDTR;
			__HAL_DMA_SET_COUNTER(&hdma_usart6_rx,BSP_USART6_RX_BUF_LEN);
			hdma_usart6_rx.Instance->CR &= ~DMA_SxCR_CT;
			__HAL_DMA_ENABLE(&hdma_usart6_rx);
			__HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx,DMA_HISR_TCIF7);
			QUEUE_PacketIn(&Queue_Date,(uint8_t*)Receive_Data[1],this_time_rx_len);
		}
	}
}
