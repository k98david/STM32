#include "stm32f10x_it.h"
#include "config.h"
#include "QUart.h"
#include "stdbool.h"   //for bool type
#include "QTimer.h"
#include "QGPIO.h"
#include "stm32_eval.h"
#include "QuanProcessCMD.h"

USART_InitTypeDef USART_InitStructure;

/*UART*/
//BLE SMT32 UART3 BLE Config
uint8_t RxBuffer_BLE[RxBufferSize3];
uint8_t TxBuffer_BLE[TxBufferSize3];
uint8_t TxHead3 = 0; 
uint8_t TxTail3 = 0;
uint8_t RxHead3 = 0; 
uint8_t RxTail3 = 0;
//BLE SMT32 UART5 MTK Config
uint8_t RxBuffer_MTK[RxBufferSize5];
uint8_t TxBuffer_MTK[TxBufferSize5];
uint8_t TxHead5 = 0; 
uint8_t TxTail5 = 0;
uint8_t RxHead5 = 0; 
uint8_t RxTail5 = 0;
//RS485 UART4 Config
uint8_t RxBuffer_UART4[RxBufferSize4];
uint8_t TxBuffer_UART4[TxBufferSize4];
uint8_t TxHead4 = 0; 
uint8_t TxTail4 = 0;
uint8_t RxHead4 = 0; 
uint8_t RxTail4 = 0;
//UART1 Config
uint8_t RxBuffer_UART1[RxBufferSize1];
uint8_t TxBuffer_UART1[TxBufferSize1];
uint8_t TxHead1 = 0; 
uint8_t TxTail1 = 0;
uint8_t RxHead1 = 0; 
uint8_t RxTail1 = 0;
//BLE SMT32 UART2 Z200 Config
uint8_t RxBuffer_Z200[RxBufferSize2];
uint8_t TxBuffer_Z200[TxBufferSize2];
uint8_t TxHead2 = 0; 
uint8_t TxTail2 = 0;
uint8_t RxHead2 = 0; 
uint8_t RxTail2 = 0;

uint8_t gTurn485ReceiveMode = 0;
/* function */
void USARTMTK_to_UARTBLE_long(uint8_t *BufferPtr, uint32_t Length);
void UARTBLE_to_USARTMTK_long(uint8_t *BufferPtr, uint32_t Length);
void USARTMTK_to_UART2_long(uint8_t *BufferPtr, uint32_t Length);
void UART2_to_USARTMTK_long(uint8_t *BufferPtr, uint32_t Length);


bool Uart_IDIE_flag=false;
extern QTimer_t gRXTimer;

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles USARTy global interrupt request.
  * @param  None
  * @retval None
  */
void USARTMTK_IRQHandler(void)  //UART5
{
   //uint8_t x=0;
				STM_EVAL_LEDOn(LEDR);  //DEbug
  if((USART_GetFlagStatus(USARTMTK, USART_FLAG_ORE) != RESET)||USART_GetITStatus(USARTMTK, USART_IT_RXNE) != RESET)
  {
		UAART_RX_Port=UARTRX_MTK;  //mark UART5
		SysTick->CTRL |=SysTick_CTRL_ENABLE_Msk;
	  USART_ClearITPendingBit(USARTMTK, USART_IT_RXNE);
		USART_ClearITPendingBit(USARTMTK, USART_FLAG_ORE);
		//USART_ClearFlag(USARTMTK,USART_FLAG_RXNE);  //?SR 
    /* Read one byte from the receive data register */
    RxBuffer_MTK[RxHead5++] = USART_ReceiveData(USARTMTK);

    if(RxHead5 >= RxBufferSize5)
    {
      /* Disable the USARTz Receive interrupt */
      //USART_ITConfig(USARTBLE, USART_IT_RXNE, DISABLE);
			RxHead5=0;
    }
		gRXTimer.enabled=true;  //for RX time out
		gRXTimer.counter=0;
  }
	
	if(USART_GetFlagStatus(USARTMTK, USART_FLAG_ORE) != RESET)
	{
			USART_ClearFlag(USARTMTK, USART_FLAG_ORE);
			USART_ReceiveData(USARTMTK);
	}
	
  if(USART_GetITStatus(USARTMTK, USART_IT_TXE) != RESET)
  {   

			if(TxHead5==TxTail5){
			  //USART_ClearITPendingBit(USARTMTK, USART_IT_TXE);    //UART can't ClearIT if ClearIT mcu be Crash 
				/* Disable the USARTz Transmit interrupt */
				USART_ITConfig(USARTMTK, USART_IT_TXE, DISABLE);
			}else{
				USART_SendData(USARTMTK, TxBuffer_MTK[TxTail5++]);
				if (TxTail5 >=RxBufferSize5){
					TxTail5=0;
				}
			}
  }
	
						STM_EVAL_LEDOff(LEDR);  //DEbug
}

/**
  * @brief  This function handles USARTz global interrupt request.
  * @param  None
  * @retval None
  */
void USARTBLE_IRQHandler(void)   //UART3
{
						STM_EVAL_LEDOn(LEDR);  //DEbug
  if((USART_GetFlagStatus(USARTBLE, USART_FLAG_ORE) != RESET)||USART_GetITStatus(USARTBLE, USART_IT_RXNE) != RESET)
  {
		UAART_RX_Port=UARTRX_BLE;  //mark 
		SysTick->CTRL |=SysTick_CTRL_ENABLE_Msk;
		USART_ClearITPendingBit(USARTBLE, USART_IT_RXNE);
		USART_ClearITPendingBit(USARTBLE, USART_FLAG_ORE);
		//USART_ClearFlag(USARTBLE,USART_FLAG_RXNE);  //?SR 
    /* Read one byte from the receive data register */
    RxBuffer_BLE[RxHead3++] = USART_ReceiveData(USARTBLE);

    if(RxHead3 >= RxBufferSize3)
    {
			RxHead3=0;
    }
		gRXTimer.enabled=true;  //for RX time out
		gRXTimer.counter=0;
		//Uart_IDIE_flag=false;
  }
	
	if(USART_GetFlagStatus(USARTBLE, USART_FLAG_ORE) != RESET)
	{
			USART_ClearFlag(USARTBLE, USART_FLAG_ORE);
			USART_ReceiveData(USARTBLE);
	}
	
  if(USART_GetITStatus(USARTBLE, USART_IT_TXE) != RESET)
  { 
		if(TxHead3==TxTail3){
			//USART_ClearITPendingBit(USARTBLE, USART_IT_TXE);
			/* Disable the USARTz Transmit interrupt */
			USART_ITConfig(USARTBLE, USART_IT_TXE, DISABLE);
		}else{
			USART_SendData(USARTBLE, TxBuffer_BLE[TxTail3++]);
			if (TxTail3 >=RxBufferSize3){
				TxTail3=0;
			}
		}
  }
  
							STM_EVAL_LEDOff(LEDR);  //DEbug
}
/**
  * @brief  This function handles USARTz global interrupt request.
  * @param  None
  * @retval None
  */
void USARTUART1_IRQHandler(void)   //UART3
{
  if((USART_GetFlagStatus(USARTUART1, USART_FLAG_ORE) != RESET)||USART_GetITStatus(USARTUART1, USART_IT_RXNE) != RESET)
  {
		//UAART_RX_Port=UARTRX_BLE;  //mark 
	  USART_ClearITPendingBit(USARTUART1, USART_IT_RXNE);
		USART_ClearITPendingBit(USARTUART1, USART_FLAG_ORE);
    /* Read one byte from the receive data register */
    RxBuffer_UART1[RxHead1++] = USART_ReceiveData(USARTUART1);

    if(RxHead1 >= RxBufferSize1)
    {
			RxHead1=0;
    }
		gRXTimer.enabled=true;  //for RX time out
		gRXTimer.counter=4;
  }

	
  
  if(USART_GetITStatus(USARTUART1, USART_IT_TXE) != RESET)
  {   
		if(TxHead1==TxTail1){
			//USART_ClearITPendingBit(USARTUART1, USART_IT_TXE);
			/* Disable the USARTz Transmit interrupt */
			USART_ITConfig(USARTUART1, USART_IT_TXE, DISABLE);
		}else{
			USART_SendData(USARTUART1, TxBuffer_UART1[TxTail1++]);
			if (TxTail1 >=RxBufferSize1){
				TxTail1=0;
			}
			//while (USART_GetFlagStatus(USARTUART2, USART_FLAG_TC) == RESET);  //for wait tx finish
		}
  }
}

/**
  * @brief  This function handles USARTz global interrupt request.
  * @param  None
  * @retval None
  */
void USARTUART2_IRQHandler(void)   //UART3
{
  if((USART_GetFlagStatus(USARTUART2, USART_FLAG_ORE) != RESET)||USART_GetITStatus(USARTUART2, USART_IT_RXNE) != RESET)
  {
		UAART_RX_Port=UARTRX_Zigbee;  //mark 
	  SysTick->CTRL |=SysTick_CTRL_ENABLE_Msk;
	  USART_ClearITPendingBit(USARTUART2, USART_IT_RXNE);
		//USART_ClearITPendingBit(USARTUART2, USART_FLAG_ORE);
    /* Read one byte from the receive data register */
    RxBuffer_Z200[RxHead2++] = USART_ReceiveData(USARTUART2);

    if(RxHead2 >= RxBufferSize2)
    {
			RxHead2=0;
    }
		gRXTimer.enabled=true;  //for RX time out
		gRXTimer.counter=0;
		//Uart_IDIE_flag=false;
  }
	
	if(USART_GetFlagStatus(USARTUART2, USART_FLAG_ORE) != RESET)
	{
			USART_ClearFlag(USARTUART2, USART_FLAG_ORE);
			USART_ReceiveData(USARTUART2);
	}
  
  if(USART_GetITStatus(USARTUART2, USART_IT_TXE) != RESET)
  {   
		if(TxHead2==TxTail2){
			//USART_ClearITPendingBit(USARTUART2, USART_IT_TXE);
			/* Disable the USARTz Transmit interrupt */
			USART_ITConfig(USARTUART2, USART_IT_TXE, DISABLE);

		}else{
			USART_SendData(USARTUART2, TxBuffer_Z200[TxTail2++]);
			if (TxTail2 >=RxBufferSize2){
				TxTail2=0;
			}
			//while (USART_GetFlagStatus(USARTUART2, USART_FLAG_TC) == RESET);  //for wait tx finish
		}
  }
}

/**
  * @brief  This function handles USARTz global interrupt request.
  * @param  None
  * @retval None
  */
void RS485UART4_IRQHandler(void)   //UART3
{
  if((USART_GetFlagStatus(RS485UART4, USART_FLAG_ORE) != RESET)||USART_GetITStatus(RS485UART4, USART_IT_RXNE) != RESET)
  {
	  USART_ClearITPendingBit(RS485UART4, USART_IT_RXNE);
		USART_ClearITPendingBit(RS485UART4, USART_FLAG_ORE);
    /* Read one byte from the receive data register */
    RxBuffer_UART4[RxHead4++] = USART_ReceiveData(RS485UART4);

    if(RxHead4 >= RxBufferSize4)
    {
			RxHead4=0;
    }
		gRXTimer.enabled=true;  //for RX time out
		gRXTimer.counter=0;
  }

  
  if(USART_GetITStatus(RS485UART4, USART_IT_TXE) != RESET)
  {   
		if(TxHead4==TxTail4){
		  /* Disable the USARTz Transmit interrupt */	
			USART_ClearITPendingBit(RS485UART4,USART_IT_TXE);
			USART_ITConfig(RS485UART4, USART_IT_TXE, DISABLE);
			gTurn485ReceiveMode=1;

		}else{
			USART_SendData(RS485UART4, TxBuffer_UART4[TxTail4++]);
			if (TxTail4 >=RxBufferSize4){
				TxTail4=0;
			}
			//while (USART_GetFlagStatus(RS485UART4, USART_FLAG_TC) == RESET);  //for wait tx finish
		}
  }
}

void USARTMTK_to_UARTBLE_long( uint8_t *BufferPtr, uint32_t Length){
	uint8_t i;
	for(i=0; i<Length; i++){
		TxBuffer_BLE[TxHead3++] = *BufferPtr;
		if (TxHead3>=RxBufferSize3){
			TxHead3=0;
		}
		BufferPtr++;
	}

  USART_ITConfig(USARTBLE, USART_IT_TXE, ENABLE);
	/* Disable the USARTz Transmit interrupt */
	/*
  USART_ITConfig(USARTBLE, USART_IT_TXE, DISABLE);
	
	for(i=0; i<Length; i++){
		TxBuffer_BLE[TxHead3++] = *BufferPtr;
		if (TxHead3>=RxBufferSize3){
			TxHead3=0;
		}
		BufferPtr++;
	}
	if (USART_GetFlagStatus(USARTBLE, USART_FLAG_TXE) != RESET){  //UART3 TX Flag
	  USART_SendData(USARTBLE, TxBuffer_BLE[TxTail3++]);
		if (TxTail3 >=RxBufferSize3){
			TxTail3=0;
		}
	}
	while (USART_GetFlagStatus(USARTBLE, USART_FLAG_TC) == RESET);  //for wait tx finish

  USART_ITConfig(USARTBLE, USART_IT_TXE, ENABLE);
	*/
}

void UARTBLE_to_USARTMTK_long( uint8_t *BufferPtr, uint32_t Length){
	uint8_t i;
	
	for(i=0; i<Length; i++){
		TxBuffer_MTK[TxHead5++] = *BufferPtr;
		if (TxHead5>=RxBufferSize5){
			TxHead5=0;
		}
		BufferPtr++;
	}

  USART_ITConfig(USARTMTK, USART_IT_TXE, ENABLE);
	/* Disable the Transmit interrupt */
	/*
  USART_ITConfig(USARTMTK, USART_IT_TXE, DISABLE);
	
	for(i=0; i<Length; i++){
		TxBuffer_MTK[TxHead5++] = *BufferPtr;
		if (TxHead5>=RxBufferSize5){
			TxHead5=0;
		}
		BufferPtr++;
	}
	if (USART_GetFlagStatus(USARTMTK, USART_FLAG_TXE) != RESET){  //UART5 TX Flag
	  USART_SendData(USARTMTK, TxBuffer_MTK[TxTail5++]);
		if (TxTail5 >=RxBufferSize5){
			TxTail5=0;
		}
	}
	while (USART_GetFlagStatus(USARTMTK, USART_FLAG_TC) == RESET);  //for wait tx finish

  USART_ITConfig(USARTMTK, USART_IT_TXE, ENABLE);
	*/
}


void USARTMTK_to_UART2_long( uint8_t *BufferPtr, uint32_t Length){
	uint8_t i;
	
	for(i=0; i<Length; i++){
		TxBuffer_Z200[TxHead2++] = *BufferPtr;
		if (TxHead2>=RxBufferSize2){
			TxHead2=0;
		}
		BufferPtr++;
	}

  USART_ITConfig(USARTUART2, USART_IT_TXE, ENABLE);
/*
  USART_ITConfig(USARTUART2, USART_IT_TXE, DISABLE);
	
	for(i=0; i<Length; i++){
		TxBuffer_UART2[TxHead2++] = *BufferPtr;
		if (TxHead2>=RxBufferSize2){
			TxHead2=0;
		}
		BufferPtr++;
	}
	if (USART_GetFlagStatus(USARTUART2, USART_FLAG_TXE) != RESET){  //UART5 TX Flag
	  USART_SendData(USARTUART2, TxBuffer_UART2[TxTail2++]);
		if (TxTail2 >=RxBufferSize2){
			TxTail2=0;
		}
	}
	while (USART_GetFlagStatus(USARTUART2, USART_FLAG_TC) == RESET);  //for wait tx finish

  USART_ITConfig(USARTUART2, USART_IT_TXE, ENABLE);
	*/
}

void UART2_to_USARTMTK_long( uint8_t *BufferPtr, uint32_t Length){
	uint8_t i;
	
	for(i=0; i<Length; i++){
		TxBuffer_MTK[TxHead5++] = *BufferPtr;
		if (TxHead5>=RxBufferSize5){
			TxHead5=0;
		}
		BufferPtr++;
	}

  USART_ITConfig(USARTMTK, USART_IT_TXE, ENABLE);
	/* Disable the Transmit interrupt */
	/*
  USART_ITConfig(USARTMTK, USART_IT_TXE, DISABLE);
	
	for(i=0; i<Length; i++){
		TxBuffer_MTK[TxHead5++] = *BufferPtr;
		if (TxHead5>=RxBufferSize5){
			TxHead5=0;
		}
		BufferPtr++;
	}
	if (USART_GetFlagStatus(USARTMTK, USART_FLAG_TXE) != RESET){  //UART5 TX Flag
	  USART_SendData(USARTMTK, TxBuffer_MTK[TxTail5++]);
		if (TxTail5 >=RxBufferSize5){
			TxTail5=0;
		}
	}
	while (USART_GetFlagStatus(USARTMTK, USART_FLAG_TC) == RESET);  //for wait tx finish
  USART_ITConfig(USARTMTK, USART_IT_TXE, ENABLE);
	*/
}


void UART4_RS485_long( uint8_t *BufferPtr, uint32_t Length){
	uint8_t i;
	
	Gpio_On(GPIODE);  //Hi DE TX
	Gpio_On(GPIORE);  //Hi RE TX
	
	for(i=0; i<Length; i++){
		TxBuffer_UART4[TxHead4++] = *BufferPtr;
		if (TxHead4>=RxBufferSize4){
			TxHead4=0;
		}
		BufferPtr++;
	}

  USART_ITConfig(RS485UART4, USART_IT_TXE, ENABLE);
	/* Disable the Transmit interrupt */
	/*
  USART_ITConfig(RS485UART4, USART_IT_TXE, DISABLE);
	Gpio_On(GPIODE);  //Hi DE TX
	Gpio_On(GPIORE);  //Hi RE TX
	
	for(i=0; i<Length; i++){
		TxBuffer_UART4[TxHead4++] = *BufferPtr;
		if (TxHead4>=RxBufferSize4){
			TxHead4=0;
		}
		BufferPtr++;
	}
	if (USART_GetFlagStatus(RS485UART4, USART_FLAG_TXE) != RESET){  //UART5 TX Flag
	  USART_SendData(RS485UART4, TxBuffer_UART4[TxTail4++]);
		if (TxTail4 >=RxBufferSize4){
			TxTail4=0;
		}
	}
	while (USART_GetFlagStatus(RS485UART4, USART_FLAG_TC) == RESET);  //for wait tx finish

  USART_ITConfig(RS485UART4, USART_IT_TXE, ENABLE);
	*/
}