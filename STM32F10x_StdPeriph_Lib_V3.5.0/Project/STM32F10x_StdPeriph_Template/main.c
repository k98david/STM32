/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32_eval.h"
#include "config.h"
#include <stdio.h>
#include "QUart.h"
#include "QuanProcessCMD.h"
#include "QKey.h"
#include "QTimer.h"
#include "Qevent_handler.h"  
#include "QGPIO.h"
#include "stm32f10x_iwdg.h"


#ifdef USE_STM32100B_EVAL
 #include "stm32100b_eval_lcd.h"
#elif defined USE_STM3210B_EVAL
 #include "stm3210b_eval_lcd.h"
#elif defined USE_STM3210E_EVAL
 #include "stm3210e_eval_lcd.h" 
#elif defined USE_STM3210C_EVAL
 #include "stm3210c_eval_lcd.h"
#elif defined USE_STM32100E_EVAL
 #include "stm32100e_eval_lcd.h"
#endif

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifdef USE_STM32100B_EVAL
  #define MESSAGE1   "STM32 MD Value Line " 
  #define MESSAGE2   " Device running on  " 
  #define MESSAGE3   "  STM32100B-EVAL    " 
#elif defined (USE_STM3210B_EVAL)
  #define MESSAGE1   "STM32 Medium Density" 
  #define MESSAGE2   " Device running on  " 
  #define MESSAGE3   "   STM3210B-EVAL    " 
#elif defined (STM32F10X_XL) && defined (USE_STM3210E_EVAL)
  #define MESSAGE1   "  STM32 XL Density  " 
  #define MESSAGE2   " Device running on  " 
  #define MESSAGE3   "   STM3210E-EVAL    "
#elif defined (USE_STM3210E_EVAL)
  #define MESSAGE1   " STM32 High Density " 
  #define MESSAGE2   " Device running on  " 
  #define MESSAGE3   "   STM3210E-EVAL    " 
#elif defined (USE_STM3210C_EVAL)
  #define MESSAGE1   " STM32 Connectivity " 
  #define MESSAGE2   " Line Device running" 
  #define MESSAGE3   " on STM3210C-EVAL   "
#elif defined (USE_STM32100E_EVAL)
  #define MESSAGE1   "STM32 HD Value Line " 
  #define MESSAGE2   " Device running on  " 
  #define MESSAGE3   "  STM32100E-EVAL    "   
#endif
#include <stm32f10x.h>

uint8_t LED_Test_flag=0;
uint8_t TestTemp[]={"ABCDEFGH"};
uint8_t i=0;
//test key
bool Key_press=false;

//test Buzzer
bool Buzzer_Flag=true;

//for Tick timer 
QTimer_t gKeyTimer;
QTimer_t gLEDflashTimer;
QTimer_t gBuzzerTimer;
QTimer_t gRXTimer;

uint8_t x=0;
void Tick_Timer(void);
/* Private macro -------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
uint16_t CCR1_Val = 333;
uint16_t CCR2_Val = 249;
uint16_t CCR3_Val = 166;
uint16_t CCR4_Val = 83;
uint16_t PrescalerValue = 0;

GPIO_InitTypeDef GPIO_InitStructure;

void GPIO_Configuration(void);
void NVIC_Configuration(void);
RCC_ClocksTypeDef  RCC_Clocks;  
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /* System Clocks Configuration */
  RCC_Configuration();
  RCC_GetClocksFreq(&RCC_Clocks);
  /* NVIC configuration */
  NVIC_Configuration();
  /* Configure the GPIO ports */
  GPIO_Configuration();
	
	SCB->CCR |= 0x10;
	
  STM_EVAL_LEDInit(LEDR);
  STM_EVAL_LEDInit(LEDG);
	
	Gpio_init(GPIODE);
	Gpio_init(GPIORE);
  Gpio_Off(GPIODE);  //low DE Receive
	Gpio_Off(GPIORE);  //low RE Receive
  STM_EVAL_LEDOff(LEDR);
  STM_EVAL_LEDOff(LEDG);
	
	key_init();
	
/* USARTy and USARTz configuration ------------------------------------------------------*/
  /* USARTy and USARTz configured as follow:
        - BaudRate = 9600 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
	//USART_InitStructure.USART_BaudRate = 146600;
	USART_InitStructure.USART_BaudRate = 115200*2;
  //USART_InitStructure.USART_BaudRate = 12188;  //9600 24M
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  /* Configure USARTy */
  USART_Init(USARTMTK, &USART_InitStructure);
  /* Configure USARTz */
  USART_Init(USARTBLE, &USART_InitStructure);
  /* Configure USARTz */
  USART_Init(USARTUART2, &USART_InitStructure);
  /* Configure USART1 */
  USART_Init(USARTUART1, &USART_InitStructure);
  /* Configure USART4 */
  USART_Init(RS485UART4, &USART_InitStructure);
	
	
  /* Enable USARTy Receive and Transmit interrupts */
  USART_ITConfig(USARTMTK, USART_IT_RXNE, ENABLE);

  /* Enable USARTz Receive and Transmit interrupts */
  USART_ITConfig(USARTBLE, USART_IT_RXNE, ENABLE);
	
  /*Enable USARTz Receive and Transmit interrupts */
  USART_ITConfig(USARTUART2, USART_IT_RXNE, ENABLE);
	
  /*Enable USART1 Receive and Transmit interrupts */
  USART_ITConfig(USARTUART1, USART_IT_RXNE, ENABLE);
	
  /*Enable USART4 Receive and Transmit interrupts */
  USART_ITConfig(RS485UART4, USART_IT_RXNE, ENABLE);

  /* Enable the USARTy */
  USART_Cmd(USARTMTK, ENABLE);
  /* Enable the USARTz */
  USART_Cmd(USARTBLE, ENABLE);
  /* Enable the USARTz */
  USART_Cmd(USARTUART2, ENABLE);
	
  /* Enable the USART1 */
  USART_Cmd(USARTUART1, ENABLE);
	
  /* Enable the USART4 */
  USART_Cmd(RS485UART4, ENABLE);
	
	initCommands();  //initCMD
	
	
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);  //   WWDG???? 
	 
   WWDG_SetPrescaler(3);//??????? 

   WWDG_SetWindowValue(0x7F);//?????? 
  
   WWDG_Enable(0x7E);     //?????????????? 
 
  if (SysTick_Config(SystemCoreClock / 10))  //100 ms 
  { 
    while (1);
  }
	
  /* Time base configuration */
	/*
  /* -----------------------------------------------------------------------
    TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles:
    The TIM3CLK frequency is set to SystemCoreClock (Hz), to get TIM3 counter
    clock at 24 MHz the Prescaler is computed as following:
     - Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
    and Connectivity line devices and to 24 MHz for Low-Density Value line and
    Medium-Density Value line devices

    The TIM3 is running at 36 KHz: TIM3 Frequency = TIM3 counter clock/(ARR + 1)
                                                  = 24 MHz / 666 = 36 KHz
    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
    TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
    TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%
  ----------------------------------------------------------------------- */

	/* TIMER1 PWM */
	/*
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE);	
	
  TIM_TimeBaseStructure.TIM_Period = 48000;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 24000;
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
*/
  //TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* TIM3 enable counter */
  //TIM_Cmd(TIM1, ENABLE);
  /* Main Output Enable */
  //TIM_CtrlPWMOutputs(TIM1, ENABLE);
	

	 //UARTBLE_to_USARTMTK_long(TestTemp,8);
  /* Infinite loop */
  while (1)
  {
		  WWDG_SetCounter(0x7E); //??,?????? 
								    Rx3ProcessCMD();   //BLE
							     	Rx5ProcessCMD();   //MTK
					    	    Rx2ProcessCMD();   //Z200
	    //UART_RxData();
			//Rx3ProcessCMD();
			handleEvent();		
  }
}

void Tick_Timer(void)
{
					    //STM_EVAL_LEDToggle(LEDG);
	Gpio_On(GPIODE);  //Hi DE TX
	//STM_EVAL_LEDOn(LEDR);  //DEbug
	if (gKeyTimer.enabled){  //for debounce KEY 
		if (gKeyTimer.counter>0){
			gKeyTimer.counter--;
		}else{
				gKeyTimer.enabled=false;
			  if(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4)){
					Key_press=true;
					sendEvent(evLEDFlash);    //flashing LED event 
					
					
					sendEvent(evPairKey);     //evPairKey
				}
		}
	}
	/*
	if(x==3){
		x=0;
			 UART4_RS485_long(TestTemp,8);
	}
	x++;
	*/
	if (gLEDflashTimer.enabled){  //for LED flashing  
		if(gLEDflashTimer.counter>0){
			  if((gLEDflashTimer.counter%5)==0){
						STM_EVAL_LEDToggle(LEDR);
				    STM_EVAL_LEDToggle(LEDG);
				}
		    gLEDflashTimer.counter--;
		}else{
				gLEDflashTimer.enabled=false;
				STM_EVAL_LEDOff(LEDR);
				STM_EVAL_LEDOff(LEDG);
		}
	}
	
	if (gBuzzerTimer.enabled){  //for LED flashing 
		if(gBuzzerTimer.counter>0){
		    gBuzzerTimer.counter--;
			  if(Buzzer_Flag){    //for Toggle buzzer 
					TIM_Cmd(TIM1, ENABLE);   //open Buzzer
					TIM_CtrlPWMOutputs(TIM1, ENABLE);
					Buzzer_Flag=false;
				}else{
					Buzzer_Flag=true;
					TIM_Cmd(TIM1, DISABLE);   //colse Buzzer
					TIM_CtrlPWMOutputs(TIM1, DISABLE);
				}
		}else{
				gBuzzerTimer.enabled=false;
		}
	}
		
	if (gRXTimer.enabled){  //for RX time out  
		if (gRXTimer.counter>0){
			gRXTimer.counter--;
		}else{
			gRXTimer.enabled=false;
      SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;
			STM_EVAL_LEDOn(LEDR);  //DEbug
			/*
			    if(UAART_RX_Port==UARTRX_BLE){
						//STM_EVAL_LEDOn(LEDG);  //DEbug
					    Rx3ProcessCMD();   //BLE
					}else if(UAART_RX_Port==UARTRX_MTK){	
					  	//STM_EVAL_LEDOn(LEDG);  //DEbug
				     	Rx5ProcessCMD();   //MTK
					}else if(UAART_RX_Port==UARTRX_Zigbee){
		    	    Rx2ProcessCMD();   //Z200
					}
*/
			STM_EVAL_LEDOff(LEDR);  //DEbug
		//	sendEvent(evRXprocess);     //evRXprocess event 
		}
	}
	
	if (gTurn485ReceiveMode){
		if (USART_GetFlagStatus(RS485UART4, USART_FLAG_TC)){
			Gpio_Off(GPIODE);  //low DE Receive
			Gpio_Off(GPIORE);  //low RE Receive
			gTurn485ReceiveMode = 0;
		}
	}

	/*
	if(Key_press){
		if(LED_Test_flag){
				LED_Test_flag=0;
				STM_EVAL_LEDOn(LEDR);
				STM_EVAL_LEDOn(LEDG);
			//  TIM_Cmd(TIM1, ENABLE);   //open Buzzer
		}else{
				LED_Test_flag=1;
				STM_EVAL_LEDOff(LEDR);
				STM_EVAL_LEDOff(LEDG);
			 // TIM_Cmd(TIM1, DISABLE);   //colse Buzzer
		}
	}
	*/
//STM_EVAL_LEDOff(LEDR);  //DEbug
	Gpio_Off(GPIODE);  //Hi DE TX
	  if (SysTick_Config(SystemCoreClock / 10))  //100 ms 
  { 
    while (1);
  }
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(EVAL_COM1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{  
	/* PWM TIMER1 */
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	/* UART BLE UART 1*/
  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd( USARTBLE_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE); 
	/* Enable USARTz Clock */
  RCC_APB1PeriphClockCmd(USARTBLE_CLK, ENABLE);  
	
	/* UART MTK UART 5*/
  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE); 
  /* Enable USARTy Clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); 

	/* UART z200 UART 2*/
  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE); 
  /* Enable USARTy Clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 
	
	/* UART4 RS485*/
  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd( RS485UART4_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE); 
  /* Enable USARTy Clock */
  RCC_APB1PeriphClockCmd(RS485UART4_CLK, ENABLE); 

}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure USARTMTK UART5 Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = USARTMTK_RxPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  /* Configure USARTBLE3 Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = USARTBLE_RxPin;
  GPIO_Init(USARTBLE_GPIO, &GPIO_InitStructure); 
	
  /* Configure USART2 Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = USARTUART2_RxPin;
  GPIO_Init(USARTBLE_GPIO, &GPIO_InitStructure); 
	
  /* Configure USARTMTK5 Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = USARTMTK_TxPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure USARTBLE3 Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = USARTBLE_TxPin;
  GPIO_Init(USARTBLE_GPIO, &GPIO_InitStructure); 

  /* Configure USARTZ200 2 Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = USARTUART2_TxPin;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	/* Configure UART4 Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = RS485UART4_TxPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	/* Configure UART4 Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = RS485UART4_RxPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);


}


/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  
  /* Enable the USARTy Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USARTMTK_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable the USARTz Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USARTBLE_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
  /* Enable the USART2 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USARTUART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	/* Enable the USART2 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = RS485UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
