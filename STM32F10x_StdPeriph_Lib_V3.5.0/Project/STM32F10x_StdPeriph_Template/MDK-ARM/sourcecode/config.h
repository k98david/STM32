/**
  ******************************************************************************
  * @file    USART/Interrupt/platform_config.h 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Evaluation board specific configuration file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the line corresponding to the STMicroelectronics evaluation board
   used to run the example */

  #define USARTMTK                   UART5
  //#define USARTMTK_GPIO              GPIOA
  #define USARTMTK_CLK               RCC_APB1Periph_UART5
  #define USARTMTK_GPIO_CLK          RCC_APB2Periph_GPIOD
  #define USARTMTK_RxPin             GPIO_Pin_2
  #define USARTMTK_TxPin             GPIO_Pin_12
  #define USARTMTK_IRQn              UART5_IRQn
  #define USARTMTK_IRQHandler        UART5_IRQHandler
  
  #define USARTBLE                   USART3
  #define USARTBLE_GPIO              GPIOB
  #define USARTBLE_CLK               RCC_APB1Periph_USART3
  #define USARTBLE_GPIO_CLK          RCC_APB2Periph_GPIOB
  #define USARTBLE_RxPin             GPIO_Pin_11
  #define USARTBLE_TxPin             GPIO_Pin_10
  #define USARTBLE_IRQn              USART3_IRQn
  #define USARTBLE_IRQHandler        USART3_IRQHandler


  #define USARTUART2                   USART2
  //#define USARTMTK_GPIO              GPIOA
  #define USARTUART2_CLK               RCC_APB1Periph_USART2
  #define USARTUART2_GPIO_CLK          RCC_APB2Periph_GPIOA
  #define USARTUART2_RxPin             GPIO_Pin_3
  #define USARTUART2_TxPin             GPIO_Pin_2
  #define USARTUART2_IRQn              USART2_IRQn
  #define USARTUART2_IRQHandler        USART2_IRQHandler
  

  #define USARTUART1                   USART1
  #define USART1_GPIO                  GPIOA
  #define USARTUART1_CLK               RCC_APB2Periph_USART1
  #define USARTUART1_GPIO_CLK          RCC_APB2Periph_GPIOA
  #define USARTUART1_RxPin             GPIO_Pin_10
  #define USARTUART1_TxPin             GPIO_Pin_9
  #define USARTUART1_IRQn              USART1_IRQn
  #define USARTUART1_IRQHandler        USART1_IRQHandler
  

  #define RS485UART4                   UART4
  #define RS485UART4_GPIO              GPIOC
  #define RS485UART4_CLK               RCC_APB1Periph_UART4
  #define RS485UART4_GPIO_CLK          RCC_APB2Periph_GPIOC
  #define RS485UART4_RxPin             GPIO_Pin_11
  #define RS485UART4_TxPin             GPIO_Pin_10
  #define RS485UART4_IRQn              UART4_IRQn
  #define RS485UART4_IRQHandler        UART4_IRQHandler
  
	
	
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
