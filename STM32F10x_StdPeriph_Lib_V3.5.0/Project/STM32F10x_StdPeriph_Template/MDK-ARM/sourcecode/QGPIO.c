#include <stm32f10x.h>
#include "QGPIO.h"

void Gpio_init(QGPIO_TypeDef Gpio);
void Gpio_On(QGPIO_TypeDef Gpio);
void Gpio_Off(QGPIO_TypeDef Gpio);


#define GPIOn                             2

const uint32_t GPIO_DERE_CLK[GPIOn] = {DE_GPIO_CLK, RE_GPIO_CLK};
const uint16_t GPIO_DERE_PIN[GPIOn] = {DE_PIN, RE_PIN};
GPIO_TypeDef* GPIO_DERE_PORT[GPIOn] = {DE_GPIO_PORT, RE_GPIO_PORT};

void Gpio_init(QGPIO_TypeDef Gpio)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
  RCC_APB2PeriphClockCmd(GPIO_DERE_CLK[Gpio], ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_DERE_PIN[Gpio];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIO_DERE_PORT[Gpio], &GPIO_InitStructure);
}


void Gpio_On(QGPIO_TypeDef Gpio)
{
  GPIO_DERE_PORT[Gpio]->BSRR = GPIO_DERE_PIN[Gpio];        
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4 
  * @retval None
  */
void Gpio_Off(QGPIO_TypeDef Gpio)
{
  GPIO_DERE_PORT[Gpio]->BRR = GPIO_DERE_PIN[Gpio];    
}
