#define DE_PIN                         GPIO_Pin_12
#define DE_GPIO_PORT                   GPIOA
#define DE_GPIO_CLK                    RCC_APB2Periph_GPIOA  


#define RE_PIN                         GPIO_Pin_11
#define RE_GPIO_PORT                   GPIOA
#define RE_GPIO_CLK                    RCC_APB2Periph_GPIOA  


typedef enum 
{
  GPIODE = 0,
  GPIORE = 1,
} QGPIO_TypeDef;




void Gpio_init(QGPIO_TypeDef Gpio);
void Gpio_On(QGPIO_TypeDef Gpio);
void Gpio_Off(QGPIO_TypeDef Gpio);

