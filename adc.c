#include "main.h"

uint8_t adc_value;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

void gpıo_congig(){
	RCC->AHB1ENR|=(1<<0);
	GPIOA->MODER|=(1<<1);
	GPIOA->MODER|=(1<<0);
	GPIOA->OSPEEDR|=(1<<0);
	GPIOA->OSPEEDR|=(1<<1);
}

void adc_config(){
	RCC->APB2ENR|=(1<<8);

	ADC1->CR1&=~(1<<24); // 24. ve 25. bitleri 10 yaptık resolution (çözünürlüğü 8 bityaptık )
	ADC1->CR1&=~(1<<25);

	ADC1->CR2|= (1<<0); // adon bitini adc yi aktif eden bbit 1 yazdık

	ADC1->SMPR2 &= ~(1<<2);
	ADC1->SMPR2|=(1<<1);   //56 cycle ile calışacağız dedik
	ADC1->SMPR2|=(1<<0);

	ADC->CCR &= ~(1<<17);
	ADC->CCR |=(1<<16);
}

uint8_t read_adc(){
	uint8_t value;
	ADC1->CR2 |= (1<<30); //

	while(!(ADC1->SR & 0x00000002));
	value = ADC1->DR;

	return value;
}

int main(void)
{
  
  HAL_Init();

  SystemClock_Config();


  MX_GPIO_Init();
 

  gpıo_congig();
  adc_config();

  while (1)
  {
	  adc_value=read_adc();
  }

}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

 
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }


  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_GPIO_Init(void)
{

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}


void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {
  }

}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif /* USE_FULL_ASSERT */
