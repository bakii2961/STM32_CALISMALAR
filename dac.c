
#include "main.h"


int i =0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

void DACC_Config(){
RCC->AHB1ENR |=(1<<0);
RCC->APB1ENR |= (1<<29);
DAC->CR|=(1<<0); //CHANNEL 1 AKTİV ETTİK
DAC->SWTRIGR &=  ~(1<<0); // yazılımsal tetikleme kapattık
DAC->DHR12R1 |= 0x00000000;// 12 bitin sağdan başlayarak başlangıç bitlerini 0 ladık  yani çıkış pinine 0 yazfık ilk başlangıç voltajı 0 oldu


}
void delay(uint32_t deger)
{

	while(deger--);
}



int main(void)
{
  HAL_Init();

 
  SystemClock_Config();

  MX_GPIO_Init();

DACC_Config();

  while (1)
  {
 
	  for(i=0;i<4096;i++){

		 DAC->DHR12R1 =i;
		 HAL_Delay(1);
	  }
	  i =0;

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

  /* GPIO Ports Clock Enable */
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
