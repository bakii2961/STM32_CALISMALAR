
#include "main.h"


typedef enum CHANNELS{
	CHANNEL1,
	CHANNEL2,
	CHANNEL3,
	CHANNEL4,
}Channels_e;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

int main(void)
{
  
  HAL_Init();

  SystemClock_Config();


  MX_GPIO_Init();
  
  void pwm_pin_configure()
    {
    	GPIO_InitTypeDef GPIO_InitStruct;

    	RCC->APB1ENR|=(1<<1);
    	RCC->AHB1ENR |= (1<<1);

    	// PB4 - TIMER3 - CH1 - D5
    	GPIO_InitStruct.Pin = GPIO_PIN_4;
    	//GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    	GPIOB->MODER &= ~(1 << 8);
    	GPIOB->MODER |= (1 << 9);
    	GPIOB->OTYPER &= ~(1<<4);
    	GPIOB->OSPEEDR |= (8<<1);
    	GPIOB->OSPEEDR |= (9<<1);
    	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    	//GPIOB->AFR[0] |= (2 << (4 * 4));    // Set AF2 (TIM3) for pin 4
    	 //GPIOB->AFR[0] |= (2 << (4 * 4));
    	//GPIOB->AFR[0] &= ~(1<<16);
    	//GPIOB->AFR[0] |= (1<<17);
    	//GPIOB->AFR[0] &= ~(1<<18);
    	//GPIOB->AFR[0] &= ~(1<<19);


    	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    	// PB5 - TIMER3 - CH2 - D4
    	GPIO_InitStruct.Pin = GPIO_PIN_5;
    	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    	// PB0 - TIMER3 - CH3 - A3
    	GPIO_InitStruct.Pin = GPIO_PIN_0;
    	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    	// PB1 - TIMER3 - CH4
    	GPIO_InitStruct.Pin = GPIO_PIN_1;
    	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
  void pwm_init(void)
  {
  	pwm_pin_configure();

  	/*TIM3->PSC = 83; // Timer clock = 84 mhz / 42 = 2Mhz
  	TIM3->ARR = 99;// Period ==> (2 Mhz / 100) = 20 Khz


  	TIM3->CCR1 = 0;//TIM3->CCR1 = 0; ifadesi, TIM3 periferinin 1. kanalının CCR1 (Capture/Compare Register 1) registerına 0 değerini atar. Bu işlem, belirli bir zamanlama veya PWM (Pulse Width Modulation) uygulamasında kullanılabilecek bir adımdır.
  	TIM3->CCR2 = 0;
  	TIM3->CCR3 = 0;
  	TIM3->CCR4 = 0;
   */ TIM3->PSC = 47; // Timer clock = 84 mhz / 42 = 2Mhz
  	TIM3->ARR = 19999;// Period ==> (2 Mhz / 100) = 20 Khz


  	TIM3->CCR1 = 10000;//TIM3->CCR1 = 0; ifadesi, TIM3 periferinin 1. kanalının CCR1 (Capture/Compare Register 1) registerına 0 değerini atar. Bu işlem, belirli bir zamanlama veya PWM (Pulse Width Modulation) uygulamasında kullanılabilecek bir adımdır.
  	TIM3->CCR2 = 10000;
  	TIM3->CCR3 = 10000;
  	TIM3->CCR4 = 10000;


  	// CH-1 PWM MODE
  	// Genellikle TIM periferinin 1. ve 2. kanallarının yakalama(CAPTURE) ve karşılaştırma(COMPARE) modları için kullanılır
  	//TIM3->CCMR1 |= TIM_CCMR1_OC1M_2;
  	TIM3->CCMR1|= (1<<6);               //OC1M[2:0] REGİSTERİ NİN PWM(110) SİNİ  SEÇTİK
  	TIM3->CCMR1|= (1<<5);
  	TIM3->CCMR1&= ~(1<<4);

  	//TIM3->CCMR1 |= TIM_CCMR1_OC1M_1;
  	TIM3->CCMR1 &= ~(1<<3);

  	// CH-2 PWM MODE
  	TIM3->CCMR1 |= TIM_CCMR1_OC2M_2;
  	TIM3->CCMR1 |= TIM_CCMR1_OC2M_1;
  	TIM3->CCMR1 |= TIM_CCMR1_OC2PE;

  	// CH-3 PWM MODE
  	TIM3->CCMR2 |= TIM_CCMR2_OC3M_2;
  	TIM3->CCMR2 |= TIM_CCMR2_OC3M_1;
  	TIM3->CCMR2 |= TIM_CCMR2_OC3PE;

  	// CH-4 PWM MODE
  	TIM3->CCMR2 |= TIM_CCMR2_OC4M_2;
  	TIM3->CCMR2 |= TIM_CCMR2_OC4M_1;
  	TIM3->CCMR2 |= TIM_CCMR2_OC4PE;

  	// Enable OC1REF and OC2REF OUTPUTS
  	TIM3->CCER |= TIM_CCER_CC1E;
  	TIM3->CCER |= TIM_CCER_CC2E;
  	TIM3->CCER |= TIM_CCER_CC3E;
  	TIM3->CCER |= TIM_CCER_CC4E;
  }
  void pwm_enable(void)
  {
  	// Enable Timer
  	TIM3->CR1 |= TIM_CR1_CEN;
  	TIM3->EGR |= TIM_EGR_UG;
  }
  void pwm_set_duty_cycle(uint32_t duty,uint32_t baki,uint32_t erkam,uint32_t bayezit, Channels_e channel)
  {
  	switch (channel) {
  	case CHANNEL1:
  		TIM3->CCR1 = duty;
  		break;
  	case CHANNEL2:
  		TIM3->CCR2 = baki;
  		break;
  	case CHANNEL3:
  		TIM3->CCR3 = erkam;
  		break;

  	case CHANNEL4:
  		TIM3->CCR4 = bayezit;
  		break;
  	}
  }


pwm_init();
 
  while (1)
  {
    /* USER CODE END WHILE */
	  pwm_enable();
	  void servo_Sweep1(){
			   pwm_set_duty_cycle(2400, 0, 0, 0, CHANNEL1);
			   pwm_set_duty_cycle(0, 2400, 0, 0, CHANNEL2);
			   HAL_Delay(1000);
		  }

	  void servo_Sweep2(){
	  		   pwm_set_duty_cycle(500, 0, 0, 0, CHANNEL1);
	  		   pwm_set_duty_cycle(0, 500, 0, 0, CHANNEL2);
	  		   HAL_Delay(1000);
	  	  }
	  servo_Sweep1();
	  //servo_Sweep2();

  }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

 
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
