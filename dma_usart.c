/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "usart-dma.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Uart2config(void)
{
	/*
	 * 1Enable the USART by writing the UE bit in USART_CR1 register to 1.
	2.
	Program the M bit in USART_CR1 to define the word length.
	3.
	Program the number of stop bits in USART_CR2.
	4.
	Select DMA enable (DMAT) in USART_CR3 if Multi buffer Communication is to take
	place. Configure the DMA register as explained in multibuffer communication.
	5.
	Select the desired baud rate using the USART_BRR register.
	6.
	Set the TE bit in USART_CR1 to send an idle frame as first transmission.
	7.
	Write the data to send in the USART_DR register (this clears the TXE bit). Repeat this
	for each data to be transmitted in case of single buffer.
	8.
	After writing the last data into the USART_DR register, wait until TC=1. This indicates
	that the transmission of the last frame is complete. This is required for instance when
	the USART is disabled or enters the Halt mode to avoid corrupting the last
	transmission.*/

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->APB1ENR |= (1<<17);// UART2 AKTİF ETTİK
	//RCC->AHB1ENR  |= (1<<0); // A PORTUNU RCC SATİNİ AÇTIK
	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	//GPIOA->MODER &= ~(1<<4); // PİNİ ALTERNATİF FONKSYON OLARAK AYARLADIK  PA1 4.BİT 0 5.BİT 1
	GPIOA->MODER |= (2<<4); // PİNİ ALTERNATİF FONKSYON OLARAK AYARLADIK  PA1 4.BİT 0 5.BİT 1
	GPIOA->MODER |= (2<<6); // BUDA İKİSİNİ BİRDEN 6. VE 7. BİTLERİ 7. 1 6. BİTİ 0 YAPTI TEK SATIRDA  YANİ 2 KOYMAK (1:0) DEMEK OLUYOR

	//GPIOA->OSPEEDR |= (3<<4)| (3<<6); // 3 KOYARAK (1:1) VERDİK YANİ PA2 VE PA3 PİNLERİ 1:1 DE YÜKSEK HIZDA  OLSUN DEDİK

	GPIOA->AFR[0] |= (7<<8); //(11:10:9:8) = 0 1 1 1 YAPTIK BURDA  ALTERNATİF FONKSYON AYARI PA2 AFR[0] DÜŞÜK HIZI BELİRLER
	GPIOA->AFR[0] |= (7<<12); //(15:14:13:12)= 0 1 1 1 YAPTIK BUNUDA  ALTERNATİF FONKSYON AYARI PA3 AFR[0] DÜŞÜK HIZI BELİRLER

	USART2->CR1 |= 0X00;// temizledik önce
	USART2->CR1 |= (1<<13); // UARTI ETKİNLEŞTİRMEK İÇİN CONTROL REGİSTEREİN 13. BİTİNE 1 YAZDIK bu böyle

	//USART2->CR3 |= (1<<7);
	USART2->CR3 |= (1<<6); // USARTI DMA İLE KULLANCAĞIMIZ İÇİN RECİV VE TRANSMİT REGİSTERLERİNİ AÇTIK 6 VE 7 . BİTLER

	USART2->CR1 &= ~(1<<12); // 8 bit kullanacaağımız için control registerini 12. bitini 0 yaptık 8 bit veri atalım diye
	USART2->BRR = (13<<0) | (22<<4);// baud rate ayarı hesaplamalardan sonra 152000 için deger 6 virgül çıktı 7 ye yuvarladık onu da 0. bite yazdık

	USART2->CR1 |= (1<<2);
	USART2->CR1 |= (1<<3);

}

void DMA_init (void){
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	//DMA1_Stream5->CR |= (1<<0) ;
	// EN TCIE HTIE TEIE /*
	//DMA1_Stream5->CR |= (1<<0) ;
	DMA1_Stream5->CR |= (1<<4) ;
	DMA1_Stream5->CR |=	(1<<3) ;
	DMA1_Stream5->CR |=	(1<<2) ;
/*EN (Channel Enable): DMA kanalını etkinleştirir veya devre dışı bırakır. Bu bit 1 olarak ayarlandığında, DMA transferi gerçekleştirilir.
TCIE (Transfer Complete Interrupt Enable): Bu bit, DMA transferinin tamamlanması durumunda bir kesmeyi (interrupt) etkinleştirir veya devre dışı bırakır. DMA transferi tamamlandığında, bu kesme tetiklenir.
HTIE (Half Transfer Interrupt Enable): Bu bit, DMA tampon yarısının dolduğu durumda bir kesmeyi etkinleştirir veya devre dışı bırakır. Tamponun yarısı dolduğunda, bu kesme tetiklenir.
TEIE (Transfer Error Interrupt Enable): DMA transfer hatası durumunda bir kesmeyi etkinleştirir veya devre dışı bırakır. DMA transferi sırasında bir hata oluştuğunda, bu kesme tetiklenir.
	 */
	DMA1_Stream5->CR &= ~(1<<6);
	DMA1_Stream5->CR &=	~(1<<7) ; // DIR VERİ YOLUDUR BUNU 0 0 YAPTIK Kİ ÇEVREDEN HAFIZAYA VERİ ALABİLEİLM DİYE
	DMA1_Stream5->CR |= (1<<8); //CIRC modu genellikle veri akışının kesintisiz olması gereken durumlar için kullanılır. Örneğin, sürekli veri akışının olduğu bir sensör uygulaması veya veri
	//kaynağının hiç durmadığı bir veri iletimi senaryosu için CIRC modu oldukça yararlı olabilir.
	DMA1_Stream5->CR |= (1<<10); // MINC AKTİF ETTİK Eğer MINC biti ayarlanırsa (1 olarak ayarlanırsa), her transfer sonrasında hedef bellek adresi otomatik olarak artırılır.
	//Bu, genellikle bellek bölgesindeki ardışık verileri DMA aracılığıyla transfer etmek istediğinizde kullanışlıdır
	DMA1_Stream5->CR &= ~(1<<11);
	DMA1_Stream5->CR &=	~(1<<12); //PSİZE  , DMA transferinin periferaller arasındaki veri boyutunu belirler.
	DMA1_Stream5->CR &= ~(1<<13);
	DMA1_Stream5->CR &=	~(1<<14); //MSIZE (Memory Size) bitleri, DMA transferinin bellek üzerindeki veri boyutunu belirler.
	DMA1_Stream5->CR &= ~(1<<16);
	DMA1_Stream5->CR &=	~(1<<17); //, DMA Stream'lerin öncelik seviyesini belirlemek için kullanılan bir konsepttir. DMA'nın birden çok Stream'i (kanalı) olabilir ve bu Stream'ler aynı anda
	//çalışabilir. Ancak, bu Stream'lerin öncelik düzeylerini ayarlamak, hangi Stream'in öncelikli olarak çalışacağını belirlemek için önemlidir.


}

void Dma_Config (uint32_t srcAdd, uint32_t destAdd,uint16_t datasize){

	DMA1_Stream5->NDTR = datasize;//transferi sırasında kaç adet veri transferi gerçekleştirileceğini belirten bir kayıttır. Bu register, DMA transferinin ne kadar süreceğini ve
		//hangi veri miktarının taşınacağını belirlemek için kullanılır.

	DMA1_Stream5->PAR=srcAdd;


	DMA1_Stream5->M0AR = destAdd; //Single Buffer (Tek Bellek Alanı): Eğer sadece tek bir bellek alanı kullanarak veri transferi yapacaksanız, yani veri okunurken veya yazılırken sadece tek bir bellek alanı kullanılıyorsa, DMA_SxM0AR kaydını kullanabilirsiniz. Bu durumda sadece tek bir bellek alanı üzerinde işlem yapılacağından, işlemi daha basit ve direkt hale getirir.
//Double Buffer (Çift Bellek Alanı): Eğer veri transferi sırasında aynı anda hem bir bellek alanından okuma hem de başka bir bellek alanına yazma işlemi yapmanız gerekiyorsa, çift döngü (double-buffer) veya ping-pong yöntemini kullanabilirsiniz. Bu durumda DMA_SxM0AR ve DMA_SxM1AR arasından birini seçmelisiniz. DMA transferi bir bellek alanı üzerinden devam ederken, diğer bellek alanında işlem yapabilirsiniz. Böylece veri transferi sırasında kesintisiz bir işlem gerçekleştirebilirsiniz.

	DMA1_Stream5->CR |= (1<<0) ;
}
#define RXSIZE 20
uint8_t RxBuf[20];
uint8_t MainBuf[50];

uint8_t indx=0;
void  DMA1_Stream5_IRQHandler (void){

	if ((DMA1->HISR)&(1<<10)){
	//HCIF (Half Transfer Complete Interrupt Flag), DMA işlemi sırasında yarım transfer tamamlandığını gösteren bir kesme bayrağıdır. Bu bayrak, DMA transferinin yarısının tamamlandığını gösterir.
	//DMA işlemi sırasında HISR (High Interrupt Status Register) kaydının içinde bulunur ve DMA transferinin yarım noktasına ulaşıldığında bu bayrak set edilir. Bu kesme bayrağı, yarım transfer noktasında bir kesme işlemi gerçekleştirmeniz gereken durumlarda kullanılır.
		memcpy (&MainBuf[indx], &RxBuf[0],RXSIZE/2);
		DMA1->HIFCR |=(1<<10);
		indx=indx+(RXSIZE/2);
		if (indx>49) indx=0;
	}
	if ((DMA1->HISR)&(1<<11)){
		memcpy (&MainBuf[indx], &RxBuf[RXSIZE/2] ,RXSIZE/2);
		DMA1->HIFCR |=(1<<11);
		indx=indx+(RXSIZE/2);
		if (indx>49) indx=0;
	}

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  Uart2config();
  DMA_init();
  //HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  NVIC_SetPriority (DMA1_Stream5_IRQn,0);
  NVIC_EnableIRQ (DMA1_Stream5_IRQn);
  Dma_Config((uint32_t) &USART2->DR, (uint32_t) RxBuf,RXSIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
