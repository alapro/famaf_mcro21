/*
 * bsp.c
 *
 *  Created on: 28 abr. 2021
 *      Author: agust
 */



#include "stm32f4xx.h"
//#include "stm32f411e_discovery.h"
#include "bsp.h"
#include "led.h"


extern void *ledRed;
extern void *ledBlue;
extern void *ledOrange;
extern void *ledGreen;

GPIOLED_TypeDef gpio_ledRed = {GPIOD, GPIO_PIN_14};
GPIOLED_TypeDef gpio_ledBlue = {GPIOD, GPIO_PIN_15};
GPIOLED_TypeDef gpio_ledOrange = {GPIOD, GPIO_PIN_13};
GPIOLED_TypeDef gpio_ledGreen = {GPIOD, GPIO_PIN_12};

TIM_HandleTypeDef htim2;

void *ledBlinky;
uint16_t timesBlinky, tonBlinky, toffBlinky;


void LED_Init(void);
void TIM2_Init(void);


extern void App_1msPeriod(void);

void BSP_Init(void){

	HAL_Init();

	//SystemClock_Config();

	LED_Init();
	TIM2_Init();

}


void LED_on(void *led){

	GPIOLED_TypeDef *gpioLed;

	gpioLed = (GPIOLED_TypeDef*)led;
	HAL_GPIO_WritePin(gpioLed->port, gpioLed->pin, GPIO_PIN_SET);
}

void LED_off(void *led){

	GPIOLED_TypeDef *gpioLed;

	gpioLed = (GPIOLED_TypeDef*)led;
	HAL_GPIO_WritePin(gpioLed->port, gpioLed->pin, GPIO_PIN_RESET);
}

void LED_toggle(void *led){
	GPIOLED_TypeDef *gpioLed;

	gpioLed = (GPIOLED_TypeDef*)led;
	HAL_GPIO_TogglePin(gpioLed->port, gpioLed->pin);
}

void LED_blinky(void *led, uint16_t ton, uint16_t toff, uint16_t times){

	ledBlinky = led;
	timesBlinky = times;
	tonBlinky = ton;
	toffBlinky = toff;
}

void LED_blinkyIRQ(void){
	static uint16_t ton = 1;
	static uint16_t toff = 1;

	if(timesBlinky){
		if(ton){
			ton--;
			if(!ton){
				LED_off(ledBlinky);
			}
		} else if(toff){
			toff--;
			if(!toff){
				timesBlinky--;
				if(timesBlinky){
					ton = tonBlinky;
					toff = toffBlinky;
					LED_on(ledBlinky);
				}
			}
		}
	}
}


void LED_Init(void){

	GPIO_InitTypeDef  GPIO_InitStruct;

	__HAL_RCC_GPIOD_CLK_ENABLE();

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

	GPIO_InitStruct.Pin = gpio_ledRed.pin;
	HAL_GPIO_Init(gpio_ledRed.port, &GPIO_InitStruct);
	ledRed = (void*)&gpio_ledRed;

	GPIO_InitStruct.Pin = gpio_ledBlue.pin;
	HAL_GPIO_Init(gpio_ledBlue.port, &GPIO_InitStruct);
	ledBlue = (void*)&gpio_ledBlue;

	GPIO_InitStruct.Pin = gpio_ledOrange.pin;
	HAL_GPIO_Init(gpio_ledOrange.port, &GPIO_InitStruct);
	ledOrange = (void*)&gpio_ledOrange;

	GPIO_InitStruct.Pin = gpio_ledGreen.pin;
	HAL_GPIO_Init(gpio_ledGreen.port, &GPIO_InitStruct);
	ledGreen = (void*)&gpio_ledGreen;
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 200;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 5;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

void TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 160;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END TIM2_Init 2 */

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /* TIM2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
}


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


////////////////////// Interrupt CallBack Services ///////////////////////////////

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
// Period Interrupt TIM2 = 1mseg.


	if (htim->Instance == TIM2) {

		LED_blinkyIRQ();
		App_1msPeriod();
    }

}




