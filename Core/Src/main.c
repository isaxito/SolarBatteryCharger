/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define I_MAX 0.4
#define V_MAX 14.3
#define VOLT_READINGS 40
#define CURRENT_READINGS 40
#define ADC_READINGS 4
#define R2 39.0 // kOhm
#define R3 10.0 // kOhm
#define R5 47.0 // kOhm
#define R6 10.0 // kOhm
//#define R17 47.0 // kOhm
//#define R12 2.7 // kOhm
#define R19 39 // kOhm
#define R20 2.7 // kOhm
#define R_PANEL_SHUNT 3.3 // Ohm
#define R_SHUNT 0.22 // Ohm
#define Vref 3.25 // Hardcode horrible para probar

#define ADC0CAL 0
#define ADC1CAL 0
#define ADC2CAL 550
#define ADC3CAL 0


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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t duty1 = 50;  // CH1 (PA0)
uint16_t duty2 = 35;  // CH2 (PA1)
uint16_t duty1_ccr = 720;
uint16_t duty2_ccr = 720;
uint16_t power_on_flag = 1;
float adc_readings[ADC_READINGS];
float voltage_panel[VOLT_READINGS];
float volt_panel_avg=0.0;
float voltage_battery[VOLT_READINGS];
float volt_batt_avg=0.0;
float test_shunt_panel[CURRENT_READINGS];
float current_panel_avg=0.0;
float volt1_shunt[CURRENT_READINGS];
float volt1_avg=0.0;
float load_current=0.0;


uint16_t tim4_counterr = 0; // Each count is 40s, up to the desired check value in standby mode.

/* State machine states:
	0: Standby
	1: Sunlight check
	2: Charge battery
*/
uint16_t sm_state = 0;

void SetDutyCycle(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t DutyPercent)
{
  uint32_t arr = htim->Init.Period + 1;
  uint32_t ccr = (DutyPercent * arr) / 100;
  __HAL_TIM_SET_COMPARE(htim, Channel, ccr);
}

void readADC_inputs()
{
	for(int i=0; i<ADC_READINGS; i++)
	{
		HAL_ADC_Start(&hadc1); // Start ADC Conversion
		HAL_ADC_PollForConversion(&hadc1, 1); // Poll ADC1 Peripheral & TimeOut = 1mSec
		adc_readings[i] = HAL_ADC_GetValue(&hadc1); // Read ADC Conversion Result
		HAL_Delay(1);
	}
}

void sunlightCheck()
{
	// Activate solar panel test with current source (view schematic)
	SetDutyCycle(&htim2, TIM_CHANNEL_2, duty2);
	// Wait 500ms
	HAL_Delay(500);
	// Read panel voltage divider and test current values
	for(int i=0; i < VOLT_READINGS; i++)
	{
		readADC_inputs();
		voltage_panel[i] = adc_readings[0];
		test_shunt_panel[i] = adc_readings[1];
		voltage_battery[i] = adc_readings[2];
		HAL_Delay(1);
	}

	// Deactivate solar panel test with current source
	SetDutyCycle(&htim2, TIM_CHANNEL_2, 0);

	// Average readings
	volt_panel_avg = 0;
	current_panel_avg = 0;
	volt_batt_avg = 0;
	for(int i=0; i < VOLT_READINGS; i++)
	{
		volt_panel_avg += voltage_panel[i] + ADC1CAL;
		current_panel_avg += test_shunt_panel[i] + ADC0CAL;
		volt_batt_avg += voltage_battery[i] + ADC2CAL;
	}
	// Calculate real average panel voltage and test current
	volt_panel_avg = ( ((volt_panel_avg / VOLT_READINGS)*Vref)/4095.0 ) * ((R5+R6)/R6) ;
	current_panel_avg = ( ((current_panel_avg / VOLT_READINGS)*Vref)/4095.0 ) / R_PANEL_SHUNT;
	volt_batt_avg = ( ((volt_panel_avg / VOLT_READINGS)*Vref)/4095.0 ) * ((R2+R3)/R3);
}

void testADC()
{
   // Read shunt terminal voltages
   for(int i=0; i < VOLT_READINGS; i++)
	{
		readADC_inputs();
		voltage_panel[i] = adc_readings[0];
		test_shunt_panel[i] = adc_readings[1];
		voltage_battery[i] = adc_readings[2];
		volt1_shunt[i] = adc_readings[3];
	}
   // Average readings
   volt_panel_avg = 0;
   current_panel_avg = 0;
   volt_batt_avg = 0;
   volt1_avg = 0;
   for(int i=0; i < VOLT_READINGS; i++)
   {
	   volt_panel_avg += voltage_panel[i];
	   current_panel_avg += test_shunt_panel[i];
	   volt_batt_avg += voltage_battery[i];
	   volt1_avg += volt1_shunt[i];
   }

   // Finish calculating average and real voltages

   volt_panel_avg = ( ((volt_panel_avg / VOLT_READINGS)*Vref)/4095.0 ) * ((R5+R6)/R6) ;
   current_panel_avg = current_panel_avg / VOLT_READINGS;
   load_current = (( (( (volt1_avg / VOLT_READINGS)*Vref)/4095.0) / (1 + R19/R20) )) / R_SHUNT ; // Current load
   volt_batt_avg = ( ((volt_batt_avg / VOLT_READINGS)*Vref)/4095.0 ) * ((R2+R3)/R3); // Battery voltage
}

void adjustChargerPWM()
{
   // Read shunt terminal voltages
   for(int i=0; i < VOLT_READINGS; i++)
	{
		readADC_inputs();
		voltage_panel[i] = adc_readings[0];
		voltage_battery[i] = adc_readings[2];
		volt1_shunt[i] = adc_readings[3];
	}
   // Average readings
   volt_panel_avg = 0;
   volt_batt_avg = 0;
   volt1_avg = 0;
   for(int i=0; i < VOLT_READINGS; i++)
   {
	   volt_panel_avg += voltage_panel[i] + ADC1CAL;
	   volt_batt_avg += voltage_battery[i] + ADC2CAL;
	   volt1_avg += volt1_shunt[i] + ADC0CAL;
   }

   // Finish calculating average and real voltages

   volt_panel_avg = ( ((volt_panel_avg / VOLT_READINGS)*Vref)/4095.0 ) * ((R5+R6)/R6) ;
   volt_batt_avg = ( ((volt_batt_avg / VOLT_READINGS)*Vref)/4095.0 ) * ((R2+R3)/R3); // Battery voltage
   load_current = (( (( (volt1_avg / VOLT_READINGS)*Vref)/4095.0) / (1 + R19/R20) )) / R_SHUNT ; // Current load

   // PWM Duty control logic for constant current and voltage modes

   if(load_current < I_MAX){
	  if(volt_batt_avg < V_MAX){
		 if(duty1<92)
			 duty1++;
		 HAL_Delay(2);
		 SetDutyCycle(&htim2, TIM_CHANNEL_1, duty1);
	  }
	  else{
		 if(duty1>6)
			 duty1--;
		 HAL_Delay(2);
		 SetDutyCycle(&htim2, TIM_CHANNEL_1, duty1);
	  }
   }
   else
   {
	   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	   HAL_Delay(100);
	   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	   HAL_Delay(100);
   }

//   if(load_current < I_MAX){
//	  if(volt_batt_avg < V_MAX){
//		 if(duty1_ccr<1430)
//			 duty1_ccr++;
//		 HAL_Delay(1);
//		 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty1_ccr);
//	  }
//	  else{
//		 if(duty1_ccr>4)
//			duty1_ccr--;
//		 HAL_Delay(1);
//		 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty1_ccr);
//	  }
//   }
}

void updateStateLed(int state_num)
{
	switch(state_num)
	{
	case 0:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		break;
	default:
		break;

	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	// When turned on, uC comes here directly without waiting for the timer, so this is a fix for the moment

	if(power_on_flag)
		tim4_counterr = 0;
	else
		tim4_counterr++;

	if(tim4_counterr==1)
	{
		// Check sunlight flag every 80 seconds (for example)
		if(sm_state == 0)
		{
			sm_state = 1;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			//HAL_Delay(1000);
		}
		tim4_counterr = 0;
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

	//uint16_t adc_reading = 0;

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
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  SetDutyCycle(&htim2, TIM_CHANNEL_1, 5);
  SetDutyCycle(&htim2, TIM_CHANNEL_2, 0);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  power_on_flag = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //while(1)
  //	  testADC();

  while (1)
  {

	switch(sm_state)
	{
	case 0: // Standby, timer will periodically change sm_state to 1.
		power_on_flag = 0;
		// Stay, do nothing but maintain PWM outputs OFF
		SetDutyCycle(&htim2, TIM_CHANNEL_1, 5);
		SetDutyCycle(&htim2, TIM_CHANNEL_2, 0);
		// Update indicator LEDs
		updateStateLed(sm_state);

		break;
	case 1: // Sunlight check
		// Update indicator LEDs
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		updateStateLed(sm_state);

		sunlightCheck();

		if((volt_panel_avg >= 12.0) && (current_panel_avg >= 0.25) && volt_batt_avg < 14.3) // volt_panel_avg >= 10.0
		{
			sm_state = 2; // Go to charge battery state
			SetDutyCycle(&htim2, TIM_CHANNEL_1, 50);
			SetDutyCycle(&htim2, TIM_CHANNEL_2, 0);
		}else
			sm_state = 0; // Go back to standby state
		break;

	case 2: // Charge battery
		// Update indicator LEDs
		updateStateLed(sm_state);
		// Charging algorithm and reading of ADCs
		adjustChargerPWM();

		if(volt_batt_avg >= 15.0 || volt_panel_avg < 12.0) // volt_panel_avg < 5.0
		{
			// Turns OFF Charger and goes to standby
			SetDutyCycle(&htim2, TIM_CHANNEL_1, 5);
			sm_state = 0;
		}
		else
			sm_state = 2;
		break;

	default:
		sm_state = 0;
		SetDutyCycle(&htim2, TIM_CHANNEL_1, 5);
		SetDutyCycle(&htim2, TIM_CHANNEL_2, 0);
		break;
	}


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1439;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 59999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 47999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
#ifdef USE_FULL_ASSERT
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
