/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdlib.h"
#include "stdio.h"
#include "GSM_stm32.h"
#include "gps.h"
#include "sleep_functions.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAIN_BLINK_INTERVAL 2000
#define LARZESH_RTC_INTERVAL 10000
#define GOING_SLEEP_INTERVAL 40000


#define ALARM_HOUR 21
#define ALARM_MINUTE 0
#define ALARM_SECOND 0


#define Vref 3.3
#define ADC_SAMPLE_NUMBER 100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

extern char uart1_rx_buffer[RX_BUFFER_SIZE];
extern char rx_temp;
extern char receive_message[RX_BUFFER_SIZE];
extern char receive_message_number[RECEIVE_NUMBER_SIZE];
extern char receive_message_date[RECEIVE_NUMBER_SIZE];
char number[]="+989378936996";


extern GPS_t GPS;
char map_link[80];
uint32_t send_locatoin_millis = 0;


RTC_TimeTypeDef myTime;
RTC_AlarmTypeDef myAlarm;
uint8_t RTC_Alarm_Flag = 0;
uint8_t inputBuffer[3] = {0};


uint8_t larzesh_flag = 0 ;
uint32_t millis_start_value = 0;


uint16_t adc_value[1000] = {0};
float adc_value_mean = 0;
float Battery_voltage = 0;
uint8_t adc_flag = 0;


uint32_t main_millis = 0;
uint8_t blink_state = SET;

extern uint8_t sleep_mode_flag;
extern uint32_t sleep_mode_millis;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void Set_Alarm(uint8_t h, uint8_t m, uint8_t s);

float check_battery_voltage(void);

void send_location(void);

void send_battery_voltage(void);

void send_larzesh_alarm(void);

void send_RTC_alarm(void);

void RTC_init(void);

void clear_all_buffer(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
		if (huart == &huart2)
		{
				GPS_UART_CallBack();
		}
		else if (huart == &huart1)
		{
				get_answer();
		}
}


void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
	
		if (GPIO_Pin == LARZESH_Pin)
		{
				millis_start_value = HAL_GetTick();
				larzesh_flag = 1;
		}
}


void HAL_RTC_AlarmAEventCallback (RTC_HandleTypeDef *hrtc)
{
		millis_start_value = HAL_GetTick();
		RTC_Alarm_Flag = 1;
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

	GSM_wakeup();
	
	GSM_init();	
	
	RTC_init();
	
	sleep_mode_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
			if ((HAL_GetTick() - main_millis) > MAIN_BLINK_INTERVAL)
			{
					HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, !blink_state);
					main_millis = HAL_GetTick();
		  }
			
			
			
			
			if (strstr(uart1_rx_buffer, "+CMTI:") != NULL)
			{
				
					GSM_read_message();
				
					if ((strstr(receive_message, "send location") != NULL) && (strstr(receive_message_number, number) != NULL) )
					{
							send_location();
					}

					
					
			/*check battery voltage*/
					
					if ((strstr(receive_message , "battery voltage") != NULL) && (strstr(receive_message_number , number) != NULL) )
					{
							send_battery_voltage();
					}
			
					
					clear_all_buffer();
					
					sleep_mode_init();
		}

		
		
		
		if ((HAL_GetTick() - millis_start_value) > LARZESH_RTC_INTERVAL)
		{
				if (larzesh_flag == 1)
				{
						send_larzesh_alarm();
				}
				

				if (RTC_Alarm_Flag == 1)
				{
						send_RTC_alarm();
				}
		}
		
		
		
		if ((HAL_GetTick() - sleep_mode_millis) > GOING_SLEEP_INTERVAL)
		{
				if (sleep_mode_flag == 1)
				{
						HAL_GPIO_WritePin(NEO6m_power_GPIO_Port, NEO6m_power_Pin, GPIO_PIN_RESET);
						GSM_go_sleep();
						HAL_Delay(1000);
						go_to_sleepMode();
				}	
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



void Set_Alarm (uint8_t h, uint8_t m, uint8_t s)
{
		myAlarm.AlarmTime.Hours = h;
		myAlarm.AlarmTime.Minutes = m ;
		myAlarm.AlarmTime.Seconds = s ;
		myAlarm.Alarm = RTC_ALARM_A;
		HAL_RTC_SetAlarm_IT(&hrtc, &myAlarm, RTC_FORMAT_BIN);
}




float check_battery_voltage (void)
{
		for (uint16_t i = 0; i < ADC_SAMPLE_NUMBER; i++)
		{
				HAL_ADC_Start(&hadc1);
				HAL_ADC_PollForConversion(&hadc1, 1);
				adc_value[i] = HAL_ADC_GetValue(&hadc1);
				HAL_ADC_Stop(&hadc1);
				HAL_Delay(100);
				adc_value_mean = adc_value_mean + adc_value[i];
		}
		
		adc_value_mean = (adc_value_mean / (float)ADC_SAMPLE_NUMBER);
		
		return ((adc_value_mean * Vref) / 4095.0);
}



void send_location (void)
{
		send_locatoin_millis = HAL_GetTick();
		
		HAL_UART_MspInit(&huart2);
		
		HAL_GPIO_WritePin(NEO6m_power_GPIO_Port, NEO6m_power_Pin, GPIO_PIN_SET);
		
		GPS_Init();
		
		
	#if DEBUG_MODE
			GSM_sendMessage("map location \r\n" , number);
	#else
		
		
		do
		{
				if (GPS.dec_latitude != 0 && GPS.dec_longitude != 0)
				{
						float Battery_voltage = 2 * check_battery_voltage();
					
						if (Battery_voltage > 3 )
						{
								sprintf(map_link, "http://maps.google.com/maps?q=loc:%f,%f \r\n", GPS.dec_latitude, GPS.dec_longitude);
						}
				}
				
				
				if ((HAL_GetTick() - send_locatoin_millis) > GOING_SLEEP_INTERVAL)
				{
					GSM_send_message("error in supply NEO-6m power \r\n", number);
					
					HAL_NVIC_SystemReset();
				}
				
		}while (GPS.dec_latitude == 0 && GPS.dec_longitude == 0);
		
		
		
		GSM_send_message(map_link, number);
		
		HAL_UART_MspDeInit(&huart2);
		
		#endif				
		
		HAL_GPIO_WritePin(NEO6m_power_GPIO_Port, NEO6m_power_Pin, GPIO_PIN_RESET);
}
	


void send_battery_voltage (void)
{
		yellow_blink(1000);
	
		Battery_voltage = 2 * check_battery_voltage();
	
		char adc_buffer[30];
	
		sprintf(adc_buffer ,"battery voltage:%0.2f \r\n",Battery_voltage);
	
		GSM_send_message(adc_buffer, number);
}




void send_larzesh_alarm (void)
{
		larzesh_flag = 0;

		GSM_wakeup();

		GSM_send_message("probable danger,check your car\r\n", number);

		sleep_mode_flag = 1;

		sleep_mode_millis = HAL_GetTick();

		rx_buffer_clear();
}



void send_RTC_alarm (void)
{
		GSM_wakeup();
		
		RTC_Alarm_Flag = 0;
		
		Battery_voltage = check_battery_voltage();
		
		char adc_buffer[30];
		sprintf(adc_buffer, "battery voltage:%f \r\n", Battery_voltage);
		GSM_send_message(adc_buffer, number);
		
		Set_Alarm(ALARM_HOUR, ALARM_MINUTE, ALARM_SECOND);
		
		sleep_mode_flag = 1;
		
		sleep_mode_millis = HAL_GetTick();
}

void clear_all_buffer (void)
{
	rx_buffer_clear();
	memset(receive_message , 0 , RX_BUFFER_SIZE);
	memset(receive_message_number , 0 , RECEIVE_NUMBER_SIZE);
	memset(receive_message_date , 0 , RECEIVE_NUMBER_SIZE);
}

void RTC_init (void)
{
/*set Alarm setting */
	/*
		Set_Alarm(ALARM_HOUR, ALARM_MINUTE, ALARM_SECOND);
		
		myTime.Hours = inputBuffer[0];
		myTime.Minutes = inputBuffer[1];
		myTime.Seconds = inputBuffer[2];
		
		HAL_RTC_SetTime(&hrtc, &myTime, RTC_FORMAT_BIN);
	*/
	
	
	/*check reset flag */
	/*
	if(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
		{
				green_blink(50);
				__HAL_RCC_CLEAR_RESET_FLAGS();
		}
	*/
}



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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
