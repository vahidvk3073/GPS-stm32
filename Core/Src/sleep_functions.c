#include "sleep_functions.h"


uint8_t sleep_mode_flag = 1 ;

uint32_t sleep_mode_millis = 0;



void go_to_sleepMode (void)
{
		green_blink(100);
		
		HAL_SuspendTick();
		HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		HAL_ResumeTick();
		
		sleep_mode_millis = HAL_GetTick();
		
		sleep_mode_flag = 1;

		green_blink(100);
}

void sleep_mode_init (void)
{
		sleep_mode_flag = 1;
	
		sleep_mode_millis = HAL_GetTick();
}