#include "GSM_stm32.h"


extern UART_HandleTypeDef huart1;

char uart1_rx_temp ;

char uart1_rx_buffer[RX_BUFFER_SIZE] = {'\0'};

char receive_message[RX_BUFFER_SIZE] = {'\0'};
char receive_message_number[RECEIVE_NUMBER_SIZE] = {'\0'};
char receive_message_date[30] = {'\0'};

uint8_t uart1_rx_index = 0, cmp_index = 0  ;


/*-------------------------Init SIM800 module---------------------------*/

void GSM_init (void)
{
		uint8_t ok = 1;
		
		/*turn off echo*/
		
		if (ok == 1)
		{
				ok = send_AT_command("ATE0\r\n", "OK\r\n");
		}
		
		/*access to network*/

		if (ok == 1)
		{
				ok = send_AT_command("AT+CSQ\r\n", "+CSQ");
		}	
		
		/*send in text mode*/
		
		if (ok == 1)
		{
				ok = send_AT_command("AT+CMGF=1\r\n", "OK\r\n");
		}
		
		/*save sms in sim card*/
		
		if (ok == 1)
		{
				ok = send_AT_command("AT+CNMI=2,1,0,0,0\r\n", "OK\r\n");
		}
		
		/*dellete all sms saved in sim memory*/

		if (ok == 1)
		{
				ok = send_AT_command("AT+CMGDA=\"DEL ALL\"\r\n", "OK\r\n");
		}	

		
		/*save setting*/
		
		if (ok == 1)
		{
				ok = send_AT_command("AT&W\r\n", "OK\r\n");
		}	

		
		rx_clear();
		HAL_UART_Receive_IT(HUART , (uint8_t *)&uart1_rx_temp , 1);
}


/*------------------------Send Command To SIM800-------------------------*/

uint8_t send_AT_command (char *command, char *GSM_received_answer)
{	
	  uint16_t timeout = 10000;
	
	  rx_clear();
	
	  send_string(command);
	
		while (timeout > 0)
		{
				if (strstr(uart1_rx_buffer, GSM_received_answer) != NULL)
				{
					  return TRUE;
				}

			  timeout -= 100;
			  HAL_Delay(50);
		}
		
		return FALSE;
}

/*--------------------------Call to a Number-------------------------*/

uint8_t GSM_call (char *number)
{
    char _command[RX_BUFFER_SIZE];
	
    sprintf(_command, "ATD+ %s;\r\n", number);
	
	  if (send_AT_command("ATH\r\n", "OK\r\n"))
		{
				rx_clear();
			  return TRUE;
		}
		else
		{
				rx_clear();
			  return FALSE;
		}
}

/*------------------------Disconnect Received Call--------------------------*/

uint8_t GSM_callDisconnect (void)
{
    if (send_AT_command("ATH\r\n", "OK\r\n"))
		{
				rx_clear();
			  return TRUE;
		}
		else
		{
				rx_clear();
			  return FALSE;
		}
}

/*------------------------------ Send SMS  ---------------------------------*/

uint8_t GSM_sendMessage (char *msg, char *number)
{
    char _buffer[RX_BUFFER_SIZE];
	
    sprintf(_buffer, "AT+CMGS=\"%s\"\r\n", number);
	
    if (send_AT_command(_buffer, "\r\n>") == FALSE)
    {
        rx_clear();
        return FALSE;
    }
		else 
		{
        sprintf(_buffer, "%s%c", msg, 26);
			
        send_string(_buffer);
			
			  if (send_AT_command(_buffer, "+CMGS"))
				{
			      rx_clear();
					  return TRUE;
		    }
		    else
		    {
			      rx_clear();
					  return FALSE;
		    }
    }
}

/*------------------------------ Send String to SIM800 On UART  ---------------------------------*/

void send_string (char *msg)
{
    HAL_UART_Receive_IT(HUART, (uint8_t *)&uart1_rx_temp, 1);
    HAL_UART_Transmit(HUART, (uint8_t *)msg, strlen(msg), 1000);
}

/*------------------------------ Read SMS Received from SIM800  ---------------------------------*/

void GSM_readMessage (void)
{
		char *a ;
		char read_SIM_cmd[30], Msg_number[20], Msg_date[30], Msg_text[100], buffer[40];
		uint8_t msg_index = 0, buffer_len = 0;
		
		msg_index = uart1_rx_buffer[14]-48;

		rx_clear();
		
		while (!GSM_wakeup());
		
		sprintf(buffer, "AT+CMGR=%d\r\n", msg_index);
		send_AT_command(buffer, "+CMGR");
		
		a = strstr(uart1_rx_buffer, "+CMGR");
		if (a)
		{
				sscanf(a, "%*[^,],\"%[^\"]\",%*[^,],\"%[^\"]\"\r\n%[^\r]", Msg_number, Msg_date, Msg_text);
				strcpy(receive_message, Msg_text);	
				strcpy(receive_message_number, Msg_number);
				strcpy(receive_message_date, Msg_date);
				rx_clear();
		}	
		
		if (msg_index == 9)
		{
				GSM_messageDelete();
				rx_clear();
		}
}

/*-------------------------- Function In UART Interrupt Callback  ---------------------------*/

void get_answer (void)
{
		if(uart1_rx_index > (RX_BUFFER_SIZE - 1))
		{
				uart1_rx_index = 0;
		}
		
		uart1_rx_buffer[uart1_rx_index++] = uart1_rx_temp;
		
		HAL_UART_Receive_IT(HUART, (uint8_t *)&uart1_rx_temp, 1);
}

/*---------------------------- Set rx_buffer and rx_index to 0  ----------------------------*/

void rx_clear (void)
{
		uart1_rx_index = 0;
	
		memset(uart1_rx_buffer, 0, RX_BUFFER_SIZE);
}

/*-------------------------- Dellete All SMS That Stored In SIM800  --------------------------*/

uint8_t GSM_messageDelete (void)
{
    if (send_AT_command("AT+CMGDA=\"DEL ALL\"\r\n", "OK\r\n"))
		{
				rx_clear();
			  return TRUE;
		}
		else
		{
				rx_clear();
			  return FALSE;
		}	
}

/*-------------------------- SIM800 Goes In Sleep Mode  --------------------------*/

uint8_t GSM_goSleep (void)
{
    if (send_AT_command("AT+CSCLK=2\r\n", "OK\r\n"))
		{
				rx_clear();
			  return TRUE;
		}
		else
		{
				rx_clear();
			  return FALSE;
		}
}

/*-------------------------- SIM800 Gets Wake Up Command  --------------------------*/

uint8_t GSM_wakeup (void)
{
	send_string("AT\r\n");
	HAL_Delay(200);
	
    if (send_AT_command("AT+CSCLK=0\r\n", "OK\r\n"))
		{
				rx_clear();
			  return TRUE;
		}
		else
		{
				rx_clear();
			  return FALSE;
		}
}

/*-------------------------- blink  --------------------------*/

void 	green_blink (uint16_t delay)
{
		HAL_GPIO_WritePin(GREEN_GPIO_Port ,GREEN_Pin ,GPIO_PIN_SET);
		HAL_Delay(delay);
		HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);
}

void 	yellow_blink (uint16_t delay)
{
		HAL_GPIO_WritePin(YELLOW_GPIO_Port, YELLOW_Pin, GPIO_PIN_SET);
		HAL_Delay(delay);
		HAL_GPIO_WritePin(YELLOW_GPIO_Port, YELLOW_Pin, GPIO_PIN_RESET);
}
