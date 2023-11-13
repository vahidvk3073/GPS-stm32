#ifndef __MYGSM_H
#define __MYGSM_H

#include "main.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define HUART &huart1
#define RX_BUFFER_SIZE 100
#define RECEIVE_NUMBER_SIZE 30
#define TRUE 1
#define FALSE 0
#define DEBUG_MODE 0



void 				send_string(char *msg);
void 				GSM_init(void);
uint8_t 		send_AT_command(char *command , char *GSM_received_answer);
uint8_t 		GSM_sendMessage(char *msg , char *number);
uint8_t 		GSM_call(char *number);
uint8_t 		GSM_callDisconnect(void);
void  			GSM_readMessage(void);
void				get_answer(void);
void 				rx_clear(void);
uint8_t 		GSM_messageDelete(void);
uint8_t			GSM_goSleep(void);
uint8_t			GSM_wakeup(void);
void				clear_all_buffers(void);
void 				yellow_blink(uint16_t delay);
void 				green_blink(uint16_t delay);
#endif
