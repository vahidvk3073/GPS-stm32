#ifndef __MYGSM_H
#define __MYGSM_H

#include "main.h"
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define HUART &huart1
#define RX_BUFFER_SIZE 100
#define RECEIVE_NUMBER_SIZE 30
#define TRUE 1
#define FALSE 0



void 				send_string(char *msg);
void				get_answer(void);
void 				rx_buffer_clear(void);
void				clear_all_buffer (void);
void 				yellow_blink(uint16_t delay);
void 				green_blink(uint16_t delay);
uint8_t 		send_AT_command(char *command, char *GSM_received_answer);

uint8_t 		GSM_init(void);

void  			GSM_read_message(void);
uint8_t 		GSM_send_message(char *msg, char *number);
uint8_t 		GSM_message_delete(void);

uint8_t 		GSM_call(char *number);
uint8_t 		GSM_call_disconnect(void);

uint8_t			GSM_go_sleep(void);
uint8_t			GSM_wakeup(void);

#endif
