/*
 * shell.h
 *
 *  Created on: 7 juin 2019
 *      Author: laurent
 */

#ifndef INC_LIB_SHELL_SHELL_H_
#define INC_LIB_SHELL_SHELL_H_

#include <stdint.h>

#define UART_DEVICE huart1

#define _SHELL_FUNC_LIST_MAX_SIZE 64

char uart_read();
int uart_write(char * s, uint16_t size);
void uart_data_ready();
void shell_char_received();
void shell_init();
int shell_add(char c, int (* pfunc)(int argc, char ** argv), char * description);
int shell_exec(char c, char * buf);
int shell_run();

#endif /* INC_LIB_SHELL_SHELL_H_ */
