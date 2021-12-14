/*
 * shell.c
 *
 *  Created on: 7 juin 2019
 *      Author: Laurent Fiack
 */

#include "shell.h"

#include <stdio.h>

#include "usart.h"
#include "gpio.h"

#define ARGC_MAX 8
#define BUFFER_SIZE 40
char help[] = "help";

typedef struct{
    char c;
    int (* func)(int argc, char ** argv);
    char * description;
} shell_func_t;

static int shell_func_list_size = 0;
static shell_func_t shell_func_list[_SHELL_FUNC_LIST_MAX_SIZE];

static int dataReady = 0;

int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

char uart_read() {
	char c;

	HAL_UART_Receive(&UART_DEVICE, (uint8_t*)(&c), 1, 0xFFFFFFFF);

	return c;
}

int uart_write(char * s, uint16_t size) {
	HAL_UART_Transmit(&UART_DEVICE, (uint8_t*)s, size, 0xFFFF);
	return size;
}

void uart_data_ready() {
	dataReady = 1;
}

int sh_help(int argc, char ** argv) {
    int i;
    for(i = 0 ; i < shell_func_list_size ; i++) {
        printf("%c %s\r\n", shell_func_list[i].c, shell_func_list[i].description);
    }

    return 0;
}

void shell_init() {
	printf("\r\n\r\n===== Monsieur Shell v0.2 =====\r\n");

	shell_add('h', sh_help, help);

	for (int i = 0 ; i < 3 ; i++) {

		HAL_Delay(200);
	}
}

int shell_add(char c, int (* pfunc)(int argc, char ** argv), char * description) {
    if (shell_func_list_size < _SHELL_FUNC_LIST_MAX_SIZE) {
        shell_func_list[shell_func_list_size].c = c;
        shell_func_list[shell_func_list_size].func = pfunc;
        shell_func_list[shell_func_list_size].description = description;
        shell_func_list_size++;
        return 0;
    }

    return -1;
}

int shell_exec(char c, char * buf) {
    int i;

    int argc;
    char * argv[ARGC_MAX];
    char *p;

    for(i = 0 ; i < shell_func_list_size ; i++) {
        if (shell_func_list[i].c == c) {
            argc = 1;
            argv[0] = buf;

            for(p = buf ; *p != '\0' && argc < ARGC_MAX ; p++){
                if(*p == ' ') {
                    *p = '\0';
                    argv[argc++] = p+1;
                }
            }

            return shell_func_list[i].func(argc, argv);
        }
    }
    printf("%c: Command not found\r\n", c);
    return -1;
}

static char buf[BUFFER_SIZE];
static char backspace[] = "\b \b";
static char prompt[] = "> ";

int shell_run() {
	int reading = 0;
	int pos = 0;

	while (1) {
		  uart_write(prompt, 2);
		  reading = 1;

		  while(reading) {
			  char c = uart_read();

			  switch (c) {
				  //process RETURN key
				  case '\r':
					  //case '\n':
					  printf("\r\n");    //finish line
					  buf[pos++] = 0;     //to use cprintf...
					  printf(":%s\r\n", buf);
					  reading = 0;        //exit read loop
					  pos = 0;            //reset buffer
					  break;
					  //backspace
				  case '\b':
					  if (pos > 0) {      //is there a char to delete?
						  pos--;          //remove it in buffer

						  uart_write(backspace, 3);
					  }
					  break;
					  //other characters
				  default:
					  //only store characters if buffer has space
					  if (pos < BUFFER_SIZE) {
						  uart_write(&c, 1);
						  buf[pos++] = c; //store
					  }
			  }
		  }
		  shell_exec(buf[0], buf);
	}
	return 0;
}
