#ifndef USART_H
#define USART_H

#include "stm32f4xx.h"

void USART1_init(void);
int USART1_PutChar(char c);
int USART1_put_str(char* str);
char USART1_take_char(void);

#endif
