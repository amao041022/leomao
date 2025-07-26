#ifndef __UART_H
#define __UART_H

#include "stm32f10x.h"

void UART_Init(void);

extern char RxBuffer[32];        // 接收缓冲区
extern uint8_t ParseComplete;    // 接收完成标志

#endif
