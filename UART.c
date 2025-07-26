//// UART.c
//#include "stm32f10x.h"

//// Buffer to store incoming data
//char RxBuffer[100];
//int RxCounter = 0;
//// Flag to indicate a full packet has been received
//int ParseComplete = 0;

//void UART_Init(void) {
//    // Enable clocks
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

//    // Configure PA9 (TX)
//    GPIO_InitTypeDef GPIO_InitStructure_TX;
//    GPIO_InitStructure_TX.GPIO_Pin = GPIO_Pin_9;
//    GPIO_InitStructure_TX.GPIO_Mode = GPIO_Mode_AF_PP;
//    GPIO_InitStructure_TX.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOA, &GPIO_InitStructure_TX);

//    // Configure PA10 (RX)
//    GPIO_InitTypeDef GPIO_InitStructure_RX;
//    GPIO_InitStructure_RX.GPIO_Pin = GPIO_Pin_10;
//    GPIO_InitStructure_RX.GPIO_Mode = GPIO_Mode_IPU; // Input Pull-up
//    GPIO_Init(GPIOA, &GPIO_InitStructure_RX);

//    // Configure USART1
//    USART_InitTypeDef USART_InitStructure;
//    USART_InitStructure.USART_BaudRate = 115200;
//    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//    USART_InitStructure.USART_StopBits = USART_StopBits_1;
//    USART_InitStructure.USART_Parity = USART_Parity_No;
//    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//    USART_Init(USART1, &USART_InitStructure);

//    // Enable USART1 Receive Interrupt
//    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

//    // Configure NVIC
//    NVIC_InitTypeDef NVIC_InitStructure;
//    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);

//    // Enable USART1
//    USART_Cmd(USART1, ENABLE);
//}


//// This function handles USART1 interrupt requests.
//void USART1_IRQHandler(void) {
//    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
//        char rx_data = USART_ReceiveData(USART1);

//        // Check for end of line character '\n'
//        if (rx_data == '\n' || RxCounter >= sizeof(RxBuffer) - 1) {
//            RxBuffer[RxCounter] = '\0'; // Null-terminate the string
//            RxCounter = 0;
//            ParseComplete = 1; // Signal to main loop to parse the buffer
//        } else {
//            RxBuffer[RxCounter++] = rx_data;
//        }
//    }
//}

#include "UART.h"
#include <string.h>

char RxBuffer[32];
uint8_t ParseComplete = 0;

static uint8_t RxIndex = 0;

void UART_Init(void)
{
    // ① 打开时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    // ② 配置 PA10 为浮空输入（RX）
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    // ③ 初始化串口参数
    USART_InitTypeDef usart;
    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_Mode = USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &usart);

    // ④ 开启接收中断
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);

    // ⑤ 配置 NVIC 中断优先级
    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

// ⑥ 串口接收中断服务函数
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        char ch = USART_ReceiveData(USART1);

        if (ch == '\n') // 接收到一整帧
        {
            RxBuffer[RxIndex] = '\0'; // 字符串结束符
            RxIndex = 0;
            ParseComplete = 1;
        }
        else
        {
            if (RxIndex < sizeof(RxBuffer) - 1)
            {
                RxBuffer[RxIndex++] = ch;
            }
        }
    }
}
