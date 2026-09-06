#include "stm32f10x.h"
#include <stdio.h>
#include <stdarg.h>

/* USART2 receive ring buffer.
   The camera/AI flow and the Bluetooth 'T'/'G' share USART2. Bytes are
   buffered in the RX interrupt so nothing is lost when several arrive
   back-to-back. 'T' (stop) and 'G' (resume) are kept in a separate
   priority slot so they are always served first, never queued behind
   ordinary flow commands.

   Single-producer/single-consumer: only the ISR writes rx_head, only the
   main loop writes rx_tail, so there is no shared counter to race on. */

#define RX_BUF_SIZE 16
#define RX_BUF_MASK (RX_BUF_SIZE - 1)   /* power-of-two, so & is modulo */

static volatile uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint8_t rx_head = 0;    /* write index (ISR only) */
static volatile uint8_t rx_tail = 0;    /* read index (main loop only) */

static volatile uint8_t urgent_cmd = 0;       /* 'T' or 'G' */
static volatile uint8_t urgent_pending = 0;   /* 1 = urgent command ready */

void Serial_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    /* PA2 = USART2_TX (AF push-pull) */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA3 = USART2_RX (floating input) */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART2, &USART_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART2, ENABLE);
}

void Serial_SendByte(uint8_t Byte)
{
    USART_SendData(USART2, Byte);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) {
        Result *= X;
    }
    return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++) {
        Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/* 1 = at least one byte available (urgent first, then buffer). */
uint8_t Serial_GetRxFlag(void)
{
    return (urgent_pending || (rx_head != rx_tail)) ? 1 : 0;
}

/* Pop one byte: urgent 'T'/'G' first, then the ring buffer. */
uint8_t Serial_GetRxData(void)
{
    if (urgent_pending) {
        urgent_pending = 0;
        return urgent_cmd;
    }
    if (rx_head != rx_tail) {
        uint8_t c = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) & RX_BUF_MASK;
        return c;
    }
    return 0;
}

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET) {
        uint8_t c = USART_ReceiveData(USART2);
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);

        if (c == 'T' || c == 'G') {
            urgent_cmd = c;          /* latest stop/resume wins, served first */
            urgent_pending = 1;
        } else {
            uint8_t next = (rx_head + 1) & RX_BUF_MASK;
            if (next != rx_tail) {   /* not full (one slot left empty) */
                rx_buf[rx_head] = c;
                rx_head = next;
            }
            /* else: buffer full, drop this byte */
        }
    }
}
