#include "stm32f10x.h"
#include "UART.h"

#define UART_RX_BUFFER_SIZE  96

static volatile char UART_RxBuffer[UART_RX_BUFFER_SIZE];
static volatile uint16_t UART_RxIndex = 0;
static volatile uint8_t UART_LineReady = 0;

void UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    /* PA9: USART1_TX */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA10: USART1_RX */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

void USART1_IRQHandler(void)
{
    char ch;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        ch = (char)(USART_ReceiveData(USART1) & 0xFF);

        if (!UART_LineReady)
        {
            if (ch == '\r' || ch == '\n')
            {
                if (UART_RxIndex > 0)
                {
                    UART_RxBuffer[UART_RxIndex] = '\0';
                    UART_LineReady = 1;
                    UART_RxIndex = 0;
                }
            }
            else if (UART_RxIndex < (UART_RX_BUFFER_SIZE - 1))
            {
                UART_RxBuffer[UART_RxIndex++] = ch;
            }
            else
            {
                /* 丢弃过长命令，从下一行重新接收 */
                UART_RxIndex = 0;
            }
        }
    }
}

uint8_t UART_ReadLine(char *buffer, uint16_t buffer_size)
{
    uint16_t i;

    if (buffer == 0 || buffer_size == 0 || !UART_LineReady)
    {
        return 0;
    }

    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
    for (i = 0; i < (buffer_size - 1) && UART_RxBuffer[i] != '\0'; i++)
    {
        buffer[i] = UART_RxBuffer[i];
    }
    buffer[i] = '\0';
    UART_LineReady = 0;
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    return 1;
}

void UART_SendChar(char ch)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, (uint16_t)(uint8_t)ch);
}

void UART_SendString(const char *text)
{
    while (*text != '\0')
    {
        UART_SendChar(*text++);
    }
}

void UART_SendUInt32(uint32_t value)
{
    char buffer[10];
    uint8_t length = 0;

    if (value == 0)
    {
        UART_SendChar('0');
        return;
    }

    while (value > 0)
    {
        buffer[length++] = (char)('0' + value % 10);
        value /= 10;
    }

    while (length > 0)
    {
        UART_SendChar(buffer[--length]);
    }
}

void UART_SendInt32(int32_t value)
{
    if (value < 0)
    {
        UART_SendChar('-');
        UART_SendUInt32((uint32_t)(-(value + 1)) + 1U);
    }
    else
    {
        UART_SendUInt32((uint32_t)value);
    }
}

void UART_SendFixed100(int32_t value_x100)
{
    uint32_t absolute_value;

    if (value_x100 < 0)
    {
        UART_SendChar('-');
        absolute_value = (uint32_t)(-(value_x100 + 1)) + 1U;
    }
    else
    {
        absolute_value = (uint32_t)value_x100;
    }

    UART_SendUInt32(absolute_value / 100U);
    UART_SendChar('.');
    UART_SendChar((char)('0' + (absolute_value / 10U) % 10U));
    UART_SendChar((char)('0' + absolute_value % 10U));
}
