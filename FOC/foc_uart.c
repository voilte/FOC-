/**
 * @file foc_uart.c
 * @brief 非阻塞调试串口 TX（环形缓冲 + DMA/IT 发送）
 */

#include "foc_uart.h"
#include "usart.h"

#define FOC_UART_RING_SIZE  1024u
#define FOC_UART_TX_CHUNK   128u

static uint8_t s_ring[FOC_UART_RING_SIZE];
static volatile uint16_t s_head;
static volatile uint16_t s_tail;
static uint8_t s_tx_chunk[FOC_UART_TX_CHUNK];
static volatile uint8_t s_tx_busy;
static volatile uint16_t s_tx_len;

/** 从环形缓冲取一块数据启动 UART 中断发送 */
static void UART_KickTx(void)
{
    uint16_t n = 0;
    uint16_t tail;

    if (s_tx_busy)
    {
        return;
    }

    if (huart1.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    tail = s_tail;
    while ((n < FOC_UART_TX_CHUNK) && (tail != s_head))
    {
        s_tx_chunk[n++] = s_ring[tail];
        tail = (uint16_t)((tail + 1u) % FOC_UART_RING_SIZE);
    }

    if (n == 0u)
    {
        return;
    }

    s_tx_len = n;
    s_tx_busy = 1;

    if (HAL_UART_Transmit_IT(&huart1, s_tx_chunk, n) != HAL_OK)
    {
        s_tx_busy = 0;
        s_tx_len = 0;
    }
}

/** 初始化环形缓冲，USART1 中断优先级低于 TIM2/TIM4 */
void FOC_UART_Init(void)
{
    s_head = 0;
    s_tail = 0;
    s_tx_busy = 0;
    s_tx_len = 0;

    HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/** 线程安全写入环形缓冲（关中断保护），满则丢弃剩余字节 */
void FOC_UART_Write(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    uint32_t primask;

    if ((data == NULL) || (len == 0u))
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    for (i = 0; i < len; i++)
    {
        uint16_t head = s_head;
        uint16_t next = (uint16_t)((head + 1u) % FOC_UART_RING_SIZE);

        if (next == s_tail)
        {
            break;
        }

        s_ring[head] = data[i];
        s_head = next;
    }

    if (primask == 0u)
    {
        __enable_irq();
    }

    UART_KickTx();
}

/** HAL 发送完成：推进 tail，继续发送下一块 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        s_tail = (uint16_t)((s_tail + s_tx_len) % FOC_UART_RING_SIZE);
        s_tx_len = 0;
        s_tx_busy = 0;
        UART_KickTx();
    }
}

/** HAL 发送错误：中止并尝试恢复 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        s_tx_len = 0;
        s_tx_busy = 0;
        (void)HAL_UART_AbortTransmit(huart);
        UART_KickTx();
    }
}
