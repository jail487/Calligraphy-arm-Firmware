#include <uxr/client/transport.h>

#include <rmw_microxrcedds_c/config.h>

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <limits.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

/**
 * @brief Select RX transport mode for micro-ROS:
 *        0 = Fixed-size IRQ ring buffer (USART2 RXNE interrupt producer, CommTask consumer) [DEFAULT]
 *        1 = Circular DMA RX mode (requires working DMA1 Stream5 request routing)
 *        2 = Bounded RXNE polling fallback in CommTask context
 */
#if defined(MICROROS_USE_RX_DMA) && (MICROROS_USE_RX_DMA == 1)
  #define MICROROS_RX_MODE 1
#elif !defined(MICROROS_RX_MODE)
  #define MICROROS_RX_MODE 0
#endif

// --- micro-ROS Transports ---
#define UART_DMA_BUFFER_SIZE  2048U
#define UART_RING_BUFFER_SIZE 2048U
#define UART_TX_TIMEOUT_MS    100U
#define TRANSPORT_ERR_IO      1U
#define USART_ISR_MAX_ITER    16U

typedef struct {
    volatile uint32_t tx_bytes;
    volatile uint32_t tx_frames;
    volatile uint32_t tx_errors;
    volatile uint32_t rx_bytes;
    volatile uint32_t rx_line_errors;
    volatile uint32_t rx_overflow_errors;
    volatile uint32_t open_failures;
} DmaTransportStats_t;

volatile DmaTransportStats_t g_dma_transport_stats = {0};

#if (MICROROS_RX_MODE == 1)
static uint8_t dma_buffer[UART_DMA_BUFFER_SIZE];
static size_t dma_head = 0, dma_tail = 0;
#elif (MICROROS_RX_MODE == 0)
static uint8_t rx_ring_buf[UART_RING_BUFFER_SIZE];
static volatile uint32_t rx_ring_head = 0U;
static volatile uint32_t rx_ring_tail = 0U;
#endif

/**
 * @brief USART RX IRQ hook for micro-ROS transport (ISR context).
 *        Consumes RXNE/error bytes into lock-free ring buffer before HAL_UART_IRQHandler.
 */
void dma_transport_usart_irq_hook(UART_HandleTypeDef *huart)
{
#if (MICROROS_RX_MODE == 0)
    if ((huart == NULL) || (huart->Instance == NULL)) {
        return;
    }

    USART_TypeDef *uart = huart->Instance;
    uint32_t sr = uart->SR;
    uint32_t cr1 = uart->CR1;
    uint32_t cr3 = uart->CR3;

    bool rxne_it_enabled = ((cr1 & USART_CR1_RXNEIE) != 0U);
    bool err_it_enabled  = ((cr3 & USART_CR3_EIE) != 0U);
    bool pe_it_enabled   = ((cr1 & USART_CR1_PEIE) != 0U);

    if (!rxne_it_enabled && !err_it_enabled && !pe_it_enabled) {
        return;
    }

    uint32_t iter = 0U;
    while (((sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U) &&
           (iter < USART_ISR_MAX_ITER)) {
        iter++;
        bool has_line_err = ((sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U);

        /* Reading SR followed by DR clears error flags (ORE, NE, FE, PE) and RXNE on STM32F4 */
        uint8_t dr = (uint8_t)(uart->DR & 0xFFU);

        if (has_line_err) {
            g_dma_transport_stats.rx_line_errors++;
        }

        if ((sr & USART_SR_RXNE) != 0U) {
            uint32_t head = rx_ring_head;
            uint32_t tail = rx_ring_tail;
            uint32_t next_head = (head + 1U) % UART_RING_BUFFER_SIZE;

            if (next_head == tail) {
                /* Ring buffer full: overflow accounting, do not overwrite */
                g_dma_transport_stats.rx_overflow_errors++;
            } else {
                rx_ring_buf[head] = dr;
                rx_ring_head = next_head;
            }
        }

        sr = uart->SR;
    }
#else
    (void)huart;
#endif
}

bool cubemx_transport_open(struct uxrCustomTransport * transport){
    if ((transport == NULL) || (transport->args == NULL)) {
        g_dma_transport_stats.open_failures++;
        return false;
    }

    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;

#if (MICROROS_RX_MODE == 1)
    if (uart->hdmarx == NULL) {
        g_dma_transport_stats.open_failures++;
        return false;
    }

    if (uart->RxState != HAL_UART_STATE_READY) {
        (void)HAL_UART_DMAStop(uart);
    }

    dma_head = 0;
    dma_tail = 0;
    __HAL_UART_CLEAR_OREFLAG(uart);
    __HAL_UART_CLEAR_NEFLAG(uart);
    __HAL_UART_CLEAR_FEFLAG(uart);
    __HAL_UART_CLEAR_PEFLAG(uart);

    HAL_StatusTypeDef status;
    taskENTER_CRITICAL();
    status = HAL_UART_Receive_DMA(uart, dma_buffer, UART_DMA_BUFFER_SIZE);
    if (status == HAL_OK) {
        __HAL_UART_DISABLE_IT(uart, UART_IT_ERR);
        __HAL_UART_DISABLE_IT(uart, UART_IT_PE);
    }
    taskEXIT_CRITICAL();

    if (status != HAL_OK) {
        g_dma_transport_stats.open_failures++;
        return false;
    }
#elif (MICROROS_RX_MODE == 0)
    /* Disable DMA RX on USART peripheral if active */
    if ((uart->hdmarx != NULL) && (uart->RxState != HAL_UART_STATE_READY)) {
        (void)HAL_UART_DMAStop(uart);
    } else {
        CLEAR_BIT(uart->Instance->CR3, USART_CR3_DMAR);
    }

    /* Initialize ring indices */
    rx_ring_head = 0U;
    rx_ring_tail = 0U;

    /* Clear pending error flags */
    __HAL_UART_CLEAR_OREFLAG(uart);
    __HAL_UART_CLEAR_NEFLAG(uart);
    __HAL_UART_CLEAR_FEFLAG(uart);
    __HAL_UART_CLEAR_PEFLAG(uart);
    volatile uint32_t dummy_sr = uart->Instance->SR;
    volatile uint32_t dummy_dr = uart->Instance->DR;
    (void)dummy_sr;
    (void)dummy_dr;

    /* Enable RXNE and error interrupts for IRQ ring producer */
    __HAL_UART_ENABLE_IT(uart, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(uart, UART_IT_ERR);
    __HAL_UART_ENABLE_IT(uart, UART_IT_PE);

    /* Ensure Receiver is enabled */
    SET_BIT(uart->Instance->CR1, USART_CR1_RE);
#else
    /* Disable DMA RX request on USART peripheral if active */
    if (uart->hdmarx != NULL && uart->RxState != HAL_UART_STATE_READY) {
        (void)HAL_UART_DMAStop(uart);
    } else {
        CLEAR_BIT(uart->Instance->CR3, USART_CR3_DMAR);
    }

    /* Disable RX interrupts to ensure direct RXNE polling in CommTask */
    __HAL_UART_DISABLE_IT(uart, UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(uart, UART_IT_PE);
    __HAL_UART_DISABLE_IT(uart, UART_IT_ERR);

    /* Clear pending error flags */
    __HAL_UART_CLEAR_OREFLAG(uart);
    __HAL_UART_CLEAR_NEFLAG(uart);
    __HAL_UART_CLEAR_FEFLAG(uart);
    __HAL_UART_CLEAR_PEFLAG(uart);

    /* Ensure Receiver is enabled */
    SET_BIT(uart->Instance->CR1, USART_CR1_RE);
#endif

    return true;
}

bool cubemx_transport_close(struct uxrCustomTransport * transport){
    if ((transport == NULL) || (transport->args == NULL)) {
        return false;
    }

    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
#if (MICROROS_RX_MODE == 1)
    return HAL_UART_DMAStop(uart) == HAL_OK;
#elif (MICROROS_RX_MODE == 0)
    __HAL_UART_DISABLE_IT(uart, UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(uart, UART_IT_ERR);
    __HAL_UART_DISABLE_IT(uart, UART_IT_PE);
    CLEAR_BIT(uart->Instance->CR3, USART_CR3_DMAR);
    return true;
#else
    __HAL_UART_DISABLE_IT(uart, UART_IT_RXNE);
    CLEAR_BIT(uart->Instance->CR3, USART_CR3_DMAR);
    return true;
#endif
}

size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err){
    if (err != NULL) {
        *err = 0U;
    }

    if ((transport == NULL) || (transport->args == NULL) ||
        (buf == NULL) || (len == 0U) || (len > UINT16_MAX)) {
        if (err != NULL) {
            *err = TRANSPORT_ERR_IO;
        }
        g_dma_transport_stats.tx_errors++;
        return 0U;
    }

    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;

    TickType_t timeout_ticks = pdMS_TO_TICKS(UART_TX_TIMEOUT_MS);
    if (timeout_ticks == 0U) {
        timeout_ticks = 1U;
    }
    TickType_t start = xTaskGetTickCount();

    /* Wait for previous transmit to complete if busy (bounded non-blocking) */
    while (uart->gState != HAL_UART_STATE_READY) {
        if ((TickType_t)(xTaskGetTickCount() - start) >= timeout_ticks) {
            (void)HAL_UART_AbortTransmit(uart);
            if (err != NULL) {
                *err = TRANSPORT_ERR_IO;
            }
            g_dma_transport_stats.tx_errors++;
            return 0U;
        }
        vTaskDelay(1U);
    }

    if (HAL_UART_Transmit_DMA(uart, buf, (uint16_t)len) != HAL_OK) {
        (void)HAL_UART_AbortTransmit(uart);
        if (err != NULL) {
            *err = TRANSPORT_ERR_IO;
        }
        g_dma_transport_stats.tx_errors++;
        return 0U;
    }

    start = xTaskGetTickCount();
    while (uart->gState != HAL_UART_STATE_READY) {
        if ((TickType_t)(xTaskGetTickCount() - start) >= timeout_ticks) {
            /* Abort TX only. HAL_UART_DMAStop() would also destroy circular RX if active. */
            (void)HAL_UART_AbortTransmit(uart);
            if (err != NULL) {
                *err = TRANSPORT_ERR_IO;
            }
            g_dma_transport_stats.tx_errors++;
            return 0U;
        }
        vTaskDelay(1U);
    }

    g_dma_transport_stats.tx_frames++;
    g_dma_transport_stats.tx_bytes += (uint32_t)len;

    return len;
}

size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    if (err != NULL) {
        *err = 0U;
    }

    if ((transport == NULL) || (transport->args == NULL) ||
        (buf == NULL) || (len == 0U)) {
        if (err != NULL) {
            *err = TRANSPORT_ERR_IO;
        }
        return 0U;
    }

    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;

#if (MICROROS_RX_MODE == 1)
    if (uart->hdmarx == NULL) {
        if (err != NULL) {
            *err = TRANSPORT_ERR_IO;
        }
        return 0U;
    }

    /* Clear any pending UART line errors */
    if (__HAL_UART_GET_FLAG(uart, UART_FLAG_ORE) || 
        __HAL_UART_GET_FLAG(uart, UART_FLAG_NE) || 
        __HAL_UART_GET_FLAG(uart, UART_FLAG_FE) ||
        __HAL_UART_GET_FLAG(uart, UART_FLAG_PE)) {
        g_dma_transport_stats.rx_line_errors++;
        __HAL_UART_CLEAR_OREFLAG(uart);
        __HAL_UART_CLEAR_NEFLAG(uart);
        __HAL_UART_CLEAR_FEFLAG(uart);
        __HAL_UART_CLEAR_PEFLAG(uart);
    }

    /* Auto-restart DMA RX if not active */
    if ((uart->RxState != HAL_UART_STATE_BUSY_RX) ||
        ((uart->Instance->CR3 & USART_CR3_DMAR) == 0U)) {
        taskENTER_CRITICAL();
        dma_head = 0;
        dma_tail = 0;
        (void)HAL_UART_Receive_DMA(uart, dma_buffer, UART_DMA_BUFFER_SIZE);
        __HAL_UART_DISABLE_IT(uart, UART_IT_ERR);
        __HAL_UART_DISABLE_IT(uart, UART_IT_PE);
        taskEXIT_CRITICAL();
    }

    TickType_t timeout_ticks =
        (timeout > 0) ? pdMS_TO_TICKS((uint32_t)timeout) : 0U;
    if ((timeout > 0) && (timeout_ticks == 0U)) {
        timeout_ticks = 1U;
    }
    const TickType_t start = xTaskGetTickCount();
    size_t wrote = 0;

    for (;;) {
        const size_t dma_tail_loc =
            UART_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(uart->hdmarx);

        while ((dma_head != dma_tail_loc) && (wrote < len)){
            buf[wrote++] = dma_buffer[dma_head];
            dma_head = (dma_head + 1U) % UART_DMA_BUFFER_SIZE;
        }

        if (wrote != 0U) {
            g_dma_transport_stats.rx_bytes += (uint32_t)wrote;
            return wrote;
        }

        if ((timeout_ticks == 0U) ||
            ((TickType_t)(xTaskGetTickCount() - start) >= timeout_ticks)) {
            return 0U;
        }
        vTaskDelay(1U);
    }
#elif (MICROROS_RX_MODE == 0)
    (void)uart;
    TickType_t timeout_ticks =
        (timeout > 0) ? pdMS_TO_TICKS((uint32_t)timeout) : 0U;
    if ((timeout > 0) && (timeout_ticks == 0U)) {
        timeout_ticks = 1U;
    }
    const TickType_t start = xTaskGetTickCount();
    size_t wrote = 0U;

    for (;;) {
        uint32_t head = rx_ring_head;
        uint32_t tail = rx_ring_tail;

        while ((tail != head) && (wrote < len)) {
            buf[wrote++] = rx_ring_buf[tail];
            tail = (tail + 1U) % UART_RING_BUFFER_SIZE;
        }
        rx_ring_tail = tail;

        if (wrote != 0U) {
            g_dma_transport_stats.rx_bytes += (uint32_t)wrote;
            return wrote;
        }

        if ((timeout_ticks == 0U) ||
            ((TickType_t)(xTaskGetTickCount() - start) >= timeout_ticks)) {
            return 0U;
        }
        vTaskDelay(1U);
    }
#else
    TickType_t timeout_ticks =
        (timeout > 0) ? pdMS_TO_TICKS((uint32_t)timeout) : 0U;
    if ((timeout > 0) && (timeout_ticks == 0U)) {
        timeout_ticks = 1U;
    }
    const TickType_t start = xTaskGetTickCount();
    TickType_t last_byte_time = start;
    size_t wrote = 0U;

    for (;;) {
        uint32_t sr = uart->Instance->SR;

        /* Check and clear UART line errors (ORE, NE, FE, PE) */
        if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
            g_dma_transport_stats.rx_line_errors++;
            /* Reading SR then DR clears ORE, NE, FE, PE on STM32F4 while retaining DR byte if RXNE was set */
            uint8_t dr = (uint8_t)(uart->Instance->DR & 0xFFU);
            if ((sr & USART_SR_RXNE) && (wrote < len)) {
                buf[wrote++] = dr;
                last_byte_time = xTaskGetTickCount();
            }
        } else if ((sr & USART_SR_RXNE) && (wrote < len)) {
            buf[wrote++] = (uint8_t)(uart->Instance->DR & 0xFFU);
            last_byte_time = xTaskGetTickCount();
        }

        /* Drain all back-to-back bytes arriving in DR during a burst without yielding */
        while ((wrote < len) && (__HAL_UART_GET_FLAG(uart, UART_FLAG_RXNE))) {
            buf[wrote++] = (uint8_t)(uart->Instance->DR & 0xFFU);
            last_byte_time = xTaskGetTickCount();
        }

        if (wrote >= len) {
            g_dma_transport_stats.rx_bytes += (uint32_t)wrote;
            return wrote;
        }

        const TickType_t now = xTaskGetTickCount();

        if (wrote > 0U) {
            if (((TickType_t)(now - last_byte_time) >= 1U) ||
                ((TickType_t)(now - start) >= timeout_ticks)) {
                g_dma_transport_stats.rx_bytes += (uint32_t)wrote;
                return wrote;
            }
        } else {
            if ((timeout_ticks == 0U) ||
                ((TickType_t)(now - start) >= timeout_ticks)) {
                return 0U;
            }
            vTaskDelay(1U);
        }
    }
#endif
}

#endif //RMW_UXRCE_TRANSPORT_CUSTOM

