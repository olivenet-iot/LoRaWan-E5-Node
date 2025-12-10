/**
 * @file rs485.c
 * @brief RS485 Half-duplex driver implementation - Production grade
 */

#include "rs485.h"
#include <string.h>

/* Private variables */
static uint8_t rs485RxBuffer[RS485_RX_BUFFER_SIZE];

/**
 * @brief Flush UART RX buffer to clear stale data
 */
static void RS485_FlushRx(void)
{
    uint8_t dummy;
    /* Read any pending bytes with short timeout */
    while (HAL_UART_Receive(&huart1, &dummy, 1, 5) == HAL_OK);
}

/**
 * @brief Initialize RS485 interface
 * @note  UART1 and PA0 GPIO are already initialized in usart.c
 */
void RS485_Init(void)
{
    /* Ensure we start in RX mode */
    RS485_DE_RX_MODE();
    /* Flush any stale data */
    RS485_FlushRx();
}

/**
 * @brief Transmit data over RS485 with proper timing
 * @param data: Pointer to data buffer
 * @param length: Number of bytes to transmit
 * @return RS485_Status_t
 */
RS485_Status_t RS485_Transmit(uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status;

    /* Flush any pending RX data */
    RS485_FlushRx();

    /* Switch to TX mode */
    RS485_DE_TX_MODE();

    /* Wait for DE pin to settle (100us minimum) */
    HAL_Delay(1);

    /* Transmit data (blocking) */
    status = HAL_UART_Transmit(&huart1, data, length, RS485_TX_TIMEOUT_MS);

    if (status != HAL_OK)
    {
        RS485_DE_RX_MODE();
        return RS485_ERROR_TX;
    }

    /* Wait for transmission complete (shift register empty) */
    uint32_t tickstart = HAL_GetTick();
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET)
    {
        if ((HAL_GetTick() - tickstart) > RS485_TX_TIMEOUT_MS)
        {
            RS485_DE_RX_MODE();
            return RS485_ERROR_TX;
        }
    }

    /* Additional delay for last byte to fully transmit */
    HAL_Delay(2);

    /* Switch back to RX mode */
    RS485_DE_RX_MODE();

    /* Small delay for bus turnaround */
    HAL_Delay(1);

    return RS485_OK;
}

/**
 * @brief Receive data from RS485 with improved timeout handling
 * @param buffer: Pointer to receive buffer
 * @param length: Pointer to store received length
 * @param timeout_ms: Receive timeout in milliseconds
 * @return RS485_Status_t
 */
RS485_Status_t RS485_Receive(uint8_t *buffer, uint16_t *length, uint32_t timeout_ms)
{
    uint16_t rxCount = 0;
    uint32_t startTick = HAL_GetTick();
    uint32_t lastByteTick = startTick;

    /* Ensure we're in RX mode */
    RS485_DE_RX_MODE();

    /* Wait for first byte with full timeout */
    while ((HAL_GetTick() - startTick) < timeout_ms)
    {
        if (HAL_UART_Receive(&huart1, &buffer[rxCount], 1, 10) == HAL_OK)
        {
            rxCount++;
            lastByteTick = HAL_GetTick();
            break;
        }
    }

    if (rxCount == 0)
    {
        *length = 0;
        return RS485_ERROR_TIMEOUT;
    }

    /* Continue receiving with inter-byte timeout (50ms for Modbus) */
    while (rxCount < RS485_RX_BUFFER_SIZE)
    {
        if (HAL_UART_Receive(&huart1, &buffer[rxCount], 1, 10) == HAL_OK)
        {
            rxCount++;
            lastByteTick = HAL_GetTick();
        }
        else if ((HAL_GetTick() - lastByteTick) > 50)
        {
            /* No byte received for 50ms - frame complete */
            break;
        }
    }

    *length = rxCount;
    return RS485_OK;
}

/**
 * @brief Transmit command and receive response (typical Modbus transaction)
 * @param txData: Command to send
 * @param txLength: Command length
 * @param rxBuffer: Buffer for response
 * @param rxLength: Pointer to store response length
 * @param rxTimeout_ms: Response timeout
 * @return RS485_Status_t
 */
RS485_Status_t RS485_TransmitReceive(uint8_t *txData, uint16_t txLength,
                                      uint8_t *rxBuffer, uint16_t *rxLength,
                                      uint32_t rxTimeout_ms)
{
    RS485_Status_t status;

    /* Transmit command */
    status = RS485_Transmit(txData, txLength);
    if (status != RS485_OK)
    {
        return status;
    }

    /* Receive response */
    status = RS485_Receive(rxBuffer, rxLength, rxTimeout_ms);

    return status;
}
