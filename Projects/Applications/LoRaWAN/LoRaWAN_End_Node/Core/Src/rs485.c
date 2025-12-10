/**
 * @file rs485.c
 * @brief RS485 Half-duplex driver implementation
 */

#include "rs485.h"
#include <string.h>

/* Private variables */
static uint8_t rs485RxBuffer[RS485_RX_BUFFER_SIZE];

/**
 * @brief Initialize RS485 interface
 * @note  UART1 and PA8 GPIO are already initialized in usart.c
 */
void RS485_Init(void)
{
    /* Ensure we start in RX mode */
    RS485_DE_RX_MODE();
}

/**
 * @brief Transmit data over RS485
 * @param data: Pointer to data buffer
 * @param length: Number of bytes to transmit
 * @return RS485_Status_t
 */
RS485_Status_t RS485_Transmit(uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status;

    /* Switch to TX mode */
    RS485_DE_TX_MODE();

    /* Small delay for DE pin settling (10us equivalent) */
    for (volatile int i = 0; i < 100; i++);

    /* Transmit data (blocking) */
    status = HAL_UART_Transmit(&huart1, data, length, RS485_TX_TIMEOUT_MS);

    /* Wait for transmission complete (shift register empty) */
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET);

    /* Small delay before switching back to RX */
    for (volatile int i = 0; i < 100; i++);

    /* Switch back to RX mode */
    RS485_DE_RX_MODE();

    return (status == HAL_OK) ? RS485_OK : RS485_ERROR_TX;
}

/**
 * @brief Receive data from RS485
 * @param buffer: Pointer to receive buffer
 * @param length: Pointer to store received length
 * @param timeout_ms: Receive timeout in milliseconds
 * @return RS485_Status_t
 */
RS485_Status_t RS485_Receive(uint8_t *buffer, uint16_t *length, uint32_t timeout_ms)
{
    HAL_StatusTypeDef status;
    uint16_t rxCount = 0;
    uint32_t startTick = HAL_GetTick();

    /* Ensure we're in RX mode */
    RS485_DE_RX_MODE();

    /* Receive bytes until timeout or buffer full */
    while ((HAL_GetTick() - startTick) < timeout_ms && rxCount < RS485_RX_BUFFER_SIZE)
    {
        status = HAL_UART_Receive(&huart1, &buffer[rxCount], 1, 10);
        if (status == HAL_OK)
        {
            rxCount++;
            startTick = HAL_GetTick();  /* Reset timeout on each byte received */
        }
    }

    *length = rxCount;
    return (rxCount > 0) ? RS485_OK : RS485_ERROR_TIMEOUT;
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

    /* Small inter-frame delay (3.5 char times at 9600 = ~4ms) */
    HAL_Delay(5);

    /* Receive response */
    status = RS485_Receive(rxBuffer, rxLength, rxTimeout_ms);

    return status;
}
