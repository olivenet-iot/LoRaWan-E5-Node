/**
 * @file rs485.h
 * @brief RS485 Half-duplex driver for Modbus RTU communication
 */

#ifndef __RS485_H__
#define __RS485_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"

/* RS485 Configuration */
#define RS485_TX_TIMEOUT_MS     100
#define RS485_RX_TIMEOUT_MS     500
#define RS485_RX_BUFFER_SIZE    256

/* RS485 Status */
typedef enum {
    RS485_OK = 0,
    RS485_ERROR_TIMEOUT,
    RS485_ERROR_TX,
    RS485_ERROR_RX
} RS485_Status_t;

/* Function Prototypes */
void RS485_Init(void);
RS485_Status_t RS485_Transmit(uint8_t *data, uint16_t length);
RS485_Status_t RS485_Receive(uint8_t *buffer, uint16_t *length, uint32_t timeout_ms);
RS485_Status_t RS485_TransmitReceive(uint8_t *txData, uint16_t txLength,
                                      uint8_t *rxBuffer, uint16_t *rxLength,
                                      uint32_t rxTimeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __RS485_H__ */
