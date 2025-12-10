/**
 * @file modbus.h
 * @brief Modbus RTU protocol helpers for Waveshare 8CH Relay
 */

#ifndef __MODBUS_H__
#define __MODBUS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "rs485.h"

/* Modbus Configuration */
#define MODBUS_SLAVE_ADDR       0x01
#define MODBUS_RELAY_COUNT      8
#define MODBUS_TIMEOUT_MS       500

/* Modbus Function Codes */
#define MODBUS_FC_READ_COILS    0x01
#define MODBUS_FC_WRITE_COIL    0x05

/* Modbus Status */
typedef enum {
    MODBUS_OK = 0,
    MODBUS_ERROR_CRC,
    MODBUS_ERROR_TIMEOUT,
    MODBUS_ERROR_RESPONSE,
    MODBUS_ERROR_INVALID_PARAM
} Modbus_Status_t;

/* Function Prototypes */
uint16_t Modbus_CRC16(uint8_t *data, uint16_t length);
Modbus_Status_t Modbus_WriteCoil(uint8_t channel, uint8_t state);
Modbus_Status_t Modbus_ReadCoils(uint8_t *relayStates);

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_H__ */
