/**
 * @file modbus.c
 * @brief Modbus RTU protocol implementation - Production grade with retry
 */

#include "modbus.h"

/* Retry configuration */
#define MODBUS_RETRY_COUNT      3
#define MODBUS_RETRY_DELAY_MS   50

/**
 * @brief Calculate Modbus CRC16
 * @param data: Data buffer
 * @param length: Data length
 * @return CRC16 value (little-endian format for Modbus)
 */
uint16_t Modbus_CRC16(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Write single coil (control one relay) with retry mechanism
 * @param channel: Relay channel (1-8)
 * @param state: 0=OFF, 1=ON
 * @return Modbus_Status_t
 */
Modbus_Status_t Modbus_WriteCoil(uint8_t channel, uint8_t state)
{
    if (channel < 1 || channel > MODBUS_RELAY_COUNT)
    {
        return MODBUS_ERROR_INVALID_PARAM;
    }

    uint8_t txBuffer[8];
    uint8_t rxBuffer[8];
    uint16_t rxLen = 0;
    uint16_t crc;

    /* Build Modbus frame */
    txBuffer[0] = MODBUS_SLAVE_ADDR;           /* Slave address */
    txBuffer[1] = MODBUS_FC_WRITE_COIL;        /* Function code 05 */
    txBuffer[2] = 0x00;                         /* Address high byte */
    txBuffer[3] = channel - 1;                  /* Address low byte (0-indexed) */
    txBuffer[4] = state ? 0xFF : 0x00;         /* Value high byte */
    txBuffer[5] = 0x00;                         /* Value low byte */

    /* Calculate and append CRC */
    crc = Modbus_CRC16(txBuffer, 6);
    txBuffer[6] = crc & 0xFF;                  /* CRC low byte */
    txBuffer[7] = (crc >> 8) & 0xFF;           /* CRC high byte */

    /* Retry loop */
    for (uint8_t retry = 0; retry < MODBUS_RETRY_COUNT; retry++)
    {
        RS485_Status_t rs485Status = RS485_TransmitReceive(txBuffer, 8, rxBuffer, &rxLen, MODBUS_TIMEOUT_MS);

        if (rs485Status != RS485_OK || rxLen < 8)
        {
            /* Communication error, retry after delay */
            HAL_Delay(MODBUS_RETRY_DELAY_MS);
            continue;
        }

        /* Verify response CRC */
        crc = Modbus_CRC16(rxBuffer, 6);
        if (rxBuffer[6] != (crc & 0xFF) || rxBuffer[7] != ((crc >> 8) & 0xFF))
        {
            /* CRC error, retry */
            HAL_Delay(MODBUS_RETRY_DELAY_MS);
            continue;
        }

        /* Verify echo (write coil echoes the command) */
        if (rxBuffer[0] == txBuffer[0] && rxBuffer[1] == txBuffer[1] &&
            rxBuffer[2] == txBuffer[2] && rxBuffer[3] == txBuffer[3])
        {
            return MODBUS_OK;
        }

        /* Response mismatch, retry */
        HAL_Delay(MODBUS_RETRY_DELAY_MS);
    }

    return MODBUS_ERROR_TIMEOUT;
}

/**
 * @brief Read all coil states (all 8 relays) with retry mechanism
 * @param relayStates: Pointer to store relay states as bitmask
 * @return Modbus_Status_t
 */
Modbus_Status_t Modbus_ReadCoils(uint8_t *relayStates)
{
    uint8_t txBuffer[8];
    uint8_t rxBuffer[8];
    uint16_t rxLen = 0;
    uint16_t crc;

    /* Build Modbus frame: Read 8 coils starting at address 0 */
    txBuffer[0] = MODBUS_SLAVE_ADDR;           /* Slave address */
    txBuffer[1] = MODBUS_FC_READ_COILS;        /* Function code 01 */
    txBuffer[2] = 0x00;                         /* Start address high */
    txBuffer[3] = 0x00;                         /* Start address low */
    txBuffer[4] = 0x00;                         /* Quantity high */
    txBuffer[5] = 0x08;                         /* Quantity low (8 coils) */

    /* Calculate and append CRC */
    crc = Modbus_CRC16(txBuffer, 6);
    txBuffer[6] = crc & 0xFF;
    txBuffer[7] = (crc >> 8) & 0xFF;

    /* Retry loop */
    for (uint8_t retry = 0; retry < MODBUS_RETRY_COUNT; retry++)
    {
        RS485_Status_t rs485Status = RS485_TransmitReceive(txBuffer, 8, rxBuffer, &rxLen, MODBUS_TIMEOUT_MS);

        if (rs485Status != RS485_OK || rxLen < 5)
        {
            /* Communication error, retry after delay */
            HAL_Delay(MODBUS_RETRY_DELAY_MS);
            continue;
        }

        /* Response format: [addr][fc][byte_count][data][crc_lo][crc_hi] */
        /* Verify response CRC */
        crc = Modbus_CRC16(rxBuffer, rxLen - 2);
        if (rxBuffer[rxLen-2] != (crc & 0xFF) || rxBuffer[rxLen-1] != ((crc >> 8) & 0xFF))
        {
            /* CRC error, retry */
            HAL_Delay(MODBUS_RETRY_DELAY_MS);
            continue;
        }

        /* Verify response header */
        if (rxBuffer[0] == MODBUS_SLAVE_ADDR && rxBuffer[1] == MODBUS_FC_READ_COILS)
        {
            /* Extract relay states (byte 3 contains the coil data) */
            *relayStates = rxBuffer[3];
            return MODBUS_OK;
        }

        /* Response mismatch, retry */
        HAL_Delay(MODBUS_RETRY_DELAY_MS);
    }

    return MODBUS_ERROR_TIMEOUT;
}
