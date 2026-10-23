#ifndef DPS310_INTERFACE_H
#define DPS310_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/*
 * STM32 HAL transport layer for the DPS310.
 *
 * I2C addresses accepted by the sensor are 0x76 and 0x77 (7-bit form).
 * HAL functions expect the address left-shifted by one; this layer handles
 * that conversion internally.
 */
#define DPS310_I2C_TIMEOUT_MS 100U

HAL_StatusTypeDef DPS310_Interface_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef DPS310_Interface_SetAddress(uint8_t address_7bit);
uint8_t DPS310_Interface_GetAddress(void);

HAL_StatusTypeDef DPS310_Interface_ReadRegister(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef DPS310_Interface_ReadRegisters(uint8_t reg,
                                                 uint8_t *data,
                                                 uint16_t length);
HAL_StatusTypeDef DPS310_Interface_WriteRegister(uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* DPS310_INTERFACE_H */
