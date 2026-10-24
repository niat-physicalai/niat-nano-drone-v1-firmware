#ifndef OPT3101_INTERFACE_H
#define OPT3101_INTERFACE_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Low-level STM32 HAL transport for OPT3101's 8-bit register address and
 * 24-bit register data format. Addresses passed to this interface are 7-bit.
 */
HAL_StatusTypeDef OPT3101_Interface_WriteRegister(I2C_HandleTypeDef *hi2c,
                                                  uint8_t address_7bit,
                                                  uint8_t reg,
                                                  uint32_t value,
                                                  uint32_t timeout_ms);

HAL_StatusTypeDef OPT3101_Interface_ReadRegister(I2C_HandleTypeDef *hi2c,
                                                 uint8_t address_7bit,
                                                 uint8_t reg,
                                                 uint32_t *value,
                                                 uint32_t timeout_ms);

/* OPT3101's reset sequence is a special two-byte transfer (reg 0, value 1).
 * Do not replace this with a generic 24-bit register write. */
HAL_StatusTypeDef OPT3101_Interface_SoftwareReset(I2C_HandleTypeDef *hi2c,
                                                  uint8_t address_7bit,
                                                  uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
#endif
