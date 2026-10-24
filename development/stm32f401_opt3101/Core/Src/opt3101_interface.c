#include "opt3101_interface.h"
#include <stddef.h>

#define OPT3101_REGISTER_BYTES 3U

static uint16_t opt3101_hal_address(uint8_t address_7bit)
{
    return (uint16_t)((uint16_t)address_7bit << 1U);
}

HAL_StatusTypeDef OPT3101_Interface_WriteRegister(I2C_HandleTypeDef *hi2c,
                                                  uint8_t address_7bit,
                                                  uint8_t reg,
                                                  uint32_t value,
                                                  uint32_t timeout_ms)
{
    uint8_t tx[1U + OPT3101_REGISTER_BYTES];

    if ((hi2c == NULL) || (address_7bit > 0x7FU))
    {
        return HAL_ERROR;
    }

    tx[0] = reg;
    tx[1] = (uint8_t)(value & 0xFFU);         /* Register bits 7:0 */
    tx[2] = (uint8_t)((value >> 8U) & 0xFFU); /* Register bits 15:8 */
    tx[3] = (uint8_t)((value >> 16U) & 0xFFU);/* Register bits 23:16 */

    return HAL_I2C_Master_Transmit(hi2c,
                                   opt3101_hal_address(address_7bit),
                                   tx,
                                   (uint16_t)sizeof(tx),
                                   timeout_ms);
}

HAL_StatusTypeDef OPT3101_Interface_ReadRegister(I2C_HandleTypeDef *hi2c,
                                                 uint8_t address_7bit,
                                                 uint8_t reg,
                                                 uint32_t *value,
                                                 uint32_t timeout_ms)
{
    uint8_t rx[OPT3101_REGISTER_BYTES] = {0U, 0U, 0U};
    HAL_StatusTypeDef result;

    if ((hi2c == NULL) || (value == NULL) || (address_7bit > 0x7FU))
    {
        return HAL_ERROR;
    }

    /* HAL_I2C_Mem_Read sends the register pointer, issues a repeated START,
     * and reads three bytes. OPT3101 sends the least-significant byte first. */
    result = HAL_I2C_Mem_Read(hi2c,
                              opt3101_hal_address(address_7bit),
                              (uint16_t)reg,
                              I2C_MEMADD_SIZE_8BIT,
                              rx,
                              (uint16_t)sizeof(rx),
                              timeout_ms);
    if (result != HAL_OK)
    {
        return result;
    }

    *value = ((uint32_t)rx[0]) |
             ((uint32_t)rx[1] << 8U) |
             ((uint32_t)rx[2] << 16U);
    return HAL_OK;
}

HAL_StatusTypeDef OPT3101_Interface_SoftwareReset(I2C_HandleTypeDef *hi2c,
                                                  uint8_t address_7bit,
                                                  uint32_t timeout_ms)
{
    uint8_t tx[2] = {0x00U, 0x01U};

    if ((hi2c == NULL) || (address_7bit > 0x7FU))
    {
        return HAL_ERROR;
    }

    /* OPT3101 stops acknowledging after the first register-value byte during
     * software reset. The TI-supported reference driver therefore sends only
     * one data byte instead of the normal three-byte register payload. */
    return HAL_I2C_Master_Transmit(hi2c,
                                  opt3101_hal_address(address_7bit),
                                  tx,
                                  (uint16_t)sizeof(tx),
                                  timeout_ms);
}
