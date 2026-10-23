#include "dps310_interface.h"
#include <stddef.h>

static I2C_HandleTypeDef *s_dps310_i2c = NULL;
static uint8_t s_dps310_address = 0x77U; /* provisional default; driver probes 0x77 and 0x76 */

HAL_StatusTypeDef DPS310_Interface_Init(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL)
    {
        s_dps310_i2c = NULL;
        return HAL_ERROR;
    }

    s_dps310_i2c = hi2c;
    return HAL_OK;
}

HAL_StatusTypeDef DPS310_Interface_SetAddress(uint8_t address_7bit)
{
    if ((address_7bit < 0x08U) || (address_7bit > 0x77U))
    {
        return HAL_ERROR;
    }

    s_dps310_address = address_7bit;
    return HAL_OK;
}

uint8_t DPS310_Interface_GetAddress(void)
{
    return s_dps310_address;
}

HAL_StatusTypeDef DPS310_Interface_ReadRegister(uint8_t reg, uint8_t *value)
{
    if ((s_dps310_i2c == NULL) || (value == NULL))
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(s_dps310_i2c,
                            (uint16_t)(s_dps310_address << 1U),
                            (uint16_t)reg,
                            I2C_MEMADD_SIZE_8BIT,
                            value,
                            1U,
                            DPS310_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef DPS310_Interface_ReadRegisters(uint8_t reg,
                                                 uint8_t *data,
                                                 uint16_t length)
{
    if ((s_dps310_i2c == NULL) || (data == NULL) || (length == 0U))
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(s_dps310_i2c,
                            (uint16_t)(s_dps310_address << 1U),
                            (uint16_t)reg,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            length,
                            DPS310_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef DPS310_Interface_WriteRegister(uint8_t reg, uint8_t value)
{
    if (s_dps310_i2c == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Write(s_dps310_i2c,
                             (uint16_t)(s_dps310_address << 1U),
                             (uint16_t)reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1U,
                             DPS310_I2C_TIMEOUT_MS);
}
