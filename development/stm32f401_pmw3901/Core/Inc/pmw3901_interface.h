#ifndef PMW3901_INTERFACE_H
#define PMW3901_INTERFACE_H

#include "main.h"
#include <stdint.h>

HAL_StatusTypeDef PMW3901_Interface_ReadRegister(
    uint8_t reg,
    uint8_t *value
);

HAL_StatusTypeDef PMW3901_Interface_WriteRegister(
    uint8_t reg,
    uint8_t value
);

HAL_StatusTypeDef PMW3901_Interface_ReadBurst(
    uint8_t reg,
    uint8_t *data,
    uint16_t length
);

void PMW3901_Interface_Reset(void);

void PMW3901_Interface_DelayUs(uint32_t delay_us);

#endif
