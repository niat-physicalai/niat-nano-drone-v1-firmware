#ifndef PMW3901_H
#define PMW3901_H

#include <stdint.h>
#include <stdbool.h>

#define PMW3901_PRODUCT_ID          0x49U
#define PMW3901_INVERSE_PRODUCT_ID  0xB6U

#define PMW3901_REG_PRODUCT_ID      0x00U
#define PMW3901_REG_REVISION_ID     0x01U
#define PMW3901_REG_MOTION          0x02U
#define PMW3901_REG_DELTA_X_L       0x03U
#define PMW3901_REG_DELTA_X_H       0x04U
#define PMW3901_REG_DELTA_Y_L       0x05U
#define PMW3901_REG_DELTA_Y_H       0x06U
#define PMW3901_REG_SQUAL           0x07U
#define PMW3901_REG_MOTION_BURST    0x16U
#define PMW3901_REG_POWER_UP_RESET  0x3AU
#define PMW3901_REG_INVERSE_ID      0x5FU

typedef enum
{
    PMW3901_OK = 0,
    PMW3901_ERROR,
    PMW3901_INVALID_ARGUMENT,
    PMW3901_COMMUNICATION_ERROR,
    PMW3901_INVALID_DEVICE,
    PMW3901_NOT_INITIALIZED
} PMW3901_Status;

typedef struct
{
    uint8_t motion;
    uint8_t observation;

    int16_t delta_x;
    int16_t delta_y;

    uint8_t surface_quality;
    uint8_t raw_data_sum;
    uint8_t maximum_raw_data;
    uint8_t minimum_raw_data;

    uint16_t shutter;
} PMW3901_MotionData;

PMW3901_Status PMW3901_Init(void);

PMW3901_Status PMW3901_ReadRegister(
    uint8_t reg,
    uint8_t *value
);

PMW3901_Status PMW3901_ReadMotion(
    PMW3901_MotionData *motion
);

bool PMW3901_IsInitialized(void);

#endif
