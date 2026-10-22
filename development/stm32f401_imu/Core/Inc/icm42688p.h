#ifndef ICM42688P_H
#define ICM42688P_H

#include "main.h"
#include <stdint.h>

/* ICM-42688-P register addresses: User Bank 0 */
#define ICM42688P_REG_DEVICE_CONFIG       0x11U
#define ICM42688P_REG_TEMP_DATA1          0x1DU
#define ICM42688P_REG_ACCEL_DATA_X1       0x1FU
#define ICM42688P_REG_GYRO_DATA_X1        0x25U
#define ICM42688P_REG_PWR_MGMT0           0x4EU
#define ICM42688P_REG_GYRO_CONFIG0        0x4FU
#define ICM42688P_REG_ACCEL_CONFIG0       0x50U
#define ICM42688P_REG_WHO_AM_I             0x75U

#define ICM42688P_WHO_AM_I_VALUE           0x47U
#define ICM42688P_DEVICE_RESET             0x01U

/* Initial sensor configuration */
#define ICM42688P_ACCEL_FS_SEL_16G          0x00U
#define ICM42688P_GYRO_FS_SEL_2000DPS       0x00U

#define ICM42688P_ACCEL_ODR_1KHZ            0x06U
#define ICM42688P_GYRO_ODR_1KHZ             0x06U

/* Physical conversion constants */
#define ICM42688P_GRAVITY_MS2               9.80665f
#define ICM42688P_DEG_TO_RAD                0.01745329252f

typedef enum
{
    ICM42688P_OK = 0,
    ICM42688P_ERROR,
    ICM42688P_TIMEOUT,
    ICM42688P_INVALID_DEVICE,
    ICM42688P_INVALID_ARGUMENT

} icm42688p_status_t;

/* Low-level register access */
icm42688p_status_t ICM42688P_ReadRegister(
    uint8_t reg,
    uint8_t *data
);

icm42688p_status_t ICM42688P_WriteRegister(
    uint8_t reg,
    uint8_t data
);

icm42688p_status_t ICM42688P_ReadRegisters(
    uint8_t reg,
    uint8_t *data,
    uint16_t length
);

/* Device identification and initialization */
icm42688p_status_t ICM42688P_ReadWhoAmI(
    uint8_t *device_id
);

icm42688p_status_t ICM42688P_Init(void);

/* Raw sensor data */
icm42688p_status_t ICM42688P_ReadRawData(
    int16_t accel[3],
    int16_t gyro[3],
    int16_t *temperature
);

/* Conversion into SI units */
void ICM42688P_ConvertToPhysicalUnits(
    const int16_t accel_raw[3],
    const int16_t gyro_raw[3],
    float accel_ms2[3],
    float gyro_rads[3]
);

#endif /* ICM42688P_H */
