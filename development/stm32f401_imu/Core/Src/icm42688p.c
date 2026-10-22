#include "icm42688p.h"
#include <stddef.h>

extern SPI_HandleTypeDef hspi1;

#define ICM42688P_SPI_TIMEOUT_MS       100U

#define ICM42688P_CS_LOW() \
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET)

#define ICM42688P_CS_HIGH() \
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET)

#define ICM42688P_ACCEL_SENSITIVITY_LSB_G  2048.0f
#define ICM42688P_GYRO_SENSITIVITY_LSB_DPS 16.4f

static icm42688p_status_t ICM42688P_MapHALStatus(
    HAL_StatusTypeDef status)
{
    if (status == HAL_TIMEOUT)
    {
        return ICM42688P_TIMEOUT;
    }

    return ICM42688P_ERROR;
}

icm42688p_status_t ICM42688P_ReadRegisters(
    uint8_t reg,
    uint8_t *data,
    uint16_t length)
{
    if ((data == NULL) || (length == 0U))
    {
        return ICM42688P_INVALID_ARGUMENT;
    }

    /*
     * ICM-42688-P SPI read:
     * bit 7 of the register address is set for reads.
     * CS remains low throughout the complete transaction.
     */
    uint8_t address = reg | 0x80U;

    ICM42688P_CS_LOW();

    HAL_StatusTypeDef status = HAL_SPI_Transmit(
        &hspi1,
        &address,
        1U,
        ICM42688P_SPI_TIMEOUT_MS
    );

    if (status == HAL_OK)
    {
        status = HAL_SPI_Receive(
            &hspi1,
            data,
            length,
            ICM42688P_SPI_TIMEOUT_MS
        );
    }

    ICM42688P_CS_HIGH();

    if (status != HAL_OK)
    {
        return ICM42688P_MapHALStatus(status);
    }

    return ICM42688P_OK;
}

icm42688p_status_t ICM42688P_ReadRegister(
    uint8_t reg,
    uint8_t *data)
{
    return ICM42688P_ReadRegisters(reg, data, 1U);
}

icm42688p_status_t ICM42688P_WriteRegister(
    uint8_t reg,
    uint8_t data)
{
    uint8_t tx_buffer[2];

    tx_buffer[0] = reg & 0x7FU;
    tx_buffer[1] = data;

    ICM42688P_CS_LOW();

    HAL_StatusTypeDef status = HAL_SPI_Transmit(
        &hspi1,
        tx_buffer,
        sizeof(tx_buffer),
        ICM42688P_SPI_TIMEOUT_MS
    );

    ICM42688P_CS_HIGH();

    if (status != HAL_OK)
    {
        return ICM42688P_MapHALStatus(status);
    }

    return ICM42688P_OK;
}

icm42688p_status_t ICM42688P_ReadWhoAmI(
    uint8_t *device_id)
{
    if (device_id == NULL)
    {
        return ICM42688P_INVALID_ARGUMENT;
    }

    icm42688p_status_t status = ICM42688P_ReadRegister(
        ICM42688P_REG_WHO_AM_I,
        device_id
    );

    if (status != ICM42688P_OK)
    {
        return status;
    }

    if (*device_id != ICM42688P_WHO_AM_I_VALUE)
    {
        return ICM42688P_INVALID_DEVICE;
    }

    return ICM42688P_OK;
}

icm42688p_status_t ICM42688P_Init(void)
{
    uint8_t device_id = 0U;

    /* Issue software reset. */
    icm42688p_status_t status = ICM42688P_WriteRegister(
        ICM42688P_REG_DEVICE_CONFIG,
        ICM42688P_DEVICE_RESET
    );

    if (status != ICM42688P_OK)
    {
        return status;
    }

    /* Allow the sensor time to reset. */
    HAL_Delay(2U);

    /* Verify device identity after reset. */
    status = ICM42688P_ReadWhoAmI(&device_id);

    if (status != ICM42688P_OK)
    {
        return status;
    }

    /*
     * Configure gyro and accelerometer for low-noise mode.
     * Confirm register values and timing against the exact
     * datasheet revision and hardware configuration before use.
     */
    status = ICM42688P_WriteRegister(
        ICM42688P_REG_PWR_MGMT0,
        0x0FU
    );

    if (status != ICM42688P_OK)
    {
        return status;
    }

    HAL_Delay(50U);

    /*
     * Initial targets:
     * Gyroscope: ±2000 degrees/s, 1 kHz ODR.
     * Accelerometer: ±16 g, 1 kHz ODR.
     */
    status = ICM42688P_WriteRegister(
        ICM42688P_REG_GYRO_CONFIG0,
        0x06U
    );

    if (status != ICM42688P_OK)
    {
        return status;
    }

    status = ICM42688P_WriteRegister(
        ICM42688P_REG_ACCEL_CONFIG0,
        0x06U
    );

    if (status != ICM42688P_OK)
    {
        return status;
    }

    return ICM42688P_OK;
}

icm42688p_status_t ICM42688P_ReadRawData(
    int16_t accel[3],
    int16_t gyro[3],
    int16_t *temperature)
{
    if ((accel == NULL) ||
        (gyro == NULL) ||
        (temperature == NULL))
    {
        return ICM42688P_INVALID_ARGUMENT;
    }

    /*
     * Read temperature (2 bytes), accelerometer (6 bytes),
     * and gyroscope (6 bytes) in one burst.
     */
    uint8_t buffer[14];

    icm42688p_status_t status = ICM42688P_ReadRegisters(
        ICM42688P_REG_TEMP_DATA1,
        buffer,
        sizeof(buffer)
    );

    if (status != ICM42688P_OK)
    {
        return status;
    }

    *temperature = (int16_t)(
        ((uint16_t)buffer[0] << 8) |
        (uint16_t)buffer[1]
    );

    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        uint8_t index = (uint8_t)(2U + (axis * 2U));

        accel[axis] = (int16_t)(
            ((uint16_t)buffer[index] << 8) |
            (uint16_t)buffer[index + 1U]
        );
    }

    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        uint8_t index = (uint8_t)(8U + (axis * 2U));

        gyro[axis] = (int16_t)(
            ((uint16_t)buffer[index] << 8) |
            (uint16_t)buffer[index + 1U]
        );
    }

    return ICM42688P_OK;
}

void ICM42688P_ConvertToPhysicalUnits(
    const int16_t accel_raw[3],
    const int16_t gyro_raw[3],
    float accel_ms2[3],
    float gyro_rads[3])
{
    if ((accel_raw == NULL) ||
        (gyro_raw == NULL) ||
        (accel_ms2 == NULL) ||
        (gyro_rads == NULL))
    {
        return;
    }

    for (uint8_t axis = 0U; axis < 3U; axis++)
    {
        accel_ms2[axis] =
            ((float)accel_raw[axis] /
             ICM42688P_ACCEL_SENSITIVITY_LSB_G) *
            ICM42688P_GRAVITY_MS2;

        gyro_rads[axis] =
            ((float)gyro_raw[axis] /
             ICM42688P_GYRO_SENSITIVITY_LSB_DPS) *
            ICM42688P_DEG_TO_RAD;
    }
}
