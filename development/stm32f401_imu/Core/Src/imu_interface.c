#include "imu_interface.h"
#include "icm42688p.h"

#include <stddef.h>

bool IMU_ConvertRawMeasurement(
    const int16_t accel_raw[3],
    const int16_t gyro_raw[3],
    int16_t temperature_raw,
    uint32_t timestamp_us,
    imu_measurement_t *measurement)
{
    if ((accel_raw == NULL) ||
        (gyro_raw == NULL) ||
        (measurement == NULL))
    {
        return false;
    }

    float accel_ms2[3];
    float gyro_rads[3];

    ICM42688P_ConvertToPhysicalUnits(
        accel_raw,
        gyro_raw,
        accel_ms2,
        gyro_rads
    );

    measurement->timestamp_us = timestamp_us;

    measurement->accel_x = accel_ms2[0];
    measurement->accel_y = accel_ms2[1];
    measurement->accel_z = accel_ms2[2];

    measurement->gyro_x = gyro_rads[0];
    measurement->gyro_y = gyro_rads[1];
    measurement->gyro_z = gyro_rads[2];

    /*
     * ICM-42688-P temperature conversion:
     * temperature in degrees Celsius = raw / 132.48 + 25.
     */
    measurement->temperature_c =
        ((float)temperature_raw / 132.48f) + 25.0f;

    measurement->valid = true;

    return true;
}
