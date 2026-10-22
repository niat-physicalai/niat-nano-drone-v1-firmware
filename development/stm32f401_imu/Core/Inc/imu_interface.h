#ifndef IMU_INTERFACE_H
#define IMU_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Standardized IMU measurement.
 *
 * Accelerometer: m/s^2
 * Gyroscope:     rad/s
 * Temperature:   degrees Celsius
 */
typedef struct
{
    uint32_t timestamp_us;

    float accel_x;
    float accel_y;
    float accel_z;

    float gyro_x;
    float gyro_y;
    float gyro_z;

    float temperature_c;

    bool valid;

} imu_measurement_t;

/**
 * @brief Convert raw IMU readings into a standardized measurement.
 *
 * @param accel_raw      Raw accelerometer readings [X, Y, Z].
 * @param gyro_raw       Raw gyroscope readings [X, Y, Z].
 * @param temperature_raw Raw temperature register value.
 * @param timestamp_us   Measurement timestamp in microseconds.
 * @param measurement    Destination measurement structure.
 *
 * @return true on success, false if an argument is invalid.
 */
bool IMU_ConvertRawMeasurement(
    const int16_t accel_raw[3],
    const int16_t gyro_raw[3],
    int16_t temperature_raw,
    uint32_t timestamp_us,
    imu_measurement_t *measurement
);

#endif /* IMU_INTERFACE_H */
