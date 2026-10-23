#ifndef DPS310_H
#define DPS310_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    DPS310_STATUS_OK = 0,
    DPS310_STATUS_ERROR,
    DPS310_STATUS_INVALID_ARGUMENT,
    DPS310_STATUS_COMMUNICATION_ERROR,
    DPS310_STATUS_INVALID_DEVICE,
    DPS310_STATUS_NOT_INITIALIZED,
    DPS310_STATUS_DATA_NOT_READY,
    DPS310_STATUS_TIMEOUT
} DPS310_Status;

typedef struct
{
    float temperature_c;       /* compensated temperature, degrees Celsius */
    float pressure_pa;          /* compensated absolute pressure, Pascals */
    float altitude_m;           /* pressure altitude relative to configured reference */
    int32_t raw_temperature;    /* signed 24-bit ADC result, sign-extended to int32_t */
    int32_t raw_pressure;       /* signed 24-bit ADC result, sign-extended to int32_t */
} DPS310_Data;

/*
 * Initialize the driver after calling DPS310_Interface_Init(&hi2c1).
 * The driver probes both legal 7-bit I2C addresses, 0x77 then 0x76.
 */
DPS310_Status DPS310_Init(void);

/* Return one new sample only when both pressure and temperature are ready. */
DPS310_Status DPS310_ReadData(DPS310_Data *data);

/*
 * Set the reference pressure in Pascals for altitude calculation.
 * Default is standard sea-level pressure, 101325 Pa.
 * For relative altitude, set this to the pressure measured at the reference
 * location before comparing subsequent readings.
 */
DPS310_Status DPS310_SetReferencePressure(float pressure_pa);
float DPS310_GetReferencePressure(void);

bool DPS310_IsInitialized(void);
uint8_t DPS310_GetDeviceAddress(void); /* 7-bit address; 0 if not detected */

#ifdef __cplusplus
}
#endif

#endif /* DPS310_H */
