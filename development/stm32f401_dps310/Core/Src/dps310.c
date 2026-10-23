#include "dps310.h"
#include "dps310_interface.h"

#include <math.h>
#include <stddef.h>

/* DPS310 register map */
#define DPS310_REG_PRS_B2          0x00U
#define DPS310_REG_PRS_CFG         0x06U
#define DPS310_REG_TMP_CFG         0x07U
#define DPS310_REG_MEAS_CFG        0x08U
#define DPS310_REG_CFG_REG         0x09U
#define DPS310_REG_PRODUCT_ID      0x0DU
#define DPS310_REG_COEF            0x10U
#define DPS310_REG_COEF_SRCE       0x28U

#define DPS310_PRODUCT_ID          0x10U
#define DPS310_MEAS_CFG_PRS_RDY    0x10U
#define DPS310_MEAS_CFG_TMP_RDY    0x20U
#define DPS310_MEAS_CFG_READY      0xC0U
#define DPS310_COEF_TMP_EXT_MASK   0x80U
#define DPS310_MODE_BACKGROUND_ALL 0x07U

#define DPS310_ADDR_PRIMARY        0x77U
#define DPS310_ADDR_SECONDARY      0x76U

/* Conservative, configurable measurement defaults: 4 Hz, OSR x4. */
#define DPS310_RATE_4_HZ_CODE      0x02U
#define DPS310_OSR_4X_CODE         0x02U
#define DPS310_PRESSURE_CFG_VALUE  ((DPS310_RATE_4_HZ_CODE << 4U) | DPS310_OSR_4X_CODE)
#define DPS310_TEMPERATURE_CFG_BASE ((DPS310_RATE_4_HZ_CODE << 4U) | DPS310_OSR_4X_CODE)
#define DPS310_SCALE_FACTOR_OSR4   3670016.0f

#define DPS310_STARTUP_DELAY_MS    40U
#define DPS310_READY_POLL_MS       10U
#define DPS310_READY_TIMEOUT_MS    500U

/*
 * DPS310 calibration coefficients. Widths follow the datasheet:
 * c0/c1 are signed 12-bit, c00/c10 signed 20-bit, others signed 16-bit.
 */
typedef struct
{
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
} DPS310_Calibration;

static DPS310_Calibration s_calibration;
static bool s_initialized = false;
static uint8_t s_device_address = 0U;
static float s_reference_pressure_pa = 101325.0f;

static int32_t DPS310_SignExtend(uint32_t value, uint8_t bit_count)
{
    const uint32_t sign_bit = (uint32_t)1U << (bit_count - 1U);
    const uint32_t mask = ((uint32_t)1U << bit_count) - 1U;

    value &= mask;
    if ((value & sign_bit) != 0U)
    {
        value |= ~mask;
    }
    return (int32_t)value;
}

static int32_t DPS310_DecodeSigned24(const uint8_t bytes[3])
{
    uint32_t value = ((uint32_t)bytes[0] << 16U) |
                     ((uint32_t)bytes[1] << 8U) |
                     (uint32_t)bytes[2];
    return DPS310_SignExtend(value, 24U);
}

static void DPS310_DecodeCalibration(const uint8_t raw[18], DPS310_Calibration *cal)
{
    uint32_t value;

    value = ((uint32_t)raw[0] << 4U) | ((uint32_t)raw[1] >> 4U);
    cal->c0 = (int16_t)DPS310_SignExtend(value, 12U);

    value = (((uint32_t)raw[1] & 0x0FU) << 8U) | (uint32_t)raw[2];
    cal->c1 = (int16_t)DPS310_SignExtend(value, 12U);

    value = ((uint32_t)raw[3] << 12U) |
            ((uint32_t)raw[4] << 4U) |
            ((uint32_t)raw[5] >> 4U);
    cal->c00 = DPS310_SignExtend(value, 20U);

    value = (((uint32_t)raw[5] & 0x0FU) << 16U) |
            ((uint32_t)raw[6] << 8U) |
            (uint32_t)raw[7];
    cal->c10 = DPS310_SignExtend(value, 20U);

    cal->c01 = (int16_t)DPS310_SignExtend(((uint32_t)raw[8] << 8U) | raw[9], 16U);
    cal->c11 = (int16_t)DPS310_SignExtend(((uint32_t)raw[10] << 8U) | raw[11], 16U);
    cal->c20 = (int16_t)DPS310_SignExtend(((uint32_t)raw[12] << 8U) | raw[13], 16U);
    cal->c21 = (int16_t)DPS310_SignExtend(((uint32_t)raw[14] << 8U) | raw[15], 16U);
    cal->c30 = (int16_t)DPS310_SignExtend(((uint32_t)raw[16] << 8U) | raw[17], 16U);
}

static DPS310_Status DPS310_FindDevice(void)
{
    const uint8_t addresses[2] = { DPS310_ADDR_PRIMARY, DPS310_ADDR_SECONDARY };
    bool any_i2c_response = false;

    for (uint8_t i = 0U; i < 2U; ++i)
    {
        uint8_t product_id = 0U;
        if (DPS310_Interface_SetAddress(addresses[i]) != HAL_OK)
        {
            continue;
        }

        if (DPS310_Interface_ReadRegister(DPS310_REG_PRODUCT_ID, &product_id) == HAL_OK)
        {
            any_i2c_response = true;
            if (product_id == DPS310_PRODUCT_ID)
            {
                s_device_address = addresses[i];
                return DPS310_STATUS_OK;
            }
        }
    }

    s_device_address = 0U;
    return any_i2c_response ? DPS310_STATUS_INVALID_DEVICE
                            : DPS310_STATUS_COMMUNICATION_ERROR;
}

static DPS310_Status DPS310_WaitForSelfInitialization(void)
{
    uint32_t elapsed_ms = 0U;

    HAL_Delay(DPS310_STARTUP_DELAY_MS);

    while (elapsed_ms <= DPS310_READY_TIMEOUT_MS)
    {
        uint8_t status = 0U;
        if (DPS310_Interface_ReadRegister(DPS310_REG_MEAS_CFG, &status) != HAL_OK)
        {
            return DPS310_STATUS_COMMUNICATION_ERROR;
        }

        if ((status & DPS310_MEAS_CFG_READY) == DPS310_MEAS_CFG_READY)
        {
            return DPS310_STATUS_OK;
        }

        HAL_Delay(DPS310_READY_POLL_MS);
        elapsed_ms += DPS310_READY_POLL_MS;
    }

    return DPS310_STATUS_TIMEOUT;
}

static DPS310_Status DPS310_LoadCalibration(void)
{
    uint8_t raw_coefficients[18];
    if (DPS310_Interface_ReadRegisters(DPS310_REG_COEF,
                                       raw_coefficients,
                                       (uint16_t)sizeof(raw_coefficients)) != HAL_OK)
    {
        return DPS310_STATUS_COMMUNICATION_ERROR;
    }

    DPS310_DecodeCalibration(raw_coefficients, &s_calibration);
    return DPS310_STATUS_OK;
}

static DPS310_Status DPS310_Configure(void)
{
    uint8_t coefficient_source = 0U;
    if (DPS310_Interface_ReadRegister(DPS310_REG_COEF_SRCE, &coefficient_source) != HAL_OK)
    {
        return DPS310_STATUS_COMMUNICATION_ERROR;
    }

    /*
     * Preserve the temperature sensor source used for factory calibration.
     * Both temperature and pressure run at 4 Hz with OSR x4. OSR x4 does not
     * require the result-shift bits in CFG_REG.
     */
    const uint8_t temperature_cfg =
        (uint8_t)((coefficient_source & DPS310_COEF_TMP_EXT_MASK) |
                  DPS310_TEMPERATURE_CFG_BASE);

    if (DPS310_Interface_WriteRegister(DPS310_REG_PRS_CFG,
                                       DPS310_PRESSURE_CFG_VALUE) != HAL_OK)
    {
        return DPS310_STATUS_COMMUNICATION_ERROR;
    }

    if (DPS310_Interface_WriteRegister(DPS310_REG_TMP_CFG, temperature_cfg) != HAL_OK)
    {
        return DPS310_STATUS_COMMUNICATION_ERROR;
    }

    if (DPS310_Interface_WriteRegister(DPS310_REG_CFG_REG, 0x00U) != HAL_OK)
    {
        return DPS310_STATUS_COMMUNICATION_ERROR;
    }

    /*
     * DPS310 temperature-path workaround also used by the Zephyr DPS310
     * driver: unlock protected register 0x62, set gain, then relock.
     */
    if (DPS310_Interface_WriteRegister(0x0EU, 0xA5U) != HAL_OK) return DPS310_STATUS_COMMUNICATION_ERROR;
    if (DPS310_Interface_WriteRegister(0x0FU, 0x96U) != HAL_OK) return DPS310_STATUS_COMMUNICATION_ERROR;
    if (DPS310_Interface_WriteRegister(0x62U, 0x02U) != HAL_OK) return DPS310_STATUS_COMMUNICATION_ERROR;
    if (DPS310_Interface_WriteRegister(0x0EU, 0x00U) != HAL_OK) return DPS310_STATUS_COMMUNICATION_ERROR;
    if (DPS310_Interface_WriteRegister(0x0FU, 0x00U) != HAL_OK) return DPS310_STATUS_COMMUNICATION_ERROR;

    /* Start continuous/background pressure and temperature measurements. */
    if (DPS310_Interface_WriteRegister(DPS310_REG_MEAS_CFG,
                                       DPS310_MODE_BACKGROUND_ALL) != HAL_OK)
    {
        return DPS310_STATUS_COMMUNICATION_ERROR;
    }

    HAL_Delay(2U);
    return DPS310_STATUS_OK;
}

static float DPS310_CompensateTemperature(int32_t raw_temperature)
{
    const float scaled_temperature = (float)raw_temperature / DPS310_SCALE_FACTOR_OSR4;
    return ((float)s_calibration.c0 * 0.5f) + ((float)s_calibration.c1 * scaled_temperature);
}

static float DPS310_CompensatePressure(int32_t raw_pressure, int32_t raw_temperature)
{
    const float scaled_pressure = (float)raw_pressure / DPS310_SCALE_FACTOR_OSR4;
    const float scaled_temperature = (float)raw_temperature / DPS310_SCALE_FACTOR_OSR4;

    float pressure = (float)s_calibration.c00;
    pressure += scaled_pressure * ((float)s_calibration.c10 +
                scaled_pressure * ((float)s_calibration.c20 +
                scaled_pressure * (float)s_calibration.c30));
    pressure += scaled_temperature * (float)s_calibration.c01;
    pressure += scaled_temperature * scaled_pressure *
                ((float)s_calibration.c11 + scaled_pressure * (float)s_calibration.c21);

    return pressure; /* DPS310 compensated pressure is in Pa. */
}

static float DPS310_CalculateAltitude(float pressure_pa)
{
    if ((pressure_pa <= 0.0f) || (s_reference_pressure_pa <= 0.0f))
    {
        return 0.0f;
    }

    /* International barometric approximation; reference pressure is configurable. */
    return 44330.0f * (1.0f - powf(pressure_pa / s_reference_pressure_pa, 0.190294957f));
}

DPS310_Status DPS310_Init(void)
{
    s_initialized = false;
    s_device_address = 0U;

    DPS310_Status status = DPS310_FindDevice();
    if (status != DPS310_STATUS_OK)
    {
        return status;
    }

    status = DPS310_WaitForSelfInitialization();
    if (status != DPS310_STATUS_OK)
    {
        return status;
    }

    status = DPS310_LoadCalibration();
    if (status != DPS310_STATUS_OK)
    {
        return status;
    }

    status = DPS310_Configure();
    if (status != DPS310_STATUS_OK)
    {
        return status;
    }

    s_initialized = true;
    return DPS310_STATUS_OK;
}

DPS310_Status DPS310_ReadData(DPS310_Data *data)
{
    if (data == NULL)
    {
        return DPS310_STATUS_INVALID_ARGUMENT;
    }

    if (!s_initialized)
    {
        return DPS310_STATUS_NOT_INITIALIZED;
    }

    uint8_t measurement_status = 0U;
    if (DPS310_Interface_ReadRegister(DPS310_REG_MEAS_CFG, &measurement_status) != HAL_OK)
    {
        return DPS310_STATUS_COMMUNICATION_ERROR;
    }

    const uint8_t ready_bits = DPS310_MEAS_CFG_PRS_RDY | DPS310_MEAS_CFG_TMP_RDY;
    if ((measurement_status & ready_bits) != ready_bits)
    {
        return DPS310_STATUS_DATA_NOT_READY;
    }

    uint8_t raw_results[6];
    if (DPS310_Interface_ReadRegisters(DPS310_REG_PRS_B2,
                                       raw_results,
                                       (uint16_t)sizeof(raw_results)) != HAL_OK)
    {
        return DPS310_STATUS_COMMUNICATION_ERROR;
    }

    const int32_t raw_pressure = DPS310_DecodeSigned24(&raw_results[0]);
    const int32_t raw_temperature = DPS310_DecodeSigned24(&raw_results[3]);
    const float temperature_c = DPS310_CompensateTemperature(raw_temperature);
    const float pressure_pa = DPS310_CompensatePressure(raw_pressure, raw_temperature);

    if (!(pressure_pa > 0.0f) || (pressure_pa > 200000.0f) ||
        (temperature_c < -60.0f) || (temperature_c > 100.0f))
    {
        /* Treat clearly impossible compensated values as a driver/data error. */
        return DPS310_STATUS_ERROR;
    }

    data->temperature_c = temperature_c;
    data->pressure_pa = pressure_pa;
    data->altitude_m = DPS310_CalculateAltitude(pressure_pa);
    data->raw_pressure = raw_pressure;
    data->raw_temperature = raw_temperature;

    return DPS310_STATUS_OK;
}

DPS310_Status DPS310_SetReferencePressure(float pressure_pa)
{
    /* Allow common local sea-level/reference pressures, reject nonsensical inputs. */
    if ((pressure_pa < 30000.0f) || (pressure_pa > 120000.0f))
    {
        return DPS310_STATUS_INVALID_ARGUMENT;
    }

    s_reference_pressure_pa = pressure_pa;
    return DPS310_STATUS_OK;
}

float DPS310_GetReferencePressure(void)
{
    return s_reference_pressure_pa;
}

bool DPS310_IsInitialized(void)
{
    return s_initialized;
}

uint8_t DPS310_GetDeviceAddress(void)
{
    return s_device_address;
}
