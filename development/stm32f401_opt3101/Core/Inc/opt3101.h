#ifndef OPT3101_H
#define OPT3101_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include "opt3101_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Frozen BOM addresses are 7-bit I2C addresses. STM32 HAL shifting is done
 * only inside opt3101_interface.c. */
#define OPT3101_ADDR_FRONT_7BIT       0x5EU
#define OPT3101_ADDR_BOTTOM_7BIT      0x5FU
#define OPT3101_DEFAULT_I2C_TIMEOUT_MS 100U
#define OPT3101_DEFAULT_INIT_TIMEOUT_MS 1000U
#define OPT3101_DEFAULT_SUBFRAMES      512U
#define OPT3101_REGISTER_MASK_24BIT    0x00FFFFFFUL

typedef enum
{
    OPT3101_STATUS_OK = 0,
    OPT3101_STATUS_INVALID_ARGUMENT,
    OPT3101_STATUS_I2C_ERROR,
    OPT3101_STATUS_INIT_TIMEOUT,
    OPT3101_STATUS_NOT_INITIALIZED,
    OPT3101_STATUS_INVALID_CONFIGURATION,
    OPT3101_STATUS_NOT_READY
} OPT3101_Status;

typedef enum
{
    OPT3101_BRIGHTNESS_LOW = 0U,
    OPT3101_BRIGHTNESS_HIGH = 1U,
    OPT3101_BRIGHTNESS_ADAPTIVE = 255U
} OPT3101_Brightness;

typedef enum
{
    OPT3101_MODE_MONOSHOT = 0U,
    OPT3101_MODE_CONTINUOUS = 1U
} OPT3101_Mode;

typedef struct
{
    uint16_t phase_raw;          /* PHASE_OUT[15:0] */
    uint32_t distance_mm_nominal;/* Nominal conversion, not factory-calibrated */
    uint16_t amplitude;          /* AMP_OUT[15:0] */
    uint16_t ambient;            /* AMB_DATA[9:0] */
    uint16_t temperature_raw;    /* TMAIN[11:0] */
    int16_t temperature_c_nominal; /* Nominal conversion from TI SDK */
    int32_t i_raw;               /* Signed 24-bit raw I data */
    int32_t q_raw;               /* Signed 24-bit raw Q data */
    uint8_t channel;
    uint8_t brightness_high;
    uint8_t frame_counter;
    uint8_t dealias_bin;
    bool phase_overflow;
    bool phase_overflow_f2;
    bool signal_overflow;
    bool ambient_overflow;
    bool frame_status;
    bool mod_freq;
    bool distance_valid;         /* false on reported overflow flags */
} OPT3101_Measurement;

typedef struct
{
    uint8_t reg;
    uint32_t value; /* Low 24 bits are written */
} OPT3101_RegisterWrite;

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t address_7bit;
    uint32_t i2c_timeout_ms;
    uint32_t init_timeout_ms;
    uint32_t frame_delay_ms;
    uint32_t sample_start_tick;
    OPT3101_Status last_status;
    HAL_StatusTypeDef last_hal_status;
    OPT3101_Mode mode;
    bool initialized;
    bool timing_generator_enabled;
    bool sample_pending;
} OPT3101_Device;

/* Attach a device context to the STM32 I2C peripheral. */
OPT3101_Status OPT3101_Attach(OPT3101_Device *dev,
                              I2C_HandleTypeDef *hi2c,
                              uint8_t address_7bit,
                              uint32_t i2c_timeout_ms);

/* Reset/boot, apply the reference standalone profile, select TX0, and use
 * low-brightness mode. This is a bring-up profile, not final drone tuning. */
OPT3101_Status OPT3101_Init(OPT3101_Device *dev);
OPT3101_Status OPT3101_ResetAndWait(OPT3101_Device *dev);
OPT3101_Status OPT3101_ConfigureDefault(OPT3101_Device *dev);

OPT3101_Status OPT3101_ReadRegister(OPT3101_Device *dev,
                                    uint8_t reg,
                                    uint32_t *value);
OPT3101_Status OPT3101_WriteRegister(OPT3101_Device *dev,
                                     uint8_t reg,
                                     uint32_t value);
OPT3101_Status OPT3101_UpdateRegisterBits(OPT3101_Device *dev,
                                          uint8_t reg,
                                          uint32_t mask,
                                          uint32_t value);
OPT3101_Status OPT3101_ApplyRegisterSequence(OPT3101_Device *dev,
                                             const OPT3101_RegisterWrite *sequence,
                                             uint16_t length);
OPT3101_Status OPT3101_Probe(OPT3101_Device *dev);

OPT3101_Status OPT3101_SetChannel(OPT3101_Device *dev, uint8_t channel);
OPT3101_Status OPT3101_SetBrightness(OPT3101_Device *dev,
                                     OPT3101_Brightness brightness);
OPT3101_Status OPT3101_SetMonoshotMode(OPT3101_Device *dev);
OPT3101_Status OPT3101_SetContinuousMode(OPT3101_Device *dev);
OPT3101_Status OPT3101_SetFrameTiming(OPT3101_Device *dev,
                                      uint16_t subframe_count);
OPT3101_Status OPT3101_EnableTimingGenerator(OPT3101_Device *dev);
OPT3101_Status OPT3101_DisableTimingGenerator(OPT3101_Device *dev);
OPT3101_Status OPT3101_EnableDataReadyOutput(OPT3101_Device *dev,
                                            uint8_t gp_pin);

OPT3101_Status OPT3101_StartSample(OPT3101_Device *dev);
bool OPT3101_IsSampleDone(const OPT3101_Device *dev);
OPT3101_Status OPT3101_ReadOutputRegisters(OPT3101_Device *dev,
                                           OPT3101_Measurement *measurement);
OPT3101_Status OPT3101_Sample(OPT3101_Device *dev,
                              OPT3101_Measurement *measurement);

/* Optional design-ID check supported by the TI SDK. It changes the I/Q mux
 * temporarily and restores its prior setting on success. */
OPT3101_Status OPT3101_ReadDesignID(OPT3101_Device *dev,
                                    uint32_t *design_id_low,
                                    uint32_t *design_id_high);
OPT3101_Status OPT3101_ValidateDesignID(OPT3101_Device *dev);

OPT3101_Status OPT3101_GetLastStatus(const OPT3101_Device *dev);
HAL_StatusTypeDef OPT3101_GetLastHALStatus(const OPT3101_Device *dev);
const char *OPT3101_StatusString(OPT3101_Status status);

#ifdef __cplusplus
}
#endif
#endif
