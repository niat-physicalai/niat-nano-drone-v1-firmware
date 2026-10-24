/*
 * OPT3101 STM32 HAL driver.
 *
 * The register protocol and standalone measurement setup are based on the
 * OPT3101 datasheet, TI OPT3101 SDK v0.9.1 documentation, and the MIT-licensed
 * Pololu OPT3101 Arduino reference library (Copyright Pololu Corporation).
 * This file is a C / STM32 HAL adaptation; see LICENSES/Pololu-MIT.txt.
 *
 * IMPORTANT: The built-in setup is a general standalone bring-up profile.
 * A final flight-board design must use a register profile generated for its
 * actual LED/photodiode wiring, SDA_M/SCL_M configuration, current limits and
 * calibration. The BOM alone does not define those design-specific settings.
 */
#include "opt3101.h"
#include <stddef.h>

#define REG_SOFTWARE_RESET       0x00U
#define REG_INIT_STATUS          0x03U
#define REG_PHASE_OUT            0x08U
#define REG_AMP_OUT              0x09U
#define REG_AMBIENT_TEMP         0x0AU
#define REG_DIG_GPO_SEL          0x0BU
#define REG_IQ_SELECTOR          0x2EU
#define REG_I_RAW                0x3BU
#define REG_Q_RAW                0x3CU
#define REG_MONOSHOT_CONFIG      0x27U
#define REG_CHANNEL_BRIGHTNESS   0x2AU
#define REG_TIMING_GENERATOR     0x80U
#define REG_FRAME_TIMING         0x9FU
#define REG_FRAME_TIMING_MASK    0x97U
#define REG_GPIO_CONFIG          0x78U

#define INIT_LOAD_DONE_MASK      (1UL << 8U)
#define IQ_READ_DATA_SEL_MASK    (7UL << 9U)
#define IQ_READ_DATA_SEL_SHIFT   9U
#define XTALK_FILTER_MASK        (0xFUL << 20U)
#define TX_SWITCH_ENABLE_MASK    (1UL << 0U)
#define TX_CHANNEL_MASK          (3UL << 1U)
#define ADAPTIVE_HDR_MASK        (1UL << 15U)
#define HDR_MODE_MASK            (1UL << 16U)
#define TG_EN_MASK               (1UL << 0U)
#define REG80_DEFAULT            0x004E1EUL
#define PHASE_MM_NUMERATOR       14990UL

static OPT3101_Status opt3101_set_status(OPT3101_Device *dev,
                                         OPT3101_Status status)
{
    if (dev != NULL)
    {
        dev->last_status = status;
        if (status == OPT3101_STATUS_OK)
        {
            dev->last_hal_status = HAL_OK;
        }
    }
    return status;
}

static OPT3101_Status opt3101_from_hal(OPT3101_Device *dev,
                                       HAL_StatusTypeDef hal_status)
{
    if (dev == NULL)
    {
        return OPT3101_STATUS_INVALID_ARGUMENT;
    }
    dev->last_hal_status = hal_status;
    dev->last_status = (hal_status == HAL_OK) ? OPT3101_STATUS_OK
                                               : OPT3101_STATUS_I2C_ERROR;
    return dev->last_status;
}

static bool opt3101_context_valid(const OPT3101_Device *dev)
{
    return (dev != NULL) && (dev->hi2c != NULL) &&
           (dev->address_7bit <= 0x7FU);
}

static int32_t opt3101_sign_extend_24(uint32_t value)
{
    value &= OPT3101_REGISTER_MASK_24BIT;
    if ((value & 0x00800000UL) != 0U)
    {
        return (int32_t)(value & 0x007FFFFFUL) - 0x00800000L;
    }
    return (int32_t)value;
}

OPT3101_Status OPT3101_Attach(OPT3101_Device *dev,
                              I2C_HandleTypeDef *hi2c,
                              uint8_t address_7bit,
                              uint32_t i2c_timeout_ms)
{
    if ((dev == NULL) || (hi2c == NULL) || (address_7bit > 0x7FU))
    {
        return OPT3101_STATUS_INVALID_ARGUMENT;
    }

    dev->hi2c = hi2c;
    dev->address_7bit = address_7bit;
    dev->i2c_timeout_ms = (i2c_timeout_ms == 0U)
                              ? OPT3101_DEFAULT_I2C_TIMEOUT_MS
                              : i2c_timeout_ms;
    dev->init_timeout_ms = OPT3101_DEFAULT_INIT_TIMEOUT_MS;
    dev->frame_delay_ms = 0U;
    dev->sample_start_tick = 0U;
    dev->last_status = OPT3101_STATUS_OK;
    dev->last_hal_status = HAL_OK;
    dev->mode = OPT3101_MODE_MONOSHOT;
    dev->initialized = false;
    dev->timing_generator_enabled = false;
    dev->sample_pending = false;
    return OPT3101_STATUS_OK;
}

OPT3101_Status OPT3101_ReadRegister(OPT3101_Device *dev,
                                    uint8_t reg,
                                    uint32_t *value)
{
    HAL_StatusTypeDef hal_status;

    if (!opt3101_context_valid(dev) || (value == NULL))
    {
        return opt3101_set_status(dev, OPT3101_STATUS_INVALID_ARGUMENT);
    }

    hal_status = OPT3101_Interface_ReadRegister(dev->hi2c,
                                                 dev->address_7bit,
                                                 reg,
                                                 value,
                                                 dev->i2c_timeout_ms);
    if (hal_status != HAL_OK)
    {
        return opt3101_from_hal(dev, hal_status);
    }
    *value &= OPT3101_REGISTER_MASK_24BIT;
    return opt3101_set_status(dev, OPT3101_STATUS_OK);
}

OPT3101_Status OPT3101_WriteRegister(OPT3101_Device *dev,
                                     uint8_t reg,
                                     uint32_t value)
{
    HAL_StatusTypeDef hal_status;

    if (!opt3101_context_valid(dev))
    {
        return opt3101_set_status(dev, OPT3101_STATUS_INVALID_ARGUMENT);
    }

    hal_status = OPT3101_Interface_WriteRegister(dev->hi2c,
                                                  dev->address_7bit,
                                                  reg,
                                                  value & OPT3101_REGISTER_MASK_24BIT,
                                                  dev->i2c_timeout_ms);
    return opt3101_from_hal(dev, hal_status);
}

OPT3101_Status OPT3101_UpdateRegisterBits(OPT3101_Device *dev,
                                          uint8_t reg,
                                          uint32_t mask,
                                          uint32_t value)
{
    uint32_t current;
    OPT3101_Status status;

    if ((mask & ~OPT3101_REGISTER_MASK_24BIT) != 0U)
    {
        return opt3101_set_status(dev, OPT3101_STATUS_INVALID_ARGUMENT);
    }

    status = OPT3101_ReadRegister(dev, reg, &current);
    if (status != OPT3101_STATUS_OK)
    {
        return status;
    }

    current = (current & (~mask & OPT3101_REGISTER_MASK_24BIT)) |
              (value & mask);
    return OPT3101_WriteRegister(dev, reg, current);
}

OPT3101_Status OPT3101_ApplyRegisterSequence(OPT3101_Device *dev,
                                             const OPT3101_RegisterWrite *sequence,
                                             uint16_t length)
{
    uint16_t i;
    OPT3101_Status status;

    if (!opt3101_context_valid(dev) || (sequence == NULL) || (length == 0U))
    {
        return opt3101_set_status(dev, OPT3101_STATUS_INVALID_ARGUMENT);
    }

    for (i = 0U; i < length; ++i)
    {
        status = OPT3101_WriteRegister(dev, sequence[i].reg, sequence[i].value);
        if (status != OPT3101_STATUS_OK)
        {
            return status;
        }
    }
    return opt3101_set_status(dev, OPT3101_STATUS_OK);
}

OPT3101_Status OPT3101_Probe(OPT3101_Device *dev)
{
    HAL_StatusTypeDef hal_status;

    if (!opt3101_context_valid(dev))
    {
        return opt3101_set_status(dev, OPT3101_STATUS_INVALID_ARGUMENT);
    }

    hal_status = HAL_I2C_IsDeviceReady(dev->hi2c,
                                       (uint16_t)((uint16_t)dev->address_7bit << 1U),
                                       2U,
                                       dev->i2c_timeout_ms);
    return opt3101_from_hal(dev, hal_status);
}

OPT3101_Status OPT3101_ResetAndWait(OPT3101_Device *dev)
{
    HAL_StatusTypeDef hal_status;
    uint32_t started;
    uint32_t status_reg = 0U;

    if (!opt3101_context_valid(dev))
    {
        return opt3101_set_status(dev, OPT3101_STATUS_INVALID_ARGUMENT);
    }

    dev->initialized = false;
    dev->timing_generator_enabled = false;
    dev->sample_pending = false;

    hal_status = OPT3101_Interface_SoftwareReset(dev->hi2c,
                                                  dev->address_7bit,
                                                  dev->i2c_timeout_ms);
    if (hal_status != HAL_OK)
    {
        return opt3101_from_hal(dev, hal_status);
    }
    opt3101_set_status(dev, OPT3101_STATUS_OK);

    HAL_Delay(5U);
    started = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - started) < dev->init_timeout_ms)
    {
        if (OPT3101_ReadRegister(dev, REG_INIT_STATUS, &status_reg) == OPT3101_STATUS_OK)
        {
            if ((status_reg & INIT_LOAD_DONE_MASK) != 0U)
            {
                return opt3101_set_status(dev, OPT3101_STATUS_OK);
            }
        }
        /* Some boards briefly NACK while loading startup configuration. Retry
         * until the bounded init timeout rather than blocking forever. */
        HAL_Delay(1U);
    }

    dev->last_status = OPT3101_STATUS_INIT_TIMEOUT;
    return OPT3101_STATUS_INIT_TIMEOUT;
}

OPT3101_Status OPT3101_SetMonoshotMode(OPT3101_Device *dev)
{
    OPT3101_Status status;

    status = OPT3101_WriteRegister(dev, REG_MONOSHOT_CONFIG, 0x26AC07UL);
    if (status != OPT3101_STATUS_OK) return status;
    status = OPT3101_WriteRegister(dev, 0x76U, 0x000121UL);
    if (status != OPT3101_STATUS_OK) return status;
    status = OPT3101_WriteRegister(dev, 0x26U, (95UL << 10U) | 0x0FUL);
    if (status != OPT3101_STATUS_OK) return status;

    dev->mode = OPT3101_MODE_MONOSHOT;
    return opt3101_set_status(dev, OPT3101_STATUS_OK);
}

OPT3101_Status OPT3101_SetContinuousMode(OPT3101_Device *dev)
{
    OPT3101_Status status = OPT3101_WriteRegister(dev,
                                                  REG_MONOSHOT_CONFIG,
                                                  0x26AC04UL);
    if (status != OPT3101_STATUS_OK) return status;
    dev->mode = OPT3101_MODE_CONTINUOUS;
    return opt3101_set_status(dev, OPT3101_STATUS_OK);
}

OPT3101_Status OPT3101_SetChannel(OPT3101_Device *dev, uint8_t channel)
{
    uint32_t value;
    OPT3101_Status status;

    if (channel > 2U)
    {
        return opt3101_set_status(dev, OPT3101_STATUS_INVALID_ARGUMENT);
    }

    status = OPT3101_ReadRegister(dev, REG_CHANNEL_BRIGHTNESS, &value);
    if (status != OPT3101_STATUS_OK) return status;

    value &= ~(TX_SWITCH_ENABLE_MASK | TX_CHANNEL_MASK);
    value |= ((uint32_t)channel << 1U) & TX_CHANNEL_MASK;
    return OPT3101_WriteRegister(dev, REG_CHANNEL_BRIGHTNESS, value);
}

OPT3101_Status OPT3101_SetBrightness(OPT3101_Device *dev,
                                     OPT3101_Brightness brightness)
{
    uint32_t value;
    OPT3101_Status status;

    if ((brightness != OPT3101_BRIGHTNESS_LOW) &&
        (brightness != OPT3101_BRIGHTNESS_HIGH) &&
        (brightness != OPT3101_BRIGHTNESS_ADAPTIVE))
    {
        return opt3101_set_status(dev, OPT3101_STATUS_INVALID_ARGUMENT);
    }

    status = OPT3101_ReadRegister(dev, REG_CHANNEL_BRIGHTNESS, &value);
    if (status != OPT3101_STATUS_OK) return status;

    if (brightness == OPT3101_BRIGHTNESS_ADAPTIVE)
    {
        value |= ADAPTIVE_HDR_MASK;
    }
    else
    {
        value &= ~(ADAPTIVE_HDR_MASK | HDR_MODE_MASK);
        value |= ((uint32_t)brightness << 16U) & HDR_MODE_MASK;
    }
    return OPT3101_WriteRegister(dev, REG_CHANNEL_BRIGHTNESS, value);
}

OPT3101_Status OPT3101_SetFrameTiming(OPT3101_Device *dev,
                                      uint16_t subframe_count)
{
    uint32_t reg2e;
    uint32_t count_minus_one;
    uint8_t time_const = 0U;
    uint32_t frame_time_ms;
    OPT3101_Status status;

    if ((subframe_count < 1U) || (subframe_count > 4096U) ||
        ((subframe_count & (uint16_t)(subframe_count - 1U)) != 0U))
    {
        return opt3101_set_status(dev, OPT3101_STATUS_INVALID_CONFIGURATION);
    }

    while (((uint32_t)subframe_count << time_const) < 1024U)
    {
        ++time_const;
    }

    status = OPT3101_ReadRegister(dev, REG_IQ_SELECTOR, &reg2e);
    if (status != OPT3101_STATUS_OK) return status;
    reg2e = (reg2e & ~XTALK_FILTER_MASK) | ((uint32_t)time_const << 20U);
    status = OPT3101_WriteRegister(dev, REG_IQ_SELECTOR, reg2e);
    if (status != OPT3101_STATUS_OK) return status;

    count_minus_one = (uint32_t)subframe_count - 1U;
    status = OPT3101_WriteRegister(dev,
                                   REG_FRAME_TIMING,
                                   count_minus_one | (count_minus_one << 12U));
    if (status != OPT3101_STATUS_OK) return status;
    status = OPT3101_WriteRegister(dev,
                                   REG_FRAME_TIMING_MASK,
                                   count_minus_one | (count_minus_one << 12U));
    if (status != OPT3101_STATUS_OK) return status;

    /* Datasheet/Pololu reference timing: 1 sub-frame is approximately 0.25 ms.
     * Add about 6% margin to accommodate oscillator tolerance. */
    frame_time_ms = ((uint32_t)subframe_count + 3U) / 4U;
    dev->frame_delay_ms = frame_time_ms + ((frame_time_ms + 15U) / 16U);
    return opt3101_set_status(dev, OPT3101_STATUS_OK);
}

OPT3101_Status OPT3101_EnableTimingGenerator(OPT3101_Device *dev)
{
    OPT3101_Status status = OPT3101_WriteRegister(dev,
                                                  REG_TIMING_GENERATOR,
                                                  REG80_DEFAULT | TG_EN_MASK);
    if (status != OPT3101_STATUS_OK) return status;
    dev->timing_generator_enabled = true;
    return opt3101_set_status(dev, OPT3101_STATUS_OK);
}

OPT3101_Status OPT3101_DisableTimingGenerator(OPT3101_Device *dev)
{
    OPT3101_Status status = OPT3101_WriteRegister(dev,
                                                  REG_TIMING_GENERATOR,
                                                  REG80_DEFAULT);
    if (status != OPT3101_STATUS_OK) return status;
    dev->timing_generator_enabled = false;
    return opt3101_set_status(dev, OPT3101_STATUS_OK);
}

OPT3101_Status OPT3101_EnableDataReadyOutput(OPT3101_Device *dev,
                                            uint8_t gp_pin)
{
    uint32_t value;
    OPT3101_Status status;

    if ((gp_pin != 1U) && (gp_pin != 2U))
    {
        return opt3101_set_status(dev, OPT3101_STATUS_INVALID_ARGUMENT);
    }

    status = OPT3101_ReadRegister(dev, REG_DIG_GPO_SEL, &value);
    if (status != OPT3101_STATUS_OK) return status;
    value = (value & ~0x0FUL) | 9UL; /* DIG_GPO_SEL0 = DATA_RDY */
    status = OPT3101_WriteRegister(dev, REG_DIG_GPO_SEL, value);
    if (status != OPT3101_STATUS_OK) return status;

    status = OPT3101_ReadRegister(dev, REG_GPIO_CONFIG, &value);
    if (status != OPT3101_STATUS_OK) return status;

    if (gp_pin == 1U)
    {
        value &= ~((7UL << 6U) | (1UL << 12U));
        value |= (2UL << 6U) | (1UL << 12U); /* GPO1 = DIG_GPO_0 */
    }
    else
    {
        value &= ~((7UL << 9U) | (1UL << 15U));
        value |= (2UL << 9U) | (1UL << 15U); /* GPO2 = DIG_GPO_0 */
    }
    return OPT3101_WriteRegister(dev, REG_GPIO_CONFIG, value);
}

OPT3101_Status OPT3101_ConfigureDefault(OPT3101_Device *dev)
{
    uint32_t reg2e;
    OPT3101_Status status;

    if (!opt3101_context_valid(dev))
    {
        return opt3101_set_status(dev, OPT3101_STATUS_INVALID_ARGUMENT);
    }

    /* Baseline standalone profile from the MIT-licensed Pololu reference
     * library. This is deliberately not presented as the final flight-board
     * profile: design-specific LED current and SDA_M/SCL_M connection choices
     * must come from the hardware schematic and TI Configurator output. */
    status = OPT3101_WriteRegister(dev, 0x89U, 7000UL);       /* TG_OVL_WINDOW_START */
    if (status != OPT3101_STATUS_OK) return status;
    status = OPT3101_WriteRegister(dev, 0x6EU, 0x0A0000UL);  /* EN_TEMP_CONV */
    if (status != OPT3101_STATUS_OK) return status;
    status = OPT3101_WriteRegister(dev, 0x50U, 0x200101UL);  /* clip-mode profile */
    if (status != OPT3101_STATUS_OK) return status;

    status = OPT3101_ReadRegister(dev, REG_IQ_SELECTOR, &reg2e);
    if (status != OPT3101_STATUS_OK) return status;
    reg2e = (reg2e & ~IQ_READ_DATA_SEL_MASK) |
            (2UL << IQ_READ_DATA_SEL_SHIFT); /* raw I/Q readout */
    status = OPT3101_WriteRegister(dev, REG_IQ_SELECTOR, reg2e);
    if (status != OPT3101_STATUS_OK) return status;

    status = OPT3101_SetMonoshotMode(dev);
    if (status != OPT3101_STATUS_OK) return status;
    status = OPT3101_SetFrameTiming(dev, OPT3101_DEFAULT_SUBFRAMES);
    if (status != OPT3101_STATUS_OK) return status;

    return opt3101_set_status(dev, OPT3101_STATUS_OK);
}

OPT3101_Status OPT3101_Init(OPT3101_Device *dev)
{
    OPT3101_Status status;

    if (!opt3101_context_valid(dev))
    {
        return opt3101_set_status(dev, OPT3101_STATUS_INVALID_ARGUMENT);
    }

    dev->initialized = false;
    status = OPT3101_ResetAndWait(dev);
    if (status != OPT3101_STATUS_OK) return status;

    status = OPT3101_ConfigureDefault(dev);
    if (status != OPT3101_STATUS_OK) return status;

    /* Frozen BOM calls for one external emitter/photodiode set per OPT3101.
     * Use TX0 only unless the hardware schematic specifies otherwise. */
    status = OPT3101_SetChannel(dev, 0U);
    if (status != OPT3101_STATUS_OK) return status;
    status = OPT3101_SetBrightness(dev, OPT3101_BRIGHTNESS_LOW);
    if (status != OPT3101_STATUS_OK) return status;

    dev->initialized = true;
    dev->sample_pending = false;
    return opt3101_set_status(dev, OPT3101_STATUS_OK);
}

OPT3101_Status OPT3101_StartSample(OPT3101_Device *dev)
{
    OPT3101_Status status;

    if (!opt3101_context_valid(dev))
    {
        return opt3101_set_status(dev, OPT3101_STATUS_INVALID_ARGUMENT);
    }
    if (!dev->initialized)
    {
        return opt3101_set_status(dev, OPT3101_STATUS_NOT_INITIALIZED);
    }
    if (dev->frame_delay_ms == 0U)
    {
        return opt3101_set_status(dev, OPT3101_STATUS_INVALID_CONFIGURATION);
    }

    if (!dev->timing_generator_enabled)
    {
        status = OPT3101_EnableTimingGenerator(dev);
        if (status != OPT3101_STATUS_OK) return status;
    }

    if (dev->mode == OPT3101_MODE_MONOSHOT)
    {
        /* Required edge: clear MONOSHOT_BIT before setting it. */
        status = OPT3101_WriteRegister(dev, REG_SOFTWARE_RESET, 0x000000UL);
        if (status != OPT3101_STATUS_OK) return status;
        status = OPT3101_WriteRegister(dev, REG_SOFTWARE_RESET, 0x800000UL);
        if (status != OPT3101_STATUS_OK) return status;
    }

    dev->sample_start_tick = HAL_GetTick();
    dev->sample_pending = true;
    return opt3101_set_status(dev, OPT3101_STATUS_OK);
}

bool OPT3101_IsSampleDone(const OPT3101_Device *dev)
{
    if ((dev == NULL) || !dev->sample_pending || (dev->frame_delay_ms == 0U))
    {
        return false;
    }
    return (uint32_t)(HAL_GetTick() - dev->sample_start_tick) >= dev->frame_delay_ms;
}

OPT3101_Status OPT3101_ReadOutputRegisters(OPT3101_Device *dev,
                                           OPT3101_Measurement *measurement)
{
    uint32_t reg08, reg09, reg0a, i_value, q_value;
    OPT3101_Measurement tmp;
    OPT3101_Status status;

    if (!opt3101_context_valid(dev) || (measurement == NULL))
    {
        return opt3101_set_status(dev, OPT3101_STATUS_INVALID_ARGUMENT);
    }
    if (!dev->initialized)
    {
        return opt3101_set_status(dev, OPT3101_STATUS_NOT_INITIALIZED);
    }
    if (dev->sample_pending && !OPT3101_IsSampleDone(dev))
    {
        return opt3101_set_status(dev, OPT3101_STATUS_NOT_READY);
    }

    status = OPT3101_ReadRegister(dev, REG_PHASE_OUT, &reg08);
    if (status != OPT3101_STATUS_OK) return status;
    status = OPT3101_ReadRegister(dev, REG_AMP_OUT, &reg09);
    if (status != OPT3101_STATUS_OK) return status;
    status = OPT3101_ReadRegister(dev, REG_AMBIENT_TEMP, &reg0a);
    if (status != OPT3101_STATUS_OK) return status;
    status = OPT3101_ReadRegister(dev, REG_I_RAW, &i_value);
    if (status != OPT3101_STATUS_OK) return status;
    status = OPT3101_ReadRegister(dev, REG_Q_RAW, &q_value);
    if (status != OPT3101_STATUS_OK) return status;

    tmp.phase_raw = (uint16_t)(reg08 & 0xFFFFU);
    tmp.distance_mm_nominal = ((uint32_t)tmp.phase_raw * PHASE_MM_NUMERATOR) >> 16U;
    tmp.brightness_high = (uint8_t)((reg08 >> 17U) & 0x01U);
    tmp.channel = (uint8_t)((reg08 >> 18U) & 0x03U);
    tmp.frame_status = (((reg08 >> 20U) & 0x01U) != 0U);
    tmp.mod_freq = (((reg08 >> 21U) & 0x01U) != 0U);
    tmp.ambient_overflow = (((reg08 >> 22U) & 0x01U) != 0U);
    tmp.phase_overflow = (((reg08 >> 16U) & 0x01U) != 0U);

    tmp.amplitude = (uint16_t)(reg09 & 0xFFFFU);
    tmp.signal_overflow = (((reg09 >> 18U) & 0x01U) != 0U);
    tmp.phase_overflow_f2 = (((reg09 >> 19U) & 0x01U) != 0U);
    tmp.dealias_bin = (uint8_t)((reg09 >> 20U) & 0x0FU);

    tmp.frame_counter = (uint8_t)(((reg08 >> 23U) & 0x01U) |
                                  (((reg09 >> 16U) & 0x03U) << 1U) |
                                  ((reg0a & 0x03U) << 3U));
    tmp.ambient = (uint16_t)((reg0a >> 2U) & 0x03FFU);
    tmp.temperature_raw = (uint16_t)((reg0a >> 12U) & 0x0FFFU);
    tmp.temperature_c_nominal = (int16_t)((tmp.temperature_raw >> 3U) - 256);
    tmp.i_raw = opt3101_sign_extend_24(i_value);
    tmp.q_raw = opt3101_sign_extend_24(q_value);
    tmp.distance_valid = !tmp.phase_overflow && !tmp.phase_overflow_f2 &&
                         !tmp.signal_overflow && !tmp.ambient_overflow;

    *measurement = tmp;
    dev->sample_pending = false;
    return opt3101_set_status(dev, OPT3101_STATUS_OK);
}

OPT3101_Status OPT3101_Sample(OPT3101_Device *dev,
                              OPT3101_Measurement *measurement)
{
    OPT3101_Status status = OPT3101_StartSample(dev);
    if (status != OPT3101_STATUS_OK) return status;

    while (!OPT3101_IsSampleDone(dev))
    {
        HAL_Delay(1U);
    }
    return OPT3101_ReadOutputRegisters(dev, measurement);
}

OPT3101_Status OPT3101_ReadDesignID(OPT3101_Device *dev,
                                    uint32_t *design_id_low,
                                    uint32_t *design_id_high)
{
    uint32_t reg2e;
    uint8_t previous_selector;
    OPT3101_Status status;

    if (!opt3101_context_valid(dev) || (design_id_low == NULL) ||
        (design_id_high == NULL))
    {
        return opt3101_set_status(dev, OPT3101_STATUS_INVALID_ARGUMENT);
    }

    status = OPT3101_ReadRegister(dev, REG_IQ_SELECTOR, &reg2e);
    if (status != OPT3101_STATUS_OK) return status;
    previous_selector = (uint8_t)((reg2e & IQ_READ_DATA_SEL_MASK) >> IQ_READ_DATA_SEL_SHIFT);

    status = OPT3101_UpdateRegisterBits(dev, REG_IQ_SELECTOR,
                                       IQ_READ_DATA_SEL_MASK, 4UL << IQ_READ_DATA_SEL_SHIFT);
    if (status != OPT3101_STATUS_OK) return status;
    status = OPT3101_ReadRegister(dev, REG_I_RAW, design_id_low);
    if (status != OPT3101_STATUS_OK) goto restore_selector;

    status = OPT3101_UpdateRegisterBits(dev, REG_IQ_SELECTOR,
                                       IQ_READ_DATA_SEL_MASK, 5UL << IQ_READ_DATA_SEL_SHIFT);
    if (status != OPT3101_STATUS_OK) goto restore_selector;
    status = OPT3101_ReadRegister(dev, REG_I_RAW, design_id_high);

restore_selector:
    {
        OPT3101_Status restore_status = OPT3101_UpdateRegisterBits(
            dev, REG_IQ_SELECTOR, IQ_READ_DATA_SEL_MASK,
            (uint32_t)previous_selector << IQ_READ_DATA_SEL_SHIFT);
        if (status == OPT3101_STATUS_OK)
        {
            status = restore_status;
        }
    }
    return status;
}

OPT3101_Status OPT3101_ValidateDesignID(OPT3101_Device *dev)
{
    uint32_t low_id, high_id;
    OPT3101_Status status = OPT3101_ReadDesignID(dev, &low_id, &high_id);
    if (status != OPT3101_STATUS_OK) return status;

    /* Expected IDs are taken from TI OPT3101 SDK v0.9.1 validateDesignID(). */
    if ((low_id != 0x100411UL) || (high_id != 0xC01000UL))
    {
        return opt3101_set_status(dev, OPT3101_STATUS_INVALID_CONFIGURATION);
    }
    return opt3101_set_status(dev, OPT3101_STATUS_OK);
}

OPT3101_Status OPT3101_GetLastStatus(const OPT3101_Device *dev)
{
    return (dev == NULL) ? OPT3101_STATUS_INVALID_ARGUMENT : dev->last_status;
}

HAL_StatusTypeDef OPT3101_GetLastHALStatus(const OPT3101_Device *dev)
{
    return (dev == NULL) ? HAL_ERROR : dev->last_hal_status;
}

const char *OPT3101_StatusString(OPT3101_Status status)
{
    switch (status)
    {
        case OPT3101_STATUS_OK: return "OK";
        case OPT3101_STATUS_INVALID_ARGUMENT: return "invalid argument";
        case OPT3101_STATUS_I2C_ERROR: return "I2C communication error";
        case OPT3101_STATUS_INIT_TIMEOUT: return "startup configuration timeout";
        case OPT3101_STATUS_NOT_INITIALIZED: return "device not initialized";
        case OPT3101_STATUS_INVALID_CONFIGURATION: return "invalid configuration / design ID mismatch";
        case OPT3101_STATUS_NOT_READY: return "measurement not ready";
        default: return "unknown status";
    }
}
