#include "pmw3901.h"
#include "pmw3901_interface.h"
#include "main.h"

static bool pmw3901_initialized = false;

typedef struct
{
    uint8_t reg;
    uint8_t value;
} PMW3901_RegisterSetting;

static PMW3901_Status PMW3901_WriteRegister(
    uint8_t reg,
    uint8_t value
)
{
    if (PMW3901_Interface_WriteRegister(reg, value) != HAL_OK)
    {
        return PMW3901_COMMUNICATION_ERROR;
    }

    return PMW3901_OK;
}

PMW3901_Status PMW3901_ReadRegister(
    uint8_t reg,
    uint8_t *value
)
{
    if (value == NULL)
    {
        return PMW3901_INVALID_ARGUMENT;
    }

    if (PMW3901_Interface_ReadRegister(reg, value) != HAL_OK)
    {
        return PMW3901_COMMUNICATION_ERROR;
    }

    return PMW3901_OK;
}

bool PMW3901_IsInitialized(void)
{
    return pmw3901_initialized;
}

/*
 * Performance initialization sequence adapted from the
 * Bitcraze PMW3901 reference driver.
 *
 * Source reference:
 * https://github.com/bitcraze/crazyflie-firmware/blob/master/src/drivers/src/pmw3901.c
 */
static PMW3901_Status PMW3901_ConfigureRegisters(void)
{
    static const PMW3901_RegisterSetting settings[] =
    {
        {0x7F, 0x00}, {0x61, 0xAD},
        {0x7F, 0x03}, {0x40, 0x00},
        {0x7F, 0x05}, {0x41, 0xB3},
        {0x43, 0xF1}, {0x45, 0x14},
        {0x5B, 0x32}, {0x5F, 0x34},
        {0x7B, 0x08},
        {0x7F, 0x06}, {0x44, 0x1B},
        {0x40, 0xBF}, {0x4E, 0x3F},
        {0x7F, 0x08}, {0x65, 0x20},
        {0x6A, 0x18},
        {0x7F, 0x09}, {0x4F, 0xAF},
        {0x5F, 0x40}, {0x48, 0x80},
        {0x49, 0x80}, {0x57, 0x77},
        {0x60, 0x78}, {0x61, 0x78},
        {0x62, 0x08}, {0x63, 0x50},
        {0x7F, 0x0A}, {0x45, 0x60},
        {0x7F, 0x00}, {0x4D, 0x11},
        {0x55, 0x80}, {0x74, 0x1F},
        {0x75, 0x1F}, {0x4A, 0x78},
        {0x4B, 0x78}, {0x44, 0x08},
        {0x45, 0x50}, {0x64, 0xFF},
        {0x65, 0x1F},
        {0x7F, 0x14}, {0x65, 0x67},
        {0x66, 0x08}, {0x63, 0x70},
        {0x7F, 0x15}, {0x48, 0x48},
        {0x7F, 0x07}, {0x41, 0x0D},
        {0x43, 0x14}, {0x4B, 0x0E},
        {0x45, 0x0F}, {0x44, 0x42},
        {0x4C, 0x80},
        {0x7F, 0x10}, {0x5B, 0x02},
        {0x7F, 0x07}, {0x40, 0x41},
        {0x70, 0x00}
    };

    for (uint32_t i = 0U;
         i < (sizeof(settings) / sizeof(settings[0]));
         i++)
    {
        if (PMW3901_WriteRegister(
                settings[i].reg,
                settings[i].value) != PMW3901_OK)
        {
            return PMW3901_COMMUNICATION_ERROR;
        }
    }

    HAL_Delay(10U);

    static const PMW3901_RegisterSetting final_settings[] =
    {
        {0x32, 0x44},
        {0x7F, 0x07},
        {0x40, 0x40},
        {0x7F, 0x06},
        {0x62, 0xF0},
        {0x63, 0x00},
        {0x7F, 0x0D},
        {0x48, 0xC0},
        {0x6F, 0xD5},
        {0x7F, 0x00},
        {0x5B, 0xA0},
        {0x4E, 0xA8},
        {0x5A, 0x50},
        {0x40, 0x80}
    };

    for (uint32_t i = 0U;
         i < (sizeof(final_settings) / sizeof(final_settings[0]));
         i++)
    {
        if (PMW3901_WriteRegister(
                final_settings[i].reg,
                final_settings[i].value) != PMW3901_OK)
        {
            return PMW3901_COMMUNICATION_ERROR;
        }
    }

    return PMW3901_OK;
}

PMW3901_Status PMW3901_Init(void)
{
    uint8_t product_id = 0U;
    uint8_t inverse_id = 0U;
    uint8_t discard = 0U;

    pmw3901_initialized = false;

    /*
     * Allow power supplies and sensor startup to settle.
     */
    HAL_Delay(40U);

    /*
     * Reset the sensor through the hardware reset pin.
     */
    PMW3901_Interface_Reset();

    /*
     * Reset the SPI interface by toggling chip select.
     */
    HAL_GPIO_WritePin(
        PMW3901_CS_GPIO_Port,
        PMW3901_CS_Pin,
        GPIO_PIN_SET
    );

    HAL_Delay(2U);

    HAL_GPIO_WritePin(
        PMW3901_CS_GPIO_Port,
        PMW3901_CS_Pin,
        GPIO_PIN_RESET
    );

    HAL_Delay(2U);

    HAL_GPIO_WritePin(
        PMW3901_CS_GPIO_Port,
        PMW3901_CS_Pin,
        GPIO_PIN_SET
    );

    HAL_Delay(2U);

    /*
     * Verify the device identity.
     */
    if (PMW3901_ReadRegister(
            PMW3901_REG_PRODUCT_ID,
            &product_id) != PMW3901_OK)
    {
        return PMW3901_COMMUNICATION_ERROR;
    }

    if (PMW3901_ReadRegister(
            PMW3901_REG_INVERSE_ID,
            &inverse_id) != PMW3901_OK)
    {
        return PMW3901_COMMUNICATION_ERROR;
    }

    if ((product_id != PMW3901_PRODUCT_ID) ||
        (inverse_id != PMW3901_INVERSE_PRODUCT_ID))
    {
        return PMW3901_INVALID_DEVICE;
    }

    /*
     * Perform the sensor power-up reset.
     */
    if (PMW3901_WriteRegister(
            PMW3901_REG_POWER_UP_RESET,
            0x5AU) != PMW3901_OK)
    {
        return PMW3901_COMMUNICATION_ERROR;
    }

    HAL_Delay(5U);

    /*
     * Read motion registers once after reset.
     */
    const uint8_t startup_registers[] =
    {
        0x02U, 0x03U, 0x04U, 0x05U, 0x06U
    };

    for (uint32_t i = 0U;
         i < sizeof(startup_registers);
         i++)
    {
        if (PMW3901_ReadRegister(
                startup_registers[i],
                &discard) != PMW3901_OK)
        {
            return PMW3901_COMMUNICATION_ERROR;
        }
    }

    HAL_Delay(1U);

    if (PMW3901_ConfigureRegisters() != PMW3901_OK)
    {
        return PMW3901_COMMUNICATION_ERROR;
    }

    pmw3901_initialized = true;

    return PMW3901_OK;
}

PMW3901_Status PMW3901_ReadMotion(
    PMW3901_MotionData *motion
)
{
    uint8_t burst[12] = {0};

    if (motion == NULL)
    {
        return PMW3901_INVALID_ARGUMENT;
    }

    if (!pmw3901_initialized)
    {
        return PMW3901_NOT_INITIALIZED;
    }

    if (PMW3901_Interface_ReadBurst(
            PMW3901_REG_MOTION_BURST,
            burst,
            sizeof(burst)) != HAL_OK)
    {
        return PMW3901_COMMUNICATION_ERROR;
    }

    motion->motion = burst[0];
    motion->observation = burst[1];

    motion->delta_x = (int16_t)(
        (uint16_t)burst[2] |
        ((uint16_t)burst[3] << 8)
    );

    motion->delta_y = (int16_t)(
        (uint16_t)burst[4] |
        ((uint16_t)burst[5] << 8)
    );

    motion->surface_quality = burst[6];
    motion->raw_data_sum = burst[7];
    motion->maximum_raw_data = burst[8];
    motion->minimum_raw_data = burst[9];

    motion->shutter = (uint16_t)(
        ((uint16_t)burst[11] << 8) |
        (uint16_t)burst[10]
    );

    return PMW3901_OK;
}
