#include "pmw3901_interface.h"

extern SPI_HandleTypeDef hspi1;

#define PMW3901_SPI_TIMEOUT_MS       100U
#define PMW3901_SPI_PRE_DELAY_US      50U
#define PMW3901_SPI_READ_DELAY_US     50U
#define PMW3901_SPI_POST_DELAY_US     50U

static void PMW3901_CS_Low(void)
{
    HAL_GPIO_WritePin(
        PMW3901_CS_GPIO_Port,
        PMW3901_CS_Pin,
        GPIO_PIN_RESET
    );
}

static void PMW3901_CS_High(void)
{
    HAL_GPIO_WritePin(
        PMW3901_CS_GPIO_Port,
        PMW3901_CS_Pin,
        GPIO_PIN_SET
    );
}

/*
 * DWT cycle-counter delay.
 *
 * Requires the Cortex-M4 DWT cycle counter to be available.
 * SystemCoreClock must reflect the configured CPU frequency.
 */
void PMW3901_Interface_DelayUs(uint32_t delay_us)
{
    uint32_t start;
    uint32_t cycles;

    if (delay_us == 0U)
    {
        return;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    cycles = (SystemCoreClock / 1000000U) * delay_us;
    start = DWT->CYCCNT;

    while ((uint32_t)(DWT->CYCCNT - start) < cycles)
    {
        /* Wait for the requested number of CPU cycles. */
    }
}

HAL_StatusTypeDef PMW3901_Interface_ReadRegister(
    uint8_t reg,
    uint8_t *value
)
{
    HAL_StatusTypeDef status;
    uint8_t address;
    uint8_t dummy = 0U;
    uint8_t received = 0U;

    if (value == NULL)
    {
        return HAL_ERROR;
    }

    address = reg & 0x7FU;

    PMW3901_CS_Low();

    PMW3901_Interface_DelayUs(PMW3901_SPI_PRE_DELAY_US);

    status = HAL_SPI_Transmit(
        &hspi1,
        &address,
        1U,
        PMW3901_SPI_TIMEOUT_MS
    );

    if (status == HAL_OK)
    {
        PMW3901_Interface_DelayUs(PMW3901_SPI_READ_DELAY_US);

        status = HAL_SPI_TransmitReceive(
            &hspi1,
            &dummy,
            &received,
            1U,
            PMW3901_SPI_TIMEOUT_MS
        );
    }

    PMW3901_Interface_DelayUs(PMW3901_SPI_POST_DELAY_US);

    PMW3901_CS_High();

    if (status == HAL_OK)
    {
        *value = received;
    }

    return status;
}

HAL_StatusTypeDef PMW3901_Interface_WriteRegister(
    uint8_t reg,
    uint8_t value
)
{
    HAL_StatusTypeDef status;
    uint8_t tx[2];

    tx[0] = reg | 0x80U;
    tx[1] = value;

    PMW3901_CS_Low();

    PMW3901_Interface_DelayUs(PMW3901_SPI_PRE_DELAY_US);

    status = HAL_SPI_Transmit(
        &hspi1,
        tx,
        sizeof(tx),
        PMW3901_SPI_TIMEOUT_MS
    );

    PMW3901_Interface_DelayUs(PMW3901_SPI_POST_DELAY_US);

    PMW3901_CS_High();

    /*
     * Allow time between register writes.
     */
    PMW3901_Interface_DelayUs(200U);

    return status;
}

HAL_StatusTypeDef PMW3901_Interface_ReadBurst(
    uint8_t reg,
    uint8_t *data,
    uint16_t length
)
{
    HAL_StatusTypeDef status;
    uint8_t address;
    uint8_t dummy[12] = {0};

    if ((data == NULL) ||
        (length == 0U) ||
        (length > sizeof(dummy)))
    {
        return HAL_ERROR;
    }

    address = reg & 0x7FU;

    PMW3901_CS_Low();

    PMW3901_Interface_DelayUs(PMW3901_SPI_PRE_DELAY_US);

    status = HAL_SPI_Transmit(
        &hspi1,
        &address,
        1U,
        PMW3901_SPI_TIMEOUT_MS
    );

    if (status == HAL_OK)
    {
        PMW3901_Interface_DelayUs(PMW3901_SPI_READ_DELAY_US);

        status = HAL_SPI_TransmitReceive(
            &hspi1,
            dummy,
            data,
            length,
            PMW3901_SPI_TIMEOUT_MS
        );
    }

    PMW3901_Interface_DelayUs(PMW3901_SPI_POST_DELAY_US);

    PMW3901_CS_High();

    return status;
}

void PMW3901_Interface_Reset(void)
{
    /*
     * Hardware reset is assumed active-low.
     * Confirm the breakout's reset circuitry before connecting it.
     */

    HAL_GPIO_WritePin(
        PMW3901_RST_GPIO_Port,
        PMW3901_RST_Pin,
        GPIO_PIN_RESET
    );

    HAL_Delay(2U);

    HAL_GPIO_WritePin(
        PMW3901_RST_GPIO_Port,
        PMW3901_RST_Pin,
        GPIO_PIN_SET
    );

    HAL_Delay(40U);
}
