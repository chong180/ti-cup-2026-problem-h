#include "drivers/timebase.h"

#include "ti_msp_dl_config.h"

static volatile uint32_t s_millis = 0U;
static volatile uint8_t s_ballControlTick = 0U;

void Timebase_Init(void)
{
    s_millis = 0U;
    s_ballControlTick = 0U;
    (void)SysTick_Config(CPUCLK_FREQ / 1000U);
    /* Camera RX remains priority 0; the 1 ms scheduler is priority 1. */
    NVIC_SetPriority(SysTick_IRQn, 1U);
}

uint32_t Timebase_GetMillis(void)
{
    return s_millis;
}

uint8_t Timebase_TakeBallControlTick(void)
{
    uint8_t pending;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    pending = s_ballControlTick;
    s_ballControlTick = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
    return pending;
}

void SysTick_Handler(void)
{
    ++s_millis;
    if ((s_millis % 5U) == 0U) {
        /*
         * Only publish a task flag here.  PD calculation and the blocking
         * stepper UART frame remain outside interrupt context.
         */
        s_ballControlTick = 1U;
    }
}
