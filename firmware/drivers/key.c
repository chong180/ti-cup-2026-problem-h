#include "drivers/key.h"

#include "drivers/delay.h"
#include "ti_msp_dl_config.h"

static uint8_t isPressed(Key_Event key)
{
    switch (key) {
    case KEY_S1:
        return (DL_GPIO_readPins(GPIO_Key_S1_PORT, GPIO_Key_S1_PIN) == 0U);
    case KEY_S2:
        return (DL_GPIO_readPins(GPIO_Key_S2_PORT, GPIO_Key_S2_PIN) == 0U);
    case KEY_S3:
        return (DL_GPIO_readPins(GPIO_Key_S3_PORT, GPIO_Key_S3_PIN) == 0U);
    case KEY_S4:
        return (DL_GPIO_readPins(GPIO_Key_S4_PORT, GPIO_Key_S4_PIN) == 0U);
    default:
        return 0U;
    }
}

Key_Event Key_GetEvent(void)
{
    Key_Event key;

    for (key = KEY_S1; key <= KEY_S4; key = (Key_Event)(key + 1)) {
        if (isPressed(key) != 0U) {
            Delay_ms(15U);
            if (isPressed(key) != 0U) {
                while (isPressed(key) != 0U) {
                    /* Wait for release so one press creates exactly one event. */
                }
                Delay_ms(15U);
                return key;
            }
        }
    }
    return KEY_NONE;
}
