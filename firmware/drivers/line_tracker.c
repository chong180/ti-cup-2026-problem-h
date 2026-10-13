#include "drivers/line_tracker.h"

#include "ti_msp_dl_config.h"

/* ===========================================================================
 *  8路灰度传感器映射说明
 * ===========================================================================
 *  传感器为低电平有效(检测到黑线时GPIO读数为0)。
 *
 *  实际物理排列(从车头看, 从左到右):
 *    Gray_8  Gray_7  Gray_6  Gray_5  Gray_4  Gray_3  Gray_2  Gray_1
 *    PB18    PA14    PB20    PB24    PB25    PA24    PB19    PA30
 *    最左                                                         最右
 *
 *  代码中数组下标 0~7 对应 Gray_1~Gray_8, 即下标0是最右端, 下标7是最左端。
 *  权重定义: 最左端=-35(负, 表示线在左侧), 最右端=+35(正, 表示线在右侧)。
 *  error < 0 : 黑线在物理左侧 -> 需要左转
 *  error > 0 : 黑线在物理右侧 -> 需要右转
 *
 *  注意: 实际接线中 Gray_1~Gray_8 的物理顺序与命名顺序相反,
 *        因此权重数组也做了对应反转, 使 error 的物理含义正确。
 * =========================================================================== */
static const int8_t kSensorWeight[8] = {35, 25, 15, 5, -5, -15, -25, -35};

static uint8_t sensorActive(GPIO_Regs *port, uint32_t pin)
{
    return (DL_GPIO_readPins(port, pin) == 0U) ? 1U : 0U;
}

void LineTracker_Read(LineTracker_State *state)
{
    static GPIO_Regs *const kSensorPort[8] = {
        GPIO_Gray_Gray_1_PORT,
        GPIO_Gray_Gray_2_PORT,
        GPIO_Gray_Gray_3_PORT,
        GPIO_Gray_Gray_4_PORT,
        GPIO_Gray_Gray_5_PORT,
        GPIO_Gray_Gray_6_PORT,
        GPIO_Gray_Gray_7_PORT,
        GPIO_Gray_Gray_8_PORT,
    };
    static const uint32_t kSensorPin[8] = {
        GPIO_Gray_Gray_1_PIN,
        GPIO_Gray_Gray_2_PIN,
        GPIO_Gray_Gray_3_PIN,
        GPIO_Gray_Gray_4_PIN,
        GPIO_Gray_Gray_5_PIN,
        GPIO_Gray_Gray_6_PIN,
        GPIO_Gray_Gray_7_PIN,
        GPIO_Gray_Gray_8_PIN,
    };
    int16_t weightedSum = 0;
    uint8_t count = 0;
    uint8_t mask = 0;
    uint8_t index;

    for (index = 0; index < 8U; ++index) {
        if (sensorActive(kSensorPort[index], kSensorPin[index]) != 0U) {
            mask |= (uint8_t)(1U << index);
            weightedSum += kSensorWeight[index];
            ++count;
        }
    }

    state->activeMask = mask;
    state->lineFound = (count != 0U) ? 1U : 0U;
    state->error = (count != 0U) ? (int8_t)(weightedSum / (int16_t)count) : 0;
}
