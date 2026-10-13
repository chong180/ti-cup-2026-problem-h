#include "drivers/motor_tb6612.h"

#include "ti_msp_dl_config.h"

/* Must match PWM_MOTOR.timerCount in empty.syscfg. */
#define TB6612_PWM_PERIOD_TICKS (1600U)

/* ===========================================================================
 *  TB6612 电机驱动接线说明
 * ===========================================================================
 *  实际物理接线中, TB6612的A通道接的是右轮, B通道接的是左轮,
 *  与TB6612数据手册的“A=左/B=右”默认惯例相反。
 *
 *  通道映射:
 *    左轮 = B通道: BIN1=PA13, BIN2=PA12, PWMB=PB21 (TIMG8 CCP0)
 *    右轮 = A通道: AIN1=PB0,  AIN2=PB1,  PWMA=PB22 (TIMG8 CCP1)
 *    STBY = +5V (常使能)
 *
 *  正转方向(前进):
 *    左轮(B通道): BIN1=0, BIN2=1
 *    右轮(A通道): AIN1=0, AIN2=1
 *  若实际电机正反方向不一致, 在下方 setLeftDirection/setRightDirection
 *  中交换 set/clear 即可。
 * =========================================================================== */

static uint8_t clampSpeed(int16_t speed)
{
    if (speed < 0) {
        speed = -speed;
    }
    if (speed > TB6612_MAX_SPEED_PERCENT) {
        speed = TB6612_MAX_SPEED_PERCENT;
    }
    return (uint8_t)speed;
}

static void setLeftDirection(int8_t speed)
{
    /* 左轮 = B通道 (BIN1/BIN2)
     * speed > 0: 前进 (BIN1=0, BIN2=1)
     * speed < 0: 后退 (BIN1=1, BIN2=0)
     * speed = 0: 停止 (BIN1=0, BIN2=0) */
    if (speed > 0) {
        DL_GPIO_clearPins(GPIO_TB6612_BIN1_PORT, GPIO_TB6612_BIN1_PIN);
        DL_GPIO_setPins(GPIO_TB6612_BIN2_PORT, GPIO_TB6612_BIN2_PIN);
    } else if (speed < 0) {
        DL_GPIO_setPins(GPIO_TB6612_BIN1_PORT, GPIO_TB6612_BIN1_PIN);
        DL_GPIO_clearPins(GPIO_TB6612_BIN2_PORT, GPIO_TB6612_BIN2_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_TB6612_BIN1_PORT, GPIO_TB6612_BIN1_PIN);
        DL_GPIO_clearPins(GPIO_TB6612_BIN2_PORT, GPIO_TB6612_BIN2_PIN);
    }
}

static void setRightDirection(int8_t speed)
{
    /* 右轮 = A通道 (AIN1/AIN2)
     * speed > 0: 前进 (AIN1=0, AIN2=1)
     * speed < 0: 后退 (AIN1=1, AIN2=0)
     * speed = 0: 停止 (AIN1=0, AIN2=0) */
    if (speed > 0) {
        DL_GPIO_clearPins(GPIO_TB6612_AIN1_PORT, GPIO_TB6612_AIN1_PIN);
        DL_GPIO_setPins(GPIO_TB6612_AIN2_PORT, GPIO_TB6612_AIN2_PIN);
    } else if (speed < 0) {
        DL_GPIO_setPins(GPIO_TB6612_AIN1_PORT, GPIO_TB6612_AIN1_PIN);
        DL_GPIO_clearPins(GPIO_TB6612_AIN2_PORT, GPIO_TB6612_AIN2_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_TB6612_AIN1_PORT, GPIO_TB6612_AIN1_PIN);
        DL_GPIO_clearPins(GPIO_TB6612_AIN2_PORT, GPIO_TB6612_AIN2_PIN);
    }
}

/* TimerG PWM output is active during the part of the period after the compare
 * event, therefore a larger compare value gives a smaller duty cycle. */
static uint32_t speedToCompare(uint8_t speed)
{
    return (TB6612_PWM_PERIOD_TICKS * (100U - speed)) / 100U;
}

static void setPwmDuty(uint8_t leftPercent, uint8_t rightPercent)
{
    /* CCP0=PB21/PWMB -> 左轮(B通道)
     * CCP1=PB22/PWMA -> 右轮(A通道) */
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,
                                     speedToCompare(leftPercent),
                                     DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST,
                                     speedToCompare(rightPercent),
                                     DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
}

void TB6612_Init(void)
{
    TB6612_Stop();
}

void TB6612_SetSpeed(int8_t leftPercent, int8_t rightPercent)
{
    uint8_t leftDuty = clampSpeed(leftPercent);
    uint8_t rightDuty = clampSpeed(rightPercent);

    setLeftDirection(leftPercent);
    setRightDirection(rightPercent);
    setPwmDuty(leftDuty, rightDuty);
}

void TB6612_Stop(void)
{
    setLeftDirection(0);
    setRightDirection(0);
    setPwmDuty(0, 0);
}
