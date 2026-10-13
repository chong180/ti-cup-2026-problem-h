#include "drivers/encoder_odometry.h"

#include "ti_msp_dl_config.h"
#include "drivers/timebase.h"

/**
 * @file encoder_distance.c
 * @brief 双轮编码器累计路程模块实现 (移植版, 适配 MSPM0G3507 当前工程)。
 *
 * 与原始模块的差异:
 *   1. 引脚改用手动配置 (本工程 SysConfig 不含编码器GPIO组);
 *   2. 增加 EncoderDistance_Update() 测速窗口采样与 mm/s 速度接口,
 *      供 OLED 显示与无线遥测使用;
 *   3. 计数/中断分发逻辑与原始模块保持一致:
 *      A、B相上升沿中断, 每组GPIO中断内逐条读取挂起IIDX并计数。
 */

#define ENCODER_DISTANCE_COUNTS_PER_WHEEL_REV \
    (ENCODER_DISTANCE_MOTOR_PPR * ENCODER_DISTANCE_GEAR_RATIO * \
     ENCODER_DISTANCE_EDGES_PER_PULSE)
#define ENCODER_DISTANCE_PI (3.14159265358979323846f)
#define ENCODER_DISTANCE_MM_PER_COUNT \
    ((ENCODER_DISTANCE_PI * ENCODER_DISTANCE_WHEEL_DIAMETER_MM) / \
     (float)ENCODER_DISTANCE_COUNTS_PER_WHEEL_REV)

#define ENCODER_DISTANCE_LEFT_PINS  (ENCODER_DISTANCE_LEFT_PHASE_A_PIN | \
                                     ENCODER_DISTANCE_LEFT_PHASE_B_PIN)
#define ENCODER_DISTANCE_RIGHT_PINS (ENCODER_DISTANCE_RIGHT_PHASE_A_PIN | \
                                     ENCODER_DISTANCE_RIGHT_PHASE_B_PIN)

static volatile uint32_t gLeftEncoderCount = 0U;
static volatile uint32_t gRightEncoderCount = 0U;

/* ======================== 测速状态 (主循环上下文) ======================== */
static uint32_t gLastLeftCount  = 0U;
static uint32_t gLastRightCount = 0U;
static uint32_t gLastSpeedMs    = 0U;
static float    gSpeedLeftMmPs  = 0.0f;
static float    gSpeedRightMmPs = 0.0f;

/**
 * @brief 原子读取左右轮计数快照。
 * @param left 用于保存左轮计数的非空指针。
 * @param right 用于保存右轮计数的非空指针。
 */
static void EncoderDistance_GetCounts(uint32_t *left, uint32_t *right)
{
    /*
     * 两组 GPIO 都属于 GROUP1。短暂关闭该 IRQ 后同时读取左右计数，
     * 避免读取期间恰好进入中断而造成左右轮快照不同步。
     */
    NVIC_DisableIRQ(ENCODER_DISTANCE_IRQN);
    *left = gLeftEncoderCount;
    *right = gRightEncoderCount;
    NVIC_EnableIRQ(ENCODER_DISTANCE_IRQN);
}

/** @brief 手动配置引脚、清零计数、清除挂起标志并使能编码器GPIO中断。 */
void EncoderDistance_Init(void)
{
    /* GPIOA/GPIOB 电源在 SYSCFG_DL_init() 中已开启, 此处为防御性幂等使能 */
    DL_GPIO_enablePower(ENCODER_DISTANCE_LEFT_PORT);
    DL_GPIO_enablePower(ENCODER_DISTANCE_RIGHT_PORT);

    /* ---- 1. 四根引脚: 数字输入 + 内部上拉 + 迟滞(抗干扰) ---- */
    DL_GPIO_initDigitalInputFeatures(ENCODER_DISTANCE_LEFT_PHASE_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(ENCODER_DISTANCE_LEFT_PHASE_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(ENCODER_DISTANCE_RIGHT_PHASE_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(ENCODER_DISTANCE_RIGHT_PHASE_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    /* ---- 2. 数字滤波 8个ULPCLK周期 (~0.25us@32MHz), 过滤毛刺防中断风暴 ---- */
    DL_GPIO_setLowerPinsInputFilter(ENCODER_DISTANCE_LEFT_PORT,
        DL_GPIO_PIN_8_INPUT_FILTER_8_CYCLES |
            DL_GPIO_PIN_9_INPUT_FILTER_8_CYCLES);
    DL_GPIO_setLowerPinsInputFilter(ENCODER_DISTANCE_RIGHT_PORT,
        DL_GPIO_PIN_15_INPUT_FILTER_8_CYCLES);
    DL_GPIO_setUpperPinsInputFilter(ENCODER_DISTANCE_RIGHT_PORT,
        DL_GPIO_PIN_16_INPUT_FILTER_8_CYCLES);

    /* ---- 3. 上升沿触发 (A、B两路均计上升沿 -> 2倍频) ----
     * PB08/PB09/PA15 在低半组极性寄存器, PA16 在高半组 */
    DL_GPIO_setLowerPinsPolarity(ENCODER_DISTANCE_LEFT_PORT,
        DL_GPIO_PIN_8_EDGE_RISE | DL_GPIO_PIN_9_EDGE_RISE);
    DL_GPIO_setLowerPinsPolarity(ENCODER_DISTANCE_RIGHT_PORT,
        DL_GPIO_PIN_15_EDGE_RISE);
    DL_GPIO_setUpperPinsPolarity(ENCODER_DISTANCE_RIGHT_PORT,
        DL_GPIO_PIN_16_EDGE_RISE);

    /* ---- 4. 清零状态 -> 清挂起 -> 使能引脚中断 -> 开NVIC ---- */
    gLeftEncoderCount  = 0U;
    gRightEncoderCount = 0U;
    gLastLeftCount     = 0U;
    gLastRightCount    = 0U;
    gSpeedLeftMmPs     = 0.0f;
    gSpeedRightMmPs    = 0.0f;

    DL_GPIO_clearInterruptStatus(ENCODER_DISTANCE_LEFT_PORT,
                                 ENCODER_DISTANCE_LEFT_PINS);
    DL_GPIO_clearInterruptStatus(ENCODER_DISTANCE_RIGHT_PORT,
                                 ENCODER_DISTANCE_RIGHT_PINS);
    DL_GPIO_enableInterrupt(ENCODER_DISTANCE_LEFT_PORT,
                            ENCODER_DISTANCE_LEFT_PINS);
    DL_GPIO_enableInterrupt(ENCODER_DISTANCE_RIGHT_PORT,
                            ENCODER_DISTANCE_RIGHT_PINS);

    /* MSPM0G3507: GPIOA/GPIOB 共用 IRQ1, 使能一次即可覆盖两个端口 */
    NVIC_SetPriority(ENCODER_DISTANCE_IRQN, 1U); /* 较高优先级, 保证不丢脉冲 */
    NVIC_ClearPendingIRQ(ENCODER_DISTANCE_IRQN);
    NVIC_EnableIRQ(ENCODER_DISTANCE_IRQN);

    gLastSpeedMs = Timebase_GetMillis();
}

/** @brief 在临界区内将左右轮累计计数与测速状态安全清零。 */
void EncoderDistance_Reset(void)
{
    NVIC_DisableIRQ(ENCODER_DISTANCE_IRQN);
    gLeftEncoderCount = 0U;
    gRightEncoderCount = 0U;
    DL_GPIO_clearInterruptStatus(ENCODER_DISTANCE_LEFT_PORT,
                                 ENCODER_DISTANCE_LEFT_PINS);
    DL_GPIO_clearInterruptStatus(ENCODER_DISTANCE_RIGHT_PORT,
                                 ENCODER_DISTANCE_RIGHT_PINS);
    NVIC_ClearPendingIRQ(ENCODER_DISTANCE_IRQN);
    NVIC_EnableIRQ(ENCODER_DISTANCE_IRQN);

    gLastLeftCount  = 0U;
    gLastRightCount = 0U;
    gSpeedLeftMmPs  = 0.0f;
    gSpeedRightMmPs = 0.0f;
    gLastSpeedMs    = Timebase_GetMillis();
}

/** @brief 测速窗口采样: 用实际经过毫秒数把窗口计数差换算成 mm/s。 */
void EncoderDistance_Update(void)
{
    uint32_t now = Timebase_GetMillis();
    uint32_t dt  = now - gLastSpeedMs; /* 无符号减法自动处理49天回绕 */

    if (dt < (uint32_t)ENCODER_DISTANCE_SPEED_WINDOW_MS) {
        return;                        /* 未到窗口, 直接返回 */
    }
    gLastSpeedMs = now;

    uint32_t left;
    uint32_t right;
    EncoderDistance_GetCounts(&left, &right);

    /* 窗口内增量 (无符号差值, 计数只增不减, 无回绕风险) */
    uint32_t dL = left - gLastLeftCount;
    uint32_t dR = right - gLastRightCount;
    gLastLeftCount  = left;
    gLastRightCount = right;

    float dtSec = (float)dt * 0.001f;
    gSpeedLeftMmPs  = (float)dL * ENCODER_DISTANCE_MM_PER_COUNT / dtSec;
    gSpeedRightMmPs = (float)dR * ENCODER_DISTANCE_MM_PER_COUNT / dtSec;
}

/** @brief 返回原子快照中的左轮累计上升沿计数。 */
uint32_t EncoderDistance_GetLeftCount(void)
{
    uint32_t left;
    uint32_t right;
    EncoderDistance_GetCounts(&left, &right);
    (void)right;
    return left;
}

/** @brief 返回原子快照中的右轮累计上升沿计数。 */
uint32_t EncoderDistance_GetRightCount(void)
{
    uint32_t left;
    uint32_t right;
    EncoderDistance_GetCounts(&left, &right);
    (void)left;
    return right;
}

/** @brief 将左轮累计计数换算为毫米。 */
float EncoderDistance_GetLeftMm(void)
{
    return (float)EncoderDistance_GetLeftCount() *
           ENCODER_DISTANCE_MM_PER_COUNT;
}

/** @brief 将右轮累计计数换算为毫米。 */
float EncoderDistance_GetRightMm(void)
{
    return (float)EncoderDistance_GetRightCount() *
           ENCODER_DISTANCE_MM_PER_COUNT;
}

/** @brief 返回左右轮累计路程的平均值，单位毫米。 */
float EncoderDistance_GetAverageMm(void)
{
    uint32_t left;
    uint32_t right;
    EncoderDistance_GetCounts(&left, &right);
    return ((float)left + (float)right) * 0.5f *
           ENCODER_DISTANCE_MM_PER_COUNT;
}

/** @brief 返回单位为0.1毫米的平均累计路程整数值。 */
uint32_t EncoderDistance_GetAverageMmX10(void)
{
    uint32_t left;
    uint32_t right;
    float distanceMmX10;

    EncoderDistance_GetCounts(&left, &right);
    distanceMmX10 =
        ((float)left + (float)right) * 5.0f *
        ENCODER_DISTANCE_MM_PER_COUNT;
    return (uint32_t)(distanceMmX10 + 0.5f);
}

/** @brief 返回左轮测速窗口均值速度，单位mm/s。 */
float EncoderDistance_GetLeftMmPs(void)
{
    return gSpeedLeftMmPs;
}

/** @brief 返回右轮测速窗口均值速度，单位mm/s。 */
float EncoderDistance_GetRightMmPs(void)
{
    return gSpeedRightMmPs;
}

/** @brief 返回左右轮平均速度，单位mm/s。 */
float EncoderDistance_GetAverageMmPs(void)
{
    return (gSpeedLeftMmPs + gSpeedRightMmPs) * 0.5f;
}

/** @brief GROUP1中断入口，分派并累计左右编码器的GPIO上升沿。 */
void GROUP1_IRQHandler(void)
{
    uint32_t groupIndex;
    DL_GPIO_IIDX pinIndex;

    /*
     * 一次 GROUP1 中断可能同时包含 GPIOA 和 GPIOB，也可能同一端口
     * 累积两个编码器边沿，因此循环读取，直到所有挂起标志都已处理。
     * (DL_GPIO_getPendingInterrupt 读取IIDX时自动清除对应标志)
     */
    while ((groupIndex =
                DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) != 0U) {
        if (groupIndex == ENCODER_DISTANCE_LEFT_GROUP_IIDX) {
            while ((pinIndex =
                        DL_GPIO_getPendingInterrupt(
                            ENCODER_DISTANCE_LEFT_PORT)) !=
                   DL_GPIO_IIDX_NO_INTR) {
                if ((pinIndex == ENCODER_DISTANCE_LEFT_PHASE_A_IIDX) ||
                    (pinIndex == ENCODER_DISTANCE_LEFT_PHASE_B_IIDX)) {
                    ++gLeftEncoderCount;
                }
            }
        } else if (groupIndex == ENCODER_DISTANCE_RIGHT_GROUP_IIDX) {
            while ((pinIndex =
                        DL_GPIO_getPendingInterrupt(
                            ENCODER_DISTANCE_RIGHT_PORT)) !=
                   DL_GPIO_IIDX_NO_INTR) {
                if ((pinIndex == ENCODER_DISTANCE_RIGHT_PHASE_A_IIDX) ||
                    (pinIndex == ENCODER_DISTANCE_RIGHT_PHASE_B_IIDX)) {
                    ++gRightEncoderCount;
                }
            }
        } else {
            break;
        }
    }
}
