#ifndef __ENCODER_ODOMETRY_H
#define __ENCODER_ODOMETRY_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

/*
 * @file encoder_odometry.h
 * @brief 双轮AB相增量编码器累计路程 + 测速模块 (移植版)。
 *
 * 移植说明 (适配 MSPM0G3507 当前工程):
 *   原模块依赖 SysConfig 生成的 ENCODERA/ENCODERB GPIO 组宏,
 *   本工程 SysConfig 未配置编码器引脚, 因此改为手动配置引脚
 *   (与 wireless_uart.c 风格一致, 重新生成 ti_msp_dl_config 不影响本模块)。
 *
 * 硬件接线 (与工程原有 encoder.c 一致):
 *   右轮编码器: A相=PA16 (PINCM38), B相=PA15 (PINCM37)  -> GPIOA
 *   左轮编码器: A相=PB09 (PINCM26), B相=PB08 (PINCM25)  -> GPIOB
 *   MSPM0G3507: GPIOA/GPIOB 共用 IRQ1, 向量名 GROUP1_IRQHandler,
 *   本模块独占该中断; 若其他模块将来也用GPIO中断, 需在ISR中追加分发。
 *
 * 计数方式:
 *   A、B两相均只计上升沿 (2倍频), 不区分正反转, 只累计路程。
 *   每轮计数 = 13 PPR × 28 (减速比) × 2 = 728 计数/轮,
 *   轮周长 = π × 65mm ≈ 204.2mm, 距离分辨率 ≈ 204.2 / 728 ≈ 0.280mm/计数。
 *
 * 使用方法:
 * 1. 在 SYSCFG_DL_init() 和 Timebase_Init() 之后调用 EncoderDistance_Init()。
 * 2. 主循环每次迭代调用 EncoderDistance_Update() (内部按窗口节流, 用于测速)。
 * 3. 新任务开始时调用 EncoderDistance_Reset() 清零里程。
 * 4. 调用距离/速度读取函数获得左右轮或车体平均值。
 *
 * 注意: 本模块不提供方向判别, 倒车时路程与速度仍正向累加。
 */

/* ======================== 机械参数 ======================== */

/* 电机轴单个编码器通道每转产生的脉冲数。 */
#define ENCODER_DISTANCE_MOTOR_PPR         (13U)
/* 减速箱传动比。例如1:28填写28。 */
#define ENCODER_DISTANCE_GEAR_RATIO        (28U)
/* 轮胎有效直径，单位mm；精确测距时应使用负载状态下的实测值。 */
#define ENCODER_DISTANCE_WHEEL_DIAMETER_MM (65.0f)
/* 当前A、B两路均计上升沿，因此每个编码器周期产生2个计数。 */
#define ENCODER_DISTANCE_EDGES_PER_PULSE   (2U)

/* 测速窗口 (ms): 窗口内计数差 / 实际时间 = 速度 */
#define ENCODER_DISTANCE_SPEED_WINDOW_MS   (10U)

/* ======================== 硬件映射 ======================== */

/*
 * 手动映射到 DriverLib 原始定义 (不依赖 SysConfig 生成宏)。
 * 左轮: GPIOB (A=PB09, B=PB08); 右轮: GPIOA (A=PA16, B=PA15)。
 */
#define ENCODER_DISTANCE_LEFT_PORT         GPIOB
#define ENCODER_DISTANCE_LEFT_PHASE_A_PIN  DL_GPIO_PIN_9
#define ENCODER_DISTANCE_LEFT_PHASE_B_PIN  DL_GPIO_PIN_8
#define ENCODER_DISTANCE_LEFT_PHASE_A_IIDX DL_GPIO_IIDX_DIO9
#define ENCODER_DISTANCE_LEFT_PHASE_B_IIDX DL_GPIO_IIDX_DIO8
#define ENCODER_DISTANCE_LEFT_PHASE_A_IOMUX IOMUX_PINCM26
#define ENCODER_DISTANCE_LEFT_PHASE_B_IOMUX IOMUX_PINCM25
#define ENCODER_DISTANCE_LEFT_GROUP_IIDX   DL_INTERRUPT_GROUP1_IIDX_GPIOB

#define ENCODER_DISTANCE_RIGHT_PORT         GPIOA
#define ENCODER_DISTANCE_RIGHT_PHASE_A_PIN  DL_GPIO_PIN_16
#define ENCODER_DISTANCE_RIGHT_PHASE_B_PIN  DL_GPIO_PIN_15
#define ENCODER_DISTANCE_RIGHT_PHASE_A_IIDX DL_GPIO_IIDX_DIO16
#define ENCODER_DISTANCE_RIGHT_PHASE_B_IIDX DL_GPIO_IIDX_DIO15
#define ENCODER_DISTANCE_RIGHT_PHASE_A_IOMUX IOMUX_PINCM38
#define ENCODER_DISTANCE_RIGHT_PHASE_B_IOMUX IOMUX_PINCM37
#define ENCODER_DISTANCE_RIGHT_GROUP_IIDX   DL_INTERRUPT_GROUP1_IIDX_GPIOA

/* MSPM0G3507: GPIOA/GPIOB 共用 IRQ1 (GPIOA_INT_IRQn == GPIOB_INT_IRQn == 1) */
#define ENCODER_DISTANCE_IRQN              GPIOA_INT_IRQn

/**
 * @brief 初始化编码器测距模块并使能GPIO中断。
 *
 * @note 必须在 SYSCFG_DL_init() 和 Timebase_Init() 之后调用。
 *       手动完成引脚配置 (上拉输入/迟滞/8周期滤波/上升沿中断)。
 * @param 无。
 * @return 无。
 */
void EncoderDistance_Init(void);

/**
 * @brief 将左右轮累计计数、距离和测速状态同时清零。
 *
 * @note 通常在一次新的行驶任务开始前调用。
 * @param 无。
 * @return 无。
 */
void EncoderDistance_Reset(void);

/**
 * @brief 测速周期更新 (内部按 ENCODER_DISTANCE_SPEED_WINDOW_MS 自动节流)。
 *
 * @note 主循环每次迭代调用一次即可; 使用实际经过的毫秒数计算,
 *       OLED刷新等阻塞不会造成测速失真。只累计路程时不调用也可以。
 * @param 无。
 * @return 无。
 */
void EncoderDistance_Update(void);

/**
 * @brief 获取左轮累计上升沿计数。
 *
 * @param 无。
 * @return 左编码器A、B两路累计上升沿数量。
 */
uint32_t EncoderDistance_GetLeftCount(void);

/**
 * @brief 获取右轮累计上升沿计数。
 *
 * @param 无。
 * @return 右编码器A、B两路累计上升沿数量。
 */
uint32_t EncoderDistance_GetRightCount(void);

/**
 * @brief 获取左轮累计路程。
 *
 * @param 无。
 * @return 左轮累计路程，单位mm。
 */
float EncoderDistance_GetLeftMm(void);

/**
 * @brief 获取右轮累计路程。
 *
 * @param 无。
 * @return 右轮累计路程，单位mm。
 */
float EncoderDistance_GetRightMm(void);

/**
 * @brief 获取车体平均累计路程。
 *
 * 计算公式为：(左轮累计路程 + 右轮累计路程) / 2。
 *
 * @param 无。
 * @return 车体平均累计路程，单位mm。
 */
float EncoderDistance_GetAverageMm(void);

/**
 * @brief 获取车体平均累计路程的定点整数值。
 *
 * 该接口不需要 printf 浮点支持，适合OLED等嵌入式显示。
 * 例如返回12345表示1234.5 mm。
 *
 * @param 无。
 * @return 车体平均累计路程，单位0.1 mm。
 */
uint32_t EncoderDistance_GetAverageMmX10(void);

/**
 * @brief 获取左轮瞬时速度 (测速窗口均值)。
 *
 * @note 需要先周期调用 EncoderDistance_Update()。无方向, 恒为非负。
 * @param 无。
 * @return 左轮速度，单位mm/s。
 */
float EncoderDistance_GetLeftMmPs(void);

/**
 * @brief 获取右轮瞬时速度 (测速窗口均值)。
 *
 * @note 需要先周期调用 EncoderDistance_Update()。无方向, 恒为非负。
 * @param 无。
 * @return 右轮速度，单位mm/s。
 */
float EncoderDistance_GetRightMmPs(void);

/**
 * @brief 获取车体平均速度: (左轮 + 右轮) / 2。
 *
 * @param 无。
 * @return 车体平均速度，单位mm/s。
 */
float EncoderDistance_GetAverageMmPs(void);

/**
 * @brief 获取当前编码器距离微调值 (S3/S4按键调节)。
 * 各模式启动时读取此值加到总目标距离。
 * @return 微调距离，单位mm (步距5mm=0.5cm)。
 */
float EncoderDistance_GetAdjustMm(void);

#endif
