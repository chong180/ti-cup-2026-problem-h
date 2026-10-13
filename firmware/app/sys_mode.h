/**
 * @file    sys_mode.h
 * @brief   2024电赛H题第一问 — A到B陀螺仪直行驱动
 *
 * 功能:
 *   1. S2(PA26) 切换模式: 循迹模式 <-> AB直行模式
 *   2. AB模式: S1(PA25) 启动后, 记录当前Yaw为目标航向
 *      - LEAVE阶段: 驶离起始黑线 (连续全白AB_LEAVE_CONFIRM_MS后切换到RUN)
 *      - RUN阶段:  陀螺仪角度P环直行, 经过AB_RUN_GRACE_MS宽限期后开始监测终点黑线
 *   3. OLED全程显示模式类型 + 陀螺仪Yaw角度
 *
 * 灰度传感器: 低电平=黑线(active), 高电平=白底
 *
 * 赛道路径: A(黑线) -> 全白直行段 -> B(黑线) 停车
 */
#ifndef __SYS_MODE_H
#define __SYS_MODE_H

#include "ti_msp_dl_config.h"
#include "drivers/line_tracker.h"

/* ======================== 系统模式 ======================== */
typedef enum {
    MODE_LINE_TRACK = 0,   /* 灰度循迹模式 (保留原有功能) */
    MODE_AB         = 1,   /* AB直行模式 (2024电赛H题第一问) */
    MODE_BALL_LINE  = 2,   /* 串级PID钢球回零模式 */
    MODE_BALL_ROUTE = 3,   /* 钢球 0 -> +5cm -> -5cm 轨迹模式 */
    MODE_BALL_AB20  = 4,   /* 钢球稳0 + 小车20%定距1.5m */
    MODE_TRACK50    = 5,   /* 高速循迹模式 (50% PWM, 在基础循迹上提速) */
    MODE_REMOTE     = 6,   /* 遥控模式 (串口遥控, 原地转弯/前进后退) */
    MODE_DRIVE1     = 7,   /* 驱动1: 灰度循迹+编码器定距440cm停车 (2026新增) */
    MODE_ABCD       = 8,   /* 1.5m直道+1.57m弯+1.5m直道+1.57m弯 */
    MODE_HF_ZERO    = 9,   /* 高频预测PD钢球回零，底盘静止 */
    MODE_HOLD6     = 10,  /* 静止定点平衡：按键时刻球位置为目标 */
    MODE_006       = 11,  /* mode6圆弧回A + 50cm直线循迹 */
    MODE_007       = 12,  /* mode6圆弧回A直接停车 */
    MODE_5NEW      = 13,  /* 线性扫描 0→+5cm→-5cm */
    MODE_006_1     = 14,  /* 006变体: 小球固定+70mm(齿轮侧7cm) */
} SysMode;

/* ======================== AB任务状态 ======================== */
typedef enum {
    AB_IDLE  = 0,   /* 空闲, 等待S1启动 */
    AB_LEAVE,       /* 驶离起始黑线 */
    AB_RUN,         /* 陀螺仪角度P环直行, 监测终点黑线 */
    AB_DONE,        /* 到达B点, 已停车 */
} AB_State;

/* ======================== AB模式参数 ======================== */
#define AB_BASE_SPEED         (29)     /* AB段低速循迹基础速度 (%)   */
#define AB_LINE_KP            (0.35f)  /* 灰度循迹纠偏比例增益        */
#define AB_TARGET_DISTANCE_MM (1500.0f)/* 编码器停车距离：1.5m        */
#define AB_KP                 (1.5f)   /* 角度环P增益               */
#define AB_KI                 (0.0f)   /* 角度环I增益               */
#define AB_KD                 (0.4f)   /* 角度环D增益               */
#define AB_CORR_LIMIT         (15.0f)  /* 低速循迹校正量限幅 (%)       */
#define AB_MAX_SPEED          (46)     /* AB模式速度上限 (%)           */
#define AB_BALL_SETTLE_MS     (400U)   /* 钢球闭环先接管，小车保持静止 */
#define AB_SOFT_START_MS      (800U)  /* 小车缓慢升至正常AB速度 */
#define AB_START_SPEED        (16)     /* 底盘软启动初始速度 (%) */
#define AB_SOFT_STOP_DISTANCE_MM (390.0f) /* 终点前提前减速距离 */
#define AB_STOP_SPEED         (13)     /* 到终点前的最低行驶速度 (%) */
#define AB_ACCEL_BALL_FF_DEG  (1.0f)   /* 启动时钢球预补偿角 */
#define AB_BRAKE_BALL_FF_DEG  (-1.0f)  /* 停车前反向预补偿角 */

/* LEAVE已确认200ms连续全白(车已离开A黑线), RUN宽限期只需很短
 * 100ms足够跨越A黑线边缘的抖动, 不会在到达B之前过期 */
#define AB_LEAVE_CONFIRM_MS   (200U)   /* LEAVE->RUN: 连续全白确认时间(ms) */
#define AB_RUN_GRACE_MS       (100U)   /* RUN宽限期: 驶离A后忽略黑线(ms)   */

/* ======================== API函数 ======================== */

/** @brief 初始化 (默认循迹模式) */
void AB_Init(void);

/** @brief 直接设置模式 (供串口命令调用) */
void AB_SetMode(SysMode m);

/** @brief 切换模式 (S2按键调用), 切换前自动停车 */
void AB_SwitchMode(void);

/** @brief 获取当前系统模式 */
SysMode AB_GetMode(void);

/** @brief 获取AB任务状态 */
AB_State AB_GetState(void);

/** @brief 获取目标航向角 (度) */
float AB_GetTargetYaw(void);

/** @brief S1按键: AB模式下启动/停止 */
void AB_OnStartStop(void);

/** @brief 主循环更新, 每1ms调用, 返回左右轮速度 */
void AB_Update(const LineTracker_State *line, int8_t *left, int8_t *right);

#endif /* __AB_H */
