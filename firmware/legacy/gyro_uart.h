/**
 * @file    gyro.h
 * @brief   单轴Z轴陀螺仪驱动 — 0x5A 帧头 / 5字节帧协议
 *
 * 硬件连接:
 *   陀螺仪 TX  ->  MSPM0 PA22 (UART2 RX)
 *   陀螺仪 RX  ->  MSPM0 PA23 (UART2 TX)
 *   VCC / GND  ->  5V / GND
 *
 * 通信协议 (上报帧, 5字节):
 *   [0] 0x5A    帧头
 *   [1] TYPE    0xBB = Yaw 航向角
 *   [2] DATAL   数据低字节
 *   [3] DATAH   数据高字节
 *   [4] SUMCRC  和校验 = (0x5A + TYPE + DATAL + DATAH) & 0xFF
 *
 * 数据换算:
 *   raw  = (int16_t)((DATAH << 8) | DATAL)   // 小端, 低字节在前
 *   Yaw  = raw / 32768.0 * 180.0             // 度
 *
 * 命令帧 (下行, 3字节):
 *   [0] 0x5A    帧头
 *   [1] CMD     命令类型
 *   [2] SUMCRC  = (0x5A + CMD) & 0xFF
 *
 * 命令帧 (下行, 4字节, 带参数):
 *   [0] 0x5A    帧头
 *   [1] CMD     命令类型
 *   [2] PARAM   参数
 *   [3] SUMCRC  = (0x5A + CMD + PARAM) & 0xFF
 */
#ifndef __GYRO_H
#define __GYRO_H

#include "ti_msp_dl_config.h"

/* ======================== 协议常量 ======================== */
#define GYRO_FRAME_HEADER   0x5A    /* 帧头              */
#define GYRO_TYPE_YAW       0xBB    /* TYPE: Yaw 航向角   */
#define GYRO_FRAME_LEN      5       /* 上报帧长度 (字节)  */

/* 命令类型码 */
#define GYRO_CMD_UNLOCK     0x50    /* 解锁              */
#define GYRO_CMD_ZERO       0x51    /* Yaw 航向归零      */
#define GYRO_CMD_CALIB      0x52    /* 自动零偏校准       */
#define GYRO_CMD_SET_RATE   0x53    /* 修改输出速率       */

/* ======================== 全局变量 ======================== */

/* 实时 Yaw 航向角 (度), 在中断中更新 */
extern volatile float gyro_yaw;

/* 收到完整帧标志 */
extern volatile uint8_t  gyro_rx_done;

/* 累计有效帧数 (调试用) */
extern volatile uint32_t gyro_frame_count;

/* 累计校验失败次数 (调试用) */
extern volatile uint32_t gyro_crc_error;

/* ======================== API 函数 ======================== */

/**
 * @brief  初始化陀螺仪: 使能 UART RX 中断
 */
void Gyro_Init(void);

/**
 * @brief  获取 Yaw 航向角 (度)
 */
float Gyro_GetYaw(void);

/** @brief 兼容旧接口, 返回 Yaw */
float Gyro_GetAngleDeg(void);

/**
 * @brief  发送解锁指令
 */
void Gyro_Unlock(void);

/**
 * @brief  Yaw 航向一键归零 (Z轴角度清零)
 */
void Gyro_HeadingZero(void);

/**
 * @brief  自动零偏校准 (校准期间保持模块静止)
 */
void Gyro_AutoCalib(void);

/**
 * @brief  修改输出速率
 * @param  rateHz  输出频率 (Hz), 如 1/10/50/100
 */
void Gyro_SetRate(uint8_t rateHz);

#endif /* __GYRO_H */
