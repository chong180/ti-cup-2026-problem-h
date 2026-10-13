/**
 * ============================================================================
 * @file    bsp_mpu6050.h
 * @brief   兼容层 — 包含新移植的 mpu6050_port.h
 *
 * 旧代码中 #include "bsp_mpu6050.h" 会自动包含新驱动
 * ============================================================================
 */

#ifndef __BSP_MPU6050_H
#define __BSP_MPU6050_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* 兼容旧宏定义 (部分代码可能引用) */
#define MPU6050_ADDR            0x68
#define MPU_GYRO_LSB            16.4f

/* 包含新移植的底层驱动 */
#include "mpu6050_port.h"

#endif /* __BSP_MPU6050_H */
