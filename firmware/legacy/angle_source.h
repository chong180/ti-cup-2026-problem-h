/**
 * ============================================================================
 * @file    angle_source.h
 * @brief   角度来源占位 — MPU6050和Gyro已移除, 仅保留空接口
 * ============================================================================
 */

#ifndef __ANGLE_SOURCE_H
#define __ANGLE_SOURCE_H

#include <stdint.h>

/* 所有角度相关功能已移除, 保留空定义避免编译错误 */
#define Angle_Init()        (void)0
#define Angle_GetYaw()      (0.0f)
#define Angle_ResetYaw()    (void)0

#endif /* __ANGLE_SOURCE_H */
