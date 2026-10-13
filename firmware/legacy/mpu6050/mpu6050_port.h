#ifndef MPU6050_PORT_H
#define MPU6050_PORT_H

#include <stdint.h>

/* ======================== 兼容旧接口声明 ======================== */
/* 旧代码中 bsp_mpu6050.h 暴露的函数, 此处保留声明以兼容 */
char  MPU6050_Init(void);
void  MPU6050ReadGyro(short *gyroData);
void  MPU6050ReadAcc(short *accData);

/* ======================== 新移植接口 ======================== */
uint8_t MPU6050_RawInit(void);
void MPU6050_ReadGyro(int16_t gyro[3]);
void MPU6050_ReadAccel(int16_t accel[3]);

#endif
