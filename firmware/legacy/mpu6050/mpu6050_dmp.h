#ifndef MPU6050_DMP_H
#define MPU6050_DMP_H

#include <stdbool.h>
#include <stdint.h>

/* ======================== 新移植API ======================== */
bool  MPU6050_DMP_Init(void);
void  MPU6050_DMP_Process(void);
bool  MPU6050_DMP_HasValidData(void);
float MPU6050_DMP_GetRoll(void);
float MPU6050_DMP_GetPitch(void);
float MPU6050_DMP_GetYaw(void);
float MPU6050_DMP_GetGyroZ(void);
void  MPU6050_DMP_ZeroYaw(void);

/* ======================== 兼容旧接口 ======================== */
/* 旧dmp.h中声明的函数, 此处映射到新API */
uint8_t mpu_dmp_init(void);
uint8_t mpu_dmp_get_data(float *pitch, float *roll, float *yaw);
float   MPU6050_GetYaw(void);
void    MPU6050_ResetYaw(void);

#endif
