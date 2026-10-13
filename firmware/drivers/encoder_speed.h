#ifndef __ENCODER_SPEED_H
#define __ENCODER_SPEED_H

#include <stdint.h>

/*
 * Encoder speed compatibility API.
 *
 * The GPIO interrupt, pulse counting and 10 ms speed window are implemented
 * once in encoder_distance.c. Keeping a single interrupt owner avoids a
 * conflict on the MSPM0 shared GPIOA/GPIOB GROUP1 interrupt.
 */
void Encode_Init(void);
void Encode_Update(void);
void Encode_Reset(void);

uint32_t Encode_GetLeftCount(void);
uint32_t Encode_GetRightCount(void);

float Encode_GetLeftSpeedMmPs(void);
float Encode_GetRightSpeedMmPs(void);
float Encode_GetAverageSpeedMmPs(void);

float Encode_GetLeftDistanceMm(void);
float Encode_GetRightDistanceMm(void);
float Encode_GetAverageDistanceMm(void);

#endif /* ENCODE_H */
