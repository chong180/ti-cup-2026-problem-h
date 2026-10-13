#include "drivers/encoder_speed.h"

#include "drivers/encoder_odometry.h"

void Encode_Init(void)
{
    EncoderDistance_Init();
}

void Encode_Update(void)
{
    EncoderDistance_Update();
}

void Encode_Reset(void)
{
    EncoderDistance_Reset();
}

uint32_t Encode_GetLeftCount(void)
{
    return EncoderDistance_GetRightCount();
}

uint32_t Encode_GetRightCount(void)
{
    return EncoderDistance_GetLeftCount();
}

float Encode_GetLeftSpeedMmPs(void)
{
    return EncoderDistance_GetRightMmPs();
}

float Encode_GetRightSpeedMmPs(void)
{
    return EncoderDistance_GetLeftMmPs();
}

float Encode_GetAverageSpeedMmPs(void)
{
    return EncoderDistance_GetAverageMmPs();
}

float Encode_GetLeftDistanceMm(void)
{
    return EncoderDistance_GetRightMm();
}

float Encode_GetRightDistanceMm(void)
{
    return EncoderDistance_GetLeftMm();
}

float Encode_GetAverageDistanceMm(void)
{
    return EncoderDistance_GetAverageMm();
}
