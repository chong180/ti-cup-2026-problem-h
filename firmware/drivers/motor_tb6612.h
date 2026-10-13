#ifndef MOTOR_TB6612_H
#define MOTOR_TB6612_H

#include <stdint.h>

/*
 * TB6612 motor driver.
 *
 * The public speed unit is percent. Positive values mean that the car moves
 * forward. Values are limited to +/- 100 (full PWM range). Each caller is
 * responsible for applying its own mode-specific speed cap.
 */
#define TB6612_MAX_SPEED_PERCENT (100)

void TB6612_Init(void);
void TB6612_SetSpeed(int8_t leftPercent, int8_t rightPercent);
void TB6612_Stop(void);

#endif
