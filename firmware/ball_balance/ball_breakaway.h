#ifndef BALL_BREAKAWAY_H
#define BALL_BREAKAWAY_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BALL_BREAKAWAY_IDLE = 0,
    BALL_BREAKAWAY_TILT,
    BALL_BREAKAWAY_RETURN
} BallBreakawayState;

void BallBreakaway_Init(void);
void BallBreakaway_Trigger(void);
void BallBreakaway_Update(void);
void BallBreakaway_Stop(void);
void BallBreakaway_AdjustAcceleration(int16_t delta);
uint8_t BallBreakaway_GetAcceleration(void);
float BallBreakaway_GetPulseAngleDeg(void);
BallBreakawayState BallBreakaway_GetState(void);

#endif
