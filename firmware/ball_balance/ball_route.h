#ifndef BALL_ROUTE_H
#define BALL_ROUTE_H

#include <stdbool.h>

typedef enum {
    BALL_ROUTE_IDLE = 0,
    BALL_ROUTE_WAIT_ZERO,
    BALL_ROUTE_TO_POSITIVE,
    BALL_ROUTE_TO_NEGATIVE,
    BALL_ROUTE_DONE
} BallRoute_State;

void BallRoute_Init(void);
void BallRoute_Start(void);
void BallRoute_Stop(void);
void BallRoute_Toggle(void);
void BallRoute_Update(void);

bool BallRoute_IsRunning(void);
BallRoute_State BallRoute_GetState(void);

#endif
