#include "ball_balance/ball_breakaway.h"

#include "drivers/stepper_emm_v5.h"
#include "drivers/timebase.h"

/*
 * Static-friction breakaway calibration.
 *
 * One S1 press sends a small absolute-angle pulse and then returns the
 * platform to level.  Consecutive pulses alternate direction so the ball
 * does not keep walking toward one edge while acceleration is tuned.
 */
#define BREAKAWAY_MOTOR_ADDR             (2U)
#define BREAKAWAY_POSITIVE_DIRECTION     (0U)
#define BREAKAWAY_NEGATIVE_DIRECTION     (1U)
#define BREAKAWAY_POSITION_RPM           (300U)
#define BREAKAWAY_PULSES_PER_REV         (12800.0f)
#define BREAKAWAY_PULSE_ANGLE_DEG        (1.50f)
#define BREAKAWAY_TILT_HOLD_MS           (300U)
#define BREAKAWAY_RETURN_SETTLE_MS       (350U)
#define BREAKAWAY_ACCEL_DEFAULT          (50U)
#define BREAKAWAY_ACCEL_MIN              (20U)
#define BREAKAWAY_ACCEL_MAX              (255U)

static uint8_t s_acceleration;
static int8_t s_nextSign;
static BallBreakawayState s_state;
static uint32_t s_stateStartMs;

static void commandAbsoluteAngle(float angleDeg)
{
    uint8_t direction;
    uint32_t pulses;
    float magnitude = angleDeg;

    if (magnitude < 0.0f) {
        magnitude = -magnitude;
        direction = BREAKAWAY_NEGATIVE_DIRECTION;
    } else {
        direction = BREAKAWAY_POSITIVE_DIRECTION;
    }

    pulses = (uint32_t) (
        magnitude * BREAKAWAY_PULSES_PER_REV / 360.0f + 0.5f);
    Emm_V5_Pos_Control(
        BREAKAWAY_MOTOR_ADDR,
        direction,
        BREAKAWAY_POSITION_RPM,
        s_acceleration,
        pulses,
        1U,
        false);
}

void BallBreakaway_Init(void)
{
    s_acceleration = BREAKAWAY_ACCEL_DEFAULT;
    s_nextSign = 1;
    s_state = BALL_BREAKAWAY_IDLE;
    s_stateStartMs = 0U;
}

void BallBreakaway_Trigger(void)
{
    float pulseAngle;

    if (s_state != BALL_BREAKAWAY_IDLE) {
        return;
    }

    Emm_V5_En_Control(BREAKAWAY_MOTOR_ADDR, true, false);
    pulseAngle = (s_nextSign > 0) ?
        BREAKAWAY_PULSE_ANGLE_DEG : -BREAKAWAY_PULSE_ANGLE_DEG;
    s_nextSign = (int8_t) -s_nextSign;
    commandAbsoluteAngle(pulseAngle);
    s_state = BALL_BREAKAWAY_TILT;
    s_stateStartMs = Timebase_GetMillis();
}

void BallBreakaway_Update(void)
{
    uint32_t now = Timebase_GetMillis();

    if (s_state == BALL_BREAKAWAY_TILT) {
        if ((uint32_t) (now - s_stateStartMs) >=
            BREAKAWAY_TILT_HOLD_MS) {
            commandAbsoluteAngle(0.0f);
            s_state = BALL_BREAKAWAY_RETURN;
            s_stateStartMs = now;
        }
    } else if (s_state == BALL_BREAKAWAY_RETURN) {
        if ((uint32_t) (now - s_stateStartMs) >=
            BREAKAWAY_RETURN_SETTLE_MS) {
            s_state = BALL_BREAKAWAY_IDLE;
        }
    }
}

void BallBreakaway_Stop(void)
{
    commandAbsoluteAngle(0.0f);
    s_state = BALL_BREAKAWAY_IDLE;
    s_stateStartMs = Timebase_GetMillis();
}

void BallBreakaway_AdjustAcceleration(int16_t delta)
{
    int16_t value;

    if (s_state != BALL_BREAKAWAY_IDLE) {
        return;
    }

    value = (int16_t) s_acceleration + delta;
    if (value < (int16_t) BREAKAWAY_ACCEL_MIN) {
        value = (int16_t) BREAKAWAY_ACCEL_MIN;
    } else if (value > (int16_t) BREAKAWAY_ACCEL_MAX) {
        value = (int16_t) BREAKAWAY_ACCEL_MAX;
    }
    s_acceleration = (uint8_t) value;
}

uint8_t BallBreakaway_GetAcceleration(void)
{
    return s_acceleration;
}

float BallBreakaway_GetPulseAngleDeg(void)
{
    return BREAKAWAY_PULSE_ANGLE_DEG;
}

BallBreakawayState BallBreakaway_GetState(void)
{
    return s_state;
}
