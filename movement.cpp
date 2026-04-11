#include "movement.h"
#include <climits>
#include <math.h>

// ── Pin definitions ───────────────────────────────────────────
const int EN_PIN  = 27;
const int STEP[4] = {25, 14, 32, 18};
const int DIR[4]  = {26,  4, 33, 19};

// ── Trapezoidal ramp config ───────────────────────────────────
// The ramp occupies this fraction of the total move at each end.
// e.g. 0.25 = first 25% ramp up, last 25% ramp down, middle 50% cruise.
static const float RAMP_FRACTION = 0.15f;

// Ratio of start/end speed to cruise speed (0.0–1.0).
// 0.1 = start at 10% of cruise speed, ramp up to 100%.
static const float MIN_SPEED_RATIO = 0.30f;

// ── Step state machine ────────────────────────────────────────
static long  s_steps[4]       = {0, 0, 0, 0};
static long  s_accumulator[4] = {0, 0, 0, 0};
static long  s_timeAccum[4]   = {0, 0, 0, 0};
static long  s_maxSteps        = 0;
static long  s_tick             = 0;
static int   s_cruiseDelayUs[4]= {0, 0, 0, 0};  // delay at full cruise speed
static int   s_masterCruise    = 0;              // fastest motor cruise delay
static bool  s_moving           = false;

// ── Init ──────────────────────────────────────────────────────
void movement_init() {
    pinMode(EN_PIN, OUTPUT);
    motors_enable();
    for (int i = 0; i < 4; i++) {
        pinMode(STEP[i], OUTPUT);
        pinMode(DIR[i],  OUTPUT);
        digitalWrite(STEP[i], LOW);
        digitalWrite(DIR[i],  LOW);
    }
}

void motors_enable()  { digitalWrite(EN_PIN, LOW);  }
void motors_disable() { digitalWrite(EN_PIN, HIGH); }
bool motors_moving()  { return s_moving; }

// ── Ramp delay calculation ────────────────────────────────────
// Given current tick and total steps, returns the delay for the
// master motor at this point in the trapezoidal profile.
// Slower motors scale proportionally from their cruise delay.
static int rampedMasterDelay(long tick, long maxSteps, int cruiseDelay) {
    if (maxSteps == 0) return cruiseDelay;

    float progress = (float)tick / (float)maxSteps;  // 0.0 → 1.0
    float ratio;

    if (progress < RAMP_FRACTION) {
        // Ramp up — lerp from MIN_SPEED_RATIO to 1.0
        ratio = MIN_SPEED_RATIO + (1.0f - MIN_SPEED_RATIO) * (progress / RAMP_FRACTION);
    } else if (progress > (1.0f - RAMP_FRACTION)) {
        // Ramp down — lerp from 1.0 to MIN_SPEED_RATIO
        float t = (progress - (1.0f - RAMP_FRACTION)) / RAMP_FRACTION;
        ratio = 1.0f - (1.0f - MIN_SPEED_RATIO) * t;
    } else {
        // Cruise
        ratio = 1.0f;
    }

    // Higher ratio = faster speed = lower delay
    // delay = cruiseDelay / ratio  (speed is inversely proportional to delay)
    int d = (int)((float)cruiseDelay / ratio);
    return max(d, 39);  // never below MIN_DELAY_US
}

// ── stepMotorsTimed ───────────────────────────────────────────
void stepMotorsTimed(long steps[4], int delayUs[4]) {
    s_maxSteps    = 0;
    s_masterCruise = INT_MAX;

    for (int i = 0; i < 4; i++) {
        s_steps[i]        = steps[i];
        s_cruiseDelayUs[i]= delayUs[i];
        s_accumulator[i]  = 0;
        s_timeAccum[i]    = 0;

        if (steps[i] != 0) {
            bool dir = steps[i] > 0 ? LOW : HIGH;
            if (i == 1 || i == 3) dir = !dir;
            digitalWrite(DIR[i], dir);

            s_maxSteps     = max(s_maxSteps, abs(steps[i]));
            if (delayUs[i] > 0)
                s_masterCruise = min(s_masterCruise, delayUs[i]);
        }
    }

    if (s_maxSteps == 0) return;
    if (s_masterCruise == INT_MAX) s_masterCruise = 156;

    s_tick   = 0;
    s_moving = true;
    delayMicroseconds(100);
}

// ── movement_update ───────────────────────────────────────────
bool movement_update() {
    if (!s_moving) return true;

    if (s_tick >= s_maxSteps) {
        for (int i = 0; i < 4; i++) digitalWrite(STEP[i], LOW);
        s_moving = false;
        return true;
    }

    // Get ramped master tick for this position in the move
    int masterTick = rampedMasterDelay(s_tick, s_maxSteps, s_masterCruise);

    for (int i = 0; i < 4; i++) {
        if (s_steps[i] == 0 || s_cruiseDelayUs[i] == 0) continue;

        s_accumulator[i] += abs(s_steps[i]);
        if (s_accumulator[i] >= s_maxSteps) {
            s_accumulator[i] -= s_maxSteps;

            // Scale this motor's delay by the same ramp ratio as master
            int motorDelay = rampedMasterDelay(s_tick, s_maxSteps, s_cruiseDelayUs[i]);

            s_timeAccum[i] += masterTick;
            if (s_timeAccum[i] >= motorDelay) {
                s_timeAccum[i] -= motorDelay;
                digitalWrite(STEP[i], HIGH);
            }
        }
    }

    delayMicroseconds(masterTick);

    for (int i = 0; i < 4; i++) digitalWrite(STEP[i], LOW);

    delayMicroseconds(masterTick);

    s_tick++;
    return false;
}
