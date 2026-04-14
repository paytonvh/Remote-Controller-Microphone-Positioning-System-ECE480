#include "kinematics.h"
#include <Arduino.h>

// Motor config
const int  STEPS_PER_REV  = 200;    // 1.8deg NEMA17
const int  MICROSTEPS     = 8;     // TMC2209 default microstep
const long STEPS_FULL_REV = STEPS_PER_REV * MICROSTEPS; // 1600

// Minimum safe pulse delay
const int MIN_DELAY_US = 10;

// Converts a signed rotation in degrees to a signed microstep
// count. Negative input = reverse direction.
long degreesToSteps(float deg) {
    return (long)((deg / 360.0f) * (float)STEPS_FULL_REV);
}

// Given a fixed move duration and each motor's step count,
// calculates the exact pulse delay each motor needs so they
// all finish at the same wall-clock time.

// duration_us = duration_ms * 1000
// delayUs[i] = duration_us / (absSteps[i] * 2)
// (* 2 because one step = HIGH pulse + LOW pulse)
void calcDelaysFromDuration(long steps[4], int duration_ms, int outDelayUs[4]) {
    long duration_us = (long)duration_ms * 1000L;

    for (int i = 0; i < 4; i++) {
        long absSteps = abs(steps[i]);
        if (absSteps == 0 || absSteps < 3) {
            steps[i] = 0;
            outDelayUs[i] = 0;
            continue;
        }
        int delay = (int)(duration_us / (absSteps * 2));

        outDelayUs[i] = max(delay, MIN_DELAY_US);
    }
}
