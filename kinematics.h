#pragma once

// ── Kinematics ────────────────────────────────────────────────
// Converts parsed degree values into microsteps and manages
// the speed/delay relationship for the TMC2209 drivers.

// Motor & pulley config
extern const int  STEPS_PER_REV;
extern const int  MICROSTEPS;
extern const long STEPS_FULL_REV;

// Convert a rotation in degrees to a signed microstep count
long degreesToSteps(float deg);

// Calculate per-motor pulse delays so all motors finish in
// exactly duration_ms milliseconds regardless of step count.
// steps[4]      — signed step count per motor
// duration_ms   — target move duration in milliseconds
// outDelayUs[4] — output: microsecond delay per pulse per motor
void calcDelaysFromDuration(long steps[4], int duration_ms, int outDelayUs[4]);
