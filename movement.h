#pragma once
#include <Arduino.h>

// ── Pin assignments ───────────────────────────────────────────
extern const int EN_PIN;
extern const int STEP[4];
extern const int DIR[4];

// ── Init ──────────────────────────────────────────────────────
void movement_init();
void motors_enable();
void motors_disable();
bool motors_moving();

// ── Blocking move with trapezoidal ramp ───────────────────────
// steps[4]   — signed microstep count per motor
// delayUs[4] — per-motor cruise pulse delay (from calcDelaysFromDuration)
void stepMotorsTimed(long steps[4], int delayUs[4]);

// Call every loop() — ticks the state machine one step.
// Returns true when move is complete.
bool movement_update();
