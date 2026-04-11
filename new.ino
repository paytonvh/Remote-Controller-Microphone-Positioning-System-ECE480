#include <Arduino.h>
#include "movement.h"
#include "kinematics.h"

#define BAUD 115200

String inputBuffer = "";
bool   needsDone   = false;

void handleCommand(String cmd) {
    cmd.trim();

    if (cmd == "STOP") {
        motors_disable();
        Serial.println("DONE");
        return;
    }

    if (cmd == "ENABLE") {
        motors_enable();
        Serial.println("DONE");
        return;
    }

    // ── Move packet ───────────────────────────────────────────
    // Format: "M1:deg,M2:deg,M3:deg,M4:deg,DUR:ms,BLOCK"
    long steps[4]    = {0, 0, 0, 0};
    int  duration_ms = 500;
    bool blocking    = false;
    int  start       = 0;

    while (start < (int)cmd.length()) {
        int    commaIdx = cmd.indexOf(',', start);
        String token    = (commaIdx == -1)
                            ? cmd.substring(start)
                            : cmd.substring(start, commaIdx);
        token.trim();

        if (token.startsWith("M")) {
            int colonIdx = token.indexOf(':');
            if (colonIdx != -1) {
                int   motorNum = token.substring(1, colonIdx).toInt();
                float degrees  = token.substring(colonIdx + 1).toFloat();
                if (motorNum >= 1 && motorNum <= 4)
                    steps[motorNum - 1] = degreesToSteps(degrees);
            }
        }
        if (token.startsWith("DUR:"))
            duration_ms = token.substring(4).toInt();
        if (token == "BLOCK")
            blocking = true;
        if (commaIdx == -1) break;
        start = commaIdx + 1;
    }

    int motorDelays[4];
    calcDelaysFromDuration(steps, duration_ms, motorDelays);
    stepMotorsTimed(steps, motorDelays);
    needsDone = blocking;
}

void setup() {
    Serial.begin(BAUD);
    movement_init();
    Serial.println("ESP32 ready.");
}

void loop() {
    movement_update();

    if (needsDone && !motors_moving()) {
        Serial.println("DONE");
        needsDone = false;
    }

    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            if (inputBuffer.length() > 0) {
                handleCommand(inputBuffer);
                inputBuffer = "";
            }
        } else {
            inputBuffer += c;
        }
    }
}
