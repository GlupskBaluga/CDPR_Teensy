#include <Arduino.h>
#include "EthernetManager.h"


// ---- CONFIG for frequency ----
constexpr uint32_t COPNTROL_PERIOD_US  = 1000;  // 1khz
constexpr uint32_t STATUS_PERIOD_MS  = 1000;    // 1hz

uint32_t nextControlUpdate;
uint32_t nextStatusUpdate;

uint32_t controlCycleCount = 0;
uint32_t missedControlCycles = 0;

void controlUpdate(uint32_t nowUs) {
    controlCycleCount++;

    // TODO:
    // 1. Read latest motor feedback
    // 2. Update robot state
    // 3. Calculate kinematics
    // 4. Validate setpoints and safety limits
    // 5. Queue motor commands
}


EthernetManager ethernet;


void setup() {
    Serial.begin(115200);

    ethernet.begin();

    nextControlUpdate = micros() + COPNTROL_PERIOD_US;
    nextStatusUpdate = millis() + STATUS_PERIOD_MS;
}

void loop() {
    const uint32_t nowUs = micros();
    
    // ---- 1 kHz control task ----
    if (static_cast<int32_t>(nowUs - nextControlUpdate) >= 0) {
        const uint32_t lateness = nowUs - nextControlUpdate;

        // Count expired deadlines; never execute a backlog.
        const uint32_t skipped = lateness / COPNTROL_PERIOD_US;
        missedControlCycles += skipped;

        // Advance to the next scheduled deadline.
        nextControlUpdate +=
            (skipped + 1) * COPNTROL_PERIOD_US;

        controlUpdate(nowUs);
    }

    // lower-hz ethernet loop

    const uint32_t nowMs = millis();

    if (static_cast<int32_t>(nowMs - nextStatusUpdate) >= 0) {
        nextStatusUpdate = nowMs + STATUS_PERIOD_MS;

        Serial.print("Control cycles in last second: ");
        Serial.print(controlCycleCount);

        Serial.print(" | Missed cycles: ");
        Serial.print(missedControlCycles);

        controlCycleCount = 0;
        missedControlCycles = 0;

        ethernet.printLinkStatus();
    }

}