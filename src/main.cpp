#include <Arduino.h>
#include <cmath>
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include "EthernetManager.h"

// ---- CONFIG for frequency ----
constexpr uint32_t CONTROL_PERIOD_US = 1000; // 1 kHz; 0 = unrestricted benchmark
constexpr uint32_t STATUS_PERIOD_MS  = 1000;    // 1hz
constexpr uint32_t PLOT_PERIOD_MS = 20;        // 50 Hz, independent of control rate
constexpr bool PLOT_CABLE_LENGTHS = true;      // false = benchmark status text
// NativeEthernet::begin() waits indefinitely for a physical Ethernet link.
// Leave disabled for standalone kinematics benchmarking over USB serial.
constexpr bool ENABLE_ETHERNET = false;

uint32_t nextControlUpdate;
uint32_t nextStatusUpdate;
uint32_t statusWindowStartUs;
uint32_t nextPlotUpdate;

uint32_t controlCycleCount = 0;
uint32_t missedControlCycles = 0;
uint64_t kinematicsTotalCycles = 0;
uint32_t kinematicsWorstCycles = 0;

struct Vec2 {
    float x, y;
};

struct Pose {
    float x, y, phi; // metres, metres, radians (positive = counterclockwise)
};

constexpr size_t CABLE_COUNT = 4;
Pose commandedPose = {0.0f, 0.0f, 0.0f};

// Placeholder planar XY + phi geometry. Origin is the frame centre.
// Frame anchors are world-frame; attachments are platform-frame. Units: metres.
constexpr Vec2 frameAnchors[CABLE_COUNT] = {
    {-1.0f, -1.0f}, {1.0f, -1.0f},
    {1.0f, 1.0f}, {-1.0f, 1.0f}
};
constexpr Vec2 platformAttachments[CABLE_COUNT] = {
    {-0.15f, -0.15f}, {0.15f, -0.15f},
    {0.15f, 0.15f}, {-0.15f, 0.15f}
};

// Volatile outputs ensure the compiler retains all benchmark calculations.
volatile float cableLengths[CABLE_COUNT];
volatile float cableSpeeds[CABLE_COUNT];
volatile float inverseJacobian[CABLE_COUNT][3];

void calculateKinematics(const Pose &pose, const Pose &velocity) {
    const float c = cosf(pose.phi), s = sinf(pose.phi);
    for (size_t i = 0; i < CABLE_COUNT; ++i) {
        const Vec2 &b = platformAttachments[i];
        const Vec2 r = {c * b.x - s * b.y, s * b.x + c * b.y};
        const Vec2 cable = {
            pose.x + r.x - frameAnchors[i].x,
            pose.y + r.y - frameAnchors[i].y
        };
        const float length = sqrtf(cable.x * cable.x + cable.y * cable.y);
        const float inverseLength = length > 0.0f ? 1.0f / length : 0.0f;
        const Vec2 u = {cable.x * inverseLength, cable.y * inverseLength};
        const float angular = r.x * u.y - r.y * u.x;
        cableLengths[i] = length;
        // length_dot = J * [vx, vy, omega], in m/s, m/s, rad/s. Positive = payout.
        inverseJacobian[i][0] = u.x;
        inverseJacobian[i][1] = u.y;
        inverseJacobian[i][2] = angular;
        cableSpeeds[i] = u.x * velocity.x + u.y * velocity.y + angular * velocity.phi;
    }
}

void skipWhitespace(const char *&cursor) {
    while (std::isspace(static_cast<unsigned char>(*cursor))) {
        ++cursor;
    }
}

bool parseGoto(const char *command, Pose &pose) {
    const char *cursor = command;
    skipWhitespace(cursor);
    if (strncmp(cursor, "goto", 4) != 0) {
        return false;
    }
    cursor += 4;
    skipWhitespace(cursor);
    if (*cursor++ != '(') {
        return false;
    }

    float values[3];
    for (size_t i = 0; i < 3; ++i) {
        skipWhitespace(cursor);
        char *end;
        errno = 0;
        values[i] = strtof(cursor, &end);
        if (end == cursor || errno == ERANGE || !std::isfinite(values[i])) {
            return false;
        }
        cursor = end;
        skipWhitespace(cursor);
        if (*cursor++ != (i < 2 ? ',' : ')')) {
            return false;
        }
    }
    skipWhitespace(cursor);
    if (*cursor != '\0') {
        return false;
    }

    const Pose candidate = {values[0] * 0.001f, values[1] * 0.001f,
                           remainderf(values[2], 360.0f) * (static_cast<float>(PI) / 180.0f)};
    // Reject numeric magnitudes that would overflow the float length calculation.
    if (!std::isfinite(candidate.x * candidate.x + candidate.y * candidate.y)) {
        return false;
    }
    pose = candidate;
    return true;
}

void pollSerialCommands() {
    static char buffer[96];
    static size_t used = 0;
    static bool overflow = false;
    // Bound work per loop; a partial command never blocks the control task.
    for (size_t budget = 0; budget < 32 && Serial.available() > 0; ++budget) {
        const char ch = static_cast<char>(Serial.read());
        if (used == 0 && !overflow && std::isspace(static_cast<unsigned char>(ch))) {
            continue;
        }
        const bool complete = ch == ')' || ch == '\n' || ch == '\r';
        if (!overflow && ch != '\n' && ch != '\r') {
            if (used < sizeof(buffer) - 1) {
                buffer[used++] = ch;
            } else {
                overflow = true;
            }
        }
        if (!complete) {
            continue;
        }
        buffer[used] = '\0';
        if (overflow || (used > 0 && !parseGoto(buffer, commandedPose))) {
            if constexpr (!PLOT_CABLE_LENGTHS) {
                Serial.println("Invalid command; use goto(x_mm,y_mm,phi_deg)");
            }
        }
        used = 0;
        overflow = false;
    }
}

void printCableLengths() {
    // Arduino Serial Plotter: one labelled sample per line, lengths in mm.
    if (!Serial || Serial.availableForWrite() < 128) {
        return;
    }
    for (size_t i = 0; i < CABLE_COUNT; ++i) {
        if (i > 0) {
            Serial.print('\t');
        }
        Serial.print("Cable");
        Serial.print(i + 1);
        Serial.print("_mm:");
        Serial.print(cableLengths[i] * 1000.0f, 3);
    }
    Serial.println();
}

void controlUpdate() {
    // Test only: goto changes the simulated pose immediately. No motor commands
    // or trajectory interpolation. The platform is stationary between commands.
    const Pose velocity = {0.0f, 0.0f, 0.0f};

    // Cortex-M7 cycle timing has sub-microsecond resolution. Keep interrupts enabled
    // so interrupt time during the calculation is included in the observed cost.
    const uint32_t startCycles = ARM_DWT_CYCCNT;
    calculateKinematics(commandedPose, velocity);
    const uint32_t elapsedCycles = ARM_DWT_CYCCNT - startCycles;
    kinematicsTotalCycles += elapsedCycles;
    if (elapsedCycles > kinematicsWorstCycles) {
        kinematicsWorstCycles = elapsedCycles;
    }
    controlCycleCount++;

    // TODO:
    // 1. Read latest motor feedback
    // 2. Update robot state
    // 3. Validate setpoints and safety limits
    // 4. Queue motor commands
}


EthernetManager ethernet;


void setup() {
    Serial.begin(115200);
    const uint32_t serialWaitStartMs = millis();
    while (!Serial && millis() - serialWaitStartMs < 3000u) {
        delay(1);
    }

    if constexpr (!PLOT_CABLE_LENGTHS) {
        Serial.println("Starting CDPR planar benchmark; goto(x_mm,y_mm,phi_deg)");
    }
    if constexpr (ENABLE_ETHERNET) {
        ethernet.begin();
    }

    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

    calculateKinematics(commandedPose, {0.0f, 0.0f, 0.0f});
    statusWindowStartUs = micros();
    nextControlUpdate = statusWindowStartUs + CONTROL_PERIOD_US;
    nextStatusUpdate = millis() + STATUS_PERIOD_MS;
    nextPlotUpdate = millis() + PLOT_PERIOD_MS;
}

void loop() {
    pollSerialCommands();
    const uint32_t nowUs = micros();
    
    // ---- Scheduled control task, or unrestricted throughput benchmark ----
    if constexpr (CONTROL_PERIOD_US == 0) {
        controlUpdate();
    } else if (static_cast<int32_t>(nowUs - nextControlUpdate) >= 0) {
        const uint32_t lateness = nowUs - nextControlUpdate;

        // Count expired deadlines; never execute a backlog.
        const uint32_t skipped = lateness / CONTROL_PERIOD_US;
        missedControlCycles += skipped;

        // Advance to the next scheduled deadline.
        nextControlUpdate +=
            (skipped + 1) * CONTROL_PERIOD_US;

        controlUpdate();
    }

    // lower-hz ethernet loop

    const uint32_t nowMs = millis();

    if (PLOT_CABLE_LENGTHS && static_cast<int32_t>(nowMs - nextPlotUpdate) >= 0) {
        nextPlotUpdate = nowMs + PLOT_PERIOD_MS;
        printCableLengths();
    }

    if (!PLOT_CABLE_LENGTHS && static_cast<int32_t>(nowMs - nextStatusUpdate) >= 0) {
        nextStatusUpdate = nowMs + STATUS_PERIOD_MS;

        const uint32_t reportUs = micros();
        const uint32_t windowUs = reportUs - statusWindowStartUs;
        statusWindowStartUs = reportUs;
        const float cpuHz = static_cast<float>(F_CPU_ACTUAL);
        const float averageCycles = controlCycleCount > 0
            ? static_cast<float>(kinematicsTotalCycles) / controlCycleCount : 0.0f;

        Serial.print("Control Hz: ");
        Serial.print(windowUs > 0 ? controlCycleCount * 1000000.0f / windowUs : 0.0f, 1);

        Serial.print(" | Missed cycles: ");
        Serial.print(missedControlCycles);
        Serial.print(" | Kinematics us avg/worst: ");
        Serial.print(averageCycles * 1000000.0f / cpuHz, 3);
        Serial.print(" / ");
        Serial.print(kinematicsWorstCycles * 1000000.0f / cpuHz, 3);
        Serial.print(" | Math-only Hz estimate (avg): ");
        Serial.println(averageCycles > 0.0f ? cpuHz / averageCycles : 0.0f, 1);

        controlCycleCount = 0;
        missedControlCycles = 0;
        kinematicsTotalCycles = 0;
        kinematicsWorstCycles = 0;

        if constexpr (ENABLE_ETHERNET) {
            ethernet.printLinkStatus();
        }
    }

}
