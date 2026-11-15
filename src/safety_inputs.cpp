#include "safety_inputs.h"

// Пауза после включения входов: внешние оптроны и RC-фильтры должны выйти
// на устойчивый уровень до самотеста.
static const uint32_t SAFETY_SELF_TEST_SETTLE_MS = 50;

void initSafetyInputs() {
    // GPIO 34/35/39 — input-only, внутренняя подтяжка недоступна.
    // Уровень задают только внешние резисторы/оптроны.
    pinMode(ESTOP_PIN, INPUT);
    pinMode(MOTORS_ALARM, INPUT);
    pinMode(ENDSTOPS_PIN, INPUT);
}

bool runSafetySelfTest(String* reason) {
#if SAFETY_INPUTS_ENABLED
    delay(SAFETY_SELF_TEST_SETTLE_MS);

    if (safetyEstopActive()) {
        if (reason) *reason = "ERROR: Safety self-test failed: ESTOP input is active (button pressed or NC loop broken).";
        return false;
    }
    if (safetyMotorAlarmActive()) {
        if (reason) *reason = "ERROR: Safety self-test failed: MOTORS_ALARM is active (driver fault or broken loop).";
        return false;
    }
    if (safetyEndstopsActive()) {
        if (reason) *reason = "ERROR: Safety self-test failed: ENDSTOPS NC loop is open (switch triggered or wire broken).";
        return false;
    }

    if (reason) *reason = "STATUS: Safety self-test passed (ESTOP/MOTORS_ALARM/ENDSTOPS are LOW).";
    return true;
#else
    if (reason) {
        *reason = "WARNING: Safety inputs DISABLED (SAFETY_INPUTS_ENABLED=0). "
                  "ESTOP/MOTORS_ALARM/ENDSTOPS are not monitored. Bench/debug use only!";
    }
    return true;
#endif
}
