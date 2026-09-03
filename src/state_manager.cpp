#include "state_manager.h"
#include "pinout.h"
#include "motion.h"
#include "config.h"
#include "wifi_manager.h"
#include <Preferences.h>

extern Preferences preferences; 

volatile MachineState currentMachineState = STATE_INIT;
volatile bool isHomed = false;
static unsigned long lastReportTime = 0;

extern volatile uint16_t currentExecutingLineNum; // подтягиваем из gcode_program

String getStateName() {
    String lockStatus = isHomed ? "" : " (LOCKED)";
    switch (currentMachineState) {
        case STATE_INIT:         return "INIT" + lockStatus;
        case STATE_ALARM:        return "ALARM";
        case STATE_IDLE:         return "IDLE" + lockStatus;
        case STATE_JOGGING:      return "JOGGING";
        case STATE_CALIBRATION:  return "CALIBRATION";
        case STATE_MAP_TRANSFER: return "MAP_TRANSFER";
        case STATE_RUNNING:      return "RUNNING";
        case STATE_HOMING:       return "HOMING";
        case STATE_HOLD:         return "HOLD";
        default:                 return "UNKNOWN";
    }
}

void changeState(MachineState newState) {
    if (currentMachineState == STATE_ALARM && newState != STATE_IDLE) return;

    if (!isHomed) {
        if (newState == STATE_JOGGING && cfg.allowJogBeforeHoming) { } 
        else if (newState == STATE_RUNNING || newState == STATE_JOGGING || newState == STATE_CALIBRATION) {
            String err = "ERROR: Machine is LOCKED. Run HOMING first.";
            Serial.println(err); sendToWiFiClient(err);
            return;
        }
    }

    switch (newState) {
        case STATE_ALARM:
            isVectorMoving = false;
            digitalWrite(MOTORS_ENABLE, HIGH); 
            digitalWrite(RELE_SPINDLE, LOW); 
            digitalWrite(RELE_COOLANT, LOW);

            preferences.begin("cnc_cfg", false);
            preferences.putInt("lastLine", cfg.lastExecutedLine);
            preferences.end();
            break;
        case STATE_IDLE:
            digitalWrite(MOTORS_ENABLE, LOW); 
            break;
        default:
            break;
    }
    currentMachineState = newState;
}

void checkHardwareSecurity() {
    if (digitalRead(ESTOP_PIN) == HIGH || digitalRead(MOTORS_ALARM) == HIGH || digitalRead(ENDSTOPS_PIN) == HIGH) {
        if (currentMachineState != STATE_ALARM) changeState(STATE_ALARM);
    }
}

void reportStatusToPC() {
    if (millis() - lastReportTime > 1000) {
        lastReportTime = millis();
        float mmX = currentStepsX / cfg.stepsPerMmX; 
        float mmY = currentStepsY / cfg.stepsPerMmY;
        float mmZ = currentStepsZ / cfg.stepsPerMmZ;
        String statusStr = "<Status:" + getStateName() +
                   "|Time=" + String(millis()) +
                   "|Pos:X=" + String(mmX, 2) + 
                   ",Y=" + String(mmY, 2) + 
                   ",Z=" + String(mmZ, 2) + 
                   "|Hom:" + String(isHomed ? "1" : "0") + 
                   "|Line=" + String(currentExecutingLineNum) + ">";
        Serial.println(statusStr); sendToWiFiClient(statusStr);
    }
}
