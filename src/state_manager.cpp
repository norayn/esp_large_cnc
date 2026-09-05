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

static String lastSentStatusStr = ""; // Хранилище для сравнения изменений


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
        case STATE_GCODE_UPLOAD: return "GCODE_UPLOAD";
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
        case STATE_GCODE_UPLOAD:
            isVectorMoving = false;
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
    // 1. ПРОВЕРКА ТАЙМЕРА: Заходим внутрь строго с шагом statusInterval (в обоих режимах)
    if (millis() - lastReportTime < (unsigned long)cfg.statusInterval) {
        return; 
    }
    lastReportTime = millis(); // Сбрасываем таймер на следующий шаг

    // 2. РАСЧЕТ ТЕКУЩИХ КООРДИНАТ ДЕТАЛИ (WCS)
    float mmX = ((float)currentStepsX / cfg.stepsPerMmX) - cfg.wcsOffsetX;
    float mmY = ((float)currentStepsY / cfg.stepsPerMmY) - cfg.wcsOffsetY;
    float mmZ = ((float)currentStepsZ / cfg.stepsPerMmZ) - cfg.wcsOffsetZ;

    // Сборка строки телеметрии
    String currentStatusStr = "<Status:" + getStateName() + 
                              //"|Time=" + String(millis()) +
                              "|Pos:X=" + String(mmX, 2) + 
                              ",Y=" + String(mmY, 2) + 
                              ",Z=" + String(mmZ, 2) + 
                              "|Hom:" + String(isHomed ? "1" : "0") + 
                              "|Line=" + String(currentExecutingLineNum) + ">";

    // 3. ФИЛЬТРАЦИЯ ОТПРАВКИ
    bool needToSend = false;

    if (!cfg.sendOnlyOnChange) {
        // Режим А: Флаг выключен — шлем всегда на каждом тике таймера
        needToSend = true;
    } 
    else if (currentStatusStr != lastSentStatusStr) {
        // Режим Б: Флаг включен — шлем на тике таймера ТОЛЬКО если данные изменились
        needToSend = true;
    }

    // 4. ФИЗИЧЕСКАЯ ОТПРАВКА В ПОРТЫ
    if (needToSend) {
        Serial.println(currentStatusStr);
        sendToWiFiClient(currentStatusStr);
        lastSentStatusStr = currentStatusStr; // Запоминаем последний отправленный пакет
    }
}
