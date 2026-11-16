#include "state_manager.h"
#include "pinout.h"
#include "motion.h"
#include "config.h"
#include "wifi_manager.h"
#include "safety_inputs.h"
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

bool changeState(MachineState newState) {
    if (currentMachineState == STATE_ALARM && newState != STATE_IDLE && newState != STATE_ALARM) {
        String err = "ERROR: Machine is in ALARM. Send RESET_ALARM first.";
        Serial.println(err); sendToWiFiClient(err);
        return false;
    }

    if (!isHomed) {
        if (newState == STATE_JOGGING && cfg.allowJogBeforeHoming) { } 
        else if (newState == STATE_RUNNING || newState == STATE_JOGGING || newState == STATE_CALIBRATION) {
            String err = "ERROR: Machine is LOCKED. Run HOMING first.";
            Serial.println(err); sendToWiFiClient(err);
            return false;
        }
    }

    switch (newState) {
        case STATE_ALARM:
            requestMotionStop();
            isVectorMoving = false;
            digitalWrite(MOTORS_ENABLE, HIGH); 
            digitalWrite(RELE_SPINDLE, LOW); 
            digitalWrite(RELE_COOLANT, LOW);

            preferences.begin("cnc_cfg", false);
            preferences.putInt("lastLine", cfg.lastExecutedLine);
            preferences.end();
            break;
        case STATE_IDLE:
            // Любой переход в IDLE — это остановка: очередь шагов сбрасывается,
            // остаток кадра не доигрывается и не «ползёт» на 0,1 мм/с.
            requestMotionStop();
            if (!safetyEstopActive() && !safetyMotorAlarmActive()) {
                digitalWrite(MOTORS_ENABLE, LOW);
            }
            break;
        case STATE_GCODE_UPLOAD:
            isVectorMoving = false;
            break;
        default:
            break;
    }
    currentMachineState = newState;
    return true;
}

// Откуда пришли в HOLD. RESUME возвращает именно туда: пауза во время JOG
// не должна превращаться в запуск УП.
static MachineState holdReturnState = STATE_RUNNING;

bool enterHold() {
    if (currentMachineState != STATE_RUNNING && currentMachineState != STATE_JOGGING) {
        return false;
    }
    holdReturnState = currentMachineState;
    return changeState(STATE_HOLD);
}

bool resumeFromHold() {
    if (currentMachineState != STATE_HOLD) return false;
    if (changeState(holdReturnState)) return true;

    // Вернуться в JOGGING можно не всегда (хоуминг/ALARM): тогда в IDLE,
    // чтобы станок не остался в HOLD без возможности выйти.
    changeState(STATE_IDLE);
    return false;
}

void checkHardwareSecurity() {
    // Аварию мог обнаружить таймерный ISR: он только обесточил драйверы
    // и выставил флаг. FSM, NVS и Serial обновляем здесь, в контексте loop().
    if (consumeAlarmRequest()) {
        if (currentMachineState != STATE_ALARM) {
            changeState(STATE_ALARM);
            Serial.println("CRITICAL: Hardware alarm triggered by timer ISR!");
        }
    }

    // Аппаратная кнопка аварийной остановки (E-STOP) должна работать ВСЕГДА
    if (safetyEstopActive()) { // HIGH = нажата / цепь разорвана
        if (currentMachineState != STATE_ALARM) {
            changeState(STATE_ALARM);
            Serial.println("CRITICAL: E-STOP Button Pressed!");
        }
        return;
    }

    if (safetyMotorAlarmActive()) {
        if (currentMachineState != STATE_ALARM) {
            changeState(STATE_ALARM);
            Serial.println("CRITICAL: Motor alarm!");
        }
        return;
    }

    // ХОУМИНГ-ПАТЧ: Если идет легальный поиск баз, отключаем аварийный триггер концевиков.
    // За их триггеры сейчас отвечает алгоритм поиска нуля в motion.cpp.
    if (currentMachineState == STATE_HOMING) {
        return; 
    }

    // Стандартная рантайм-защита (работает в IDLE, RUNNING, JOGGING и т.д.)
    if (safetyEndstopsActive()) {
        if (currentMachineState != STATE_ALARM) {
            changeState(STATE_ALARM);
            Serial.println("CRITICAL: Limit switch triggered during runtime operation!");
        }
    }
}


void reportStatusToPC() {
    // 1. ПРОВЕРКА ТАЙМЕРА: Заходим внутрь строго с шагом statusInterval (в обоих режимах).
    // SET_CONFIG пока не валидирует поля (P2-5), поэтому 0 или отрицательное
    // значение слало бы телеметрию каждый круг supervisorTask и забило бы
    // сокет и USB. Нижняя граница — 50 мс (20 строк/с).
    static const long MIN_STATUS_INTERVAL_MS = 50;
    const long configured = (long)cfg.statusInterval;
    const unsigned long interval = (configured < MIN_STATUS_INTERVAL_MS)
                                       ? (unsigned long)MIN_STATUS_INTERVAL_MS
                                       : (unsigned long)configured;
    if (millis() - lastReportTime < interval) {
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

    // 4. ФИЗИЧЕСКАЯ ОТПРАВКА В ПОРТЫ (ровно одна строка в TCP и одна в USB)
    if (needToSend) {
        sendToBothClients(currentStatusStr);
        lastSentStatusStr = currentStatusStr; // Запоминаем последний отправленный пакет
    }
}
