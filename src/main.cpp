#include <Arduino.h>
#include "pinout.h"
#include "config.h"
#include "motion.h"
#include "gcode_program.h"
#include "laser_map.h"
#include "state_manager.h"
#include "wifi_manager.h"
#include "indicator_probe.h"

hw_timer_t * cncTimer = NULL;
String inputBuffer = "";

void setup() {
    setupAndLoadConfig();
    initMotion();
    initGCodeModule();
    initLaserMap();
    initIndicatorModule();
    initWiFiManager();

    // Силовые выходы
    pinMode(X_STEP_PIN, OUTPUT); pinMode(X_DIR_PIN, OUTPUT);
    pinMode(Y_STEP_PIN, OUTPUT); pinMode(Y_DIR_PIN, OUTPUT);
    pinMode(Z_STEP_PIN, OUTPUT); pinMode(Z_DIR_PIN, OUTPUT);
    pinMode(MOTORS_ENABLE, OUTPUT);
    pinMode(RELE_SPINDLE, OUTPUT); pinMode(RELE_COOLANT, OUTPUT);

    // Входы защиты
    pinMode(ESTOP_PIN, INPUT_PULLUP);
    pinMode(MOTORS_ALARM, INPUT_PULLUP);
    pinMode(ENDSTOPS_PIN, INPUT_PULLUP);

    Serial.begin(115200);
    Serial.setTxBufferSize(2560); 

    // Порт для лазера ESP32-CAM (длина строк малая, оставляем дефолтным)
    Serial2.begin(115200, SERIAL_8N1, CAM_RX2_PIN, CAM_TX2_PIN);

    // Инициализация аппаратного таймера (100 кГц)
    cncTimer = timerBegin(0, 80, true); 
    timerAttachInterrupt(cncTimer, &onTimerInterrupt, true); 
    timerAlarmWrite(cncTimer, 10, true); 
    timerAlarmEnable(cncTimer); 

    // Запуск фонового потока планировщика на Ядре 1
    startMotionTask(); 

    changeState(STATE_IDLE);
    Serial.println("SYSTEM: Modular CNC Firmware Fully Loaded.");
}

void loop() {
    checkHardwareSecurity();
    reportStatusToPC();
    updateWiFiCommunication();
    updateLaserMapCommunication();
    updateFeedrateFading();

    // ОБРАБОТКА КОМАНД ИЗ USB-UART ПОРТА ПК
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (inputBuffer.length() > 0) {
                // Прямой вызов единого диспетчера
                String response = processIncomingCommand(inputBuffer);
                
                if (response.length() > 0) {
                    Serial.println(response); // Ответ "ok" или "ERROR" улетает в USB
                }
                inputBuffer = ""; 
            }
        } else { 
            inputBuffer += c; 
        }
    }

    // Очередь автоматической программы G-кода из ОЗУ
    if (currentMachineState == STATE_RUNNING) {
        // Берем следующий шаг программы из ОЗУ только если физические буферы пусты
        if (bufferTail == bufferHead && !nextSeg.hasNewData && !isVectorMoving) {
            executeNextProgramStep(); 
        }
    }
}
