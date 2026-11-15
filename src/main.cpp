#include <Arduino.h>
#include "pinout.h"
#include "config.h"
#include "motion.h"
#include "gcode_program.h"
#include "laser_map.h"
#include "state_manager.h"
#include "wifi_manager.h"
#include "indicator_probe.h"
#include "safety_inputs.h"

hw_timer_t * cncTimer = NULL;
String inputBuffer = "";

// Супервизор прикладного уровня, привязан к Core 0 (см. P1-2).
static void supervisorTask(void *parameter);

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

    // Входы защиты: GPIO 34/35/39 — input-only, INPUT_PULLUP не действует.
    initSafetyInputs();

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

    // P0-9: самотест NC-цепи до перехода в готовность. При обрыве/нажатии
    // любого входа станок сразу уходит в ALARM, а не рапортует IDLE.
    String safetyReason;
    bool safetyOk = runSafetySelfTest(&safetyReason);
    if (safetyReason.length() > 0) sendToWiFiClient(safetyReason);

    if (safetyOk) {
        changeState(STATE_IDLE);
    } else {
        changeState(STATE_ALARM);
    }

    // P1-2: FSM/Wi-Fi/USB/телеметрия уходят на Core 0, чтобы не конкурировать
    // с генератором шагов (motionTask) и ISR на Core 1.
    BaseType_t created = xTaskCreatePinnedToCore(
        supervisorTask,
        "supervisorTask",
        8192,
        NULL,
        1,
        NULL,
        0
    );
    if (created != pdPASS) {
        Serial.println("ERROR: Failed to create supervisorTask on Core 0!");
    } else {
        Serial.printf("SYSTEM: setup/timer ISR core=%d, supervisor=Core 0, motionTask=Core 1\n",
                      xPortGetCoreID());
    }

    Serial.println("SYSTEM: Modular CNC Firmware Fully Loaded.");
}

// ============================================================================
// СУПЕРВИЗОР (Core 0): FSM, защита, Wi-Fi, USB, телеметрия, выдача кадров.
// ============================================================================
static void supervisorTask(void *parameter) {
    (void)parameter;

    Serial.printf("SYSTEM: supervisorTask started on Core %d\n", xPortGetCoreID());

    for (;;) {
        checkHardwareSecurity();
        reportStatusToPC();
        updateWiFiCommunication();
        updateLaserMapCommunication();

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

        // ====================================================================
        // АКТУАЛИЗИРОВАННЫЙ АВТОМАТ ОЧЕРЕДИ ЧПУ (Look-Ahead Синхронизация)
        // ====================================================================
        // Мы запрашиваем следующий шаг программы из ОЗУ только если:
        // 1. Станок находится в автоматическом режиме выполнения УП (STATE_RUNNING)
        // 2. Предыдущий физический вектор ПОЛНОСТЬЮ выполнен (isPlannerBusy == false)
        // 3. Структура обмена nextSeg свободна и Ядро 1 готово принять новый кадр
        if (currentMachineState == STATE_RUNNING) {
            if (!isPlannerBusy && !nextSeg.hasNewData) {
                executeNextProgramStep();
            }
        }

        // Плавное гашение подачи (HOLD / Пауза)
        updateFeedrateFading();

        // 1 мс достаточно для USB/Wi-Fi/телеметрии и не грузит Core 0 впустую.
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Arduino loopTask остаётся на Core 1 только для setup() и привязки ISR
// таймера к этому ядру. Вся прикладная работа — в supervisorTask (Core 0).
void loop() {
    vTaskDelay(pdMS_TO_TICKS(10));
}
