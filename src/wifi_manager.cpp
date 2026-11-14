#include "wifi_manager.h"
#include <WiFi.h>
#include "gcode_program.h"
#include "state_manager.h"
#include "laser_map.h"
#include "config.h"
#include "motion.h"

// Сетевые настройки (Станок как точка доступа AP)
static const char* ssid = "CNC_4METERS_ROUTER";
static const char* password = "CNC_password_123";
WiFiServer tcpServer(8888);
WiFiClient tcpClient;
bool isWiFiClientConnected = false;
static String wifiInputBuffer = "";

// --- ПРОТОТИПЫ СЕРВИСНЫХ ФУНКЦИЙ ОБРАБОТКИ КОМАНД ---
static bool  handleGlobalCommands(const String& cmd);
static bool  handleIdleCommands(const String& cmd);
static bool  handleMapTransferCommands(const String& cmd);
static bool  handleBinaryCommandPacket(const String& cmd);

void initWiFiManager() {
    WiFi.softAP(ssid, password);
    tcpServer.begin();
    wifiInputBuffer = "";
}

void sendToWiFiClient(String message) {
    if (isWiFiClientConnected && tcpClient.connected()) {
        tcpClient.println(message);
    }

    Serial.println(message); 
}

String processIncomingCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return "";

    // 1. Глобальные аварийные команды (работают всегда)
    if (handleGlobalCommands(cmd)) return "ok";

    // 2. Если мы НАХОДИМСЯ в режиме загрузки G-кода
    if (currentMachineState == STATE_GCODE_UPLOAD) {
        if (cmd.startsWith("B:")) {
            return handleBinaryCommandPacket(cmd) ? "ok" : "ERROR: Add to RAM buffer failed";
        }
        if (cmd.startsWith("GCODE_UPLOAD_END")) {
            changeState(STATE_IDLE); // Выходим из режима загрузки
            return "STATUS: Upload complete. Loaded " + String(totalLoadedCommands) + " commands.\nok"; 
            // Возвращаем и статус, и финальный ok одной строкой!
        }
    }

    // 3. Если мы в режиме ожидания
    if (currentMachineState == STATE_IDLE) {
        if (handleIdleCommands(cmd)) return "ok";
    }

    // 4. Передача лазерных карт
    if (currentMachineState == STATE_MAP_TRANSFER) {
        if (handleMapTransferCommands(cmd)) return "ok";
    }

    return "ERROR: Unknown or blocked command: " + cmd;
}


// ГЛАВНЫЙ СЕТЕВОЙ ЦИКЛ (Теперь он чистый и читаемый)
void updateWiFiCommunication() {
    if (!isWiFiClientConnected) {
        tcpClient = tcpServer.available();
        if (tcpClient) isWiFiClientConnected = true;
    }
    if (!isWiFiClientConnected) return;
    if (!tcpClient.connected()) { isWiFiClientConnected = false; return; }

    while (tcpClient.available() > 0) {
        char c = tcpClient.read();
        
        // Читаем строго до разделителей кадра ЧПУ
        if (c == '\n' || c == '\r') {
            wifiInputBuffer.trim();
            
            if (wifiInputBuffer.length() > 0) {
                // МГНОВЕННО скармливаем строку единому диспетчеру ЧПУ!
                // Он сам знает все стейты, бинарные пакеты и команды
                String response = processIncomingCommand(wifiInputBuffer);
                
                if (response.length() > 0) {
                    sendToWiFiClient(response); // Шлем "ok" или "ERROR" Питону
                }
                
                wifiInputBuffer = ""; // ЖЕСТКАЯ ОЧИСТКА БУФЕРА СРАЗУ ПОСЛЕ ОТВЕТА!
            }
        } else {
            wifiInputBuffer += c;
        }
    }

    // ЗАЩИТА ОТ ОБРЫВА СТРОКИ: Если Питон прислал команду без \n в конце пакета,
    // и буфер сокета опустел, принудительно обрабатываем накопленный остаток
    if (wifiInputBuffer.length() > 0 && tcpClient.available() == 0) {
        wifiInputBuffer.trim();
        if (wifiInputBuffer.length() > 0) {
            String response = processIncomingCommand(wifiInputBuffer);
            if (response.length() > 0) {
                sendToWiFiClient(response);
            }
        }
        wifiInputBuffer = ""; // Обнуляем хвост намертво
    }
}


// ==========================================================
// РЕАЛИЗАЦИЯ ИЗОЛИРОВАННЫХ МОДУЛЕЙ ОБРАБОТКИ
// ==========================================================

// 1. Глобальные команды безопасности (Доступны в любом состоянии станка)
static bool handleGlobalCommands(const String& cmd) {
    if (cmd == "ESTOP") {
        changeState(STATE_ALARM);
        return true;
    } 
    if (cmd == "RESET_ALARM") {
        isHomed = false; 
        changeState(STATE_IDLE);
        return true;
    }
    if (cmd == "STOP") {
        // Мягкая остановка: сохраняем isHomed, но полностью прекращаем движение
        // и очищаем очередь. В ALARM остаёмся в ALARM до RESET_ALARM.
        requestMotionStop();
        if (currentMachineState != STATE_ALARM) {
            changeState(STATE_IDLE);
        }
        return true;
    }
    if (cmd == "HOLD" || cmd == "!") {
        if (currentMachineState == STATE_RUNNING || currentMachineState == STATE_JOGGING) {
            changeState(STATE_HOLD);
            return true;
        }
    }
    if (cmd == "RESUME" || cmd == "~") {
        if (currentMachineState == STATE_HOLD) {
            changeState(STATE_RUNNING); // при отказе changeState сам выдаст ERROR
            return true;
        }
    }
    // Динамический Override скорости тоже должен быть доступен прямо во время пиления!
    if (cmd.startsWith("OVERRIDE=")) {
        float value = cmd.substring(9).toFloat();
        if (value >= 0.1f && value <= 2.0f) {
            pythonFeedrateOverride = value;
            return true;
        }
    }
    return false;
}

// 2. Команды управления в режиме ожидания (IDLE)
static bool handleIdleCommands(const String& cmd) {
    if (cmd == "SET_HOMED_DEBUG") {
        // Не обнуляем currentSteps* без физического движения (P0-4): иначе
        // телеметрия и первый JOG считают от неверной позиции. Только снимаем
        // блокировку и синхронизируем математику планировщика с фактом.
        syncPlannerToActual();
        isHomed = true;
        sendToWiFiClient("STATUS: DEBUG HOMING SUCCESSFUL. Machine unlocked.");
        return true;
    }
    // Обнуление координат детали (WCS G54)
    if (cmd.startsWith("SET_ZERO:")) {
        String ax = cmd.substring(9);
        if (ax == "ALL") setWcsZero('A');
        else if (ax.length() > 0) setWcsZero(ax[0]);
        sendToWiFiClient("STATUS: WCS Zero Updated.");
        return true;
    }
    if (cmd == "GET_CONFIG" || cmd.startsWith("SET_CONFIG:")) {
        String response = handleConfigCommand(cmd);
        if (response.length() > 0) {
            sendToWiFiClient(response);
            return true;
        }
        return false;
    }
    if (cmd == "START_CALIBRATION") {
        if (!changeState(STATE_CALIBRATION)) return true; // отказ: ERROR уже отправлен
        runAutoCalibration();
        if (currentMachineState == STATE_CALIBRATION) changeState(STATE_IDLE);
        return true;
    } 
    if (cmd == "EXPORT_MAP") {
        if (!changeState(STATE_MAP_TRANSFER)) return true;
        exportMapToPC();
        changeState(STATE_IDLE);
        return true;
    }
    if (cmd == "MAP_LOAD_START") {
        changeState(STATE_MAP_TRANSFER);
        return true;
    }
    if (cmd == "SINGLE_BLOCK_ON") { isSingleBlockMode = true; waitNextBlockTrigger = false; return true; }
    if (cmd == "SINGLE_BLOCK_OFF") { isSingleBlockMode = false; waitNextBlockTrigger = false; return true; }
    if (cmd == "SET_ALIGN_A") { setAlignmentPoint('A'); return true; }
    if (cmd == "SET_ALIGN_B") { setAlignmentPoint('B'); return true; }
    if (cmd == "ACTIVATE_ALIGN") { calculateAlignmentMatrix(); return true; }
    if (cmd == "DEACTIVATE_ALIGN") {
        extern bool isAlignmentActive; isAlignmentActive = false; cfg.isAlignmentActive = false;
        extern void saveAlignmentToEEPROM(); saveAlignmentToEEPROM();
        return true;
    }
    if (cmd == "GCODE_UPLOAD_START") {
        // Жестко переключаем автомат в режим приема G-кода!
        if (!changeState(STATE_GCODE_UPLOAD)) return true;
        clearGCodeBuffer();
        sendToWiFiClient("STATUS: Ready");
        return true;
    }
    if (cmd == "START_PROGRAM") {
        startProgramExecution(); // внутри проверки ALARM/хоуминга/пустого буфера
        return true;
    }
    if (cmd.startsWith("START_FROM_LINE:")) {
        int targetLine = cmd.substring(16).toInt();
        bool lineFound = false;
        for (int i = 0; i < totalLoadedCommands; i++) {
            if (gcodeBuffer[i].lineNum == targetLine) { currentCommandIndex = i; lineFound = true; break; }
        }
        if (lineFound) { startProgramExecution(); }
        return true;
    }
    if (cmd == "GET_LAST_ABORTED_LINE") { sendToWiFiClient("LAST_LINE:" + String(cfg.lastExecutedLine)); return true; }
    if (cmd.startsWith("JOG:")) {
        if (!changeState(STATE_JOGGING)) return true; // отказ: станок не двигаем
        if (!processSingleManualCommand(cmd.substring(4))) {
            // Команда/цель отклонена — возвращаем управление, не оставляя JOGGING.
            changeState(STATE_IDLE);
        }
        //changeState(STATE_IDLE);
        return true;
    }
    if (cmd == "HOME" || cmd == "$H") {
        if (!changeState(STATE_HOMING)) return true; // отказ: ERROR уже отправлен
        runFullHoming();
        if (currentMachineState == STATE_HOMING) changeState(STATE_IDLE);
        return true;
    }
    if (cmd.startsWith("SCAN_GEOMETRY:")) {
        String data = cmd.substring(14);
        int idx1 = data.indexOf(';');
        int idx2 = data.indexOf(';', idx1 + 1);
        int idx3 = data.indexOf(';', idx2 + 1);

        if (idx1 != -1 && idx2 != -1 && idx3 != -1) {
            float startX = data.substring(0, idx1).toFloat();
            float endX   = data.substring(idx1 + 1, idx2).toFloat();
            float stepX  = data.substring(idx2 + 1, idx3).toFloat();
            // P0-7: SPEED в SCAN_GEOMETRY — мм/мин (как в GUI и G-коде).
            float speedMmMin = data.substring(idx3 + 1).toFloat();

            if (!isfinite(startX) || !isfinite(endX) || !isfinite(stepX) ||
                !isfinite(speedMmMin) || stepX <= 0.0f || startX == endX ||
                speedMmMin < 1.0f || speedMmMin > 20000.0f) {
                String err = "ERROR: SCAN_GEOMETRY requires finite START/END, STEP>0 and SPEED 1..20000 mm/min.";
                Serial.println(err); sendToWiFiClient(err);
                return true;
            }

            // P0-8: скан не начинаем, если любая крайняя точка вне поля.
            String limitErr;
            if (!validateMachineTarget(startX, 0.0f, 0.0f, &limitErr) ||
                !validateMachineTarget(endX, 0.0f, 0.0f, &limitErr)) {
                Serial.println(limitErr); sendToWiFiClient(limitErr);
                return true;
            }

            if (!changeState(STATE_CALIBRATION)) return true; // отказ: ERROR уже отправлен
            extern void runGeometryScan(float startX, float endX, float stepX, float feedRateMM_Sec);
            runGeometryScan(startX, endX, stepX, speedMmMin / 60.0f);
            if (currentMachineState == STATE_CALIBRATION) changeState(STATE_IDLE);
            return true; // Команда успешно выполнена
        }
        return false;
    }
    
    return false; // Ни одна команда не совпала
}

// 3. Команды импорта карты лазера с ПК
static bool handleMapTransferCommands(const String& cmd) {
    if (cmd.startsWith("MAP_LOAD:")) { parseMapRowFromPC(cmd); return true; } 
    if (cmd == "MAP_LOAD_END") { changeState(STATE_IDLE); return true; }
    return false;
}

// 4. Прием и декомпозиция бинарных кадров G-кода (Универсально, доступно при заливке)
// Вызывается из основного цикла, если пришел маркер "B:"
bool handleBinaryCommandPacket(const String& cmd) {
    // 1. Извлекаем указатель на сырой сишный буфер строки, полностью исключая фрагментацию кучи
    const char* payload = cmd.c_str();

    // 2. Локальные переменные для временного хранения данных кадра на стеке
    int typeInt = 0;
    int lineInt = 0;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float f = 0.0f;
    float v_start = 0.0f;
    float v_end = 0.0f;

    // 3. Вызываем sscanf для быстрого разбора по жесткому шаблону разделителей
    int parsedFields = sscanf(payload, "B:%d;%d;%f;%f;%f;%f;%f;%f", 
                              &typeInt, &lineInt, &x, &y, &z, &f, &v_start, &v_end);

    // 4. Защита: sscanf обязан успешно заполнить все 8 объявленных полей
    if (parsedFields != 8) {
        Serial.printf("ERROR: Look-Ahead binary packet corrupt! Parsed fields: %d/8. Raw: %s\n", parsedFields, payload);
        return false; 
    }

    // Приведение типов под сигнатуру буфера УП ЧПУ (gcode_program)
    uint8_t type = (uint8_t)typeInt;
    uint16_t line = (uint16_t)lineInt;

    // 5. Передаем данные в ОЗУ-массив станка
    return addBinaryCommand(type, line, x, y, z, f, v_start, v_end);
}

