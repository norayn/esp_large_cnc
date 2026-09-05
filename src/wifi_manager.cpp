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
    if (cmd == "HOLD" || cmd == "!") {
        if (currentMachineState == STATE_RUNNING || currentMachineState == STATE_JOGGING) {
            changeState(STATE_HOLD);
            return true;
        }
    }
    if (cmd == "RESUME" || cmd == "~") {
        if (currentMachineState == STATE_HOLD) {
            changeState(STATE_RUNNING);
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
        currentStepsX = 0;
        currentStepsY = 0;
        currentStepsZ = 0;
        isHomed = true; // Снимаем замок станка принудительно!
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
        changeState(STATE_CALIBRATION); runAutoCalibration(); changeState(STATE_IDLE);
        return true;
    } 
    if (cmd == "EXPORT_MAP") {
        changeState(STATE_MAP_TRANSFER); exportMapToPC(); changeState(STATE_IDLE);
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
        clearGCodeBuffer();
        changeState(STATE_GCODE_UPLOAD); // Жестко переключаем автомат в режим приема G-кода!
        sendToWiFiClient("STATUS: Ready");
        return true;
    }
    if (cmd == "START_PROGRAM") {
        if (totalLoadedCommands > 0) { changeState(STATE_RUNNING); executeNextProgramStep(); }
        return true;
    }
    if (cmd.startsWith("START_FROM_LINE:")) {
        int targetLine = cmd.substring(16).toInt();
        bool lineFound = false;
        for (int i = 0; i < totalLoadedCommands; i++) {
            if (gcodeBuffer[i].lineNum == targetLine) { currentCommandIndex = i; lineFound = true; break; }
        }
        if (lineFound) { changeState(STATE_RUNNING); executeNextProgramStep(); }
        return true;
    }
    if (cmd == "GET_LAST_ABORTED_LINE") { sendToWiFiClient("LAST_LINE:" + String(cfg.lastExecutedLine)); return true; }
    if (cmd.startsWith("JOG:")) {
        changeState(STATE_JOGGING); 
        processSingleManualCommand(cmd.substring(4)); 
        //changeState(STATE_IDLE);
        return true;
    }
    if (cmd == "HOME" || cmd == "$H") {
        changeState(STATE_HOMING); runFullHoming(); changeState(STATE_IDLE);
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
            float speed  = data.substring(idx3 + 1).toFloat();

            changeState(STATE_CALIBRATION);
            extern void runGeometryScan(float startX, float endX, float stepX, float feedRate);
            runGeometryScan(startX, endX, stepX, speed);
            changeState(STATE_IDLE);
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
    int firstColon = cmd.indexOf(':');
    if (firstColon == -1) return false;
    
    String data = cmd.substring(firstColon + 1);
    
    int semicolons[5];
    int currentIdx = 0;
    int pos = 0;
    
    while ((pos = data.indexOf(';', pos)) != -1 && currentIdx < 5) {
        semicolons[currentIdx++] = pos;
        pos++;
    }
    
    if (currentIdx < 5) return false;
    
    uint8_t type  = data.substring(0, semicolons[0]).toInt();
    uint16_t line = data.substring(semicolons[0] + 1, semicolons[1]).toInt();
    float x       = data.substring(semicolons[1] + 1, semicolons[2]).toFloat();
    float y       = data.substring(semicolons[2] + 1, semicolons[3]).toFloat();
    float z       = data.substring(semicolons[3] + 1, semicolons[4]).toFloat();
    float f       = data.substring(semicolons[4] + 1).toFloat();
    
    // Просто вызывем функцию, память уже выделена на родине в clearGCodeBuffer()!
    bool success = addBinaryCommand(type, line, x, y, z, f);
    
    if (!success) {
        Serial.printf("PARSER_ERROR: addBinaryCommand failed! Total: %d, Max: %d\n", totalLoadedCommands, MAX_COMMANDS_BUFFER);
    }
    
    return success;
}


