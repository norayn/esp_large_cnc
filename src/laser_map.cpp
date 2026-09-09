#include "laser_map.h"
#include "motion.h"
#include "config.h"
#include "pinout.h"
#include "wifi_manager.h"
#include "state_manager.h"
#include "gcode_program.h"

static String camBuffer = "";

void initLaserMap() { camBuffer = ""; }

void parseCamResponse(String line, float &y_off, float &z_off, bool &success) {
    success = false;
    if (!line.startsWith("OFFSET:")) return;
    int yIndex = line.indexOf("Y="); int zIndex = line.indexOf("Z="); int delim = line.indexOf(';', yIndex);
    if (yIndex != -1 && zIndex != -1 && delim != -1) {
        y_off = line.substring(yIndex + 2, delim).toFloat();
        z_off = line.substring(zIndex + 2).toFloat();
        success = true;
    }
}

void updateLaserMapCommunication() {
    while (Serial2.available() > 0) {
        char c = Serial2.read();
        if (c == '\n' || c == '\r') {
            if (camBuffer.length() > 0) {
                float y=0, z=0; bool ok=false;
                parseCamResponse(camBuffer, y, z, ok);
                camBuffer = "";
            }
        } else { camBuffer += c; }
    }
}

void runAutoCalibration() {
    Serial.println("STATUS: Starting laser map auto-calibration...");
    
    for (int i = 0; i < cfg.laserMapSize; i++) {
        // Если станок уже в АВАРИИ, прекращаем итерации по точкам
        if (currentMachineState == STATE_ALARM) {
            Serial.println("ERROR: Calibration cancelled. Machine is in ALARM state.");
            return;
        }

        // 1. Позиционируем каретку в точку X
        float targetX = i * cfg.laserGridStep;
        // Команда на перемещение в машинных координатах...
        
        // 2. Ждем остановки физических осей
        while (isPlannerBusy) {
            checkHardwareSecurity();
            if (currentMachineState == STATE_ALARM) return;
            delay(10);
        }
        
        // Пауза на гашение вибраций станины
        delay(500); 
        
        // 3. Запрос к камере
        Serial2.println("MEASURE");
        unsigned long sWait = millis(); String resp = ""; bool rec = false;
        while (millis() - sWait < 2000) {
            
            
            if (Serial2.available() > 0) {
                char c = Serial2.read();
                if (c == '\n' || c == '\r') { if (resp.length() > 0) { rec = true; break; } }
                else { resp += c; }
            }
            delay(1);
        }

        if (rec) {
            float y = 0, z = 0; bool pOk = false;
            parseCamResponse(resp, y, z, pOk);
            if (pOk) {
                updateCurvaturePoint(i, y, z);
                String dataLog = "DATA: Point " + String(i) + " -> Y=" + String(y) + ", Z=" + String(z);
                Serial.println(dataLog); sendToWiFiClient(dataLog);
            }
        }
    }
}

void exportMapToPC() {
    Serial.println("=== START LASER MAP EXPORT ===");
    sendToWiFiClient("=== START LASER MAP EXPORT ===");
    for (int i = 0; i < cfg.laserMapSize; i++) {
        float x_pos = i * cfg.laserGridStep;
        CorrectionPoint corr = getCorrection(x_pos);
        String row = String(i) + ";" + String(x_pos,1) + ";" + String(corr.y_offset,3) + ";" + String(corr.z_offset,3);
        Serial.println(row); sendToWiFiClient(row);
    }
    Serial.println("=== END LASER MAP EXPORT ===");
    sendToWiFiClient("=== END LASER MAP EXPORT ===");
}

void parseMapRowFromPC(String pcLine) {
    if (!pcLine.startsWith("MAP_LOAD:")) return;
    int idxIndex = pcLine.indexOf("IDX="); int yIndex = pcLine.indexOf("Y="); int zIndex = pcLine.indexOf("Z=");
    int d1 = pcLine.indexOf(';', idxIndex); int d2 = pcLine.indexOf(';', yIndex);
    if (idxIndex != -1 && yIndex != -1 && zIndex != -1) {
        int index = pcLine.substring(idxIndex + 4, d1).toInt();
        float y_off = pcLine.substring(yIndex + 2, d2).toFloat();
        float z_off = pcLine.substring(zIndex + 2).toFloat();
        updateCurvaturePoint(index, y_off, z_off);
    }
}
