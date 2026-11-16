#include "laser_map.h"
#include "motion.h"
#include "config.h"
#include "pinout.h"
#include "wifi_manager.h"
#include "state_manager.h"
#include "gcode_program.h"

static String camBuffer = "";

// Последний разобранный ответ камеры. Нужен сканам: Serial2 читает только
// updateLaserMapCommunication(), второй читатель украл бы строку.
static float lastCamY = 0.0f;
static float lastCamZ = 0.0f;
static unsigned long lastCamRespMs = 0;
static bool lastCamOk = false;

void initLaserMap() { camBuffer = ""; lastCamOk = false; }

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
                if (ok) {
                    lastCamY = y; lastCamZ = z;
                    lastCamRespMs = millis();
                    lastCamOk = true;
                }
                camBuffer = "";
            }
        } else { camBuffer += c; }
    }
}

// Ответ камеры забирается один раз: сканы ждут именно свою порцию, а не
// любую строку из буфера.
bool takeCamResponse(unsigned long sinceMs, float &y_off, float &z_off) {
    if (!lastCamOk) return false;
    if ((long)(lastCamRespMs - sinceMs) < 0) return false; // ответ старше запроса

    y_off = lastCamY;
    z_off = lastCamZ;
    lastCamOk = false;
    return true;
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
