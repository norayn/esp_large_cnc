#include "indicator_probe.h"
#include "pinout.h"
#include "motion.h"
#include "config.h"
#include "wifi_manager.h"
#include "state_manager.h"

void initIndicatorModule() {
    pinMode(INDICATOR_CLK, INPUT_PULLUP);
    pinMode(INDICATOR_DATA, INPUT_PULLUP);
}

float readDialIndicator() {
    unsigned long timeout = millis();
    // Ждем начала пакета данных от индикатора (когда CLK упадет в LOW)
    while (digitalRead(INDICATOR_CLK) == HIGH) {
        if (millis() - timeout > 50) { 
            return -999.0f; // Физически отключен (нет тактовых импульсов)
        }
    }

    long value = 0;
    int sign = 1;
    
    // Китайский протокол: передается 24 бита данных
    for (int i = 0; i < 24; i++) {
        // Ждем спада тактового импульса
        while (digitalRead(INDICATOR_CLK) == HIGH);
        
        // Читаем бит данных в момент низкого уровня CLK
        if (digitalRead(INDICATOR_DATA) == HIGH) {
            if (i == 20) sign = -1; // 21-й бит обычно отвечает за знак минус
            else if (i < 20) {
                value |= (1L << i);
            }
        }
        // Ждем возврата CLK в HIGH
        while (digitalRead(INDICATOR_CLK) == LOW);
    }

    // Перевод попугаев индикатора в честные миллиметры (обычно 100 или 200 шагов на 1 мм)
    float result = (float)value / 100.0f * sign;
    return result;
}

void runGeometryScan(float startX, float endX, float stepX, float feedRate) {
    Serial.println("=== START GEOMETRY SCAN ===");
    sendToWiFiClient("=== START GEOMETRY SCAN ===");
    Serial.println("X_POS;DIAL_INDICATOR;LASER_Y;LASER_Z");
    sendToWiFiClient("X_POS;DIAL_INDICATOR;LASER_Y;LASER_Z");

    // Вычисляем направление движения
    int stepsCount = abs((endX - startX) / stepX) + 1;
    float currentX = startX;

    for (int i = 0; i < stepsCount; i++) {
        // Командуем планировщику ехать в точку замера (Y и Z удерживаем в нулях)
        prepareVectorSegment(currentX, 0.0f, 0.0f, feedRate, cfg.maxAcceleration, cfg.minVectorSpeed, cfg.minVectorSpeed);
        while (isVectorMoving) { delay(10); }
        
        delay(600); // Стабилизация пространственной фермы после остановки

        // 1. Опрос часового индикатора
        float dialValue = readDialIndicator();
        String dialStr = (dialValue == -999.0f) ? "DISCONNECTED" : String(dialValue, 3);

        // 2. Опрос лазерного модуля ESP32-CAM с таймаутом проверки связи
        Serial2.println("MEASURE");
        unsigned long sWait = millis(); 
        String laserResp = ""; 
        bool laserConnected = false;

        while (millis() - sWait < 1500) { // Таймаут 1.5 сек
            if (Serial2.available() > 0) {
                char c = Serial2.read();
                if (c == '\n' || c == '\r') {
                    if (laserResp.length() > 0) { laserConnected = true; break; }
                } else { laserResp += c; }
            }
            delay(1);
        }

        float laserY = 0.0f, laserZ = 0.0f;
        String laserYStr = "DISCONNECTED";
        String laserZStr = "DISCONNECTED";

        if (laserConnected && laserResp.startsWith("OFFSET:")) {
            int yIdx = laserResp.indexOf("Y="); 
            int zIdx = laserResp.indexOf("Z="); 
            int delim = laserResp.indexOf(';', yIdx);
            if (yIdx != -1 && zIdx != -1 && delim != -1) {
                laserY = laserResp.substring(yIdx + 2, delim).toFloat();
                laserZ = laserResp.substring(zIdx + 2).toFloat();
                laserYStr = String(laserY, 3);
                laserZStr = String(laserZ, 3);
            }
        }

        // 3. Формируем единую CSV-строку и выплевываем на ПК
        String csvRow = String(currentX, 1) + ";" + dialStr + ";" + laserYStr + ";" + laserZStr;
        Serial.println(csvRow);
        sendToWiFiClient(csvRow);

        // Шагаем дальше
        if (endX > startX) currentX += stepX;
        else currentX -= stepX;
    }

    Serial.println("=== END GEOMETRY SCAN ===");
    sendToWiFiClient("=== END GEOMETRY SCAN ===");
}
