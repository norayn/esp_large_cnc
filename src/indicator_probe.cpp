#include "indicator_probe.h"
#include "pinout.h"
#include "state_manager.h"

void initIndicatorModule() {
    pinMode(INDICATOR_CLK, INPUT_PULLUP);
    pinMode(INDICATOR_DATA, INPUT_PULLUP);
}

float readDialIndicator() {
    if (currentMachineState == STATE_ALARM) return -999.0f;

    unsigned long startTime = millis();
    // 1. Ожидание стартового падения CLK с защитой от Watchdog
    while (digitalRead(INDICATOR_CLK) == HIGH) {
        if (millis() - startTime > 50) return -999.0f;
        delayMicroseconds(5); // Снимаем нагрузку с процессора
    }

    long value = 0;
    int sign = 1;

    // Чтение 24-битного кадра данных
    for (int i = 0; i < 24; i++) {
        unsigned long uStart;

        // 2. Ожидание перехода CLK в HIGH с таймаутом
        uStart = micros();
        while (digitalRead(INDICATOR_CLK) == LOW) {
            if (micros() - uStart > 1000) return -999.0f; // Таймаут 1 мс (частота CLK обычно > 1 кГц)
            delayMicroseconds(2);
        }

        // 3. Ожидание спада CLK в LOW с таймаутом (Защита от обрыва провода)
        uStart = micros();
        while (digitalRead(INDICATOR_CLK) == HIGH) {
            if (micros() - uStart > 1000) return -999.0f; // Таймаут 1 мс
            delayMicroseconds(2);
        }

        // Фиксация бита строго по спаду CLK
        if (digitalRead(INDICATOR_DATA) == HIGH) {
            if (i == 20) {
                sign = -1; // 21-й бит определяет знак минуса
            } else if (i < 20) {
                value |= (1L << i);
            }
        }
    }

    return (float)value * sign / 100.0f;
}
