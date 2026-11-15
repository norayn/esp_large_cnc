#ifndef SAFETY_INPUTS_H
#define SAFETY_INPUTS_H

#include <Arduino.h>
#include "pinout.h"

// ============================================================================
// АППАРАТНЫЕ ВХОДЫ БЕЗОПАСНОСТИ (P0-9)
// ============================================================================
// Все три входа — GPIO 34/35/39 — это input-only выводы ESP32 без внутренней
// подтяжки: pinMode(INPUT_PULLUP) для них не действует. Нужны внешние
// резисторы (pull-down к GND) и оптроны/гальваническая развязка.
//
// Активный уровень — HIGH:
//   ESTOP         HIGH = кнопка нажата / цепь разорвана
//   MOTORS_ALARM  HIGH = хотя бы один драйвер сообщил аварию
//   ENDSTOPS      HIGH = NC-цепь концевиков разомкнута (сработал датчик/обрыв)
// Норма (станок готов): все три входа LOW.
//
// Флаг SAFETY_INPUTS_ENABLED задаётся в pinout.h:
//   1 — проверки включены (штатная эксплуатация);
//   0 — проверки отключены (стенд/отладка, датчики физически не подключены).
// ============================================================================

// Настройка входов. Внутренняя подтяжка для GPIO 34/35/39 не используется.
void initSafetyInputs();

// Самотест NC-цепи при старте. Возвращает false, если любой вход уже активен
// (норма — все LOW). При SAFETY_INPUTS_ENABLED == 0 всегда true и выдаёт
// предупреждение в reason.
bool runSafetySelfTest(String* reason = nullptr);

// --- Быстрые проверки, пригодные для вызова из ISR ---
// При SAFETY_INPUTS_ENABLED == 0 компилятор сворачивает их в константу false,
// поэтому digitalRead() в прерывании не выполняется.
static inline bool IRAM_ATTR safetyEstopActive() {
#if SAFETY_INPUTS_ENABLED
    return digitalRead(ESTOP_PIN) == HIGH;
#else
    return false;
#endif
}

static inline bool IRAM_ATTR safetyMotorAlarmActive() {
#if SAFETY_INPUTS_ENABLED
    return digitalRead(MOTORS_ALARM) == HIGH;
#else
    return false;
#endif
}

static inline bool IRAM_ATTR safetyEndstopsActive() {
#if SAFETY_INPUTS_ENABLED
    return digitalRead(ENDSTOPS_PIN) == HIGH;
#else
    return false;
#endif
}

#endif
