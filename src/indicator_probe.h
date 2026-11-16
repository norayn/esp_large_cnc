#ifndef INDICATOR_PROBE_H
#define INDICATOR_PROBE_H

#include <Arduino.h>

// Инициализация пинов индикатора
void initIndicatorModule();

// Быстрый одиночный опрос индикатора часового типа с таймаутом (проверка связи)
// Возвращает значение в мм. Если датчик отключен, возвращает -999.0
float readDialIndicator();

// Сам скан геометрии живёт в scan_task.cpp (P1-8): он идёт по шагам из
// supervisorTask, а не блокирует loop() на всё время процедуры.

#endif
