#ifndef INDICATOR_PROBE_H
#define INDICATOR_PROBE_H

#include <Arduino.h>

// Инициализация пинов индикатора
void initIndicatorModule();

// Быстрый одиночный опрос индикатора часового типа с таймаутом (проверка связи)
// Возвращает значение в мм. Если датчик отключен, возвращает -999.0
float readDialIndicator();

// Функция запуска комплексного замера геометрии вдоль оси X
// На входе: начальная координата X, конечная X, шаг замера, скорость подачи
void runGeometryScan(float startX, float endX, float stepX, float feedRate);

#endif
