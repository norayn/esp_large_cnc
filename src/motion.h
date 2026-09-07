#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>

// Структура точки оптической лазерной карты
struct CorrectionPoint {
    float y_offset;
    float z_offset;
};

// Монолитная структура-посредник для неблокирующего обмена кадрами между ядрами
struct InputSegment {
    float x;            // Целевая координата X кадра (мм)
    float y;            // Целевая координата Y кадра (мм)
    float z;            // Целевая координата Z кадра (мм)
    float v_frame;      // Скорость самого кадра/подача (мм/сек, приведенная к секунде)
    float accel;        // Ускорение кадра (мм/сек^2)
    float v_start;      // Скорость входа в этот кадр от Look-Ahead (мм/сек)
    float v_end;        // Скорость выхода из этого кадра на стыке (мм/сек)
    volatile bool hasNewData; // Атомарный флаг-отмашка для Ядра 1
};

// Экспорт физических счетчиков шагов осей станка (для вывода в телеметрию)
extern volatile long currentStepsX;
extern volatile long currentStepsY;
extern volatile long currentStepsZ;

// Главный и единственный флаг синхронизации: true = станок едет, false = полностью замер
extern volatile bool isVectorMoving;

// Экспорт сквозных математических осей планировщика (для синхронизации при START_PROGRAM)
extern volatile long plannerStepsX;
extern volatile long plannerStepsY;
extern volatile long plannerStepsZ;

// Динамический программный Override общей подачи со слайдера PyQt6 (1.0 = 100%)
extern float pythonFeedrateOverride;

// Объявление глобального объекта межъядерного обмена
extern volatile InputSegment nextSeg;

// Прототипы функций модуля движения
void initMotion();
void prepareVectorSegment(float newX, float newY, float newZ, float feedRateMM_Min, 
                          float accelMM_Sec2, float vStartMM_Sec, float vEndMM_Sec);
void updateCurvaturePoint(int index, float y_off, float z_off);
CorrectionPoint getCorrection(float currentX);
bool homeAxis(int stepPin, int dirPin, int dirSign, volatile long &axisSteps, int endstopPin, long pullOffSteps);
void runFullHoming();
void IRAM_ATTR onTimerInterrupt();
void updateFeedrateFading();
void startMotionTask();

#endif // MOTION_H
