#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>

// Временная сетка шагового таймера. Таймер тикает каждые 5 мкс (200 кГц),
// но ISR двухфазный: фронт STEP формируется на одном тике, спад — на
// следующем. Поэтому одна ячейка кольца = 2 тика = 10 мкс (100 кГц).
// Единственное место, где задаётся частота: main.cpp/timerAlarmWrite().
#define CNC_TIMER_TICK_US 5

// Ячейка кольца и производные от неё частоты.
#define CNC_CELL_US (CNC_TIMER_TICK_US * 2.0f)
#define CNC_CELL_HZ (1000000.0f / CNC_CELL_US)

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
};

// Глубина межъядерной очереди кадров (P1-3). Один кадр = до 40,96 мс в кольце,
// поэтому 4 слота с запасом покрывают паузу планировщика между кадрами: к
// моменту, когда кольцо тактов опустеет, следующий кадр уже ждёт на Core 1.
#define FRAME_QUEUE_SIZE 4

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

// Немедленная остановка движения (STOP / RESET_ALARM / ALARM).
// Останавливает таймер, очищает кольцевой буфер и синхронизирует базу планировщика
// с фактическими счётчиками шагов. Вызывать только из контекста loop/задачи.
void requestMotionStop();

// true, пока движение остановлено и требует явного разрешения (prepareVectorSegment).
bool isMotionStopRequested();

// Забирает флаг аварии, установленный ISR. true = была авария, требуется changeState(ALARM).
bool consumeAlarmRequest();

// Приравнивает математическую базу планировщика к фактическим счётчикам шагов.
// Вызывать перед JOG/стартом УП, после аварии и после SET_HOMED_DEBUG.
void syncPlannerToActual();

// Проверка точки в машинных координатах с учётом лазерной поправки Y/Z.
// false = координата не конечна или выходит за minX…maxZ; reason — текст ошибки.
bool validateMachineTarget(float x, float y, float z, String* reason = nullptr);

// Объявление межъядерной очереди кадров (P1-3). Слоты и индексы скрыты в
// motion.cpp: наружу нужен только ответ «есть ли куда положить кадр».
// true = свободный слот есть, Core 0 может поставить кадр без ожидания.
bool canQueueFrame();

// Число кадров, ожидающих генерации на Core 1 (телеметрия/отладка).
uint8_t queuedFrameCount();

// Результат поиска базы одной оси
enum HomingResult {
    HOMING_OK = 0,     // база найдена, счётчик оси обнулён
    HOMING_ABORTED,    // мягкая остановка (STOP/смена состояния), не авария
    HOMING_FAILED      // ESTOP/MOTORS_ALARM/таймаут/концевик -> нужен ALARM
};

// Прототипы функций модуля движения
void initMotion();
// false = кадр не поставлен в очередь (стоп/авария/некорректная цель).
bool prepareVectorSegment(float newX, float newY, float newZ, float feedRateMM_Sec,
                          float accelMM_Sec2, float vStartMM_Sec, float vEndMM_Sec);
void updateCurvaturePoint(int index, float y_off, float z_off);
CorrectionPoint getCorrection(float currentX);
HomingResult homeAxis(int stepPin, int dirPin, int dirSign, volatile long &axisSteps, int endstopPin, long pullOffSteps);
void runFullHoming();
void IRAM_ATTR onTimerInterrupt();
void updateFeedrateFading();
void startMotionTask();

#endif // MOTION_H
