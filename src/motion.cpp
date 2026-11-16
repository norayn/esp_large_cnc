#include "motion.h"
#include "config.h"
#include "pinout.h"
#include "gcode_program.h"
#include "state_manager.h"
#include "safety_inputs.h"
#include "wifi_manager.h"
#include <Arduino.h>

// Аппаратный таймер создаётся в main.cpp; нужен для остановки/запуска выдачи шагов.
extern hw_timer_t* cncTimer;

// Размер кольцевой ленты времени: 4096 ячеек × 10 мкс = 40,96 мс подушки ОЗУ.
#define STEP_BUFFER_SIZE 4096
// Структура одной микросекундной ячейки времени
struct StepCmd {
    uint32_t stepX : 1; // 1 = импульс STEP HIGH на нечетном тике таймера, 0 = пустой такт
    uint32_t dirX  : 1; // 1 = HIGH, 0 = LOW
    uint32_t stepY : 1;
    uint32_t dirY  : 1;
    uint32_t stepZ : 1;
    uint32_t dirZ  : 1;
    uint32_t valid : 1; // Флаг готовности такта времени к чтению из ISR прерывания
};

// Статический массив кольцевого буфера тактов времени
static StepCmd stepRingBuffer[STEP_BUFFER_SIZE];

// Локальные внутренние указатели кольца (намертво заперты внутри модуля, не видны извне)
static volatile uint32_t bufferHead = 0; 
static volatile uint32_t bufferTail = 0; 

// Статический флаг для блокировки триггера концевиков в ISR во время поиска баз
static volatile bool isHomingActive = false;

// Аварийный / остановочный запросы. ISR только выставляет флаги и обесточивает
// драйверы; изменение FSM, Preferences и Serial выполняет loop().
static volatile bool motionStopRequested = false;
static volatile bool alarmRequested = false;

// Флаг фазы таймера: true = нечетный такт (HIGH), false = четный такт (LOW)
static volatile bool timerPhaseHigh = true;

// Динамический программный Override общей подачи со слайдера PyQt6 (1.0 = 100%)
float pythonFeedrateOverride = 1.0f;

// Внутренний ПЛАВНЫЙ override, который будет рассчитываться в updateFeedrateFading()
volatile float smoothedFeedrateOverride = 1.0f;

// Переменная плавного аппаратного ограничения скорости для паузы (в мм/сек)
volatile float virtualSpeedLimit = 1000.0f; // Изначально открыт на максимум

// Флаг, сигнализирующий, что генератор DDA был заморожен в мертвой точке паузы.
// Пишется motionTask (Core 1) и сбрасывается requestMotionStop() (Core 0).
static volatile bool isGeneratorSuspendedOnHold = false;

// Глобальное зеркало текущей скорости DDA для связи между ядрами
volatile float sharedCurrentVelocity = 0.0f;


// Честные физические счетчики шагов станка (изменяются ИСКЛЮЧИТЕЛЬНО в прерывании таймера)
volatile long currentStepsX = 0;
volatile long currentStepsY = 0;
volatile long currentStepsZ = 0;

// Единый флаг активности движения для автомата gcode_program и main.cpp
volatile bool isVectorMoving = false;

volatile bool isPlannerBusy = false;

// Абсолютная сквозная математическая координатная база планировщика ЧПУ (в шагах)
volatile long plannerStepsX = 0;
volatile long plannerStepsY = 0;
volatile long plannerStepsZ = 0;

// Массив точек оптической лазерной карты коррекции провиса балки фермы (до 250 точек)
static CorrectionPoint curvatureMap[250];

// Межъядерная очередь кадров (P1-3): Core 0 складывает готовые кадры,
// Core 1 забирает по одному. Все три счётчика меняются только под xferMux,
// поэтому другой ядро всегда видит согласованное состояние (без флага-отмашки).
static InputSegment frameQueue[FRAME_QUEUE_SIZE];
static uint8_t frameHead  = 0; // слот для записи (Core 0)
static uint8_t frameTail  = 0; // слот для чтения (Core 1)
static uint8_t frameCount = 0; // количество занятых слотов

// ============================================================================
// МЕЖЪЯДЕРНАЯ СИНХРОНИЗАЦИЯ (P1-2)
// ============================================================================
// ringMux защищает индексы/ячейки кольцевого буфера и тройку plannerSteps*
// при обмене между Core 0 (homing/stop) и Core 1 (ISR). motionTask пишет
// кольцо на том же ядре, что и ISR, поэтому ему отдельный лок не нужен.
static portMUX_TYPE ringMux = portMUX_INITIALIZER_UNLOCKED;

// xferMux защищает межъядерную очередь кадров frameQueue (Core 0 — producer,
// Core 1 — consumer). Индексы и счётчик меняются только под ним, поэтому
// «стоп во время передачи» закрывается без отдельного флага-отмашки.
static portMUX_TYPE xferMux = portMUX_INITIALIZER_UNLOCKED;

// true, пока motionTask реально генерирует шаги текущего кадра. По нему
// requestMotionStop() на Core 0 понимает, что кольцо уже никто не пишет.
static volatile bool isMotionGenerating = false;

// Вызывать только под xferMux.
static void clearFrameQueueLocked() {
    frameHead  = 0;
    frameTail  = 0;
    frameCount = 0;
}

bool canQueueFrame() {
    bool hasRoom;
    portENTER_CRITICAL(&xferMux);
    hasRoom = (frameCount < FRAME_QUEUE_SIZE);
    portEXIT_CRITICAL(&xferMux);
    return hasRoom;
}

uint8_t queuedFrameCount() {
    uint8_t n;
    portENTER_CRITICAL(&xferMux);
    n = frameCount;
    portEXIT_CRITICAL(&xferMux);
    return n;
}


// ============================================================================
// СТРУКТУРНЫЙ БЛОК: ОСТАНОВКА ДВИЖЕНИЯ И АВАРИЙНЫЕ ЗАПРОСЫ
// ============================================================================

// Разрешает выдачу шагов. Вызывается перед новым кадром/хоумингом.
// Возвращает false, если станок в ALARM или авария ещё не обработана.
static bool armMotionTimer() {
    if (currentMachineState == STATE_ALARM || alarmRequested) return false;

    if (motionStopRequested) {
        // ISR живёт на Core 1, поэтому noInterrupts() здесь не работает.
        portENTER_CRITICAL(&ringMux);
        bufferHead = 0;
        bufferTail = 0;
        motionStopRequested = false;
        portEXIT_CRITICAL(&ringMux);
    }

    // Фазу таймера здесь не трогаем: ISR сам завершит текущий импульс STEP.
    if (cncTimer != nullptr) timerAlarmEnable(cncTimer);
    return true;
}

// Копия базы под уже взятым ringMux (чтобы не брать лок повторно).
static void syncPlannerToActualLocked() {
    plannerStepsX = currentStepsX;
    plannerStepsY = currentStepsY;
    plannerStepsZ = currentStepsZ;
}

// Математическая база планировщика = фактическое положение моторов.
// Вызывать только когда движение остановлено: во время кадра plannerSteps*
// уже указывает на конец поставленных кадров.
void syncPlannerToActual() {
    portENTER_CRITICAL(&ringMux);
    syncPlannerToActualLocked();
    portEXIT_CRITICAL(&ringMux);
}

// Мягкая остановка: выдача шагов прекращается сразу, очередь сбрасывается,
// математическая база планировщика приравнивается к фактической позиции.
void requestMotionStop() {
    motionStopRequested = true;

    if (cncTimer != nullptr) timerAlarmDisable(cncTimer);

    // Отменяем все подготовленные, но ещё не сгенерированные кадры. Лок
    // закрывает окно, когда motionTask уже собирается взять кадр из очереди.
    portENTER_CRITICAL(&xferMux);
    clearFrameQueueLocked();
    portEXIT_CRITICAL(&xferMux);

    // Ждём, пока генератор на Core 1 корректно прервёт текущий кадр.
    const unsigned long STOP_WAIT_MS = 100;
    unsigned long waitStart = millis();
    while (isMotionGenerating && (millis() - waitStart) < STOP_WAIT_MS) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (isMotionGenerating) {
        Serial.println("WARNING: motion generator did not stop in time; forcing stop.");
    }
    __sync_synchronize();

    // Генератор больше не пишет в кольцо: очищаем очередь и синхронизируем
    // математическую базу с фактическим положением.
    portENTER_CRITICAL(&ringMux);
    bufferHead = 0;
    bufferTail = 0;
    syncPlannerToActualLocked();
    portEXIT_CRITICAL(&ringMux);

    digitalWrite(X_STEP_PIN, LOW);
    digitalWrite(Y_STEP_PIN, LOW);
    digitalWrite(Z_STEP_PIN, LOW);

    timerPhaseHigh = true;
    isVectorMoving = false;
    isPlannerBusy = false;
    isGeneratorSuspendedOnHold = false;
    // Следующая пауза не должна гаситься от скорости, которая была до STOP:
    // иначе ramp HOLD стартует с устаревшего значения и тормозит «в пустоту».
    sharedCurrentVelocity = 0.0f;

    // Таймер снова включаем: ISR шагов не выдаёт (motionStopRequested), но
    // продолжает следить за ESTOP/MOTORS_ALARM независимо от loop().
    if (cncTimer != nullptr) timerAlarmEnable(cncTimer);
}

bool isMotionStopRequested() {
    return motionStopRequested;
}

bool consumeAlarmRequest() {
    if (!alarmRequested) return false;
    alarmRequested = false;
    return true;
}


// ============================================================================
// СТРУКТУРНЫЙ БЛОК: ИНИЦИАЛИЗАЦИЯ И ЛАЗЕРНАЯ КОРРЕКЦИЯ
// ============================================================================

void initMotion() {
    isVectorMoving = false;
    isPlannerBusy = false;
    currentStepsX = 0; currentStepsY = 0; currentStepsZ = 0;
    plannerStepsX = 0; plannerStepsY = 0; plannerStepsZ = 0;
    bufferHead = 0;
    bufferTail = 0;
    
    // Безопасное зануление всей лазерной карты при старте контроллера
    for (int i = 0; i < 250; i++) {
        curvatureMap[i].y_offset = 0.0f;
        curvatureMap[i].z_offset = 0.0f;
    }
}

// Запись точки лазерного сканирования из Python-интерфейса в ОЗУ станка
void updateCurvaturePoint(int index, float y_off, float z_off) {
    if (index >= 0 && index < cfg.laserMapSize) {
        curvatureMap[index].y_offset = y_off;
        curvatureMap[index].z_offset = z_off;
    }
}

// Линейная микронная интерполяция провиса фермы между двумя ближайшими точками сканирования
CorrectionPoint getCorrection(float currentX) {
    if (currentX < 0.0f) currentX = 0.0f;
    
    // Ограничиваем максимальную длину рабочей зоны по карте лазера
    float max_len = (cfg.laserMapSize - 1) * cfg.laserGridStep;
    if (currentX > max_len) currentX = max_len;

    // Находим индекс левой опорной точки
    int index = (int)(currentX / cfg.laserGridStep);
    if (index >= cfg.laserMapSize - 1) return curvatureMap[cfg.laserMapSize - 1];

    // Вычисляем коэффициент положения (от 0.0 до 1.0) между точками
    float t = (currentX - (index * cfg.laserGridStep)) / cfg.laserGridStep;
    
    CorrectionPoint corr;
    // Находим промежуточное смещение по Y и Z
    corr.y_offset = curvatureMap[index].y_offset + t * (curvatureMap[index+1].y_offset - curvatureMap[index].y_offset);
    corr.z_offset = curvatureMap[index].z_offset + t * (curvatureMap[index+1].z_offset - curvatureMap[index].z_offset);
    
    return corr;
}

// ============================================================================
// СТРУКТУРНЫЙ БЛОК: ПРОГРАММНЫЕ ЛИМИТЫ (SOFT LIMITS)
// ============================================================================

// Допуск на округление float/шагов, мм. Не является «запасом» рабочей зоны.
static const float SOFT_LIMIT_TOL_MM = 0.01f;

bool validateMachineTarget(float x, float y, float z, String* reason) {
    if (!isfinite(x) || !isfinite(y) || !isfinite(z)) {
        if (reason) *reason = "ERROR: Non-finite target coordinate.";
        return false;
    }

    // В motionTask лазерная коррекция применяется к Y/Z, поэтому лимиты
    // проверяем уже с её учётом. При некорректном конфиге карту не трогаем.
    CorrectionPoint corr = {0.0f, 0.0f};
    if (cfg.laserGridStep > 0.0f && cfg.laserMapSize > 0) corr = getCorrection(x);

    const float mx = x;
    const float my = y + corr.y_offset;
    const float mz = z + corr.z_offset;

    struct AxisLimit {
        const char* name;
        float value;
        float lo;
        float hi;
    };
    const AxisLimit axes[] = {
        {"X", mx, cfg.minX, cfg.maxX},
        {"Y", my, cfg.minY, cfg.maxY},
        {"Z", mz, cfg.minZ, cfg.maxZ},
    };

    for (const AxisLimit &a : axes) {
        if (a.value < a.lo - SOFT_LIMIT_TOL_MM || a.value > a.hi + SOFT_LIMIT_TOL_MM) {
            if (reason) {
                *reason = "ERROR: Soft limit " + String(a.name) + "=" + String(a.value, 3) +
                          " mm outside [" + String(a.lo, 3) + ".." + String(a.hi, 3) + "]";
            }
            return false;
        }
    }
    return true;
}

// ============================================================================
// СТРУКТУРНЫЙ БЛОК: НЕБЛОКИРУЮЩАЯ ПЕРЕДАЧА КАДРА МЕЖДУ ЯДРАМИ (Core 0)
// ============================================================================

// Пределы физики кадра (P1-4). Ниже них математика профиля перестаёт иметь
// смысл: отрицательные длины сегментов, шаг длиной в секунды, NaN в sqrt.
static const float MIN_FRAME_SPEED_MM_S = 0.1f;  // жёсткий низ скорости генерации
// Один шаг не может длиться дольше 100 мс (10 шагов/с). Иначе при почти нулевой
// подаче шаг растягивается на секунды и станок «зависает» на одном микрошаге.
static const uint32_t MAX_STEP_TICK_CELLS = (uint32_t)(CNC_CELL_HZ / 10.0f);

// Проверка физических параметров пакета. NaN/Inf из GUI (деление на 0 в
// Look-Ahead) обязан отклонить кадр, а не превратить его в sqrt(-x) и вечный цикл.
static bool validateFramePhysics(float feedRateMM_Sec, float accelMM_Sec2,
                                 float vStartMM_Sec, float vEndMM_Sec, String* reason) {
    auto fail = [reason](const char* msg) {
        if (reason) *reason = msg;
        return false;
    };

    if (!isfinite(feedRateMM_Sec) || !isfinite(accelMM_Sec2) ||
        !isfinite(vStartMM_Sec) || !isfinite(vEndMM_Sec)) {
        return fail("ERROR: Frame physics is not finite (NaN/Inf).");
    }
    if (feedRateMM_Sec <= 0.0f) {
        return fail("ERROR: Frame feedrate must be > 0 mm/s.");
    }
    if (accelMM_Sec2 <= 0.0f) {
        return fail("ERROR: Frame acceleration must be > 0 mm/s^2.");
    }
    return true;
}

// Скорость стыка в допустимом коридоре [низ, v_max]: выход за верхнюю границу
// делает длины сегментов отрицательными (движение назад), ниже нижней —
// растягивает шаг и рвёт непрерывность стыка с соседним кадром.
static float clampJunctionSpeed(float v, float vMin, float vMax) {
    if (!isfinite(v)) return vMin;
    if (v < vMin) return vMin;
    if (v > vMax) return vMax;
    return v;
}

// Допуск на стык. Look-Ahead считает скорости во float, поэтому 1 % сверху —
// это округление, а не ошибка планировщика; всё, что выше, — ошибка GUI.
static const float JUNCTION_SPEED_TOL = 0.01f;

// v_start/v_end больше подачи кадра — это не «немного быстро», а отрицательные
// длины сегментов: первый сегмент поедет назад. Такой кадр не исполняем вообще.
static bool validateJunctionSpeeds(float vMax, float vStartMM_Sec, float vEndMM_Sec,
                                   String* reason) {
    const float limit = vMax * (1.0f + JUNCTION_SPEED_TOL) + 0.01f;
    if (vStartMM_Sec > limit || vEndMM_Sec > limit) {
        if (reason) *reason = "ERROR: Junction speed exceeds frame feedrate (v_start/v_end > F).";
        return false;
    }
    return true;
}

bool prepareVectorSegment(float newX, float newY, float newZ, float feedRateMM_Sec,
                          float accelMM_Sec2, float vStartMM_Sec, float vEndMM_Sec,
                          String* reason) {
    // Защита от обхода проверки: даже если вызывающий забыл validateMachineTarget().
    if (!validateMachineTarget(newX, newY, newZ, reason)) return false;

    // P1-4: пакет с F=0, отрицательным F или NaN не двигает станок вообще.
    if (!validateFramePhysics(feedRateMM_Sec, accelMM_Sec2,
                              vStartMM_Sec, vEndMM_Sec, reason)) {
        return false;
    }

    if (!armMotionTimer()) return false;

    // P1-4: коридор скоростей стыка. Грубое превышение подачи — ошибка кадра:
    // такой профиль поехал бы назад. В пределах допуска скорость зажимается.
    // Нижняя граница — minVectorSpeed, но не выше подачи самого кадра: иначе
    // медленный JOG (F=1 мм/мин = 0,017 мм/с) подскочил бы до 2 мм/с.
    float vMax = feedRateMM_Sec;
    if (!validateJunctionSpeeds(vMax, vStartMM_Sec, vEndMM_Sec, reason)) return false;

    float vMin = cfg.minVectorSpeed;
    if (!(vMin > MIN_FRAME_SPEED_MM_S)) vMin = MIN_FRAME_SPEED_MM_S;
    if (vMin > vMax) vMin = vMax;
    vStartMM_Sec = clampJunctionSpeed(vStartMM_Sec, vMin, vMax);
    vEndMM_Sec   = clampJunctionSpeed(vEndMM_Sec, vMin, vMax);

    // P1-3: кадр не публикуется напрямую в motionTask, а кладётся в очередь.
    // Пока в очереди есть слот, Core 1 уже генерирует предыдущий кадр — станок
    // не ждёт полного слива кольца на стыке и проходит его без остановки.
    // Ждём только если все слоты заняты: это значит, что Core 1 не успевает
    // (кольцо тактов заполнено) и торопить его бессмысленно.
    for (;;) {
        bool queued = false;

        // Публикация кадра атомарна относительно Core 1: индексы и счётчик
        // меняются в одной критической секции, недописанный слот другой ядро
        // не увидит.
        portENTER_CRITICAL(&xferMux);
        if (frameCount < FRAME_QUEUE_SIZE) {
            InputSegment &slot = frameQueue[frameHead];
            slot.x       = newX;
            slot.y       = newY;
            slot.z       = newZ;
            slot.accel   = accelMM_Sec2;
            slot.v_frame = feedRateMM_Sec; // мм/с — то, что нужно физике Core 1
            slot.v_start = vStartMM_Sec;   // готовые скорости стыка от Look-Ahead Python
            slot.v_end   = vEndMM_Sec;

            frameHead = (frameHead + 1) % FRAME_QUEUE_SIZE;
            frameCount++;

            // Флаги движения поднимаем в той же секции, что и запись кадра:
            // Core 1 не увидит «кадр без движения» или наоборот.
            isPlannerBusy  = true;
            isVectorMoving = true;
            queued = true;
        }
        portEXIT_CRITICAL(&xferMux);

        if (queued) return true;

        // Очередь заполнена. Остановка/авария прерывает ожидание, иначе
        // Core 0 не должен висеть дольше одного кадра генерации.
        if (motionStopRequested || alarmRequested) return false;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ============================================================================
// СТРУКТУРНЫЙ БЛОК: ЯДРО ПЛАНИРОВЩИКА ДВИЖЕНИЯ ЧПУ (Core 1)
// ============================================================================

void motionTask(void * parameter) {
    Serial.printf("SYSTEM: motionTask started on Core %d\n", xPortGetCoreID());

    // Локальные переменные потока для отслеживания шагов внутри кадра
    long localStepsX = 0;
    long localStepsY = 0;
    long localStepsZ = 0;

    while (true) {
        // Остановка/авария: не начинаем новый кадр и не трогаем буфер.
        // Флаг останова проверяем повторно под xferMux: Core 0 мог уже снять
        // STOP и опубликовать новый кадр, пока мы читали флаг.
        if (motionStopRequested || alarmRequested) {
            bool stillStopped;
            portENTER_CRITICAL(&xferMux);
            stillStopped = (motionStopRequested || alarmRequested);
            if (stillStopped) {
                clearFrameQueueLocked();
                isMotionGenerating = false;
                isVectorMoving = false;
                isPlannerBusy = false;
            }
            portEXIT_CRITICAL(&xferMux);

            if (stillStopped) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            continue;
        }

        // --- 4.1.1. ОПРОС И БЛОКИРОВКА МЕЖЪЯДЕРНОЙ ОЧЕРЕДИ ---
        // Снимок кадра и handshake выполняются под xferMux: Core 0 не сможет
        // сбросить hasNewData между проверкой и взводом isMotionGenerating.
        float nX = 0.0f, nY = 0.0f, nZ = 0.0f, accel = 0.0f;
        float v_max = 0.0f, v_start = 0.0f, v_end = 0.0f;
        bool hasSegment = false;

        portENTER_CRITICAL(&xferMux);
        if (frameCount > 0) {
            const InputSegment &slot = frameQueue[frameTail];
            nX = slot.x;
            nY = slot.y;
            nZ = slot.z;
            accel = slot.accel;
            v_max   = slot.v_frame; // Ограничение подачи кадра от CAM
            v_start = slot.v_start; // Скорость входа от Look-Ahead Python
            v_end   = slot.v_end;   // Скорость выхода на стыке угла от Look-Ahead Python

            frameTail = (frameTail + 1) % FRAME_QUEUE_SIZE;
            frameCount--;

            // Освободили слот для Core 0 и сразу занимаемся этим кадром.
            isMotionGenerating = true;
            hasSegment = true;
        } else if (bufferTail == bufferHead) {
            // Очередь пуста и кольцо пусто. Только в этот момент движение
            // действительно закончилось: isPlannerBusy = false — это «станок
            // стоит и шагов больше не будет», а не «кадр отдан в генерацию».
            isPlannerBusy = false;
        }
        portEXIT_CRITICAL(&xferMux);

        if (!hasSegment) {
            vTaskDelay(pdMS_TO_TICKS(1)); // Неблокирующее ожидание FreeRTOS (1 мс)
            continue;
        }

        // STOP/ALARM могли прийти сразу после того, как кадр взят из очереди.
        if (motionStopRequested || alarmRequested) {
            portENTER_CRITICAL(&xferMux);
            isMotionGenerating = false; // отпускаем requestMotionStop()
            // Если Core 0 уже успел снять STOP и положить новый кадр,
            // его флаги не трогаем — обработаем на следующей итерации.
            if (frameCount == 0) {
                isVectorMoving = false;
                isPlannerBusy = false;
            }
            portEXIT_CRITICAL(&xferMux);
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        // --- 4.1.3. ОПТИЧЕСКАЯ ЛАЗЕРНАЯ КОМПЕНСАЦИЯ ПРОВИСА ФЕРМЫ ---
        CorrectionPoint corr = getCorrection(nX);
        float realMachineY = nY + corr.y_offset;
        float realMachineZ = nZ + corr.z_offset;

        // --- 4.1.4. ПЕРЕВОД МИЛЛИМЕТРОВ В ЦЕЛОЧИСЛЕННЫЕ НАПРАВЛЕНИЯ ЧПУ ---
        // Рассчитываем точные целевые шаги, которые требует CAM-модель
        long targetStepsX = nX * cfg.stepsPerMmX;
        long targetStepsY = realMachineY * cfg.stepsPerMmY;
        long targetStepsZ = realMachineZ * cfg.stepsPerMmZ;

        // Вычисляем чистые дельты перемещения строго от идеальной математической базы
        // прошлого кадра. Снимок тройки берём под ringMux, чтобы Core 0 не изменил
        // plannerSteps* между осями (syncPlannerToActual/стоп).
        long baseStepsX, baseStepsY, baseStepsZ;
        portENTER_CRITICAL(&ringMux);
        baseStepsX = plannerStepsX;
        baseStepsY = plannerStepsY;
        baseStepsZ = plannerStepsZ;
        portEXIT_CRITICAL(&ringMux);

        long totalDeltaX = targetStepsX - baseStepsX;
        long totalDeltaY = targetStepsY - baseStepsY;
        long totalDeltaZ = targetStepsZ - baseStepsZ;

        long absDeltaX = labs(totalDeltaX);
        long absDeltaY = labs(totalDeltaY);
        long absDeltaZ = labs(totalDeltaZ);

        // Находим ведущую ось (максимальное дискретное перемещение в импульсах внутри этого кадра)
        long maxSegmentSteps = absDeltaX;
        if (absDeltaY > maxSegmentSteps) maxSegmentSteps = absDeltaY;
        if (absDeltaZ > maxSegmentSteps) maxSegmentSteps = absDeltaZ;

        // Защита от пустых кадров (команды шпинделя, охлаждения и т.д.)
        if (maxSegmentSteps == 0) {
            isMotionGenerating = false;
            continue; 
        }

        // ====================================================================
        // СТРУКТУРНЫЙ БЛОК: MULTI-SEGMENT VECTOR SPLITTING & VALIDATION (Core 1)
        // ====================================================================

        // Идеальные длины участков в мм
        float s_accel_ideal = (v_max * v_max - v_start * v_start) / (2.0f * accel);
        float s_decel_ideal = (v_max * v_max - v_end * v_end) / (2.0f * accel);
        
        float dx_mm = (float)totalDeltaX / cfg.stepsPerMmX;
        float dy_mm = (float)totalDeltaY / cfg.stepsPerMmY;
        float dz_mm = (float)totalDeltaZ / cfg.stepsPerMmZ;
        float totalLength_mm = sqrt(dx_mm * dx_mm + dy_mm * dy_mm + dz_mm * dz_mm);

        // Объявляем массивы целочисленных дельт для каждого из трех сегментов
        long dX[3] = {0, 0, 0}; // [0] - разгон, [1] - марш, [2] - торможение
        long dY[3] = {0, 0, 0};
        long dZ[3] = {0, 0, 0};

        // Процентные доли сегментов в общем векторе
        float pct_accel = 0.0f;
        float pct_cruise = 0.0f;

        if (s_accel_ideal + s_decel_ideal > totalLength_mm) {
            // --- СЦЕНАРИЙ А: КОРОТКИЙ ОТРЕЗК (ТРЕУГОЛЬНИК, 2 сегмента) ---
            float v_peak = sqrt((2.0f * accel * totalLength_mm + v_start * v_start + v_end * v_end) / 2.0f);
            v_max = v_peak; 

            float s_accel_actual = (v_peak * v_peak - v_start * v_start) / (2.0f * accel);
            pct_accel = s_accel_actual / totalLength_mm;
            pct_cruise = 0.0f; // Марша нет

            // Целочисленные дельты разгона
            dX[0] = (long)(totalDeltaX * pct_accel);
            dY[0] = (long)(totalDeltaY * pct_accel);
            dZ[0] = (long)(totalDeltaZ * pct_accel);

            // Марш равен нулю
            dX[1] = 0; dY[1] = 0; dZ[1] = 0;

            // Торможение — это строго остаток от исходного вектора
            dX[2] = totalDeltaX - dX[0];
            dY[2] = totalDeltaY - dY[0];
            dZ[2] = totalDeltaZ - dZ[0];
        } 
        else {
            // --- СЦЕНАРИЙ Б: КЛАССИЧЕСКАЯ ТРАПЕЦИЯ (3 сегмента) ---
            pct_accel = s_accel_ideal / totalLength_mm;
            float s_cruise_ideal = totalLength_mm - (s_accel_ideal + s_decel_ideal);
            pct_cruise = s_cruise_ideal / totalLength_mm;

            // Дельты разгона
            dX[0] = (long)(totalDeltaX * pct_accel);
            dY[0] = (long)(totalDeltaY * pct_accel);
            dZ[0] = (long)(totalDeltaZ * pct_accel);

            // Дельты маршевого круиза
            dX[1] = (long)(totalDeltaX * pct_cruise);
            dY[1] = (long)(totalDeltaY * pct_cruise);
            dZ[1] = (long)(totalDeltaZ * pct_cruise);

            // Дельты торможения — строго вычитанием из общего баланса, поглощая округления float
            dX[2] = totalDeltaX - dX[0] - dX[1];
            dY[2] = totalDeltaY - dY[0] - dY[1];
            dZ[2] = totalDeltaZ - dZ[0] - dZ[1];
        }

        // --- 4.2.3. ЖЕСТКАЯ ЦЕЛОЧИСЛЕННАЯ ВАЛИДАЦИЯ БАЛАНСА ШАГОВ ---
        // Проверяем, что сумма всех дельт сегментов математически тождественна общему вектору
        // (Это защита на уровне компилятора. Если баланс сошелся — мы гарантированно не пропустим микрошаг)
        long checkSumX = dX[0] + dX[1] + dX[2];
        long checkSumY = dY[0] + dY[1] + dY[2];
        long checkSumZ = dZ[0] + dZ[1] + dZ[2];

        if (checkSumX != totalDeltaX || checkSumY != totalDeltaY || checkSumZ != totalDeltaZ) {
            // Если из-за краевых эффектов округления где-то потерялся 1 микрошаг,
            // принудительно корректируем его в сегменте торможения [2]
            if (checkSumX != totalDeltaX) dX[2] += (totalDeltaX - checkSumX);
            if (checkSumY != totalDeltaY) dY[2] += (totalDeltaY - checkSumY);
            if (checkSumZ != totalDeltaZ) dZ[2] += (totalDeltaZ - checkSumZ);
        }

        // ====================================================================
        // ТЕПЕРЬ У НАС ЕСТЬ ЖЕСТКИЕ ЦЕЛОЧИСЛЕННЫЕ ЗАДАЧИ ДЛЯ БРЕЗЕНХЕМА:
        // Сегмент 0 (Разгон):     сделать шаги dX[0], dY[0], dZ[0]
        // Сегмент 1 (Марш):       сделать шаги dX[1], dY[1], dZ[1]
        // Сегмент 2 (Торможение): сделать шаги dX[2], dY[2], dZ[2]
        // СУММА ДЕЛЬТ СЕГМЕНТОВ ВСЕГДА СТРОГО РАВНА ИСХОДНОМУ ВЕКТОРУ!
        // ====================================================================


        // ====================================================================
        // СТРУКТУРНЫЙ БЛОК: ЧИСТАЯ ПОТАКТОВАЯ ОТРАБОТКА ПОДВЕКТОРОВ (Core 1)
        // ====================================================================
        // Базовая ЧПУ-сетка планирования: одна ячейка кольца = 10 мкс (100 кГц).
        // Выводится из CNC_TIMER_TICK_US (5 мкс × 2 фазы), чтобы частота таймера
        // и математика профиля не разъезжались.
        const float BASE_CNC_FREQ = CNC_CELL_HZ;

        // Текущая физическая скорость вектора в начале кадра (мм/сек)
        float currentVelocity = v_start; 

        localStepsX = baseStepsX;
        localStepsY = baseStepsY;
        localStepsZ = baseStepsZ;

        bool frameAborted = false;

        // Поочередно прогоняем 3 рассчитанных подвектора (Разгон, Марш, Торможение)
        for (uint8_t segIdx = 0; segIdx < 3; segIdx++) {
            
            long segDeltaX = dX[segIdx];
            long segDeltaY = dY[segIdx];
            long segDeltaZ = dZ[segIdx];

            long segAbsX = labs(segDeltaX);
            long segAbsY = labs(segDeltaY);
            long segAbsZ = labs(segDeltaZ);

            // Определяем, какая физическая ось является Мастер-Осью для текущего подвектора
            long totalSegSteps = segAbsX;
            float masterStepsPerMm = cfg.stepsPerMmX; // Цена шага Мастер-Оси

            if (segAbsY > totalSegSteps) {
                totalSegSteps = segAbsY;
                masterStepsPerMm = cfg.stepsPerMmY;
            }
            if (segAbsZ > totalSegSteps) {
                totalSegSteps = segAbsZ;
                masterStepsPerMm = cfg.stepsPerMmZ;
            }

            // Если сегмент пустой (например, марш в треугольнике), просто идем дальше
            if (totalSegSteps == 0) continue;

            // Инициализация накопителей ошибки Брезенхема для этого подвектора
            long errX = totalSegSteps / 2;
            long errY = totalSegSteps / 2;
            long errZ = totalSegSteps / 2;

            uint8_t dirBitX = (segDeltaX >= 0) ? 1 : 0;
            uint8_t dirBitY = (segDeltaY >= 0) ? 1 : 0;
            uint8_t dirBitZ = (segDeltaZ >= 0) ? 1 : 0;

            // --- ХОДОВОЙ ЦИКЛ ГЕНЕРАЦИИ ТАКТОВ ПОДВЕКТОРА ---
            // Цикл итерируется строго по физическому количеству импульсов, которые нужно выдать
            
            // maxSegmentSteps — полный объем шагов ведущей оси на ВСЕМ макро-кадре (для Z это 2004, для Y в кадре 5 это 10352)
            // totalLength_mm — полная пространственная 3D длина всего кадра в миллиметрах (5.01 или 100.0)
            float globalMasterStepsPerMm = (float)maxSegmentSteps / totalLength_mm;
            if (globalMasterStepsPerMm < 0.001f) globalMasterStepsPerMm = 1.0f; // Защита от деления на ноль

            for (long step = 0; step < totalSegSteps; step++) {
                if (motionStopRequested || alarmRequested) {
                    frameAborted = true;
                    break;
                }

                // --- 1. ФИЗИЧЕСКАЯ ЗАМОРОЗКА ГЕОМЕТРИИ DDA НА ПАУЗЕ ---
                // Виртуальный лимит уже погашен до нуля: генератор стоит на
                // границе шага, накопители Брезенхема не сдвигаются, в кольцо
                // ничего не пишется. Уже выданные ячейки доигрывает ISR.
                if (currentMachineState == STATE_HOLD && virtualSpeedLimit <= 0.001f) {
                    isGeneratorSuspendedOnHold = true;

                    // Генератор честно не пишет: requestMotionStop() не должен
                    // ждать его 100 мс при HOLD -> IDLE/ALARM и ругаться
                    // таймаутом. Кольцо в этом состоянии чистить безопасно.
                    isMotionGenerating = false;

                    // Крутимся в легковесном цикле, пока удерживается пауза.
                    // Математика Брезенхема/DDA замерла строго на текущем шаге.
                    while (currentMachineState == STATE_HOLD) {
                        if (motionStopRequested || alarmRequested) break;
                        vTaskDelay(pdMS_TO_TICKS(5)); 
                    }

                    if (motionStopRequested || alarmRequested) {
                        frameAborted = true;
                        break;
                    }

                    // RESUME: снова пишем такты. Индекс DDA, накопленная скорость
                    // и накопители Брезенхема не сброшены, поэтому кадр
                    // продолжается с той же точки — итоговая координата от паузы
                    // не зависит, кольцо не очищается.
                    isGeneratorSuspendedOnHold = false;
                    isMotionGenerating = true;
                }

                if (motionStopRequested || alarmRequested) {
                    frameAborted = true;
                    break;
                }
                // =====================================================

                // ВЫЧИСЛЯЕМ ЭФФЕКТИВНУЮ СКОРОСТЬ ВЕКТОРНОГО ШАГА:
                // Берем текущую скорость из G-код трапеции (currentVelocity) 
                // и искусственно зажимаем её сверху плавным лимитом паузы virtualSpeedLimit
                float effectiveVelocity = currentVelocity;
                if (effectiveVelocity > virtualSpeedLimit) {
                    effectiveVelocity = virtualSpeedLimit;
                }
                // Нижний предохранитель для стабильного наката на микро-скоростях
                if (effectiveVelocity < MIN_FRAME_SPEED_MM_S) {
                    effectiveVelocity = MIN_FRAME_SPEED_MM_S;
                }
            
                // Считаем частоту шагов на основе эффективной скорости
                float effectiveMasterFrequency = effectiveVelocity * globalMasterStepsPerMm;
                if (effectiveMasterFrequency < 1.0f) effectiveMasterFrequency = 1.0f;

                // OVERRIDE ограничен 0,1…2, но при 0 делитель дал бы inf → UB в
                // приведении к uint32_t. Страхуемся явно.
                float overrideFactor = smoothedFeedrateOverride;
                if (!(overrideFactor > 0.01f)) overrideFactor = 0.01f;
            
                // РАСЧЕТ ЗАДЕРЖКИ ТАЙМЕРА:
                // Но теперь частота плавно падает до нуля и плавно растет при старте за счет effectiveMasterFrequency!
                // uint32_t, а не uint16_t: при 1 Гц задержка = 100 000 ячеек и
                // раньше переполняла 16 бит (100000 mod 65536 = 34464).
                uint32_t ticksPerStep = (uint32_t)( BASE_CNC_FREQ / (effectiveMasterFrequency * overrideFactor) );
            
                if (ticksPerStep < 1) ticksPerStep = 1;
                // P1-4: верхний предохранитель. Без него подача, близкая к нулю,
                // растягивала один шаг на 100 000 ячеек = 1 с («ползок»).
                if (ticksPerStep > MAX_STEP_TICK_CELLS) ticksPerStep = MAX_STEP_TICK_CELLS;
            
            
                // 3. РАСЧЕТ РЕАЛЬНОГО ФИЗИЧЕСКОГО ВРЕМЕНИ ДЛИТЕЛЬНОСТИ ТЕКУЩЕГО ШАГА
                // Переводим полученные ячейки обратно во float-секунды для физических формул разгона/торможения.
                // Пример: 10 ячеек / 100 000 Гц = 0.0001 секунды (100 микросекунд) длится этот конкретный шаг.
                float stepDurationSec = (float)ticksPerStep / BASE_CNC_FREQ;


                // ====================================================================
                // ДИНАМИЧЕСКИЙ ПЕРЕСЧЕТ СКОРОСТИ И АДАПТИВНЫЙ БАРЬЕР БЕЗОПАСНОСТИ
                // ====================================================================

                // --- СЦЕНАРИЙ А: СЕГМЕНТ РАЗГОНА (segIdx == 0) ---
                if (segIdx == 0) {
                    // Вычисляем теоретический прирост скорости за время текущего шага по формуле: V = a * t
                    // Пример: 500 мм/сек2 (ускорение) * 0.0001 сек (время шага) = 0.05 мм/сек прироста
                    float deltaV = accel * stepDurationSec;
                
                    // РАСЧЕТ ПРОГРАММНОГО БАРЬЕРА БЕЗОПАСНОСТИ (Anti-Jerk Guard):
                    // Прирост скорости за ОДИН дискретный шаг не имеет права превысить половину 
                    // от всей разницы между скоростью входа в кадр и маршевой (круизной) скоростью.
                    // Это защищает тяжелый портал от ударного срыва роторов на сверхвысоких частотах таймера.
                    float maxSafeDeltaV = (v_max - v_start) / 2.0f;

                    // Нижний барьер-предохранитель, чтобы рампа не застряла на околонулевых скоростях
                    if (maxSafeDeltaV < 0.1f) maxSafeDeltaV = 0.1f;
                
                    // Если математический расчет выдал слишком агрессивный прыжок скорости — зажимаем его по лимиту
                    if (deltaV > maxSafeDeltaV) deltaV = maxSafeDeltaV;
                
                    // Физически наращиваем текущую скорость вектора на безопасную величину дельты
                    currentVelocity += deltaV;
                
                    // Срезаем скорость по верхней границе маршевой подачи кадра (ограничение G-кода / CAM)
                    if (currentVelocity > v_max) currentVelocity = v_max;
                }

                // --- СЦЕНАРИЙ Б: СЕГМЕНТ ТОРМОЖЕНИЯ (segIdx == 2) ---
                else if (segIdx == 2) {
                    // Вычисляем теоретическое падение скорости за время шага: V = a * t
                    float deltaV = accel * stepDurationSec;
                
                    // Симметричный барьер безопасности для торможения перед стыком со следующим кадром.
                    // Защищает от мгновенного удара и «втыкания» в угол при переходе на новую траекторию.
                    float maxSafeDeltaV = (v_max - v_end) / 2.0f;
                    if (maxSafeDeltaV < 0.1f) maxSafeDeltaV = 0.1f;
                    if (deltaV > maxSafeDeltaV) deltaV = maxSafeDeltaV;
                
                    // Физически гасим текущую скорость вектора
                    currentVelocity -= deltaV;
                
                    // Срезаем скорость по нижней границе, заданной Python Look-Ahead планировщиком (скорость выхода)
                    if (currentVelocity < v_end) currentVelocity = v_end;
                }

                // --- СЦЕНАРИЙ В: СЕГМЕНТ МАРШЕВОГО КРУИЗА (segIdx == 1) ---
                else {
                    // На маршевом участке скорость станка строго стабильна и равна максимальной подаче кадра
                    currentVelocity = v_max;
                }

                sharedCurrentVelocity = currentVelocity; 

                // ====================================================================
                // БЛОК 3: ИЗОЛИРОВАННЫЙ ЦЕЛОЧИСЛЕННЫЙ АЛГОРИТМ БРЕЗЕНХЕМА (Core 1)
                // ====================================================================

                // Объявляем локальные битовые флаги-триггеры для этого конкретного шага.
                // 1 = в этот такт нужно физически выдать импульс STEP, 0 = ось стоит на месте.
                uint8_t issueX = 0;
                uint8_t issueY = 0;
                uint8_t issueZ = 0;

                // --- ЛИНЕЙНАЯ ИНТЕРПОЛЯЦИЯ ОСИ X ---
                // Вычитаем из накопителя ошибки Брезенхема (errX) модуль перемещения текущей оси (segAbsX)
                errX -= segAbsX;

                // Если ошибка упала ниже нуля — пороговое значение пересечено, пора делать физический шаг!
                if (errX < 0) {
                    errX += totalSegSteps;       // Возвращаем накопитель ошибки в рабочую зону ведущей оси
                    issueX = 1;                  // Взводим флаг генерации импульса STEP для прерывания
                    // Обновляем виртуальную координату планировщика в ОЗУ в зависимости от знака направления
                    localStepsX += (dirBitX == 1) ? 1 : -1; 
                }
            
                // --- ЛИНЕЙНАЯ ИНТЕРПОЛЯЦИЯ ОСИ Y ---
                // Математика полностью идентична оси X, но выполняется параллельно на независимых регистрах
                errY -= segAbsY;
                if (errY < 0) {
                    errY += totalSegSteps;
                    issueY = 1;
                    localStepsY += (dirBitY == 1) ? 1 : -1;
                }
            
                // --- ЛИНЕЙНАЯ ИНТЕРПОЛЯЦИЯ ОСИ Z ---
                errZ -= segAbsZ;
                if (errZ < 0) {
                    errZ += totalSegSteps;
                    issueZ = 1;
                    localStepsZ += (dirBitZ == 1) ? 1 : -1;
                }

                

                // ====================================================================
                // БЛОК 4: ЗАПОЛНЕНИЕ БУФЕРА СЕТКИ ВРЕМЕНИ (Внутренний цикл паузы)
                // ====================================================================
                        
                // Цикл крутится ровно ticksPerStep раз, нарезая 10-микросекундные кванты времени.
                // Если ticksPerStep равен 10, то мы запишем 1 такт с импульсом и 9 тактов чистой паузы.
                for (uint32_t t = 0; t < ticksPerStep; t++) {
                
                    // 1. КОНТРОЛЬ ПЕРЕПОЛНЕНИЯ И ЗАЩИТА ОЧЕРЕДИ FREE ROOT
                    // Вычисляем виртуальный следующий индекс для головы кольцевого буфера
                    uint32_t nextHead = (bufferHead + 1) % STEP_BUFFER_SIZE;
                
                    // Если голова догнала хвост (буфер ОЗУ забит на все 40.96 мс вперед)
                    while (nextHead == bufferTail) {
                        if (motionStopRequested || alarmRequested) {
                            frameAborted = true;
                            break;
                        }
                        // Уступаем квант времени операционной системе FreeRTOS на 1 мс.
                        // Прерывание таймера на Core 1 продолжит читать буфер и освободит место.
                        vTaskDelay(pdMS_TO_TICKS(1)); 
                    }

                    if (frameAborted) break;
                
                    // 2. ИНИЦИАЛИЗАЦИЯ ЧИСТОЙ БИНАРНОЙ КОМАНДЫ ВРЕМЕНИ
                    // Создаем пустую микроструктуру (по умолчанию все шаги равны 0, valid = 0)
                    StepCmd cmd = { 0, 0, 0, 0, 0, 0, 0 };
                
                    // 3. ПОТАКТОВОЕ ФОРМИРОВАНИЕ ПЕРЕДНЕГО ФРОНТА (Только в первый квант)
                    // Физический импульс шага и биты направлений закладываются СТРОГО в самый первый такт (t == 0)
                    if (t == 0) {
                        // Подмешиваем флаги и направления, которые рассчитал Брезенхем в Блоке 3
                        if (issueX) { cmd.stepX = 1; cmd.dirX = dirBitX; }
                        if (issueY) { cmd.stepY = 1; cmd.dirY = dirBitY; }
                        if (issueZ) { cmd.stepZ = 1; cmd.dirZ = dirBitZ; }
                    }
                    // Во всех остальных итерациях (при t > 0) поля cmd.stepX/Y/Z остаются равными 0.
                    // Это и формирует необходимую временную паузу между шагами моторов!
                
                    // Взводим флаг готовности ячейки. Аппаратный таймер теперь имеет право её прочесть.
                    cmd.valid = 1;
                
                    // Физически записываем структуру в оперативную память кольцевого буфера
                    stepRingBuffer[bufferHead] = cmd;

                    // Барьер: содержимое ячейки обязано стать видимым до сдвига
                    // головы, иначе ISR может прочитать недописанный такт.
                    __sync_synchronize();

                    // Сдвигаем индекс головы кольца вперед
                    bufferHead = nextHead;
                }
            
            } // Конец цикла по шагам текущего подвектора
        } // Конец цикла по 3-м подвекторам

        // Остановка во время генерации: кадр отбрасывается, plannerSteps уже
        // синхронизирован с фактическими счётчиками в requestMotionStop().
        if (frameAborted || motionStopRequested || alarmRequested) {
            portENTER_CRITICAL(&xferMux);
            isMotionGenerating = false; // отпускаем requestMotionStop()
            // Новый кадр мог быть положен в очередь сразу после снятия STOP —
            // не затираем его флаги и обработаем на следующей итерации.
            if (frameCount == 0) {
                isVectorMoving = false;
                isPlannerBusy = false;
            }
            portEXIT_CRITICAL(&xferMux);
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        // Фиксируем эталонную базу и только после этого разрешаем Core 0
        // считать генератор остановленным (requestMotionStop).
        portENTER_CRITICAL(&ringMux);
        plannerStepsX = targetStepsX;
        plannerStepsY = targetStepsY;
        plannerStepsZ = targetStepsZ;
        isVectorMoving = true; // Жестко взводим флаг физического старта движения!
        portEXIT_CRITICAL(&ringMux);
        isMotionGenerating = false;

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ============================================================================
// СТРУКТУРНЫЙ БЛОК: ВЫСОКОСКОРОСТНОЕ ДВУХФАЗНОЕ ПРЕРЫВАНИЕ ТАЙМЕРА
// ============================================================================

// Аварийная ветка ISR: только аппаратные действия и флаги.
// Никаких changeState(), Preferences, Serial и String — всё это делает loop().
static void IRAM_ATTR emergencyStopFromISR() {
    motionStopRequested = true;
    alarmRequested = true;

    digitalWrite(MOTORS_ENABLE, HIGH); // Немедленно обесточить драйверы Leadshine
    digitalWrite(RELE_SPINDLE, LOW);
    digitalWrite(RELE_COOLANT, LOW);

    digitalWrite(X_STEP_PIN, LOW);
    digitalWrite(Y_STEP_PIN, LOW);
    digitalWrite(Z_STEP_PIN, LOW);

    // Best-effort: выбрасываем очередь. Генератор увидит motionStopRequested
    // и прекратит запись; точную очистку выполнит requestMotionStop() в loop().
    portENTER_CRITICAL_ISR(&ringMux);
    bufferTail = bufferHead;
    portEXIT_CRITICAL_ISR(&ringMux);

    isVectorMoving = false;
    isPlannerBusy = false;
    // Кадры, ещё не взятые генератором, отменяем прямо в прерывании.
    portENTER_CRITICAL_ISR(&xferMux);
    frameHead  = 0;
    frameTail  = 0;
    frameCount = 0;
    portEXIT_CRITICAL_ISR(&xferMux);
    timerPhaseHigh = true;
}

void IRAM_ATTR onTimerInterrupt() {
    // Авария уже зафиксирована: драйверы обесточены, шаги заблокированы.
    // Повторно не дёргаем пины каждые 5 мкс, пока supervisorTask не обработает флаг.
    if (alarmRequested) {
        digitalWrite(X_STEP_PIN, LOW);
        digitalWrite(Y_STEP_PIN, LOW);
        digitalWrite(Z_STEP_PIN, LOW);
        timerPhaseHigh = true;
        return;
    }

    // Проверяем критические аварии, которые должны сработать ВСЕГДА,
    // в том числе когда выдача шагов остановлена (IDLE/STOP/ALARM).
    bool criticalAlarm = (safetyEstopActive() || safetyMotorAlarmActive());
    
    // Линию концевиков подмешиваем к аварии ТОЛЬКО если сейчас не идет процедура хоуминга
    if (!isHomingActive && safetyEndstopsActive()) {
        criticalAlarm = true;
    }

    if (criticalAlarm) {
        emergencyStopFromISR();
        return;
    }

    // После STOP/ALARM шаги не выдаются вообще.
    if (motionStopRequested) {
        digitalWrite(X_STEP_PIN, LOW);
        digitalWrite(Y_STEP_PIN, LOW);
        digitalWrite(Z_STEP_PIN, LOW);
        timerPhaseHigh = true;
        return;
    }

    // --- 5.2. ФАЗА 2: ЧЕТНЫЙ ТАКТ ТАЙМЕРА (СПАД ИМПУЛЬСА STEP) ---
    if (!timerPhaseHigh) {
        // Вслепую, за наносекунды опускаем все пины STEP в LOW
        // Процессор не висит в delayMicroseconds, освобождая 20% мощности ядра!
        digitalWrite(X_STEP_PIN, LOW);
        digitalWrite(Y_STEP_PIN, LOW);
        digitalWrite(Z_STEP_PIN, LOW);
        
        timerPhaseHigh = true; // Переключаем триггер на следующую фазу HIGH
        return; // Мгновенно выходим из прерывания
    }

    // --- 5.3. ФАЗА 1: НЕЧЕТНЫЙ ТАКТ ТАЙМЕРА (ФОРМИРОВАНИЕ ФРОНТА STEP) ---
    // Мы зашли сюда строго раз в 5 микросекунд (200 кГц); ячейка кольца = 2 тика.
    timerPhaseHigh = false; // Следующий тик таймера гарантированно пойдет в фазу LOW

    // Если в кольцевом буфере есть готовые такты времени от планировщика.
    // ringMux нужен только против Core 0 (homing/stop): motionTask пишет
    // кольцо на этом же ядре, поэтому отдельная защита от него не требуется.
    portENTER_CRITICAL_ISR(&ringMux);
    if (bufferTail != bufferHead) {
        StepCmd &cmd = stepRingBuffer[bufferTail];
        
        if (cmd.valid) {
            bool stepIssued = false;

            // --- Ось X ---
            if (cmd.stepX) {
                digitalWrite(X_DIR_PIN, cmd.dirX ? HIGH : LOW);
                digitalWrite(X_STEP_PIN, HIGH);
                currentStepsX += cmd.dirX ? 1 : -1; // Честный целочисленный инкремент ЧПУ
                stepIssued = true;
            }
            
            // --- Ось Y ---
            if (cmd.stepY) {
                digitalWrite(Y_DIR_PIN, cmd.dirY ? HIGH : LOW);
                digitalWrite(Y_STEP_PIN, HIGH);
                currentStepsY += cmd.dirY ? 1 : -1;
                stepIssued = true;
            }
            
            // --- Ось Z ---
            if (cmd.stepZ) {
                digitalWrite(Z_DIR_PIN, cmd.dirZ ? HIGH : LOW);
                digitalWrite(Z_STEP_PIN, HIGH);
                currentStepsZ += cmd.dirZ ? 1 : -1;
                stepIssued = true;
            }

            // Теперь эта ячейка считается пустой и планировщик на Ядре 1 сможет записать сюда новый такт
            cmd.valid = 0; 

            // Ячейка (10 мкс) успешно отработана — сдвигаем указатель чтения кольца!
            bufferTail = (bufferTail + 1) % STEP_BUFFER_SIZE;
        } else {
            // Защита от битых или неполных ячеек
            bufferTail = (bufferTail + 1) % STEP_BUFFER_SIZE;
        }
    } else {
        // Если буфер пуст (станок приехал в цель и замер в IDLE)
        isVectorMoving = false; 
    }
    portEXIT_CRITICAL_ISR(&ringMux);
}

// ============================================================================
// СТРУКТУРНЫЙ БЛОК: АВТОМАТИЧЕСКИЙ ПОИСК БАЗ СТАНКА (HOMING)
// ============================================================================

// Период одной ячейки кольца, мкс (100 кГц). Нужен для расчёта таймаутов хоуминга.
static const float HOMING_TICK_US = CNC_CELL_US;
// 50 ячеек на шаг = 0,5 мс -> ~2 кГц. Один шаг = одна пачка ячеек.
static const uint16_t HOMING_DELAY_TICKS = 50;
// Пауза после остановки оси для гашения инерции фермы.
static const unsigned long HOMING_SETTLE_MS = 100;

// Проверка аварийных входов и запросов остановки во время хоуминга.
static HomingResult homingCheckInputs() {
    if (alarmRequested || safetyEstopActive() ||
        safetyMotorAlarmActive() || currentMachineState == STATE_ALARM) {
        return HOMING_FAILED;
    }
    if (motionStopRequested) return HOMING_ABORTED;
    return HOMING_OK;
}

// Ставит в кольцевой буфер один шаг нужной оси (пачку тиков ожидания).
// Возвращает HOMING_OK, если такт записан; иначе — причину остановки.
static HomingResult enqueueHomingStep(int stepPin, int dirBit) {
    // Core 0 — единственный producer кольца в режиме хоуминга, но ISR читает
    // кольцо на Core 1. Каждый такт публикуем под ringMux и проверяем место
    // для него отдельно (пачка из 50 тиков не должна переполнить буфер).
    for (uint16_t t = 0; t < HOMING_DELAY_TICKS; t++) {
        for (;;) {
            HomingResult st = homingCheckInputs();
            if (st != HOMING_OK) return st;

            bool hasRoom = false;
            portENTER_CRITICAL(&ringMux);
            uint32_t nextHead = (bufferHead + 1) % STEP_BUFFER_SIZE;
            if (nextHead != bufferTail) {
                StepCmd cmd = {0, 0, 0, 0, 0, 0, 0};
                if (t == 0) {
                    if (stepPin == X_STEP_PIN) { cmd.stepX = 1; cmd.dirX = dirBit; }
                    if (stepPin == Y_STEP_PIN) { cmd.stepY = 1; cmd.dirY = dirBit; }
                    if (stepPin == Z_STEP_PIN) { cmd.stepZ = 1; cmd.dirZ = dirBit; }
                }
                cmd.valid = 1;
                stepRingBuffer[bufferHead] = cmd;
                bufferHead = nextHead;
                hasRoom = true;
            }
            portEXIT_CRITICAL(&ringMux);

            if (hasRoom) break;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    return HOMING_OK;
}

// Ждёт слива кольцевого буфера, контролируя аварии и таймаут.
static HomingResult waitHomingBufferDrain(unsigned long timeoutMs) {
    unsigned long start = millis();
    while (bufferTail != bufferHead) {
        HomingResult st = homingCheckInputs();
        if (st != HOMING_OK) return st;
        if (millis() - start > timeoutMs) return HOMING_FAILED;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return HOMING_OK;
}

static float homingAxisTravelMm(int stepPin) {
    if (stepPin == X_STEP_PIN) return fabsf(cfg.maxX - cfg.minX);
    if (stepPin == Y_STEP_PIN) return fabsf(cfg.maxY - cfg.minY);
    return fabsf(cfg.maxZ - cfg.minZ);
}

static float homingAxisStepsPerMm(int stepPin) {
    float spm = cfg.stepsPerMmZ;
    if (stepPin == X_STEP_PIN) spm = cfg.stepsPerMmX;
    else if (stepPin == Y_STEP_PIN) spm = cfg.stepsPerMmY;
    return (spm > 0.001f) ? spm : 100.0f;
}

// Функция поиска базы для одной конкретной оси через кольцевой буфер времени.
// Всегда сбрасывает isHomingActive, чтобы ISR снова следил за концевиками.
HomingResult homeAxis(int stepPin, int dirPin, int dirSign, volatile long &axisSteps, int endstopPin, long pullOffSteps) {
    isHomingActive = true;

    // P1-6: концевики всех осей — одна NC-цепь. Разомкнутая цепь на входе значит,
    // что какая-то ось уже стоит на своём концевике: поиск этой оси «нашёл бы
    // базу» на первом же шаге. Отказываемся вместо ложной базы.
    if (digitalRead(endstopPin) == HIGH) {
        Serial.println("HOMING: ERROR - endstop chain is already open (another axis is on its endstop).");
        isHomingActive = false;
        return HOMING_FAILED;
    }

    // Ограничение хода и времени поиска: заведомо больше рабочего поля + запас.
    float travelMm = homingAxisTravelMm(stepPin);
    float stepsPerMm = homingAxisStepsPerMm(stepPin);
    long maxSearchSteps = (long)((travelMm + 100.0f) * stepsPerMm);
    if (maxSearchSteps < 1000) maxSearchSteps = 1000;

    const float msPerStep = (float)HOMING_DELAY_TICKS * HOMING_TICK_US / 1000.0f;
    unsigned long searchTimeoutMs = (unsigned long)((float)maxSearchSteps * msPerStep) + 10000UL;
    unsigned long drainTimeoutMs  = 10000UL;

    bool failed = false;
    bool aborted = false;

    // --- Шаг 1: едем в сторону датчика до физического срабатывания ---
    unsigned long searchStart = millis();
    long searchSteps = 0;
    while (digitalRead(endstopPin) == LOW) {
        HomingResult st = homingCheckInputs();
        if (st == HOMING_FAILED)  { failed = true; break; }
        if (st == HOMING_ABORTED) { aborted = true; break; }

        if (searchSteps >= maxSearchSteps || (millis() - searchStart) > searchTimeoutMs) {
            Serial.println("HOMING: ERROR - endstop not reached in time.");
            failed = true;
            break;
        }

        st = enqueueHomingStep(stepPin, (dirSign > 0) ? 1 : 0);
        if (st == HOMING_FAILED)  { failed = true; break; }
        if (st == HOMING_ABORTED) { aborted = true; break; }
        searchSteps++;
    }

    if (!failed && !aborted) {
        HomingResult st = waitHomingBufferDrain(drainTimeoutMs);
        if (st == HOMING_FAILED)  failed = true;
        if (st == HOMING_ABORTED) aborted = true;
    }

    if (!failed && !aborted) {
        vTaskDelay(pdMS_TO_TICKS(HOMING_SETTLE_MS));
        HomingResult st = homingCheckInputs();
        if (st == HOMING_FAILED)  failed = true;
        if (st == HOMING_ABORTED) aborted = true;
    }

    // --- Шаг 2: обязательный целочисленный отъезд назад (Pull-off) ---
    long pullOff = (pullOffSteps > 0) ? pullOffSteps : 0;
    if (!failed && !aborted) {
        unsigned long pullOffStart = millis();
        unsigned long pullOffTimeoutMs = (unsigned long)((float)pullOff * msPerStep) + 5000UL;

        for (long i = 0; i < pullOff; i++) {
            HomingResult st = homingCheckInputs();
            if (st == HOMING_FAILED)  { failed = true; break; }
            if (st == HOMING_ABORTED) { aborted = true; break; }

            if (millis() - pullOffStart > pullOffTimeoutMs) {
                Serial.println("HOMING: ERROR - pull-off timeout.");
                failed = true;
                break;
            }

            // Едем строго в противоположную сторону (разворачиваем dirSign)
            st = enqueueHomingStep(stepPin, (dirSign > 0) ? 0 : 1);
            if (st == HOMING_FAILED)  { failed = true; break; }
            if (st == HOMING_ABORTED) { aborted = true; break; }
        }

        if (!failed && !aborted) {
            HomingResult st = waitHomingBufferDrain(drainTimeoutMs);
            if (st == HOMING_FAILED)  failed = true;
            if (st == HOMING_ABORTED) aborted = true;
        }

        if (!failed && !aborted) {
            vTaskDelay(pdMS_TO_TICKS(HOMING_SETTLE_MS));
            // После отъезда концевик обязан отпустить. Если цепь осталась разомкнутой
            // (залипший/неподключённый датчик) — это авария, а не успешная база.
            if (digitalRead(endstopPin) == HIGH) {
                Serial.println("HOMING: ERROR - endstop still active after pull-off.");
                failed = true;
            }
        }
    }

    isHomingActive = false;

    if (failed) {
        requestMotionStop();
        return HOMING_FAILED;
    }
    if (aborted) {
        requestMotionStop();
        return HOMING_ABORTED;
    }

    // Жестко зануляем физический счетчик оси! База официально найдена.
    axisSteps = 0;
    return HOMING_OK;
}

// Главный последовательный автомат поиска баз ЧПУ (Вызывается из handleIdleCommands на Core 0)
void runFullHoming() {
    // P1-6: концевики X, Y и Z висят на одной NC-цепи, отдельного входа на ось
    // нет — так решено на этапе железа. Поэтому перед циклом цепь обязана быть
    // замкнутой: иначе какая-то ось уже на концевике, и её «база» была бы взята
    // без движения. Оператор снимает оси с концевиков вручную и повторяет HOME.
    if (digitalRead(ENDSTOPS_PIN) == HIGH) {
        const char* msg = "ERROR: Endstop chain is open before homing. An axis sits on its endstop: "
                          "move it off manually, check the wiring, then run HOME again.";
        Serial.println(msg);
        sendToWiFiClient(msg);
        return; // состояние не меняем: станок остаётся в IDLE
    }

    changeState(STATE_HOMING);
    // Переход мог быть отклонён (ALARM / запрет) — тогда хоуминг не начинаем.
    if (currentMachineState != STATE_HOMING) return;

    isHomed = false;
    isHomingActive = false;

    // Таймер мог быть остановлен предыдущим STOP/ALARM.
    if (!armMotionTimer()) {
        changeState(STATE_ALARM);
        return;
    }

    // Возвращает true, если ось можно двигать дальше.
    auto homeOneAxis = [](int stepPin, int dirPin, int dirSign, volatile long &steps,
                          long pullOff, const char* name) -> bool {
        HomingResult res = homeAxis(stepPin, dirPin, dirSign, steps, ENDSTOPS_PIN, pullOff);
        if (res == HOMING_OK) return true;

        if (res == HOMING_FAILED) {
            Serial.print("HOMING: axis ");
            Serial.print(name);
            Serial.println(" failed -> ALARM");
            changeState(STATE_ALARM);
        } else {
            Serial.print("HOMING: axis ");
            Serial.print(name);
            Serial.println(" aborted by stop request.");
        }
        return false;
    };

    // 1. Первой всегда строго вверх уходит безопасная ось Z, чтобы не сломать фрезу об прижимы
    if (!homeOneAxis(Z_STEP_PIN, Z_DIR_PIN, cfg.homingDirZ, currentStepsZ, cfg.pullOffZ, "Z")) return;
    plannerStepsZ = 0; // Синхронизируем базу планировщика

    // 2. Затем в базовый угол синхронно или последовательно уходят оси X и Y
    if (!homeOneAxis(X_STEP_PIN, X_DIR_PIN, cfg.homingDirX, currentStepsX, cfg.pullOffX, "X")) return;
    plannerStepsX = 0;

    if (!homeOneAxis(Y_STEP_PIN, Y_DIR_PIN, cfg.homingDirY, currentStepsY, cfg.pullOffY, "Y")) return;
    plannerStepsY = 0;

    // Все базы успешно привязаны
    isHomed = true;
    changeState(STATE_IDLE);
    Serial.println("STATUS: Homing cycle completed successfully. Machine unlocked.");
}

// ============================================================================
// СТРУКТУРНЫЙ БЛОК: ПЛАВНЫЙ FEEDRATE OVERRIDE И РАМПА ПАУЗЫ (Core 0)
// ============================================================================

void updateFeedrateFading() {
    static unsigned long lastFadeTime = 0;
    static MachineState lastState = STATE_IDLE; 
    
    unsigned long now = millis();
    if (now - lastFadeTime < 5) return;
    
    // Честная дельта времени (0.005 сек)
    float dt = (float)(now - lastFadeTime) / 1000.0f;
    lastFadeTime = now;

    // Максимальный шаг изменения физической скорости за 5 мс на основе ускорения
    float maxDeltaV = cfg.maxAcceleration * dt; 

    // Определяем физическую цель для лимита скорости
    float targetSpeedLimit = 150.0f; // Любое заведомо высокое значение (мм/сек)

    if (currentMachineState == STATE_HOLD) {
        if (lastState != STATE_HOLD) {
            // Подхватываем актуальную внутреннюю скорость из motionTask
            virtualSpeedLimit = sharedCurrentVelocity; 
        }
        targetSpeedLimit = 0.0f; // При паузе цель — полная физическая остановка
    }

    lastState = currentMachineState; 

    // Плавно ведем лимит к цели строго по физическому ускорению
    if (virtualSpeedLimit < targetSpeedLimit) {
        virtualSpeedLimit += maxDeltaV;
        if (virtualSpeedLimit > targetSpeedLimit) virtualSpeedLimit = targetSpeedLimit;
    } 
    else if (virtualSpeedLimit > targetSpeedLimit) {
        virtualSpeedLimit -= maxDeltaV;
        if (virtualSpeedLimit < targetSpeedLimit) virtualSpeedLimit = targetSpeedLimit;
    }

    // Здесь целевое значение берется из глобальных настроек ползунка (по умолчанию 1.0f)
    //float targetOverride = cfg.feedMultiplier;

    const float FADE_STEP = 5.0f / 300.0f; // ~0.0166 на один шаг

    if (smoothedFeedrateOverride < pythonFeedrateOverride) {
        smoothedFeedrateOverride += FADE_STEP;
        if (smoothedFeedrateOverride > pythonFeedrateOverride) smoothedFeedrateOverride = pythonFeedrateOverride;
    } 
    else if (smoothedFeedrateOverride > pythonFeedrateOverride) {
        smoothedFeedrateOverride -= FADE_STEP;
        if (smoothedFeedrateOverride < pythonFeedrateOverride) smoothedFeedrateOverride = pythonFeedrateOverride;
    }
}


void startMotionTask() {
    // Создаем изолированную задачу для Ядра 1 с наивысшим приоритетом 24
    xTaskCreatePinnedToCore(
        motionTask,     // Функция задачи
        "motionTask",   // Имя задачи
        8192,           // Размер стека (выделяем с запасом под float-вычисления)
        NULL,           // Параметры
        24,             // Приоритет
        NULL,           // Хэндл
        1               // Жесткая привязка к Core 1
    );
}
