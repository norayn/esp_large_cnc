#include "motion.h"
#include "config.h"
#include "pinout.h"
#include "gcode_program.h"
#include "state_manager.h"
#include <Arduino.h>

// Аппаратный таймер создаётся в main.cpp; нужен для остановки/запуска выдачи шагов.
extern hw_timer_t* cncTimer;

// Размер кольцевой ленты времени (8192 тика = 40.96 миллисекунд подушки безопасности ОЗУ)
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

// Флаг, сигнализирующий, что генератор DDA был заморожен в мертвой точке паузы
static bool isGeneratorSuspendedOnHold = false;

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

// Объявление структуры-посредника неблокирующего обмена кадрами между Ядром 0 и Ядром 1
volatile InputSegment nextSeg = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};


// ============================================================================
// СТРУКТУРНЫЙ БЛОК: ОСТАНОВКА ДВИЖЕНИЯ И АВАРИЙНЫЕ ЗАПРОСЫ
// ============================================================================

// Разрешает выдачу шагов. Вызывается перед новым кадром/хоумингом.
// Возвращает false, если станок в ALARM или авария ещё не обработана.
static bool armMotionTimer() {
    if (currentMachineState == STATE_ALARM || alarmRequested) return false;

    if (motionStopRequested) {
        noInterrupts();
        bufferHead = 0;
        bufferTail = 0;
        motionStopRequested = false;
        interrupts();
    }

    // Фазу таймера здесь не трогаем: ISR сам завершит текущий импульс STEP.
    if (cncTimer != nullptr) timerAlarmEnable(cncTimer);
    return true;
}

// Мягкая остановка: выдача шагов прекращается сразу, очередь сбрасывается,
// математическая база планировщика приравнивается к фактической позиции.
void requestMotionStop() {
    motionStopRequested = true;

    if (cncTimer != nullptr) timerAlarmDisable(cncTimer);

    noInterrupts();
    bufferHead = 0;
    bufferTail = 0;
    plannerStepsX = currentStepsX;
    plannerStepsY = currentStepsY;
    plannerStepsZ = currentStepsZ;
    interrupts();

    digitalWrite(X_STEP_PIN, LOW);
    digitalWrite(Y_STEP_PIN, LOW);
    digitalWrite(Z_STEP_PIN, LOW);

    timerPhaseHigh = true;
    isVectorMoving = false;
    isPlannerBusy = false;
    isGeneratorSuspendedOnHold = false;
    nextSeg.hasNewData = false;

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
// СТРУКТУРНЫЙ БЛОК: НЕБЛОКИРУЮЩАЯ ПЕРЕДАЧА КАДРА МЕЖДУ ЯДРАМИ (Core 0)
// ============================================================================

void prepareVectorSegment(float newX, float newY, float newZ, float feedRateMM_Sec, 
                          float accelMM_Sec2, float vStartMM_Sec, float vEndMM_Sec) {
    
    if (!armMotionTimer()) return;

    // Блокировка от затирания: ждем, пока Ядро 1 заберет предыдущий пакет
    while (nextSeg.hasNewData) { 
        if (motionStopRequested || alarmRequested) return;
        delay(1); 
    }

    // Остановка могла прийти, пока ждали освобождения посредника.
    if (motionStopRequested || alarmRequested) return;
    
    // Перекладываем геометрию и скорости кадра в структуру межъядерного обмена
    nextSeg.x = newX; 
    nextSeg.y = newY; 
    nextSeg.z = newZ;
    nextSeg.accel = accelMM_Sec2;
    
    // Подача кадра  мм/секунду для физических формул Ядра 1
    nextSeg.v_frame = feedRateMM_Sec; 
    
    // Принимаем готовые Look-Ahead скорости сопряжения от Python (они уже в мм/сек)
    nextSeg.v_start = vStartMM_Sec;         
    nextSeg.v_end = vEndMM_Sec;             
    
    // Атомарная отмашка для потока Ядра 1 и взвод флага движения для main.cpp
    nextSeg.hasNewData = true; 
    isPlannerBusy  = true; 
    isVectorMoving = true;
}

// ============================================================================
// СТРУКТУРНЫЙ БЛОК: ЯДРО ПЛАНИРОВЩИКА ДВИЖЕНИЯ ЧПУ (Core 1)
// ============================================================================

void motionTask(void * parameter) {
    // Внутренняя фиксированная частота работы нашего двухфазного таймера (200 кГц)
    const float TIMER_FREQ = 200000.0f; 
    
    // Локальные переменные потока для отслеживания шагов внутри кадра
    long localStepsX = 0;
    long localStepsY = 0;
    long localStepsZ = 0;

    while (true) {
        // Остановка/авария: не начинаем новый кадр и не трогаем буфер.
        if (motionStopRequested || alarmRequested) {
            nextSeg.hasNewData = false;
            isVectorMoving = false;
            isPlannerBusy = false;
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        // --- 4.1.1. ОПРОС И БЛОКИРОВКА МЕЖЪЯДЕРНОЙ ОЧЕРЕДИ ---
        if (!nextSeg.hasNewData) {
            // Если в кольцевом буфере физически кончились шаги, сигнализируем Ядру 0 в main.cpp
            if (bufferTail == bufferHead) {
                isPlannerBusy = false;
            }
            vTaskDelay(pdMS_TO_TICKS(1)); // Неблокирующее ожидание FreeRTOS (1 мс)
            continue;
        }

        // --- 4.1.2. ИЗВЛЕЧЕНИЕ ПАРАМЕТРОВ КАДРА И СКОРОСТЕЙ (в мм/сек) ---
        float nX = nextSeg.x; 
        float nY = nextSeg.y; 
        float nZ = nextSeg.z;
        float accel = nextSeg.accel;
        
        float v_max   = nextSeg.v_frame; // Ограничение подачи кадра от CAM
        float v_start = nextSeg.v_start; // Скорость входа от Look-Ahead Python
        float v_end   = nextSeg.v_end;   // Скорость выхода на стыке угла от Look-Ahead Python
        
        nextSeg.hasNewData = false;      // Мгновенный Handshake: освобождаем ячейку для Ядра 0

        // --- 4.1.3. ОПТИЧЕСКАЯ ЛАЗЕРНАЯ КОМПЕНСАЦИЯ ПРОВИСА ФЕРМЫ ---
        CorrectionPoint corr = getCorrection(nX);
        float realMachineY = nY + corr.y_offset;
        float realMachineZ = nZ + corr.z_offset;

        // --- 4.1.4. ПЕРЕВОД МИЛЛИМЕТРОВ В ЦЕЛОЧИСЛЕННЫЕ НАПРАВЛЕНИЯ ЧПУ ---
        // Рассчитываем точные целевые шаги, которые требует CAM-модель
        long targetStepsX = nX * cfg.stepsPerMmX;
        long targetStepsY = realMachineY * cfg.stepsPerMmY;
        long targetStepsZ = realMachineZ * cfg.stepsPerMmZ;

        // Вычисляем чистые дельты перемещения строго от идеальной математической базы прошлого кадра
        long totalDeltaX = targetStepsX - plannerStepsX;
        long totalDeltaY = targetStepsY - plannerStepsY;
        long totalDeltaZ = targetStepsZ - plannerStepsZ;

        long absDeltaX = labs(totalDeltaX);
        long absDeltaY = labs(totalDeltaY);
        long absDeltaZ = labs(totalDeltaZ);

        // Находим ведущую ось (максимальное дискретное перемещение в импульсах внутри этого кадра)
        long maxSegmentSteps = absDeltaX;
        if (absDeltaY > maxSegmentSteps) maxSegmentSteps = absDeltaY;
        if (absDeltaZ > maxSegmentSteps) maxSegmentSteps = absDeltaZ;

        // Защита от пустых кадров (команды шпинделя, охлаждения и т.д.)
        if (maxSegmentSteps == 0) {
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
        // Базовая ЧПУ-сетка частоты планирования — 100 кГц (1 такт = 10 микросекунд)
        const float BASE_CNC_FREQ = 100000.0f; 

        // Текущая физическая скорость вектора в начале кадра (мм/сек)
        float currentVelocity = v_start; 

        localStepsX = plannerStepsX;
        localStepsY = plannerStepsY;
        localStepsZ = plannerStepsZ;

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

                // АППАРАТНАЯ ЗАМОРОЗКА ГЕОМЕТРИИ DDA НА ПАУЗЕ (Проверка перед каждым шагом)
                // 1. Физическая заморозка геометрического шага DDA
                if (currentMachineState == STATE_HOLD && virtualSpeedLimit  <= 0.001f) {
                    isGeneratorSuspendedOnHold = true;

                    // Крутимся в легковесном цикле, пока удерживается пауза. 
                    // Математика Брезенхема/DDA замерла строго на текущем шаге.
                    while (currentMachineState == STATE_HOLD) {
                        if (motionStopRequested || alarmRequested) break;
                        vTaskDelay(pdMS_TO_TICKS(5)); 
                    }
                }

                if (motionStopRequested || alarmRequested) {
                    frameAborted = true;
                    break;
                }

                // 2. Выход из заморозки при нажатии RESUME.
                // Буфер не очищаем: уже рассчитанные шаги должны быть доиграны,
                // иначе теряется геометрия и plannerSteps уходит от реальности.
                if (isGeneratorSuspendedOnHold && currentMachineState == STATE_RUNNING) {
                    isGeneratorSuspendedOnHold = false;
                }
                // =====================================================

                // Считаем частоту Master-оси на основе её реального вклада в пространственную траекторию:
                float masterFrequency = currentVelocity * globalMasterStepsPerMm;
            
                // Защита от деления на ноль: частота не может быть ниже 1 Гц (1 шаг в секунду)
                if (masterFrequency < 1.0f) masterFrequency = 1.0f;

                // ВЫЧИСЛЯЕМ ЭФФЕКТИВНУЮ СКОРОСТЬ ВЕКТОРНОГО ШАГА:
                // Берем текущую скорость из G-код трапеции (currentVelocity) 
                // и искусственно зажимаем её сверху плавным лимитом паузы virtualSpeedLimit
                float effectiveVelocity = currentVelocity;
                if (effectiveVelocity > virtualSpeedLimit) {
                    effectiveVelocity = virtualSpeedLimit;
                }
                // Нижний предохранитель для стабильного наката на микро-скоростях
                if (effectiveVelocity < 0.1f) effectiveVelocity = 0.1f;
            
                // Считаем частоту шагов на основе эффективной скорости
                float effectiveMasterFrequency = effectiveVelocity * globalMasterStepsPerMm;
                if (effectiveMasterFrequency < 1.0f) effectiveMasterFrequency = 1.0f;
            
                // РАСЧЕТ ЗАДЕРЖКИ ТАЙМЕРА:
                // Но теперь частота плавно падает до нуля и плавно растет при старте за счет effectiveMasterFrequency!
                uint16_t delay100kHz = ( uint16_t)( BASE_CNC_FREQ / (effectiveMasterFrequency * smoothedFeedrateOverride ));
            
                if (delay100kHz < 1) delay100kHz = 1;
            
            
                // 3. РАСЧЕТ РЕАЛЬНОГО ФИЗИЧЕСКОГО ВРЕМЕНИ ДЛИТЕЛЬНОСТИ ТЕКУЩЕГО ШАГА
                // Переводим полученные такты обратно во float-секунды для физических формул разгона/торможения.
                // Пример: 10 тактов / 100 000 Гц = 0.0001 секунды (100 микросекунд) длится этот конкретный шаг.
                float stepDurationSec = (float)delay100kHz / BASE_CNC_FREQ;


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
                        
                // Цикл крутится ровно delay100kHz раз, нарезая 10-микросекундные кванты времени.
                // Если delay100kHz равен 10, то мы запишем 1 такт с импульсом и 9 тактов чистой паузы.
                for (uint16_t t = 0; t < delay100kHz; t++) {
                
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
                
                    // Сдвигаем индекс головы кольца вперед
                    bufferHead = nextHead;
                }
            
            } // Конец цикла по шагам текущего подвектора
        } // Конец цикла по 3-м подвекторам

        // Остановка во время генерации: кадр отбрасывается, plannerSteps уже
        // синхронизирован с фактическими счётчиками в requestMotionStop().
        if (frameAborted || motionStopRequested || alarmRequested) {
            nextSeg.hasNewData = false;
            isVectorMoving = false;
            isPlannerBusy = false;
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        isVectorMoving = true; // Жестко взводим флаг физического старта движения!

        // Фиксируем эталонную базу
        plannerStepsX = targetStepsX;
        plannerStepsY = targetStepsY;
        plannerStepsZ = targetStepsZ;

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
    bufferTail = bufferHead;

    isVectorMoving = false;
    isPlannerBusy = false;
    nextSeg.hasNewData = false;
    timerPhaseHigh = true;
}

void IRAM_ATTR onTimerInterrupt() {
    // Авария уже зафиксирована: драйверы обесточены, шаги заблокированы.
    // Повторно не дёргаем пины каждые 10 мкс, пока loop() не обработает флаг.
    if (alarmRequested) {
        digitalWrite(X_STEP_PIN, LOW);
        digitalWrite(Y_STEP_PIN, LOW);
        digitalWrite(Z_STEP_PIN, LOW);
        timerPhaseHigh = true;
        return;
    }

    // Проверяем критические аварии, которые должны сработать ВСЕГДА,
    // в том числе когда выдача шагов остановлена (IDLE/STOP/ALARM).
    bool criticalAlarm = (digitalRead(ESTOP_PIN) == HIGH || digitalRead(MOTORS_ALARM) == HIGH);
    
    // Линию концевиков подмешиваем к аварии ТОЛЬКО если сейчас не идет процедура хоуминга
    if (!isHomingActive && digitalRead(ENDSTOPS_PIN) == HIGH) {
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
    // Мы зашли сюда строго раз в 10 микросекунд (эквивалент базовой частоты ЧПУ 100 кГц)
    timerPhaseHigh = false; // Следующий тик таймера гарантированно пойдет в фазу LOW

    // Если в кольцевом буфере есть готовые такты времени от планировщика
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

            // Такт времени (10 мкс) успешно отработан — сдвигаем указатель чтения кольца!
            bufferTail = (bufferTail + 1) % STEP_BUFFER_SIZE;
        } else {
            // Защита от битых или неполных ячеек
            bufferTail = (bufferTail + 1) % STEP_BUFFER_SIZE;
        }
    } else {
        // Если буфер пуст (станок приехал в цель и замер в IDLE)
        isVectorMoving = false; 
    }
}

// ============================================================================
// СТРУКТУРНЫЙ БЛОК: АВТОМАТИЧЕСКИЙ ПОИСК БАЗ СТАНКА (HOMING)
// ============================================================================

// Период одного тика таймера, мкс (100 кГц). Нужен для расчёта таймаутов хоуминга.
static const float HOMING_TICK_US = 10.0f;
// 50 тиков на шаг = 0,5 мс -> ~2 кГц. Один шаг = одна пачка тиков.
static const uint16_t HOMING_DELAY_TICKS = 50;
// Пауза после остановки оси для гашения инерции фермы.
static const unsigned long HOMING_SETTLE_MS = 100;

// Проверка аварийных входов и запросов остановки во время хоуминга.
static HomingResult homingCheckInputs() {
    if (alarmRequested || digitalRead(ESTOP_PIN) == HIGH ||
        digitalRead(MOTORS_ALARM) == HIGH || currentMachineState == STATE_ALARM) {
        return HOMING_FAILED;
    }
    if (motionStopRequested) return HOMING_ABORTED;
    return HOMING_OK;
}

// Ставит в кольцевой буфер один шаг нужной оси (пачку тиков ожидания).
// Возвращает HOMING_OK, если такт записан; иначе — причину остановки.
static HomingResult enqueueHomingStep(int stepPin, int dirBit) {
    uint32_t nextHead = (bufferHead + 1) % STEP_BUFFER_SIZE;
    while (nextHead == bufferTail) {
        HomingResult st = homingCheckInputs();
        if (st != HOMING_OK) return st;
        vTaskDelay(pdMS_TO_TICKS(1));
        nextHead = (bufferHead + 1) % STEP_BUFFER_SIZE;
    }

    for (uint16_t t = 0; t < HOMING_DELAY_TICKS; t++) {
        StepCmd cmd = {0, 0, 0, 0, 0, 0, 0};
        if (t == 0) {
            if (stepPin == X_STEP_PIN) { cmd.stepX = 1; cmd.dirX = dirBit; }
            if (stepPin == Y_STEP_PIN) { cmd.stepY = 1; cmd.dirY = dirBit; }
            if (stepPin == Z_STEP_PIN) { cmd.stepZ = 1; cmd.dirZ = dirBit; }
        }
        cmd.valid = 1;
        stepRingBuffer[bufferHead] = cmd;
        bufferHead = (bufferHead + 1) % STEP_BUFFER_SIZE;
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
