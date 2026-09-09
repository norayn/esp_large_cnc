#include "motion.h"
#include "config.h"
#include "pinout.h"
#include "gcode_program.h"
#include "state_manager.h"
#include <Arduino.h>

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

// Динамический программный Override общей подачи со слайдера PyQt6 (1.0 = 100%)
float pythonFeedrateOverride = 1.0f;


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

void prepareVectorSegment(float newX, float newY, float newZ, float feedRateMM_Min, 
                          float accelMM_Sec2, float vStartMM_Sec, float vEndMM_Sec) {
    
    // Блокировка от затирания: ждем, пока Ядро 1 заберет предыдущий пакет
    while (nextSeg.hasNewData) { 
        delay(1); 
    }
    
    // Перекладываем геометрию и скорости кадра в структуру межъядерного обмена
    nextSeg.x = newX; 
    nextSeg.y = newY; 
    nextSeg.z = newZ;
    nextSeg.accel = accelMM_Sec2;
    
    // Приводим подачу кадра из мм/минуту в мм/секунду для физических формул Ядра 1
    nextSeg.v_frame = feedRateMM_Min / 60.0f; 
    
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
            for (long step = 0; step < totalSegSteps; step++) {
            
                // 1. ВЫЧИСЛЕНИЕ ТЕКУЩЕЙ ЧАСТОТЫ ИМПУЛЬСОВ МАСТЕР-ОСИ (в шагах в секунду)
                // Умножаем текущую скорость (мм/сек) на цену шага Мастер-Оси (шагов/мм) из конфига.
                // Пример: 25.0 мм/сек * 400 шагов/мм = 10 000 Импульсов/сек (Гц)
                float masterFrequency = currentVelocity * masterStepsPerMm;
            
                // Защита от деления на ноль: частота не может быть ниже 1 Гц (1 шаг в секунду)
                if (masterFrequency < 1.0f) masterFrequency = 1.0f;
            
            
                // 2. ПЕРЕВОД ФИЗИЧЕСКОЙ ЧАСТОТЫ В ДИСКРЕТНЫЕ КВАНТЫ ЧПУ-ЛЕНТЫ
                // Делим базовую частоту планирования сетки (BASE_CNC_FREQ = 100 000 Гц) на частоту шагов,
                // дополнительно умноженную на глобальный множитель подачи pythonFeedrateOverride.
                // Пример при 100% подаче (1.0f): 100 000 / (10 000 * 1.0) = 10 тактов паузы на этот шаг.
                uint16_t delay100kHz = (uint16_t)(BASE_CNC_FREQ / (masterFrequency * pythonFeedrateOverride));
            
                // Аппаратная защита таймера: задержка не может быть меньше 1 такта сетки (10 микросекунд).
                // Это ограничивает максимальную частоту импульсов на отметке 100 кГц, предотвращая зависание ISR.
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
                        // Уступаем квант времени операционной системе FreeRTOS на 1 мс.
                        // Прерывание таймера на Core 1 продолжит читать буфер и освободит место.
                        vTaskDelay(pdMS_TO_TICKS(1)); 
                    }
                
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

        isVectorMoving = true; // Жестко взводим флаг физического старта движения!

        // Фиксируем эталонную базу
        plannerStepsX = targetStepsX;
        plannerStepsY = targetStepsY;
        plannerStepsZ = targetStepsZ;

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ============================================================================
// СТРУКТУРНЫЙ БЛОК: ВЫСОКОСКОРОСТНОЕ ДВУХФАЗНОЕ ПРЕРЫВАНИЕ ТАЙМЕРА (200 кГц)
// ============================================================================

// Флаг фазы таймера: true = нечетный такт (HIGH), false = четный такт (LOW)
static volatile bool timerPhaseHigh = true;

void IRAM_ATTR onTimerInterrupt() {
    // Проверяем критические аварии, которые должны сработать ВСЕГДА
    bool criticalAlarm = (digitalRead(ESTOP_PIN) == HIGH || digitalRead(MOTORS_ALARM) == HIGH);
    
    // Линию концевиков подмешиваем к аварии ТОЛЬКО если сейчас не идет процедура хоуминга
    if (!isHomingActive && digitalRead(ENDSTOPS_PIN) == HIGH) {
        criticalAlarm = true;
    }

    if (criticalAlarm) {
        digitalWrite(MOTORS_ENABLE, HIGH); // Немедленно обесточить драйверы Leadshine
        
        // Жестко сбрасываем буфер времени и останавливаем станок
        bufferTail = bufferHead; 
        isVectorMoving = false; // Мгновенно гасим физический статус движения
        isPlannerBusy = false;  // Освобождаем конвейер данных
        
        timerPhaseHigh = true; // Сбрасываем фазу
        
        // Переводим станок в аварийный стейт ALARM на Ядре 0
        changeState(STATE_ALARM); 
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

// Функция поиска базы для одной конкретной оси через кольцевой буфер времени
bool homeAxis(int stepPin, int dirPin, int dirSign, volatile long &axisSteps, int endstopPin, long pullOffSteps) {
    const uint16_t HOMING_DELAY_TICKS = 50; // Жесткая задержка скорости хоуминга (эквивалент ~2 кГц)
    
    isHomingActive = true;
    // Шаг 1: Едем в сторону датчика до физического срабатывания
    while (digitalRead(endstopPin) == LOW) {
        // Если оператор нажал E-STOP во время поиска баз — немедленно выходим
        if (digitalRead(ESTOP_PIN) == HIGH) return false;

        uint32_t nextHead = (bufferHead + 1) % STEP_BUFFER_SIZE;
        while (nextHead == bufferTail) { vTaskDelay(pdMS_TO_TICKS(1)); }

        // Формируем потактовую пачку времени для одного шага хоуминга
        for (uint16_t t = 0; t < HOMING_DELAY_TICKS; t++) {
            StepCmd cmd = {0, 0, 0, 0, 0, 0, 0};
            if (t == 0) {
                if (stepPin == X_STEP_PIN) { cmd.stepX = 1; cmd.dirX = (dirSign > 0) ? 1 : 0; }
                if (stepPin == Y_STEP_PIN) { cmd.stepY = 1; cmd.dirY = (dirSign > 0) ? 1 : 0; }
                if (stepPin == Z_STEP_PIN) { cmd.stepZ = 1; cmd.dirZ = (dirSign > 0) ? 1 : 0; }
            }
            cmd.valid = 1;
            stepRingBuffer[bufferHead] = cmd;
            bufferHead = (bufferHead + 1) % STEP_BUFFER_SIZE;
        }
    }

    // Ждем, пока таймер полностью выгребет остатки шагов из буфера и мотор физически замрет
    while (bufferTail != bufferHead) { vTaskDelay(pdMS_TO_TICKS(1)); }
    vTaskDelay(pdMS_TO_TICKS(100)); // Короткая пауза для гашения инерции фермы

    // Шаг 2: Обязательный целочисленный отъезд назад (Pull-off) для освобождения датчика
    for (long i = 0; i < pullOffSteps; i++) {
        uint32_t nextHead = (bufferHead + 1) % STEP_BUFFER_SIZE;
        while (nextHead == bufferTail) { vTaskDelay(pdMS_TO_TICKS(1)); }

        for (uint16_t t = 0; t < HOMING_DELAY_TICKS; t++) {
            StepCmd cmd = {0, 0, 0, 0, 0, 0, 0};
            if (t == 0) {
                // Едем строго в противоположную сторону (разворачиваем dirSign)
                if (stepPin == X_STEP_PIN) { cmd.stepX = 1; cmd.dirX = (dirSign > 0) ? 0 : 1; }
                if (stepPin == Y_STEP_PIN) { cmd.stepY = 1; cmd.dirY = (dirSign > 0) ? 0 : 1; }
                if (stepPin == Z_STEP_PIN) { cmd.stepZ = 1; cmd.dirZ = (dirSign > 0) ? 0 : 1; }
            }
            cmd.valid = 1;
            stepRingBuffer[bufferHead] = cmd;
            bufferHead = (bufferHead + 1) % STEP_BUFFER_SIZE;
        }
    }

    // Снова ждем полной физической остановки
    while (bufferTail != bufferHead) { vTaskDelay(pdMS_TO_TICKS(1)); }
    
    // Жестко зануляем физический и математический счетчик оси! База официально найдена.
    axisSteps = 0; 
    isHomingActive = false;
    return true;
}

// Главный последовательный автомат поиска баз ЧПУ (Вызывается из handleIdleCommands на Core 0)
void runFullHoming() {
    changeState(STATE_HOMING);
    isHomed = false;

    // 1. Первой всегда строго вверх уходит безопасная ось Z, чтобы не сломать фрезу об прижимы
    if (!homeAxis(Z_STEP_PIN, Z_DIR_PIN, cfg.homingDirZ, currentStepsZ, ENDSTOPS_PIN, cfg.pullOffZ)) {
        changeState(STATE_ALARM); return;
    }
    plannerStepsZ = 0; // Синхронизируем базу планировщика

    // 2. Затем в базовый угол синхронно или последовательно уходят оси X и Y
    if (!homeAxis(X_STEP_PIN, X_DIR_PIN, cfg.homingDirX, currentStepsX, ENDSTOPS_PIN, cfg.pullOffX)) {
        changeState(STATE_ALARM); return;
    }
    plannerStepsX = 0;

    if (!homeAxis(Y_STEP_PIN, Y_DIR_PIN, cfg.homingDirY, currentStepsY, ENDSTOPS_PIN, cfg.pullOffY)) {
        changeState(STATE_ALARM); return;
    }
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
    unsigned long now = millis();
    
    // Выдерживаем шаг изменения рампы раз в 5 миллисекунд
    if (now - lastFadeTime < 5) return;
    lastFadeTime = now;

    // Шаг изменения коэффициента за 5 мс, чтобы вся рампа от 0% до 100% длилась ровно 300 мс
    const float FADE_STEP = 5.0f / 300.0f; // ~0.0166 на один шаг

    // СЦЕНАРИЙ А: Нажата кнопка Паузы (HOLD). Плавно гасим скорость до полного нуля
    if (currentMachineState == STATE_HOLD) {
        if (pythonFeedrateOverride > 0.0f) {
            pythonFeedrateOverride -= FADE_STEP;
            if (pythonFeedrateOverride < 0.0f) {
                pythonFeedrateOverride = 0.0f; // Станок полностью замер, но буфер и траектория сохранены!
            }
        }
    }
    // СЦЕНАРИЙ Б: Работает автоматическая программа. Плавно возвращаем скорость к 100% (или значению слайдера)
    else if (currentMachineState == STATE_RUNNING) {
        // Здесь целевое значение берется из глобальных настроек ползунка (по умолчанию 1.0f)
        float targetOverride = cfg.feedMultiplier; 
        
        if (pythonFeedrateOverride < targetOverride) {
            pythonFeedrateOverride += FADE_STEP;
            if (pythonFeedrateOverride > targetOverride) pythonFeedrateOverride = targetOverride;
        }
        else if (pythonFeedrateOverride > targetOverride) {
            pythonFeedrateOverride -= FADE_STEP;
            if (pythonFeedrateOverride < targetOverride) pythonFeedrateOverride = targetOverride;
        }
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
