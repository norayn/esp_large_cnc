#include "motion.h"
#include "config.h"
#include "pinout.h"
#include "gcode_program.h"
#include "state_manager.h"

// Физическое положение станка (изменяется только в прерывании)
volatile long currentStepsX = 0;
volatile long currentStepsY = 0;
volatile long currentStepsZ = 0;
volatile bool isVectorMoving = false;

static CorrectionPoint curvatureMap[250]; 

// --- КОЛЬЦЕВОЙ БУФЕР ШАГОВ ---
#define STEP_BUFFER_SIZE 4096 // Увеличим буфер для стабильности

struct StepCmd {
    uint8_t stepX : 1;
    uint8_t dirX  : 1;
    uint8_t stepY : 1;
    uint8_t dirY  : 1;
    uint8_t stepZ : 1;
    uint8_t dirZ  : 1;
    uint8_t valid : 1; 
};

// Убираем volatile со всего массива, оставляем атомарными только указатели
static StepCmd stepRingBuffer[STEP_BUFFER_SIZE];
volatile uint32_t bufferHead = 0; 
volatile uint32_t bufferTail = 0; 

// Переменные плавного изменения скорости
static float targetMultiplier = 1.0f;
static unsigned long lastFadingTime = 0;
const unsigned long FADING_DURATION = 300; // Плавный стоп/разгон за 300 мс

// Глобальная переменная для ползунка скорости из Python (определена в motion.h)
float pythonFeedrateOverride = 1.0f;

volatile InputSegment nextSeg = {0, 0, 0, 0, 0, false};

void initMotion() {
    isVectorMoving = false;
    currentStepsX = 0; currentStepsY = 0; currentStepsZ = 0;
    bufferHead = 0;
    bufferTail = 0;
    
    for (int i = 0; i < 250; i++) {
        curvatureMap[i].y_offset = 0.0f;
        curvatureMap[i].z_offset = 0.0f;
    }
    for (int i = 0; i < STEP_BUFFER_SIZE; i++) {
        stepRingBuffer[i].valid = 0;
    }
}

void updateCurvaturePoint(int index, float y_off, float z_off) {
    if (index >= 0 && index < cfg.laserMapSize) {
        curvatureMap[index].y_offset = y_off;
        curvatureMap[index].z_offset = z_off;
    }
}

CorrectionPoint getCorrection(float currentX) {
    if (currentX < 0) currentX = 0;
    float max_len = (cfg.laserMapSize - 1) * cfg.laserGridStep;
    if (currentX > max_len) currentX = max_len;

    int index = (int)(currentX / cfg.laserGridStep);
    if (index >= cfg.laserMapSize - 1) return curvatureMap[cfg.laserMapSize - 1];

    float t = (currentX - (index * cfg.laserGridStep)) / cfg.laserGridStep;
    CorrectionPoint corr;
    corr.y_offset = curvatureMap[index].y_offset + t * (curvatureMap[index+1].y_offset - curvatureMap[index].y_offset);
    corr.z_offset = curvatureMap[index].z_offset + t * (curvatureMap[index+1].z_offset - curvatureMap[index].z_offset);
    return corr;
}

// Вызывается Ядром 0: теперь она работает мгновенно (не ждет физического доезда!)
void prepareVectorSegment(float newX, float newY, float newZ, float feedRateMM_Min, float accelMM_Sec2) {
    // Ждем ТОЛЬКО если поток Ядра 1 еще не успел забрать ПРЕДЫДУЩУЮ команду из ячейки-посредника
    while (nextSeg.hasNewData) { 
        delay(1); 
    }
    
    nextSeg.x = newX; 
    nextSeg.y = newY; 
    nextSeg.z = newZ;
    nextSeg.f = feedRateMM_Min; 
    nextSeg.a = accelMM_Sec2;
    
    nextSeg.hasNewData = true; // Пинаем Ядро 1
    
    // Взводим флаг движения. Теперь Ядро 0 мгновенно выходит из функции и возвращается в loop()
    isVectorMoving = true; 
}

// --- ПОТОК ПЛАНИРОВЩИКА НА ЯДРЕ 1 ---
// --- ПОТОК ПЛАНИРОВЩИКА НА ЯДРЕ 1 ---
void motionTask(void * parameter) {
    const float DT = 0.00001f; 
 
    // Переменные положения планировщика (живут на протяжении всей работы потока)
    long localStepsX = 0; 
    long localStepsY = 0; 
    long localStepsZ = 0;

    while (true) {
        if (!nextSeg.hasNewData) {
            if (bufferTail == bufferHead) {
                if (isVectorMoving) {
                    isVectorMoving = false;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1)); 
            continue;
        }

        // Синхронизируем виртуальный счетчик с реальным положением ПЕРЕД началом нового кадра!
        localStepsX = currentStepsX;
        localStepsY = currentStepsY;
        localStepsZ = currentStepsZ;

        float nX = nextSeg.x; float nY = nextSeg.y; float nZ = nextSeg.z;
        float feed = nextSeg.f; float accel = nextSeg.a;
        nextSeg.hasNewData = false; 

        if (isHomed) {
            if (nX < cfg.minX) nX = cfg.minX; if (nX > cfg.maxX) nX = cfg.maxX;
            if (nY < cfg.minY) nY = cfg.minY; if (nY > cfg.maxY) nY = cfg.maxY;
            if (nZ < cfg.minZ) nZ = cfg.minZ; if (nZ > cfg.maxZ) nZ = cfg.maxZ;
        }

        CorrectionPoint corr = getCorrection(nX);
        float realMachineY = nY + corr.y_offset;
        float realMachineZ = nZ + corr.z_offset;

        float dx = nX - ((float)localStepsX / cfg.stepsPerMmX);
        float dy = realMachineY - ((float)localStepsY / cfg.stepsPerMmY);
        float dz = realMachineZ - ((float)localStepsZ / cfg.stepsPerMmZ);
        float totalLength = sqrt(dx*dx + dy*dy + dz*dz);

        if (totalLength < 0.001f) { continue; }

        float Kx = fabs(dx) / totalLength; int dirX = (dx >= 0) ? 1 : -1;
        float Ky = fabs(dy) / totalLength; int dirY = (dy >= 0) ? 1 : -1;
        float dz_val = fabs(dz) / totalLength; int dirZ = (dz >= 0) ? 1 : -1;

        float maxSpeed = feed / 60.0f;
        float minSpeed = cfg.minVectorSpeed;
        float currentVectorSpeed = minSpeed;

        float t_accel = (maxSpeed - minSpeed) / accel;
        float s_accel = minSpeed * t_accel + 0.5f * accel * t_accel * t_accel;
        long totalTicks = 0; long accelTicks = 0, decelTicks = 0;

        if (s_accel * 2.0f > totalLength) {
            float halfLen = totalLength / 2.0f;
            float t_real = (-minSpeed + sqrt(minSpeed*minSpeed + 2.0f*accel*halfLen)) / accel;
            totalTicks = (long)((t_real * 2.0f) / DT);
            accelTicks = (long)(t_real / DT); decelTicks = accelTicks;
        } else {
            float s_cruise = totalLength - (s_accel * 2.0f);
            float t_cruise = s_cruise / maxSpeed;
            totalTicks = (long)((t_accel * 2.0f + t_cruise) / DT);
            accelTicks = (long)(t_accel / DT); decelTicks = accelTicks;
        }

        // Инициализируем идеальные координаты строго от текущей точки локального счетчика кадра!
        float idealX = (float)localStepsX / cfg.stepsPerMmX;
        float idealY = (float)localStepsY / cfg.stepsPerMmY;
        float idealZ = (float)localStepsZ / cfg.stepsPerMmZ;

        if (cfg.feedMultiplier < 0.01f) cfg.feedMultiplier = 1.0f;

        float currentS = 0.0f; // Абсолютный пройденный путь по вектору
        // Запоминаем стартовые физические координаты кадра
        float startX = (float)localStepsX / cfg.stepsPerMmX;
        float startY = (float)localStepsY / cfg.stepsPerMmY;
        float startZ = (float)localStepsZ / cfg.stepsPerMmZ;

        // ГЕНЕРАЦИЯ ТАКТОВ ВРЕМЕНИ
        for (long tick = 0; tick < totalTicks; tick++) {
            
            while (cfg.feedMultiplier <= 0.001f) {
                currentVectorSpeed = minSpeed;
                vTaskDelay(pdMS_TO_TICKS(5)); 
            }

            float targetSpeed = maxSpeed * cfg.feedMultiplier;
            long ticksToExtremity = totalTicks - tick;
            if (ticksToExtremity <= decelTicks) {
                targetSpeed = minSpeed * cfg.feedMultiplier;
            }

            if (currentVectorSpeed < targetSpeed) {
                currentVectorSpeed += accel * DT;
                if (currentVectorSpeed > targetSpeed) currentVectorSpeed = targetSpeed;
            } else if (currentVectorSpeed > targetSpeed) {
                currentVectorSpeed -= accel * DT;
                if (currentVectorSpeed < targetSpeed) currentVectorSpeed = targetSpeed;
            }

            currentS += currentVectorSpeed * DT;
            if (currentS > totalLength) { currentS = totalLength; }
            idealX = startX + currentS * Kx * dirX;
            idealY = startY + currentS * Ky * dirY;
            idealZ = startZ + currentS * dz_val * dirZ;

            long tStepsX = (long)roundf(idealX * cfg.stepsPerMmX);
            long tStepsY = (long)roundf(idealY * cfg.stepsPerMmY);
            long tStepsZ = (long)roundf(idealZ * cfg.stepsPerMmZ);

            uint32_t nextHead = (bufferHead + 1) % STEP_BUFFER_SIZE;
            while (nextHead == bufferTail) { 
                vTaskDelay(pdMS_TO_TICKS(2)); 
            }

            StepCmd cmd = {0, 0, 0, 0, 0, 0, 0};
            bool stateChanged = false;

            // За один такт времени (10 мкс) мы делаем максимум 1 дискретный шаг (через IF)
            if (localStepsX != tStepsX) { cmd.stepX = 1; cmd.dirX = (tStepsX > localStepsX) ? 1 : 0; localStepsX += (tStepsX > localStepsX) ? 1 : -1; stateChanged = true; }
            if (localStepsY != tStepsY) { cmd.stepY = 1; cmd.dirY = (tStepsY > localStepsY) ? 1 : 0; localStepsY += (tStepsY > localStepsY) ? 1 : -1; stateChanged = true; }
            if (localStepsZ != tStepsZ) { cmd.stepZ = 1; cmd.dirZ = (tStepsZ > localStepsZ) ? 1 : 0; localStepsZ += (tStepsZ > localStepsZ) ? 1 : -1; stateChanged = true; }
            
            cmd.valid = 1;
            stepRingBuffer[bufferHead] = cmd;
            bufferHead = nextHead; 
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}



void startMotionTask() {
    xTaskCreatePinnedToCore(motionTask, "CNC_Motion", 4096, NULL, 24, NULL, 1);
}

// --- ФИНАЛЬНОЕ ПРЕРЫВАНИЕ ТАЙМЕРА (100 кГц) ---
void IRAM_ATTR onTimerInterrupt() {
    // Таймер больше не зависит от флага и кэша ядер. Он работает всегда!
    // Проверка физической безопасности
    if (digitalRead(ESTOP_PIN) == HIGH || digitalRead(MOTORS_ALARM) == HIGH || digitalRead(ENDSTOPS_PIN) == HIGH) {
        digitalWrite(MOTORS_ENABLE, HIGH); 
        bufferTail = bufferHead; // Сброс буфера
        return;
    }

    // Если указатели не равны — в буфере ЕСТЬ шаги, выполняем их мгновенно!
    if (bufferTail != bufferHead) {
        StepCmd cmd = stepRingBuffer[bufferTail];
        
        if (cmd.valid) {
            bool stepIssued = false;

            if (cmd.stepX) {
                digitalWrite(X_DIR_PIN, cmd.dirX ? HIGH : LOW);
                digitalWrite(X_STEP_PIN, HIGH);
                currentStepsX += cmd.dirX ? 1 : -1; 
                stepIssued = true;
            }
            if (cmd.stepY) {
                digitalWrite(Y_DIR_PIN, cmd.dirY ? HIGH : LOW);
                digitalWrite(Y_STEP_PIN, HIGH);
                currentStepsY += cmd.dirY ? 1 : -1;
                stepIssued = true;
            }
            if (cmd.stepZ) {
                digitalWrite(Z_DIR_PIN, cmd.dirZ ? HIGH : LOW);
                digitalWrite(Z_STEP_PIN, HIGH);
                currentStepsZ += cmd.dirZ ? 1 : -1;
                stepIssued = true;
            }

            if (stepIssued) {
                delayMicroseconds(2);
                digitalWrite(X_STEP_PIN, LOW);
                digitalWrite(Y_STEP_PIN, LOW);
                digitalWrite(Z_STEP_PIN, LOW);
            }
            // Сдвигаем указатель чтения (освобождая место для потока)
            bufferTail = (bufferTail + 1) % STEP_BUFFER_SIZE;
        } else {
            // Аварийный сброс хвоста при пустом буфере
            bufferTail = bufferHead; 
        }
    }
}

// Обычные функции хоуминга (работают в контексте задач, а не прерываний)
bool homeAxis(int stepPin, int dirPin, int dirSign, volatile long &axisSteps, long pullOffSteps) {
    digitalWrite(dirPin, (dirSign == 1) ? HIGH : LOW);
    while (digitalRead(ENDSTOPS_PIN) == LOW) {
        digitalWrite(stepPin, HIGH); 
        delayMicroseconds(50);
        digitalWrite(stepPin, LOW);  
        delayMicroseconds(50);
        if (digitalRead(ESTOP_PIN) == HIGH) return false;
    }

    digitalWrite(dirPin, (dirSign == 1) ? LOW : HIGH);
    for (long i = 0; i < pullOffSteps; i++) {
        digitalWrite(stepPin, HIGH); 
        delayMicroseconds(100);
        digitalWrite(stepPin, LOW);  
        delayMicroseconds(100);
    }

    delay(200);
    digitalWrite(dirPin, (dirSign == 1) ? HIGH : LOW);
    while (digitalRead(ENDSTOPS_PIN) == LOW) {
        digitalWrite(stepPin, HIGH); 
        delayMicroseconds(200);
        digitalWrite(stepPin, LOW);  
        delayMicroseconds(200);
    }

    axisSteps = 0;
    digitalWrite(dirPin, (dirSign == 1) ? LOW : HIGH);
    for (long i = 0; i < pullOffSteps; i++) {
        digitalWrite(stepPin, HIGH); 
        delayMicroseconds(100);
        digitalWrite(stepPin, LOW);  
        delayMicroseconds(100);
    }

    delay(200);
    return true;
}

void runFullHoming() {
    isVectorMoving = false;
    bufferTail = bufferHead;
    if (!homeAxis(Z_STEP_PIN, Z_DIR_PIN, 1, currentStepsZ, 1200)) { 
        isHomed = false; 
        return; 
    }
    
    if (!homeAxis(Y_STEP_PIN, Y_DIR_PIN, -1, currentStepsY, 1200)) { 
        isHomed = false; 
        return; 
    }

    if (!homeAxis(X_STEP_PIN, X_DIR_PIN, -1, currentStepsX, 1200)) { 
        isHomed = false; 
        return; 
    }

    isHomed = true;
}

void updateFeedrateFading() {
    // 1. Определяем целевую скорость на основе состояния автомата ЧПУ
    if (currentMachineState == STATE_HOLD || currentMachineState == STATE_ALARM) {
        targetMultiplier = 0.0f; // Пауза или Авария -> плавно гасим скорость в ноль
    } else {
        targetMultiplier = pythonFeedrateOverride; // Штатный режим -> берем значение с ПК
    }

    // 2. Линейная рампа изменения коэффициента каждые 5 миллисекунд
    if (millis() - lastFadingTime > 5) {
        lastFadingTime = millis();
        float step = 5.0f / (float)FADING_DURATION;

        if (cfg.feedMultiplier < targetMultiplier) {
            cfg.feedMultiplier += step;
            if (cfg.feedMultiplier > targetMultiplier) cfg.feedMultiplier = targetMultiplier;
        } 
        else if (cfg.feedMultiplier > targetMultiplier) {
            cfg.feedMultiplier -= step;
            if (cfg.feedMultiplier < targetMultiplier) cfg.feedMultiplier = targetMultiplier;
        }
    }
}