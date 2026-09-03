/*
#include "motion.h"
#include "config.h"
#include "pinout.h"
#include "gcode_program.h"
#include "state_manager.h"

volatile long currentStepsX = 0;
volatile long currentStepsY = 0;
volatile long currentStepsZ = 0;
volatile bool isVectorMoving = false;

volatile VectorSegment vSeg;
volatile long vectorTicksCount = 0; 
volatile long totalVectorTicks = 0; 
volatile float currentVectorSpeed = 0; 

static float targetX = 0, targetY = 0, targetZ = 0;
static CorrectionPoint curvatureMap[250]; 
const float DT = 0.00001; // 100 кГц

void initMotion() {
    isVectorMoving = false;
    currentStepsX = 0; currentStepsY = 0; currentStepsZ = 0;
    for (int i = 0; i < 250; i++) {
        curvatureMap[i].y_offset = 0.0;
        curvatureMap[i].z_offset = 0.0;
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

void prepareVectorSegment(float newX, float newY, float newZ, float feedRateMM_Min, float accelMM_Sec2) {
    while (isVectorMoving) { delay(1); }

    if (isHomed) {
        if (newX < cfg.minX) newX = cfg.minX; if (newX > cfg.maxX) newX = cfg.maxX;
        if (newY < cfg.minY) newY = cfg.minY; if (newY > cfg.maxY) newY = cfg.maxY;
        if (newZ < cfg.minZ) newZ = cfg.minZ; if (newZ > cfg.maxZ) newZ = cfg.maxZ;
    }

    float dx = newX - (currentStepsX / cfg.stepsPerMmX);
    float dy = newY - (currentStepsY / cfg.stepsPerMmY); 
    float dz = newZ - (currentStepsZ / cfg.stepsPerMmZ);

    vSeg.totalLength_mm = sqrt(dx*dx + dy*dy + dz*dz);
    if (vSeg.totalLength_mm < 0.001) return; 

    vSeg.dirX = (dx >= 0) ? 1 : -1; vSeg.Kx = fabs(dx) / vSeg.totalLength_mm;
    vSeg.dirY = (dy >= 0) ? 1 : -1; vSeg.Ky = fabs(dy) / vSeg.totalLength_mm;
    vSeg.dirZ = (dz >= 0) ? 1 : -1; vSeg.Kz = fabs(dz) / vSeg.totalLength_mm;

    vSeg.maxTicksPerSec = feedRateMM_Min / 60.0; 
    vSeg.minTicksPerSec = cfg.minVectorSpeed;                  
    vSeg.accelRate = accelMM_Sec2;

    float t_accel = (vSeg.maxTicksPerSec - vSeg.minTicksPerSec) / vSeg.accelRate;
    float s_accel = vSeg.minTicksPerSec * t_accel + 0.5 * vSeg.accelRate * t_accel * t_accel;

    if (s_accel * 2 > vSeg.totalLength_mm) {
        float halfLength = vSeg.totalLength_mm / 2.0;
        float t_real = (-vSeg.minTicksPerSec + sqrt(vSeg.minTicksPerSec*vSeg.minTicksPerSec + 2*vSeg.accelRate*halfLength)) / vSeg.accelRate;
        totalVectorTicks = (long)((t_real * 2) / DT);
        vSeg.accelTicks = (long)(t_real / DT);
        vSeg.decelTicks = vSeg.accelTicks;
    } else {
        float s_cruise = vSeg.totalLength_mm - (s_accel * 2);
        float t_cruise = s_cruise / vSeg.maxTicksPerSec;
        totalVectorTicks = (long)((t_accel * 2 + t_cruise) / DT);
        vSeg.accelTicks = (long)(t_accel / DT);
        vSeg.decelTicks = (long)(t_accel / DT);
    }

    targetX = newX; targetY = newY; targetZ = newZ;
    vectorTicksCount = 0;
    currentVectorSpeed = vSeg.minTicksPerSec;
    isVectorMoving = true; 
}

bool homeAxis(int stepPin, int dirPin, int dirSign, volatile long &axisSteps, long pullOffSteps) {
    digitalWrite(dirPin, (dirSign == 1) ? HIGH : LOW);
    while (digitalRead(ENDSTOPS_PIN) == LOW) {
        digitalWrite(stepPin, HIGH); delayMicroseconds(50);
        digitalWrite(stepPin, LOW);  delayMicroseconds(50);
        if (digitalRead(ESTOP_PIN) == HIGH) return false; 
    }
    digitalWrite(dirPin, (dirSign == 1) ? LOW : HIGH);
    for (long i = 0; i < pullOffSteps; i++) {
        digitalWrite(stepPin, HIGH); delayMicroseconds(100);
        digitalWrite(stepPin, LOW);  delayMicroseconds(100);
    }
    delay(200);
    digitalWrite(dirPin, (dirSign == 1) ? HIGH : LOW);
    while (digitalRead(ENDSTOPS_PIN) == LOW) {
        digitalWrite(stepPin, HIGH); delayMicroseconds(200);
        digitalWrite(stepPin, LOW);  delayMicroseconds(200);
    }
    axisSteps = 0;
    digitalWrite(dirPin, (dirSign == 1) ? LOW : HIGH);
    for (long i = 0; i < pullOffSteps; i++) {
        digitalWrite(stepPin, HIGH); delayMicroseconds(100);
        digitalWrite(stepPin, LOW);  delayMicroseconds(100);
    }
    delay(200);
    return true;
}

void runFullHoming() {
    isVectorMoving = false; 
    if (!homeAxis(Z_STEP_PIN, Z_DIR_PIN, 1, currentStepsZ, 1200)) { isHomed = false; return; }
    if (!homeAxis(Y_STEP_PIN, Y_DIR_PIN, -1, currentStepsY, 1200)) { isHomed = false; return; }
    if (!homeAxis(X_STEP_PIN, X_DIR_PIN, -1, currentStepsX, 1200)) { isHomed = false; return; }
    isHomed = true; 
}

void IRAM_ATTR onTimerInterrupt() {
    if (!isVectorMoving) return;

    if (digitalRead(ESTOP_PIN) == HIGH || digitalRead(MOTORS_ALARM) == HIGH || digitalRead(ENDSTOPS_PIN) == HIGH) {
        digitalWrite(MOTORS_ENABLE, HIGH); 
        isVectorMoving = false;
        return;
    }

    if (vectorTicksCount < vSeg.accelTicks) {
        currentVectorSpeed += vSeg.accelRate * DT;
    } else if (vectorTicksCount > (totalVectorTicks - vSeg.decelTicks)) {
        currentVectorSpeed -= vSeg.accelRate * DT;
        if (currentVectorSpeed < vSeg.minTicksPerSec) currentVectorSpeed = vSeg.minTicksPerSec;
    }

    static float idealX = currentStepsX / cfg.stepsPerMmX;
    static float idealY = currentStepsY / cfg.stepsPerMmY;
    static float idealZ = currentStepsZ / cfg.stepsPerMmZ;

    float deltaS = currentVectorSpeed * DT; 
    idealX += deltaS * vSeg.Kx * vSeg.dirX;
    idealY += deltaS * vSeg.Ky * vSeg.dirY;
    idealZ += deltaS * vSeg.Kz * vSeg.dirZ;

    long targetStepsX = idealX * cfg.stepsPerMmX;
    CorrectionPoint corr = getCorrection(idealX);
    long targetStepsY = (idealY + corr.y_offset) * cfg.stepsPerMmY;
    long targetStepsZ = (idealZ + corr.z_offset) * cfg.stepsPerMmZ;

    bool stepIssued = false;
    if (currentStepsX != targetStepsX) {
        digitalWrite(X_DIR_PIN, (targetStepsX > currentStepsX) ? HIGH : LOW);
        digitalWrite(X_STEP_PIN, HIGH); currentStepsX += (targetStepsX > currentStepsX) ? 1 : -1;
        stepIssued = true;
    }
    if (currentStepsY != targetStepsY) {
        digitalWrite(Y_DIR_PIN, (targetStepsY > currentStepsY) ? HIGH : LOW);
        digitalWrite(Y_STEP_PIN, HIGH); currentStepsY += (targetStepsY > currentStepsY) ? 1 : -1;
        stepIssued = true;
    }
    if (currentStepsZ != targetStepsZ) {
        digitalWrite(Z_DIR_PIN, (targetStepsZ > currentStepsZ) ? HIGH : LOW);
        digitalWrite(Z_STEP_PIN, HIGH); currentStepsZ += (targetStepsZ > currentStepsZ) ? 1 : -1;
        stepIssued = true;
    }

    if (stepIssued) {
        delayMicroseconds(2);
        digitalWrite(X_STEP_PIN, LOW); digitalWrite(Y_STEP_PIN, LOW); digitalWrite(Z_STEP_PIN, LOW);
    }

    vectorTicksCount++;
    if (vectorTicksCount >= totalVectorTicks) {
        isVectorMoving = false;
        currentStepsX = targetX * cfg.stepsPerMmX;
        currentStepsY = (targetY + getCorrection(targetX).y_offset) * cfg.stepsPerMmY;
        currentStepsZ = (targetZ + getCorrection(targetX).z_offset) * cfg.stepsPerMmZ;
    }
}

void IRAM_ATTR onTimerInterrupt() {
    if (!isVectorMoving) return;

    // Контроль безопасности (оставляем)
    if (digitalRead(34) == HIGH || digitalRead(35) == HIGH || digitalRead(39) == HIGH) {
        digitalWrite(13, HIGH);
        return; 
    }

    // --- ОТКЛЮЧАЕМ ВСЮ МАТЕМАТИКУ FLOAT ДЛЯ ТЕСТА ---
    // --- ИМИТИРУЕМ ЦЕЛОЧИСЛЕННЫЙ ТЕСТОВЫЙ ПЕРЕЕЗД ---
    // Жестко пропишем целые числа шагов (например, эквивалент X=200, Y=50, Z=-5)
    long testTargetX = 200 * 100; // 20000 шагов
    long testTargetY = 50 * 400;  // 20000 шагов
    long testTargetZ = -5 * 400;  // -2000 шагов

    bool stepIssued = false;

    if (currentStepsX != testTargetX) {
        digitalWrite(26, (testTargetX > currentStepsX) ? HIGH : LOW);
        digitalWrite(25, HIGH);
        currentStepsX += (testTargetX > currentStepsX) ? 1 : -1;
        stepIssued = true;
    }
    if (currentStepsY != testTargetY) {
        digitalWrite(14, (testTargetY > currentStepsY) ? HIGH : LOW);
        digitalWrite(27, HIGH);
        currentStepsY += (testTargetY > currentStepsY) ? 1 : -1;
        stepIssued = true;
    }

    if (stepIssued) {
        delayMicroseconds(2); 
        digitalWrite(25, LOW);
        digitalWrite(27, LOW);
    }

    if (currentStepsX == testTargetX && currentStepsY == testTargetY) {
        isVectorMoving = false;
    }
}
*/