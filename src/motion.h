#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>

struct CorrectionPoint {
    float y_offset;
    float z_offset;
};

struct VectorSegment {
    float totalLength_mm; 
    float Kx, Ky, Kz;     
    int dirX, dirY, dirZ; 
    
    long accelTicks;      
    long decelTicks;      
    float maxTicksPerSec; 
    float minTicksPerSec; 
    float accelRate;      
};
struct InputSegment {
    float x, y, z, f, a;
    volatile bool hasNewData;
};

extern volatile long currentStepsX;
extern volatile long currentStepsY;
extern volatile long currentStepsZ;
extern volatile bool isVectorMoving;
extern volatile long plannerStepsX;
extern volatile long plannerStepsY;
extern volatile long plannerStepsZ;

extern float pythonFeedrateOverride; // Переменная множителя скорости от ползунка ПК

extern volatile uint32_t bufferHead;
extern volatile uint32_t bufferTail;
extern volatile InputSegment nextSeg; 

void initMotion();
void prepareVectorSegment(float newX, float newY, float newZ, float feedRateMM_Min, float accelMM_Sec2);
void updateCurvaturePoint(int index, float y_off, float z_off);
CorrectionPoint getCorrection(float currentX);
bool homeAxis(int stepPin, int dirPin, int dirSign, volatile long &axisSteps, long pullOffSteps);
void runFullHoming();
void IRAM_ATTR onTimerInterrupt();

void updateFeedrateFading();

void startMotionTask(); 

#endif
