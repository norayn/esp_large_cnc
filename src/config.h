#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

struct MachineConfig {
    float stepsPerMmX;
    float stepsPerMmY;
    float stepsPerMmZ;

    float defaultFeedRate;  
    float rapidFeedRate;    
    float maxAcceleration;  
    float minVectorSpeed;   

    float laserGridStep;    
    int   laserMapSize;     

    int pwmFrequency;       
    int pwmResolutionBits;  

    float minX, maxX; 
    float minY, maxY; 
    float minZ, maxZ; 

    bool allowJogBeforeHoming; 

    float wcsOffsetX; 
    float wcsOffsetY;
    float wcsOffsetZ;

    bool  isAlignmentActive;
    float slopeY;
    float slopeZ;
    float pointA_x, pointA_y, pointA_z; // Сохраняем и сами опорные точки для контроля
    float pointB_x, pointB_y, pointB_z;

    int lastExecutedLine; // НОВОЕ: Сюда пишется номер строки для восстановления при сбое
    
    volatile float feedMultiplier; 
};

extern MachineConfig cfg;

extern volatile uint16_t currentExecutingLineNum; // Номер строки, которая пилится прямо сейчас

// Инициализация и чтение памяти (Вызывается один раз в main.cpp)
void setupAndLoadConfig(); 

// Сохранение текущих координат WCS (Вызывается из gcode_program при выставлении нуля)
void saveWcsToEEPROM();

void saveAlignmentToEEPROM();

// ЦЕНТРАЛЬНЫЙ ОБРАБОТЧИК НАСТРОЕК (Парсит и отвечает модулю связи)
String handleConfigCommand(String cmd);

#endif
