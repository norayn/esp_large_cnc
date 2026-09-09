#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================================
// ЦЕНТРАЛЬНЫЙ ОПИСАТЕЛЬ КОНФИГУРАЦИИ (ДОБАВЛЯТЬ НОВЫЕ НАСТРОЙКИ СТРОГО СЮДА!)
// Формат: X(тип, имя_в_коде, ключ_eeprom, дефолт)
// ============================================================================
#define CONFIG_FIELDS \
    X(float, stepsPerMmX,      "stepsX",    100.0f)  \
    X(float, stepsPerMmY,      "stepsY",    400.0f)  \
    X(float, stepsPerMmZ,      "stepsZ",    400.0f)  \
    X(float, defaultFeedRate,   "defFeed",   600.0f)  \
    X(float, rapidFeedRate,     "rapFeed",   2000.0f) \
    X(float, maxAcceleration,   "accel",     150.0f)  \
    X(float, minVectorSpeed,    "minSpeed",  2.0f)    \
    X(float, laserGridStep,     "lGrid",     20.0f)   \
    X(int,   laserMapSize,      "lSize",     201)     \
    X(int,   pwmFrequency,      "pwmFreq",   5000)    \
    X(int,   pwmResolutionBits, "pwmRes",    10)      \
    X(float, minX,              "minX",      0.0f)    \
    X(float, maxX,              "maxX",      4000.0f) \
    X(float, minY,              "minY",      0.0f)    \
    X(float, maxY,              "maxY",      500.0f)  \
    \
    X(float, minZ,              "minZ",      -200.0f) \
    X(float, maxZ,              "maxZ",      0.0f)    \
    X(bool,  allowJogBeforeHoming,"jogBefore",true)   \
    X(float, feedMultiplier,    "feedMult",  1.0f)    \
    X(bool,  isAlignmentActive, "alignAct",  false)   \
    X(float, slopeY,            "slopeY",    0.0f)    \
    X(float, slopeZ,            "slopeZ",    0.0f)    \
    X(float, pointA_x,          "ptAx",      0.0f)    \
    X(float, pointA_y,          "ptAy",      0.0f)    \
    X(float, pointA_z,          "ptAz",      0.0f)    \
    X(float, pointB_x,          "ptBx",      0.0f)    \
    X(float, pointB_y,          "ptBy",      0.0f)    \
    X(float, pointB_z,          "ptBz",      0.0f)    \
    \
    X(float, wcsOffsetX,        "wcsX",      0.0f)    \
    X(float, wcsOffsetY,        "wcsY",      0.0f)    \
    X(float, wcsOffsetZ,        "wcsZ",      0.0f)    \
    X(int,   lastExecutedLine,  "lastLine",  0)       \
    \
    X(int,   statusInterval,    "statInt",   1000)    \
    X(bool,  sendOnlyOnChange,  "sendChg",   false)   \
    \
    X(int,   homingDirX,        "homDirX",  -1)       \
    X(int,   homingDirY,        "homDirY",  -1)       \
    X(int,   homingDirZ,        "homDirZ",  1)        \
    X(int,   pullOffX,          "pullOffX",  400)     \
    X(int,   pullOffY,          "pullOffY",  800)     \
    X(int,   pullOffZ,          "pullOffZ",  800)

// Автоматическая генерация структуры MachineConfig на основе макроса
struct MachineConfig {
#define X(type, name, eeprom_key, default_val) type name;
    CONFIG_FIELDS
#undef X
};

extern MachineConfig cfg;

void setupAndLoadConfig(); 
void saveWcsToEEPROM();
void saveAlignmentToEEPROM();
void saveConfigToEEPROM();
String handleConfigCommand(String cmd);

#endif
