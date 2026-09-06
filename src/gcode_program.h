#ifndef GCODE_PROGRAM_H
#define GCODE_PROGRAM_H

#include <Arduino.h>

#define MAX_COMMANDS_BUFFER  3000 

// Компактная бинарная ЧПУ-команда (13 байт)
struct __attribute__((packed)) BinaryCommand {
    uint8_t  cmdType; // 0=G0, 1=G1, 3=M3, 5=M5, 8=M8, 9=M9
    uint16_t lineNum;  // НОВОЕ: Номер исходной строки G-кода из файла на ПК
    float    x;       // Координата в системе детали (WCS)
    float    y;
    float    z;
    float    f;       
};

// Смещения рабочей системы координат (WCS), например G54
struct WorkCoordinateSystem {
    float x;
    float y;
    float z;
};

// Глобальные переменные программы
extern BinaryCommand gcodeBuffer[MAX_COMMANDS_BUFFER];
extern int totalLoadedCommands;  
extern int currentCommandIndex;  
extern WorkCoordinateSystem wcsOffset; // Текущие смещения нуля детали

extern volatile bool isSingleBlockMode; // true = выполнять по 1 кадру и ждать команду "СЛЕДУЮЩИЙ"
extern volatile bool waitNextBlockTrigger; // флаг ожидания пинка от ПК

struct AlignmentPoint {
    float x;
    float y;
    float z;
};

extern AlignmentPoint pointA;
extern AlignmentPoint pointB;
extern bool isAlignmentActive; // Включено ли программное выравнивание по точкам A-B

// Интерфейс модуля
void initGCodeModule();
void clearGCodeBuffer();
bool addBinaryCommand(uint8_t type, uint16_t line, float x, float y, float z, float f);
void setWcsZero(char axis); // Обнуление осей детали ('X', 'Y', 'Z', 'A' - все)

void setAlignmentPoint(char pointLetter); // Запомнить текущую физическую позицию как точку А или В
void calculateAlignmentMatrix();          // Расчет коэффициентов наклона
void deactivateAlignmentExternal(); 

// ГЛАВНЫЙ ИСПОЛНИТЕЛЬ: Выдергивает команду из буфера и распределяет её по модулям
void executeNextProgramStep();

// Обработчик одиночных ручных команд (JOG)
void processSingleManualCommand(String line);

void startProgramExecution();

#endif
