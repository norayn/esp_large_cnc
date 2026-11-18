#ifndef GCODE_PROGRAM_H
#define GCODE_PROGRAM_H

#include <Arduino.h>

#define MAX_COMMANDS_BUFFER  2000 

// Компактная бинарная ЧПУ-команда (13 байт)
struct __attribute__((packed)) BinaryCommand {
    uint8_t cmdType;       // 0 = G0, 1 = G1, 3 = M3, 5 = M5 и т.д.
    uint16_t lineNum;   // Номер строки оригинального файла для телеметрии
    float x;            // Координата X (мм)
    float y;            // Координата Y (мм)
    float z;            // Координата Z (мм)
    float f;            // Подача кадра (мм/с — в таком виде приходит в пакете B:)
    float v_start;      // Скорость входа от Python Look-Ahead (мм/сек)
    float v_end;        // Скорость выхода на стыке от Python Look-Ahead (мм/сек)     
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

extern volatile bool isPlannerBusy;

// Интерфейс модуля
void initGCodeModule();
void clearGCodeBuffer();
bool addBinaryCommand(uint8_t type, uint16_t line, float x, float y, float z, float f, float v_start, float v_end);
void setWcsZero(char axis); // Обнуление осей детали ('X', 'Y', 'Z', 'A' - все)

void setAlignmentPoint(char pointLetter); // Запомнить текущую физическую позицию как точку А или В
void calculateAlignmentMatrix();          // Расчет коэффициентов наклона
void deactivateAlignmentExternal(); 

// ГЛАВНЫЙ ИСПОЛНИТЕЛЬ: выдергивает одну команду из буфера и распределяет её
// по модулям. true = команда исполнена и индекс продвинулся; false = двигаться
// дальше нельзя (программа кончилась, ждём Single Block, M-код ждёт слива
// движения или кадр отклонён).
bool executeNextProgramStep();

// Обработчик одиночных ручных команд (JOG). false = команда отклонена.
bool processSingleManualCommand(String line);

// P1-7: JOG режется по узлам карты провиса. isSplitActive() — нарезка ещё не
// закончена, continueManualJogSplit() ставит следующий кусок (вызывать из
// supervisorTask, пока FSM в JOGGING).
bool isSplitActive();
bool continueManualJogSplit();

// startIndex — индекс кадра в буфере, с которого начинается запуск
// (START_FROM_LINE). true = программа запущена; false = отказ (ALARM, нет
// хоуминга, пустой буфер или startIndex вне 0..totalLoadedCommands-1).
bool startProgramExecution(int startIndex = 0);

#endif
