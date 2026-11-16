#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include <Arduino.h>

enum MachineState {
    STATE_INIT,          
    STATE_ALARM,         
    STATE_IDLE,          
    STATE_JOGGING,       
    STATE_CALIBRATION,   
    STATE_MAP_TRANSFER,  
    STATE_RUNNING,       
    STATE_HOMING,        
    STATE_HOLD,
    STATE_GCODE_UPLOAD           
};

extern volatile bool isHomed;
extern volatile MachineState currentMachineState;

String getStateName();
// true = переход выполнен; false = запрещён (ALARM или нет хоуминга).
bool changeState(MachineState newState);

// Технологическая пауза и выход из неё. HOLD запоминает, откуда пришли
// (RUNNING или JOGGING), RESUME возвращает туда же: иначе пауза во время JOG
// продолжилась бы как RUNNING и запустила загруженную УП (P1-5).
bool enterHold();
bool resumeFromHold();
void checkHardwareSecurity();
void reportStatusToPC();

#endif
