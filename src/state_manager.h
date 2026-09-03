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
    STATE_HOLD           
};

extern volatile bool isHomed;
extern volatile MachineState currentMachineState;

String getStateName();
void changeState(MachineState newState);
void checkHardwareSecurity();
void reportStatusToPC();

#endif
