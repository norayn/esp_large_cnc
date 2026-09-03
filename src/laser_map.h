#ifndef LASER_MAP_H
#define LASER_MAP_H

#include <Arduino.h>

void initLaserMap();
void updateLaserMapCommunication();
void runAutoCalibration();
void exportMapToPC();
void parseMapRowFromPC(String pcLine);

#endif
