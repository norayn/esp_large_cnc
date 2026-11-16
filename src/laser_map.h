#ifndef LASER_MAP_H
#define LASER_MAP_H

#include <Arduino.h>

void initLaserMap();

// Единственный читатель Serial2: разбирает ответы камеры OFFSET:Y=..;Z=.. и
// складывает их в lastCam*. Живого контура нет — ответ забирает только скан
// или калибровка (takeCamResponse), на движение во время реза он не влияет.
void updateLaserMapCommunication();
void exportMapToPC();
void parseMapRowFromPC(String pcLine);

// Забирает последний разобранный ответ камеры, пришедший не раньше sinceMs.
// true = свежий OFFSET:Y=..;Z=.. забран (повторно не отдаётся).
// Раньше ответ разбирался и выбрасывался, а сканы читали Serial2 сами.
bool takeCamResponse(unsigned long sinceMs, float &y_off, float &z_off);

#endif
