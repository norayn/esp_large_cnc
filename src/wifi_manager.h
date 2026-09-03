#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>

void initWiFiManager();
void updateWiFiCommunication();
void sendToWiFiClient(String message);

String processIncomingCommand(String cmd);

extern bool isWiFiClientConnected;

#endif
