#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>

void initWiFiManager();
void updateWiFiCommunication();

// Только TCP-клиенту GUI. В USB строку печатает тот, кому она нужна, — иначе
// вывод задваивается (P2-6).
void sendToWiFiClient(String message);

// Одна строка в TCP и одна в USB: телеметрия и ответы на команды.
void sendToBothClients(String message);

String processIncomingCommand(String cmd);

extern bool isWiFiClientConnected;

#endif
