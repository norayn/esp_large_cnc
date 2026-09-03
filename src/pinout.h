#ifndef PINOUT_H
#define PINOUT_H

// Блок драйверов сервошагов
#define X_STEP_PIN     25  
#define X_DIR_PIN      26  
#define Y_STEP_PIN     27  
#define Y_DIR_PIN      14  
#define Z_STEP_PIN     32  
#define Z_DIR_PIN      33  
#define MOTORS_ENABLE  13  

// Блок безопасности (Входы)
#define ESTOP_PIN      34  
#define MOTORS_ALARM   35  
#define ENDSTOPS_PIN   39  

// Модуль лазерной коррекции (ESP32-CAM)
#define CAM_RX2_PIN    16  
#define CAM_TX2_PIN    17  

// Китайский цифровой индикатор
#define INDICATOR_CLK  18  
#define INDICATOR_DATA 19  

// Силовая периферия
#define SPINDLE_PWM    23  
#define RELE_SPINDLE   21  
#define RELE_COOLANT   22  

// Резерв ПК (USB)
#define PC_RX0_PIN      3  
#define PC_TX0_PIN      1  

#endif
