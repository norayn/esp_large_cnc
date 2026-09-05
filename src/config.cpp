#include "config.h"
#include <Preferences.h>

MachineConfig cfg;
Preferences preferences;

// Вспомогательные перегруженные функции для Preferences (чтобы макрос работал с любыми типами)
void p_put(const char* key, float val) { preferences.putFloat(key, val); }
void p_put(const char* key, int val)   { preferences.putInt(key, val); }
void p_put(const char* key, bool val)  { preferences.putBool(key, val); }

float p_get(const char* key, float def) { return preferences.getFloat(key, def); }
int   p_get(const char* key, int def)   { return preferences.getInt(key, def); }
bool  p_get(const char* key, bool def)  { return preferences.getBool(key, def); }

void initMachineConfigDefault() {
#define X(type, name, eeprom_key, default_val) cfg.name = default_val;
    CONFIG_FIELDS
#undef X
}

void saveConfigToEEPROM() {
    preferences.begin("cnc_cfg", false);
#define X(type, name, eeprom_key, default_val) p_put(eeprom_key, cfg.name);
    CONFIG_FIELDS
#undef X
    preferences.end();
}

void setupAndLoadConfig() {
    preferences.begin("cnc_cfg", true);
    
    // Проверяем первый запуск по маркерному ключу базового параметра
    if (!preferences.isKey("stepsX")) {
        preferences.end();
        initMachineConfigDefault();
        saveConfigToEEPROM();
        return;
    }

#define X(type, name, eeprom_key, default_val) cfg.name = p_get(eeprom_key, (type)default_val);
    CONFIG_FIELDS
#undef X

    preferences.end();
}

void saveWcsToEEPROM() {
    preferences.begin("cnc_cfg", false);
    preferences.putFloat("wcsX", cfg.wcsOffsetX);
    preferences.putFloat("wcsY", cfg.wcsOffsetY);
    preferences.putFloat("wcsZ", cfg.wcsOffsetZ);
    preferences.end();
}

void saveAlignmentToEEPROM() {
    preferences.begin("cnc_cfg", false);
    preferences.putBool("alignAct", cfg.isAlignmentActive);
    preferences.putFloat("slopeY", cfg.slopeY);
    preferences.putFloat("slopeZ", cfg.slopeZ);
    preferences.putFloat("ptAx", cfg.pointA_x);
    preferences.putFloat("ptAy", cfg.pointA_y);
    preferences.putFloat("ptAz", cfg.pointA_z);
    preferences.putFloat("ptBx", cfg.pointB_x);
    preferences.putFloat("ptBy", cfg.pointB_y);
    preferences.putFloat("ptBz", cfg.pointB_z);
    preferences.end();
}

// АВТОМАТИЧЕСКАЯ СЕРИАЛИЗАЦИЯ И ДЕСЕРИАЛИЗАЦИЯ ДЛЯ ПК
String handleConfigCommand(String cmd) {
    if (cmd == "GET_CONFIG") {
        String cStr = "CONFIG_DATA:";
        // Макрос сам соберет строку вида "ИМЯ=ЗНАЧЕНИЕ;ИМЯ=ЗНАЧЕНИЕ;" для ВСЕХ параметров
#define X(type, name, eeprom_key, default_val) cStr += String(#name) + "=" + String(cfg.name) + ";";
        CONFIG_FIELDS
#undef X
        return cStr;
    }
    
    if (cmd.startsWith("SET_CONFIG:")) {
        String pair = cmd.substring(11); // получаем "имя_в_коде=значение"
        int eq = pair.indexOf('=');
        if (eq != -1) {
            String targetKey = pair.substring(0, eq);
            String valStr = pair.substring(eq + 1);
            bool found = false;

            // Сравниваем имя со всеми полями из макроса автоматически
#define X(type, name, eeprom_key, default_val) \
            if (targetKey == #name) { \
                if (sizeof(type) == sizeof(bool)) cfg.name = (valStr.toInt() != 0); \
                else if (String(#type) == "int") cfg.name = valStr.toInt(); \
                else cfg.name = valStr.toFloat(); \
                found = true; \
            }
            CONFIG_FIELDS
#undef X

            if (found) {
                saveConfigToEEPROM(); // сохраняем обновленный конфиг
                return "STATUS: Config parameter " + targetKey + " updated and saved.";
            } else {
                return "ERROR: Config parameter " + targetKey + " not found!";
            }
        }
    }
    return "";
}
