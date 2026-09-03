#include "config.h"
#include <Preferences.h>

MachineConfig cfg;
Preferences preferences;

void initMachineConfigDefault() {
    cfg.stepsPerMmX = 100.0; 
    cfg.stepsPerMmY = 400.0; 
    cfg.stepsPerMmZ = 400.0; 
    cfg.defaultFeedRate = 600.0;  
    cfg.rapidFeedRate   = 2000.0; 
    cfg.maxAcceleration = 150.0;  
    cfg.minVectorSpeed  = 2.0;    
    cfg.laserGridStep   = 20.0;   
    cfg.laserMapSize    = 201;    
    cfg.pwmFrequency      = 5000; 
    cfg.pwmResolutionBits = 10;   
    cfg.minX = 0.0;    cfg.maxX = 4000.0; 
    cfg.minY = 0.0;    cfg.maxY = 500.0;  
    cfg.minZ = -200.0; cfg.maxZ = 0.0;    
    cfg.allowJogBeforeHoming = true; 
    cfg.wcsOffsetX = 0.0;
    cfg.wcsOffsetY = 0.0;
    cfg.wcsOffsetZ = 0.0;
    cfg.isAlignmentActive = false;
    cfg.slopeY = 0.0f;
    cfg.slopeZ = 0.0f;
    cfg.pointA_x = 0; cfg.pointA_y = 0; cfg.pointA_z = 0;
    cfg.pointB_x = 0; cfg.pointB_y = 0; cfg.pointB_z = 0;
    cfg.feedMultiplier = 1.0f;
}

void saveConfigToEEPROM() {
    preferences.begin("cnc_cfg", false);
    preferences.putFloat("stepsX", cfg.stepsPerMmX);
    preferences.putFloat("stepsY", cfg.stepsPerMmY);
    preferences.putFloat("stepsZ", cfg.stepsPerMmZ);
    preferences.putFloat("defFeed", cfg.defaultFeedRate);
    preferences.putFloat("rapFeed", cfg.rapidFeedRate);
    preferences.putFloat("accel", cfg.maxAcceleration);
    preferences.putFloat("minSpeed", cfg.minVectorSpeed);
    preferences.putFloat("lGrid", cfg.laserGridStep);
    preferences.putInt("lSize", cfg.laserMapSize);
    preferences.putInt("pwmFreq", cfg.pwmFrequency);
    preferences.putInt("pwmRes", cfg.pwmResolutionBits);
    preferences.putFloat("minX", cfg.minX);
    preferences.putFloat("maxX", cfg.maxX);
    preferences.putFloat("minY", cfg.minY);
    preferences.putFloat("maxY", cfg.maxY);
    preferences.putFloat("minZ", cfg.minZ);
    preferences.putFloat("maxZ", cfg.maxZ);
    preferences.putBool("jogBefore", cfg.allowJogBeforeHoming);
    // wcs нули сохраняются отдельно через saveWcsToEEPROM
    preferences.end();
}

void setupAndLoadConfig() {
    preferences.begin("cnc_cfg", true);
    
    if (!preferences.isKey("stepsX")) {
        preferences.end();
        initMachineConfigDefault();
        saveConfigToEEPROM();
        saveWcsToEEPROM();
        return;
    }

    cfg.stepsPerMmX = preferences.getFloat("stepsX", 100.0);
    cfg.stepsPerMmY = preferences.getFloat("stepsY", 400.0);
    cfg.stepsPerMmZ = preferences.getFloat("stepsZ", 400.0);
    cfg.defaultFeedRate = preferences.getFloat("defFeed", 600.0);
    cfg.rapidFeedRate   = preferences.getFloat("rapFeed", 2000.0);
    cfg.maxAcceleration = preferences.getFloat("accel", 150.0);
    cfg.minVectorSpeed  = preferences.getFloat("minSpeed", 2.0);
    cfg.laserGridStep   = preferences.getFloat("lGrid", 20.0);
    cfg.laserMapSize    = preferences.getInt("lSize", 201);
    cfg.pwmFrequency    = preferences.getInt("pwmFreq", 5000);
    cfg.pwmResolutionBits = preferences.getInt("pwmRes", 10);
    cfg.minX = preferences.getFloat("minX", 0.0);
    cfg.maxX = preferences.getFloat("maxX", 4000.0);
    cfg.minY = preferences.getFloat("minY", 0.0);
    cfg.maxY = preferences.getFloat("maxY", 500.0);
    cfg.minZ = preferences.getFloat("minZ", -200.0);
    cfg.maxZ = preferences.getFloat("maxZ", 0.0);
    cfg.allowJogBeforeHoming = preferences.getBool("jogBefore", true);
    
    // Загрузка временных рабочих нулей детали G54 из памяти
    cfg.wcsOffsetX = preferences.getFloat("wcsX", 0.0);
    cfg.wcsOffsetY = preferences.getFloat("wcsY", 0.0);
    cfg.wcsOffsetZ = preferences.getFloat("wcsZ", 0.0);

    cfg.isAlignmentActive = preferences.getBool("alignAct", false);
    cfg.slopeY = preferences.getFloat("slopeY", 0.0f);
    cfg.slopeZ = preferences.getFloat("slopeZ", 0.0f);
    cfg.pointA_x = preferences.getFloat("ptAx", 0.0f);
    cfg.pointA_y = preferences.getFloat("ptAy", 0.0f);
    cfg.pointA_z = preferences.getFloat("ptAz", 0.0f);
    cfg.pointB_x = preferences.getFloat("ptBx", 0.0f);
    cfg.pointB_y = preferences.getFloat("ptBy", 0.0f);
    cfg.pointB_z = preferences.getFloat("ptBz", 0.0f);
    
    cfg.lastExecutedLine = preferences.getInt("lastLine", 0);

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

String handleConfigCommand(String cmd) {
    if (cmd == "GET_CONFIG") {
        String cStr = "CONFIG_DATA:";
        cStr += "stepsX=" + String(cfg.stepsPerMmX, 1) + ";";
        cStr += "stepsY=" + String(cfg.stepsPerMmY, 1) + ";";
        cStr += "stepsZ=" + String(cfg.stepsPerMmZ, 1) + ";";
        cStr += "accel=" + String(cfg.maxAcceleration, 1) + ";";
        cStr += "rapFeed=" + String(cfg.rapidFeedRate, 1) + ";";
        cStr += "maxX=" + String(cfg.maxX, 1) + ";";
        cStr += "maxY=" + String(cfg.maxY, 1) + ";";
        cStr += "maxZ=" + String(cfg.maxZ, 1);
        return cStr;
    }
    
    if (cmd.startsWith("SET_CONFIG:")) {
        String pair = cmd.substring(11);
        int eq = pair.indexOf('=');
        if (eq != -1) {
            String key = pair.substring(0, eq);
            float val = pair.substring(eq + 1).toFloat();
            bool changed = true;
            
            if (key == "stepsX")  cfg.stepsPerMmX = val;
            else if (key == "stepsY")  cfg.stepsPerMmY = val;
            else if (key == "stepsZ")  cfg.stepsPerMmZ = val;
            else if (key == "accel")    cfg.maxAcceleration = val;
            else if (key == "rapFeed")  cfg.rapidFeedRate = val;
            else if (key == "maxX")     cfg.maxX = val;
            else if (key == "maxY")     cfg.maxY = val;
            else if (key == "maxZ")     cfg.maxZ = val;
            else changed = false;

            if (changed) {
                saveConfigToEEPROM();
                return "STATUS: Config updated and saved to EEPROM.";
            } else {
                return "ERROR: Unknown config key!";
            }
        }
    }
    return "";
}

