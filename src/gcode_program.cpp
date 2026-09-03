#include "gcode_program.h"
#include "motion.h"
#include "config.h"
#include "pinout.h"
#include "state_manager.h"

BinaryCommand* gcodeBuffer = NULL;
int totalLoadedCommands = 0;
int currentCommandIndex = 0;
WorkCoordinateSystem wcsOffset = {0.0, 0.0, 0.0};

volatile bool isSingleBlockMode = false;
volatile bool waitNextBlockTrigger = false;
volatile uint16_t currentExecutingLineNum = 0;

AlignmentPoint pointA = {0, 0, 0};
AlignmentPoint pointB = {0, 0, 0};
bool isAlignmentActive = false;

// Коэффициенты наклона детали (приращение Y и Z на 1 мм хода по X)
static float slopeY = 0.0;
static float slopeZ = 0.0;

void initGCodeModule() {
    gcodeBuffer = (BinaryCommand*)malloc(MAX_COMMANDS_BUFFER * sizeof(BinaryCommand));
    clearGCodeBuffer();

    // Восстанавливаем сохраненные из EEPROM нули детали G54
    wcsOffset.x = cfg.wcsOffsetX;
    wcsOffset.y = cfg.wcsOffsetY;
    wcsOffset.z = cfg.wcsOffsetZ;

    // ВОССТАНАВЛИВАЕМ МАТРИЦУ НАКЛОНА ИЗ ПАМЯТИ ЧИПА
    isAlignmentActive = cfg.isAlignmentActive;
    slopeY = cfg.slopeY;
    slopeZ = cfg.slopeZ;
    pointA = {cfg.pointA_x, cfg.pointA_y, cfg.pointA_z};
    pointB = {cfg.pointB_x, cfg.pointB_y, cfg.pointB_z};
}

void clearGCodeBuffer() {
    totalLoadedCommands = 0;
    currentCommandIndex = 0;
    if (gcodeBuffer != NULL) {
        memset(gcodeBuffer, 0, MAX_COMMANDS_BUFFER * sizeof(BinaryCommand));
    }
}

bool addBinaryCommand(uint8_t type, uint16_t line, float x, float y, float z, float f) {
    if (gcodeBuffer == NULL || totalLoadedCommands >= MAX_COMMANDS_BUFFER) return false;
    gcodeBuffer[totalLoadedCommands] = { type, line, x, y, z, f };
    totalLoadedCommands++;
    return true;
}


void setWcsZero(char axis) {
    // Рассчитываем смещение: WCS_Offset = Текущая машинная позиция в мм
    if (axis == 'X' || axis == 'A') wcsOffset.x = -(currentStepsX / cfg.stepsPerMmX);
    if (axis == 'Y' || axis == 'A') wcsOffset.y = -(currentStepsY / cfg.stepsPerMmY);
    if (axis == 'Z' || axis == 'A') wcsOffset.z = -(currentStepsZ / cfg.stepsPerMmZ);
    
    // Синхронизируем глобальный конфиг и сохраняем его во флэш
    cfg.wcsOffsetX = wcsOffset.x;
    cfg.wcsOffsetY = wcsOffset.y;
    cfg.wcsOffsetZ = wcsOffset.z;
    saveWcsToEEPROM();
}

// ЦЕНТРАЛЬНЫЙ ДИСПЕТЧЕР КОМАНД
void executeNextProgramStep() {
    // 1. Проверка покадрового режима (Single Block)
    if (isSingleBlockMode && waitNextBlockTrigger) {
        // Зависаем в этом состоянии, Ядро 1 крутит удержание, моторы стоят.
        // Выход из условия произойдет, когда Wi-Fi модуль сбросит waitNextBlockTrigger в false
        return; 
    }

    if (currentCommandIndex >= totalLoadedCommands) {
        isVectorMoving = false;
        changeState(STATE_IDLE);
        return; 
    }

    BinaryCommand cmd = gcodeBuffer[currentCommandIndex];
    currentCommandIndex++; 
    currentExecutingLineNum = cmd.lineNum; 
    cfg.lastExecutedLine = cmd.lineNum;

    if (cmd.cmdType == 0 || cmd.cmdType == 1) {
        // Базовый перевод рабочей координаты детали (WCS G54) в машинную координату станины
        float machineX = cmd.x + wcsOffset.x;
        float machineY = cmd.y + wcsOffset.y;
        float machineZ = cmd.z + wcsOffset.z;

        // --- МАТЕМАТИКА КОРРЕКЦИИ НАКЛОНА ЗАГОТОВКИ ---
        if (isAlignmentActive) {
            // Находим, насколько далеко мы уехали от опорной Точки А по оси X
            float deltaX_from_A = machineX - pointA.x;
            
            // Динамически подмешиваем компенсацию непараллельности станины к детали
            machineY += deltaX_from_A * slopeY;
            machineZ += deltaX_from_A * slopeZ;
        }

        // Отправляем результирующий чистый вектор в физический планировщик motion
        prepareVectorSegment(machineX, machineY, machineZ, cmd.f, cfg.maxAcceleration);

        // Если включен покадровый режим, взводим флаг ожидания для СЛЕДУЮЩЕГО кадра
        if (isSingleBlockMode) {
            waitNextBlockTrigger = true;
            Serial.printf("STATUS: Block %d executed. Waiting next block trigger...\n", currentCommandIndex - 1);
            // Шлем в сокет оповещение для Python, чтобы кнопка "След. Кадр" стала активной
            extern void sendToWiFiClient(String message);
            sendToWiFiClient("SINGLE_BLOCK_WAIT");
        }
    } 
    else {
        // Исполнение М-кодов
        switch (cmd.cmdType) {
            case 3: digitalWrite(RELE_SPINDLE, HIGH); break;
            case 5: digitalWrite(RELE_SPINDLE, LOW);  break;
            case 8: digitalWrite(RELE_COOLANT, HIGH); break;
            case 9: digitalWrite(RELE_COOLANT, LOW);  break;
            default: break;
        }
        // М-коды пролетают мгновенно, Single Block на них не вешаем, запрашиваем следующий шаг
        executeNextProgramStep(); 
    }
}

void processSingleManualCommand(String line) {
    line.toUpperCase();
    uint8_t type = 255;
    if (line.indexOf("G0") != -1) type = 0;
    if (line.indexOf("G1") != -1) type = 1;
    
    // Вычисляем целевую позицию JOG перемещения (если координата пропущена — остаемся на месте в WCS)
    float x = (line.indexOf('X') != -1) ? line.substring(line.indexOf('X')+1).toFloat() : (currentStepsX / cfg.stepsPerMmX) - wcsOffset.x;
    float y = (line.indexOf('Y') != -1) ? line.substring(line.indexOf('Y')+1).toFloat() : (currentStepsY / cfg.stepsPerMmY) - wcsOffset.y;
    float z = (line.indexOf('Z') != -1) ? line.substring(line.indexOf('Z')+1).toFloat() : (currentStepsZ / cfg.stepsPerMmZ) - wcsOffset.z;
    float f = (line.indexOf('F') != -1) ? line.substring(line.indexOf('F')+1).toFloat() : cfg.defaultFeedRate;

    if (type == 0 || type == 1) {
        float machineX = x + wcsOffset.x;
        float machineY = y + wcsOffset.y;
        float machineZ = z + wcsOffset.z;
        prepareVectorSegment(machineX, machineY, machineZ, f, cfg.maxAcceleration);
    }
}

void setAlignmentPoint(char pointLetter) {
    float currentX = currentStepsX / cfg.stepsPerMmX;
    float currentY = currentStepsY / cfg.stepsPerMmY;
    float currentZ = currentStepsZ / cfg.stepsPerMmZ;

    if (pointLetter == 'A') {
        pointA = {currentX, currentY, currentZ};
        // Пишем в зеркало конфига
        cfg.pointA_x = currentX; cfg.pointA_y = currentY; cfg.pointA_z = currentZ;
        saveAlignmentToEEPROM();
        Serial.printf("STATUS: Point A set at X=%.2f Y=%.2f Z=%.2f\n", currentX, currentY, currentZ);
    } else if (pointLetter == 'B') {
        pointB = {currentX, currentY, currentZ};
        cfg.pointB_x = currentX; cfg.pointB_y = currentY; cfg.pointB_z = currentZ;
        saveAlignmentToEEPROM();
        Serial.printf("STATUS: Point B set at X=%.2f Y=%.2f Z=%.2f\n", currentX, currentY, currentZ);
    }
}

void calculateAlignmentMatrix() {
    float deltaX = pointB.x - pointA.x;
    if (abs(deltaX) < 10.0f) {
        isAlignmentActive = false;
        cfg.isAlignmentActive = false;
        saveAlignmentToEEPROM();
        Serial.println("ERROR: Alignment failed. Points A and B are too close!");
        return;
    }

    slopeY = (pointB.y - pointA.y) / deltaX;
    slopeZ = (pointB.z - pointA.z) / deltaX;
    isAlignmentActive = true;

    // Сохраняем всё в энергонезависимую память через центральный конфиг
    cfg.slopeY = slopeY;
    cfg.slopeZ = slopeZ;
    cfg.isAlignmentActive = true;
    saveAlignmentToEEPROM();

    Serial.printf("STATUS: Bed Alignment Saved to EEPROM. SlopeY=%.5f, SlopeZ=%.5f\n", slopeY, slopeZ);
}

void deactivateAlignmentExternal() {
    isAlignmentActive = false;
    cfg.isAlignmentActive = false;
    saveAlignmentToEEPROM();
}