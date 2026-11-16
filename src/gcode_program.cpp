#include "gcode_program.h"
#include "motion.h"
#include "config.h"
#include "pinout.h"
#include "state_manager.h"
#include "wifi_manager.h"

BinaryCommand gcodeBuffer[MAX_COMMANDS_BUFFER];
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

// ============================================================================
// P1-7: НАРЕЗКА КАДРА ПО УЗЛАМ КАРТЫ ПРОВИСА
// ============================================================================
// Карта задана точками X = i * laserGridStep, между ними она линейна, а
// motionTask берёт поправку Y/Z только на конце кадра. На длинном кадре
// (например, G0 на 3 м) изгибы карты внутри кадра теряются, поэтому кадр
// режется на куски не длиннее одного интервала сетки: тогда линейная
// интерполяция поправки совпадает с картой.
//
// Нарезка живёт в фидере: один вызов executeNextProgramStep()/continuation
// ставит один кусок. supervisorTask не блокируется на весь длинный ход,
// телеметрия и STOP/ESTOP продолжают работать.
struct SplitSegment {
    bool active;
    float fromX, fromY, fromZ;   // текущая точка нарезки (машинные координаты)
    float toX, toY, toZ;         // конец исходного кадра
    float feed, accel;
    float vEnd;                  // скорость выхода исходного кадра
    float entry;                 // скорость на текущей точке нарезки
};
static SplitSegment split = {false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static void resetSplit() { split.active = false; }

// Сетка мельче 0,1 мм не даёт точности (механика и карта грубее), зато дала бы
// десятки тысяч кадров на один ход. Такую карту не нарезаем.
static const float SPLIT_MIN_GRID_MM = 0.1f;

bool isSplitActive() { return split.active; }

// Ставит в очередь очередной кусок и сдвигает начало нарезки. false — кадр
// отклонён (STOP/ALARM/лимит/физика); причина в *err, если она не пустая.
static bool queueSplitPiece(bool &isLast, String *err) {
    const float grid = cfg.laserGridStep;
    const float span = split.toX - split.fromX;

    // Ближайший узел карты строго внутри кадра; если его нет — кусок до конца.
    float nextX = split.toX;
    if (cfg.laserMapSize > 1 && isfinite(grid) && grid >= SPLIT_MIN_GRID_MM && span != 0.0f) {
        // node > fromX (node < fromX при ходе назад) отсекает вырожденный
        // кусок нулевой длины от ошибки округления float.
        if (span > 0.0f) {
            float node = (floorf(split.fromX / grid) + 1.0f) * grid;
            if (node > split.fromX && node < split.toX) nextX = node;
        } else {
            float node = (ceilf(split.fromX / grid) - 1.0f) * grid;
            if (node < split.fromX && node > split.toX) nextX = node;
        }
    }

    isLast = (nextX == split.toX);
    const float t = (span != 0.0f) ? (nextX - split.fromX) / span : 1.0f;
    const float nextY = split.fromY + (split.toY - split.fromY) * t;
    const float nextZ = split.fromZ + (split.toZ - split.fromZ) * t;

    const float dx = nextX - split.fromX, dy = nextY - split.fromY, dz = nextZ - split.fromZ;
    const float pieceLen = sqrtf(dx * dx + dy * dy + dz * dz);
    const float rx = split.toX - nextX, ry = split.toY - nextY, rz = split.toZ - nextZ;
    const float restLen = sqrtf(rx * rx + ry * ry + rz * rz);

    // Куски коллинеарны, углов нет: ограничивает только разгон/торможение.
    // Скорость стыка должна быть физически достижима — иначе подготовка кадра
    // поднимет её клампом и на стыке кусков будет рывок. Поэтому:
    //   v_in → v_out за длину куска (разгон ≤ a), и от v_out ещё нужно успеть
    //   затормозить до v_end на остатке кадра.
    const float vIn = split.entry;
    float vOut = split.vEnd;
    if (!isLast) {
        vOut = split.feed;
        const float vReach = sqrtf(vIn * vIn + 2.0f * split.accel * pieceLen);
        if (vReach < vOut) vOut = vReach;
        const float vBrake = sqrtf(split.vEnd * split.vEnd + 2.0f * split.accel * restLen);
        if (vBrake < vOut) vOut = vBrake;
    }

    if (!prepareVectorSegment(nextX, nextY, nextZ, split.feed, split.accel,
                              vIn, vOut, err)) {
        return false;
    }

    split.fromX = nextX;
    split.fromY = nextY;
    split.fromZ = nextZ;
    split.entry = vOut;
    if (isLast) split.active = false;
    return true;
}

// Готовит нарезку кадра от текущей базы планировщика до (toX, toY, toZ).
static void beginSplit(float toX, float toY, float toZ, float feed, float accel,
                       float vStart, float vEnd) {
    getPlannerPositionMm(split.fromX, split.fromY, split.fromZ);
    split.toX = toX;
    split.toY = toY;
    split.toZ = toZ;
    split.feed = feed;
    split.accel = accel;
    split.vEnd = vEnd;
    split.entry = vStart;
    split.active = true;
}

// Продолжение нарезки JOG: вызывается из supervisorTask, пока FSM в JOGGING.
// true = поставлен ещё один кусок, нарезка продолжается.
bool continueManualJogSplit() {
    if (!split.active) return false;

    String err;
    bool isLast = false;
    if (!queueSplitPiece(isLast, &err)) {
        resetSplit();
        if (err.length() > 0) {
            Serial.println(err);
            sendToWiFiClient(err);
            changeState(STATE_IDLE);
        }
        return false;
    }
    return split.active;
}

void initGCodeModule() {
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

    memset(gcodeBuffer, 0, sizeof(gcodeBuffer));
    split.active = false;
}


bool addBinaryCommand(uint8_t type, uint16_t line, float x, float y, float z, float f, float v_start, float v_end) {
    if (totalLoadedCommands >= MAX_COMMANDS_BUFFER) return false;
    
    gcodeBuffer[totalLoadedCommands] = { type, line, x, y, z, f, v_start, v_end };
    totalLoadedCommands++;
    return true;
}


void setWcsZero(char axis) {
    // Вычисляем текущую физическую координату станка в миллиметрах
    float machineX = (float)currentStepsX / cfg.stepsPerMmX;
    float machineY = (float)currentStepsY / cfg.stepsPerMmY;
    float machineZ = (float)currentStepsZ / cfg.stepsPerMmZ;

    // Смещение WCS (оффсет) должно быть в точности равно машинной позиции!
    if (axis == 'X' || axis == 'A') cfg.wcsOffsetX = machineX;
    if (axis == 'Y' || axis == 'A') cfg.wcsOffsetY = machineY;
    if (axis == 'Z' || axis == 'A') cfg.wcsOffsetZ = machineZ;
    
    // Синхронизируем локальные переменные оффсетов (если они используются в gcode_program)
    wcsOffset.x = cfg.wcsOffsetX;
    wcsOffset.y = cfg.wcsOffsetY;
    wcsOffset.z = cfg.wcsOffsetZ;

    // Намертво сохраняем смещение G54 в энергонезависимую память Preferences
    saveWcsToEEPROM(); 
    Serial.printf("STATUS: WCS Zero set. Offsets: X=%.3f, Y=%.3f, Z=%.3f\n", cfg.wcsOffsetX, cfg.wcsOffsetY, cfg.wcsOffsetZ);
}

// ЦЕНТРАЛЬНЫЙ ДИСПЕТЧЕР КОМАНД
// Единый хвост после постановки кадра движения: покадровый режим ждёт
// триггера, но только когда кадр (или вся его нарезка) уже в очереди.
static void afterMotionFrameQueued() {
    if (!isSingleBlockMode) return;

    waitNextBlockTrigger = true;
    Serial.printf("STATUS: Block %d executed. Waiting next block trigger...\n", currentCommandIndex - 1);
    // Шлем в сокет оповещение для Python, чтобы кнопка «След. Кадр» стала активной
    sendToWiFiClient("SINGLE_BLOCK_WAIT");
}

// P1-3: вызывается в цикле из supervisorTask, пока в очереди кадров есть место.
// Возвращает true, если команда исполнена (индекс сдвинулся): тогда можно
// немедленно взять следующую и держать конвейер заполненным.
bool executeNextProgramStep() {
    // 1. Проверка покадрового режима (Single Block)
    if (isSingleBlockMode && waitNextBlockTrigger) {
        // Зависаем в этом состоянии, Ядро 1 крутит удержание, моторы стоят.
        // Выход из условия произойдет, когда Wi-Fi модуль сбросит waitNextBlockTrigger в false
        return false;
    }

    // 2. P1-7: доделываем нарезку предыдущего кадра. Индекс УП уже сдвинут,
    // поэтому проверка «программа кончилась» идёт после нарезки.
    if (split.active) {
        String err;
        bool isLast = false;
        if (!queueSplitPiece(isLast, &err)) {
            resetSplit();
            if (err.length() > 0) {
                Serial.println(err);
                sendToWiFiClient(err);
                changeState(STATE_IDLE);
            }
            return false;
        }
        if (isLast) afterMotionFrameQueued();
        return true;
    }

    // Если все кадры из статического ОЗУ-буфера полностью закончились
    if (currentCommandIndex >= totalLoadedCommands) {
        // Ждем, пока физическое движение полностью прекратится на столе.
        // Из HOLD программу не закрываем: пауза не должна сама сниматься.
        if (!isPlannerBusy) {
            if (currentMachineState == STATE_RUNNING) {
                changeState(STATE_IDLE);
                Serial.println("STATUS: Program finished successfully.");
            }
        }
        return false;
    }

    BinaryCommand cmd = gcodeBuffer[currentCommandIndex];
    const bool isMotion = (cmd.cmdType == 0 || cmd.cmdType == 1);

    // 3. P1-3: неподвижные команды (M3/M5/M8/M9) — барьер конвейера.
    // Класть их в очередь кадров нельзя: реле обязано сработать в своей точке
    // траектории, а не за несколько кадров до неё. Поэтому M-код исполняется
    // только после полного слива кольца тактов и очереди. Индекс при этом не
    // сдвигаем — вернёмся к этой же команде на следующей итерации.
    if (!isMotion && isPlannerBusy) return false;

    currentCommandIndex++; 
    currentExecutingLineNum = cmd.lineNum; 
    cfg.lastExecutedLine = cmd.lineNum;

    if (!isMotion) {
        // Исполнение М-кодов. Рекурсии нет (P2-8): следующую команду заберёт
        // цикл в supervisorTask, поэтому серия M3/M5/M8/M9 не растёт по стеку.
        switch (cmd.cmdType) {
            case 3: digitalWrite(RELE_SPINDLE, HIGH); break;
            case 5: digitalWrite(RELE_SPINDLE, LOW);  break;
            case 8: digitalWrite(RELE_COOLANT, HIGH); break;
            case 9: digitalWrite(RELE_COOLANT, LOW);  break;
            default: break;
        }
        return true;
    }

    // --- КАДР ДВИЖЕНИЯ ---
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

    // P0-8: цель проверяется целиком до первого куска, иначе станок успел бы
    // проехать часть пути и только потом узнать про лимит.
    String limitErr;
    if (!validateMachineTarget(machineX, machineY, machineZ, &limitErr)) {
        Serial.println(limitErr);
        sendToWiFiClient(limitErr);
        changeState(STATE_IDLE);
        return false;
    }

    // P1-4 + P1-7: физика и стыки проверяются на всём кадре, а не на последнем
    // куске нарезки (иначе v_end > F всплыл бы после половины хода).
    if (!validateFrameParameters(cmd.f, cfg.maxAcceleration, cmd.v_start, cmd.v_end,
                                 &limitErr)) {
        Serial.println(limitErr);
        sendToWiFiClient(limitErr);
        changeState(STATE_IDLE);
        return false;
    }

    // P1-7: кадр режется по узлам карты провиса, по одному куску за вызов.
    beginSplit(machineX, machineY, machineZ, cmd.f, cfg.maxAcceleration,
               cmd.v_start, cmd.v_end);

    bool isLast = false;
    if (!queueSplitPiece(isLast, &limitErr)) {
        resetSplit();
        if (limitErr.length() > 0) {
            Serial.println(limitErr);
            sendToWiFiClient(limitErr);
            changeState(STATE_IDLE); // остановить программу, а не ехать дальше
        }
        return false; // пусто = STOP/ALARM, состояние уже сменено извне
    }

    if (isLast) afterMotionFrameQueued();
    return true;
}

// Протокольные пределы JOG: F задаётся в мм/мин, как в обычном G-коде.
static const float JOG_MIN_FEED_MM_MIN = 1.0f;
static const float JOG_MAX_FEED_MM_MIN = 20000.0f;

bool processSingleManualCommand(String line) {
    line.toUpperCase();
    uint8_t type = 255;
    if (line.indexOf("G0") != -1) type = 0;
    if (line.indexOf("G1") != -1) type = 1;
    if ((type == 0 || type == 1) && !isPlannerBusy) {
        // P0-4: первый JOG после аварии/отладки не должен считать дельту
        // от устаревшей математической базы. Во время движения базу не трогаем:
        // plannerSteps* уже указывает на конец поставленных кадров, иначе
        // потеряются ещё не выданные шаги (P1-2, разные ядра).
        syncPlannerToActual();
    }
    
    // Вычисляем целевую позицию JOG перемещения (если координата пропущена — остаемся на месте в WCS)
    float x = (line.indexOf('X') != -1) ? line.substring(line.indexOf('X')+1).toFloat() : (currentStepsX / cfg.stepsPerMmX) - wcsOffset.x;
    float y = (line.indexOf('Y') != -1) ? line.substring(line.indexOf('Y')+1).toFloat() : (currentStepsY / cfg.stepsPerMmY) - wcsOffset.y;
    float z = (line.indexOf('Z') != -1) ? line.substring(line.indexOf('Z')+1).toFloat() : (currentStepsZ / cfg.stepsPerMmZ) - wcsOffset.z;
    // P0-7: F в JOG — мм/мин (как в G-коде и в GUI), внутри переводим в мм/с.
    float f_mm_min = (line.indexOf('F') != -1) ? line.substring(line.indexOf('F')+1).toFloat() : cfg.defaultFeedRate;

    if (type != 0 && type != 1) {
        Serial.println("ERROR: JOG requires G0 or G1.");
        sendToWiFiClient("ERROR: JOG requires G0 or G1.");
        return false;
    }
    if (!isfinite(f_mm_min) || f_mm_min < JOG_MIN_FEED_MM_MIN || f_mm_min > JOG_MAX_FEED_MM_MIN) {
        String err = "ERROR: JOG feed must be " + String(JOG_MIN_FEED_MM_MIN, 1) + ".." +
                     String(JOG_MAX_FEED_MM_MIN, 0) + " mm/min.";
        Serial.println(err);
        sendToWiFiClient(err);
        return false;
    }

    // Скорость входа/выхода JOG. Она не может быть выше подачи самого кадра:
    // иначе v_start > F даёт отрицательный разгонный участок (P1-4). Медленный
    // ручной ход (например, 60 мм/мин) идёт целиком на своей подаче.
    const float feedMM_Sec = f_mm_min / 60.0f;
    const float v_jog_base = (cfg.minVectorSpeed < feedMM_Sec) ? cfg.minVectorSpeed : feedMM_Sec;

    float machineX = x + wcsOffset.x;
    float machineY = y + wcsOffset.y;
    float machineZ = z + wcsOffset.z;

    // P0-8: точка за пределами рабочего поля не ставится в очередь вообще.
    String limitErr;
    if (!validateMachineTarget(machineX, machineY, machineZ, &limitErr)) {
        Serial.println(limitErr);
        sendToWiFiClient(limitErr);
        return false;
    }

    if (!validateFrameParameters(feedMM_Sec, cfg.maxAcceleration, v_jog_base, v_jog_base,
                                 &limitErr)) {
        Serial.println(limitErr);
        sendToWiFiClient(limitErr);
        return false;
    }

    // P1-7: JOG режется по узлам карты так же, как кадр УП. Первый кусок
    // ставим здесь, остальные — continueManualJogSplit() из supervisorTask.
    beginSplit(machineX, machineY, machineZ, feedMM_Sec, cfg.maxAcceleration,
               v_jog_base, v_jog_base);

    bool isLast = false;
    if (!queueSplitPiece(isLast, &limitErr)) {
        resetSplit();
        if (limitErr.length() > 0) {
            Serial.println(limitErr);
            sendToWiFiClient(limitErr);
        }
        return false;
    }
    return true;
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

bool startProgramExecution() {
    // Запуск УП запрещён в ALARM, без хоуминга и при пустом буфере.
    if (currentMachineState == STATE_ALARM) {
        String err = "ERROR: Machine is in ALARM. Send RESET_ALARM first.";
        Serial.println(err); sendToWiFiClient(err);
        return false;
    }
    if (!isHomed) {
        String err = "ERROR: Machine is LOCKED. Run HOMING first.";
        Serial.println(err); sendToWiFiClient(err);
        return false;
    }
    if (totalLoadedCommands <= 0) {
        String err = "ERROR: No G-code program loaded.";
        Serial.println(err); sendToWiFiClient(err);
        return false;
    }

    // ЖЕСТКИЙ HANDSHAKE БАЗЫ: Перед запуском первого кадра УП 
    // приравниваем виртуальные оси планировщика к реальному положению моторов на столе
    syncPlannerToActual();

    resetSplit();             // P1-7: недоигранная нарезка прошлого запуска не нужна
    currentCommandIndex = 0; // Сброс указателя на начало массива в ОЗУ
    if (!changeState(STATE_RUNNING)) return false;

    executeNextProgramStep(); // Вызываем первый шаг
    return true;
}

