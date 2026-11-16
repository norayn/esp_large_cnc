#include "scan_task.h"
#include "motion.h"
#include "config.h"
#include "gcode_program.h"
#include "state_manager.h"
#include "wifi_manager.h"
#include "indicator_probe.h"
#include "laser_map.h"

// ============================================================================
// ПАРАМЕТРЫ ПРОЦЕДУРЫ
// ============================================================================

// Пауза на гашение вибраций фермы после остановки, мс.
static const unsigned long SCAN_SETTLE_MS = 600;
// Ожидание ответа ESP32-CAM на MEASURE, мс.
static const unsigned long SCAN_CAM_TIMEOUT_MS = 1500;
// Предохранитель по числу точек: STEP = 0,001 мм иначе дал бы миллионы кадров.
static const int SCAN_MAX_POINTS = 2000;
// Карта провиса лежит в статическом массиве curvatureMap[250] (motion.cpp).
static const int LASER_MAP_POINTS_MAX = 250;

enum ScanMode {
    SCAN_NONE = 0,
    SCAN_GEOMETRY,
    SCAN_LASER_MAP
};

// Шаг автомата: одна фаза — один вызов updateScanTask().
enum ScanPhase {
    PHASE_POSITION,   // поставить кадр в очередь
    PHASE_MOVE,       // ждать полного слива (isPlannerBusy == false)
    PHASE_SETTLE,     // пауза на вибрации
    PHASE_MEASURE     // запрос камеры + ожидание ответа
};

static ScanMode scanMode = SCAN_NONE;
static ScanPhase scanPhase = PHASE_POSITION;

static float scanStartX = 0.0f;
static float scanStepX = 0.0f;
static float scanFeedMM_Sec = 0.0f;
static float scanTargetX = 0.0f;   // точка, к которой едем сейчас
static int scanPointsTotal = 0;    // для геометрии
static int scanPointIndex = 0;     // для геометрии — счётчик, для карты — индекс точки
static unsigned long scanPhaseStart = 0;
static unsigned long scanCamRequestMs = 0;
static float scanDialValue = 0.0f;
static bool scanDialValid = false;

static void scanReport(const String &message) {
    Serial.println(message);
    sendToWiFiClient(message);
}

static void finishScan(const char *endMessage, bool changeToIdle) {
    scanMode = SCAN_NONE;
    scanReport(endMessage);

    // changeToIdle == false, если состояние сменили извне (STOP/ESTOP/ALARM).
    if (changeToIdle && currentMachineState == STATE_CALIBRATION) {
        changeState(STATE_IDLE);
    }
}

// --- ФАЗА 1: поставить кадр в точку замера ---
static void positionNextPoint() {
    if (scanMode == SCAN_LASER_MAP) {
        if (scanPointIndex >= cfg.laserMapSize) {
            finishScan("=== END LASER MAP CALIBRATION ===", true);
            return;
        }
        scanTargetX = (float)scanPointIndex * cfg.laserGridStep;
    } else {
        if (scanPointIndex >= scanPointsTotal) {
            finishScan("=== END GEOMETRY SCAN ===", true);
            return;
        }
        scanTargetX = scanStartX + (float)scanPointIndex * scanStepX;
    }

    // P0-8: точка может быть вне поля, если оператор сменил конфиг/лимиты.
    String limitErr;
    if (!validateMachineTarget(scanTargetX, 0.0f, 0.0f, &limitErr)) {
        scanReport(limitErr);
        finishScan("=== SCAN ABORTED ===", true);
        return;
    }

    // Y и Z держим в нулях машинных координат: скан — это линия по X.
    if (!prepareVectorSegment(scanTargetX, 0.0f, 0.0f, scanFeedMM_Sec,
                              cfg.maxAcceleration, cfg.minVectorSpeed, cfg.minVectorSpeed)) {
        // Кадр не поставлен: STOP/ALARM уже сменили состояние, дублировать нечего.
        finishScan("=== SCAN ABORTED ===", false);
        return;
    }

    scanPhase = PHASE_MOVE;
}

// --- ФАЗА 4: разложить замер по месту назначения ---
static void reportMeasurement(float laserY, float laserZ, bool camOk) {
    if (scanMode == SCAN_LASER_MAP) {
        if (!camOk) {
            scanReport("DATA: Point " + String(scanPointIndex) + " -> NO CAMERA RESPONSE");
            return;
        }
        updateCurvaturePoint(scanPointIndex, laserY, laserZ);
        scanReport("DATA: Point " + String(scanPointIndex) +
                   " -> Y=" + String(laserY) + ", Z=" + String(laserZ));
        return;
    }

    // Строка CSV для стенда и GUI: X;DIAL;LASER_Y;LASER_Z (формат не менялся).
    String dialStr = "DISCONNECTED";
    if (scanDialValid && scanDialValue != -999.0f) dialStr = String(scanDialValue, 3);
    String laserYStr = camOk ? String(laserY, 3) : String("DISCONNECTED");
    String laserZStr = camOk ? String(laserZ, 3) : String("DISCONNECTED");

    scanReport(String(scanTargetX, 1) + ";" + dialStr + ";" + laserYStr + ";" + laserZStr);
}

// Переход к следующей точке: индекс сдвигаем только после замера.
static void advancePoint() {
    scanPointIndex++;
    scanPhase = PHASE_POSITION;
}

void updateScanTask() {
    if (scanMode == SCAN_NONE) return;

    // STOP / ESTOP / RESET_ALARM сменили состояние — движение уже остановлено
    // requestMotionStop(), скану остаётся только закрыться.
    if (currentMachineState != STATE_CALIBRATION) {
        finishScan("=== SCAN ABORTED ===", false);
        return;
    }

    switch (scanPhase) {
        case PHASE_POSITION:
            positionNextPoint();
            return;

        case PHASE_MOVE:
            // isPlannerBusy == false означает «станок стоит, шагов больше не будет»
            // (очередь кадров и кольцо пусты), а не «кадр отдан в генерацию».
            if (isPlannerBusy) return;
            scanPhaseStart = millis();
            scanPhase = PHASE_SETTLE;
            return;

        case PHASE_SETTLE:
            if (millis() - scanPhaseStart < SCAN_SETTLE_MS) return;

            scanDialValid = false;
            if (scanMode == SCAN_GEOMETRY) {
                // Ограниченное по времени чтение (свои таймауты внутри): максимум
                // ~100 мс, за которые supervisorTask успевает вернуться в loop().
                scanDialValue = readDialIndicator();
                scanDialValid = true;
            }

            Serial2.println("MEASURE");
            scanCamRequestMs = millis();
            scanPhase = PHASE_MEASURE;
            return;

        case PHASE_MEASURE: {
            float y = 0.0f, z = 0.0f;
            if (takeCamResponse(scanCamRequestMs, y, z)) {
                reportMeasurement(y, z, true);
                advancePoint();
            } else if (millis() - scanCamRequestMs >= SCAN_CAM_TIMEOUT_MS) {
                reportMeasurement(0.0f, 0.0f, false);
                advancePoint();
            }
            return;
        }
    }
}

bool isScanActive() {
    return scanMode != SCAN_NONE;
}

void cancelScan() {
    if (scanMode == SCAN_NONE) return;
    scanMode = SCAN_NONE;
    Serial.println("=== SCAN ABORTED ===");
}

bool startGeometryScan(float startX, float endX, float stepX, float feedRateMM_Sec) {
    if (isScanActive()) {
        scanReport("ERROR: A scan is already running.");
        return false;
    }

    // P1-8: STEP = 0 раньше давал деление на ноль, END <= START — пустой скан.
    if (!isfinite(startX) || !isfinite(endX) || !isfinite(stepX) || !isfinite(feedRateMM_Sec) ||
        stepX <= 0.0f || endX <= startX || feedRateMM_Sec <= 0.0f) {
        scanReport("ERROR: SCAN_GEOMETRY requires finite START/END, STEP>0, END>START and SPEED>0.");
        return false;
    }

    // Считаем в float: (int) от 1e10 — уже UB, а STEP может быть каким угодно.
    float span = endX - startX;
    float pointsF = span / stepX + 1.0f;
    if (!isfinite(pointsF) || pointsF > (float)SCAN_MAX_POINTS) {
        scanReport("ERROR: SCAN_GEOMETRY needs more than " + String(SCAN_MAX_POINTS) +
                   " points. Increase STEP.");
        return false;
    }
    int points = (int)pointsF;

    // P0-8: крайние точки проверяем до перехода в CALIBRATION, чтобы отказ не
    // мигал состоянием. Каждая точка дополнительно проверяется в positionNextPoint().
    String limitErr;
    if (!validateMachineTarget(startX, 0.0f, 0.0f, &limitErr) ||
        !validateMachineTarget(endX, 0.0f, 0.0f, &limitErr)) {
        scanReport(limitErr);
        return false;
    }

    // Хоуминг и запрет ALARM проверяет changeState.
    if (!changeState(STATE_CALIBRATION)) return false;

    scanMode = SCAN_GEOMETRY;
    scanPhase = PHASE_POSITION;
    scanStartX = startX;
    scanStepX = stepX;
    scanFeedMM_Sec = feedRateMM_Sec;
    scanPointsTotal = points;
    scanPointIndex = 0;
    scanTargetX = startX;

    // Скан идёт от фактической позиции: база планировщика = факт (P0-4).
    syncPlannerToActual();

    scanReport("=== START GEOMETRY SCAN ===");
    scanReport("X_POS;DIAL_INDICATOR;LASER_Y;LASER_Z");
    return true;
}

bool startLaserMapCalibration() {
    if (isScanActive()) {
        scanReport("ERROR: A scan is already running.");
        return false;
    }

    // P1-7 проверит конфиг целиком; здесь — минимум, без которого процедура
    // вышла бы за массив curvatureMap[250] или поделила на ноль.
    if (cfg.laserMapSize <= 0 || cfg.laserMapSize > LASER_MAP_POINTS_MAX) {
        scanReport("ERROR: laserMapSize must be 1.." + String(LASER_MAP_POINTS_MAX) + ".");
        return false;
    }
    if (!(cfg.laserGridStep > 0.0f) || !isfinite(cfg.laserGridStep)) {
        scanReport("ERROR: laserGridStep must be > 0 mm.");
        return false;
    }

    float lastX = (float)(cfg.laserMapSize - 1) * cfg.laserGridStep;
    String limitErr;
    if (!validateMachineTarget(lastX, 0.0f, 0.0f, &limitErr)) {
        scanReport(limitErr);
        return false;
    }

    if (!changeState(STATE_CALIBRATION)) return false;

    scanMode = SCAN_LASER_MAP;
    scanPhase = PHASE_POSITION;
    // Подача калибровки — defaultFeedRate (та же, что fallback JOG): отдельного
    // поля в конфиге нет, а это заведомо безопасная ручная скорость.
    scanFeedMM_Sec = cfg.defaultFeedRate / 60.0f;
    if (!(scanFeedMM_Sec > 0.0f)) scanFeedMM_Sec = cfg.minVectorSpeed;
    scanPointIndex = 0;
    scanTargetX = 0.0f;

    syncPlannerToActual();

    scanReport("=== START LASER MAP CALIBRATION ===");
    return true;
}
