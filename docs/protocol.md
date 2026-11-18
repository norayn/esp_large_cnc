# 6. Сетевой протокол (TCP, порт 8888)

> Часть паспорта прошивки. Оглавление — [INDEX.md](INDEX.md).
> **Читать перед:** правками `wifi_manager.cpp`, парсеров, форматов ответов, единиц.
> **Источник истины:** `src/wifi_manager.cpp`, `src/gcode_program.cpp`, `src/state_manager.cpp`.

## 6.1 Транспорт

- ESP32 поднимает точку доступа `CNC_4METERS_ROUTER` / `CNC_password_123` (пароль зашит в `wifi_manager.cpp`; **сменить перед эксплуатацией**).
- TCP-сервер слушает **8888**, одновременно обслуживается один клиент.
- Команды и ответы — строки, разделитель `\n` (парсер также принимает `\r`).
- Тот же диспетчер команд используется для USB-UART `Serial` (115200), поэтому USB и Wi-Fi равноправны.

## 6.2 Телеметрия

Каждые `statusInterval` мс (по умолчанию **1000**) в Wi-Fi и USB уходит:

```text
<Status:IDLE (LOCKED)|Pos:X=0.00,Y=0.00,Z=0.00|Hom:0|Line=0>
```

- `<Status:…>` — имя состояния; суффикс ` (LOCKED)` добавляется только для `INIT`/`IDLE` при `!isHomed`.
- `Pos` — координаты **детали** (WCS), 2 знака.
- `Hom` — 1/0.
- `Line` — номер последнего кадра, **поставленного в конвейер** (`uint16_t`).
  **⚠** С P1-3 опережает физику до чем длина очереди кадров (до 4 кадров), поэтому
  не годится как «где режет сейчас».
- При `sendOnlyOnChange = false` строка отправляется каждый интервал; при `true` — только при изменении.
- `sendToWiFiClient()` шлёт строку **только в TCP**; в USB её печатает тот, кому она нужна. Телеметрия и ответы на команды идут через `sendToBothClients()` — ровно одна строка в TCP и одна в USB (P2-6).
- Период `statusInterval` ограничен снизу **50 мс**: `0` или отрицательное значение (пока их принимает `SET_CONFIG`, P2-5) не зальёт сокет и USB телеметрией.
- Дополнительные строки: `SINGLE_BLOCK_WAIT` (при покадровом режиме), CSV сканера, `DATA: Point …`, `LAST_LINE:N`, `CONFIG_DATA:…`.

## 6.3 Глобальные команды (доступны в любом состоянии)

| Команда | Действие |
|---|---|
| `ESTOP` | `STATE_ALARM`, моторы обесточены, реле выключены |
| `STOP` | мягкая остановка: очистка очереди, `IDLE`, `isHomed` сохраняется; в `ALARM` остаётся `ALARM` |
| `RESET_ALARM` | `isHomed = false`, остановка движения, переход в `IDLE` (машина снова LOCKED) |
| `HOLD` или `!` | пауза из `RUNNING`/`JOGGING` |
| `RESUME` или `~` | продолжение из `HOLD` без потери шагов, в то состояние, откуда пришли |
| `OVERRIDE=<0.1…2.0>` | плавный множитель подачи |

> `RESET_ALARM`, `STOP` и переход в `ALARM` вызывают `requestMotionStop()`: выдача шагов прекращается сразу, буфер очищается в критической секции. Таймер остаётся работоспособным, поэтому `ESTOP`/`MOTORS_ALARM` по-прежнему обрабатываются в ISR без участия `supervisorTask`.

## 6.4 Команды режима IDLE

| Команда | Действие |
|---|---|
| `HOME` / `$H` | полный цикл хоуминга Z→X→Y |
| `JOG:<G0/G1 … F…>` | ручное перемещение, `F` — мм/мин (см. 6.8) |
| `SET_HOMED_DEBUG` | обнулить счётчики и снять блокировку без хоуминга |
| `SET_ZERO:X|Y|Z|ALL` | обнулить WCS, сохранить в NVS |
| `GET_CONFIG` | выдать `CONFIG_DATA:…` (все поля `CONFIG_FIELDS`) |
| `SET_CONFIG:<name>=<value>` | изменить параметр и сохранить **весь** конфиг в NVS |
| `START_PROGRAM` | старт загруженной УП |
| `START_FROM_LINE:<N>` | старт с строки (фактически всегда с 0, P2-1) |
| `GET_LAST_ABORTED_LINE` | `LAST_LINE:<N>` (значение сохраняется только в `ALARM`) |
| `SINGLE_BLOCK_ON/OFF` | включить/выключить покадровый режим |
| `SET_ALIGN_A` / `SET_ALIGN_B` | запомнить точки A/B |
| `ACTIVATE_ALIGN` / `DEACTIVATE_ALIGN` | включить/выключить наклон |
| `GCODE_UPLOAD_START` | очистить буфер и перейти в приём УП |
| `START_CALIBRATION` | калибровка лазерной карты по `laserMapSize` / `laserGridStep` |
| `EXPORT_MAP` | выгрузка карты CSV в сокет |
| `MAP_LOAD_START` | переход в режим импорта карты |
| `SCAN_GEOMETRY:<START>;<END>;<STEP>;<SPEED>` | скан геометрии, `SPEED` — мм/мин |

> Команды, которые могут начать движение (`START_PROGRAM`, `START_FROM_LINE`, `JOG`, `SCAN_GEOMETRY`, `HOME`, `START_CALIBRATION`), сначала вызывают `changeState()`. Если переход запрещён (`ALARM` или `!isHomed`), в сокет уходит `ERROR: ...`, и движение не начинается. Исключение — `JOG` при `allowJogBeforeHoming = true` (по умолчанию).

`JOG` завершается сам: когда очередь кадров и кольцо шагов пусты, FSM возвращается в `IDLE`. Новый `JOG` принимается только в `IDLE` — серию клиент шлёт по факту `IDLE` в телеметрии; во время хода команда получает `ERROR: Unknown or blocked command` ([fsm-safety.md §3.2](fsm-safety.md#32-переходы-и-блокировки)).

`SCAN_GEOMETRY` и `START_CALIBRATION` только запускают процедуру и сразу отвечают: дальше скан идёт по шагам из `supervisorTask`, поэтому телеметрия и `STOP`/`ESTOP` обрабатываются всё время прохода. Ход процедуры виден по строкам `=== START … ===` / `DATA: …` / `=== END … ===`, прерывание — `=== SCAN ABORTED ===` (форматы — [geometry.md §5.4](geometry.md#54-комплексное-сканирование-геометрии)).

## 6.5 Загрузка УП

- `GCODE_UPLOAD_START` → ответ `STATUS: Ready`.
- Далее пакеты:

```text
B:<TYPE>;<LINE>;<X>;<Y>;<Z>;<F>;<V_START>;<V_END>
```

| Поле | Тип | Смысл |
|---|---|---|
| `TYPE` | int | 0/1 — движение (G0/G1), 3/5 — шпиндель, 8/9 — СОЖ |
| `LINE` | int | номер строки исходного файла (`uint16_t`) |
| `X`,`Y`,`Z` | float | целевые координаты **WCS**, мм |
| `F` | float | подача кадра, **мм/с** |
| `V_START`,`V_END` | float | скорости стыка от Look-Ahead, **мм/с** |

- `GCODE_UPLOAD_END` → `STATUS: Upload complete. Loaded N commands.` + `ok`, возврат в `IDLE`.
- При переполнении (2000 кадров) — `ERROR: Add to RAM buffer failed`.
- Физика кадра проверяется при постановке в очередь (§4.7 в [motion.md](motion.md)): `F <= 0`,
  `accel <= 0` и `NaN/Inf` в любом поле дают `ERROR: Frame feedrate must be > 0 mm/s.` /
  `ERROR: Frame acceleration must be > 0 mm/s^2.` / `ERROR: Frame physics is not finite (NaN/Inf).`,
  станок при этом не двигается, УП останавливается (`IDLE`). `v_start`/`v_end` больше
  подачи кадра более чем на 1 % — тоже `ERROR: Frame …`; в пределах допуска скорости
  зажимаются в коридор.

## 6.6 Импорт карты лазера

`MAP_LOAD_START` → серия строк:

```text
MAP_LOAD:IDX=<i>;Y=<offset>;Z=<offset>
```

→ `MAP_LOAD_END` → `IDLE`.

## 6.7 Конфигурация

- `GET_CONFIG` возвращает `CONFIG_DATA:` + `поле=значение;` для **всех** полей `CONFIG_FIELDS` (имена — как в коде, не ключи EEPROM).
- `SET_CONFIG:<name>=<value>` принимает любое поле, сохраняет весь конфиг в NVS и отвечает `STATUS: Config parameter <name> updated and saved.`
- **⚠** Значения `float` в `GET_CONFIG` печатаются с **2 знаками** (`String(float)`), поэтому малые величины (например, `slopeY`) теряют точность.
- **⚠** Валидации нет: можно задать `stepsPerMmX = 0`, `laserMapSize > 250`, отрицательный `laserGridStep` и т.п. (P2-5).

Полный список параметров — [config.md](config.md).

## 6.8 Единицы измерения (ФАКТ)

| Где | Единица | Комментарий |
|---|---|---|
| `B:…;F;…` | **мм/с** | GUI переводит F из мм/мин: `curr_f / 60` |
| `B:…;V_START;V_END` | **мм/с** | результат Look-Ahead GUI |
| `maxAcceleration` | мм/с² | |
| `JOG:…F…` | **мм/мин** | как в G-коде; прошивка переводит в мм/с. Допустимо 1…20000 |
| `SCAN_GEOMETRY:…;SPEED` | **мм/мин** | прошивка переводит в мм/с. Допустимо 1…20000 |
| `defaultFeedRate` | мм/мин | fallback JOG, переводится в мм/с |
| `pullOffX/Y/Z` | шаги | |
| координаты в протоколе | мм | |

## 6.9 Ответы

- `ok` — команда принята/выполнена.
- `ERROR: Unknown or blocked command: …` — команда не разрешена в текущем состоянии.
- `ERROR: Machine is LOCKED. Run HOMING first.` — нарушена блокировка хоуминга.
- `STATUS: …` — текстовые статусы (многие команды выдают `STATUS` и затем `ok`).
- `ERROR: Add to RAM buffer failed`, `ERROR: Look-Ahead binary packet corrupt! …`, `ERROR: Frame …` (битая физика кадра).
- Скан и калибровка: `ERROR: SCAN_GEOMETRY requires finite START/END, STEP>0, END>START and SPEED>0.`, `ERROR: SCAN_GEOMETRY SPEED must be 1..20000 mm/min.`, `ERROR: SCAN_GEOMETRY needs more than 2000 points. Increase STEP.`, `ERROR: laserMapSize must be 1..250.`, `ERROR: laserGridStep must be > 0 mm.`, `ERROR: A scan is already running.` Отказ приходит до перехода в `STATE_CALIBRATION`, станок остаётся в `IDLE`.
- `ERROR: Endstop chain is open before homing. …` — общая NC-цепь концевиков разомкнута, `HOME` не начат ([homing.md §3.4](homing.md#34-хоуминг)).
- `ERROR: laserMapSize must be 1..250 (was N).`, `ERROR: laserGridStep must be finite and > 0 mm.` — `SET_CONFIG` отклонён, прежние значения карты сохранены ([geometry.md §5.3](geometry.md#53-оптическая-карта-провиса-фермы)).

---

Смежное: состояния и переходы — [fsm-safety.md](fsm-safety.md); что ждёт GUI — [gui.md](gui.md); параметры — [config.md](config.md).
