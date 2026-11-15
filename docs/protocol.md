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
- `Line` — номер текущего кадра (`uint16_t`).
- При `sendOnlyOnChange = false` строка отправляется каждый интервал; при `true` — только при изменении.
- **⚠** `sendToWiFiClient()` дополнительно печатает всё в `Serial`, а `reportStatusToPC()` печатает статус ещё раз → в USB каждая строка статуса дублируется (P2-6).
- Дополнительные строки: `SINGLE_BLOCK_WAIT` (при покадровом режиме), CSV сканера, `DATA: Point …`, `LAST_LINE:N`, `CONFIG_DATA:…`.

## 6.3 Глобальные команды (доступны в любом состоянии)

| Команда | Действие |
|---|---|
| `ESTOP` | `STATE_ALARM`, моторы обесточены, реле выключены |
| `STOP` | мягкая остановка: очистка очереди, `IDLE`, `isHomed` сохраняется; в `ALARM` остаётся `ALARM` |
| `RESET_ALARM` | `isHomed = false`, остановка движения, переход в `IDLE` (машина снова LOCKED) |
| `HOLD` или `!` | пауза из `RUNNING`/`JOGGING` |
| `RESUME` или `~` | продолжение из `HOLD` без потери шагов |
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
| `START_CALIBRATION` | прогон лазерной карты (сейчас без движения) |
| `EXPORT_MAP` | выгрузка карты CSV в сокет |
| `MAP_LOAD_START` | переход в режим импорта карты |
| `SCAN_GEOMETRY:<START>;<END>;<STEP>;<SPEED>` | скан геометрии, `SPEED` — мм/мин |

> Команды, которые могут начать движение (`START_PROGRAM`, `START_FROM_LINE`, `JOG`, `SCAN_GEOMETRY`, `HOME`, `START_CALIBRATION`), сначала вызывают `changeState()`. Если переход запрещён (`ALARM` или `!isHomed`), в сокет уходит `ERROR: ...`, и движение не начинается. Исключение — `JOG` при `allowJogBeforeHoming = true` (по умолчанию).

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
- `ERROR: Add to RAM buffer failed`, `ERROR: Look-Ahead binary packet corrupt! …`.

---

Смежное: состояния и переходы — [fsm-safety.md](fsm-safety.md); что ждёт GUI — [gui.md](gui.md); параметры — [config.md](config.md).
