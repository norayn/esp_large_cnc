# 8, 9. Аппаратура: pinout, сборка, прошивка

> Часть паспорта прошивки. Оглавление — [INDEX.md](INDEX.md).
> **Читать перед:** правками `pinout.h`, подключением периферии, обновлением платформы.
> **Источник истины:** `src/pinout.h`, `platformio.ini`.
>
> Числа в таблице сверяются с `src/pinout.h` скриптом: `python3 scripts/docs.py check`.
> Меняете GPIO — меняйте `pinout.h`, а не только таблицу.

## 8. Карта подключения периферии

| Узел (`pinout.h`) | GPIO | Направление | Примечание |
|---|---:|---|---|
| `X_STEP_PIN` | 25 | выход | драйвер X (рейка) |
| `X_DIR_PIN` | 26 | выход | |
| `Y_STEP_PIN` | 27 | выход | драйвер Y (винт) |
| `Y_DIR_PIN` | 14 | выход | |
| `Z_STEP_PIN` | 32 | выход | драйвер Z (винт) |
| `Z_DIR_PIN` | 33 | выход | |
| `MOTORS_ENABLE` | 13 | выход | **HIGH = обесточить**, LOW = удержание |
| `ESTOP_PIN` | 34 | вход (only) | активный HIGH, внешний pull-down + оптрон |
| `MOTORS_ALARM` | 35 | вход (only) | активный HIGH, внешний pull-down + оптрон |
| `ENDSTOPS_PIN` | 39 | вход (only) | одна NC-цепь X+Y+Z, норма = LOW, срабатывание/обрыв = HIGH |
| `CAM_RX2_PIN` | 16 | вход | UART2, 115200, защита 330 Ом + TVS |
| `CAM_TX2_PIN` | 17 | выход | UART2 |
| `INDICATOR_CLK` | 18 | вход | `INPUT_PULLUP` |
| `INDICATOR_DATA` | 19 | вход | `INPUT_PULLUP` |
| `SPINDLE_PWM` | 23 | выход | **не используется** (резерв под 0–10 В, P2-11) |
| `RELE_SPINDLE` | 21 | выход | реле шпинделя (M3/M5) |
| `RELE_COOLANT` | 22 | выход | реле СОЖ (M8/M9) |
| `PC_RX0_PIN` | 3 | вход | UART0 через CH9102 |
| `PC_TX0_PIN` | 1 | выход | UART0 |

**Примечания:**

- GPIO 34/35/39 не имеют внутренней подтяжки — `INPUT_PULLUP` для них не действует; в коде используется `INPUT`, уровень задают только внешние резисторы (pull-down) и оптроны.
- Входы безопасности проверяются самотестом при старте и отключаются флагом `SAFETY_INPUTS_ENABLED` в `src/pinout.h` (см. [fsm-safety.md §3.3](fsm-safety.md#33-аварийные-входы)).
- GPIO 16/17 свободны на ESP32-WROOM без PSRAM; на модуле с PSRAM эти выводы заняты.
- GPIO 4, 2, 5, 15 свободны (резерв: Z-probe, обдув, светофор).
- Экраны кабелей заземляются на PE с одной стороны — внутри шкафа.

## 9. Сборка и прошивка

`platformio.ini`:

```ini
[env:esp32dev]
platform = espressif32@6.12.0
board = esp32dev
framework = arduino
monitor_speed = 115200
upload_speed = 921600
board_build.f_cpu = 240000000L
monitor_filters = send_on_enter
```

- Внешних библиотек нет (Wi-Fi, Preferences — из состава Arduino core).
- Версия зафиксирована: `espressif32@6.12.0` → **Arduino core 2.0.17**. `pio run` проходит: RAM 35,9 % (117,6 КБ из 320 КБ), Flash 63,1 % (827 КБ из 1,25 МБ раздела приложения).
- Код таймера использует API core 2.x: `timerBegin(0, 80, true)`, `timerAttachInterrupt(t, fn, true)`, `timerAlarmWrite()`.
- Для перехода на core 3.x: `hw_timer_t *t = timerBegin(1000000); timerAttachInterrupt(t, fn); timerAlarm(t, CNC_TIMER_TICK_US, true, 0);` — та же сетка 1 МГц и 5 мкс на тик.
- Прошивка: `pio run -t upload`; монитор: `pio device monitor` (115200, отправка по Enter).

---

Смежное: аварийные входы и самотест — [fsm-safety.md §3.3](fsm-safety.md#33-аварийные-входы); сетка таймера — [architecture.md §1.3](architecture.md#13-таймер-и-сетка-времени).
