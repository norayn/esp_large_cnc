# AGENTS.md

Навигатор по репозиторию. Читается первым; заменяет чтение README целиком.

**Что это:** прошивка промышленного ЧПУ-контроллера на ESP32 (Arduino/PlatformIO) для 4-метрового фрезера. Управление по TCP 8888 и USB-UART, УП загружается бинарными кадрами `B:`, шаги генерируются на Core 1 аппаратным таймером 200 кГц.

## Сборка и запуск

```bash
pio run                  # сборка
pio run -t upload        # прошивка
pio device monitor       # USB, 115200, отправка по Enter
python3 scripts/docs.py check   # доки не разошлись с кодом, структура задач цела
python3 scripts/docs.py links   # относительные ссылки и анкоры
```

Платформа зафиксирована: `espressif32@6.12.0` = **Arduino core 2.0.17**. API таймера — core 2.x (`timerBegin(0, 80, true)`). Внешних библиотек нет. Для core 3.x нужен порт таймера (см. [docs/hardware.md §9](docs/hardware.md#9-сборка-и-прошивка)).

## Карта кода

| Файл | Ответственность |
|---|---|
| `src/main.cpp` | `setup`/`loop`, таймер, USB-команды, запуск задач |
| `src/motion.cpp/.h` | планировщик, DDA/Брезенхем, кольцевой буфер, ISR, хоуминг, HOLD/OVERRIDE |
| `src/gcode_program.cpp/.h` | буфер УП, WCS, наклон A/B, исполнение кадров, JOG |
| `src/state_manager.cpp/.h` | FSM, безопасность, телеметрия |
| `src/wifi_manager.cpp/.h` | TCP-сервер, диспетчер команд, разбор `B:` |
| `src/config.cpp/.h` | `CONFIG_FIELDS`, NVS, `GET/SET_CONFIG` |
| `src/laser_map.cpp/.h` | карта провиса, калибровка, экспорт/импорт |
| `src/indicator_probe.cpp/.h` | цифровой щуп, скан геометрии |
| `src/safety_inputs.cpp/.h` | входы ESTOP/MOTORS_ALARM/ENDSTOPS, самотест NC-цепи |
| `src/pinout.h` | карта GPIO и флаг `SAFETY_INPUTS_ENABLED` |

## Что читать перед правкой

| Трогаете | Прочитайте |
|---|---|
| задачи, приоритеты, ядра, частоту таймера | [docs/architecture.md](docs/architecture.md) |
| FSM, аварии, хоуминг, soft limits | [docs/fsm-safety.md](docs/fsm-safety.md) |
| планировщик, буфер, ISR, HOLD/OVERRIDE | [docs/motion.md](docs/motion.md) |
| WCS, наклон, карту провиса, сканы | [docs/geometry.md](docs/geometry.md) |
| команды, форматы, единицы | [docs/protocol.md](docs/protocol.md) |
| протокол в связке с GUI | [docs/gui.md](docs/gui.md) |
| GPIO, платформу, прошивку | [docs/hardware.md](docs/hardware.md) |
| поле конфига, NVS | [docs/config.md](docs/config.md) |
| что делать дальше | [TODO.md](TODO.md) — индекс открытых пунктов, затем только нужный ID в `docs/tasks/pN.md` |

Оглавление всей документации: [docs/INDEX.md](docs/INDEX.md).

## Инварианты — не ломать

1. **ISR не блокируется и не вызывает сервисные функции:** никаких `changeState()`, `Preferences`, `Serial`, `String`, аллокаций в прерывании. Авария из ISR — только обесточить, сбросить очередь и выставить `volatile`-флаг; FSM обновляет `supervisorTask`.
2. **`motionTask` и ISR живут на Core 1**, `supervisorTask` — на Core 0. ISR привязан к ядру, где вызван `timerAttachInterrupt()` из `setup()`.
3. **Обмен между ядрами только под мьютексами:** `xferMux` для очереди кадров `frameQueue`, `ringMux` для `stepRingBuffer`/`plannerSteps*`. Кадры ставить только через `prepareVectorSegment()`, место проверять через `canQueueFrame()`. Очистка кольца — только после остановки генератора (`isMotionGenerating`) и в критической секции.
4. **`plannerSteps*` и `currentSteps*` не расходятся:** после любой остановки/аварии/`SET_HOMED_DEBUG` вызывается `syncPlannerToActual()`.
5. **`isPlannerBusy` снимается только когда пусты и очередь кадров, и кольцо тактов.** Это флаг «станок стоит», а не «кадр отдан в генерацию»: на нём держатся завершение программы, синхронизация JOG и сканы.
6. **Любое движение проходит через `validateMachineTarget()`** и через проверку результата `changeState()`.
7. **`SAFETY_INPUTS_ENABLED = 0` — только стенд.** Перед эксплуатацией вернуть `1`; при `0` E-STOP, ошибки драйверов и концевики не контролируются.
8. **Сетка времени задаётся одним макросом** `CNC_TIMER_TICK_US`; не хардкодить 5 мкс/10 мкс/100 кГц в других местах.
9. **`CONFIG_FIELDS` — единственный список параметров.** Новое поле без него не попадёт в NVS и в `GET_CONFIG`.
## Правила документации

- **Один факт — одно место.** Нужен в двух местах — ссылка, а не копия.
- **Не редактируйте сгенерированную таблицу** в [docs/config.md](docs/config.md): поле добавляется в `src/config.h`, описание — в `DESCRIPTIONS` (`scripts/docs.py`), затем `python3 scripts/docs.py gen`.
- GPIO в [docs/hardware.md](docs/hardware.md) сверяются с `src/pinout.h` тем же скриптом. `check` должен проходить перед коммитом.
- `README.md` — только паспорт и ссылки. Всё содержательное — в `docs/`.
- `TODO.md` — единственный источник списка работ и известных ограничений. Не дублируйте его в других файлах.
- **Статус задачи живёт только в таблице `TODO.md`.** Тело пункта — в `docs/tasks/p1.md`/`p2.md`/`p3.md` без чекбоксов. Пункт закрыли: убрали строку из индекса, тело перенесли в `docs/tasks/done.md` с датой и коммитом. `docs.py check` следит за этим соответствием.
- Номера разделов (`§3.3`, `§6.8`) сохранены от прежнего README — на них ссылаются код и TODO, не переименовывайте без причины.
- Язык документации — русский; имена, команды и ключи — как в коде (латиницей).
- Файл в `docs/` больше 12 КБ — разделить по темам.

## Стиль правок

- Минимальный diff: `edit` по уникальному контексту, а не перезапись файла.
- Не перечитывать файл после своей правки и не читать `docs/` целиком — только нужный раздел.
- Правки кода и правки документации — отдельными коммитами: историю доков видно через `git log -- docs/`.
