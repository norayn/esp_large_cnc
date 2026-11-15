# 10. Параметры конфигурации (`CONFIG_FIELDS`)

> Часть паспорта прошивки. Оглавление — [INDEX.md](INDEX.md).
> **Читать перед:** добавлением поля, правками `config.cpp`, NVS-логики.
> **Источник истины:** `src/config.h` (сам список), `src/config.cpp` (NVS).
>
> Таблица ниже **генерируется** из `src/config.h`. Не редактируйте её руками:
> добавьте `X(...)` в `CONFIG_FIELDS` и описание в `scripts/docs.py`, затем
> `python3 scripts/docs.py gen`. `check` падает, если таблица отстала от кода.

## Как это устроено

- Список полей задаётся макросом `CONFIG_FIELDS` в `src/config.h`; из него генерируются структура `MachineConfig`, дефолты, загрузка и сохранение в NVS.
- Ключи в NVS (`Preferences`, namespace `cnc_cfg`) отличаются от имён полей; `GET_CONFIG`/`SET_CONFIG` работают с **именами полей**, а не с ключами.
- Новое поле добавляется только в `CONFIG_FIELDS` (плюс `DESCRIPTIONS` в `scripts/docs.py`), иначе оно не попадёт ни в структуру, ни в NVS.

## Поля

<!-- BEGIN GENERATED: config-fields -->

| Поле | Ключ NVS | Тип | По умолчанию | Назначение |
|---|---|---|---|---|
| `stepsPerMmX` | `stepsX` | float | 100.0 | шагов/мм по X (рейка) |
| `stepsPerMmY` | `stepsY` | float | 400.0 | шагов/мм по Y (винт) |
| `stepsPerMmZ` | `stepsZ` | float | 400.0 | шагов/мм по Z (винт) |
| `defaultFeedRate` | `defFeed` | float | 600.0 | подача JOG по умолчанию, мм/мин (переводится в мм/с) |
| `rapidFeedRate` | `rapFeed` | float | 2000.0 | **не используется** |
| `maxAcceleration` | `accel` | float | 150.0 | ускорение профиля, мм/с² |
| `minVectorSpeed` | `minSpeed` | float | 2.0 | стартовая/стыковочная скорость, мм/с |
| `laserGridStep` | `lGrid` | float | 20.0 | шаг сетки карты провиса, мм |
| `laserMapSize` | `lSize` | int | 201 | число точек карты (массив — 250, см. P1-7) |
| `pwmFrequency` | `pwmFreq` | int | 5000 | **не используется** (резерв под ШИМ шпинделя, P2-11) |
| `pwmResolutionBits` | `pwmRes` | int | 10 | **не используется** (резерв, P2-11) |
| `minX` | `minX` | float | 0.0 | soft limit, мм |
| `maxX` | `maxX` | float | 4000.0 | soft limit, мм |
| `minY` | `minY` | float | 0.0 | soft limit, мм |
| `maxY` | `maxY` | float | 500.0 | soft limit, мм |
| `minZ` | `minZ` | float | -200.0 | soft limit, мм |
| `maxZ` | `maxZ` | float | 0.0 | soft limit, мм |
| `allowJogBeforeHoming` | `jogBefore` | bool | true | разрешить JOG до хоуминга |
| `feedMultiplier` | `feedMult` | float | 1.0 | **не используется** |
| `isAlignmentActive` | `alignAct` | bool | false | коррекция наклона включена |
| `slopeY` | `slopeY` | float | 0.0 | коэффициент наклона Y (ΔY/ΔX) |
| `slopeZ` | `slopeZ` | float | 0.0 | коэффициент наклона Z (ΔZ/ΔX) |
| `pointA_x` | `ptAx` | float | 0.0 | опорная точка A, MCS |
| `pointA_y` | `ptAy` | float | 0.0 | опорная точка A, MCS |
| `pointA_z` | `ptAz` | float | 0.0 | опорная точка A, MCS |
| `pointB_x` | `ptBx` | float | 0.0 | опорная точка B, MCS |
| `pointB_y` | `ptBy` | float | 0.0 | опорная точка B, MCS |
| `pointB_z` | `ptBz` | float | 0.0 | опорная точка B, MCS |
| `wcsOffsetX` | `wcsX` | float | 0.0 | смещение G54 по X |
| `wcsOffsetY` | `wcsY` | float | 0.0 | смещение G54 по Y |
| `wcsOffsetZ` | `wcsZ` | float | 0.0 | смещение G54 по Z |
| `lastExecutedLine` | `lastLine` | int | 0 | последний кадр (в NVS пишется при ALARM) |
| `statusInterval` | `statInt` | int | 1000 | период телеметрии, мс |
| `sendOnlyOnChange` | `sendChg` | bool | false | слать телеметрию только при изменении |
| `homingDirX` | `homDirX` | int | -1 | направление хоуминга X |
| `homingDirY` | `homDirY` | int | -1 | направление хоуминга Y |
| `homingDirZ` | `homDirZ` | int | 1 | направление хоуминга Z |
| `pullOffX` | `pullOffX` | int | 400 | отъезд после датчика, шагов |
| `pullOffY` | `pullOffY` | int | 800 | отъезд после датчика, шагов |
| `pullOffZ` | `pullOffZ` | int | 800 | отъезд после датчика, шагов |

<!-- END GENERATED: config-fields -->

## Сохранение

`SET_CONFIG` пишет **все** поля в NVS; `SET_ZERO` и `SET_ALIGN_*` — только соответствующие смещения. Циклической записи в Flash во время выполнения УП нет: `lastExecutedLine` в RAM, в NVS попадает при `ALARM` (и при любом `SET_CONFIG`).

## ⚠ Известные ограничения

- `GET_CONFIG` печатает `float` с 2 знаками — малые величины (`slopeY`, `slopeZ`) теряют точность (P2-5).
- `SET_CONFIG` не валидирует значения: `stepsPerMmX = 0`, `laserMapSize > 250`, отрицательный `laserGridStep` принимаются (P2-5, P1-7).
- `bool` парсится через `toInt()`, поэтому `false` превращается в `0`, а не в «ложь по умолчанию» (P2-5).
- Поля, помеченные **не используется**, существуют, но ни на что не влияют (P3-2).

---

Смежное: протокол `GET/SET_CONFIG` — [protocol.md §6.7](protocol.md#67-конфигурация); soft limits — [fsm-safety.md §3.6](fsm-safety.md#36-программные-лимиты-soft-limits).
