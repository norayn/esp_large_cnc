#!/usr/bin/env python3
"""Генерация и проверка документации.

Единственный источник истины для параметров конфигурации — `src/config.h`
(`CONFIG_FIELDS`), для GPIO — `src/pinout.h`. Документация не хранит их копию:
таблица в `docs/config.md` генерируется, числа в `docs/hardware.md` сверяются.

`check` также следит за структурой: статус задачи живёт только в индексе
`TODO.md`, тело — в `docs/tasks/`; закрытый пункт не должен остаться в индексе.
Используется в CI (`.github/workflows/docs.yml`).

    python3 scripts/docs.py check   # проверить (код возврата 1 при расхождении)
    python3 scripts/docs.py links   # проверить относительные ссылки и анкоры
    python3 scripts/docs.py gen     # перегенерировать таблицу в docs/config.md

Требуется только стандартная библиотека Python 3.8+.
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CONFIG_H = ROOT / "src" / "config.h"
PINOUT_H = ROOT / "src" / "pinout.h"
CONFIG_MD = ROOT / "docs" / "config.md"
HARDWARE_MD = ROOT / "docs" / "hardware.md"
INDEX_MD = ROOT / "docs" / "INDEX.md"
TODO_MD = ROOT / "TODO.md"
TASKS_DIR = ROOT / "docs" / "tasks"
OPEN_TASK_FILES = ("p2.md", "p3.md")
DONE_TASK_GLOB = "done*.md"

BEGIN = "<!-- BEGIN GENERATED: config-fields -->"
END = "<!-- END GENERATED: config-fields -->"

# Назначение полей — единственное, чего нет в коде. Ключи должны совпадать
# с CONFIG_FIELDS, иначе `check` падает.
DESCRIPTIONS = {
    "stepsPerMmX": "шагов/мм по X (рейка)",
    "stepsPerMmY": "шагов/мм по Y (винт)",
    "stepsPerMmZ": "шагов/мм по Z (винт)",
    "defaultFeedRate": "подача JOG по умолчанию, мм/мин (переводится в мм/с)",
    "rapidFeedRate": "**не используется**",
    "maxAcceleration": "ускорение профиля, мм/с²",
    "minVectorSpeed": "стартовая/стыковочная скорость, мм/с",
    "laserGridStep": "шаг сетки карты провиса, мм (> 0; длиннее не бывает кусок кадра)",
    "laserMapSize": "число точек карты, 1…250",
    "pwmFrequency": "**не используется** (резерв под ШИМ шпинделя, P2-11)",
    "pwmResolutionBits": "**не используется** (резерв, P2-11)",
    "minX": "soft limit, мм",
    "maxX": "soft limit, мм",
    "minY": "soft limit, мм",
    "maxY": "soft limit, мм",
    "minZ": "soft limit, мм",
    "maxZ": "soft limit, мм",
    "allowJogBeforeHoming": "разрешить JOG до хоуминга",
    "feedMultiplier": "**не используется**",
    "isAlignmentActive": "коррекция наклона включена",
    "slopeY": "коэффициент наклона Y (ΔY/ΔX)",
    "slopeZ": "коэффициент наклона Z (ΔZ/ΔX)",
    "pointA_x": "опорная точка A, MCS",
    "pointA_y": "опорная точка A, MCS",
    "pointA_z": "опорная точка A, MCS",
    "pointB_x": "опорная точка B, MCS",
    "pointB_y": "опорная точка B, MCS",
    "pointB_z": "опорная точка B, MCS",
    "wcsOffsetX": "смещение G54 по X",
    "wcsOffsetY": "смещение G54 по Y",
    "wcsOffsetZ": "смещение G54 по Z",
    "lastExecutedLine": "последний кадр (в NVS пишется при ALARM)",
    "statusInterval": "период телеметрии, мс (минимум 50)",
    "sendOnlyOnChange": "слать телеметрию только при изменении",
    "homingDirX": "направление хоуминга X",
    "homingDirY": "направление хоуминга Y",
    "homingDirZ": "направление хоуминга Z",
    "pullOffX": "отъезд после датчика, шагов",
    "pullOffY": "отъезд после датчика, шагов",
    "pullOffZ": "отъезд после датчика, шагов",
}

FIELD_RE = re.compile(
    r'^\s*X\(\s*(?P<type>\w+)\s*,\s*(?P<name>\w+)\s*,\s*'
    r'"(?P<key>[^"]+)"\s*,\s*(?P<default>[^)]*?)\s*\)\s*\\?\s*$'
)
PIN_RE = re.compile(r"^#define\s+(?P<name>[A-Z0-9_]+)\s+(?P<num>-?\d+)\s*$")
ROW_RE = re.compile(r"^\|\s*`(?P<name>[A-Z0-9_]+)`\s*\|\s*(?P<num>-?\d+)\s*\|")

SIZE_LIMITS = {
    "README.md": 6 * 1024,
    "AGENTS.md": 8 * 1024,
    "TODO.md": 10 * 1024,
}
DOCS_DIR_LIMIT = 12 * 1024
DONE_TASKS_LIMIT = 20 * 1024
TASK_ID_RE = re.compile(r"\*\*(?P<id>P\d-\d+)\b")
INDEX_ROW_RE = re.compile(r"^\|\s*(?P<id>P\d-\d+)\s*\|", re.M)


def parse_config_fields():
    """Возвращает список (тип, имя, ключ NVS, дефолт) из CONFIG_FIELDS."""
    text = CONFIG_H.read_text(encoding="utf-8")
    start = text.index("#define CONFIG_FIELDS")
    end = text.index("\n\n", start)
    block = text[start:end]
    fields = []
    for line in block.splitlines()[1:]:
        m = FIELD_RE.match(line)
        if m:
            fields.append(
                (
                    m.group("type"),
                    m.group("name"),
                    m.group("key"),
                    m.group("default").rstrip("f").strip(),
                )
            )
    if not fields:
        raise SystemExit("docs.py: не удалось разобрать CONFIG_FIELDS")
    return fields


def parse_pins():
    pins = {}
    for line in PINOUT_H.read_text(encoding="utf-8").splitlines():
        m = PIN_RE.match(line.strip())
        if m:
            pins[m.group("name")] = int(m.group("num"))
    return pins


def render_table(fields):
    lines = [
        BEGIN,
        "",
        "| Поле | Ключ NVS | Тип | По умолчанию | Назначение |",
        "|---|---|---|---|---|",
    ]
    for ftype, name, key, default in fields:
        desc = DESCRIPTIONS.get(name, "—")
        lines.append(f"| `{name}` | `{key}` | {ftype} | {default} | {desc} |")
    lines += ["", END]
    return "\n".join(lines)


def read_generated(path):
    text = path.read_text(encoding="utf-8")
    if BEGIN not in text or END not in text:
        raise SystemExit(f"docs.py: в {path.name} нет маркеров {BEGIN} / {END}")
    start = text.index(BEGIN)
    stop = text.index(END) + len(END)
    return text, start, stop


def cmd_gen():
    fields = parse_config_fields()
    text, start, stop = read_generated(CONFIG_MD)
    CONFIG_MD.write_text(text[:start] + render_table(fields) + text[stop:], encoding="utf-8")
    print(f"обновлено: {CONFIG_MD.relative_to(ROOT)} ({len(fields)} полей)")


def task_errors():
    """Статус задачи живёт только в индексе TODO.md, тело — в docs/tasks/."""
    errors = []
    index_ids = INDEX_ROW_RE.findall(TODO_MD.read_text(encoding="utf-8"))

    # Архив — все done*.md: один файл перерос лимит, поэтому P0 и P1 разведены.
    done_files = sorted(p.name for p in TASKS_DIR.glob(DONE_TASK_GLOB))
    if not done_files:
        errors.append(f"docs/tasks/: нет ни одного архива {DONE_TASK_GLOB}")

    defined = {}
    for name in OPEN_TASK_FILES + tuple(done_files):
        path = TASKS_DIR / name
        if not path.exists():
            errors.append(f"docs/tasks/{name}: файл отсутствует")
            continue
        limit = DONE_TASKS_LIMIT if name in done_files else DOCS_DIR_LIMIT
        if len(path.read_bytes()) > limit:
            errors.append(f"docs/tasks/{name}: файл слишком большой — разделите по темам")
        for tid in TASK_ID_RE.findall(path.read_text(encoding="utf-8")):
            if tid in defined:
                errors.append(f"{tid}: описан дважды ({defined[tid]} и {name})")
            defined[tid] = name

    for tid in index_ids:
        if tid not in defined:
            errors.append(f"TODO.md: {tid} есть в индексе, но его тела нет в docs/tasks/")
        elif defined[tid] in done_files:
            errors.append(f"TODO.md: {tid} закрыт (лежит в docs/tasks/{defined[tid]}), но всё ещё в индексе открытых")
    for tid, name in sorted(defined.items()):
        if name not in done_files and tid not in index_ids:
            errors.append(f"docs/tasks/{name}: {tid} нет в индексе TODO.md — добавьте строку или перенесите в архив {DONE_TASK_GLOB}")

    open_count = len([t for t in defined if defined[t] not in done_files])
    if not errors and open_count != len(index_ids):
        errors.append(f"TODO.md: строк в индексе {len(index_ids)}, открытых пунктов {open_count}")
    return errors


def _slug(heading):
    s = heading.strip().lower()
    s = s.replace("`", "")
    s = re.sub(r"[^\w\s\-а-яё]", "", s)
    return s.replace(" ", "-")


def cmd_links():
    """Проверяет относительные ссылки и анкоры в markdown."""
    files = [ROOT / "README.md", ROOT / "AGENTS.md", ROOT / "TODO.md"]
    files += sorted(INDEX_MD.parent.glob("*.md")) + sorted(TASKS_DIR.glob("*.md"))
    anchors = {}
    for f in files:
        anchors[f.resolve()] = {
            _slug(m.group(2))
            for m in re.finditer(r"^(#{1,6})\s+(.*)$", f.read_text(encoding="utf-8"), re.M)
        }

    errors = []
    for f in files:
        rel = f.relative_to(ROOT)
        for m in re.finditer(r"\[[^\]]*\]\(([^)]+)\)", f.read_text(encoding="utf-8")):
            url = m.group(1)
            if url.startswith(("http://", "https://", "mailto:")):
                continue
            path, _, frag = url.partition("#")
            target = (f.parent / path).resolve() if path else f.resolve()
            if not target.exists():
                errors.append(f"{rel}: битая ссылка {url}")
            elif frag and frag not in anchors.get(target, set()):
                errors.append(f"{rel}: нет анкора #{frag} в {target.name}")

    if errors:
        print("Битые ссылки:\n")
        for e in errors:
            print(f"  • {e}")
        return 1
    print(f"docs.py: OK — ссылки и анкоры целы ({len(files)} файлов)")
    return 0


def cmd_check():
    errors = []
    fields = parse_config_fields()
    names = {f[1] for f in fields}

    for missing in sorted(names - DESCRIPTIONS.keys()):
        errors.append(f"docs/config.md: поле `{missing}` добавлено в CONFIG_FIELDS, но не описано в scripts/docs.py (DESCRIPTIONS)")
    for extra in sorted(DESCRIPTIONS.keys() - names):
        errors.append(f"scripts/docs.py: описание `{extra}` не соответствует ни одному полю CONFIG_FIELDS")

    text, start, stop = read_generated(CONFIG_MD)
    actual = text[start:stop]
    expected = render_table(fields)
    if actual.strip() != expected.strip():
        errors.append("docs/config.md: таблица полей расходится с src/config.h — запустите `python3 scripts/docs.py gen`")

    pins = parse_pins()
    seen = {}
    for lineno, line in enumerate(HARDWARE_MD.read_text(encoding="utf-8").splitlines(), 1):
        m = ROW_RE.match(line.strip())
        if not m:
            continue
        name, num = m.group("name"), int(m.group("num"))
        if name not in pins:
            errors.append(f"docs/hardware.md:{lineno}: `{name}` нет в src/pinout.h")
            continue
        if pins[name] != num:
            errors.append(
                f"docs/hardware.md:{lineno}: `{name}` = GPIO {num}, а в src/pinout.h — {pins[name]}"
            )
        seen[name] = num
    for name in sorted(pins.keys() - seen.keys()):
        if name.endswith(("_PIN", "ENABLE")) or name in {
            "MOTORS_ALARM",
            "ENDSTOPS_PIN",
            "INDICATOR_CLK",
            "INDICATOR_DATA",
            "SPINDLE_PWM",
            "RELE_SPINDLE",
            "RELE_COOLANT",
        }:
            errors.append(f"docs/hardware.md: не описан `{name}` (GPIO {pins[name]})")

    for rel, limit in SIZE_LIMITS.items():
        p = ROOT / rel
        if p.exists() and p.stat().st_size > limit:
            errors.append(f"{rel}: {p.stat().st_size} байт > лимита {limit} — разнесите по docs/")

    errors.extend(task_errors())

    for p in sorted(INDEX_MD.parent.glob("*.md")):
        size = p.stat().st_size
        if size > DOCS_DIR_LIMIT:
            errors.append(f"docs/{p.name}: {size} байт > {DOCS_DIR_LIMIT} — файл стал монолитом")
        if p.name != "INDEX.md" and "](INDEX.md)" not in p.read_text(encoding="utf-8"):
            errors.append(f"docs/{p.name}: нет ссылки на INDEX.md в шапке")

    if errors:
        print("Документация расходится с кодом:\n")
        for e in errors:
            print(f"  • {e}")
        return 1
    print("docs.py: OK — конфиг, GPIO и размеры файлов согласованы")
    return 0


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "check"
    if cmd == "gen":
        cmd_gen()
        return 0
    if cmd == "check":
        return cmd_check()
    if cmd == "links":
        return cmd_links()
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main())
