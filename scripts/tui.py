#!/usr/bin/env python3
"""Terminal UI for picking Hydra config-group presets before launching
run.py. Cycles through one preset per group (run/sim/events/physics,
discovered from conf/<group>/*.yaml) plus a gui on/off toggle, then execs
`python run.py <group>=<preset> ... [gui=true]` with the current selections.

Arrow keys / j,k move between rows; left/right / h,l cycle the selected
row's value; enter launches; q quits without running.

No new dependency: built on the stdlib curses module only.

Each preset's one-line description (shown live for the highlighted row) is
read from a `# description: ...` comment on the second line of its YAML
file -- see conf/<group>/*.yaml. This is a plain comment, never touched by
Hydra/OmegaConf composition, so it can't leak into TeleopConfig.
"""
from __future__ import annotations

import curses
import os
import subprocess
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
_CONF_DIR = _REPO_ROOT / "conf"
_GROUPS = ["run", "sim", "events", "physics"]
_GUI_DESCRIPTIONS = {
    "false": "Headless batch mode (p.DIRECT, no sleep pacing).",
    "true": "Interactive PyBullet GUI demo (chase camera, live HUD overlay).",
}


def _preset_description(path: Path) -> str:
    for line in path.read_text().splitlines()[:5]:
        line = line.strip()
        if line.startswith("# description:"):
            return line[len("# description:"):].strip()
    return ""


def _discover_presets(group: str) -> list[dict]:
    group_dir = _CONF_DIR / group
    paths = sorted(group_dir.glob("*.yaml"), key=lambda p: p.stem)
    presets = [{"name": p.stem, "description": _preset_description(p)} for p in paths]
    # "default" first if present, so it's the initial selection.
    for i, preset in enumerate(presets):
        if preset["name"] == "default":
            presets.insert(0, presets.pop(i))
            break
    return presets


def _build_rows() -> list[dict]:
    rows = [{"label": g, "values": _discover_presets(g), "idx": 0} for g in _GROUPS]
    gui_values = [{"name": v, "description": d} for v, d in _GUI_DESCRIPTIONS.items()]
    rows.append({"label": "gui", "values": gui_values, "idx": 0})
    return rows


def _command_for(rows: list[dict]) -> list[str]:
    cmd = [sys.executable, str(_REPO_ROOT / "run.py")]
    for row in rows:
        name = row["values"][row["idx"]]["name"]
        if row["label"] == "gui":
            if name == "true":
                cmd.append("gui=true")
            continue
        if name != "default":
            cmd.append(f"{row['label']}={name}")
    return cmd


def _draw(stdscr, rows: list[dict], selected: int) -> None:
    stdscr.erase()
    stdscr.addstr(0, 0, "run.py config picker", curses.A_BOLD)
    stdscr.addstr(1, 0, "up/down (or j/k): move   left/right (or h/l): cycle value   enter: launch   q: quit")
    stdscr.addstr(2, 0, "-" * 70)

    for i, row in enumerate(rows):
        y = 4 + i
        marker = ">" if i == selected else " "
        value = row["values"][row["idx"]]
        attr = curses.A_REVERSE if i == selected else curses.A_NORMAL
        line = f"{marker} {row['label']:<8} {value['name']}"
        stdscr.addstr(y, 0, line, attr)

    desc = rows[selected]["values"][rows[selected]["idx"]]["description"]
    desc_y = 4 + len(rows) + 1
    stdscr.addstr(desc_y, 0, "-" * 70)
    stdscr.addstr(desc_y + 1, 0, (desc or "(no description)")[: max(0, curses.COLS - 1)])

    cmd_line = " ".join(_command_for(rows))
    stdscr.addstr(desc_y + 3, 0, "-" * 70)
    stdscr.addstr(desc_y + 4, 0, "will run:")
    stdscr.addstr(desc_y + 5, 0, cmd_line[: max(0, curses.COLS - 1)])
    stdscr.refresh()


def _main(stdscr) -> list[str] | None:
    curses.curs_set(0)
    rows = _build_rows()
    selected = 0

    while True:
        _draw(stdscr, rows, selected)
        key = stdscr.getch()

        if key in (curses.KEY_UP, ord("k")):
            selected = (selected - 1) % len(rows)
        elif key in (curses.KEY_DOWN, ord("j")):
            selected = (selected + 1) % len(rows)
        elif key in (curses.KEY_LEFT, ord("h")):
            row = rows[selected]
            row["idx"] = (row["idx"] - 1) % len(row["values"])
        elif key in (curses.KEY_RIGHT, ord("l")):
            row = rows[selected]
            row["idx"] = (row["idx"] + 1) % len(row["values"])
        elif key in (curses.KEY_ENTER, 10, 13):
            return _command_for(rows)
        elif key in (ord("q"), 27):  # 27 = Esc
            return None


def main() -> None:
    os.chdir(_REPO_ROOT)
    cmd = curses.wrapper(_main)
    if cmd is None:
        print("Cancelled.")
        return
    print("Running:", " ".join(cmd))
    subprocess.run(cmd)


if __name__ == "__main__":
    main()
