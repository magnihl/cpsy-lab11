#!/usr/bin/env python3
"""Prove steering.steer() decides exactly as follow.py's steer() did.

Runs the Lab 10 steer() (taken from follow.py at rev f08af50) and the
new one on the same inputs, every row of every Lab 10 log, and counts
the rows where the two disagree. Runs on the laptop, no Pi needed.

  python3 replay_steer.py logs/follow_*.csv
"""

import ast
import csv
import subprocess
import sys
import types

# colour.py imports the Pi libraries, which the laptop does not have
for lib in ("pigpio", "gpiozero"):
    sys.modules[lib] = types.ModuleType(lib)
sys.modules["gpiozero"].Motor = sys.modules["gpiozero"].Robot = None

import config
import colour
import steering

OLD_REV = "f08af50"

source = subprocess.run(["git", "show", f"{OLD_REV}:follow.py"],
                        capture_output=True, text=True, check=True).stdout
tree = ast.parse(source)
keep = [n for n in tree.body
        if isinstance(n, ast.FunctionDef) and n.name == "steer"]
old = {"drift": colour.drift, "edge_side": colour.edge_side,
       "side_of": colour.side_of}
for n in tree.body:
    # the two gains, so the old function sees the values it ran with
    if isinstance(n, ast.Assign) and n.targets[0].id in ("SOFT_TURN",
                                                          "STEER_GAIN"):
        keep.append(n)
exec(compile(ast.Module(keep, []), "old_follow", "exec"), old)

rows_total = mismatches = 0
for name in sys.argv[1:]:
    rpm = float(name.split("_")[2])
    with open(name) as f:
        rows = list(csv.DictReader(f))
    order = config.TRACK_ORDER
    centre = None
    for row in rows:
        if row.get("error"):
            reading = tuple(int(row[k]) for k in ("red", "green", "blue",
                                                   "clear"))
            # error is drift minus centre, rounded to 3 places. both
            # versions get the same centre, so the rounding cannot matter
            centre = colour.drift(reading, "blue", order) - float(row["error"])
            break
    last_turn = None
    flips = 0
    for row in rows:
        f_now = int(row.get("flips") or 0)
        if f_now != flips:
            # follow.py flips the order and the sign of the centre
            order = tuple(reversed(order))
            centre = -centre if centre is not None else None
            flips = f_now
        reading = tuple(int(row[k]) for k in ("red", "green", "blue", "clear"))
        seen = row["seen"]
        old.update(RPM=rpm, ORDER=order, centre=centre)
        a = old["steer"](seen, reading, "blue", last_turn)
        b = steering.steer(seen, reading, "blue", last_turn, rpm, order, centre)
        rows_total += 1
        if a != b:
            mismatches += 1
            print(f"{name} t={row['t']}  old {a}  new {b}")
        last_turn = row["turn"] or None

print(f"{len(sys.argv) - 1} logs, {rows_total} rows, {mismatches} differ")
