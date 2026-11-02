#!/usr/bin/env python3
"""Measure power to speed on the floor, for the convoy slave's table.

  python3 floortable.py                  0.08 0.10 0.15 0.20 0.25 0.30
  python3 floortable.py 0.15 0.20        only these powers

For each power both wheels get that power, open loop through hw.py, as
the slave drives them, for RUN_S. The speed is measured over the last
MEASURE_S, after the rover has got up to speed. Between powers it stops
and waits for Enter, so you can carry it back. Needs about 1 m of floor
per power. Uses the cm per pulse in config.py, so calibrate first.

Prints a SPEED_TABLE line for config.py. Logged to floortable_<time>.csv.
"""

import csv
import sys
import time

import hw

POWERS = [float(p) for p in sys.argv[1:]] or [0.08, 0.10, 0.15, 0.20,
                                               0.25, 0.30]
RUN_S = 2.0
MEASURE_S = 1.0

rows = []
for power in POWERS:
    input(f"power {power}: rover on the floor, about 1 m clear ahead, "
          "Enter to drive ")
    hw.motors(power, power)
    time.sleep(RUN_S - MEASURE_S)
    t0, (l0, r0) = time.monotonic(), hw.ticks()
    time.sleep(MEASURE_S)
    t1, (l1, r1) = time.monotonic(), hw.ticks()
    hw.motors(0, 0)
    left = (l1 - l0) / 100 / (t1 - t0)
    right = (r1 - r0) / 100 / (t1 - t0)
    rows.append((power, round(left, 1), round(right, 1),
                 round((left + right) / 2, 1)))
    print(f"power {power}: left {left:.1f} cm/s, right {right:.1f} cm/s")
    time.sleep(1.0)

name = f"floortable_{time.strftime('%H%M%S')}.csv"
with open(name, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["power", "left_cm_s", "right_cm_s", "mean_cm_s"])
    w.writerows(rows)
print(f"wrote {name}")
table = ", ".join(f"({p}, {m})" for p, _, _, m in rows)
print(f"FLOOR_SPEED_TABLE = ((0.0, 0.0), {table})")
