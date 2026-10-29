#!/usr/bin/env python3
"""Measure cm per pulse for each wheel, and the track width.

  python3 calibrate.py roll            drive straight, Enter to stop
  python3 calibrate.py circle left     40 / 20 rpm circle, Enter to stop
  python3 calibrate.py circle right

roll: drive slowly past a measured length, stop with Enter, then type
the distance the rover really moved, from a ruler. Prints the pulses and
cm per pulse of each wheel. Put them in config.py.

circle: stop at the start mark after some full turns, then type how many.
Uses the cm per pulse in config.py, so run roll first.
track = (outer cm - inner cm) / (2 x pi x turns). Do both sides and use
the mean: it cancels a difference between the wheels.

Every run is logged to calibrate_<mode>_<time>.csv.
"""

import csv
import math
import select
import sys
import time

import config
from estop import EStop
from rover import disconnect, Rover, SpeedController, Wheel

MODE = sys.argv[1] if len(sys.argv) > 1 else "roll"
SIDE = sys.argv[2] if len(sys.argv) > 2 else "left"
ROLL_RPM = 30.0
OUTER_RPM = 40.0
INNER_RPM = 20.0

if MODE not in ("roll", "circle"):
    sys.exit(f"{MODE} is not roll or circle")
if SIDE not in ("left", "right"):
    sys.exit(f"{SIDE} is not left or right")

if MODE == "roll":
    targets = (ROLL_RPM, ROLL_RPM)
elif SIDE == "left":
    targets = (INNER_RPM, OUTER_RPM)
else:
    targets = (OUTER_RPM, INNER_RPM)

estop = EStop()
rover = Rover(estop)
left = Wheel(config.LEFT_ENCODER)
right = Wheel(config.RIGHT_ENCODER)
control = SpeedController(rover, left, right)

print(f"{MODE} {SIDE if MODE == 'circle' else ''} at "
      f"{targets[0]:.0f} / {targets[1]:.0f} rpm, Enter to stop")

try:
    estop.arm()
    left.reset()
    right.reset()
    control.target(*targets)
    # Enter stops it. Ctrl+C left GPIO25 unwritable in estop.close(), 29 Oct.
    # select waits one control interval for a key, so it is the loop sleep
    while not select.select([sys.stdin], [], [], config.CONTROL_INTERVAL)[0]:
        control.update()
    sys.stdin.readline()
    control.stop()
    # the wheels coast a little after the stop, count that too
    time.sleep(0.5)
    pulses = (left.pulses, right.pulses)
finally:
    estop.close()
    left.close()
    right.close()
    disconnect()

print(f"pulses  left {pulses[0]}  right {pulses[1]}")
if MODE == "roll":
    cm = float(input("cm the rover moved, by ruler: "))
    result = (cm / pulses[0], cm / pulses[1])
    print(f"LEFT_CM_PER_PULSE = {result[0]:.5f}")
    print(f"RIGHT_CM_PER_PULSE = {result[1]:.5f}")
else:
    turns = float(input("full turns: "))
    left_cm = pulses[0] * config.LEFT_CM_PER_PULSE
    right_cm = pulses[1] * config.RIGHT_CM_PER_PULSE
    track = abs(right_cm - left_cm) / (2 * math.pi * turns)
    result = (left_cm, right_cm, track)
    print(f"left {left_cm:.1f} cm  right {right_cm:.1f} cm  "
          f"track {track:.2f} cm")

name = f"calibrate_{MODE}_{time.strftime('%H%M%S')}.csv"
with open(name, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["mode", "side", "left_target", "right_target",
                "left_pulses", "right_pulses", "entered", "result"])
    w.writerow([MODE, SIDE if MODE == "circle" else "", *targets, *pulses,
                cm if MODE == "roll" else turns,
                " ".join(f"{r:.5f}" for r in result)])
print(f"wrote {name}")
