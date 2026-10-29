#!/usr/bin/env python3
"""Measure cm per pulse for each wheel, and the track width.

  python3 calibrate.py roll            drive straight, Enter to stop
  python3 calibrate.py circle left     40 / 20 rpm circle, Enter to stop
  python3 calibrate.py circle right
  python3 calibrate.py check 100       drive 100 cm by encoder, then stop

roll: drive slowly past a measured length, stop with Enter, then type
how far the rover moved forward along a tape measure laid beside the
path, and how far it ended to the side of the line it was aimed along.
The straight line from start to end is worked out from the two. A curve means the outer
wheel rolled further, so each wheel gets its own distance:
heading = 2 x side / distance, outer - inner = track x heading.
Prints the pulses and cm per pulse of each wheel. Put them in config.py.

circle: stop at the start mark after some full turns, then type how many.
Uses the cm per pulse in config.py, so run roll first.
track = (outer cm - inner cm) / (2 x pi x turns). Do both sides and use
the mean: it cancels a difference between the wheels.

check: drive straight until the encoders say the distance is done, stop,
then type forward and side by tape measure. Prints the encoder distance
after coasting against the tape, the test of the calibration.

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
CHECK_CM = float(sys.argv[2]) if MODE == "check" and len(sys.argv) > 2 else 100.0
ROLL_RPM = 30.0
OUTER_RPM = 40.0
INNER_RPM = 20.0

if MODE not in ("roll", "circle", "check"):
    sys.exit(f"{MODE} is not roll, circle or check")
if MODE == "circle" and SIDE not in ("left", "right"):
    sys.exit(f"{SIDE} is not left or right")

if MODE in ("roll", "check"):
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
        if MODE == "check" and (left.pulses * config.LEFT_CM_PER_PULSE
                                + right.pulses * config.RIGHT_CM_PER_PULSE) / 2 >= CHECK_CM:
            break
    else:
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
    forward = float(input("cm forward, along the tape measure: "))
    side = float(input("cm to the side, right positive: "))
    cm = math.hypot(forward, side)
    # a right curve has the left wheel outside
    gap = config.TRACK_CM * 2 * side / cm
    result = ((cm + gap / 2) / pulses[0], (cm - gap / 2) / pulses[1])
    print(f"LEFT_CM_PER_PULSE = {result[0]:.5f}")
    print(f"RIGHT_CM_PER_PULSE = {result[1]:.5f}")
elif MODE == "check":
    left_cm = pulses[0] * config.LEFT_CM_PER_PULSE
    right_cm = pulses[1] * config.RIGHT_CM_PER_PULSE
    encoder = (left_cm + right_cm) / 2
    forward = float(input("cm forward, along the tape measure: "))
    side = float(input("cm to the side, right positive: "))
    cm = math.hypot(forward, side)
    result = (encoder, cm, (encoder - cm) / cm * 100)
    print(f"encoders {encoder:.1f} cm  tape {cm:.1f} cm  "
          f"error {result[2]:+.2f} percent")
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
                turns if MODE == "circle" else f"{forward} {side}",
                " ".join(f"{r:.5f}" for r in result)])
print(f"wrote {name}")
