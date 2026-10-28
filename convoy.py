#!/usr/bin/env python3
"""Drive as the leader or a follower of a convoy.

The leader drives at a fixed speed, on the blue stripe (line) or with
both wheels at the same speed (straight). A follower keeps 20 to 40 cm
behind whatever is in front of it: the gap sets its speed, the line
sets its steering. Every rover is on the same track, so the line takes
the followers through the leader's curves.

  python3 convoy.py leader line 30 60       leader on blue, 30 rpm, 60 s
  python3 convoy.py follower line 30 60     follower on blue, up to 45 rpm
  python3 convoy.py follower straight 30 20

In line mode place the rover red on its left, the sensor centred on
blue, as for follow.py.

Speed from the gap, proportional: RPM + GAP_GAIN x RPM x (cm - GAP_CM),
clamped 0 to MAX_SHARE x RPM. At 30 cm it matches the leader, at 40 cm
or more it is 1.5 x RPM. At MIN_CM or closer, or with nothing in range,
it stops, and drives again as soon as the gap is back. The leader stops
the same way if something is in front of it, so it never hits anything.

Every loop is logged to convoy_<role>_<mode>_<rpm>_<time>.csv.
Ctrl+C stops it, so does the e-stop button.
"""

import csv
import sys
import time

import config
from colour import ColourSensor, classify, drift
from distance import Distance
from estop import EStop
from rover import disconnect, Rover, SpeedController, Wheel
from steering import steer

ROLE = sys.argv[1] if len(sys.argv) > 1 else "follower"
MODE = sys.argv[2] if len(sys.argv) > 2 else "line"
RPM = float(sys.argv[3]) if len(sys.argv) > 3 else 30.0
DURATION = float(sys.argv[4]) if len(sys.argv) > 4 else 20.0

# the page: "follow any rover in front of it at a distance of 20 to 40
# cm". aim for the middle, stop at the bottom: under 20 cm the IR sensor
# reads far again (Lab 9), so the follower must never get closer
GAP_CM = 30.0
MIN_CM = 20.0
# share of RPM added per cm of gap error, so 40 cm gives 1.5 x RPM
GAP_GAIN = 0.05
MAX_SHARE = 1.5
TARGET = "blue"
ORDER = config.TRACK_ORDER

if ROLE not in ("leader", "follower"):
    sys.exit(f"{ROLE} is not leader or follower")
if MODE not in ("line", "straight"):
    sys.exit(f"{MODE} is not line or straight")


def gap_speed(cm):
    """Speed from the gap ahead, and why: "go", "close" or "lost".

    cm is 0 when too close to measure and None when nothing is in range,
    as Distance.read() returns it.
    """
    if cm is None:
        return 0.0, "lost"
    if cm <= MIN_CM:
        return 0.0, "close"
    if ROLE == "leader":
        return RPM, "go"
    speed = RPM * (1.0 + GAP_GAIN * (cm - GAP_CM))
    return min(max(speed, 0.0), MAX_SHARE * RPM), "go"


estop = EStop()
rover = Rover(estop)
left = Wheel(config.LEFT_ENCODER)
right = Wheel(config.RIGHT_ENCODER)
control = SpeedController(rover, left, right)
sensor = Distance()
colour = ColourSensor() if MODE == "line" else None

print(f"{ROLE} {MODE} at {RPM:.0f} rpm ({config.rpm_to_cm_s(RPM):.0f} cm/s) "
      f"for {DURATION:.0f} s")

name = f"convoy_{ROLE}_{MODE}_{RPM:.0f}_{time.strftime('%H%M%S')}.csv"
rows = []
turn = None
centre = None
state = None
reading = (0, 0, 0, 0)
seen = ""

if colour is not None:
    # sensor centred on blue at start, as in follow.py
    centre = drift(colour.read(), TARGET, ORDER)
    if centre is not None:
        print(f"centre drift {centre:+.3f}")

try:
    estop.arm()
    start = time.monotonic()
    while time.monotonic() - start < DURATION:
        time.sleep(config.CONTROL_INTERVAL)
        volts, cm = sensor.read()
        speed, now = gap_speed(cm)
        if now != state:
            t = time.monotonic() - start
            shown = "out of range" if cm is None else f"{cm:.0f} cm"
            print(f"{t:5.2f} s  {now.upper()}  {shown}")
            if now != "go":
                control.stop()
            state = now

        if colour is not None:
            reading = colour.read()
            seen = classify(reading)
            left_rpm, right_rpm, turn = steer(seen, reading, TARGET, turn,
                                              speed, ORDER, centre)
        else:
            left_rpm = right_rpm = speed
        control.target(left_rpm, right_rpm)

        (lr, ld), (rr, rd) = control.update()
        t = time.monotonic() - start
        rows.append((round(t, 3), ROLE, MODE, round(volts, 3),
                     "" if cm is None else round(cm, 1), state,
                     round(speed, 1), *reading, seen, turn or "",
                     round(lr, 1), round(ld, 4), round(rr, 1), round(rd, 4)))
    control.stop()
finally:
    with open(name, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "role", "mode", "volts", "cm", "state", "speed",
                    "red", "green", "blue", "clear", "seen", "turn",
                    "left_rpm", "left_duty", "right_rpm", "right_duty"])
        w.writerows(rows)
    print(f"wrote {name}, {len(rows)} rows")
    estop.close()
    sensor.close()
    if colour is not None:
        colour.close()
    left.close()
    right.close()
    disconnect()
