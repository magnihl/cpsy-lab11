#!/usr/bin/env python3
"""Follow one coloured stripe of the track, stop before obstacles.

The three stripes touch side by side (config.TRACK_ORDER), so drifting
off the target lands the sensor on a neighbour, and the neighbour says
which side it drifted to. Turns stop one wheel instead of reversing it,
as the lab page suggests: the sensor sits near the axle.

  python3 follow.py                     blue at 30 rpm for 20 s
  python3 follow.py red 30 20           red at 30 rpm for 20 s
  python3 follow.py red 30 20 reverse   the track driven the other way
  python3 follow.py blue 30 90 forward 0   distance stop off, for testing

Driving the track the other way round swaps which stripe is on the left,
so "reverse" flips config.TRACK_ORDER. Following the middle stripe the
rover also notices by itself: facing the right way, a hard turn away
from red or green brings blue back. If white comes first, the turn went
the wrong way, so the order is flipped.

Every loop is logged to follow_<colour>_<rpm>_<direction>_<time>.csv.
Ctrl+C stops it, so does the e-stop button.
"""

import csv
import sys
import time

import config
from colour import ColourSensor, classify, drift
from steering import steer
from distance import Distance
from estop import EStop
from rover import disconnect, Rover, SpeedController, Wheel

TARGET = sys.argv[1] if len(sys.argv) > 1 else "blue"
RPM = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
DURATION = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0
DIRECTION = sys.argv[4] if len(sys.argv) > 4 else "forward"
STOP_CM = float(sys.argv[5]) if len(sys.argv) > 5 else 30.0
# drift with the sensor centred on blue, measured at start. None for an
# outer stripe, where drift cannot be read
centre = None

if TARGET not in config.TRACK_ORDER:
    sys.exit(f"{TARGET} is not one of {', '.join(config.TRACK_ORDER)}")
if DIRECTION not in ("forward", "reverse"):
    sys.exit(f"{DIRECTION} is not forward or reverse")
ORDER = (config.TRACK_ORDER if DIRECTION == "forward"
         else tuple(reversed(config.TRACK_ORDER)))


estop = EStop()
rover = Rover(estop)
left = Wheel(config.LEFT_ENCODER)
right = Wheel(config.RIGHT_ENCODER)
control = SpeedController(rover, left, right)
sensor = Distance()
colour = ColourSensor()

print(f"follow {TARGET} at {RPM:.0f} rpm ({config.rpm_to_cm_s(RPM):.0f} cm/s) "
      f"for {DURATION:.0f} s, "
      f"{'distance stop off' if STOP_CM <= 0 else f'stop at {STOP_CM:.0f} cm'}, "
      f"stripes left to right {' '.join(ORDER)}")

name = f"follow_{TARGET}_{RPM:.0f}_{DIRECTION}_{time.strftime('%H%M%S')}.csv"
rows = []
turn = None
stopped = False
# the last red or green seen since blue, for spotting a wrong way start
last_neighbour = None
flips = 0

# the rover is placed with the sensor centred on the target, so the
# first reading defines the centre for this run's light and canvas
centre = drift(colour.read(), TARGET, ORDER)
if centre is not None:
    print(f"centre drift {centre:+.3f}")

try:
    estop.arm()
    start = time.monotonic()
    while time.monotonic() - start < DURATION:
        time.sleep(config.CONTROL_INTERVAL)
        volts, cm = sensor.read()
        reading = colour.read()
        seen = classify(reading)

        if seen in ("red", "green"):
            last_neighbour = seen
        elif seen == TARGET:
            last_neighbour = None
        elif seen == "white" and last_neighbour and ORDER.index(TARGET) == 1:
            # a neighbour then white: the turn away from it drove further
            # out, so the stripes are the other way round. a wrong flip
            # gives the same pattern again and flips back
            ORDER = tuple(reversed(ORDER))
            turn = {"left": "right", "right": "left"}.get(turn)
            # drift is right neighbour minus left, so it changes sign too
            centre = -centre if centre is not None else None
            last_neighbour = None
            flips += 1
            print(f"{time.monotonic() - start:5.2f} s  FLIP  stripes left to right "
                  f"{' '.join(ORDER)}")

        # latched, as in avoid.py: once stopped it stays stopped
        if STOP_CM > 0 and not stopped and cm is not None and cm <= STOP_CM:
            stopped = True
            control.stop()
            print(f"{time.monotonic() - start:5.2f} s  STOP  {cm:.0f} cm")

        if not stopped:
            left_rpm, right_rpm, turn = steer(seen, reading, TARGET, turn,
                                                 RPM, ORDER, centre)
            control.target(left_rpm, right_rpm)

        (lr, ld), (rr, rd) = control.update()
        t = time.monotonic() - start
        rows.append((round(t, 3), *reading, seen, turn or "",
                     "" if cm is None else round(cm, 1), int(stopped),
                     round(lr, 1), round(ld, 4), round(rr, 1), round(rd, 4),
                     "" if centre is None else
                     round(drift(reading, TARGET, ORDER) - centre, 3),
                     flips))
    control.stop()
finally:
    with open(name, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "red", "green", "blue", "clear", "seen", "turn",
                    "cm", "stopped",
                    "left_rpm", "left_duty", "right_rpm", "right_duty",
                    "error", "flips"])
        w.writerows(rows)
    print(f"wrote {name}, {len(rows)} rows")
    estop.close()
    sensor.close()
    colour.close()
    left.close()
    right.close()
    disconnect()
