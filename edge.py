#!/usr/bin/env python3
"""Follow the edge of a black line, the way Mani's line follower does.

The sensor sits half on the line and half on the floor. edge_read gives
0.5 there, more when it drifts onto the floor, less when it drifts onto
the line, and the difference steers the rover back (PD, like his sim).
Turns slow or stop one wheel, never reverse it.

  python3 edge.py                  20 rpm, 20 s, left edge, stop at 30 cm
  python3 edge.py 25 60            25 rpm for 60 s
  python3 edge.py 20 20 right      the right edge of the line
  python3 edge.py 20 20 left 0     distance stop off, for testing

Start with the sensor on the chosen edge. Every loop is logged to
edge_<rpm>_<side>_<time>.csv. Ctrl+C stops it, so does the e-stop button.
"""

import csv
import sys
import time

import config
from colour import ColourSensor, edge_read
from distance import Distance
from estop import EStop
from rover import disconnect, Rover, SpeedController, Wheel

RPM = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
SIDE = sys.argv[3] if len(sys.argv) > 3 else "left"
STOP_CM = float(sys.argv[4]) if len(sys.argv) > 4 else 30.0

# Mani's sim steers with kp 0.7 and kd 0.035 on top of a 0.37 power base;
# the same gains scaled to this rover's rpm base
KP = 1.9 * RPM
KD = 0.095 * RPM
# the sim needs a fresh reading every 20 ms or less; the wheel controller
# keeps its own slower interval, its encoder speed gets noisy any faster
STEER_INTERVAL = 0.01

if config.EDGE_BLACK is None or config.EDGE_WHITE is None:
    sys.exit("set EDGE_BLACK and EDGE_WHITE in config.py first, "
             "measured with colourtest.py on the line and on the floor")
if SIDE not in ("left", "right"):
    sys.exit(f"{SIDE} is not left or right")
# on the left edge more white means the rover drifted left, so it turns
# right: left wheel faster. the right edge is the mirror image
SIGN = 1 if SIDE == "left" else -1

estop = EStop()
rover = Rover(estop)
left = Wheel(config.LEFT_ENCODER)
right = Wheel(config.RIGHT_ENCODER)
control = SpeedController(rover, left, right)
sensor = Distance()
colour = ColourSensor()

print(f"edge {SIDE} at {RPM:.0f} rpm ({config.rpm_to_cm_s(RPM):.0f} cm/s) "
      f"for {DURATION:.0f} s, kp {KP:.1f} kd {KD:.2f}, "
      f"{'distance stop off' if STOP_CM <= 0 else f'stop at {STOP_CM:.0f} cm'}")

name = f"edge_{RPM:.0f}_{SIDE}_{time.strftime('%H%M%S')}.csv"
rows = []
stopped = False
cm = None
wheels = ((0.0, 0.0), (0.0, 0.0))

try:
    estop.arm()
    start = time.monotonic()
    last_t = start
    last_control = start
    last_err = edge_read(colour.read()) - 0.5
    while time.monotonic() - start < DURATION:
        time.sleep(STEER_INTERVAL)
        now = time.monotonic()
        reading = colour.read()
        read = edge_read(reading)
        err = read - 0.5
        dt = now - last_t
        der = (err - last_err) / dt if dt > 0 else 0.0
        last_err, last_t = err, now
        turn = SIGN * (KP * err + KD * der)

        if not stopped:
            # a turn slows or stops the inner wheel, it never runs backwards
            control.target(max(0.0, RPM + turn), max(0.0, RPM - turn))

        # wheel control and the distance check at their usual pace
        if now - last_control >= config.CONTROL_INTERVAL:
            last_control = now
            volts, cm = sensor.read()
            # latched, as in follow.py: once stopped it stays stopped
            if STOP_CM > 0 and not stopped and cm is not None and cm <= STOP_CM:
                stopped = True
                control.stop()
                print(f"{now - start:5.2f} s  STOP  {cm:.0f} cm")
            wheels = control.update()

        (lr, ld), (rr, rd) = wheels
        rows.append((round(now - start, 3), *reading, round(read, 3),
                     round(err, 3), round(turn, 1),
                     "" if cm is None else round(cm, 1), int(stopped),
                     round(lr, 1), round(ld, 4), round(rr, 1), round(rd, 4)))
    control.stop()
finally:
    with open(name, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "red", "green", "blue", "clear", "read", "error",
                    "turn", "cm", "stopped",
                    "left_rpm", "left_duty", "right_rpm", "right_duty"])
        w.writerows(rows)
    print(f"wrote {name}, {len(rows)} rows")
    estop.close()
    sensor.close()
    colour.close()
    left.close()
    right.close()
    disconnect()
