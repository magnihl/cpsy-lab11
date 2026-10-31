#!/usr/bin/env python3
"""Run slave.py on this rover, with this rover's constants.

  python3 slave_rover.py --gap 60          options as slave.py
  python3 slave_rover.py --gap 60 --heading-scale 1.04 --dist-scale 0.98
  python3 slave_rover.py --wiggle-test --wiggle-min 0.08 --wiggle-max 0.12

slave.py is Mani's convoy slave, unchanged. Its constants are his car,
so they are set here before it runs. hw.py is this rover's adapter.

The two scales match this rover to the master rather than to reality.
Both cars count their own wheels and both are a few percent off, the
master's heading reads 91 to 99 percent of its real turning. If this
rover is off the same way it drives the master's real path, so the
master's map is scaled before it is followed:
  heading-scale  the master turned more than its map says: raise it
  dist-scale     the master went further than its map says: raise it
Tune them on a closed lap, the master back on its start mark. At exit
the turn and distance of the map and of this rover are printed; with
the error by eye, new scale = old x 360 / (360 - degrees short).
"""

import atexit
import csv
import math
import sys
import time

import config
import slave

# hw.py counts in hundredths of a cm, each wheel with its own calibration
slave.TICKS_PER_CM = 100.0
slave.TRACK_CM = config.TRACK_CM
# duty to cm/s, from the 13 Oct duty table (config.py), wheels off the
# ground, mean of both wheels, rpm x 693 x 0.02039 / 60. the floor is
# slower, the PI loop in slave.py makes up the difference
slave.SPEED_TABLE = ((0.0, 0.0), (0.05, 2.9), (0.10, 22.0), (0.15, 32.0),
                     (0.20, 37.8), (0.25, 41.4), (0.30, 44.3), (0.50, 49.8),
                     (1.0, 54.4))
# no speed limit: a target above the table's top just gives full power
slave.MAX_CM_S = float("inf")


def _take(flag, default):
    """Remove --flag VALUE from the command line, slave.py does not know it."""
    if flag not in sys.argv:
        return default
    i = sys.argv.index(flag)
    value = float(sys.argv[i + 1])
    del sys.argv[i:i + 2]
    return value


HEADING_SCALE = _take("--heading-scale", config.CONVOY_HEADING_SCALE)

DIST_SCALE = _take("--dist-scale", config.CONVOY_DIST_SCALE)

_hear = slave.Slave.hear


def _scaled_hear(self, packet, now):
    """The master's map, scaled to how this rover counts, before it is used."""
    if "s_cm" in packet:
        packet = dict(packet, s_cm=packet["s_cm"] * DIST_SCALE)
    if "heading" in packet:
        packet = dict(packet, heading=packet["heading"] * HEADING_SCALE)
    _hear(self, packet, now)


# every control step logged, to see why a run does not move: time since
# the last packet, the master's distance and speed, our gap and powers
_rows = []
_step = slave.Slave.step


def _logged_step(self, now, ticks):
    powers = _step(self, now, ticks)
    heard = "" if self.heard is None else round(now - self.heard, 3)
    _rows.append((round(now, 3), self.run, self.state, heard,
                  round(self.master_s, 2), round(self.master_speed(), 2),
                  round(self.path.heading, 4),
                  round(self.odometer.s, 2), round(self.odometer.heading, 4),
                  round(self.master_s - (self.odometer.s - self.gap), 2),
                  round(ticks[0], 1), round(ticks[1], 1),
                  round(powers[0], 3), round(powers[1], 3), HEADING_SCALE,
                  DIST_SCALE))
    return powers


def _write():
    name = f"slave_{time.strftime('%H%M%S')}.csv"
    with open(name, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "run", "state", "since_packet", "master_s",
                    "master_cm_s", "master_heading", "own_s", "own_heading",
                    "gap", "left_ticks", "right_ticks", "left_power",
                    "right_power", "heading_scale", "dist_scale"])
        w.writerows(_rows)
    print(f"wrote {name}, {len(_rows)} rows, heading scale {HEADING_SCALE}, "
          f"distance scale {DIST_SCALE}")
    # the last run that drove, for tuning the scales
    driven = [r for r in _rows if r[2] in ("follow", "stopping")]
    if driven:
        last = [r for r in driven if r[1] == driven[-1][1]][-1]
        print(f"last run: map {last[4]:.0f} cm, turned "
              f"{math.degrees(last[6]):+.0f} deg (scaled); this rover "
              f"{last[7]:.0f} cm, turned {math.degrees(last[8]):+.0f} deg")


# slave.py's wiggle steers the heading only. on this rover the wheels run
# at different speeds back and forth, so it crept backwards and ended off
# its start mark, 31 Oct. this one steers each wheel to its own position:
# for +5 degrees the left wheel goes back and the right forward by the
# same arc, and at the end both go back to where they started. same
# steps as slave.py's, so the master's "wiggle" means the same
# slave.py's 0.2 to 0.3 power is 38 to 44 cm/s here, so the wheels shot
# past the 0.6 cm a wheel moves for 5 degrees and hunted, 31 Oct. just
# above where the motors start (about 0.05) instead
WIGGLE_MIN_POWER = _take("--wiggle-min", 0.08)
WIGGLE_MAX_POWER = _take("--wiggle-max", 0.12)
WIGGLE_GAIN = 0.002       # power per hundredth of a cm off target
WIGGLE_TOLERANCE = 8.0    # hundredths of a cm, under half a degree
WIGGLE_SETTLE_S = 0.15


def _wheel_wiggle(ticks, motors, ticks_per_degree, on_step=None):
    l0, r0 = ticks()

    def turned():
        left, right = ticks()
        return ((right - r0) - (left - l0)) / ticks_per_degree

    try:
        for target in ([slave.WIGGLE_DEGREES, -slave.WIGGLE_DEGREES]
                       * slave.WIGGLE_TIMES + [0.0]):
            half = target * ticks_per_degree / 2
            goal = (l0 - half, r0 + half)
            end = time.monotonic() + slave.WIGGLE_STEP_S
            settled = None
            while time.monotonic() < end:
                errors = [g - t for g, t in zip(goal, ticks())]
                if all(abs(e) < WIGGLE_TOLERANCE for e in errors):
                    motors(0, 0)
                    settled = settled or time.monotonic()
                    if time.monotonic() - settled > WIGGLE_SETTLE_S:
                        break
                else:
                    settled = None
                    motors(*(0.0 if abs(e) < WIGGLE_TOLERANCE else
                             math.copysign(slave.clamp(
                                 abs(e) * WIGGLE_GAIN,
                                 WIGGLE_MIN_POWER,
                                 WIGGLE_MAX_POWER), e)
                             for e in errors))
                if on_step and on_step():
                    return turned()
                time.sleep(0.005)
            motors(0, 0)
    finally:
        motors(0, 0)
    left, right = ticks()
    print(f"wiggle: wheels {(left - l0) / 100:+.2f} / {(right - r0) / 100:+.2f}"
          f" cm from start")
    return turned()


if __name__ == "__main__" and "--wiggle-test" in sys.argv:
    # one wiggle on the spot, no master needed, for tuning the powers
    import hw
    per_degree = slave.TICKS_PER_CM * slave.TRACK_CM * math.pi / 180
    print(f"wiggle test, power {WIGGLE_MIN_POWER} to {WIGGLE_MAX_POWER}")
    left_over = _wheel_wiggle(hw.ticks, hw.motors, per_degree)
    print(f"ended {left_over:+.1f} degrees from the start heading")
    sys.exit()

if __name__ == "__main__":
    slave.wiggle = _wheel_wiggle
    slave.Slave.hear = _scaled_hear
    slave.Slave.step = _logged_step
    atexit.register(_write)
    print(f"heading scale {HEADING_SCALE}, distance scale {DIST_SCALE}")
    slave.main()
