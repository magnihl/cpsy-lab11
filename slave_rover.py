#!/usr/bin/env python3
"""Run slave.py on this rover, with this rover's constants.

  python3 slave_rover.py --gap 60          options as slave.py

slave.py is Mani's convoy slave, unchanged. Its constants are his car,
so they are set here before it runs. hw.py is this rover's adapter.
"""

import atexit
import csv
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

# every control step logged, to see why a run does not move: time since
# the last packet, the master's distance and speed, our gap and powers
_rows = []
_step = slave.Slave.step


def _logged_step(self, now, ticks):
    powers = _step(self, now, ticks)
    heard = "" if self.heard is None else round(now - self.heard, 3)
    _rows.append((round(now, 3), self.run, self.state, heard,
                  round(self.master_s, 2), round(self.master_speed(), 2),
                  round(self.odometer.s, 2),
                  round(self.master_s - (self.odometer.s - self.gap), 2),
                  round(ticks[0], 1), round(ticks[1], 1),
                  round(powers[0], 3), round(powers[1], 3)))
    return powers


def _write():
    name = f"slave_{time.strftime('%H%M%S')}.csv"
    with open(name, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "run", "state", "since_packet", "master_s",
                    "master_cm_s", "own_s", "gap", "left_ticks",
                    "right_ticks", "left_power", "right_power"])
        w.writerows(_rows)
    print(f"wrote {name}, {len(_rows)} rows")


if __name__ == "__main__":
    slave.Slave.step = _logged_step
    atexit.register(_write)
    slave.main()
