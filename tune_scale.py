#!/usr/bin/env python3
"""Heading scale for slave_rover.py from a closed lap.

  python3 tune_scale.py logs/slave_HHMMSS.csv LAPS

The master drove LAPS full laps and stopped on its start mark, so it
really turned LAPS x 360 degrees. Its map says something else, by its
own counting error. The scale that makes the map turn the real amount
is LAPS x 360 / map degrees. It assumes this rover's own heading is
right (TRACK_CM from the circle calibration).

Runs on the laptop. Uses the last run in the log that drove.
"""

import csv
import math
import sys

if len(sys.argv) < 3:
    sys.exit("usage: python3 tune_scale.py SLAVE_LOG.csv LAPS")
rows = list(csv.DictReader(open(sys.argv[1])))
laps = float(sys.argv[2])
driven = [r for r in rows if r["state"] in ("follow", "stopping")]
if not driven:
    sys.exit("no run that drove in this log")
run = [r for r in rows if r["run"] == driven[-1]["run"]]
last = run[-1]
used = float(last.get("heading_scale") or 1.0)
# the log holds the map after scaling, undo it to get the master's own
raw = math.degrees(float(last["master_heading"])) / used
own = math.degrees(float(last["own_heading"]))
scale = laps * 360 / abs(raw)
print(f"run {last['run']}: the master's map turned {raw:+.0f} deg for "
      f"{laps:g} laps ({laps * 360:.0f} deg real)")
print(f"this rover turned {own:+.0f} deg by its encoders with scale {used}")
print(f"heading scale: {scale:.3f}   use --heading-scale {scale:.3f}")
