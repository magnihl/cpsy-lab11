#!/usr/bin/env python3
"""Run slave.py on this rover, with this rover's constants.

  python3 slave_rover.py --gap 60          options as slave.py

slave.py is Mani's convoy slave, unchanged. Its constants are his car,
so they are set here before it runs. hw.py is this rover's adapter.
"""

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
# the master's convoy speed peaks near 39 cm/s. kept at slave.py's 45
# until the floor top speed is measured
slave.MAX_CM_S = 45.0

if __name__ == "__main__":
    slave.main()
