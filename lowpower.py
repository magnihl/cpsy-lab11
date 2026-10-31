#!/usr/bin/env python3
"""Drive straight through hw.py at a low power, as slave.py does, and
print the encoder distance, to check it against a tape measure.

  python3 lowpower.py POWER SECONDS [GLITCH_US]
  python3 lowpower.py 0.07 5            filter off
  python3 lowpower.py 0.07 5 50         spikes under 50 us ignored

The power is set 50 times a second with a little noise around POWER,
like slave.py's PI loop does, so the motors run in the same jerks.
"""

import random
import sys
import time

import config

POWER = float(sys.argv[1]) if len(sys.argv) > 1 else 0.07
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
if len(sys.argv) > 3:
    # before hw imports rover, so the wheels are made with it
    config.ENCODER_GLITCH_US = int(sys.argv[3])

import hw

start = hw.ticks()
end = time.monotonic() + SECONDS
while time.monotonic() < end:
    p = POWER + random.uniform(-0.02, 0.02)
    hw.motors(p, p)
    time.sleep(0.02)
hw.motors(0, 0)
time.sleep(0.8)
left, right = (b - a for a, b in zip(start, hw.ticks()))
print(f"power {POWER} for {SECONDS:.0f} s, glitch filter "
      f"{config.ENCODER_GLITCH_US} us: encoders left {left / 100:.1f} cm, "
      f"right {right / 100:.1f} cm, mean {(left + right) / 200:.1f} cm")
