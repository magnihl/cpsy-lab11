#!/usr/bin/env python3
"""Can this Pi count both encoder wires (quadrature)? Lift the rover.

  python3 quadtest.py            power 0.3, as encodertest.py
  python3 quadtest.py 0.6

Each wheel turns forward for 1.5 s, then both together. Prints each
wheel's signed count (forward should be positive, about 4 per pulse of
one wire), edges a second, edges lost, and how busy the CPU was.
"""

import sys
import time

import config
from estop import EStop
from quad import QuadEncoders
from rover import Rover

POWER = float(sys.argv[1]) if len(sys.argv) > 1 else 0.3
PINS = {"left": tuple(int(p[4:]) for p in config.ENCODER_B),
        "right": tuple(int(p[4:]) for p in config.ENCODER_A)}

input("Lift the rover so both wheels spin freely, then press Enter. ")
enc = QuadEncoders(PINS)
estop = EStop()
rover = Rover(estop)
try:
    estop.arm()
    for name, motors in (("left", (rover.left_motor,)),
                         ("right", (rover.right_motor,)),
                         ("both", (rover.left_motor, rover.right_motor))):
        before, events, missed = dict(enc.counts), enc.events, enc.missed
        wall, cpu = time.monotonic(), time.process_time()
        for m in motors:
            m.forward(POWER)
        time.sleep(1.5)
        for m in motors:
            m.stop()
        time.sleep(0.8)
        seconds = time.monotonic() - wall
        busy = (time.process_time() - cpu) / seconds
        moved = {w: enc.counts[w] - before[w] for w in enc.counts}
        print(f"{name:5} running: left {moved['left']:+6}  right "
              f"{moved['right']:+6}  {(enc.events - events) / 1.5:6.0f} "
              f"edges/s  lost {enc.missed - missed}  cpu {busy:.0%}")
finally:
    rover.stop()
    estop.close()
    enc.close()
