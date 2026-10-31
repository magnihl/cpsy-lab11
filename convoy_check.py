#!/usr/bin/env python3
"""Drive convoy_slave.py's Slave against a recorded master run, on the
laptop, with a simulated rover.

  python3 convoy_check.py ~/Projects/raunkerfi/linefollower/convoy/master-*.jsonl

The recordings stay in Mani's repo, read from a local clone. The rover is
simulated: each wheel goes to the table speed of its power with a 0.15 s
lag, encoders exact. Prints how far the slave strays from the master's
path and how its gap holds, at a 60 and a 120 cm gap and with 20 percent
of packets lost.
"""

import json
import math
import random
import sys

import convoy_slave as cs

STEP_S = 1 / cs.CONTROL_HZ
LAG_S = 0.15


def table_speed(power):
    for (p0, v0), (p1, v1) in zip(cs.SPEED_TABLE, cs.SPEED_TABLE[1:]):
        if abs(power) <= p1:
            return math.copysign(v0 + (v1 - v0) * (abs(power) - p0)
                                 / (p1 - p0), power)
    return math.copysign(cs.SPEED_TABLE[-1][1], power)


class SimRover:
    def __init__(self):
        self.powers, self.speeds, self.cm = [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]

    def step(self, dt):
        for i in range(2):
            self.speeds[i] += ((table_speed(self.powers[i]) - self.speeds[i])
                               * min(1.0, dt / LAG_S))
            self.cm[i] += self.speeds[i] * dt

    def ticks(self):
        return tuple(c * cs.TICKS_PER_CM for c in self.cm)


def drive(packets, gap, loss, seed=1):
    rng = random.Random(seed)
    truth = cs.Path()
    for p in packets:
        truth.add(p["s_cm"], p["heading"])
    t0 = packets[0]["t"]
    stream = [dict(p, type="map", run=1, running=True, t=p["t"] - t0)
              for p in packets]
    last = stream[-1]
    stream += [dict(last, running=False, t=last["t"] + k * 0.05)
               for k in range(1, 21)]
    rover, slave = SimRover(), cs.Slave("check", gap)
    slave.hear({"type": "map", "run": 0, "running": False, "t": -1.0}, 0.0)
    slave.wants_wiggle = None
    now, i, off, gaps = 1.0, 0, [], []
    end = 1.0 + stream[-1]["t"] + 3.0
    while now < end:
        now += STEP_S
        while i < len(stream) and 1.0 + stream[i]["t"] <= now:
            if rng.random() >= loss or not stream[i]["running"]:
                slave.hear(stream[i], now)
            i += 1
        rover.powers = list(slave.step(now, rover.ticks()))
        rover.step(STEP_S)
        here = slave.pose.s - gap
        if slave.state == "follow" and 20 < here < truth.s[-1] - 5:
            lo = bisect_left(truth.s, here - 40)
            hi = bisect_left(truth.s, here + 40)
            off.append(min(math.hypot(slave.pose.x - truth.x[k],
                                      slave.pose.y - truth.y[k])
                           for k in range(lo, hi)))
            gaps.append(slave.gap_now())
    off.sort()
    return (f"gap {gap:3.0f} cm, {loss:4.0%} lost | off path p95 "
            f"{off[int(len(off) * 0.95)]:4.1f} cm, max {off[-1]:4.1f} cm | "
            f"gap {min(gaps):5.1f} to {max(gaps):5.1f} cm | stopped "
            f"{slave.gap_now():5.1f} cm behind, {slave.state}")


def bisect_left(values, x):
    return cs.bisect.bisect_left(values, x)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for name in sys.argv[1:]:
        packets = [json.loads(line) for line in open(name)]
        print(name.split("/")[-1])
        for gap, loss in ((60, 0.0), (60, 0.2), (120, 0.0)):
            print("  " + drive(packets, gap, loss))
