import json
import math
import random
import sys
from pathlib import Path as FilePath

sys.path.insert(0, str(FilePath(__file__).resolve().parent))

import slave

STEP_S = 1 / slave.CONTROL_HZ
FINAL_S = 1.0


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def off_path(truth, x, y, s):
    lo, hi = slave.bisect.bisect_left(truth.s, s - 40), slave.bisect.bisect_right(truth.s, s + 40)
    return min((math.hypot(x - truth.x[i], y - truth.y[i]) for i in range(lo, hi)), default=None)


def drive(packets, gap, loss, seed=1):
    rng = random.Random(seed)
    truth = slave.Path()
    for p in packets:
        truth.add(p["s_cm"], p["heading"])
    last = packets[-1]
    final = [dict(last, t=last["t"] + k * 0.05, running=False) for k in range(1, int(FINAL_S / 0.05) + 1)]
    idle_from = final[-1]["t"]
    idle = [{"t": idle_from + k * 0.2, "running": False, "s_cm": 0.0, "heading": 0.0} for k in range(1, 21)]
    stream = [dict(p, type="map", run=1, running=True) for p in packets] + [dict(p, type="map", run=1) for p in final + idle]
    start = 1.0
    clock = Clock()
    car = slave.FakeCar(clock)
    follower = slave.Slave("check", gap)
    follower.hear({"type": "map", "run": 0, "running": False, "s_cm": 0.0, "heading": 0.0, "t": 0.0}, 0.0)
    follower.wants_wiggle = None
    i, off, gaps, states = 0, [], [], []
    while clock.now < start + stream[-1]["t"]:
        clock.now += STEP_S
        while i < len(stream) and start + stream[i]["t"] <= clock.now:
            if rng.random() >= loss:
                follower.hear(stream[i], clock.now)
            i += 1
        car.motors(*follower.step(clock.now, car.ticks()))
        here = follower.odometer.s - gap
        if follower.state == "follow" and 20 < here < truth.s[-1] - 5:
            d = off_path(truth, follower.odometer.x, follower.odometer.y, here)
            if d is not None:
                off.append(d)
            gaps.append(follower.master_s - here)
        if not states or states[-1] != follower.state:
            states.append(follower.state)
    off.sort()
    end_gap = follower.master_s - (follower.odometer.s - gap)
    return (f"gap {gap:3.0f} cm, {loss:4.0%} lost | off the master's path p95 {off[int(len(off) * 0.95)]:4.1f} cm, "
            f"max {off[-1]:4.1f} cm | gap while following {min(gaps):5.1f} to {max(gaps):5.1f} cm | "
            f"stopped {end_gap:5.1f} cm behind | {' > '.join(states)}")


def main():
    if len(sys.argv) < 2:
        raise SystemExit("usage: python3 slave_check.py REPLAY.jsonl [...]")
    for name in sys.argv[1:]:
        packets = [json.loads(line) for line in open(name)]
        print(name)
        for gap, loss in ((60, 0.0), (60, 0.2), (120, 0.0)):
            print("  " + drive(packets, gap, loss))


if __name__ == "__main__":
    main()
