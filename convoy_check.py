#!/usr/bin/env python3
"""Drive convoy_slave.py's Slave against a recorded master run, on the
laptop, with a simulated rover.

  python3 convoy_check.py ~/Projects/raunkerfi/linefollower/convoy/master-*.jsonl
  python3 convoy_check.py --gaps logs/<slave log>.csv master-*.jsonl

The recordings stay in Mani's repo, read from a local clone. The rover is
simulated: each wheel goes to the table speed of its power with a 0.15 s
lag, encoders exact. Prints how far the slave strays from the master's
path and how its gap holds, at a 60 and a 120 cm gap and with 20 percent
of packets lost.

The last line per run drops the packets the real track run missed: the
stretches of the master's path where master_s jumped in a slave log
(default: the 2 Nov run that went off the course). Random loss drops
single packets, the real WiFi lost whole stretches, mostly after the
hairpin, and that is what bends the slave's map.
"""

import csv
import json
import math
import random
import sys

import convoy_slave as cs

GAPS_LOG = "logs/convoy_slave_130407_run1793623050.csv"
GAP_MIN_CM = 5.0

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


def real_gaps(path, min_cm=GAP_MIN_CM):
    """Stretches of master path (from s, to s) a slave log never heard."""
    with open(path) as f:
        rows = list(csv.DictReader(f))
    gaps, prev = [], None
    for row in rows:
        s = float(row["master_s"])
        if prev is not None and s - prev > min_cm:
            gaps.append((prev, s))
        prev = s
    return gaps


HISTORY_S = 2.0  # how much of its recent path the master would resend; 1 s left 3 of 75 runs over 5 cm


def with_extras(packets, truth, master):
    """The packets as a changed master would send them. "xy": its own x, y,
    which it integrates anyway. "history": also its last HISTORY_S seconds
    of (s, heading). None: as Mani's master sends them now."""
    out = []
    for k, p in enumerate(packets):
        p = dict(p)
        if master in ("xy", "both"):
            p["x_cm"], p["y_cm"] = truth.point(p["s_cm"])
        if master in ("history", "both"):
            p["recent_path"] = [[q["s_cm"], q["heading"]] for q in packets[max(0, k - 200):k]
                                if p["t"] - q["t"] <= HISTORY_S]
        out.append(p)
    return out


def simulate(packets, gap, loss=0.0, seed=1, cut=(), arc=False, master=None):
    """One convoy run against a recording. Returns what happened, as lists:
    the true path, the slave's map of it, where the slave drove, and how far
    off the path and how far behind it was over time."""
    rng = random.Random(seed)
    truth = cs.Path()
    for p in packets:
        truth.add(p["s_cm"], p["heading"])
    t0 = packets[0]["t"]
    stream = [dict(p, type="map", run=1, running=True, t=p["t"] - t0)
              for p in with_extras(packets, truth, master)]
    last = stream[-1]
    stream += [dict(last, running=False, t=last["t"] + k * 0.05)
               for k in range(1, 21)]
    rover, slave = SimRover(), cs.Slave("check", gap, arc=arc)
    slave.hear({"type": "map", "run": 0, "running": False, "t": -1.0}, 0.0)
    slave.wants_wiggle = None
    now, i = 1.0, 0
    off, gaps, times, drove, dropped = [], [], [], [], []
    end = 1.0 + stream[-1]["t"] + 3.0
    while now < end:
        now += STEP_S
        while i < len(stream) and 1.0 + stream[i]["t"] <= now:
            p = stream[i]
            missed = any(a < p["s_cm"] < b for a, b in cut)
            if (rng.random() >= loss and not missed) or not p["running"]:
                slave.hear(stream[i], now)
            elif p["running"]:
                dropped.append(p["s_cm"])
            i += 1
        rover.powers = list(slave.step(now, rover.ticks()))
        rover.step(STEP_S)
        drove.append((slave.pose.x, slave.pose.y))
        here = slave.pose.s - gap
        if slave.state == "follow" and 20 < here < truth.s[-1] - 5:
            lo = bisect_left(truth.s, here - 40)
            hi = bisect_left(truth.s, here + 40)
            off.append(min(math.hypot(slave.pose.x - truth.x[k],
                                      slave.pose.y - truth.y[k])
                           for k in range(lo, hi)))
            gaps.append(slave.gap_now())
            times.append(now - 1.0)
    return {"truth": list(zip(truth.x, truth.y)),
            "map": list(zip(slave.path.x, slave.path.y)),
            "drove": drove, "off": off, "gaps": gaps, "times": times,
            "dropped": dropped, "end_gap": slave.gap_now(),
            "state": slave.state}


def drive(packets, gap, loss, seed=1, cut=(), arc=False):
    r = simulate(packets, gap, loss, seed, cut, arc)
    off = sorted(r["off"])
    lost = f"{loss:4.0%} lost" if not cut else f"{len(cut):2d} gaps "
    how = " arc" if arc else ""
    return (f"gap {gap:3.0f} cm, {lost}{how} | off path p95 "
            f"{off[int(len(off) * 0.95)]:4.1f} cm, max {off[-1]:4.1f} cm | "
            f"gap {min(r['gaps']):5.1f} to {max(r['gaps']):5.1f} cm | "
            f"stopped {r['end_gap']:5.1f} cm behind, {r['state']}")


def bisect_left(values, x):
    return cs.bisect.bisect_left(values, x)


if __name__ == "__main__":
    args = sys.argv[1:]
    gaps_log = GAPS_LOG
    if args[:1] == ["--gaps"]:
        gaps_log, args = args[1], args[2:]
    if not args:
        sys.exit(__doc__)
    cut = real_gaps(gaps_log)
    print(f"real gaps from {gaps_log.split('/')[-1]}: {len(cut)} stretches, "
          f"first at {cut[0][0]:.0f} to {cut[0][1]:.0f} cm" if cut else
          f"no gaps over {GAP_MIN_CM:.0f} cm in {gaps_log}")
    for name in args:
        packets = [json.loads(line) for line in open(name)]
        print(name.split("/")[-1])
        for gap, loss in ((60, 0.0), (60, 0.2), (120, 0.0)):
            print("  " + drive(packets, gap, loss))
        if cut:
            print("  " + drive(packets, 60, 0.0, cut=cut))
            print("  " + drive(packets, 60, 0.0, cut=cut, arc=True))
