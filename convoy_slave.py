#!/usr/bin/env python3
"""Convoy slave for this rover, speaking the master protocol of Mani's
convoy (github.com/ManiTiTz/raunkerfi, linefollower/convoy/README.md).

  python3 convoy_slave.py --gap 60
  python3 convoy_slave.py --gap ir             start gap from the IR sensor
  python3 convoy_slave.py --gap 60 --heading-scale 1.01
  python3 convoy_slave.py --wiggle-test        one wiggle, no master
  python3 convoy_slave.py --wiggle-test --wiggle-power 0.12 --wiggle-coast 0.2

The master streams its map on UDP 5005: one JSON object per packet with
its run number, whether it is driving, and its distance s_cm and heading
(radians, positive left) since the start of the run. This rover starts
gap cm straight behind the master, rebuilds the master's path as points
(s, x, y) and drives it by pure pursuit, its speed set by the gap along
the path. It joins only a run whose start it heard, stops if the master
goes quiet for LOST_S and carries on when it is heard again, and brakes
to its gap when the run ends. A "wiggle" command turns it 5 degrees left,
then right, twice, on the spot, and back to straight, as the master does.
Status goes to the laptop's live view on UDP 5006.

The two scales match this rover to the master's counting: both count
their own wheels and are off by a few percent, the master's heading
reads 91 to 99 percent of its real turning. tune_scale.py finds the
heading scale from a closed lap.

Every control step of a run is logged to convoy_slave_<time>_run<N>.csv,
written as the run ends. convoy-slave.service starts it at boot.
"""

import argparse
import bisect
import collections
import csv
import json
import math
import signal
import socket
import time

import config

MAP_PORT = 5005
LIVE_PORT = 5006
CONTROL_HZ = 50
STATUS_HZ = 10
# the master is lost after this long without a packet while driving
LOST_S = 0.5
# silent for this long and the next packet counts as hearing it anew
RECONNECT_S = 2.0
# a run counts as heard from its start if its first packet is under this
START_CM = 1.0
SPEED_WINDOW_S = 0.5

# hw.py ticks are hundredths of a cm, each wheel with its own calibration
TICKS_PER_CM = 100.0
LOOKAHEAD_CM = 10.0
# cm/s added per cm the path gap is above the set gap
GAP_GAIN = 1.0
# closer than this share of the set gap stops it
MIN_GAP_SHARE = 0.5
STOP_CM_S = 2.0
# power to cm/s, 13 Oct duty table in config.py, wheels off the ground,
# mean of both, rpm x 693 x 0.02039 / 60. slower on the floor, the PI
# loop below makes up the difference
SPEED_TABLE = ((0.0, 0.0), (0.05, 2.9), (0.10, 22.0), (0.15, 32.0),
               (0.20, 37.8), (0.25, 41.4), (0.30, 44.3), (0.50, 49.8),
               (1.0, 54.4))
if config.FLOOR_SPEED_TABLE:
    SPEED_TABLE = config.FLOOR_SPEED_TABLE
MAX_CM_S = SPEED_TABLE[-1][1]
KP = 0.004
KI = 0.02
I_MAX = 10.0

WIGGLE_DEGREES = 5.0
WIGGLE_TIMES = 2
WIGGLE_STEP_S = 1.0
WIGGLE_SETTLE_S = 0.05
# the master wiggles at 0.2 to 0.3. at 0.10 this rover fell behind it,
# 31 Oct. the early stop and the nudges keep it on target
WIGGLE_POWER = 0.20
# the nudges for a wheel that stopped short, gentle so they do not overshoot
WIGGLE_NUDGE_POWER = 0.10
# hundredths of a cm, a wheel this close to its goal stops
WIGGLE_TOLERANCE = 5.0
# how long a wheel keeps rolling after its power goes, roughly
WIGGLE_COAST_S = 0.15
# one nudge for a wheel that stopped short
WIGGLE_NUDGE_S = 0.015


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def table_power(speed):
    """Power for a speed, by straight lines through SPEED_TABLE."""
    for (p0, v0), (p1, v1) in zip(SPEED_TABLE, SPEED_TABLE[1:]):
        if speed <= v1:
            return p0 + (p1 - p0) * (speed - v0) / (v1 - v0)
    return SPEED_TABLE[-1][0]


class Path:
    """The master's path as points (s, x, y) in its start frame.

    arc: bridge each step as a circular arc instead of a straight chord.
    The same for small steps; across a lost stretch in a curve the chord
    goes too far, the arc does not. Off until convoy_check shows it helps.
    """

    def __init__(self, arc=False):
        self.s, self.x, self.y = [], [], []
        self.heading = 0.0
        self.arc = arc

    def add(self, s, heading):
        if self.s and s <= self.s[-1]:
            return
        if self.s:
            # integrate the step along the mean heading
            ds = s - self.s[-1]
            mid = (heading + self.heading) / 2
            if self.arc:
                # an arc of length ds turning by turn ends ds * sinc(turn / 2) away
                half = (heading - self.heading) / 2
                ds *= math.sin(half) / half if abs(half) > 1e-9 else 1.0
            x = self.x[-1] + ds * math.cos(mid)
            y = self.y[-1] + ds * math.sin(mid)
        else:
            x, y = 0.0, 0.0
        self.s.append(s)
        self.x.append(x)
        self.y.append(y)
        self.heading = heading

    def add_at(self, s, x, y, heading):
        """A point whose position the master sent itself, nothing integrated."""
        if self.s and s <= self.s[-1]:
            return
        self.s.append(s)
        self.x.append(x)
        self.y.append(y)
        self.heading = heading

    def point(self, s):
        """Where the master was at distance s, between two points by a
        straight line. Before its start, straight ahead of it."""
        if s <= 0 or not self.s:
            return s, 0.0
        i = bisect.bisect_right(self.s, s)
        if i >= len(self.s):
            return self.x[-1], self.y[-1]
        f = (s - self.s[i - 1]) / (self.s[i] - self.s[i - 1])
        return (self.x[i - 1] + f * (self.x[i] - self.x[i - 1]),
                self.y[i - 1] + f * (self.y[i] - self.y[i - 1]))


class Pose:
    """This rover's distance, heading and x, y from its own wheels."""

    def __init__(self, gap):
        self.x, self.y = -gap, 0.0
        self.s = self.heading = 0.0
        self.origin = None

    def update(self, ticks):
        if self.origin is None:
            self.origin = ticks
        left, right = ((t - o) / TICKS_PER_CM for t, o in
                       zip(ticks, self.origin))
        s = (left + right) / 2
        heading = (right - left) / config.TRACK_CM
        mid = (heading + self.heading) / 2
        self.x += (s - self.s) * math.cos(mid)
        self.y += (s - self.s) * math.sin(mid)
        self.s, self.heading = s, heading


class WheelSpeed:
    """One PI loop per wheel on its speed, around the table's power."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.last = None
        self.integral = [0.0, 0.0]
        self.speeds = [0.0, 0.0]

    def powers(self, now, ticks, targets):
        if self.last is not None:
            dt = max(now - self.last[0], 1e-3)
            self.speeds = [(t - was) / TICKS_PER_CM / dt
                           for t, was in zip(ticks, self.last[1])]
        else:
            dt = 0.0
        self.last = (now, ticks)
        out = []
        for i, target in enumerate(targets):
            if target <= 0:
                self.integral[i] = 0.0
                out.append(0.0)
                continue
            error = target - self.speeds[i]
            self.integral[i] = clamp(self.integral[i] + error * dt,
                                     -I_MAX, I_MAX)
            out.append(clamp(table_power(target) + KP * error
                             + KI * self.integral[i], 0.0, 1.0))
        return out


class Slave:
    """What the protocol asks of a slave, without any hardware."""

    def __init__(self, name, gap, heading_scale=1.0, dist_scale=1.0,
                 target=None, arc=False):
        # gap: where it starts behind the master, axle to axle, which places
        # it on the master's path. target: the gap it steers toward
        self.name, self.gap = name, gap
        self.target = gap if target is None else target
        self.heading_scale, self.dist_scale = heading_scale, dist_scale
        self.arc = arc
        self.state, self.run, self.heard = "waiting", None, None
        self.done = set()
        self.wants_wiggle = None
        self.live_ip = None
        self.wheels = WheelSpeed()
        self.new_run(None)

    def new_run(self, run):
        self.run = run
        self.path = Path(arc=self.arc)
        self.recent = collections.deque()
        self.master_s = 0.0
        self.pose = Pose(self.gap)
        self.wheels.reset()

    @property
    def driving(self):
        # lost is still inside the run: it waits for the master to be heard
        # again and carries on, the packets hold totals so nothing is missed
        return self.state in ("follow", "stopping", "lost")

    def hear(self, packet, now):
        """One map packet from the master."""
        anew = self.heard is None or now - self.heard > RECONNECT_S
        self.heard = now
        self.live_ip = packet.get("live")
        cmd = packet.get("cmd")
        if cmd and cmd.get("id") not in self.done:
            self.done.add(cmd.get("id"))
            if cmd.get("wiggle") in ("all", self.name) and not self.driving:
                self.wants_wiggle = f"told to (wiggle {cmd.get('wiggle')})"
        run = packet.get("run")
        running = packet.get("running")
        s = packet.get("s_cm", 0.0) * self.dist_scale
        heading = packet.get("heading", 0.0) * self.heading_scale

        if not running and not self.driving:
            self.run = run
        if running and run != self.run:
            if s < START_CM and self.state not in ("follow", "stopping"):
                self.new_run(run)
                self.path.add(0.0, 0.0)
                self.state, self.wants_wiggle = "follow", None
            else:
                # joined after the start: sit this run out
                self.run, self.state = run, "missed"
        if run == self.run and self.state == "lost" and running:
            print("heard the master again, carrying on")
            self.state = "follow"
        if run == self.run and self.driving:
            # optional, if the master sends it: its last second of (s, heading),
            # which fills a stretch of lost packets in the right shape
            for old_s, old_heading in packet.get("recent_path", ()):
                old_s *= self.dist_scale
                if self.master_s < old_s < s:
                    self.path.add(old_s, old_heading * self.heading_scale)
            if s > self.master_s:
                self.master_s = s
                if "x_cm" in packet and "y_cm" in packet:
                    # optional: the master's own position, so a lost stretch
                    # leaves a straight gap in the map instead of bending it
                    self.path.add_at(s, packet["x_cm"] * self.dist_scale,
                                     packet["y_cm"] * self.dist_scale, heading)
                else:
                    self.path.add(s, heading)
            t = packet.get("t", 0.0)
            if running and (not self.recent or t > self.recent[-1][0]):
                self.recent.append((t, self.master_s))
                while t - self.recent[0][0] > SPEED_WINDOW_S:
                    self.recent.popleft()
            if not running and self.state == "follow":
                self.state = "stopping"
        if not running and self.state in ("waiting", "missed", "lost"):
            self.state = "idle"
        if anew and not running and not self.driving:
            self.wants_wiggle = "heard the master"

    def master_speed(self):
        if self.state == "stopping" or len(self.recent) < 2:
            return 0.0
        (t0, s0), (t1, s1) = self.recent[0], self.recent[-1]
        return (s1 - s0) / (t1 - t0) if t1 > t0 else 0.0

    def gap_now(self):
        return self.master_s - (self.pose.s - self.gap)

    def step(self, now, ticks):
        """Wheel powers for this control step."""
        if not self.driving:
            self.wheels.reset()
            return 0.0, 0.0
        # the wheels still roll on after a stop, so the pose keeps counting
        self.pose.update(ticks)
        if self.heard is None or now - self.heard > LOST_S:
            self.state = "lost"
            self.wheels.reset()
            return 0.0, 0.0
        gap = self.gap_now()
        speed = clamp(self.master_speed() + GAP_GAIN * (gap - self.target),
                      0.0, MAX_CM_S)
        if gap < self.target * MIN_GAP_SHARE:
            speed = 0.0
        if self.state == "stopping" and speed < STOP_CM_S:
            self.state = "idle"
            self.wheels.reset()
            return 0.0, 0.0
        # pure pursuit: the arc through the point LOOKAHEAD_CM ahead on the
        # master's path, curvature 2 sin(error) / distance
        here = self.pose.s - self.gap
        tx, ty = self.path.point(here + LOOKAHEAD_CM)
        dx, dy = tx - self.pose.x, ty - self.pose.y
        error = wrap(math.atan2(dy, dx) - self.pose.heading)
        half = math.sin(error) / max(math.hypot(dx, dy), 1.0) * config.TRACK_CM
        left, right = speed * (1 - half), speed * (1 + half)
        top = max(left, right)
        if top > MAX_CM_S:
            left, right = left * MAX_CM_S / top, right * MAX_CM_S / top
        targets = (max(left, 0.0), max(right, 0.0))
        return tuple(self.wheels.powers(now, ticks, targets))

    def status(self, wiggling=False):
        return {"type": "slave", "name": self.name, "run": self.run,
                "state": "wiggle" if wiggling else self.state,
                "path_cm": round(self.pose.s - self.gap, 1),
                "x": round(self.pose.x, 1), "y": round(self.pose.y, 1),
                "heading": round(self.pose.heading, 4),
                "gap_cm": round(self.gap_now(), 1),
                "set_gap_cm": round(self.target, 1)}


def wiggle(ticks, motors, interrupted=None):
    """The master's wiggle: 5 degrees left, 5 right, twice, then straight.

    Each move: both wheels drive at WIGGLE_POWER toward their own goals
    and stop a little early, by how far they still roll. A wheel still
    short of its goal then gets short nudges the same way, never back, so
    it cannot hunt. Every goal is measured from the start, so errors do
    not add up. Returns the degrees left over from the start heading.
    """
    per_degree = TICKS_PER_CM * config.TRACK_CM * math.pi / 180
    l0, r0 = ticks()
    try:
        for target in [WIGGLE_DEGREES, -WIGGLE_DEGREES] * WIGGLE_TIMES + [0.0]:
            half = target * per_degree / 2
            goals = (l0 - half, r0 + half)
            now = ticks()
            # which way each wheel has to go, 0 if it is there already
            ways = [0 if abs(g - t) < WIGGLE_TOLERANCE else
                    math.copysign(1, g - t) for g, t in zip(goals, now)]
            first_ways = list(ways)
            end = time.monotonic() + WIGGLE_STEP_S
            then, was = time.monotonic(), now
            while any(ways) and time.monotonic() < end:
                now = ticks()
                dt = max(time.monotonic() - then, 1e-3)
                for i in range(2):
                    # the wheel rolls on about speed x WIGGLE_COAST_S after
                    # its power goes, so it stops that much early
                    coast = abs(now[i] - was[i]) / dt * WIGGLE_COAST_S
                    if ways[i] and ((goals[i] - now[i]) * ways[i]
                                    < WIGGLE_TOLERANCE + coast):
                        ways[i] = 0
                then, was = time.monotonic(), now
                motors(*(w * WIGGLE_POWER for w in ways))
                if interrupted and interrupted():
                    return 0.0
                time.sleep(0.005)
            motors(0, 0)
            time.sleep(WIGGLE_SETTLE_S)
            # nudge a wheel that stopped short, the same way only
            end = time.monotonic() + WIGGLE_STEP_S / 2
            while time.monotonic() < end:
                now = ticks()
                short = [math.copysign(1, g - t)
                         if abs(g - t) > WIGGLE_TOLERANCE else 0
                         for g, t in zip(goals, now)]
                # only the way it was going, a wheel past its goal stays
                short = [w if w == way or not way else 0
                         for w, way in zip(short, first_ways)]
                if not any(short):
                    break
                motors(*(w * WIGGLE_NUDGE_POWER for w in short))
                time.sleep(WIGGLE_NUDGE_S)
                motors(0, 0)
                time.sleep(WIGGLE_SETTLE_S)
                if interrupted and interrupted():
                    return 0.0
    finally:
        motors(0, 0)
    left, right = ticks()
    print(f"wiggle: wheels {(left - l0) / 100:+.2f} / {(right - r0) / 100:+.2f}"
          f" cm from start")
    return ((right - r0) - (left - l0)) / per_degree


LOG_HEADER = ["t", "run", "state", "since_packet", "master_s", "master_cm_s",
              "master_heading", "own_s", "own_heading", "gap", "left_power",
              "right_power", "heading_scale", "dist_scale"]


def write_log(rows, run):
    """One csv per run, written as the run ends, so switching the rover
    off loses nothing."""
    name = f"convoy_slave_{time.strftime('%H%M%S')}_run{run}.csv"
    with open(name, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(LOG_HEADER)
        w.writerows(rows)
    print(f"wrote {name}, {len(rows)} rows")


def _stop_on_signal(number, _frame):
    # systemd stops the service with SIGTERM, closing ssh sends SIGHUP:
    # both stop it the way Ctrl+C does, motors off and the log written
    raise KeyboardInterrupt


def main():
    global WIGGLE_POWER, WIGGLE_COAST_S
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--name", default=socket.gethostname().split(".")[0])
    parser.add_argument("--gap", default="ir",
                        help="cm behind the master along its path, axle to "
                        "axle, or ir: measured before each run")
    parser.add_argument("--live", help="laptop running the live view, "
                        "default: the address in the master's packets")
    parser.add_argument("--heading-scale", type=float,
                        default=config.CONVOY_HEADING_SCALE)
    parser.add_argument("--dist-scale", type=float,
                        default=config.CONVOY_DIST_SCALE)
    parser.add_argument("--wiggle-test", action="store_true",
                        help="one wiggle on the spot and exit")
    parser.add_argument("--wiggle-power", type=float, default=WIGGLE_POWER)
    parser.add_argument("--wiggle-coast", type=float, default=WIGGLE_COAST_S,
                        help="s a wheel rolls on after stopping: more stops "
                        "earlier")
    args = parser.parse_args()
    WIGGLE_POWER, WIGGLE_COAST_S = args.wiggle_power, args.wiggle_coast

    for number in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(number, _stop_on_signal)
    import hw
    if args.wiggle_test:
        print(f"wiggle test, power {WIGGLE_POWER}, coast {WIGGLE_COAST_S} s")
        print(f"ended {wiggle(hw.ticks, hw.motors):+.1f} degrees "
              "from the start heading")
        return

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", MAP_PORT))
    sock.setblocking(False)
    live = (socket.gethostbyname(args.live), LIVE_PORT) if args.live else None
    use_ir = args.gap == "ir"
    # with ir, a start value until the first reading: 30 cm between them
    gap = (30.0 + config.NOSE_TO_AXLE_CM + config.LEADER_TAIL_TO_AXLE_CM
           if use_ir else float(args.gap))
    sensor = None
    if use_ir:
        from distance import Distance
        sensor = Distance()
    # with ir it closes up to the page's middle, 30 cm nose to tail, from
    # wherever it was put down
    target = (config.CONVOY_NOSE_TO_TAIL_CM + config.NOSE_TO_AXLE_CM
              + config.LEADER_TAIL_TO_AXLE_CM if use_ir else gap)
    slave = Slave(args.name, gap, args.heading_scale, args.dist_scale,
                  target)
    print(f"slave {args.name}, "
          f"{'gap from the IR sensor' if use_ir else f'{gap:.0f} cm behind the master'}, heading "
          f"scale {args.heading_scale}, distance scale {args.dist_scale}. "
          "Ctrl+C stops.")
    rows = []
    wiggling = False

    def receive():
        now = time.monotonic()
        while True:
            try:
                data = sock.recv(2048)
            except BlockingIOError:
                return
            try:
                packet = json.loads(data)
            except ValueError:
                continue
            if packet.get("type") == "map":
                slave.hear(packet, now)

    def report():
        target = live or ((slave.live_ip, LIVE_PORT) if slave.live_ip else None)
        if target:
            try:
                sock.sendto(json.dumps(slave.status(wiggling)).encode(), target)
            except OSError:
                pass

    next_status = 0.0

    def busy_listening():
        # called while wiggling: keep hearing, stop if a run starts
        nonlocal next_status
        receive()
        if time.monotonic() >= next_status:
            report()
            next_status = time.monotonic() + 1 / STATUS_HZ
        return slave.driving

    shown = None
    next_ir = 0.0
    seen_ir = False
    recent_ir = collections.deque(maxlen=8)
    next_step = time.monotonic()
    try:
        hw.motors(0, 0)
        while True:
            now = time.monotonic()
            receive()
            if slave.wants_wiggle and not slave.driving:
                print(f"wiggle: {slave.wants_wiggle}")
                slave.wants_wiggle, wiggling = None, True
                left_over = wiggle(hw.ticks, hw.motors, busy_listening)
                wiggling = False
                print(f"wiggle done, {left_over:+.1f} degrees from the start")
                next_step = time.monotonic()
                continue
            if sensor and not slave.driving and now >= next_ir:
                # the page's gap is nose to tail, which the IR sees. the
                # path gap is axle to axle, so both overhangs are added.
                # only between runs: it is the start pose of the next run
                next_ir = now + 0.25
                _, cm = sensor.read()
                if cm and 20 <= cm <= 150:
                    # median of the last 2 s: the beam half on, half under
                    # the master's spoiler jumped 20 to 37 cm standing still
                    recent_ir.append(cm)
                    cm = sorted(recent_ir)[len(recent_ir) // 2]
                    new = (cm + config.NOSE_TO_AXLE_CM
                           + config.LEADER_TAIL_TO_AXLE_CM)
                    if not seen_ir or abs(new - slave.gap) > 1.0:
                        print(f"IR {cm:.0f} cm nose to tail, start gap "
                              f"{new:.0f} cm axle to axle, steers to "
                              f"{slave.target:.0f}")
                    slave.gap, seen_ir = new, True
                    slave.pose = Pose(slave.gap)
            ticks = hw.ticks()
            left, right = slave.step(now, ticks)
            hw.motors(left, right)
            if slave.state != shown:
                print(f"{slave.state} (run {slave.run})")
                shown = slave.state
                if rows and not slave.driving:
                    write_log(rows, rows[-1][1])
                    rows = []
            if now >= next_status:
                report()
                next_status = now + 1 / STATUS_HZ
            if slave.driving:
                rows.append((round(now, 3), slave.run, slave.state,
                             round(now - slave.heard, 3),
                             round(slave.master_s, 2),
                             round(slave.master_speed(), 2),
                             round(slave.path.heading, 4),
                             round(slave.pose.s, 2),
                             round(slave.pose.heading, 4),
                             round(slave.gap_now(), 2),
                             round(left, 3), round(right, 3),
                             slave.heading_scale, slave.dist_scale))
            next_step += 1 / CONTROL_HZ
            time.sleep(max(0.0, next_step - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        hw.motors(0, 0)
        if rows:
            write_log(rows, rows[-1][1])


if __name__ == "__main__":
    main()
