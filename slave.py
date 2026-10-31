import argparse
import bisect
import collections
import importlib
import json
import math
import signal
import socket
import threading
import time

MAP_PORT = 5005
LIVE_PORT = 5006

TICKS_PER_CM = 196.85
TRACK_CM = 13.1
SPEED_TABLE = ((0.0, 0.0), (0.12, 9.0), (0.20, 28.0), (0.30, 41.0), (0.50, 46.0), (1.0, 50.0))
MAX_CM_S = 45.0

LOOKAHEAD_CM = 10.0
GAP_GAIN = 1.0
MIN_GAP_SHARE = 0.5
STOP_CM_S = 2.0
KP = 0.004
KI = 0.02
I_MAX = 10.0

CONTROL_HZ = 50
STATUS_HZ = 10
LOST_S = 0.5
RECONNECT_S = 2.0
START_S_CM = 1.0
SPEED_WINDOW_S = 0.5

WIGGLE_DEGREES = 5.0
WIGGLE_TIMES = 2
WIGGLE_POWER = 0.3
WIGGLE_MIN_POWER = 0.2
WIGGLE_GAIN = 0.05
WIGGLE_TOLERANCE = 0.7
WIGGLE_STEP_S = 1.0
WIGGLE_SETTLE_S = 0.1


def clamp(value, low, high):
    return max(low, min(high, value))


def interpolate(x, points):
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return points[-1][1]


def table_speed(power):
    return math.copysign(interpolate(abs(power), SPEED_TABLE), power)


def table_power(speed):
    return interpolate(speed, [(v, p) for p, v in SPEED_TABLE])


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Peer:
    def __init__(self, host, port):
        self.host, self.port, self.address = host, port, None
        threading.Thread(target=self._resolve, daemon=True).start()

    def _resolve(self):
        while True:
            try:
                address = (socket.gethostbyname(self.host), self.port)
            except OSError:
                address = self.address
            if address != self.address:
                print(f"{self.host} is {address[0]}")
                self.address = address
            time.sleep(5)

    def send(self, sock, data):
        if self.address:
            try:
                sock.sendto(data, self.address)
            except OSError:
                pass


class Path:
    def __init__(self):
        self.s, self.x, self.y, self.heading = [], [], [], 0.0

    def add(self, s, heading):
        if self.s and s <= self.s[-1]:
            self.heading = heading if s == self.s[-1] else self.heading
            return
        if self.s:
            ds, mid = s - self.s[-1], (heading + self.heading) / 2
            x, y = self.x[-1] + ds * math.cos(mid), self.y[-1] + ds * math.sin(mid)
        else:
            x, y = 0.0, 0.0
        self.s.append(s)
        self.x.append(x)
        self.y.append(y)
        self.heading = heading

    def point(self, s):
        if s <= 0 or not self.s:
            return s, 0.0
        i = bisect.bisect_right(self.s, s)
        if i >= len(self.s):
            return self.x[-1], self.y[-1]
        f = (s - self.s[i - 1]) / (self.s[i] - self.s[i - 1])
        return self.x[i - 1] + f * (self.x[i] - self.x[i - 1]), self.y[i - 1] + f * (self.y[i] - self.y[i - 1])


class Odometer:
    def __init__(self, x):
        self.x, self.y, self.s, self.heading = x, 0.0, 0.0, 0.0

    def update(self, left_ticks, right_ticks):
        s = (left_ticks + right_ticks) / 2 / TICKS_PER_CM
        heading = (right_ticks - left_ticks) / TICKS_PER_CM / TRACK_CM
        ds, mid = s - self.s, (heading + self.heading) / 2
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.s, self.heading = s, heading


class Wheels:
    def __init__(self):
        self.reset()

    def reset(self):
        self.last, self.integral = None, [0.0, 0.0]

    def powers(self, now, ticks, targets):
        if self.last is None:
            speeds, dt = list(targets), 0.0
        else:
            dt = max(now - self.last[0], 1e-3)
            speeds = [(tick - was) / TICKS_PER_CM / dt for tick, was in zip(ticks, self.last[1])]
        self.last = (now, ticks)
        out = []
        for i, target in enumerate(targets):
            if target <= 0:
                self.integral[i] = 0.0
                out.append(0.0)
                continue
            error = target - speeds[i]
            self.integral[i] = clamp(self.integral[i] + error * dt, -I_MAX, I_MAX)
            out.append(clamp(table_power(target) + KP * error + KI * self.integral[i], 0.0, 1.0))
        return out


def pure_pursuit(speed, error, distance):
    half = math.sin(error) / max(distance, 1.0) * TRACK_CM
    left, right = speed * (1 - half), speed * (1 + half)
    top = max(left, right)
    if top > MAX_CM_S:
        left, right = left * MAX_CM_S / top, right * MAX_CM_S / top
    return max(left, 0.0), max(right, 0.0)


class Slave:
    def __init__(self, name, gap_cm):
        self.name, self.gap = name, gap_cm
        self.state, self.run, self.heard = "waiting", None, None
        self.done, self.wants_wiggle, self.wiggle_home, self.live_ip = set(), None, None, None
        self.wheels = Wheels()
        self._reset_run(None)

    def _reset_run(self, run):
        self.run, self.path, self.recent = run, Path(), collections.deque()
        self.master_s, self.origin, self.odometer = 0.0, self.wiggle_home, Odometer(-self.gap)
        self.wheels.reset()

    @property
    def driving(self):
        return self.state in ("follow", "stopping")

    def hear(self, packet, now):
        silent = self.heard is None or now - self.heard > RECONNECT_S
        self.heard, self.live_ip = now, packet.get("live")
        cmd = packet.get("cmd")
        if cmd and cmd.get("id") not in self.done:
            self.done.add(cmd.get("id"))
            if cmd.get("wiggle") in ("all", self.name) and self.state != "follow":
                self.wants_wiggle = f"told to (wiggle {cmd.get('wiggle')})"
        run, running, s = packet.get("run"), packet.get("running"), packet.get("s_cm", 0.0)
        if not running and not self.driving:
            self.run = run
        if running and run != self.run:
            if s < START_S_CM and self.state in ("waiting", "idle", "missed", "lost", "stopping"):
                self._reset_run(run)
                self.path.add(0.0, 0.0)
                self.state, self.wants_wiggle = "follow", None
            else:
                self.run, self.state = run, "missed"
        if run == self.run and self.driving:
            if s > self.master_s:
                self.master_s = s
                self.path.add(s, packet["heading"])
            if running and (not self.recent or packet["t"] > self.recent[-1][0]):
                self.recent.append((packet["t"], self.master_s))
                while self.recent and packet["t"] - self.recent[0][0] > SPEED_WINDOW_S:
                    self.recent.popleft()
            if not running and self.state == "follow":
                self.state = "stopping"
        if not running and self.state in ("waiting", "missed", "lost"):
            self.state = "idle"
        if silent and not running and not self.driving:
            self.wants_wiggle = "heard the master"

    def master_speed(self):
        if self.state == "stopping" or len(self.recent) < 2:
            return 0.0
        (t0, s0), (t1, s1) = self.recent[0], self.recent[-1]
        return (s1 - s0) / (t1 - t0) if t1 > t0 else 0.0

    def step(self, now, ticks):
        if not self.driving:
            self.wheels.reset()
            return 0.0, 0.0
        if self.heard is None or now - self.heard > LOST_S:
            self.state = "lost"
            self.wheels.reset()
            return 0.0, 0.0
        if self.origin is None:
            self.origin = ticks
        self.odometer.update(ticks[0] - self.origin[0], ticks[1] - self.origin[1])
        here = self.odometer.s - self.gap
        gap_now = self.master_s - here
        speed = clamp(self.master_speed() + GAP_GAIN * (gap_now - self.gap), 0.0, MAX_CM_S)
        if gap_now < self.gap * MIN_GAP_SHARE:
            speed = 0.0
        if self.state == "stopping" and speed < STOP_CM_S:
            self.state = "idle"
            self.wheels.reset()
            return 0.0, 0.0
        tx, ty = self.path.point(here + LOOKAHEAD_CM)
        dx, dy = tx - self.odometer.x, ty - self.odometer.y
        error = wrap(math.atan2(dy, dx) - self.odometer.heading)
        targets = pure_pursuit(speed, error, math.hypot(dx, dy))
        return tuple(self.wheels.powers(now, ticks, targets))

    def status(self):
        here = self.odometer.s - self.gap
        state = "wiggle" if self.wiggle_home is not None else self.state
        return {"type": "slave", "name": self.name, "run": self.run, "state": state,
                "path_cm": round(here, 1), "x": round(self.odometer.x, 1), "y": round(self.odometer.y, 1),
                "heading": round(self.odometer.heading, 4), "gap_cm": round(self.master_s - here, 1),
                "set_gap_cm": self.gap}


def wiggle(ticks, motors, ticks_per_degree, on_step=None):
    l0, r0 = ticks()

    def turned():
        left, right = ticks()
        return ((right - r0) - (left - l0)) / ticks_per_degree

    try:
        for target in [WIGGLE_DEGREES, -WIGGLE_DEGREES] * WIGGLE_TIMES + [0.0]:
            end, settled = time.monotonic() + WIGGLE_STEP_S, None
            while time.monotonic() < end:
                error = target - turned()
                if abs(error) < WIGGLE_TOLERANCE:
                    motors(0, 0)
                    settled = settled or time.monotonic()
                    if time.monotonic() - settled > WIGGLE_SETTLE_S:
                        break
                else:
                    settled = None
                    power = math.copysign(clamp(abs(error) * WIGGLE_GAIN, WIGGLE_MIN_POWER, WIGGLE_POWER), error)
                    motors(-power, power)
                if on_step and on_step():
                    return turned()
                time.sleep(0.005)
            motors(0, 0)
    finally:
        motors(0, 0)
    return turned()


class FakeCar:
    def __init__(self, clock=time.monotonic, lag_s=0.15):
        self.clock, self.lag = clock, lag_s
        self.powers, self.speeds, self.cm = [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]
        self.last = clock()

    def _advance(self):
        now = self.clock()
        dt, self.last = now - self.last, now
        for i in range(2):
            self.speeds[i] += (table_speed(self.powers[i]) - self.speeds[i]) * min(1.0, dt / self.lag)
            self.cm[i] += self.speeds[i] * dt

    def ticks(self):
        self._advance()
        return tuple(round(c * TICKS_PER_CM) for c in self.cm)

    def motors(self, left, right):
        self._advance()
        self.powers = [clamp(left, -1.0, 1.0), clamp(right, -1.0, 1.0)]


def _exit_on_signal(number, _frame):
    raise SystemExit(128 + number)


def main():
    parser = argparse.ArgumentParser(description="Convoy slave: drive the master's map, received over UDP.")
    parser.add_argument("--name", default=socket.gethostname().split(".")[0])
    parser.add_argument("--gap", type=float, default=60.0, help="cm behind the master along its path, axle to axle")
    parser.add_argument("--live", help="hostname of the laptop running live.py (default: the address in the master's packets)")
    parser.add_argument("--port", type=int, default=MAP_PORT)
    parser.add_argument("--fake", action="store_true", help="simulated car, for tests on a laptop")
    args = parser.parse_args()

    car = FakeCar() if args.fake else importlib.import_module("hw")
    for number in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(number, _exit_on_signal)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", args.port))
    sock.setblocking(False)
    live = Peer(args.live, LIVE_PORT) if args.live else None
    slave = Slave(args.name, args.gap)
    ticks_per_degree = TICKS_PER_CM * TRACK_CM * math.pi / 180
    print(f"Slave {args.name}, {args.gap:.0f} cm behind the master, listening on UDP {args.port}. Ctrl+C stops.")

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
        data = json.dumps(slave.status()).encode()
        if live:
            live.send(sock, data)
        elif slave.live_ip:
            try:
                sock.sendto(data, (slave.live_ip, LIVE_PORT))
            except OSError:
                pass

    def interrupted():
        nonlocal next_status
        receive()
        if time.monotonic() >= next_status:
            report()
            next_status = time.monotonic() + 1 / STATUS_HZ
        return slave.driving

    shown, next_status, next_step = None, 0.0, time.monotonic()
    try:
        car.motors(0, 0)
        while True:
            now = time.monotonic()
            receive()
            if slave.wants_wiggle and not slave.driving:
                print(f"Wiggle: {slave.wants_wiggle}")
                slave.wants_wiggle, slave.wiggle_home = None, car.ticks()
                report()
                left_over = wiggle(car.ticks, car.motors, ticks_per_degree, on_step=interrupted)
                slave.wiggle_home = None
                if slave.driving:
                    print(f"Wiggle cut short by a new run, {left_over:+.1f} degrees from the start heading (counted)")
                else:
                    print(f"Wiggle done, {left_over:+.1f} degrees from the start heading")
                next_step = time.monotonic()
                continue
            left, right = slave.step(now, car.ticks())
            car.motors(left, right)
            if slave.state != shown:
                print(f"{slave.state} (run {slave.run})")
                shown = slave.state
            if now >= next_status:
                report()
                next_status = now + 1 / STATUS_HZ
            next_step += 1 / CONTROL_HZ
            time.sleep(max(0.0, next_step - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        car.motors(0, 0)


if __name__ == "__main__":
    main()
