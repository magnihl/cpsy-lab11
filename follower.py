#!/usr/bin/env python3
"""Follow the rover in front by replaying its path, and pass ours on.

  python3 follower.py START_GAP [NEXT_IP] [noir]

START_GAP is the distance in cm from the front rover's wheel centres
to ours at the start, both lined up along one straight edge, pointing
the same way. Heading 0 is each rover's own start direction.

The front rover sends its distance s and heading every 0.1 s
(odometry.py). From those its path is rebuilt as points (s, x, y) in
its own start frame, where we start at (-START_GAP, 0). We work out our
own x, y the same way and steer at the path point LOOKAHEAD cm ahead of
our place on it (pure pursuit). Copying only the heading would leave
any sideways error uncorrected, simulated up to 8 cm.

Speed comes from the gap along the path, front s + START_GAP - our s,
which still works in a curve where the IR beam misses the rover in
front. Proportional, around the front rover's own speed. The IR is only
a safety stop: 20 cm or closer, or too close to read, stops. Nothing in
range does not stop, the beam misses in a curve.

With NEXT_IP our own odometry goes on to the rover behind, the same
message format, so every follower runs this same program. noir runs
without the IR sensor.

Every loop is logged to follower_<gap>_<time>.csv.
Ctrl+C stops it, so does the e-stop button.
"""

import bisect
import csv
import math
import socket
import sys
import time

import config
import odometry

ARGS = sys.argv[1:]
if not ARGS:
    sys.exit("usage: python3 follower.py START_GAP [NEXT_IP] [noir]")
START_GAP = float(ARGS[0])
USE_IR = "noir" not in ARGS[1:]
NEXT_IP = next((a for a in ARGS[1:] if a != "noir"), None)

# the page's 20 to 40 cm. closer than this by IR stops, under 20 cm the
# sensor reads far again (Lab 9)
MIN_CM = 20.0
# rpm added per cm the path gap is above START_GAP, so 10 cm behind
# speeds up by 15 rpm
GAP_GAIN = 1.5
MAX_RPM = 60.0
# rpm difference between the wheels per radian of heading error.
# 0.1 rad off gives 6 rpm, a gentle correction at 30 rpm
HEADING_GAIN = 60.0
# how far ahead on the front rover's path we aim
LOOKAHEAD = 10.0
# no packet for this long and the front rover is lost: stop
TIMEOUT = 0.5
# cm per second at 1 rpm, from the calibration
CM_S_PER_RPM = (config.PULSES_PER_REV / 60 *
                (config.LEFT_CM_PER_PULSE + config.RIGHT_CM_PER_PULSE) / 2)


class Path:
    """The front rover's path as points (s, x, y), in its start frame."""

    def __init__(self):
        self.s, self.x, self.y = [], [], []
        self.heading = 0.0

    def add(self, s, heading):
        if self.s and s < self.s[-1]:
            return
        if self.s:
            # integrate along the step with the mean heading
            ds = s - self.s[-1]
            mid = (heading + self.heading) / 2
            x = self.x[-1] + ds * math.cos(mid)
            y = self.y[-1] + ds * math.sin(mid)
        else:
            x, y = 0.0, 0.0
        self.s.append(s)
        self.x.append(x)
        self.y.append(y)
        self.heading = heading

    def point(self, s):
        """Where the front rover was at distance s. Before its start,
        s < 0, that is straight ahead of where it started."""
        if s <= 0 or not self.s:
            return s, 0.0
        i = bisect.bisect_right(self.s, s) - 1
        return self.x[i], self.y[i]


class Position:
    """Our own x, y in the front rover's start frame, from our odometry."""

    def __init__(self):
        self.x, self.y = -START_GAP, 0.0
        self._s, self._heading = 0.0, 0.0

    def update(self, s, heading):
        ds = s - self._s
        mid = (heading + self._heading) / 2
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self._s, self._heading = s, heading


def decide(path, front_rpm, own_s, own_heading, own_xy, ir_cm, fresh):
    """Wheel rpm for this loop, and why.

    Returns (left_rpm, right_rpm, gap_cm, heading_error, state), state
    "go", "close" (IR stop), "lost" (no recent packet) or "wait" (no
    packet yet).
    """
    if not path.s:
        return 0.0, 0.0, None, 0.0, "wait"
    gap = path.s[-1] + START_GAP - own_s
    tx, ty = path.point(own_s - START_GAP + LOOKAHEAD)
    want = math.atan2(ty - own_xy[1], tx - own_xy[0])
    # wrapped to -pi..pi, so a full turn of heading is not a full turn back
    error = math.atan2(math.sin(want - own_heading),
                       math.cos(want - own_heading))
    if not fresh:
        return 0.0, 0.0, gap, error, "lost"
    if ir_cm is not None and ir_cm <= MIN_CM:
        return 0.0, 0.0, gap, error, "close"
    speed = min(max(front_rpm + GAP_GAIN * (gap - START_GAP), 0.0), MAX_RPM)
    if speed == 0.0:
        return 0.0, 0.0, gap, error, "go"
    # forward only: the inner wheel slows, it never reverses
    diff = HEADING_GAIN * error
    left = min(max(speed - diff / 2, 0.0), MAX_RPM)
    right = min(max(speed + diff / 2, 0.0), MAX_RPM)
    return left, right, gap, error, "go"


def main():
    from distance import Distance
    from estop import EStop
    from rover import disconnect, Rover, SpeedController, Wheel

    estop = EStop()
    rover = Rover(estop)
    left = Wheel(config.LEFT_ENCODER)
    right = Wheel(config.RIGHT_ENCODER)
    control = SpeedController(rover, left, right)
    odo = odometry.Odometer(left, right)
    sensor = Distance() if USE_IR else None

    listen = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listen.bind(("", odometry.PORT))
    listen.setblocking(False)
    send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"follower, start gap {START_GAP:.0f} cm, "
          f"{'IR on' if USE_IR else 'no IR'}, "
          f"{'passing on to ' + NEXT_IP if NEXT_IP else 'last in line'}")

    name = f"follower_{START_GAP:.0f}_{time.strftime('%H%M%S')}.csv"
    rows = []
    path = Path()
    where = Position()
    # (time, s) of recent front packets, for the front rover's speed
    recent = []
    last_seq = -1
    last_heard = None
    state = None
    seq = 0
    next_send = time.monotonic()

    try:
        estop.arm()
        start = time.monotonic()
        while True:
            time.sleep(config.CONTROL_INTERVAL)
            now = time.monotonic()
            # read every waiting packet, keep the newest
            while True:
                try:
                    msg = odometry.decode(listen.recv(512))
                except BlockingIOError:
                    break
                if msg is None or msg["seq"] <= last_seq:
                    continue
                last_seq = msg["seq"]
                last_heard = now
                s = float(msg["s_cm"])
                path.add(s, float(msg["heading"]))
                recent = [r for r in recent if now - r[0] < 0.5]
                recent.append((now, s))

            front_rpm = 0.0
            if len(recent) >= 2 and recent[-1][0] > recent[0][0]:
                cm_s = ((recent[-1][1] - recent[0][1]) /
                        (recent[-1][0] - recent[0][0]))
                front_rpm = cm_s / CM_S_PER_RPM
            fresh = last_heard is not None and now - last_heard < TIMEOUT

            ir_cm = None
            if sensor is not None:
                _, ir_cm = sensor.read()
            own = odo.read()
            where.update(own[2], own[3])
            left_rpm, right_rpm, gap, error, now_state = decide(
                path, front_rpm, own[2], own[3], (where.x, where.y), ir_cm,
                fresh)
            if now_state != state:
                print(f"{now - start:6.2f} s  {now_state.upper()}")
                if now_state != "go":
                    control.stop()
                state = now_state
            control.target(left_rpm, right_rpm)
            (lr, _), (rr, _) = control.update()

            if NEXT_IP and now >= next_send:
                next_send += odometry.SEND_INTERVAL
                try:
                    send.sendto(odometry.encode(seq, now - start, own),
                                (NEXT_IP, odometry.PORT))
                except OSError:
                    pass
                seq += 1

            rows.append((round(now - start, 3), last_seq,
                         round(path.s[-1], 2) if path.s else "",
                         round(path.heading, 4),
                         round(front_rpm, 1), round(own[2], 2),
                         round(own[3], 4), round(where.x, 2),
                         round(where.y, 2), "" if gap is None else round(gap, 2),
                         round(error, 4), "" if ir_cm is None else round(ir_cm, 1),
                         state, round(left_rpm, 1), round(right_rpm, 1),
                         round(lr, 1), round(rr, 1)))
    except KeyboardInterrupt:
        pass
    finally:
        control.stop()
        with open(name, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "front_seq", "front_s", "front_heading",
                        "front_rpm", "own_s", "own_heading", "x", "y", "gap",
                        "heading_error", "ir_cm", "state", "left_target",
                        "right_target", "left_rpm", "right_rpm"])
            w.writerows(rows)
        print(f"wrote {name}, {len(rows)} rows")
        estop.close()
        if sensor is not None:
            sensor.close()
        left.close()
        right.close()
        disconnect()


if __name__ == "__main__":
    main()
