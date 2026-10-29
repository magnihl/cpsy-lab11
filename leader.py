#!/usr/bin/env python3
"""Drive by keyboard as rc.py does, and send the wheel distances.

  python3 leader.py FOLLOWER_IP [PORT]     default port 5005

Every SEND_INTERVAL the leader sends one UDP packet to the follower,
a line of JSON:

  {"seq": 12, "t": 1.20, "left_cm": 31.4, "right_cm": 29.8}

left_cm and right_cm are each wheel's signed distance since start, in
cm, positive forward. They are totals, not changes, so a lost packet
loses nothing: the next one carries the same totals. The follower turns
them into distance driven, (left + right) / 2, and heading,
(right - left) / track width, with its own track width.

Every packet is also logged to leader_<time>.csv.
"""

import csv
import curses
import json
import socket
import sys
import time

import config
import rc
from estop import EStop
from rover import disconnect, Rover, Wheel

if len(sys.argv) < 2:
    sys.exit("usage: python3 leader.py FOLLOWER_IP [PORT]")
ADDRESS = (sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 5005)
SEND_INTERVAL = 0.1

# the tally has no sign, so the sign comes from what the wheels were told.
# gpiozero's Robot turns on the spot: left runs the left wheel backward
SIGNS = {"forward": (1, 1), "backward": (-1, -1),
         "left": (-1, 1), "right": (1, -1), None: (0, 0)}
CM_PER_PULSE = (config.LEFT_CM_PER_PULSE, config.RIGHT_CM_PER_PULSE)


class Sender:
    """Called every rc loop. Adds up signed distance, sends it on time."""

    def __init__(self, left, right):
        self._wheels = (left, right)
        self._last = [left.pulses, right.pulses]
        self._cm = [0.0, 0.0]
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._start = time.monotonic()
        self._next = self._start
        self.seq = 0
        self.rows = []

    def __call__(self, driving):
        for i, wheel in enumerate(self._wheels):
            now = wheel.pulses
            self._cm[i] += SIGNS[driving][i] * (now - self._last[i]) * CM_PER_PULSE[i]
            self._last[i] = now
        t = time.monotonic()
        if t < self._next:
            return
        self._next += SEND_INTERVAL
        msg = {"seq": self.seq, "t": round(t - self._start, 3),
               "left_cm": round(self._cm[0], 2),
               "right_cm": round(self._cm[1], 2)}
        try:
            self._sock.sendto(json.dumps(msg).encode() + b"\n", ADDRESS)
        except OSError:
            # no route yet, keep driving and try again next time
            pass
        self.rows.append((msg["seq"], msg["t"], driving or "",
                          msg["left_cm"], msg["right_cm"]))
        self.seq += 1


def main():
    estop = EStop()
    rover = Rover(estop)
    left = Wheel(config.LEFT_ENCODER)
    right = Wheel(config.RIGHT_ENCODER)
    send = Sender(left, right)
    name = f"leader_{time.strftime('%H%M%S')}.csv"
    estop.arm()
    try:
        curses.wrapper(rc.run, rover, estop, send)
    finally:
        rover.stop()
        with open(name, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["seq", "t", "driving", "left_cm", "right_cm"])
            w.writerows(send.rows)
        print(f"sent {send.seq} packets to {ADDRESS[0]}:{ADDRESS[1]}, "
              f"wrote {name}")
        estop.close()
        left.close()
        right.close()
        disconnect()


if __name__ == "__main__":
    main()
