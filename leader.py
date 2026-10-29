#!/usr/bin/env python3
"""Drive the convoy leader by keyboard, and send its odometry behind it.

  python3 leader.py FOLLOWER_IP [RPM]       default 30 rpm

w forward, a arc left, d arc right, e stop, r re-arm, q quit. Hold a key
to keep driving, as in rc.py. Speed controlled at RPM, the speed the
calibration was done at. Arcs run the inner wheel at half speed, the
20 / 40 ratio the track width was measured with, and never reverse a
wheel, so the encoders always count the right way.

Every SEND_INTERVAL one packet goes to FOLLOWER_IP, format in
odometry.py. Every packet is also logged to leader_<rpm>_<time>.csv.
"""

import csv
import curses
import socket
import sys
import time

import config
import odometry
from estop import EStop
from rover import disconnect, Rover, SpeedController, Wheel

if len(sys.argv) < 2:
    sys.exit("usage: python3 leader.py FOLLOWER_IP [RPM]")
ADDRESS = (sys.argv[1], odometry.PORT)
RPM = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
INNER = 0.5

KEYS = {ord("w"): ("forward", (RPM, RPM)),
        ord("a"): ("arc left", (INNER * RPM, RPM)),
        ord("d"): ("arc right", (RPM, INNER * RPM)),
        ord("e"): ("stop", (0.0, 0.0))}


def run(stdscr, estop, control, odo, rows):
    stdscr.nodelay(1)
    stdscr.addstr(0, 2, f"w a d drive at {RPM:.0f} rpm, e stop, r re-arm, "
                        "q quit")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start = time.monotonic()
    next_send = next_control = start
    last_press = prev_press = 0.0
    moving = False
    label = "idle"
    seq = 0

    while True:
        key = stdscr.getch()
        now = time.monotonic()
        if key == ord("q"):
            return
        if not estop.armed:
            stdscr.addstr(2, 2, "E-STOP TRIPPED, press r")
            if key == ord("r"):
                control.stop()
                estop.rearm()
                moving = False
        else:
            stdscr.addstr(2, 2, "armed                  ")

        if estop.armed and key in KEYS:
            label, targets = KEYS[key]
            control.target(*targets)
            prev_press, last_press = last_press, now
            moving = label != "stop"
        elif moving:
            # same hold logic as rc.py: stop shortly after the key is let go
            repeating = (last_press - prev_press) < config.REPEAT_GAP
            timeout = config.STOP_AFTER if repeating else config.FIRST_HOLD
            if now - last_press > timeout:
                control.stop()
                label, moving = "idle", False
        stdscr.addstr(1, 2, f"{label:10}")

        if now >= next_control:
            next_control += config.CONTROL_INTERVAL
            (lr, _), (rr, _) = control.update()
        if now >= next_send:
            next_send += odometry.SEND_INTERVAL
            reading = odo.read()
            try:
                sock.sendto(odometry.encode(seq, now - start, reading),
                            ADDRESS)
            except OSError:
                # no route yet, keep driving and try again next time
                pass
            rows.append((seq, round(now - start, 3), label,
                         *(round(x, 4) for x in reading),
                         round(lr, 1), round(rr, 1)))
            stdscr.addstr(3, 2, f"s {reading[2]:7.1f} cm  heading "
                                f"{reading[3]:+.3f} rad  sent {seq}")
            seq += 1
        stdscr.refresh()
        time.sleep(0.01)


def main():
    estop = EStop()
    rover = Rover(estop)
    left = Wheel(config.LEFT_ENCODER)
    right = Wheel(config.RIGHT_ENCODER)
    control = SpeedController(rover, left, right)
    odo = odometry.Odometer(left, right)
    rows = []
    name = f"leader_{RPM:.0f}_{time.strftime('%H%M%S')}.csv"
    estop.arm()
    try:
        curses.wrapper(run, estop, control, odo, rows)
    finally:
        control.stop()
        with open(name, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["seq", "t", "key", "left_cm", "right_cm", "s_cm",
                        "heading", "left_rpm", "right_rpm"])
            w.writerows(rows)
        print(f"sent {len(rows)} packets to {ADDRESS[0]}, wrote {name}")
        estop.close()
        left.close()
        right.close()
        disconnect()


if __name__ == "__main__":
    main()
