#!/usr/bin/env python3
"""Keyboard remote control. w a s d to drive, e stop, r re-arm, q quit.

Run on the Pi:  python3 rc.py
System python3, not the venv. gpiozero is installed system wide.
"""

import curses
import time

import config
from estop import EStop
from rover import Rover


def run(stdscr, rover, estop):
    # curses.wrapper does initscr, cbreak, noecho and keypad, but not
    # this. Without it getch blocks and the idle timeout never runs.
    stdscr.nodelay(1)
    stdscr.addstr(0, 2, "w a s d drive, e stop, r re-arm, q quit")

    actions = {
        ord("w"): ("forward ", rover.forward),
        ord("s"): ("backward", rover.backward),
        ord("a"): ("left    ", rover.left),
        ord("d"): ("right   ", rover.right),
        ord("e"): ("stop    ", rover.stop),
    }

    last_press = 0.0
    prev_press = 0.0
    moving = False
    shown = None

    while True:
        stdscr.refresh()
        key = stdscr.getch()
        now = time.monotonic()

        if key == ord("q"):
            return

        state = "armed            " if estop.armed else "E-STOP TRIPPED, press r"
        if state != shown:
            stdscr.addstr(2, 2, state)
            shown = state

        if not estop.armed:
            if key == ord("r"):
                # zero the motor pins before SLP goes high again, or a
                # drive command left set when the stop hit resumes
                rover.stop()
                estop.rearm()
                moving = False
            time.sleep(0.02)
            continue

        if key in actions:
            label, action = actions[key]
            stdscr.addstr(1, 2, label)
            action()
            prev_press = last_press
            last_press = now
            moving = True
        elif moving:
            repeating = (last_press - prev_press) < config.REPEAT_GAP
            timeout = config.STOP_AFTER if repeating else config.FIRST_HOLD
            if now - last_press > timeout:
                rover.stop()
                stdscr.addstr(1, 2, "idle    ")
                moving = False

        time.sleep(0.02)


def main():
    estop = EStop()
    rover = Rover(estop)
    estop.arm()
    try:
        curses.wrapper(run, rover, estop)
    finally:
        estop.close()


if __name__ == "__main__":
    main()
