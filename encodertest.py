#!/usr/bin/env python3
"""Check both wires of both encoders. Adapted from a friend's script.

Each wheel turns forward for 1.5 s, one at a time, and every edge on
all four encoder wires is counted. A wire that pulses while its wheel
turns works; one that stays at 0 is dead or loose.

  python3 encodertest.py        lift the rover first, wheels free

Run on the Pi with system python3, as rc.py.
"""

import time

import pigpio

import config
from estop import EStop
from rover import _connect, disconnect, Rover

DUTY = 0.3
# both channels of each wheel, from config.py. LEFT_MOTOR is MOTOR_B,
# whose encoder is ENCODER_B, RIGHT_MOTOR is MOTOR_A with ENCODER_A
WIRES = {
    "left": dict(zip(("C1", "C2"), config.ENCODER_B)),
    "right": dict(zip(("C1", "C2"), config.ENCODER_A)),
}


def main():
    input("Lift the rover so both wheels spin freely, then press Enter. "
          "Each wheel turns forward for 1.5 s, one at a time.")
    pi = _connect()
    callbacks = {}
    for side, wires in WIRES.items():
        for name, pin in wires.items():
            gpio = int(pin.replace("GPIO", ""))
            pi.set_mode(gpio, pigpio.INPUT)
            pi.set_pull_up_down(gpio, pigpio.PUD_UP)
            callbacks[(side, name)] = (pin, pi.callback(gpio,
                                                        pigpio.EITHER_EDGE))
    print("levels at rest:", {f"{s} {n} {p}": pi.read(int(p[4:]))
                              for (s, n), (p, _) in callbacks.items()})

    estop = EStop()
    rover = Rover(estop)
    motors = {"left": rover.left_motor, "right": rover.right_motor}
    results = {}
    try:
        estop.arm()
        for side, motor in motors.items():
            before = {k: cb.tally() for k, (_, cb) in callbacks.items()}
            motor.forward(DUTY)
            time.sleep(1.5)
            motor.stop()
            time.sleep(0.8)
            results[side] = {k: cb.tally() - before[k]
                             for k, (_, cb) in callbacks.items()}
            print(f"{side} wheel running: " + ", ".join(
                f"{s} {n} ({callbacks[(s, n)][0]}) {c}"
                for (s, n), c in results[side].items()))
    finally:
        rover.stop()
        estop.close()
        for _, cb in callbacks.values():
            cb.cancel()
        disconnect()

    for side in motors:
        own = {n: results[side][(side, n)] for n in ("C1", "C2")}
        pins = WIRES[side]
        if not any(own.values()):
            print(f"\n{side} encoder: no edges on either wire. Did the {side} "
                  "wheel turn? Check the motor plug, encoder power and ground.")
        elif 0 in own.values():
            dead = next(n for n, c in own.items() if c == 0)
            print(f"\n{side} encoder: {dead} ({pins[dead]}) is dead, the "
                  f"other wire works. Check that jumper at the Pi and the plug.")
        else:
            print(f"\n{side} encoder: both wires pulse, "
                  f"C1 {own['C1']}, C2 {own['C2']} edges.")


if __name__ == "__main__":
    main()
