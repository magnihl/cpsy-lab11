"""ticks() and motors() for slave.py on this rover.

slave.py (github.com/ManiTiTz/raunkerfi, linefollower/convoy) drives any
car through a module named hw with these two functions. This one wraps
rover.py.

ticks() returns each wheel's signed distance in hundredths of a cm, so
slave_rover.py sets TICKS_PER_CM = 100 and both wheels keep their own
calibration. The tally has no sign, so new pulses take the sign of the
direction last commanded to that wheel, kept through power 0, since the
wheel coasts the way it was going.

A wheel told to reverse first gets power 0, and turns the new way only
once no pulse has come for SETTLE_S: a pulse from the old direction
counted with the new sign was the wiggle's drift, 31 Oct.
"""

import atexit
import time

import config
from estop import EStop
from rover import disconnect, Rover, Wheel

_estop = EStop()
_rover = Rover(_estop)
_wheels = (Wheel(config.LEFT_ENCODER), Wheel(config.RIGHT_ENCODER))
_motors = (_rover.left_motor, _rover.right_motor)
_cm_per_pulse = (config.LEFT_CM_PER_PULSE, config.RIGHT_CM_PER_PULSE)
_sign = [1, 1]
_seen = [w.pulses for w in _wheels]
_cm = [0.0, 0.0]
_moved = [0.0, 0.0]
SETTLE_S = 0.05
_estop.arm()


def ticks():
    now = time.monotonic()
    for i, wheel in enumerate(_wheels):
        pulses = wheel.pulses
        if pulses != _seen[i]:
            _moved[i] = now
        _cm[i] += _sign[i] * (pulses - _seen[i]) * _cm_per_pulse[i]
        _seen[i] = pulses
    return tuple(c * 100 for c in _cm)


def motors(left, right):
    # count what has come in under the old sign before changing it
    ticks()
    now = time.monotonic()
    for i, (motor, power) in enumerate(zip(_motors, (left, right))):
        if power and (power > 0) != (_sign[i] > 0):
            if now - _moved[i] < SETTLE_S:
                # still turning the old way: let it stop, keep counting
                # with the old sign, reverse on a later call
                motor.stop()
                continue
        if power > 0:
            _sign[i] = 1
            motor.forward(min(power, 1.0))
        elif power < 0:
            _sign[i] = -1
            motor.backward(min(-power, 1.0))
        else:
            motor.stop()


def _close():
    # slave.py has already set the motors to 0. after Ctrl+C, close() once
    # found GPIO25 unwritable (29 Oct), and a crash here would hide that
    for step in (*(m.stop for m in _motors), _estop.close,
                 *(w.close for w in _wheels), disconnect):
        try:
            step()
        except Exception as error:
            print(f"cleanup: {error}")


atexit.register(_close)
