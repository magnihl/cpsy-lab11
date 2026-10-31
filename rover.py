"""The rover's drive and its wheel encoders."""

import time

import config  # sets the pin factory, import before gpiozero

import pigpio
from gpiozero import Motor, Robot

from vendor.simple_pid import PID

_pi = None


def _connect():
    """One shared pigpio connection for every wheel."""
    global _pi
    if _pi is None:
        _pi = pigpio.pi()
        if not _pi.connected:
            raise RuntimeError(
                "pigpiod is not running. "
                "sudo systemctl enable --now pigpiod"
            )
    return _pi


def disconnect():
    """Close the shared pigpio connection.

    Without this, pigpio's own atexit handler runs after the socket has
    gone and prints a Bad file descriptor traceback.
    """
    global _pi
    if _pi is not None:
        _pi.stop()
        _pi = None


class Wheel:
    """Odometer and speedometer for one wheel.

    Counts rising edges on one encoder channel using pigpio's tally,
    which is kept in C and never calls back into Python. gpiozero's
    RotaryEncoder decodes quadrature in Python and tops out around 2300
    edges a second across all four pins, well below what these motors
    produce.

    The tally only ever increases, so distance and speed are unsigned.
    The caller knows which way it commanded the motor.
    """

    def __init__(self, pin, pulses_per_rev=config.PULSES_PER_REV):
        pi = _connect()
        # pigpio takes BCM numbers, config stores gpiozero's GPIOnn form
        self._gpio = int(pin.replace("GPIO", ""))
        pi.set_mode(self._gpio, pigpio.INPUT)
        pi.set_pull_up_down(self._gpio, pigpio.PUD_UP)
        # pigpiod ignores level changes shorter than this, in C. 0 is off
        pi.set_glitch_filter(self._gpio, config.ENCODER_GLITCH_US)
        # no function, so pigpio keeps the count itself
        self._cb = pi.callback(self._gpio, pigpio.RISING_EDGE)
        self._ppr = pulses_per_rev
        self._origin = 0
        self._last_count = 0
        self._last_time = time.monotonic()

    @property
    def pulses(self):
        """Raw edges since the last reset. Used for calibration."""
        return self._cb.tally() - self._origin

    @property
    def rotations(self):
        return self.pulses / self._ppr

    def speed(self):
        """Rotations per minute since the last call to speed().

        Measured by sampling, so call it on a steady interval. A longer
        interval is less noisy but slower to react.
        """
        now = time.monotonic()
        count = self._cb.tally()

        elapsed = now - self._last_time
        moved = count - self._last_count

        self._last_time = now
        self._last_count = count

        if elapsed <= 0:
            return 0.0
        return (moved / self._ppr) / elapsed * 60

    def reset(self):
        self._origin = self._cb.tally()
        self._last_count = self._origin
        self._last_time = time.monotonic()

    def close(self):
        self._cb.cancel()


class Rover:
    def __init__(self, estop, left=config.LEFT_MOTOR, right=config.RIGHT_MOTOR):
        # kept as separate attributes as well as inside Robot, because
        # closed loop control sets each wheel's duty independently
        self.left_motor = Motor(**left)
        self.right_motor = Motor(**right)
        self._robot = Robot(left=self.left_motor, right=self.right_motor)
        self._estop = estop

    def forward(self, speed=config.SPEED):
        self._drive(self._robot.forward, speed)

    def backward(self, speed=config.SPEED):
        self._drive(self._robot.backward, speed)

    def left(self, speed=config.SPEED):
        self._drive(self._robot.left, speed)

    def right(self, speed=config.SPEED):
        self._drive(self._robot.right, speed)

    def stop(self):
        self._robot.stop()

    def _drive(self, action, speed):
        if not self._estop.armed:
            return
        action(speed)


class SpeedController:
    """Closed loop speed control, one PID per wheel.

    The two wheels differ by about 55 percent at duty 0.05 and 2 percent
    at duty 1.0, so they need separate controllers rather than one for
    the pair.

    Forward only. The wheel tally has no sign, so a reversing controller
    would need the quadrature channel back.
    """

    def __init__(self, rover, left_wheel, right_wheel, gains=None):
        kp, ki, kd = gains if gains else (config.KP, config.KI, config.KD)
        self._rover = rover
        self._wheels = (left_wheel, right_wheel)
        self._motors = (rover.left_motor, rover.right_motor)
        self._pids = tuple(
            PID(kp, ki, kd, setpoint=0.0, sample_time=None,
                output_limits=(0.0, 1.0))
            for _ in self._motors
        )
        self._forward = [True, True]

    @property
    def tunings(self):
        return self._pids[0].tunings

    @tunings.setter
    def tunings(self, gains):
        for pid in self._pids:
            pid.tunings = gains

    def target(self, left_rpm, right_rpm=None):
        """Set the wanted speed per wheel. One value sets both.

        Signed: negative drives that wheel backwards. Equal and opposite
        spins the rover on the spot, unequal and same sign turns it in
        an arc.
        """
        if right_rpm is None:
            right_rpm = left_rpm
        for i, rpm in enumerate((left_rpm, right_rpm)):
            forward = rpm >= 0
            if forward != self._forward[i]:
                # the integral built up driving the other way means
                # nothing now, and would fight the new direction
                self._pids[i].reset()
                self._forward[i] = forward
            # the wheel tally is unsigned, so the PID works on magnitude
            self._pids[i].setpoint = abs(rpm)

    def update(self):
        """Run one control step. Returns (rpm, duty) for each wheel.

        rpm is unsigned, matching what the encoder can report.
        """
        armed = self._rover._estop.armed
        out = []
        for i, (pid, wheel, motor) in enumerate(
                zip(self._pids, self._wheels, self._motors)):
            rpm = wheel.speed()
            duty = pid(rpm)
            if not armed:
                duty = 0.0
            if self._forward[i]:
                motor.forward(duty)
            else:
                motor.backward(duty)
            out.append((rpm, duty))
        return out

    def stop(self):
        for pid in self._pids:
            pid.reset()
            pid.setpoint = 0.0
        self._forward = [True, True]
        self._rover.stop()
