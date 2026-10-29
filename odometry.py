"""Distance and heading from the wheel encoders, and the convoy message.

Each rover sends the rover behind it one UDP packet every SEND_INTERVAL,
a line of JSON:

  {"seq": 12, "t": 1.20, "left_cm": 31.4, "right_cm": 29.8,
   "s_cm": 30.6, "heading": -0.121}

left_cm and right_cm are each wheel's distance since start. s_cm is the
distance the rover's centre has driven, (left + right) / 2, and heading
its turn since start in radians, (right - left) / track width, positive
left. They are totals, not changes, so a lost packet loses nothing.

Forward only: the tally has no sign, and convoy driving never reverses.
"""

import json

import config

PORT = 5005
SEND_INTERVAL = 0.1


class Odometer:
    """Adds up each wheel's distance with its own calibration."""

    def __init__(self, left, right):
        self._wheels = (left, right)
        self._cm_per_pulse = (config.LEFT_CM_PER_PULSE,
                              config.RIGHT_CM_PER_PULSE)
        self._start = (left.pulses, right.pulses)

    def read(self):
        """Return (left_cm, right_cm, s_cm, heading)."""
        left_cm, right_cm = ((w.pulses - p0) * k for w, p0, k in
                             zip(self._wheels, self._start,
                                 self._cm_per_pulse))
        return (left_cm, right_cm, (left_cm + right_cm) / 2,
                (right_cm - left_cm) / config.TRACK_CM)


def encode(seq, t, odo):
    left_cm, right_cm, s_cm, heading = odo
    return (json.dumps({"seq": seq, "t": round(t, 3),
                        "left_cm": round(left_cm, 2),
                        "right_cm": round(right_cm, 2),
                        "s_cm": round(s_cm, 2),
                        "heading": round(heading, 4)}) + "\n").encode()


def decode(data):
    """A packet as a dict, or None if it is not a convoy message."""
    try:
        msg = json.loads(data)
        return msg if {"seq", "s_cm", "heading"} <= msg.keys() else None
    except (ValueError, AttributeError):
        return None
