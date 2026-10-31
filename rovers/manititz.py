# Pins of the rover in github.com/ManiTiTz/raunkerfi, from its
# linefollower/pi/hw.py. On that rover: cp rovers/manititz.py config_local.py
# then check with rc.py and encodertest.py before any convoy run.

SLEEP_PIN = "GPIO26"
LEFT_MOTOR = {"forward": "GPIO19", "backward": "GPIO13"}
RIGHT_MOTOR = {"forward": "GPIO12", "backward": "GPIO18"}
# encodertest.py checks both wires of each wheel
ENCODER_B = ("GPIO5", "GPIO21")
ENCODER_A = ("GPIO27", "GPIO22")
# one wire per wheel, rising edges, as this repo counts
LEFT_ENCODER = "GPIO5"
RIGHT_ENCODER = "GPIO27"

# from linefollower/notes/measured.md: 19,685 quadrature ticks per metre,
# one wire's rising edges are a quarter of that, 4921 per metre. track
# from its spin test. not checked with calibrate.py
LEFT_CM_PER_PULSE = 0.02032
RIGHT_CM_PER_PULSE = 0.02032
TRACK_CM = 13.1
