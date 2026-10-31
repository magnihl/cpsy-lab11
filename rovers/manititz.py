# Pins of the rover in github.com/ManiTiTz/raunkerfi, from its
# linefollower/pi/hw.py. On that rover: cp rovers/manititz.py config_local.py
# then check with rc.py and encodertest.py, and calibrate with calibrate.py
# before any convoy run. Paste the calibration in below.

SLEEP_PIN = "GPIO26"
LEFT_MOTOR = {"forward": "GPIO19", "backward": "GPIO13"}
RIGHT_MOTOR = {"forward": "GPIO12", "backward": "GPIO18"}
# encodertest.py checks both wires of each wheel
ENCODER_B = ("GPIO5", "GPIO21")
ENCODER_A = ("GPIO27", "GPIO22")
# one wire per wheel, rising edges, as this repo counts
LEFT_ENCODER = "GPIO5"
RIGHT_ENCODER = "GPIO27"

# from calibrate.py on this rover, replace both
# LEFT_CM_PER_PULSE = 0.0
# RIGHT_CM_PER_PULSE = 0.0
# TRACK_CM = 0.0
