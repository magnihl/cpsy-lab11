import math
import os

# gpiozero resolves this when the first device is created, so setting it
# here is enough as long as config is imported before any device exists.
# pigpio also gives DMA timed PWM instead of CPU timed.
os.environ.setdefault("GPIOZERO_PIN_FACTORY", "pigpio")

MOTOR_A = {"forward": "GPIO12", "backward": "GPIO13"}
MOTOR_B = {"forward": "GPIO19", "backward": "GPIO18"}

# both encoder channels, kept for reference. we no longer read the B
# channel: decoding quadrature in Python could not keep up, see
# ENCODER_PIN_A below
ENCODER_A = ("GPIO5", "GPIO6")
ENCODER_B = ("GPIO16", "GPIO26")

# C1 only, one channel per wheel. counted by pigpio's tally in C, which
# never wakes Python. no direction, the commanded direction gives that
ENCODER_PIN_A = "GPIO5"
# GPIO16 is motor B's C1 on paper but reads no edges at all, 13 Oct, so
# motor B uses C2 instead. likely a loose jumper on Pi pin 36.
# single channel counting does not care which of the two it uses
ENCODER_PIN_B = "GPIO26"

# spikes shorter than this on an encoder wire are not counted. 0 turns the
# filter off. the shortest real pulse, full power 54 cm/s, lasts about
# 190 us. on since 31 Oct: the convoy slave crept while its odometry said 9 cm/s
ENCODER_GLITCH_US = 50

# which driver channel is which side. wheeltest.py, 20 Oct: the code's
# left spun the physical right wheel. never mattered before Lab 10,
# both wheels always ran at the same speed. each encoder stays with its
# own motor, the PID in Lab 7 proved those pairs belong together
LEFT_MOTOR = MOTOR_B
RIGHT_MOTOR = MOTOR_A
# one wire per wheel. counting every change on both (2 Nov) made the
# convoy slave worse: without decoding, a wire's noise while its motor
# runs is counted too, which quadrature would cancel. Wheel still takes
# a pair for that
LEFT_ENCODER = ENCODER_PIN_B
RIGHT_ENCODER = ENCODER_PIN_A

SLEEP_PIN = "GPIO25"
STOP_PIN = "GPIO4"
IR_CHANNEL = 0

# measured 13 Oct with calibrate.py over 5 hand turns per wheel:
# left 3475 edges, right 3450, so (3475 + 3450) / 10. the two agree to
# 0.7 percent, inside hand turning error, so one constant serves both.
# the Lab 6 page's "about 700 pulses" is one channel's rising edges,
# not the quadrature total
PULSES_PER_REV = 693

SPEED = 0.5

# the terminal waits before it starts repeating a held key, so the first
# press gets a long grace period and the rest get a short one
STOP_AFTER = 0.1
FIRST_HOLD = 0.7
REPEAT_GAP = 0.25

DEBOUNCE = 0.05

# --- speed control -------------------------------------------------
# control loop period. at 150 rpm and 693 pulses a revolution a 0.05 s
# window holds about 87 pulses, so speed is quantised to about 1 percent
CONTROL_INTERVAL = 0.05

# measured 13 Oct, wheels off the ground, duty to rpm:
#   0.05   15 / 10      0.25   175 / 177
#   0.10   91 / 96      0.30   187 / 189
#   0.15  134 / 138     0.50   212 / 211
#   0.20  159 / 162     1.00   234 / 228
# the plant is steep at the bottom and nearly flat above 0.3. local gain
# near duty 0.15 is (159 - 91) / (0.20 - 0.10) = 680 rpm per unit duty,
# which is where these starting gains come from: correcting a 20 rpm
# error wants about 0.03 duty, so Kp is about 0.0015. started lower.
# TUNED 13 Oct, wheels off the ground, target 150 rpm. three runs:
#   Kp 0.001 Ki 0.002   90% in 1.7 s, no overshoot, band 147-152
#   Kp 0.002 Ki 0.004   90% in 0.3 s, slight dip,   band 146-156
#   Kp 0.004 Ki 0.008   90% in 0.2 s, overshoots to 166 then rings
#                       with a 0.35 to 0.4 s period, band 144-158
# oscillation starts at 0.004, so back off to half of it
KP = 0.002
KI = 0.004
KD = 0.0

# below about 0.05 duty the motors do not turn at all
DEADBAND = 0.05

# measured across the wheel with a ruler, 13 Oct
WHEEL_DIAMETER_CM = 4.8
WHEEL_CIRCUMFERENCE_CM = 3.14159 * WHEEL_DIAMETER_CM   # 15.1 cm


# effective track width, calibrate.py circle at 20 / 40 rpm, 2 turns,
# 29 Oct: left 13.10 13.07 13.22, right 13.28 13.34 13.16. 22 percent
# above the 10.8 cm ruler measurement centre to centre: the tyres slip
# sideways in a turn, so the rover turns less than the geometry says
# 2 Nov, Mani's spin method instead: 2 turns on the spot at 60 rpm a
# wheel, 12.59 12.78 12.64 with the push values above. the convoy copies
# his map, which he measured this way, so his method is used here
TRACK_CM = 12.67

# cm per encoder pulse, per wheel. calibrate.py roll, 2 m, three runs,
# 29 Oct: left 0.02036 0.02048 0.02039, right 0.02031 0.02044 0.02037.
# 6.6 percent below circumference / PULSES_PER_REV (0.02176), likely the
# tyres squashing under the rover's weight
# 2 Nov, Mani's push method instead: pushed by hand 100 cm, motors off,
# three times: left 0.02032 0.02017 0.02014, right 0.02040 0.02020
# 0.02015. both wires counted 4.00 per pulse, so the counting is clean.
# about 1 percent under the rolls, which slip a little under power
LEFT_CM_PER_PULSE = 0.02021
RIGHT_CM_PER_PULSE = 0.02025


def rpm_to_cm_s(rpm):
    """Wheel speed in rpm to ground speed in cm per second."""
    return rpm / 60 * WHEEL_CIRCUMFERENCE_CM


def turn_rate_deg_s(left_rpm, right_rpm, track_cm=TRACK_CM):
    """Signed wheel speeds to how fast the rover rotates, degrees/s.

    Positive is a left turn. A spin on the spot is equal and opposite
    wheel speeds, which gives twice the rate of one wheel alone.
    """
    diff = rpm_to_cm_s(right_rpm) - rpm_to_cm_s(left_rpm)
    return math.degrees(diff / track_cm)


# --- colour sensor -------------------------------------------------
# 100 ms and gain 4 as in Lab 2. at the default 2.4 ms the blue line
# read (4, 3, 3, 11), the same as no light. 20 Oct
COLOUR_INTEGRATION_MS = 10
COLOUR_GAIN = 4  # at 10 ms the clear count tops out at 4096; gain 16 put white at the top

# (red, green, blue, clear) on the white canvas, Exhibit 10A, 20 Oct.
# depends on room light and sensor height, measure again each session
WHITE = (6640, 5100, 2815, 15100)
# clear above this fraction of white counts as white. the brightest lines,
# red and green, measured 0.41
WHITE_FRACTION = 0.6

# a colour must beat the next ratio by this factor, or it is grey. the
# lines measured 1.5 (green) to 1.9, the grey table 1.0. 20 Oct
COLOUR_MARGIN = 1.25

# on an edge, one neighbour's ratio must beat the other's by this to say
# which side. edges measured 1.24 (blue/green) and 1.55 (blue/red), the
# floor 1.0 to 1.02. 20 Oct
EDGE_MARGIN = 1.1

# the stripes left to right, seen in the direction the rover drives.
# driving the track the other way round reverses this
TRACK_ORDER = ("red", "blue", "green")


# --- convoy --------------------------------------------------------
# convoy_slave.py scales the master's map by these before following it,
# to match this rover to the master's counting. 1.0 is no change. tuned
# on closed laps, see convoy_slave.py and tune_scale.py
CONVOY_HEADING_SCALE = 1.0
CONVOY_DIST_SCALE = 1.0
# from the front of this rover (the IR sensor) back to its axle, the
# wheel centres, measured 2 Nov. and from where the IR hits Mani's car to
# its axle: his notes give a 14 cm body with the nose 9.5 cm ahead of the
# axle. his spoiler sticks out further: axle to the back of the spoiler
# 12 to 13 cm, measured 2 Nov
# the gap the slave keeps, nose to tail: the middle of the page's 20 to 40
CONVOY_NOSE_TO_TAIL_CM = 30.0
NOSE_TO_AXLE_CM = 7.0
LEADER_TAIL_TO_AXLE_CM = 12.5
# power to cm/s on the floor for the convoy slave, from floortable.py.
# None uses the wheels-in-the-air table in convoy_slave.py
FLOOR_SPEED_TABLE = None

# --- per rover -----------------------------------------------------
# another rover running this code puts its own pins and calibration in
# config_local.py (gitignored), e.g. LEFT_CM_PER_PULSE = 0.0205, so it
# never overwrites these
try:
    from config_local import *  # noqa: F401,F403
except ImportError:
    pass

# --- edge following ------------------------------------------------
# clear count with the sensor fully on the black line and fully on the
# white floor, at the integration and gain above. measure both with
# colourtest.py before running edge.py
EDGE_BLACK = None
EDGE_WHITE = None
