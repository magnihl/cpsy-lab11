"""TCS34725 colour sensor over pigpio's I2C.

Lab 2 read this sensor through Adafruit Blinka, which lives in a
separate virtualenv from gpiozero and pigpio. Lab 10 needs the colour
sensor and the motors in one program, so it is read here through pigpio
instead, the same move adc.py made for the MCP3008 in Lab 9.

Registers are reached through a command byte: 0x80 marks it as a
command, 0x20 makes a block read step through consecutive registers.
The four 16 bit counts sit little endian from 0x14: clear, red, green,
blue.
"""

import time

import config
import rover

ADDRESS = 0x29
BUS = 1

COMMAND = 0x80
AUTO_INCREMENT = 0x20

ENABLE = 0x00
ATIME = 0x01
CONTROL = 0x0F
ID = 0x12
CDATA = 0x14

POWER_ON = 0x01
RGBC_ENABLE = 0x02

# CONTROL register value for each gain
GAINS = {1: 0x00, 4: 0x01, 16: 0x02, 60: 0x03}
# TCS34725 is 0x44, TCS34727 is 0x4D, same registers
CHIP_IDS = (0x44, 0x4D)
CYCLE_MS = 2.4

COLOURS = ("red", "green", "blue")


class ColourSensor:
    def __init__(self, integration_ms=config.COLOUR_INTEGRATION_MS,
                 gain=config.COLOUR_GAIN):
        self._pi = rover._connect()
        self._handle = self._pi.i2c_open(BUS, ADDRESS)
        chip = self._pi.i2c_read_byte_data(self._handle, COMMAND | ID)
        if chip not in CHIP_IDS:
            raise RuntimeError(f"0x{chip:02x} at 0x29 is not a TCS34725")

        # same rounding as the Adafruit library, so 100 ms here matches
        # the Lab 2 and Exhibit 10A readings
        cycles = max(1, min(256, int(integration_ms / CYCLE_MS)))
        self._write(ATIME, 256 - cycles)
        self._write(CONTROL, GAINS[gain])
        # the datasheet wants 2.4 ms between power on and enabling the ADC
        self._write(ENABLE, POWER_ON)
        time.sleep(0.003)
        self._write(ENABLE, POWER_ON | RGBC_ENABLE)
        # the first result is ready one integration later, reads before
        # that are zeros
        time.sleep(cycles * CYCLE_MS / 1000 + 0.01)

    def _write(self, register, value):
        self._pi.i2c_write_byte_data(self._handle, COMMAND | register, value)

    def read(self):
        """Latest (red, green, blue, clear), the order Blinka's color_raw uses."""
        _, data = self._pi.i2c_read_i2c_block_data(
            self._handle, COMMAND | AUTO_INCREMENT | CDATA, 8)
        c, r, g, b = (data[i] | data[i + 1] << 8 for i in range(0, 8, 2))
        return r, g, b, c

    def close(self):
        self._write(ENABLE, 0x00)
        self._pi.i2c_close(self._handle)


def classify(reading, white=config.WHITE):
    """Name what the sensor sees: "white", "grey", "red", "green" or "blue".

    The LED is weak in blue, so the raw blue count is never the largest,
    even on the blue line. Each channel is divided by its white reading
    first, and then a line's own channel comes out on top (Exhibit 10A).
    """
    r, g, b, c = reading
    if c / white[3] > config.WHITE_FRACTION:
        return "white"
    ratios = (r / white[0], g / white[1], b / white[2])
    first, second = sorted(ratios, reverse=True)[:2]
    # grey is dim white, all three ratios about equal. without this the
    # grey table read 0.31 / 0.31 / 0.31 and came out "red" by a hair
    if first < config.COLOUR_MARGIN * second:
        return "grey"
    return COLOURS[ratios.index(first)]


def side_of(seen, target, order=config.TRACK_ORDER):
    """Which side of the target stripe the sensor has drifted to.

    Returns "left", "right", or None when it is on the target or cannot
    tell. White next to an outer stripe gives the side; white next to the
    middle stripe could be either edge, so it gives None. Grey is off the
    canvas altogether, so it gives None too.
    """
    if seen == target or seen == "grey":
        return None
    if seen == "white":
        if target == order[0]:
            return "left"
        if target == order[-1]:
            return "right"
        return None
    return "left" if order.index(seen) < order.index(target) else "right"


def edge_side(reading, target, order=config.TRACK_ORDER, white=config.WHITE):
    """On the edge between the target and a neighbour, which side it is.

    A half and half reading fails classify's margin and comes out grey,
    but the neighbour's channel is still raised: on the blue/green edge
    green is above red, on the blue/red edge red is above green. Only
    works for the middle stripe, which has a colour on both sides.
    Returns "left", "right", or None for an outer stripe or the floor,
    where the two are about equal.
    """
    i = order.index(target)
    if i == 0 or i == len(order) - 1:
        return None
    r, g, b, _ = reading
    ratio = dict(zip(COLOURS, (r / white[0], g / white[1], b / white[2])))
    left, right = ratio[order[i - 1]], ratio[order[i + 1]]
    if max(left, right) < config.EDGE_MARGIN * min(left, right):
        return None
    return "left" if left > right else "right"


def drift(reading, target, order=config.TRACK_ORDER, white=config.WHITE):
    """How far the sensor sits towards the right neighbour, as a number.

    The right neighbour's ratio minus the left one's, each divided by
    white. On the middle stripe it rises as the sensor creeps right and
    falls as it creeps left, before the reading turns into either
    colour. Measured 20 Oct on blue: about +0.07 centred, +0.11 on the
    blue/green edge, +0.16 on green, -0.32 on red. None for an outer
    stripe, which has white on one side.
    """
    i = order.index(target)
    if i == 0 or i == len(order) - 1:
        return None
    r, g, b, _ = reading
    ratio = dict(zip(COLOURS, (r / white[0], g / white[1], b / white[2])))
    return ratio[order[i + 1]] - ratio[order[i - 1]]


def edge_read(reading, black=None, white=None):
    """How much of the sensor spot sees white: 0 all black, 1 all white.

    Sitting on the edge of the line gives 0.5, the point edge.py steers to.
    """
    black = config.EDGE_BLACK if black is None else black
    white = config.EDGE_WHITE if white is None else white
    clear = reading[3]
    return max(0.0, min(1.0, (clear - black) / (white - black)))
