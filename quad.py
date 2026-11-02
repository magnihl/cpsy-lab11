"""Quadrature counting of both encoder wires, through the kernel's GPIO
edge events (libgpiod), read in batches by one thread.

Each wheel's two wires are a quarter step apart, so the order in which
they change gives the direction: four counts per pulse of one wire,
signed. The kernel numbers every event, so a gap in the numbers is an
edge that was lost, counted in missed.

  enc = QuadEncoders({"left": (16, 26), "right": (5, 6)})
  enc.counts["left"]     signed running total
"""

import threading

import gpiod
from gpiod.line import Bias, Edge

CHIP = "/dev/gpiochip0"

# (old a, old b, new a, new b) to a step: +1 one way, -1 the other. both
# wires changing at once cannot be told apart, it counts as missed
STEP = {}
for a0 in (0, 1):
    for b0 in (0, 1):
        # one way round the four states is 00 01 11 10
        order = [(0, 0), (0, 1), (1, 1), (1, 0)]
        i = order.index((a0, b0))
        STEP[(a0, b0) + order[(i + 1) % 4]] = 1
        STEP[(a0, b0) + order[(i - 1) % 4]] = -1


class QuadEncoders:
    def __init__(self, pins):
        """pins: {wheel: (a, b)} as BCM numbers."""
        self.counts = dict.fromkeys(pins, 0)
        self.missed = 0
        self.events = 0
        self._wheel = {}
        self._is_a = {}
        for wheel, (a, b) in pins.items():
            self._wheel[a], self._is_a[a] = wheel, True
            self._wheel[b], self._is_a[b] = wheel, False
        offsets = tuple(self._wheel)
        self._request = gpiod.request_lines(
            CHIP, consumer="quad",
            config={offsets: gpiod.LineSettings(edge_detection=Edge.BOTH,
                                                bias=Bias.PULL_UP)})
        values = self._request.get_values(list(offsets))
        level = {o: int(v == gpiod.line.Value.ACTIVE)
                 for o, v in zip(offsets, values)}
        self._state = {w: [level[a], level[b]] for w, (a, b) in pins.items()}
        self._last_seqno = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._watch, daemon=True)
        self._thread.start()

    def _watch(self):
        rising = gpiod.EdgeEvent.Type.RISING_EDGE
        while not self._stop.is_set():
            if not self._request.wait_edge_events(0.1):
                continue
            for event in self._request.read_edge_events(256):
                seqno = event.global_seqno
                if self._last_seqno is not None:
                    self.missed += seqno - self._last_seqno - 1
                self._last_seqno = seqno
                self.events += 1
                offset = event.line_offset
                wheel = self._wheel[offset]
                state = self._state[wheel]
                old = tuple(state)
                state[0 if self._is_a[offset] else 1] = int(
                    event.event_type == rising)
                step = STEP.get(old + tuple(state))
                if step is None:
                    if old != tuple(state):
                        self.missed += 1
                else:
                    self.counts[wheel] += step

    def close(self):
        # let the thread finish its wait before the lines go, or it reads
        # a released request and dies with an exception
        self._stop.set()
        self._thread.join(timeout=0.5)
        self._request.release()
