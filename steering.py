"""Line steering, shared by follow.py (Lab 10) and convoy.py (Lab 11).

Moved out of follow.py unchanged, except that rpm, the stripe order and
the centre are now arguments instead of globals. replay_steer.py runs
this and the Lab 10 version on every Lab 10 log to show they agree.
"""

from colour import drift, edge_side, side_of

# inner wheel speed, as a share of rpm, on the edge between two
# stripes. stopping it outright made every small drift on the straight a
# hard correction, but half speed on a whole neighbour stripe was too
# weak for the curves, it swung out onto white. 20 Oct
SOFT_TURN = 0.5
# proportional steering on the middle stripe: inner wheel share is
# 1 - STEER_GAIN x drift from centre. the blue/green edge is about 0.04
# from centre, so 12 gives about half speed there, as SOFT_TURN did
STEER_GAIN = 12.0


def steer(seen, reading, target, last_turn, rpm, order, centre):
    """Decide the wheel speeds for this loop.

    seen is what the sensor reads now: "white", "grey", "red", "green"
    or "blue", and reading the raw (red, green, blue, clear) it came
    from. side_of(seen, target) says where the sensor has drifted:
    "left", "right", or None when on the target or it cannot tell.
    last_turn is the last correction made: "left", "right" or None.
    rpm is the outer wheel speed, order the stripes left to right, and
    centre the drift read at start (None for an outer stripe).

    Returns (left_rpm, right_rpm, turn). The turn grows with the drift:
    on the middle stripe the inner wheel slows in proportion to the
    drift from centre, on an edge to SOFT_TURN or less, on a neighbour
    stripe or white past the outer stripes it stops.
    """
    if centre is not None and seen == target:
        error = drift(reading, target, order) - centre
        inner = rpm * max(0.0, 1.0 - STEER_GAIN * abs(error))
        # positive error is drift to the right, so turn left
        turn = "left" if error > 0 else "right"
        if turn == "right":
            return rpm, inner, turn
        return inner, rpm, turn
    if seen == target:
        # keep the last correction as memory: if the line is lost next,
        # the curve most likely bends the way it was last corrected
        return rpm, rpm, last_turn
    side = side_of(seen, target, order)
    inner = 0.0
    if seen == "grey":
        # an edge between two stripes: the mix says which side, so turn
        # before the reading becomes the neighbour. the side comes from
        # edge_side, which never got one wrong in the logs; a centre that
        # is slightly off could flip the sign of the error. the error only
        # makes the turn harder, never softer than SOFT_TURN
        side = edge_side(reading, target, order)
        inner = rpm * SOFT_TURN
        if centre is not None:
            error = drift(reading, target, order) - centre
            inner = min(inner, rpm * max(0.0, 1.0 - STEER_GAIN * abs(error)))
    if side == "left":
        turn = "right"
    elif side == "right":
        turn = "left"
    elif last_turn is not None:
        # no colour says where: keep turning the same way. on white, past
        # the outer stripes, that turn stays hard
        turn = last_turn
    else:
        return rpm, rpm, None
    if turn == "right":
        return rpm, inner, turn
    return inner, rpm, turn
