#!/usr/bin/python3
import math


def exp(k, *args):
    p, t = k
    x, y, wp, v = args
    dx, dy = camera2muzzle(p)
    return [-1000 * (t ** 2) + (6000 - 2500 * p) * t - ((x + dx) * 371.43 + v * t),
            (2600 * p - 2200) * (t ** 2) + 5000 * p * t - ((y + dy) * 371.43 + wp * t)]
# (2600 * p - 2200) * (t ** 2) + 5000 * p * t - (y - 0.4 * math.sin(p)) * 371.43]


def camera2muzzle(pitch):
    length = -0.02
    dx = math.cos(pitch) * length
    dy = math.sin(pitch) * length
    if pitch > 0:
        return [-dx, dy]
    else:
        return [dx, -dy]
