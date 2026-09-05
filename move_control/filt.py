"""Median + Fourier (1st-order low-pass) filter for jumpy sensors.

Cutoff a = 1 - exp(-2*pi*fc*dt) is the RC/Fourier low-pass at fc Hz.
Median (odd window) kills single-sample spikes before the low-pass.
"""
import math


class MedianLp:
    def __init__(self, median_n=5, fc_hz=1.5, dt=0.05):
        self.n = max(1, int(median_n) | 1)  # odd
        self.buf = []
        self.y = None
        self.set_cutoff(fc_hz, dt)

    def set_cutoff(self, fc_hz, dt):
        fc = max(0.05, float(fc_hz))
        dt = max(1e-3, float(dt))
        self.a = 1.0 - math.exp(-2.0 * math.pi * fc * dt)

    def push(self, x):
        if x is None or (isinstance(x, float) and (not math.isfinite(x) or x < 0.0)):
            return self.y
        self.buf.append(float(x))
        if len(self.buf) > self.n:
            del self.buf[0]
        med = sorted(self.buf)[len(self.buf) // 2]
        if self.y is None:
            self.y = med
        else:
            self.y += self.a * (med - self.y)
        return self.y

    def value(self):
        return self.y


class IrMedian:
    def __init__(self, n=5):
        self.n = max(1, int(n))
        self.ch = [[], [], []]

    def push(self, ir):
        out = []
        for i, v in enumerate(list(ir)[:3]):
            self.ch[i].append(int(v))
            if len(self.ch[i]) > self.n:
                del self.ch[i][0]
            s = sorted(self.ch[i])
            out.append(s[len(s) // 2])
        while len(out) < 3:
            out.append(out[-1] if out else 0)
        return tuple(out)
