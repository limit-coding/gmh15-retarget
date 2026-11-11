"""One-Euro filter (Casiez et al. 2012): low lag when moving fast, strong smoothing when still."""
import math

import numpy as np


class OneEuro:
    def __init__(self, min_cutoff=1.0, beta=0.3, d_cutoff=1.0):
        self.min_cutoff, self.beta, self.d_cutoff = min_cutoff, beta, d_cutoff
        self.reset()

    def reset(self):
        self._x = None
        self._dx = None
        self._t = None

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, x, t):
        x = np.asarray(x, dtype=float)
        if self._x is None or self._t is None or t <= self._t:
            self._x, self._dx, self._t = x.copy(), np.zeros_like(x), t
            return x.copy()
        dt = t - self._t
        dx = (x - self._x) / dt
        a_d = self._alpha(self.d_cutoff, dt)
        self._dx = a_d * dx + (1 - a_d) * self._dx
        cutoff = self.min_cutoff + self.beta * np.abs(self._dx)
        a = 1.0 / (1.0 + (1.0 / (2 * math.pi * cutoff)) / dt)
        self._x = a * x + (1 - a) * self._x
        self._t = t
        return self._x.copy()


class MotionShaper:
    """Second-order follower: position tracks the target through a velocity that is itself
    rate-limited (bounded acceleration), so the output has no steps and no velocity jumps.

      v_des = sign(e)·min(|e|/tau, v_max, sqrt(2·a_max·|e|))   (brakes in time: no hard stop)
      v    += clip(v_des - v, ±a_max·dt)
      pos  += v·dt              (overshoot-free: v is also clipped to what reaches the target)

    target=None -> keep following the last target (hold). Units: deg, s.
    """

    def __init__(self, v_max=120.0, a_max=600.0, tau=0.12, max_dt=0.1):
        self.v_max, self.a_max, self.tau, self.max_dt = v_max, a_max, tau, max_dt
        self.reset()

    def reset(self, pos=None):
        self.pos = None if pos is None else np.asarray(pos, float).copy()
        self.vel = None if pos is None else np.zeros_like(self.pos)
        self.target, self._t = None, None

    def __call__(self, target, t):
        if target is not None:
            self.target = np.asarray(target, float).copy()
        if self.pos is None:
            if self.target is None:
                return None
            self.reset(self.target)
            self.target, self._t = self.pos.copy(), t
            return self.pos.copy()
        dt = 0.0 if self._t is None else min(max(t - self._t, 0.0), self.max_dt)
        self._t = t
        if dt > 0 and self.target is not None:
            err = self.target - self.pos
            mag = np.minimum(np.minimum(np.abs(err) / self.tau, self.v_max), np.sqrt(2 * self.a_max * np.abs(err)))
            v_des = np.sign(err) * mag
            self.vel += np.clip(v_des - self.vel, -self.a_max * dt, self.a_max * dt)
            step = self.vel * dt
            # never step past the target (no overshoot); kill velocity on arrival
            over = np.abs(step) > np.abs(err)
            step[over] = err[over]
            self.vel[over] = 0.0
            self.pos += step
        return self.pos.copy()
