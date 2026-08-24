"""Quintic (5th-order) joint-space trajectories.

Smooth pose-to-pose with zero velocity AND acceleration at both ends -> no jerk
step, matches the conservative ESP32 jog limits.  No path planning: straight
interpolation between preset joint poses (standby / receive / slosh ...).
"""
from __future__ import annotations

from dataclasses import dataclass, field


def quintic_coeffs(q0, qf, T, v0=0.0, vf=0.0, a0=0.0, af=0.0):
    """5th-order polynomial coeffs (c0..c5) for q0->qf over time T.

    Boundary conditions: position q0/qf, velocity v0/vf, acceleration a0/af.
    """
    if T <= 0.0:
        raise ValueError('T must be > 0')
    T2 = T * T
    T3 = T2 * T
    T4 = T3 * T
    T5 = T4 * T
    c0 = q0
    c1 = v0
    c2 = 0.5 * a0
    c3 = (20 * (qf - q0) - (8 * vf + 12 * v0) * T - (3 * a0 - af) * T2) / (2 * T3)
    c4 = (30 * (q0 - qf) + (14 * vf + 16 * v0) * T + (3 * a0 - 2 * af) * T2) / (2 * T4)
    c5 = (12 * (qf - q0) - (6 * vf + 6 * v0) * T - (a0 - af) * T2) / (2 * T5)
    return (c0, c1, c2, c3, c4, c5)


def quintic_eval(c, t):
    """Return (pos, vel, acc) at time t for coeffs c."""
    pos = c[0] + c[1] * t + c[2] * t**2 + c[3] * t**3 + c[4] * t**4 + c[5] * t**5
    vel = c[1] + 2 * c[2] * t + 3 * c[3] * t**2 + 4 * c[4] * t**3 + 5 * c[5] * t**4
    acc = 2 * c[2] + 6 * c[3] * t + 12 * c[4] * t**2 + 20 * c[5] * t**3
    return (pos, vel, acc)


@dataclass
class JointTrajectory:
    """Multi-joint quintic from start[] to goal[] over duration T [s]."""
    start: list
    goal: list
    T: float
    _coeffs: list = field(default_factory=list, init=False, repr=False)

    def __post_init__(self):
        if len(self.start) != len(self.goal):
            raise ValueError('start/goal length mismatch')
        self._coeffs = [quintic_coeffs(s, g, self.T)
                        for s, g in zip(self.start, self.goal)]

    def sample(self, t):
        """Joint positions [rad] at time t (clamped to [0, T])."""
        t = max(0.0, min(self.T, t))
        return [quintic_eval(c, t)[0] for c in self._coeffs]

    def sample_vel(self, t):
        """Joint velocities [rad/s] at time t (clamped to [0, T])."""
        t = max(0.0, min(self.T, t))
        return [quintic_eval(c, t)[1] for c in self._coeffs]

    def done(self, t):
        return t >= self.T
