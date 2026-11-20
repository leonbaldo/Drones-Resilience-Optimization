"""Control allocation [G23, DATASET.md §15]: wrench u = [T, Mx, My, Mz] -> six commanded squared rotor speeds.

Python mirror of the allocation MATLAB Function in hexa_waypoints.slx (patched at session start by sim.py when a flight
uses mode 1; mode 0 is the original block, unchanged). Keep both in step: tests/test_allocation.py checks them.

mode 0 (legacy, every flight before G23): ω² = S·pinv(F·S)·u, S = diag(1/√w), then the plain clip to [200², 840²].
mode 1 (G23):
    thrust  : B_T·T with B_T = (1/b)·[s14, s25, s36, s14, s25, s36]/2  (pair shares; moment-free by construction)
    moments : B_M·[Mx, My, Mz], B_M = columns 2-4 of S_M·pinv(F·S_M), S_M = diag(1/√w_M)
    limits  : [200², 840²] per motor, M1's ceiling lowered to om1_max² (derating)
    sat 0 S0 clip         : each motor clipped on its own
    sat 1 S1 priority     : keep roll/pitch; shed yaw first; then shift the collective thrust; scale roll/pitch last
    sat 2 S2 redistribute : redistributed pseudo-inverse (Bodson 2002): saturated motors are fixed at their limit and
                            the rest of the wrench is re-allocated to the free ones, roll/pitch weighted above yaw
Only pinv, min/max and fixed-count loops, so the MATLAB copy stays R2022b code-generation compatible.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# geometry, as the MATLAB Function (l, r, C_T, C_Q, rho)
_L, _R, _CT, _CQ, _RHO = 0.30, 0.129, 0.0141, 0.0012, 1.225
_A = np.pi * _R ** 2
B_COEF = _CT * _RHO * _A * _R ** 2
D_COEF = _CQ * _RHO * _A * _R ** 3
_b, _d, _l = B_COEF, D_COEF, _L
FORCES = np.array([
    [_b, _b, _b, _b, _b, _b],
    [0, _b * _l * np.sqrt(3) / 2, _b * _l * np.sqrt(3) / 2, 0, -_b * _l * np.sqrt(3) / 2, -_b * _l * np.sqrt(3) / 2],
    [-_b * _l, -_b * _l / 2, _b * _l / 2, _b * _l, _b * _l / 2, -_b * _l / 2],
    [_d, -_d, _d, -_d, _d, -_d]])
OMEGA2_MIN, OMEGA2_MAX = 200.0 ** 2, 840.0 ** 2
PAIRS = (0, 1, 2, 0, 1, 2)                 # motor i belongs to pair PAIRS[i]: (M1, M4), (M2, M5), (M3, M6)
S2_ROW_WEIGHTS = np.array([1.0, 1.0, 1.0, 0.1])   # S2: thrust, roll, pitch above yaw when not everything fits
SAT_NAMES = ("S0 clip", "S1 priority", "S2 redistribute")


@dataclass
class AllocConfig:
    """One allocation configuration (part of C). mode 0 uses only `weights`."""
    mode: int = 0
    weights: np.ndarray = field(default_factory=lambda: np.ones(6))        # mode 0
    shares: np.ndarray = field(default_factory=lambda: np.full(3, 1 / 3))  # mode 1: s14, s25, s36 (sum 1)
    w_m: np.ndarray = field(default_factory=lambda: np.ones(6))            # mode 1: moment weights
    sat: int = 0                                                           # mode 1: 0 S0, 1 S1, 2 S2
    om1_max: float = 840.0                                                 # mode 1: M1 ceiling (rad/s)

    def as_dict(self):
        return dict(mode=int(self.mode), weights=np.asarray(self.weights, float).tolist(),
                    shares=np.asarray(self.shares, float).tolist(), w_m=np.asarray(self.w_m, float).tolist(),
                    sat=int(self.sat), om1_max=float(self.om1_max))

    @classmethod
    def legacy(cls, weights):
        return cls(mode=0, weights=np.asarray(weights, float))


def bounds(cfg: AllocConfig):
    lo = np.full(6, OMEGA2_MIN)
    hi = np.full(6, OMEGA2_MAX)
    if cfg.mode == 1:
        hi[0] = min(OMEGA2_MAX, float(cfg.om1_max) ** 2)
    return lo, hi


def thrust_column(shares):
    s = np.asarray(shares, float)
    return np.array([s[p] / 2 for p in PAIRS]) / B_COEF


def moment_columns(w_m):
    S = np.diag(1 / np.sqrt(np.asarray(w_m, float)))
    return (S @ np.linalg.pinv(FORCES @ S))[:, 1:4]


def _feasible_t_interval(base, bt, lo, hi):
    """Interval of the collective thrust T for which base + bt·T stays within [lo, hi] (bt > 0)."""
    t_lo = np.max((lo - base) / bt)
    t_hi = np.min((hi - base) / bt)
    return t_lo, t_hi


def _yaw_scale(x, by, lo, hi):
    """Largest γ in [0, 1] with lo <= x + γ·by <= hi (x itself inside)."""
    g = 1.0
    for i in range(6):
        if by[i] > 1e-12:
            g = min(g, (hi[i] - x[i]) / by[i])
        elif by[i] < -1e-12:
            g = min(g, (lo[i] - x[i]) / by[i])
    return max(0.0, g)


def _priority(u, bt, bm, lo, hi):
    """S1: roll/pitch first, yaw shed first, then the collective, roll/pitch scaled last."""
    T, mrp, mz = u[0], u[1:3], u[3]
    brp, by = bm[:, 0:2] @ mrp, bm[:, 2] * mz
    x = bt * T + brp + by
    if np.all(x >= lo) and np.all(x <= hi):
        return x
    x_rp = bt * T + brp
    if np.all(x_rp >= lo) and np.all(x_rp <= hi):          # shed yaw only
        return x_rp + _yaw_scale(x_rp, by, lo, hi) * by
    t_lo, t_hi = _feasible_t_interval(brp, bt, lo, hi)      # shift the collective for roll/pitch
    if t_lo <= t_hi:
        T2 = min(max(T, t_lo), t_hi)
        x_rp = bt * T2 + brp
        return x_rp + _yaw_scale(x_rp, by, lo, hi) * by
    a_lo, a_hi = 0.0, 1.0                                   # scale roll/pitch (bisection, fixed count)
    for _ in range(30):
        a = 0.5 * (a_lo + a_hi)
        t_lo, t_hi = _feasible_t_interval(a * brp, bt, lo, hi)
        if t_lo <= t_hi:
            a_lo = a
        else:
            a_hi = a
    t_lo, t_hi = _feasible_t_interval(a_lo * brp, bt, lo, hi)
    T2 = min(max(T, t_lo), max(t_lo, t_hi))
    return np.clip(bt * T2 + a_lo * brp, lo, hi)


def _redistribute(u, x0, w_m, lo, hi):
    """S2: redistributed pseudo-inverse. Fix the violators at their limit, re-allocate the rest to the free motors
    (weighted by w_m, rows weighted S2_ROW_WEIGHTS), repeat (at most 6 passes), final clip."""
    x = x0.copy()
    free = np.ones(6, bool)
    Q = np.diag(S2_ROW_WEIGHTS)
    sw = 1 / np.sqrt(np.asarray(w_m, float))
    for _ in range(6):
        over, under = x > hi + 1e-9, x < lo - 1e-9
        viol = (over | under) & free
        if not viol.any():
            break
        x[over & free] = hi[over & free]
        x[under & free] = lo[under & free]
        free &= ~viol
        if not free.any():
            break
        u_r = u - FORCES @ (x * ~free)                      # masked form (fixed-size, as the MATLAB copy)
        S = np.diag(sw * free)
        xn = S @ np.linalg.pinv(Q @ FORCES @ S) @ (Q @ u_r)
        x[free] = xn[free]
    return np.clip(x, lo, hi)


def allocate(u, cfg: AllocConfig, clip=True):
    """Commanded squared rotor speeds for wrench u. clip=True also applies the limits of mode 0 (the downstream
    Saturation block of the model), so the result is what reaches the motors in every mode."""
    u = np.asarray(u, float)
    if cfg.mode == 0:
        w = np.asarray(cfg.weights, float)
        S = np.diag(1 / np.sqrt(w))
        x = S @ np.linalg.pinv(FORCES @ S) @ u
        return np.clip(x, OMEGA2_MIN, OMEGA2_MAX) if clip else x
    lo, hi = bounds(cfg)
    bt, bm = thrust_column(cfg.shares), moment_columns(cfg.w_m)
    x = bt * u[0] + bm @ u[1:4]
    if cfg.sat == 0:
        x = np.clip(x, lo, hi)
    elif cfg.sat == 1:
        x = _priority(u, bt, bm, lo, hi)
    else:
        x = _redistribute(u, x, cfg.w_m, lo, hi)
    return np.clip(x, OMEGA2_MIN, OMEGA2_MAX) if clip else x


def hover_shares_feasible(shares, mass_kg, g=9.81):
    """[G23] The thrust pair shares give every motor a hover command inside [200², 840²] (healthy, any voltage:
    commands only)."""
    x = thrust_column(shares) * mass_kg * g
    return bool(np.all(x >= OMEGA2_MIN) and np.all(x <= OMEGA2_MAX))
