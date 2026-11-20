"""Python copy of the battery block in hexa_waypoints.slx, plus the column map of the logged `dd2` matrix.

Simulink block "Battery" (batt_new_conti4.m + OCV MATLAB Function), one-RC Thevenin model:
    dSoC/dt = -I / Ct                      Ct = 6.2 Ah · 3600
    dVd/dt  = -Vd / (Rd·Cd) + I / Cd       Rd = 0.0237 Ω, Cd = 2400 F  (τ ≈ 57 s)
    V       = OCV(SoC) - Vd - R0·I         R0 = 0.03 Ω
with OCV a 6th-order polynomial + s·ln(s) term (4S LiPo: 16.79 V full, 14.17 V empty).
`check_replay` verifies on a logged flight that this copy reproduces the Simulink signals.  [ASSUMPTIONS B0]
"""
from __future__ import annotations

import numpy as np

# dd2 columns (verified 2026-11-09 from the logged signals)
DD2 = dict(ref_x=0, x=1, ref_y=2, y=3, ref_z=4, z=5, pwm=slice(24, 30), pwm_m1=24, motor_current=slice(30, 36),
           current=36, voltage=37, soc=38, t=39, temperature=40)

OCV_THETA = (14.1649289567373, -21.1041837178247, 58.6627475274973, -76.8160652816408,
             61.9133146064340, -20.0350914742760, -9.94858967091727)
CAPACITY_AH = 6.2
CT = CAPACITY_AH * 3600.0
R0, RD, CD = 0.03, 0.0237, 2400.0
TAU = RD * CD


def ocv(soc):
    s = np.maximum(np.asarray(soc, float), 1e-6)
    t = OCV_THETA
    return t[0] + t[1] * s + t[2] * s**2 + t[3] * s**3 + t[4] * s**4 + t[5] * s**5 + t[6] * s * np.log(s)


def step(soc, vd, current, dt):
    """Exact zero-order-hold discretisation over dt (current constant in the step)."""
    a = np.exp(-dt / TAU)
    return soc - current * dt / CT, a * vd + RD * (1 - a) * current


def terminal_voltage(soc, vd, current):
    return ocv(soc) - vd - R0 * current


def replay(t, current, soc0=1.0, vd0=0.0):
    """Forward-Euler replay at the logged time stamps, as the Simulink ode1 integrator does."""
    t = np.asarray(t, float); i = np.asarray(current, float)
    soc = np.empty_like(t); vd = np.empty_like(t)
    soc[0], vd[0] = soc0, vd0
    dt = np.diff(t)
    for k in range(len(t) - 1):
        soc[k + 1] = soc[k] - i[k] * dt[k] / CT
        vd[k + 1] = vd[k] + dt[k] * (-vd[k] / TAU + i[k] / CD)
    return soc, vd, terminal_voltage(soc, vd, i)


def check_replay(dd2):
    """Max |error| of the Python copy against a logged flight: (SoC, voltage in V)."""
    t, i = dd2[:, DD2["t"]], dd2[:, DD2["current"]]
    soc, _, v = replay(t, i)
    return float(np.abs(soc - dd2[:, DD2["soc"]]).max()), float(np.abs(v - dd2[:, DD2["voltage"]]).max())


def energy_wh(dd2):
    t = dd2[:, DD2["t"]]
    p = dd2[:, DD2["current"]] * dd2[:, DD2["voltage"]]
    return float(np.sum(np.diff(t) * (p[1:] + p[:-1]) / 2) / 3600)
