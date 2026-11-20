"""Rotor torque factor for the airflow through the rotor (momentum theory) [S10]. Exact copy of
rotor_inflow_factor.m, which the six BLDC motor blocks of hexa_waypoints.slx call:

    Q = d·ω² · f,   f = (v_i + V_c) / v_h

v_h = √(T/(2ρA)) is the hover induced velocity at the rotor's own thrust T = b·ω², v_i the induced velocity with
the airflow, V_c the flow through the disc (+ = enters from above: climb, or the disc tilted into the wind).
f = 1 with no airflow. Climb/forward flight: Glauert; descent: Leishman's empirical axial curve normalised to 1 at
V_c = 0 times the in-plane Glauert reduction; windmill branch below V_c = −2 v_h.

The body-frame air velocity of a steady phase comes from load_model.steady_attitude.
"""
from __future__ import annotations

import numpy as np

RHO, R_PROP, C_T = 1.225, 0.129, 0.0141
A_PROP = np.pi * R_PROP**2
B_THRUST = C_T * RHO * A_PROP * R_PROP**2


def _factor(omega_sq, vx, vy, vz):
    T = B_THRUST * max(omega_sq, 0.0)
    vh = np.sqrt(T / (2 * RHO * A_PROP))
    if vh < 0.05:
        return 1.0
    Vt = np.hypot(vx, vy) / vh
    Vc = vz / vh
    g = np.sqrt((-Vt**2 + np.sqrt(Vt**4 + 4)) / 2)
    if Vc >= 0:
        x = 1.0
        for _ in range(30):
            s = Vt**2 + (Vc + x) ** 2
            F = x * x * s - 1
            if abs(F) < 1e-12:
                break
            x -= F / (2 * x * s + 2 * x * x * (Vc + x))
    elif Vc >= -2:
        x = (1.15 - 1.125 * Vc - 1.372 * Vc**2 - 1.718 * Vc**3 - 0.655 * Vc**4) / 1.15 * g
    else:
        x = (-Vc / 2 - np.sqrt(Vc**2 / 4 - 1)) * g
    return max(x + Vc, 0.0)


def inflow_factor(omega_sq, v_air_b):
    """f for rotor speed(s) squared and body-frame air velocity [x, y, z] (broadcasts over omega_sq)."""
    v = np.asarray(v_air_b, float)
    w = np.atleast_1d(np.asarray(omega_sq, float))
    out = np.array([_factor(o, v[0], v[1], v[2]) for o in w])
    return out if np.ndim(omega_sq) else float(out[0])

