"""White-box physics copied unchanged from the project package `resilience/` (2026-11-19).

allocation   mixer matrix FORCES (4x6), AllocConfig, allocate(u, cfg): the Simulink allocation block in Python
load_model   steady_state(mass, frictions, alloc, v_batt, v_h, v_z): closed-loop trim -> rotor speeds, PWM, currents
rotor        rotor inflow factor (airflow through the rotor changes thrust)
degradation  FrictionWear: the wear law db/dt = lam * b * (|w|/w_ref)^n, closed form
battery      4S LiPo 1-RC Thevenin model: ocv(soc), step(soc, vd, I, dt), replay(t, I)

Only the steady-state / closed-form functions are meant for use here. Functions that need the MATLAB engine or
the project's flight cache (run_sweep, fit_surrogates, flight_friction, ...) will not work outside the project.
"""
from . import allocation, load_model, rotor, degradation, battery  # noqa: F401
