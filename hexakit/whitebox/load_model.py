"""Pack current as a function of the operating case: white-box steady state, in-flight friction estimate,
Simulink sweep and a trained surrogate.

Signal chain in hexa_waypoints.slx (verified 2026-11-09):
    LQR u = [T, Mx, My, Mz] → allocation ω²_cmd = S·pinv(F·S)·u, S = diag(1/√w)        [MATLAB Function, chart_59]
    → Saturation [200², 840²] → √ → ω_ref → PWM = 0.0008842·ω_ref + 0.03436                 [BLDCMi/Fcn2]
    → BLDC: I_m = (V·PWM − K_E·ω)/R,  J·dω/dt = K_T·I_m − T_f − D_i·ω − d·ω²·f                [BLDCMi/BLDC1]
      f = rotor_inflow_factor(ω², v_air_b): airflow through the rotor, 1 at hover [S10, resilience/rotor.py]
    → battery current of motor i = 0.3·PWM·I_m; pack current = Σ_i                              [Gain 0.3, Subsystem]
    → thrust/torques from the actual ω²: F·ω²                                                   [chart_78, fault factors 1]
with D_i = FrictionCoefficient (degraded on M1 only), K_E = K_T = 9.549/920, R = 0.2 Ω, T_f = 0.04 N·m.

`steady_state` solves the closed-loop equilibrium (actual net thrust = |m·g − F_D|, zero moments) for given mass,
friction, allocation weights, pack voltage and steady velocity. The form drag F_D is the one added to
arellano_hexa_payload.m (Darrah et al. 2020, Eq. 7; ASSUMPTIONS L2), with the body tilted to the exact steady
equilibrium (`steady_attitude`; until 2026-11-11 a level body). The rotor inflow factor uses the same body-frame
air velocity [S10]. v_h is the
horizontal AIRSPEED (ground speed minus wind).
Assumptions: ASSUMPTIONS.md section L.
"""
from __future__ import annotations

import numpy as np
from scipy.optimize import fsolve

from .rotor import inflow_factor

# ---- parameters copied from the Simulink model -------------------------------------------------------------------
L_ARM, R_PROP, C_T, C_Q, RHO, G = 0.30, 0.129, 0.0141, 0.0012, 1.225, 9.81
A_PROP = np.pi * R_PROP**2
B_THRUST = C_T * RHO * A_PROP * R_PROP**2
D_DRAG = C_Q * RHO * A_PROP * R_PROP**3
KE = KT = 9.549 / 920
R_MOTOR, T_FRICTION = 0.2, 0.04
PWM_A, PWM_B = 0.0008842, 0.03436
OMEGA2_MIN, OMEGA2_MAX = 40000.0, 840.0**2
BATT_GAIN = 0.3
# Form drag, identical to arellano_hexa_payload.m [L2]
RHO_AIR, C_D = 1.225, 1.0
A_XY, A_XZ, A_YZ = 0.9 * (0.30 / 0.45) ** 2, 0.5 * (0.30 / 0.45) ** 2, 0.5 * (0.30 / 0.45) ** 2

_b, _d, _l = B_THRUST, D_DRAG, L_ARM
FORCES = np.array([
    [_b, _b, _b, _b, _b, _b],
    [0, _b * _l * np.sqrt(3) / 2, _b * _l * np.sqrt(3) / 2, 0, -_b * _l * np.sqrt(3) / 2, -_b * _l * np.sqrt(3) / 2],
    [-_b * _l, -_b * _l / 2, _b * _l / 2, _b * _l, _b * _l / 2, -_b * _l / 2],
    [_d, -_d, _d, -_d, _d, -_d]])


def allocate(u, weights):
    """Commanded squared rotor speeds, as the Simulink MATLAB Function, then the saturation. `weights` is either the
    six legacy weights (mode 0) or an allocation.AllocConfig (G23)."""
    from .allocation import AllocConfig, allocate as _alloc
    if isinstance(weights, AllocConfig):
        return _alloc(u, weights)
    w = np.asarray(weights, float)
    S = np.diag(1 / np.sqrt(w))
    return np.clip(S @ np.linalg.pinv(FORCES @ S) @ np.asarray(u, float), OMEGA2_MIN, OMEGA2_MAX)


def motor_steady(pwm, v_batt, friction, v_air_b=None):
    """Steady rotor speed, motor current and battery current of the motors for given PWM and pack voltage.
    v_air_b: body-frame air velocity for the rotor inflow factor [S10] (None or zero: f = 1)."""
    b = friction + KT * KE / R_MOTOR
    c = T_FRICTION - KT * v_batt * pwm / R_MOTOR
    f = np.ones_like(np.asarray(pwm, float))
    still = v_air_b is None or not np.any(v_air_b)
    for _ in range(1 if still else 50):          # f depends weakly on ω: fixed point
        a = D_DRAG * f
        omega = np.maximum((-b + np.sqrt(np.maximum(b * b - 4 * a * c, 0))) / (2 * a), 0.0)
        if still:
            break
        f_new = inflow_factor(omega**2, v_air_b)
        if np.max(np.abs(f_new - f)) < 1e-12:
            break
        f = f_new
    i_m = (v_batt * pwm - KE * omega) / R_MOTOR
    return omega, i_m, BATT_GAIN * pwm * i_m


def steady_attitude(mass_kg, v_h=0.0, v_z=0.0):
    """Exact steady equilibrium with the body-frame form drag of arellano_hexa_payload.m [L2], body tilted by α in
    the plane of the air velocity (v_h = horizontal airspeed ≥ 0, v_z = vertical speed, + up).
    Along x_b: m·g·sin α = c_yz·v_bx|v_bx|;  along z_b: T = m·g·cos α + c_xy·v_bz|v_bz|,
    with v_bx = v_h cos α − v_z sin α, v_bz = v_h sin α + v_z cos α. Returns (T, α, [v_bx, 0, v_bz])."""
    from scipy.optimize import brentq
    V, mg = abs(v_h), mass_kg * G
    c_yz, c_xy = 0.5 * RHO_AIR * C_D * A_YZ, 0.5 * RHO_AIR * C_D * A_XY
    vb = lambda a: (V * np.cos(a) - v_z * np.sin(a), V * np.sin(a) + v_z * np.cos(a))
    alpha = 0.0
    if V > 0:
        alpha = brentq(lambda a: mg * np.sin(a) - c_yz * vb(a)[0] * abs(vb(a)[0]), 0.0, 1.5, xtol=1e-14)
    bx, bz = vb(alpha)
    return mg * np.cos(alpha) + c_xy * bz * abs(bz), alpha, np.array([bx, 0.0, bz])


def required_thrust(mass_kg, v_h=0.0, v_z=0.0):
    """Thrust magnitude for steady flight at horizontal airspeed v_h and vertical speed v_z (tilted-body drag)."""
    return float(steady_attitude(mass_kg, v_h, v_z)[0])


def phase_air_velocity(mass_kg, v_h=0.0, v_z=0.0):
    """Body-frame air velocity of a steady phase, for the rotor inflow factor [S10]."""
    return steady_attitude(mass_kg, v_h, v_z)[2]


def steady_state(mass_kg, frictions, weights, v_batt, u0=None, v_h=0.0, v_z=0.0):
    """Closed-loop equilibrium: find u so that the actual rotor speeds give F·ω² = [T, 0, 0, 0], T = required_thrust.
    Returns a dict with u, per-motor ω_ref, PWM, ω, battery currents, pack current, saturation flags, residual."""
    fr = np.broadcast_to(np.asarray(frictions, float), (6,))
    target = np.array([required_thrust(mass_kg, v_h, v_z), 0, 0, 0])
    v_air_b = phase_air_velocity(mass_kg, v_h, v_z)

    def actual(u):
        w2 = allocate(u, weights)
        pwm = PWM_A * np.sqrt(w2) + PWM_B
        om, im, ib = motor_steady(pwm, v_batt, fr, v_air_b)
        return FORCES @ om**2, (w2, pwm, om, im, ib)

    u0 = np.array([mass_kg * G, 0, 0, 0]) if u0 is None else u0
    u, info, ier, _ = fsolve(lambda u: (actual(u)[0] - target) / np.array([10, 1, 1, 0.1]), u0, full_output=True)
    f, (w2, pwm, om, im, ib) = actual(u)
    return dict(u=u, omega_ref=np.sqrt(w2), pwm=pwm, omega=om, i_motor=im, i_batt=ib, current=float(ib.sum()),
                saturated=bool(np.any(w2 <= OMEGA2_MIN + 1e-6) or np.any(w2 >= OMEGA2_MAX - 1e-6)),
                residual=float(np.abs(f - target).max()), converged=ier == 1)


def current_vs_voltage(mass_kg, frictions, weights, v_grid, v_h=0.0, v_z=0.0):
    """Pack current at each voltage of v_grid (warm-started); NaN where no equilibrium exists."""
    out, u0 = [], None
    for v in v_grid:
        s = steady_state(mass_kg, frictions, weights, v, u0, v_h, v_z)
        ok = s["converged"] and s["residual"] < 1e-3
        out.append(s["current"] if ok else np.nan)
        u0 = s["u"] if ok else None
    return np.array(out)


# ---- friction estimated in flight from ESC telemetry ---------------------------------------------------------------
def estimate_friction(dd2, t0, t1, sigma_i=0.0, rng=None):
    """Per-motor viscous friction from logged PWM, per-motor battery current and pack voltage over [t0, t1],
    inverting the steady motor equations (ESC telemetry assumption). Optional Gaussian noise σ_i on the currents.
    Returns (D per motor, ω per motor) as window means."""
    t = dd2[:, 39]; m = (t >= t0) & (t <= t1)
    pwm = dd2[m, 24:30]; ib = dd2[m, 30:36].copy(); v = dd2[m, 37][:, None]
    if sigma_i and rng is not None:
        ib = ib + rng.normal(0, sigma_i, ib.shape)
    i_m = ib / (BATT_GAIN * pwm)
    omega = (v * pwm - R_MOTOR * i_m) / KE
    D = (KT * i_m - T_FRICTION - D_DRAG * omega**2) / omega
    return D.mean(axis=0), omega.mean(axis=0)


# ---- Simulink sweep ---------------------------------------------------------------------------------------------------
SWEEP_ROUTE = np.array([[0, 0, 0, 0], [20, 0, 0, 20], [30, 0, 0, 20], [70, 80, 0, 20], [85, 80, 0, 20], [105, 80, 0, 0]], float)
SWEEP_SPEEDS = {"climb": (0.0, 1.0), "hover": (0.0, 0.0), "cruise": (2.0, 0.0), "hover2": (0.0, 0.0),
                "descent": (0.0, -1.0)}   # (v_h, v_z) of the sweep reference in each window
SWEEP_WINDOWS = {"climb": (5.0, 18.0), "hover": (24.0, 30.0), "cruise": (40.0, 68.0), "hover2": (76.0, 85.0),
                 "descent": (90.0, 103.0)}


def sweep_points(cfg, seed=0, n_test=24):
    """Training grid (friction × mass × M1 weight) and random held-out points."""
    train = [dict(friction_factor=f, mass_kg=m, w1=w, split="train")
             for f in (1, 2, 3, 4, 5, 6, 7, 8) for m in (1.60, 1.75, 1.90) for w in (1, 2, 4, 8)]
    rng = np.random.default_rng(seed)
    test = [dict(friction_factor=float(rng.uniform(1, 8)), mass_kg=float(rng.uniform(1.6, 1.9)),
                 w1=float(np.exp(rng.uniform(0, np.log(8)))), split="test") for _ in range(n_test)]
    return train + test


def run_sweep(sim, cfg, L, M, points, verbose=True):
    """Fly SWEEP_ROUTE for every point (weights and mass from takeoff) and extract per-window means.
    Returns (rows, flights)."""
    rows, flights = [], {}
    for k, p in enumerate(points):
        w = np.array([p["w1"], 1, 1, 1, 1, 1], float)
        f = sim.run(SWEEP_ROUTE, L, M, friction=p["friction_factor"] * cfg.healthy_friction, weights=w,
                    mass_kg=p["mass_kg"])
        flights[k] = f
        a = f["dd2"]
        ok_end = a[-1, 39] if len(a) else 0.0
        for ph, (t0, t1) in SWEEP_WINDOWS.items():
            m = (a[:, 39] >= t0) & (a[:, 39] <= t1) if len(a) else np.zeros(0, bool)
            if not len(a) or ok_end < t1 or not m.any():
                continue
            rows.append(dict(run=k, **p, phase=ph, v_batt=a[m, 37].mean(), current=a[m, 36].mean(),
                             current_m1=a[m, 30].mean(), pwm_m1=a[m, 24].mean(), soc=a[m, 38].mean(),
                             completed=not f["error"] and not f["stop_why"]))
        if verbose:
            print(f"{k + 1:3d}/{len(points)} friction {p['friction_factor']:.2f}× mass {p['mass_kg']:.2f} kg "
                  f"w1 {p['w1']:.2f}: {'ok' if not f['error'] and not f['stop_why'] else 'STOPPED ' + (f['stop_why'] or f['error'])[:60]}"
                  f"{' (cached)' if f['from_cache'] else ''}", flush=True)
    return rows, flights


# ---- surrogate ----------------------------------------------------------------------------------------------------------
FEATURES = ["friction_factor", "mass_kg", "log_w1", "v_batt", "phase_climb", "phase_descent", "phase_cruise"]


def features(df):
    X = df.assign(log_w1=np.log(df["w1"]), phase_climb=(df["phase"] == "climb").astype(float),
                  phase_descent=(df["phase"].isin(["descent", "landing"])).astype(float),
                  phase_cruise=(df["phase"] == "cruise").astype(float))
    return X[FEATURES].to_numpy(float)


def fit_surrogates(train, seed=0):
    """Gradient boosting (the paper uses LightGBM for power; sklearn's HistGradientBoosting is the same family) and a
    Gaussian process (gives an uncertainty). Both predict the pack current from FEATURES."""
    from sklearn.ensemble import HistGradientBoostingRegressor
    from sklearn.gaussian_process import GaussianProcessRegressor
    from sklearn.gaussian_process.kernels import RBF, ConstantKernel, WhiteKernel
    from sklearn.pipeline import make_pipeline
    from sklearn.preprocessing import StandardScaler
    X, y = features(train), train["current"].to_numpy(float)
    gbm = HistGradientBoostingRegressor(max_iter=600, learning_rate=0.05, max_leaf_nodes=15, min_samples_leaf=3,
                                        random_state=seed).fit(X, y)
    kernel = ConstantKernel(10.0) * RBF(length_scale=np.ones(len(FEATURES))) + WhiteKernel(0.01)
    gp = make_pipeline(StandardScaler(), GaussianProcessRegressor(kernel=kernel, normalize_y=True,
                                                                  n_restarts_optimizer=3, random_state=seed)).fit(X, y)
    return {"Gradient boosting": gbm, "Gaussian process": gp}


def whitebox_predict(df, cfg):
    """White-box steady-state current for each row (friction on M1 only, other motors healthy)."""
    out = []
    for _, r in df.iterrows():
        fr = np.full(6, cfg.healthy_friction); fr[0] = r["friction_factor"] * cfg.healthy_friction
        s = steady_state(r["mass_kg"], fr, [r["w1"], 1, 1, 1, 1, 1], r["v_batt"], None, *SWEEP_SPEEDS[r["phase"]])
        out.append(s["current"] if s["converged"] and s["residual"] < 1e-3 else np.nan)
    return np.array(out)


def error_metrics(y, yhat):
    e = np.asarray(yhat) - np.asarray(y); ok = np.isfinite(e)
    return {"N": int(ok.sum()), "RMSE (A)": float(np.sqrt(np.mean(e[ok] ** 2))), "MAE (A)": float(np.mean(np.abs(e[ok]))),
            "Max |err| (A)": float(np.abs(e[ok]).max()), "Bias (A)": float(e[ok].mean()),
            "MAPE (%)": float(100 * np.mean(np.abs(e[ok] / np.asarray(y)[ok])))}


# ---- hybrid: white-box where it is valid, surrogate where a motor saturates ------------------------------------------
def save_surrogate(model, path):
    import joblib
    joblib.dump(model, path)


def load_surrogate(path):
    import joblib
    return joblib.load(path)


def latest_surrogate(root=None):
    """Gaussian-process surrogate saved by the most recent load_model_check run (None if there is none)."""
    from pathlib import Path
    root = Path(root) if root else Path(__file__).resolve().parent.parent / "results" / "load_model"
    found = sorted(root.glob("*/surrogate_gp.joblib"), key=lambda p: p.stat().st_mtime)
    return (load_surrogate(found[-1]), found[-1]) if found else (None, None)


def hybrid_current(mass_kg, frictions, weights, v_batt, healthy_friction, surrogate=None, u0=None, phase="hover",
                   v_h=0.0, v_z=0.0):
    """White-box steady state if no motor saturates; otherwise the surrogate (if given). Returns (current, source, u)."""
    import pandas as pd
    s = steady_state(mass_kg, frictions, weights, v_batt, u0, v_h, v_z)
    ok = s["converged"] and s["residual"] < 1e-3
    if ok and not s["saturated"]:
        return s["current"], "white-box", s["u"]
    if surrogate is None:
        return (s["current"] if ok else np.nan), "white-box (saturated)", s["u"] if ok else None
    row = pd.DataFrame([dict(friction_factor=float(np.asarray(frictions)[0]) / healthy_friction, mass_kg=mass_kg,
                             w1=float(weights[0]), v_batt=v_batt, phase=phase)])
    return float(surrogate.predict(features(row))[0]), "surrogate", s["u"] if ok else None
