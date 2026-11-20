"""Progressive motor wear: speed-driven growth of the viscous-friction coefficient.

For motor i with equivalent viscous friction b_i(t) [N·m·s/rad]:

    db_i/dt = λ_i · b_i(t) · (|ω_i(t)| / ω_ref)^n,          b_i(t0) = b_i0 > 0

    b_i(t)  = b_i0 · exp[ λ_i · E_i(t) ],   E_i(t) = ∫_{t0}^{t} (|ω_i(τ)| / ω_ref)^n dτ   ("exposure", in seconds)

n = 2 (default) reads the speed term as propeller load (thrust ∝ ω²). n = 7 follows bearing fatigue: damage rate
∝ speed × load³ (basic rating life, ISO 281, ball bearings), with the bearing load ∝ thrust ∝ ω².

λ_i [1/s] is the growth rate while the motor turns at ω_ref: at constant ω = ω_ref, b doubles every ln 2 / λ_i.
Because the rate scales with ω², a motor that turns faster wears faster, and E_i(t) is the equivalent time at
ω_ref. With ω_ref = the healthy hover speed, λ_i reads as "wear rate per second of healthy hover".

Use:
    wear = FrictionWear(b0=2e-4, lam=lam_for(6.0, 100 * 3600), omega_ref=OMEGA_HOVER)   # 6× after 100 h of hover
    b = wear.b(t, omega)                       # friction history for a logged rotor-speed history
    b_next = wear.step(b, omega, dt)           # exact one-step update (for a time-stepped simulation)
    out = hover_wear(wear_per_motor, ...)       # closed loop: friction ↔ rotor speeds through the hover equilibrium

The closed-loop hover model uses load_model.steady_state: as b_i rises, the controller still holds the thrust and
zero moments, so motor i draws more current and its speed changes, which changes its own wear rate. The run ends
when a motor command reaches the limit of the saturation block (with M1 worn: the opposite motor M4 is pushed down
to its 200 rad/s floor to keep the moments balanced), or optionally when no hover equilibrium exists at all.
Assumptions: ASSUMPTIONS.md section D (D3–D6).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import load_model as lm

OMEGA_HOVER = 455.0          # rad/s, healthy hover speed at 1.9 kg (load_model.steady_state, all motors healthy) [D4]


@dataclass(frozen=True)
class FrictionWear:
    """Wear law for one motor (or several, with array-valued b0 / lam)."""
    b0: float | np.ndarray = 2e-4          # initial friction [N·m·s/rad], possibly already degraded [D1]
    lam: float | np.ndarray = 0.0          # growth rate at ω_ref [1/s]; 0 = no wear
    omega_ref: float = OMEGA_HOVER         # reference speed [rad/s]
    n: float = 2.0                         # speed exponent (2 = propeller load, 7 = bearing fatigue) [D8]

    def __post_init__(self):
        if np.any(np.asarray(self.b0) <= 0) or np.any(np.asarray(self.lam) < 0) or self.omega_ref <= 0 or self.n < 0:
            raise ValueError("Need b0 > 0, lam >= 0, omega_ref > 0, n >= 0")

    def speed_term(self, omega):
        """(|ω| / ω_ref)^n"""
        return (np.abs(np.asarray(omega, float)) / self.omega_ref) ** self.n

    def rate(self, b, omega):
        """db/dt for friction b at speed omega."""
        return self.lam * b * self.speed_term(omega)

    def exposure(self, t, omega):
        """E(t) = ∫ (|ω|/ω_ref)^n dτ from t[0], trapezoidal. t: (N,), omega: (N,) or (N, motors). Returns same shape."""
        t = np.asarray(t, float); w2 = self.speed_term(omega)
        dt = np.diff(t).reshape((-1,) + (1,) * (w2.ndim - 1))
        inc = 0.5 * (w2[1:] + w2[:-1]) * dt
        return np.concatenate([np.zeros((1,) + w2.shape[1:]), np.cumsum(inc, axis=0)])

    def b(self, t, omega, b_start=None):
        """Friction history along a rotor-speed history (closed form). b_start overrides b0 (e.g. next flight)."""
        b_start = self.b0 if b_start is None else b_start
        return b_start * np.exp(self.lam * self.exposure(t, omega))

    def step(self, b, omega, dt):
        """Exact update over dt with ω held constant (no integration error for piecewise-constant ω)."""
        return b * np.exp(self.lam * self.speed_term(omega) * dt)

    def time_to(self, b_target, omega=None):
        """Time [s] to grow from b0 to b_target at a constant speed (default ω_ref). inf if lam = 0."""
        omega = self.omega_ref if omega is None else omega
        k = self.lam * self.speed_term(omega)
        with np.errstate(divide="ignore"):
            return np.log(np.asarray(b_target) / self.b0) / k


def lam_for(ratio, seconds_at_ref):
    """λ that multiplies the friction by `ratio` after `seconds_at_ref` seconds at ω = ω_ref."""
    if ratio <= 1 or seconds_at_ref <= 0:
        raise ValueError("Need ratio > 1 and seconds_at_ref > 0")
    return np.log(ratio) / seconds_at_ref


def omega_from_dd2(dd2):
    """Actual rotor speeds of the six motors rebuilt from the logged PWM, per-motor battery currents and pack
    voltage (same inversion as load_model.estimate_friction). Returns (t [s], ω [rad/s] of shape (N, 6))."""
    t = dd2[:, 39]
    pwm = dd2[:, 24:30]; ib = dd2[:, 30:36]; v = dd2[:, 37][:, None]
    with np.errstate(divide="ignore", invalid="ignore"):
        i_m = np.where(pwm > 0, ib / (lm.BATT_GAIN * pwm), 0.0)
    omega = np.clip((v * pwm - lm.R_MOTOR * i_m) / lm.KE, 0.0, None)
    return t, omega


def hover_wear(wears, mass_kg, v_batt, weights=(1, 1, 1, 1, 1, 1), t_end=np.inf, growth_per_step=0.01,
               max_steps=5000, end_of_life="saturation"):
    """Closed-loop wear at hover: friction ↔ rotor speeds through load_model.steady_state.

    wears: one FrictionWear per motor (6), with its own b0 and lam (lam = 0 for a motor that does not wear).
    Each step lets the fastest-growing friction rise by `growth_per_step` (exact update with the speeds held),
    then re-solves the equilibrium. Stops at t_end or at the end of life:
      "saturation"  (default) the first motor command reaches a limit of the saturation block [200, 840] rad/s.
                    With M1 worn this is the opposite motor M4 reaching its 200 rad/s floor: the controller can
                    still hover but has lost part of its roll/pitch authority. Matches the Simulink finding that
                    10× friction is unflyable at 2.2 kg (ASSUMPTIONS D2).
      "equilibrium" no hover equilibrium exists at all (or PWM > 1), allowing saturated commands: the hard limit.
    Returns dict of arrays: t [s], b (N, 6), omega (N, 6), pwm (N, 6), current [A], and `end` (reason)."""
    if len(wears) != 6:
        raise ValueError("Need one FrictionWear per motor (6)")
    b = np.array([float(w.b0) for w in wears])
    lam = np.array([float(w.lam) for w in wears])
    t, u0, rows, end, limits = 0.0, None, [], "t_end reached", []
    if end_of_life not in ("saturation", "equilibrium"):
        raise ValueError("end_of_life must be 'saturation' or 'equilibrium'")

    def ok(s):
        sat = s["saturated"] if end_of_life == "saturation" else False
        return s["converged"] and s["residual"] < 1e-3 and not sat and s["pwm"].max() <= 1

    for _ in range(max_steps):
        s = lm.steady_state(mass_kg, b, weights, v_batt, u0)
        if not ok(s) and u0 is not None:                      # the warm start can fail where a cold start works
            s = lm.steady_state(mass_kg, b, weights, v_batt)
        feasible = ok(s)
        if not feasible:
            if s["converged"] and s["residual"] < 1e-3:          # which motor commands sit at a limit
                w2 = s["omega_ref"] ** 2
                limits = [(i + 1, "200 rad/s floor") for i in np.flatnonzero(w2 <= lm.OMEGA2_MIN + 1e-6)] + \
                         [(i + 1, "840 rad/s maximum") for i in np.flatnonzero(w2 >= lm.OMEGA2_MAX - 1e-6)] + \
                         [(i + 1, "PWM > 1") for i in np.flatnonzero(s["pwm"] > 1)]
            elif rows:                                            # no equilibrium: motors at their maximum
                pwm_max = lm.PWM_A * np.sqrt(lm.OMEGA2_MAX) + lm.PWM_B
                limits = [(i + 1, "840 rad/s maximum (thrust limit)") for i in np.flatnonzero(rows[-1][3] >= 0.99 * pwm_max)]
            end = ("end of life: a motor command saturates" if end_of_life == "saturation"
                   else "end of life: no hover equilibrium")
            break
        rows.append((t, b.copy(), s["omega"].copy(), s["pwm"].copy(), s["current"]))
        u0 = s["u"]
        k = lam * np.array([w.speed_term(o) for w, o in zip(wears, s["omega"])])   # growth rate of ln b, 1/s
        if not np.any(k > 0) or t >= t_end:
            end = "no wear" if not np.any(k > 0) else end
            break
        dt = min(np.log1p(growth_per_step) / k.max(), t_end - t)
        b = b * np.exp(k * dt); t += dt
    else:
        end = "max_steps reached"
    if not rows:
        raise ValueError("The initial frictions already have no hover equilibrium")
    T, B, W, P, I = zip(*rows)
    return dict(t=np.array(T), b=np.array(B), omega=np.array(W), pwm=np.array(P), current=np.array(I), end=end,
                t_last=t, b_last=b,        # time and frictions at the step that ended the run
                limits=limits)             # [(motor 1-6, which limit)] at the end of life; [] if not identified


# ---- wear along a planned route ---------------------------------------------------------------------------------
def route_thrust(ref, t, mass_kg, smooth_s=2.0):
    """Thrust magnitude [N] needed to follow the reference ref ([t, x, y, z]) at times t, quasi-steady:
    |m·(a + g·ẑ) + drag(v)|, with velocity and acceleration from the reference, smoothed over `smooth_s`
    (the closed loop does not follow corners and step changes instantly). Drag as load_model.required_thrust
    (level body, form drag of arellano_hexa_payload.m). mass_kg: scalar or callable of t."""
    t = np.asarray(t, float)
    dt = np.median(np.diff(t))
    pos = np.column_stack([np.interp(t, ref[:, 0], ref[:, j]) for j in (1, 2, 3)])
    k = max(int(round(smooth_s / dt)), 1)
    kern = np.ones(k) / k
    sm = lambda x: np.column_stack([np.convolve(np.pad(x[:, j], (k // 2, k - 1 - k // 2), mode="edge"), kern, "valid")
                                    for j in range(x.shape[1])])
    v = sm(np.gradient(pos, t, axis=0))
    a = sm(np.gradient(v, t, axis=0))
    m = np.array([mass_kg(x) for x in t]) if callable(mass_kg) else np.full(len(t), float(mass_kg))
    vh = np.linalg.norm(v[:, :2], axis=1)
    drag_h = 0.5 * lm.RHO_AIR * lm.C_D * lm.A_YZ * vh[:, None] * v[:, :2]          # opposes motion → thrust adds it
    drag_z = 0.5 * lm.RHO_AIR * lm.C_D * lm.A_XY * np.abs(v[:, 2]) * v[:, 2]
    F = np.column_stack([m[:, None] * a[:, :2] + drag_h, m * (a[:, 2] + lm.G) + drag_z])
    return np.linalg.norm(F, axis=1)


def wear_along(ref, wears, mass_kg, v_batt, weights=(1, 1, 1, 1, 1, 1), t_from=None, t_to=None, dt=0.5,
               end_of_life="saturation", smooth_s=2.0):
    """Closed-loop wear along a reference route: at each step the thrust needed by the route (route_thrust)
    and the current frictions give the rotor speeds (load_model.steady_state, zero moments); the speeds then
    grow the frictions over dt (exact update). wears: one FrictionWear per motor (b0 = friction at t_from).
    weights: allocation weights, a 6-tuple or a callable of t (e.g. a switch at the decision).
    v_batt: pack voltage, a number or a callable of t.
    Returns dict: t, b (N, 6), omega (N, 6) actual speeds, cmd (N, 6) commanded speeds (saturation block limits
    200 and 840 rad/s), pwm (N, 6), current, thrust, t_eol (first time without a valid
    equilibrium under `end_of_life`, or None), exposure (N, 6) = ∫ (ω/ω_ref)^n dt per motor."""
    if len(wears) != 6:
        raise ValueError("Need one FrictionWear per motor (6)")
    t0 = ref[0, 0] if t_from is None else t_from
    t1 = ref[-1, 0] if t_to is None else t_to
    t = np.arange(t0, t1 + 1e-9, dt)
    thrust = route_thrust(ref, t, mass_kg, smooth_s)
    b = np.array([float(w.b0) for w in wears]); lam = np.array([float(w.lam) for w in wears])
    W, C, P, I, B, Ex, t_eol, u0 = [], [], [], [], [], [], None, None
    ex = np.zeros(6)
    for j, (tj, Tj) in enumerate(zip(t, thrust)):
        wj = weights(tj) if callable(weights) else weights
        vj = v_batt(tj) if callable(v_batt) else v_batt
        s = lm.steady_state(Tj / lm.G, b, wj, vj, u0)              # thrust given through an equivalent mass
        good = s["converged"] and s["residual"] < 1e-3 and s["pwm"].max() <= 1
        if not good and u0 is not None:
            s = lm.steady_state(Tj / lm.G, b, wj, vj); good = s["converged"] and s["residual"] < 1e-3 and s["pwm"].max() <= 1
        if (not good or (end_of_life == "saturation" and s["saturated"])) and t_eol is None:
            t_eol = tj
        u0 = s["u"] if good else None
        B.append(b.copy()); W.append(s["omega"]); C.append(s["omega_ref"]); P.append(s["pwm"]); I.append(s["current"])
        Ex.append(ex.copy())
        if j + 1 < len(t):
            h = t[j + 1] - tj
            term = np.array([w.speed_term(o) for w, o in zip(wears, s["omega"])])
            b = b * np.exp(lam * term * h); ex = ex + term * h
    return dict(t=t, b=np.array(B), omega=np.array(W), cmd=np.array(C), pwm=np.array(P), current=np.array(I),
                thrust=thrust, t_eol=t_eol, exposure=np.array(Ex))


def flight_friction(flight, wear, b0):
    """M1 friction of a simulated flight [D10]: Simulink's logged `m1_friction` and the same wear law integrated
    in Python on the M1 speed rebuilt from dd2 (an independent check of the Simulink wear block).
    `wear` = HexaSim/cfg.wear dict, b0 = friction at takeoff. Returns dict of arrays on the dd2 time grid."""
    t, om = omega_from_dd2(flight["dd2"])
    lam = np.where(t >= wear["onset_s"], wear["lam"] + wear["lam_fault"], wear["lam"])
    rate = lam * (om[:, 0] / wear["omega_ref"]) ** wear["n"]
    x = np.concatenate([[0.0], np.cumsum(0.5 * (rate[1:] + rate[:-1]) * np.diff(t))])
    b_sim = np.interp(t, flight["m1_friction_t"], flight["m1_friction"][:, 0])
    # growth at constant ω_ref after the onset (what the doubling time means), for reference
    b_ref = b0 * np.exp(np.clip(t - wear["onset_s"], 0, None) * wear["lam_fault"] + t * wear["lam"])
    return dict(t=t, omega=om, b_sim=b_sim, b_python=b0 * np.exp(x), b_at_ref=b_ref)
