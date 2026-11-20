"""Pilot optimisation helpers: white-box steady-state trim, a static allocation scan, and a quasi-steady mission
rollout (wear law + battery) for ranking candidate actions at a decision time.

Everything here is a deliberately simple baseline built from the project's white-box physics (hexakit.whitebox):
- steady phases only (no turns, gusts, transients, controller dynamics); the trim is the closed-loop equilibrium;
- mean wind known (forecast), turbulence ignored;
- 'loss of authority' = the trim needs a command outside [200, 840] rad/s or has no equilibrium.
It is meant as a yardstick for a better formulation, not as the project's predictor.
"""
from __future__ import annotations

import numpy as np
import pandas as pd

from .whitebox import load_model as lm, battery as bat
from .whitebox.allocation import AllocConfig, hover_shares_feasible
from .whitebox.degradation import FrictionWear

B_HEALTHY = 2e-4
OMEGA_REF = 455.0
W_MIN, W_MAX = 200.0, 840.0


# ---- trim -------------------------------------------------------------------------------------------------------------
def trim(mass_kg, d, alloc=None, v_batt=15.8, v_h=0.0, v_z=0.0, u0=None):
    """Closed-loop steady state for friction factors d (scalar = M1 only, or 6 values), allocation, pack voltage,
    horizontal airspeed v_h and vertical speed v_z. Adds the M1 wear-rate factor d1·(ω1/ω_ref)² and the headroom."""
    d = np.asarray(d, float)
    dv = np.r_[d, np.ones(5)] if d.ndim == 0 else d
    alloc = AllocConfig() if alloc is None else alloc
    s = lm.steady_state(mass_kg, B_HEALTHY * dv, alloc, v_batt, u0=u0, v_h=v_h, v_z=v_z)
    ok = bool(s["converged"] and s["residual"] < 1e-3)
    s.update(ok=ok, d=dv,
             wear_rate_m1=float(dv[0] * (s["omega"][0] / OMEGA_REF) ** 2),        # ∝ db1/dt / (λ·b_healthy)
             headroom_hi=float(W_MAX - s["omega_ref"].max()),                    # rad/s below the 840 ceiling
             headroom_lo=float(s["omega_ref"].min() - W_MIN))                    # rad/s above the 200 floor
    return s


def simplex_grid(n=40, s_min=0.02):
    """Grid of pair shares (s14, s25, s36) on the simplex, step 1/n, every share ≥ s_min."""
    pts = [(i / n, j / n, 1 - (i + j) / n) for i in range(n + 1) for j in range(n + 1 - i)]
    return np.array([p for p in pts if min(p) >= s_min - 1e-12])


def static_allocation_scan(mass_kg, d1, v_batt=15.8, v_h=0.0, shares=None, sat=2, om1_max=840.0):
    """Pilot problem P1: trim over a grid of pair shares (mode-1 allocation, uniform moment weights).
    Returns one row per share vector: feasibility, M1 wear-rate factor, pack current, headroom, commands."""
    shares = simplex_grid() if shares is None else shares
    rows = []
    for s in shares:
        if not hover_shares_feasible(s, mass_kg):
            continue
        cfg = AllocConfig(mode=1, shares=np.asarray(s), sat=sat, om1_max=om1_max)
        t = trim(mass_kg, d1, cfg, v_batt, v_h)
        rows.append(dict(s14=s[0], s25=s[1], s36=s[2], ok=t["ok"], wear_rate_m1=t["wear_rate_m1"],
                         current_A=t["current"], headroom_hi=t["headroom_hi"], headroom_lo=t["headroom_lo"],
                         **{f"w_cmd{k + 1}": t["omega_ref"][k] for k in range(6)},
                         **{f"w{k + 1}": t["omega"][k] for k in range(6)}))
    return pd.DataFrame(rows)


# ---- mission rollout --------------------------------------------------------------------------------------------------
def wind_vector(speed_ms, from_deg):
    """Mean wind (x east, y north) for a meteorological 'from' direction."""
    th = np.deg2rad(from_deg)
    return -speed_ms * np.array([np.sin(th), np.cos(th)])


def remaining_plan(f, t_d, dt=2.0):
    """The planned remainder of a flight after t_d, from its navigator reference (the plan is known at t_d):
    times, horizontal AIRSPEED (reference ground velocity minus the mean wind), vertical speed, phase.
    Ends at touchdown (first time the reference is back on the ground)."""
    t, ref, ph = f["t"], f["reference_m"].astype(float), f["phase"]
    v = np.gradient(ref, t, axis=0)
    w = wind_vector(f["meta"]["wind_speed_ms"], f["meta"]["wind_from_deg"])
    after = t >= t_d
    landed = np.where(after & (ph == 0) & (t > t_d + 1))[0]
    t_end = t[landed[0]] if len(landed) else t[-1]
    grid = np.arange(t_d, t_end, dt)
    idx = np.searchsorted(t, grid)
    vh = np.linalg.norm(v[idx, :2] - w * (ref[idx, 2:3] > 10.8), axis=1)        # no wind below 10.8 m [W2]
    return pd.DataFrame(dict(t=grid, v_h=vh, v_z=v[idx, 2], phase=ph[idx], alt=ref[idx, 2]))


def lambda_oracle(meta):
    """True fault wear rate λ_f [1/s] of a growing-fault flight (from its design)."""
    return float(np.log(2) / meta["fault_doubling_s"])


def lambda_from_history(f, t_d, window_s=60.0, n=2.0):
    """λ estimated from the friction history before t_d: slope of log d against the M1 exposure ∫(ω1/ω_ref)^n.
    Uses the TRUE friction d(t) (an oracle on the health state, not on λ). With the encoder, replace d by a z → d
    map, or fit on z directly."""
    t, d, w1 = f["t"], f["d"], f["rotor_speed_rads"][:, 0].astype(float)
    m = (t >= t_d - window_s) & (t <= t_d)
    E = FrictionWear(n=n).exposure(t[m], w1[m])
    A = np.c_[E, np.ones_like(E)]
    return float(np.linalg.lstsq(A, np.log(d[m]), rcond=None)[0][0])


def rollout(plan, mass_kg, d1_0, lam, alloc=None, soc0=1.0, i0=15.0, n=2.0, d_others=None):
    """Quasi-steady forward integration over a plan (remaining_plan): at each step the trim gives M1's actual speed
    and the pack current; friction grows by the wear law (exact step), the battery by its 1-RC model, and the pack
    voltage feeds the next trim. Health → load → wear feedback is therefore included.
    Returns a DataFrame per step: d1, soc, v_batt, current, M1 command, headroom, ok (equilibrium inside limits)."""
    alloc = AllocConfig() if alloc is None else alloc
    others = np.ones(5) if d_others is None else np.asarray(d_others, float)
    wear = FrictionWear(b0=B_HEALTHY, lam=lam, n=n)
    b1, soc, vd = B_HEALTHY * d1_0, soc0, bat.RD * i0
    rows, u0 = [], None
    dts = np.diff(plan["t"].to_numpy(), append=plan["t"].iloc[-1] + (plan["t"].diff().median() or 2.0))
    for (_, p), dt in zip(plan.iterrows(), dts):
        v = float(bat.terminal_voltage(soc, vd, i0))
        tr = trim(mass_kg, np.r_[b1 / B_HEALTHY, others], alloc, v, p.v_h, p.v_z, u0=u0)
        u0 = tr["u"] if tr["ok"] else None
        i0 = tr["current"]
        rows.append(dict(t=p.t, d1=b1 / B_HEALTHY, soc=soc, v_batt=v, current_A=i0, w_cmd1=tr["omega_ref"][0],
                         w1=tr["omega"][0], headroom_hi=tr["headroom_hi"], headroom_lo=tr["headroom_lo"],
                         ok=tr["ok"] and tr["headroom_hi"] > 1e-6 and tr["headroom_lo"] > 1e-6))
        b1 = wear.step(b1, tr["omega"][0], dt)
        soc, vd = bat.step(soc, vd, i0, dt)
    return pd.DataFrame(rows)


def summarize(r):
    """End-of-mission figures of a rollout."""
    bad = r[~r.ok]
    return dict(d1_end=float(r.d1.iloc[-1]), soc_end=float(r.soc.iloc[-1]),
                min_headroom_hi=float(r.headroom_hi.min()), min_headroom_lo=float(r.headroom_lo.min()),
                t_first_limit=float(bad.t.iloc[0]) if len(bad) else np.nan, time_at_limit_s=float(len(bad) * r.t.diff().median()))


CF_ACTIONS = {                       # the four branches of evalfault_cf (FC3)
    "none": dict(alloc=AllocConfig(), drop_kg=0.0),
    "payload": dict(alloc=AllocConfig(), drop_kg=0.15),
    "alloc": dict(alloc=AllocConfig.legacy([4, 1, 1, 1, 1, 1]), drop_kg=0.0),
    "alloc_payload": dict(alloc=AllocConfig.legacy([4, 1, 1, 1, 1, 1]), drop_kg=0.15),
}


def state_at(f, t_d, avg_s=30.0):
    """What is known at t_d from the flight log: mass, M1 friction factor (truth; replace by the encoder's estimate),
    SoC (truth; a particle filter gives it within ~1-2 %), recent mean pack current."""
    t = f["t"]; k = np.searchsorted(t, t_d)
    m = (t > t_d - avg_s) & (t <= t_d)
    return dict(mass_kg=float(f["meta"]["mass_kg"]), d1=float(f["d"][k]), soc=float(f["soc"][k]),
                i_pack=float(f["pack_current_A"][m].mean()), pos=f["position_m"][k].astype(float))


def evaluate_actions(f_none, t_d, actions=None, lam=None, dt=2.0):
    """Pilot problem P2: predicted end-of-mission figures for each candidate action at t_d, from the shared history
    (any branch's log up to t_d) and the plan. lam defaults to the oracle λ of the flight design."""
    actions = CF_ACTIONS if actions is None else actions
    st = state_at(f_none, t_d)
    plan = remaining_plan(f_none, t_d, dt)
    lam = lambda_oracle(f_none["meta"]) if lam is None else lam
    out = {}
    for name, a in actions.items():
        r = rollout(plan, st["mass_kg"] - a["drop_kg"], st["d1"], lam, a["alloc"], st["soc"], st["i_pack"])
        out[name] = (summarize(r), r)
    return out


def divert_plan(state, site_xy, speed_ms, altitude_m, wind=(0.0, 0.0), dt=2.0, detour=1.0, hold_s=2.0,
                v_down=1.0):
    """A crude plan for a divert from the vehicle's position at t_d to a landing site: straight cruise (times a
    detour factor; the real planner goes around buildings), hold, vertical descent at v_down. Braking and the
    join leg are ignored. Returns a plan in the format of remaining_plan (t starts at 0)."""
    from .io import A_XY
    pos = state["pos"][:2] + A_XY
    dist = detour * float(np.linalg.norm(np.asarray(site_xy) - pos))
    heading = (np.asarray(site_xy) - pos) / max(dist, 1e-9)
    v_air = float(np.linalg.norm(speed_ms * heading - np.asarray(wind)))
    t_cruise, t_down = dist / speed_ms, altitude_m / v_down
    t = np.arange(0.0, t_cruise + hold_s + t_down, dt)
    ph = np.where(t < t_cruise, 3, np.where(t < t_cruise + hold_s, 2, 5))
    return pd.DataFrame(dict(t=t, v_h=np.where(ph == 3, v_air, 0.0), v_z=np.where(ph == 5, -v_down, 0.0), phase=ph,
                             alt=altitude_m))
