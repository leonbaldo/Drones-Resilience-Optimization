"""Loaders for the starter-kit data. Pure numpy / pandas; no MATLAB, no torch."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "data"

PHASES = ("ground", "climb", "hover", "cruise", "turn", "descent")      # values of the `phase` signal
OMEGA_REF = 455.0           # rad/s, healthy hover rotor speed at 1.9 kg; reference speed of the wear law [D4]
FRICTION_HEALTHY = 2e-4     # N*m*s/rad, healthy viscous friction of every motor [D1]
LQR_FACTOR_NAMES = ("Q_pos_xy", "Q_vel_xy", "Q_z", "Q_vz", "Q_rollpitch", "Q_rollpitch_rate", "Q_yaw",
                    "Q_yaw_rate", "R_thrust", "R_moments")             # multiplicative factors on the baseline C1
SAT_NAMES = ("S0 clip", "S1 priority", "S2 redistribute")
# columns of the health traces (data/health_traces)
OM_COLS = ["speed_cmd", "altitude_ref", "phase_climb", "phase_hover", "phase_cruise", "phase_turn", "phase_descent",
           "ref_vx", "ref_vy", "ref_vz", "ref_ax", "ref_ay", "ref_az"]
C_COLS = (["mass"] + [f"log_lqr{k}" for k in range(10)] + ["share14", "share25", "share36"]
          + [f"log_wm{k + 1}" for k in range(6)] + ["sat_S0", "sat_S1", "sat_S2", "om1max_rel"])

_JSON_COLS = ("speeds", "cuts", "weights", "lqr_factors", "alloc_shares", "alloc_wm", "cf_weights_b")


def _parse(v):
    if isinstance(v, str) and v[:1] == "[":
        return json.loads(v)
    return v


def flight_index() -> pd.DataFrame:
    """One row per flight in data/flights: subset, source dataset, design factors, grading and diagnostics."""
    return pd.read_csv(DATA / "flights" / "index.csv")


def all_flights(dataset: str) -> pd.DataFrame:
    """Metadata + outcome of EVERY flight of a full dataset (no time series): dbdev_v2, dbdev_v3, evalfault_v2,
    evalfault_cf. Useful for statistics (e.g. PASS rate vs friction, mass, wind, allocation)."""
    return pd.read_csv(DATA / "manifests" / f"{dataset}_all_flights.csv")


def flight_path(flight_id: str) -> Path:
    hits = list((DATA / "flights").glob(f"*/{flight_id}.npz"))
    if not hits:
        raise FileNotFoundError(f"{flight_id} is not in the starter-kit subset (see flight_index())")
    return hits[0]


def load_flight(flight_id: str) -> dict:
    """50 Hz signals of one flight (dict of numpy arrays) plus 'meta' (the flight's JSON: design + outcome).
    The friction truth is m1_friction_truth [N*m*s/rad]; divide by FRICTION_HEALTHY for the factor d(t)."""
    p = flight_path(flight_id)
    with np.load(p) as z:
        out = {k: z[k] for k in z.files}
    meta = json.loads(p.with_suffix(".json").read_text())
    out["meta"] = {k: _parse(v) if k in _JSON_COLS else v for k, v in meta.items()}
    out["d"] = out["m1_friction_truth"] / FRICTION_HEALTHY
    return out


def load_map():
    """Occupancy grid of Zurich-Enge (1 = building footprint), 2 m cells, plus A, B and landing sites.
    grid[i, j]: row i counts from the TOP (north) edge, column j from the west edge, like an image;
    use extent=(0, 1200, 0, 950) and origin='upper' in imshow to get map metres (x east, y north)."""
    from scipy.io import loadmat
    m = loadmat(DATA / "map" / "zurich_gablerstrasse.mat")
    grid = m["complexMap"].astype(np.uint8)
    sites = json.loads((DATA / "map" / "sites.json").read_text())
    return dict(grid=grid, cell_m=2.0, width_m=1200.0, height_m=950.0, sites=sites)


def is_free(grid, x, y, cell=2.0):
    """True if map point (x, y) [m] is outside every building footprint."""
    H = grid.shape[0]
    i = np.clip(H - 1 - (np.asarray(y) / cell).astype(int), 0, H - 1)
    j = np.clip((np.asarray(x) / cell).astype(int), 0, grid.shape[1] - 1)
    return grid[i, j] == 0


def load_routes() -> dict:
    """Route library A (330, 150) -> B (740, 860): {route_id: {'xy': (N, 2) path in map metres, 'length_m', 'split'}}.
    'split' = train / test (held-out routes used by the test families) / benchmark (shortest route, never trained on)."""
    summ = pd.read_csv(DATA / "routes" / "routes_summary.csv").set_index("route_id")
    xy = pd.read_csv(DATA / "routes" / "routes_xy.csv")
    out = {}
    for rid, g in xy.groupby("route_id"):
        out[int(rid)] = dict(xy=g[["x_m", "y_m"]].to_numpy(), length_m=float(summ.loc[rid, "length_m"]),
                             split=summ.loc[rid, "split"])
    return out


def load_health_traces(dataset: str) -> dict:
    """1-s binned health traces of the subset flights (dbdev_v3, evalfault_v2, evalfault_cf). Keys:
    flight_id, t [s], d (true M1 friction factor, NEVER an input), u / ucmd ((N, 6) actual / commanded (w/w_ref)^2:
    the motor 'load' that drives wear), Om (N, 13) operating point (OM_COLS), phase,
    z_R3 (N, 1) M1 health, z_D0 / z_E1 (N, 6) per-motor health, *_n = operating-point-normalised versions,
    C (flights, 24) configuration (C_COLS) with C_flight_ids."""
    with np.load(DATA / "health_traces" / f"z_{dataset}.npz", allow_pickle=False) as z:
        return {k: z[k] for k in z.files}


def trace_of(tr: dict, flight_id: str) -> dict:
    """The rows of one flight from load_health_traces()."""
    m = tr["flight_id"] == flight_id
    return {k: v[m] for k, v in tr.items() if v.shape[:1] == tr["flight_id"].shape}


A_XY = np.array([330.0, 150.0])      # start / home in map metres; flight positions are relative to A


def map_xy(flight: dict) -> np.ndarray:
    """(N, 2) horizontal position of a flight in map metres (position_m is relative to A, x east, y north)."""
    return flight["position_m"][:, :2] + A_XY
