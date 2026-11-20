# Development dataset DB_dev: specification (step 0)

Status 2026-11-12: **specification for review. Nothing has been run, and the model is unchanged.**
ASSUMPTIONS.md section G holds every value in this document with its status.

## 1. Purpose and scope

DB_dev trains and tests the offline part of the paper architecture: the conditional autoencoder (encoder
E: X_s → z, decoder X̂_s = D_nom(Ω, C) + D_res(z, Ω, C)). It answers one question:

> Can the encoder estimate motor health from flight data, separately from how the motor is being used
> (operating conditions Ω) and from how the aircraft is configured (C)?

**Staging (user decision 2026-11-12).** In this dataset M1's friction is **constant within each flight** and varies
**between flights**. There is no wear growth, no fault onset and no forecaster F yet. Growing faults (the D10 wear
law) and the action-conditioned forecaster are a later dataset. The D11 benchmark (growing fault) is therefore
out of distribution for models trained here, and its results must be reported that way.

Each flight has **one configuration from takeoff to landing**. There are no mid-flight interventions (no controller,
allocation or payload switches, no diverts). The commanded speed may change during the flight: that is an operating
condition, not an intervention.

## 2. What is fixed

| Item | Value | Reason |
|---|---|---|
| Map and endpoints | **Zurich-Enge only** (R9): A (330, 150) → B (594, 464), B lands (Q8) | User decision; the variation comes from routes, not endpoints |
| Degrading motor | M1 only. M2–M6 at healthy friction (D1) | User decision |
| Friction in flight | Constant: wear law off (`sim.no_wear`) | Staging |
| Initial SoC | 100 % | User decision |
| Mid-flight actions | None | User decision |
| Model, solver | hexa_waypoints.slx, ode1, 1.25 ms (S1), navigator on (N1–N10) | Unchanged plant |
| LQR structure | Full-state tracking, no integral action (C1b) | Design choice |
| LQR design mass | 1.90 kg, the original value (M3), for every flight | User decision |

## 3. Factors (design of experiments)

| Group | Factor | Range / levels | Sampling |
|---|---|---|---|
| **Health d** | M1 friction multiplier d (b₁ = d · b_healthy) | 1–8× | Matched levels {1, 2, 4, 6, 8} and continuous log-uniform (§5) |
| **Mission r(t)** | Route | One of many Zurich-Enge A→B alternatives (§4) | Uniform over the training routes |
| | Cruise altitude | 15–40 m | Uniform |
| | Speed schedule | 1–4 segments over the cruise, each 1–8 m/s **within the airspeed budget** (§6), change points uniform along the path | Sobol, then the duration filter (§6) |
| **Configuration C** | Take-off mass | 1.6–2.2 kg | Uniform |
| | LQR Q (12 states) | 8 groups, each a log-uniform factor ×0.1–×10 on C1: horizontal position (x = y), horizontal velocity (vx = vy), z, vz, roll = pitch angle, roll = pitch rate, yaw, yaw rate | Sobol in log space |
| | LQR R (4 inputs) | 2 groups, log-uniform ×0.1–×10 on C1: thrust, moments | Sobol in log space |
| | Allocation weights w₁…w₆ | Log-uniform 0.25–8 per motor, normalised to geometric mean 1 (only ratios matter: S = diag(1/√w)) | Sobol in log space |
| | Anchor share | 20 % of scenarios use the baseline LQR (C1) and uniform allocation (A1) | Healthy-nominal anchor for D_nom |
| **Disturbance** (unmeasured) | Mean wind | 0–6 m/s **within the airspeed budget** (§6), direction 0–360° | Uniform in [0, U_max(altitude)], drawn before the speeds |
| | Turbulence | On (W3), seed per scenario | Shared within a matched family |

Wind is an **unmeasured disturbance**: it is varied and saved, but never given to the encoder or the decoder.

## 4. Route library: many A → B alternatives

The planner returns at most 5 alternatives and stops when no further distinct corridor exists. To obtain many:

1. Sample via points V inside the ellipse with foci A and B whose route-length bound is 1.6 × the shortest route.
2. Plan A → V → B with the existing planner (via points are already supported), keeping clearance R3, the turn radius
   R5 and the G2 path (N2, N3).
3. Reject routes longer than 1.6 × the shortest, and **near-duplicates**: discrete Fréchet distance < 15 m to a
   route already in the library.
4. Target 100 routes. Record each route's length, number of turns and minimum turn radius.
5. **Split by route.** About 15 % of the routes, chosen as geometrically distinct clusters, are test-only. The
   original study route (R9) is kept out of training (benchmark).

## 5. Pilot design (300 flights)

| Block | Design | Flights |
|---|---|---:|
| **Matched families** | 40 scenarios (Sobol over every factor except d) × 5 friction levels {1, 2, 4, 6, 8}. Within a family everything else is identical, including the wind seed | 200 |
| **Continuous** | 100 scenarios, Sobol over every factor **including** d (log-uniform 1–8) | 100 |
| **Total** | 40 healthy matched flights plus the continuous flights near 1× | **300** |

8 of the 40 matched scenarios are anchors (C1 + A1). Balance check: the assigned d is uncorrelated with route,
altitude, speed, mass, LQR, allocation and wind. Friction is **expected** to correlate with measured currents,
rotor speeds and tracking errors: that is the health signal.

## 6. Pre-flight filters (no simulation time spent on certain failures; user decision 2026-11-12)

| Filter | Rule | Record |
|---|---|---|
| Duration | Nominal duration from the **navigator preview** (perfect-tracking replay to touchdown) between 120 and 420 s. Each scenario owns 8 fixed candidate Sobol points and keeps the first in range, so the result does not depend on the worker count. A distance/speed estimate was tried first and was 20 % short (corner slow-downs, jerk-limited ramps, final stop) | Number of candidates tried per scenario |
| Take-off feasibility | A hover equilibrium exists with every rotor command below the 840 rad/s limit, at full battery (16.8 V), at the **highest friction of the family** (white-box `load_model.steady_state`). No extra headroom | Rejected allocations are logged and **replaced** with the next Sobol point of the weights |
| Controller, linear | Fastest closed-loop pole ≤ 3 × the baseline (C1) and minimum damping ≥ 0.3 | Rejected designs are logged and **replaced** with the next Sobol point of the LQR factors |
| Controller, Simulink | ~40 s screening flight (climb, hover, 20 m lateral move; healthy, 1.9 kg): no envelope stop, no divergence, cross-track ≤ 1 m. Each LQR design is screened once (~15 s) | As above; the pass/fail record is saved |
| **Airspeed budget** (re-pilot, G20) | top speed + mean wind + 2 σ_u (turbulence) ≤ **10 m/s**. Built into the sampling, not a rejection: the wind is drawn first in [0, U_max(altitude)] (3.9 m/s at 15 m, 5.6 m/s at 40 m, because turbulence is stronger low down) and the speeds in [1 m/s, what the wind leaves] | Wind and speeds saved per flight; the cap follows from wind and altitude (`dataset.airspeed_budget`) |

**Why no hover headroom.** A 10 % headroom (≤ 756 rad/s) at late-flight voltage rejects uniform allocation at 6× and above
(and 8× at almost every mass). The matched families fly the same configuration at every friction level, so only
M1-unloading allocations would survive: the allocation would become a proxy for "degraded". Flights that saturate
later as the voltage falls are kept and graded (F12, requirement violations).

**Why a Simulink screen.** An LQR is always stable on its own linear design model. Instability comes from what the
design ignores: the motor dynamics (time constant ≈ 0.08 s), PWM and command saturation, the solver step.

**Why an airspeed budget (pilot 1, 2026-11-13).** 15 of 40 healthy matched flights were unsafe. Every healthy
divergence happened at 11–14 m/s actual airspeed (mean wind plus gusts on top of the commanded speed), where the
vehicle tilts 35–45° and the commands reach 840 rad/s. Within the 10 m/s budget, 18 of 20 healthy pilot flights
passed with no divergence. Two alternatives did not separate the failures: a steady "all commands < 840 rad/s"
check at the worst airspeed (still feasible up to ~13 m/s), and a tougher LQR screening flight (8 m/s with a 6 m/s
side gust), which all 11 designs tried passed, including both soft designs that diverged in the pilot.

**F3 with the drag tilt (G19).** Pilot 1 also showed that F3's reference attitude (navigator acceleration / g)
ignores the tilt needed to hold airspeed against drag, so healthy fast flights failed F3 while tracking within
1.5 m. For dataset flights the reference now includes the steady drag tilt at the true air velocity.

## 7. Requirements per flight (grading)

| Item | Rule |
|---|---|
| Deadline | **1.2 × the nominal healthy duration** of that route and speed schedule (navigator preview). Replaces the fixed Q1 for this dataset |
| SoC reserve | 70 % at touchdown (Q4) |
| Corridor | 1 m (Q5) |
| Safety | F1–F11 (attitude, rates, altitude, landing). Saturation is a diagnostic only (F12) |
| Outcome | Completed with requirements met; completed with requirement violations; stopped by an envelope rule; numerically unresolved |

## 8. What each flight records

Signals are exported at **50 Hz** with anti-aliasing (block mean over each 20 ms). Metrics that depend on peaks
(corridor, actuator extremes, event times) are computed on the **full-rate** signals before the export.

| Group | Signals | Used as |
|---|---|---|
| **X_s** (observables) | 6 motor currents; 6 commanded rotor speeds; 6 PWM duties; **6 actual rotor speeds** (new log); pack voltage and current; attitude and rates; **body accelerations** (new log); position and velocity; tracking error to the reference | Encoder input. A per-flight flag marks the actual rotor speeds as an "onboard-available" signal (ESC eRPM telemetry), so models can be trained with or without them |
| **Ω(t)** (operating) | Commanded speed, mission phase (climb, hover, cruise, turn, descent, landed), altitude reference | Conditioning |
| **C, r(t)** (configuration) | Q and R diagonals, L and M gains, allocation weights, take-off mass; route ID and reference trajectory | Conditioning |
| **Ground truth** (never an input) | d and the applied b₁(t) (must be constant), true wind series, seeds | Validation and analysis only |
| **Outcomes** | Max cross-track and 3D deviation, arrival time, distance to B at the deadline, final SoC, energy, time at actuator limits per motor, safety checks, termination reason | Labels for the mission questions |
| **Metadata** | flight_id, family_id, route ID, split, all factor values, filter results, model and file hashes, cache version | Bookkeeping |

The raw full-rate logs are kept for **5 %** of the flights (stratified by d) to check the export.

## 9. New logs (done 2026-11-12, G15)

The **six actual rotor speeds** (`BLDCMk/BLDC1/\omega1`) and the **12 state derivatives** (inertial accelerations
and angular accelerations) are logged at 100 Hz when a flight is run with `HexaSim.run(..., dataset_logs=True)`.
Logging is switched on at session start, so the .slx file is **not** edited, and the cache keys of all other flights are
unchanged. Checks: existing signals bit-identical with and without the logs; logged speeds equal the speeds rebuilt
from PWM, current and voltage; accelerations match the numerical derivative of the logged velocity. The accelerometer
signal (body specific force) is computed at export: f_b = R_IBᵀ (a_I + [0, 0, g]).

## 10. Runner (option A: Python worker pool)

- Each worker owns one MATLAB engine (`HexaSim`) and private temporary paths. The settings for each flight are
  reset before every flight.
- **Raw cache off** for dataset flights, except for the 5 % subset. The 50 Hz export is written as soon as the flight ends.
- **Resumable**: a flight ID is skipped only if its export exists and matches the manifest (hash).
- **Disk preflight**: start only if free space is ≥ 3 × the estimated total plus 10 GB.
- **Before the pilot:** (a) serial and parallel runs agree for a healthy flight, a degraded flight and an anchor flight;
  (b) a healthy flight run right after a degraded one on the same worker matches a fresh run, to show no settings
  carry over; (c) a benchmark with 1, 4, 6 and 8 workers sets the worker count.

## 11. Splits

- **Test (unseen routes):** every flight on the held-out routes (§4).
- **Train / validation:** the remaining flights, split **by family**. A matched family never spans two splits.
- **Friction interpolation:** a separate training run that withholds the 4× level. Reported separately from the
  unseen-route results.
- **Benchmark:** D11 on the original Zurich-Enge route, never used in training, reported as out of distribution.

## 12. Pilot checks (go / no-go before scaling to 1,000–5,000)

- In every flight the applied b₁ is constant and equal to d. The configuration is constant.
- Within a family, flights at different d have the same reference, the same wind and the same settings.
- Metrics from the export match the full-rate metrics on the raw subset.
- Completion and failure rates per friction level, mass, LQR and allocation region. If one region is mostly
  failures, the dataset cannot support it.
- Coverage: each factor's marginal, and the balance of d against every other factor.
- Disk per flight and wall time per flight (measured), which give the cost of the full dataset.
- Sanity: the physics proxy b̂ (`load_model.estimate_friction`) tracks d on the healthy-to-8× matched families.

## 13. Inspection notebook (`dataset_inspection.ipynb`)

1. Manifest summary: flights per route, split and outcome.
2. Coverage: a histogram of each factor; pairwise plots of the continuous factors; d against each factor.
3. The route library: every route drawn on the Zurich-Enge map, held-out routes highlighted; length and turn statistics.
4. Outcomes: completion, requirement violations and safety failures by d, mass, speed, altitude, LQR and allocation
   region.
5. Time series browser: choose a flight or filter by any factor. Plots of position, tracking error, motor currents,
   commanded and actual rotor speeds, PWM, pack V and I, accelerations.
6. Matched families: one scenario at 1, 2, 4, 6, 8×, overlaid (M1 current, M1 speed, tracking error, SoC).
7. Configuration effects: the same d across LQR and allocation settings (anchor vs sampled).
8. Export checks: 50 Hz vs full-rate on the raw subset.

## 14. Still to check during implementation

- Whether 100 distinct routes exist under the 1.6 × length bound and the 15 m Fréchet threshold. Lower the
  target rather than relax the clearance.
- Whether the duration filter rejects too many low-speed schedules on long routes.
- Wall time and disk per flight at 50 Hz with the new logs.

## 15. Allocation factors (APPROVED 2026-11-13; implementation in progress)

Goal: the configuration C should span **every allocation the airframe allows**, not only one family of weighted
pseudo-inverses. This section replaces the six allocation weights of §3 (G8) for the full run.

### 15.1 What the allocator does today

`hexa_waypoints/Signal control to angular velocities without fault` (MATLAB Function):
ω² = S · pinv(F·S) · u, S = diag(1/√w), u = [T, Mx, My, Mz] from the LQR, F the nominal 4×6 force matrix. Then
each motor's command is clipped to [200², 840²] (rad/s)² on its own. **One weight vector decides both who carries the
thrust and who produces the correcting moments, and saturation is handled by plain clipping.**

### 15.2 What the airframe allows (computed from F, `load_model.FORCES`)

- F has rank 4, so the exact solutions of F·ω² = u form a **2-D family** (null space of F).
- **Every null-space direction changes opposite motors identically** (M1 = M4, M2 = M5, M3 = M6). So in steady
  flight with all moments balanced, **no allocator of any kind can load M1 differently from M4**. This is a property
  of the hexacopter geometry, not of the pseudo-inverse.
- Steady thrust distributions are therefore described by **three pair shares** s₁₄ + s₂₅ + s₃₆ = 1, i.e. two numbers.
  At hover (1.9 kg, all limits respected) a pair can go from 6.4 % to 87.2 % of the total ω² (one motor 3.2 % to 43.6 %).
  The pilot's six weights (0.25–8) reach M1 5–30 % only, with redundant dimensions.
- **Giving up yaw** is the only way to unload M1 alone. Keeping M4 at 16.7 % and M1 at 10 % leaves at least
  0.041 N·m of yaw torque; the plant has no yaw damping (ψ̈ = Mz / Izz, Izz = 0.0432 kg·m²), so the vehicle would
  spin up by ~1 rad/s every second (> 500 °/s after 10 s). **Not included** (§15.6).

### 15.3 Proposed factors (configuration C)

| Factor | Meaning | Range / levels | Sampling |
|---|---|---|---|
| **Thrust pair shares** s₁₄, s₂₅, s₃₆ | Share of the collective thrust per opposite pair (each motor of a pair: half). Moment-free by construction | The whole feasible simplex at hover for the flight's mass, every motor within [200², 840²] | Uniform on the simplex (Dirichlet(1,1,1)), outside-feasible points replaced; then the take-off filter (§6) as today |
| **Moment weights** w_M,1…6 | Who produces the correcting roll/pitch/yaw moments: weighted pseudo-inverse of the moment columns only, S_M = diag(1/√w_M). Larger = that motor does less correction work | Log-uniform 0.25–8, geometric mean 1 (as G8) | Sobol in log space |
| **Saturation strategy** | What happens when a command leaves [200², 840²] | **S0 clip** (today); **S1 priority**: keep roll/pitch, give up yaw first, then shift thrust, scale roll/pitch only as a last resort (in the spirit of ArduPilot's motor mixer, not a reimplementation of it); **S2 redistribution**: redistributed pseudo-inverse: saturated motors are fixed at their limit and the remaining wrench is re-allocated to the free motors, roll/pitch before yaw | Categorical, one third each |
| **M1 derating** | The allocator treats M1's ceiling as ω₁,max < 840 rad/s, so S1/S2 move work away from M1 before it saturates (what a health-aware reconfiguration would do) | 70 % of families: 840 (none); 30 %: uniform 500–840 rad/s | Per family |

The allocator becomes ω² = B_T·T + B_M·[Mx, My, Mz]ᵀ, with B_T from the pair shares and B_M from the moment weights,
followed by the saturation strategy. The anchors (20 %) fly today's allocation exactly: uniform shares, uniform
moment weights, S0, no derating.

### 15.4 Implementation and verification (after approval)

1. New code in the same MATLAB Function block, with a mode switch: **mode 0 = today's code path, unchanged**. The new
   parameters enter like `alloc_weights` (workspace variables set per run); they are absent from the cache key in
   mode 0, so every existing flight keeps its key. R2022b-compatible: only pinv, min/max and loops (no quadprog).
2. **Bit-identical check**: the cluster reference flights in mode 0 vs the current model.
3. A Python mirror in `load_model` (used by the take-off filter and `steady_state`) with unit tests: F·ω² = u when
   nothing saturates; the thrust pair shares are realised exactly; S1 keeps roll/pitch and sheds yaw first on crafted
   saturating wrenches; S2 re-allocates to the free motors; Python and Simulink outputs agree on logged commands.
4. Short Simulink checks per strategy (healthy and 6×), then a small re-pilot before the full run.
5. Rebuild and re-verify the cluster bundle (`cluster/make_bundle.py`).

Estimate: about one day of work, most of it in steps 1–3.

### 15.4b Implementation status (2026-11-13)

- `resilience/allocation.py` (Python) and `resilience/allocation_block.m` (Simulink, installed in memory at session
  start by `HexaSim._enable_alloc_v2`; the .slx file is unchanged). Unit tests `tests/test_allocation.py`: 10/10.
- **Mode 0 is bit-identical** to the original block: both cluster reference flights (54 logged signals, incl. an
  allocation switch) are exactly equal with and without the new code.
- **Simulink = Python** in mode 1 for S0, S1, S2: max difference 7·10⁻¹⁶ of full scale, on 40 s flights at 6× friction
  with M1 capped at 450 rad/s (a motor at a limit 95–100 % of the time).
- In that harsh test **S1 lost the vehicle after ~10 s** (it keeps roll/pitch by giving up collective thrust, so an
  under-powered vehicle sinks), while S0 and S2 flew: a strategy property, to be checked on realistic configurations
  in the re-pilot.
- Sampling: the strategy is **fixed per family (family number mod 3)** and only the shares, moment weights and
  derating are redrawn until that strategy passes the take-off filter. Letting the filter choose among strategies
  gave 34/16/30 (S1 sheds yaw at a saturated hover, so it fails the zero-moment equilibrium more often); fixed per
  family it is 27/27/26. Consequence, reported as a property: S1 families have a narrower M1-M4 share range
  (0.13–0.59 vs 0.06–0.81).

### 15.5 What each flight records

The pair shares, the moment weights, the saturation strategy and the M1 ceiling (manifest = C); the diagnostics
already logged (time at a limit, wrench residual) show how often each strategy acts.

### 15.6 Not included, and why

- **Yaw-sacrificing steady flight** (unloading M1 alone): with no yaw damping in the plant the vehicle spins up
  without bound, and the LQR is not designed for a spinning frame.
- **Shifting the centre of gravity** (e.g. moving the payload): it would break the M1 = M4 symmetry, but the plant
  has no movable payload position.
- **Rotor tilting or airframe changes**: outside the vehicle.

### 15.7 Decisions (user, 2026-11-13)

1. Thrust pair shares over the **whole feasible region**: the extremes are legitimate allocations; the take-off
   filter still removes infeasible ones.
2. **M1 derating included**: the most direct reconfiguration lever for the paper.
3. Saturation strategies in **equal thirds**.

## 16. DB_dev v2 (2026-11-17; done 2026-11-18: all 1,164 flights flown, jobs 168104 + 168766; graded at 30 % SoC, G26; ASSUMPTIONS G25)

Final manifest: 1,164 flights, 344 families (train 300 on 111 routes, val 14, test 30 on 19 routes); family wind median 2.1 m/s (train, 29 % > 3 m/s), 1.9 m/s (test, 23 %); nominal duration 120–896 s, 176 h simulated. Smoke test (job 168092, 8 flights) passed; turbulence of the 3 flights of a train family differs (corr. 0.1–0.3).

**Why.** The CVAE work (ASSUMPTIONS Z14–Z24) showed:
- What limits generalisation to new families is family-specific information. Mean wind and the turbulence realisation
  are shared by the 6 flights of a family. The configuration is not the limit (Z19).
- An onboard wind estimate in W removes most of this (Z24: −22 % val error, learning continues to epoch ~7). It needs
  no new data: the signals are already logged.
- More families still help, but with diminishing returns: −11 % from 68 to 137 train families with wind, −5 % on PASS
  flights (Z24).
- The test split (4 families, 4 routes) is too small to support any claim.

So v2 is a moderate extension that mainly fixes the evaluation and the gust memorisation. It is not a large scale-up.

| # | Change | Train | Val / test |
|---|---|---|---|
| 1 | Flights per family | **3**: 1× (needed for the healthy anchor of z and D_nom) + one d log-uniform in [1.5, 3] + one in [3, 8] | **6** matched levels as now (ordering τ, twins) |
| 2 | Turbulence seed | **one per flight** (mean wind still shared in the family): gusts can no longer identify a family | shared in the family, as now |
| 3 | Test split | — | **~30 families on ≥ 8 held-out routes** (now 4 / 4) |
| 3b | Duration filter | **120–900 s** (DB_dev: 120–600 s). With 600 s the filter kept mostly calm, fast missions (manifest at 600 s: train wind median 1.6 m/s, 12 % > 3 m/s; test 1.0 m/s). Cost: longer flights (~+20–25 % CPU), end SoC down to ~33 % | same |
| 3c | Sobol layout | candidate-major: candidate c of scenario k is row c·n_scen + k. The DB_dev layout (k·K + c) put every first candidate into 1/K of the route list: at 900 s, 300 train families on 19 routes, now 111 (test 7 → 19); DB_dev v1 (K = 8) is affected less: 137 train families on 70 routes | same |
| 4 | Configuration factors | unchanged (Z17: most do not change X, but they are the study's levers; the model uses the minimal set, Z19) | unchanged |
| 5 | Existing 1,002 flights | kept as they are (137 train families with 6 levels) | current val kept; current 4 test families kept inside the new test split |

**Size (example):** 300 new train families × 3 = 900 flights, 30 test families × 6 = 180, 14 extra val families × 6 = 84.
That is ~1,160 flights, ~110 CPU-h (≈ 9 h at 12 workers). Train families go from 137 to 437 (×3.2) and val to 40.
Expected gain on the val error from Z24's slope: a few percent. The main gains are a usable test split and no gust
memorisation.

**Implementation (after approval):**
- The scenario generator (`resilience/dataset.py`) needs a flights-per-family setting and a per-flight turbulence seed.
  Today `wind_seed` is per scenario (line ~624).
- Splits: the new held-out routes have to be chosen from the route library before sampling.
- A short re-pilot (~30 flights) checks the new family structure before the full run.

**Decisions (2026-11-17):** run now (scheduled 19:00); 3 flights per train family; ~30 test families on the DB_dev
test routes. The duration filter gets 32 candidates per scenario (DB_dev: 8): with 8, almost every scenario drawn on
the long test routes (1.3–1.75 km) failed it and was redrawn on a train route, which is why DB_dev has only 4 test families.


## 17. DB_dev v3: in-flight increasing friction (2026-11-18; scheduled 18:30; ASSUMPTIONS G27)

**Why.** All DB_dev v1 and v2 flights keep the M1 friction constant during the flight. The temporal encoders (Z28)
need flights where health changes during the flight, both to learn it and to measure how quickly z reacts.

| Item | v3 (training) | evalfault_v2 (evaluation) |
|---|---|---|
| Missions | 333 new families on the train routes, sampled as v2 train | the 30 v2 test families (held-out routes), copied from their v2 twins |
| Flights per family | 3: 1× constant, fault from 1×, fault from b₀ ∈ [1.5, 3]× | 8: b₀ ∈ {1×, band 2} × onset {25 %, 50 %} × d_end {4, 8}× |
| Fault | D10 wear law from the onset (15–60 % of the flight); rate set so friction reaches d_end at landing, d_end log-uniform in [max(1.5, 1.25·b₀), 8]× | same rule |
| Turbulence | own seed per flight | the twin's (shared in the family) |
| Splits | train / val (~10 % of families) | eval_fault |
| Size | 999 flights | 240 flights |

The end friction is set in advance from the predicted mean M1 speed (fit on 80 healthy v2 flights), so the realised value
is ~±15 % off the target. Both are recorded (`fault_d_end_target`, and the truth `m1_friction_truth` in each export).
Fault flights have `friction_band = −1`. For the anchor loss, the healthy rows of a flight are those with d(t) = 1×
(per-row flag, user decision).


## 18. DB_dev v4 and v5: faults on several motors (2026-11-19; approved, flight scheduled 18:30; ASSUMPTIONS G28)

**Why.** Every training flight so far (v1–v3) degrades M1 only; M2–M6 faults exist only in the small evaluation sets
(evalmotor, evalmotor2, evalmulti, all at constant friction). The six-latent models (Z33–Z38) learned per-motor
health from M1 faults alone. v4 and v5 give training data with faults on any motor, several at once, growing in flight.

**User decisions (2026-11-19):** v4 = 2 or 3 motors, any of M1–M6, each with its own onset and end friction;
v5 = M1 degrading in flight + 1–2 other motors at constant friction 1.5–8×; one 48-core job, ~900 + ~600 flights.

| Item | v4 (several motors degrading) | v5 (M1 degrading + fixed others) |
|---|---|---|
| Missions | 300 new families on the train routes, sampled as v3 (v2 sampler, own turbulence seed per flight, new LQR designs from the same screened stream), ~10 % of families relabelled val | 180 new families, same rules |
| Flights per family | 3: (a) healthy anchor, all motors 1×, constant; (b) fault flight, every degrading motor starts at 1×; (c) fault flight, every degrading motor starts at its own b₀ ~ log-U[1.5, 3]× | 3: (a) healthy anchor; (b) M1 wear from 1×; (c) M1 wear from b₀ ~ log-U[1.5, 3]× |
| Which motors | per fault flight: k = 2 or 3 (50/50); the motor set is drawn from a seeded, shuffled round robin over all 15 pairs and 20 triples, so every pair and triple appears ~equally (incl. the opposite pair partners M1+M4, M2+M5, M3+M6) | per fault flight: 1 or 2 (50/50) fixed motors drawn uniformly from M2–M6 (round robin over the 5 singles and 10 pairs) |
| Fault law | per degrading motor, independently: D10 wear law from its own onset ~ U[15 %, 60 %] of the nominal duration; rate set so friction reaches its own d_end ~ log-U[max(1.5, 1.25·b₀), 8]× at landing (v3 rule, per motor) | M1: the v3 rule. Fixed motors: constant friction d_fix ~ log-U[1.5, 8]× for the whole flight (no growth) |
| Wear-rate prediction | `wear_speed_factor(mass, share of the motor's pair)`: s₁₄ for M1/M4, s₂₅ for M2/M5, s₃₆ for M3/M6 (the fit is M1's; reused for the other motors, approximation ~±15 % as v3) | as v3 |
| Evaluation sets (held-out routes) | `evalwear4`: 10 DB_dev v2 test families (evalmotor's) × 6 flights (k = 2 / 3 × motor sets fixed in advance, b₀ = 1×) = 60 | `evalwear5`: same 10 families × 6 flights (M1 wear from 1× + 1 or 2 fixed motors) = 60 |
| Size | 900 + 60 | 540 + 60 → **1,560 flights in all** |

**Labels / exports.** Manifest: `fault_motors` (JSON {motor: friction at take-off}, as evalmulti) and new
`fault_wear` (JSON {motor: {onset_s, doubling_s, d_end_target}}; a motor listed in `fault_motors` but not in
`fault_wear` keeps a constant friction). Export: new per-row `motor_friction_truth` (6 columns, raw viscous friction of M1..M6, same units as
`m1_friction_truth`, which is kept = its first column; divide by 2e-4 for the factor). Fault flights `friction_band = −1`; the
healthy rows of a flight are those where all six motors are at 1× (per-row flag, as v3).

**Simulator change (needed for v4 only).** M2–M6 have constant friction only (`hexa_waypoints_pm`, Z33). New
**copy** `hexa_waypoints_multi_degradation.slx` (`model_edit_multi_degradation.m`, built from `hexa_waypoints_pm.slx`): the 'M1 wear' subsystem of D10 is replicated on
BLDCM2..6 with per-motor parameters `wear_lam_fault_mk`, `wear_t_onset_s_mk` (defaults 0 / 1e6; shared
`wear_n`, `wear_omega_ref`; b₀ = `friction_mk`), and each motor's friction is logged (`mk_friction`).
`hexa_waypoints.slx` and `hexa_waypoints_pm.slx` stay untouched. HexaSim gets an opt-in `wear_motors` argument
(absent → cache keys and results unchanged). v5 needs no model change, but is flown with the same model so the
whole night is one job.

**Verification before the run (pre-registered, job on the cluster, ~30 min):**
1. multi-degradation model with no wear = pm, bit-identical, on one v2 test flight (pm = hexa_waypoints was shown in Z33).
2. multi-degradation model with M1 wear only = the same v3 flight flown on hexa_waypoints, bit-identical.
3. Wear on M3 (and on M2 + M5 at once): logged friction follows exp of the integrated law (≤ 1 % vs Python
   integration of the logged speed), the faulty motors' current rises, the others stay at 1×.
4. A 12-flight test of the v4 + v5 manifests (`TEST=12`) ends with "TEST PASSED".

**Expected outcomes / risks.** v3 fault flights FAIL 41 % (one motor up to 8×). With 2–3 motors up to 8× in v4,
FAIL will be clearly higher (guess 55–70 %), and some flights will stop early (shorter, less data per flight).
Kept as is (failures are data; motor saturation is a diagnostic only). If the 12-flight test shows almost no
PASS fault flights, the fallback is d_end max 6× for triples (user decision then).

**Timeline (2026-11-19).** After approval: code + model edit ~1.5 h; verification job ~30 min; manifest job (LQR
screen for ~480 more designs + duration previews; v3's took ~45 min for 333 families) ~1–1.5 h, may run in parallel
with the verification; flight job `sbatch --begin=18:30 --export=ALL,RUN=dbdev_v45,DBDEV_MODEL=hexa_waypoints_pmw`
on 48 cores, 150 GB (24 workers; ≥ 6 GB/worker, the v2 OOM was at 150 GB / 24 workers on long flights → 180 GB), ~13 h
→ done ~07:30–08:00 on 2026-11-20. Resumable; order: eval sets first, then v5, then v4.

**Implementation (2026-11-19).** Code: `model_edit_multi_degradation.m`; `resilience/sim.py` (`wear_motors`, `MW_LOGS`;
opt-in, other models unchanged); `resilience/dataset.py` (`V4Design`, `V5Design`, `sample_multi`, `multi_fault_columns`,
`wear_spec`, `row_fault_wear`, runner + export); `resilience/eval_faults.py` (`build_manifest_wear`, motor sets
`EVAL_WEAR4_SETS` = M1+M4, M2+M6, M3+M5, M1+M2+M3, M2+M4+M6, M1+M3+M5 and `EVAL_WEAR5_SETS` = M4, M2, M5, M2+M4,
M3+M6, M5+M6); generator `results/dataset/make_dbdev_v45.py` (stages screen / manifest; job
`cluster/run_dbdev_v45_manifest.sbatch`); verification `cluster/verify_multi_degradation.py` (+ .sbatch). Ids: v4
S874–S1173 / F3165–F4064, v5 S1174–S1353 / F4065–F4604, eval FW000–FW059 (v4), FY000–FY059 (v5). Run folders: the
whole manifest in `results/dataset/dbdev_v45/manifest.csv` (column `block` = v4 / v5 / evalwear4 / evalwear5), flown
as two halves `dbdev_v45a`, `dbdev_v45b` (whole families, balanced by flight time) by two 24-core jobs on the `gpu`
partition (the `cpu` queue estimated a start on 2026-11-23; `gpu_top` is at the 5-job user limit); afterwards the
flights are merged into `dbdev_v45/flights`. Dry run of the samplers (fake previews): v4 300 pairs + 300 triples, all
35 motor sets used 15–20 times; v5 164 flights with 1 fixed motor, 196 with 2; 384 new LQR designs.
Caveat (as Z33): the G23 allocation factors (pair shares, M1 ceiling) were designed around M1; they stay as sampled.

**Verification result (job 169947, 2026-11-19 15:55; `results/dataset/verify_multi/summary.json`).** Check 1
(no wear: pm = multi-degradation, 23/23 signals bit-identical) and check 2 (v3 M1-wear flight: default model =
multi-degradation, 23/23 bit-identical) **pass**. Check 3: the friction law **passes** on every wearing motor (end ratio
vs Python integration: M3 0.03 %, M1 0.01 %; FX103 M3 reached 3.79× for a 4× target); the script reported "law
False" only because of a bug in the check (constant motors are 0.99999997 in the float32 export, compared with an
exact 1e-9 tolerance; tolerance changed to relative 1e-6 after the run). FX125 (M2 + M5): the flight diverged (F1)
at 152 s, before M5's onset at 163 s, so M5 correctly never grew (M2 1.53×). The current criterion **failed on
FX514** (M1 wearing from 1×, M4 constant 3×): M1's mean current after the onset rose 8 %, less than the healthy M2
(14 %). Cause: G23 pair coupling. With its partner M4 at 3×, M1 at 1× draws 25 % less current than in the healthy
flight; as M1 wears its current rises monotonically (−25 %, −12 %, −1 %, +26 % vs healthy at 1.0, 1.27, 1.49,
2.0×). So the wear acts on M1; the pre-registered criterion is confounded by the partner and is reported as failed,
not redefined. All 5 fault verification flights FAIL (F1 divergence / F7 landing), including FX125 with only M2 at
≤ 1.5×; the healthy flight passes (family S496). Unit tests: `tests/test_multi_degradation.py` (5/5).


**Manifest (job 169960, 16:37) and 12-flight test (job 169980, 16:50).** Manifest: 1,560 flights (v4 900 on 300
families, 28 val; v5 540 on 180, 24 val; evalwear4/5 60 each), none dropped, 384 new LQR designs, 240 h nominal
flight time (halves a/b: 780 flights, ~120 h each). Test (`results/dataset/dbdev_test`, the 3 shortest fault flights
of each block, the most motors first): 12/12 flown, TEST PASSED (pipeline); every motor's logged friction matches its
manifest entry (constant motors exact, wearing motors grow). Outcomes: **2/12 MISSION PASS** (one v4 triple starting at
2.8/2.9/1.8×, one v5), 10 FAIL (F1 envelope/divergence + F7 landing), all 6 eval flights FAIL (one family, the
shortest evaluation family). Several failures happen far below 8× (FW053: M1 1.16×, M3 1.66×, M5 2.48× at the end), so
the pre-agreed fallback (d_end ≤ 6× for triples) would hardly change the FAIL share. The sample is small and
deliberately the hardest; v3 fault flights FAIL 41 %. Run kept as specified (failures are data); FAIL share to be read
on the full set.

**Severity change to "mild" (user decision 2026-11-19 ~16:55, after the test above: "i want milder flights"; option
"every motor ≤ 3×").** Evidence used: two motors at constant ~1.9× each PASS 55 %, ~4.3× + ~1.9× PASS 6 % (evalmulti,
99 flights so far); v3 single-motor wear PASS 85 % for d_end ≤ 2×, 68 % for 2–3×, 49 % for 6–8×. New values (G28):
v4 d_end log-U[max(1.2, 1.25·b₀), 3]×, b₀ log-U[1.2, 1.8]× (was [1.5, 8] and [1.5, 3]); v5 M1 by the same rule, fixed
motors log-U[1.2, 2.5]× (was [1.5, 8]); eval sets follow (they use the design defaults). Missions, routes, LQR
designs, wind, mass and ids are unchanged; the allocation factors of 72 % of rows changed, because the v2 sampler
draws the allocation feasible for the family's take-off friction (as in v3). The severe manifests are kept as
`manifest_severe8x.csv` in dbdev_v45, dbdev_v45a/b, dbdev_v45_test; the severe test flights in
`dbdev_test/flights_severe8x_169980`. Mild 12-flight test: job 170019.

Mild test (job 170019, 17:10): 12/12 flown, **9/12 MISSION PASS** (severe version of the same 12 flights: 2/12). The 3 FAIL: one F7 landing only (v5, M4 2.0× + M6 1.3× fixed); FW051 (M1+M2+M3 reaching 2.1–2.4×) and FW053 (M1+M3+M5, stopped with M1 at 1.2×, M3 1.8×, M5 2.3×), both triples on the same evaluation family. Night run unchanged otherwise (jobs 169981/169982, 18:30).
