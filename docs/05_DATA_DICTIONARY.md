# 5. Data dictionary

All loaders are in [hexakit/io.py](../hexakit/io.py). Units are SI unless stated. "Map metres" means the frame of
the Zurich-Enge map: origin at the lower-left corner, x east, y north.

## 5.1 `data/flights/<subset>/<flight_id>.npz`: time series (50 Hz)

Exported at 50 Hz as block means over 20 ms. N ≈ 15,000–45,000 rows. All float32 except `phase`.

| Key | Shape | Unit | Meaning |
|---|---|---|---|
| `t` | (N,) | s | time since the start of the simulation |
| `motor_current_A` | (N, 6) | A | motor winding currents I₁…I₆ |
| `pwm` | (N, 6) | – | PWM duty of each motor (0–1) |
| `omega_cmd_rads` | (N, 6) | rad/s | commanded rotor speeds **after** the [200, 840] saturation |
| `omega_cmd_unclipped_rads` | (N, 6) | rad/s | allocator output **before** the saturation (shows how far a demand exceeds the limit) |
| `rotor_speed_rads` | (N, 6) | rad/s | actual rotor speeds (ESC telemetry) |
| `pack_voltage_V`, `pack_current_A` | (N,) | V, A | battery terminal voltage and current |
| `soc` | (N,) | 0–1 | state of charge (true, from the battery model) |
| `position_m` | (N, 3) | m | position **relative to A**, x east, y north, z up. Map position = `position_m[:, :2] + (330, 150)` (`io.map_xy`) |
| `velocity_ms` | (N, 3) | m/s | inertial velocity |
| `attitude_rad` | (N, 3) | rad | roll, pitch, yaw |
| `rates_rads` | (N, 3) | rad/s | body rates p, q, r |
| `accel_inertial_ms2`, `angular_accel_rads2` | (N, 3) | – | state derivatives |
| `accel_body_specific_ms2` | (N, 3) | m/s² | accelerometer signal (body specific force) |
| `reference_m` | (N, 3) | m | navigator reference position (relative to A) |
| `tracking_error_m` | (N, 3) | m | reference − position |
| `speed_cmd_ms` | (N,) | m/s | commanded cruise speed of the current segment |
| `altitude_ref_m` | (N,) | m | altitude reference |
| `phase` | (N,) | int8 | 0 ground, 1 climb, 2 hover, 3 cruise, 4 turn, 5 descent |
| `m1_friction_truth` | (N,) | N·m·s/rad | **ground truth** M1 friction (never an input). `io.load_flight` adds `d` = this / 2e-4, the friction factor |
| `wind_truth_ms` | (N, 3) | m/s | **ground truth** wind (mean + turbulence) at the vehicle |

## 5.2 `<flight_id>.json` and `data/flights/index.csv`: metadata and outcome

`index.csv` has one row per kit flight, with every field below plus `subset` and `source_dataset`.
`data/manifests/<dataset>_all_flights.csv` has the same fields for every flight of the full dataset.

### Design (inputs of the flight)

| Field | Meaning |
|---|---|
| `flight_id`, `family_id` | flight and family (same mission; only health or action differs within a family) |
| `block`, `split` | dataset block; train / val / test (test = held-out routes), eval_fault, eval_cf |
| `anchor` | True = baseline configuration (LQR C1, uniform allocation, S0, no derating) |
| `route_id`, `route_length_m` | route from the library (`data/routes`) |
| `altitude_m` | cruise altitude |
| `mass_kg` | take-off mass |
| `n_segments`, `speeds`, `cuts` | speed schedule: the cruise is split into `n_segments` at the path fractions `cuts`; `speeds` (m/s) per segment |
| `nominal_duration_s` | healthy, perfect-tracking duration from the navigator preview (to touchdown) |
| `deadline_s` | 1.2 × the nominal duration (arrival over B) |
| `wind_speed_ms`, `wind_from_deg` | mean wind at cruise altitude, direction it blows from (clockwise from north) |
| `wind_seed`, `turb_seed_per_flight` | turbulence seed; True = own realisation per flight |
| `friction_factor` | M1 friction at take-off, × healthy (b₀ / 2e-4) |
| `friction_band` | 0 = healthy; 1–5 = log-spaced bands of 1–8× (edges 1, 1.52, 2.30, 3.48, 5.28, 8); −1 = growing fault |
| `friction_constant` | True if constant in flight |
| `lqr_factors` | 10 factors on the baseline LQR: Q pos xy, Q vel xy, Q z, Q vz, Q roll/pitch, Q roll/pitch rate, Q yaw, Q yaw rate, R thrust, R moments (`io.LQR_FACTOR_NAMES`) |
| `alloc_mode` | 0 = legacy weights (`weights`), 1 = G23 allocation |
| `weights` | mode-0 allocation weights (uniform = all 1) |
| `alloc_shares` | mode 1: thrust pair shares (s₁₄, s₂₅, s₃₆) |
| `alloc_wm` | mode 1: moment weights w_M (6) |
| `alloc_sat` | mode 1: saturation strategy 0 S0 clip, 1 S1 priority, 2 S2 redistribute |
| `alloc_om1max` | mode 1: M1 ceiling (rad/s; 840 = no derating) |
| `takeoff_max_cmd` | highest rotor command at the take-off hover equilibrium (pre-flight filter) |
| `soc_reserve_pct` | SoC reserve used for grading (30) |

### Growing fault (v3, evalfault_v2, cf)

| Field | Meaning |
|---|---|
| `fault_onset_s`, `fault_onset_frac` | onset of the abnormal wear (s; fraction of the nominal duration) |
| `fault_doubling_s` | doubling time of the friction at ω_ref after the onset; λ_f = ln 2 / this |
| `fault_d_end_target` | designed friction at landing (× healthy); the realised value is ±15 % |
| `fault_speed_factor` | predicted mean (ω₁/ω_ref)² used to set the rate |
| `friction_end_x`, `friction_max_x` | realised friction factor at the end / maximum |
| `twin_id`, `twin_band` | (eval sets) the constant-friction v2 flight with the same mission |

### Counterfactual (cf only)

| Field | Meaning |
|---|---|
| `cf_branch` | none / alloc / payload / alloc_payload |
| `cf_t_dec_s` | decision time t_d (all branches are identical before it) |
| `cf_weights_b` | allocation weights after t_d (alloc branches): (4, 1, 1, 1, 1, 1) |
| `cf_drop_kg` | payload released at t_d (0.15 kg in payload branches) |
| `source_flight` | the evalfault_v2 flight it derives from |

### Outcome and grading

| Field | Meaning |
|---|---|
| `outcome` | **MISSION PASS** or **FAIL** (all requirements, doc 01 §1.6) |
| `Failed checks` | the checks that failed, e.g. "Envelope / divergence (F1); Landing (F7)" |
| `<check> ok` | one boolean per check: F1 envelope, F2 tilt, F3 attitude tracking, F4 rates, F5 altitude, F6a obstacle clearance, F6b ground impact, F7 landing, F8 corridor, F8 SoC reserve, F9 destination and deadline |
| `stop_why`, `t_end_s` | why the simulation stopped early (envelope rule), end time |
| `Destination`, `Arrival / touchdown (s)`, `Final SOC (%)`, `Max deviation (m)`, `energy_Wh` | mission figures |
| `max tilt (deg)`, `max attitude error (deg)`, `max roll/pitch rate (deg/s)`, `max altitude error (m)`, `contact …` | peak values used by the checks (computed at full rate) |
| `diag: Mk at 840 (s)` | time motor k spent at its upper limit (**diagnostic only**) |
| `diag: M1 min headroom (rad/s)`, `diag: min headroom any motor (rad/s)` | smallest 840 − ω_cmd |
| `diag: max thrust / moment residual` | wrench the allocation could not deliver |
| `diag: M1 max winding current (A)`, `diag: M1 max copper loss (W)` | thermal proxies |
| `outcome_soc50`, … (v2 only) | the original grade with a 50 % SoC reserve (before the regrade to 30 %) |

## 5.3 `data/health_traces/z_<dataset>.npz`: health estimates (1 s bins)

For the kit flights of dbdev_v3, evalfault_v2 and evalfault_cf. Airborne samples only, binned to 1 s (z = median in
the bin, other columns = mean). Use `io.load_health_traces(ds)` and `io.trace_of(tr, flight_id)`.

| Key | Shape | Meaning |
|---|---|---|
| `flight_id`, `t` | (N,) | flight and bin centre time (s) |
| `d` | (N,) | true M1 friction factor (evaluation only) |
| `u`, `ucmd` | (N, 6) | (ω/ω_ref)², actual / commanded, ω_ref = 455 rad/s: the **motor load** that drives wear |
| `Om` | (N, 13) | operating point: `io.OM_COLS` |
| `phase` | (N,) | 0 climb, 1 hover, 2 cruise, 3 turn, 4 descent (argmax of the airborne phase one-hot; **different coding from the 50 Hz `phase`**) |
| `z_R3` | (N, 1) | M1 health, single-latent model |
| `z_D0`, `z_E1` | (N, 6) | per-motor health, six-latent models (doc 03 §3.3) |
| `z_R3n`, `z_D0n`, `z_E1n` | – | operating-point normalised versions |
| `C`, `C_flight_ids` | (flights, 24) | configuration vector per flight, columns `io.C_COLS` (mass, log LQR factors, shares, log moment weights, saturation one-hot, M1 ceiling / 840) |

## 5.4 `data/map/`

| File | Content |
|---|---|
| `zurich_gablerstrasse.mat` | `complexMap` (475 × 600, 1 = building), `cell_m` = 2, origin in Swiss coordinates (EPSG:2056). **Row 0 is the north edge** (image order): use `io.is_free(grid, x, y)` or `imshow(grid, origin='upper', extent=(0, 1200, 0, 950))` |
| `occupancy_grid.npz` | the same grid as uint8 |
| `zurich_gablerstrasse.json` | map metadata (centre lat/lon 47.35919, 8.52895; CRS; source OSM, ODbL licence) |
| `zurich_gablerstrasse_buildings.geojson` | 864 building footprints (OSM) |
| `zurich_gablerstrasse_map.png`, `…_osm_overlay.png` | pictures of the map |
| `zurich_gablerstrasse_wind.json` | wind climatology (ERA5 2015–2024, urban log profile) used for the design wind |
| `sites.json` | A, B (dataset and short study), landing sites L1–L4 and their open radius |

## 5.5 `data/routes/`

| File | Content |
|---|---|
| `routes_summary.csv` | route_id, length_m, split (train 113 / test 33 / benchmark 1 / unused 4), number of points, via point(s) |
| `routes_xy.csv` | route_id, x_m, y_m: the planned 2-D polyline in map metres, A → B |
| `route_library_long.json/.npz` | the original library (with corner fillet data used by the navigator) |

The flown path differs from the polyline by corner rounding and tracking error; the altitude comes from the flight
(`altitude_m`).
