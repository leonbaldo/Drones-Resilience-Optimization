# Online waypoint navigation for the hexacopter model

An ArduPilot-inspired online waypoint-navigation architecture with dynamically constrained trajectory generation
and feedback-dependent path progression. Added 2026-11-11. ASSUMPTIONS.md tags: N1–N10.

It is **not** an implementation of ArduPilot. The ArduPilot sources (AC_WPNav, SCurve, AP_Math/control.cpp; GPLv3)
were used only as a reference for behaviour and architecture. No code was copied. §4 lists what was adopted and
§5 what differs.

---

## 1. The original architecture and its limitations

| Item | Before (still available, the default) |
|---|---|
| Reference | `route_ref` = [t x y z] from the planner, with timestamps `t = t0 + arc length / speed_mps` (2 m/s). It is sampled by the `PATH` From Workspace block with linear interpolation |
| Speed | Fixed in advance by the timestamps. The velocity steps from 0 to 2 m/s at the start of cruise, at the climb start and end, and at the landing |
| Geometry | Planner corners are circular fillets (G1). Curvature jumps from 0 to 1/R at every line–arc junction |
| Controller | `u = M r − L x`, with r = [x y z ψ] positions only. Velocity, attitude and rate states are regulated to **zero**. There is **no integral action** and no gravity feedforward: hover thrust comes from a z error |
| Consequences | Steady along-track lag of about 1.1 m at 2 m/s. Steady z sag of about 0.10 m at 1.9 kg. If the drone cannot keep up, the reference keeps moving on schedule and the error grows without bound |

The task brief described the controller as "LQR with integral action". The model has **none**: `LQR_design.m` is
a 12-state LQR and the Simulink controller is the static law above. This was checked in the block diagram and by
the identity `M = L·Cref'` (numerically 3e-14).

## 2. New architecture

```
Python (offline)                                   Simulink (online, every 2.5 ms)
---------------------------------------------      -------------------------------------------------------
planner.Plan  -> corners (polygon, radii)          'Waypoint navigator' (MATLAB Function -> wpnav_step.m)
path.GeometricPath  (G2: clothoid-arc-clothoid)      inputs : vehicle x,y,z,vx,vy,vz (ZOH), t, v_cmd, pause
navigation.Mission  (legs, stops, waypoints)         state  : (s, s', s''), leg, mode, hold timer, waypoint
navigation.speed_envelope (curvature limits)         outputs: p,v,a,j reference, xr_dyn, u_ff, 32 diagnostics
NavInput.arrays() -> nav_path/legs/wps/par  ---->  'Ref select'  r(1:3) = navigator p  (if nav_par(1) = 1)
NavInput speed / pause schedules -> *_ts    ---->  u_cmd = (M r − L x) + blend(dM r − dL x) + u_ff + (L + blend dL) xr_dyn
                                                   motors, battery, wear, wind: unchanged
```

- **Geometry** (`resilience/path.py`) has no time in it. A path is sampled in arc length, with position, unit
  tangent T, curvature vector K = dT/ds and dK/ds. Each planner corner keeps the planner's tangent points, so its
  footprint and the corridor checks are unchanged. The corner is rebuilt as a symmetric clothoid–arc–clothoid with
  transition length Lt = λR′ (λ = 1). Because the shape scales with R′ when Lt = λR′, R′ follows from the
  fillet's tangent length in closed form, and it is kept ≥ the planner's minimum radius. The path is G2, so
  lateral acceleration is continuous. Clearance is re-checked with the planner's inflated obstacles. Repeated and
  collinear vertices are removed. A turn above 175° cannot be rounded inside a leg and raises an error (make it a
  stop).
- **Mission** (`resilience/navigation.py`) is a list of legs. Each leg ends at a **stop** waypoint. Inside a leg
  the polygon vertices are **pass-through** waypoints, flown at non-zero speed. The study mission is: climb
  (vertical leg, hover `hover_s` at the top) → cruise A→B (G2 path) → hold at B until the end of the simulation.
  A landing leg is optional.
- **Navigator** (`wpnav_step.m` and helpers): one source is used by both Simulink and the tests
  (`wpnav_replay.m`). It runs at 400 Hz (`nav_Ts`), the ArduPilot Copter main-loop rate. It is deterministic, and
  its persistent state is re-initialised at every simulation start.
- **Controller**: the LQR gains are unchanged. The navigator's full reference state is tracked (§3.4).
- **Default off**: with `nav_par(1) = 0` the navigator outputs zeros and the reference comes from `PATH`. Legacy
  flights are **bit-identical** to the model before the edit (test 1 in `tests/test_simulink_nav.py`).

### Route change (divert) [N11, added 2026-11-11]

`NavInput(..., divert=(t_d, divert_mission))`. At t_d the navigator:
1. **brakes** to rest along the current path with the same constant-jerk law as a pause (state 8). The vehicle
   is not used, so the stop point depends only on the navigator state at t_d;
2. **joins** the first point of the divert route with a straight leg built online (state 9), if the gap is
   more than 1 µm;
3. flies the **divert legs**: cruise along a G2 path to the site, hold `divert_hold_s`, vertical landing.
   The waypoints of the abandoned route are skipped.

The divert route is planned in Python (`nav_options.divert_mission`) from the **predicted stop point**:
`predicted_stop` replays the navigator from the state logged at t_d (`navigation.state_from_log`). This gives
the flight's own stop point, so the join leg is normally absent. It is there as a guard if the prediction is off.
This is a stop-and-go handover. ArduPilot instead starts the new leg from its stopping point while still moving
(deliberate difference 8).

### Navigation state machine

| Code | State | Meaning |
|---|---|---|
| 0 | init | before `t_start`; reference held at the first point |
| 1 | takeoff | moving on a climb leg |
| 2 | cruise | moving on a cruise leg; the binding limit is the command, curvature or feedback |
| 3 | approach | moving; the stop curve toward the leg end is the binding limit |
| 4 | hold | the stop waypoint was reached; holding for the leg's `hold_s` (−1 = until the end) |
| 5 | land | moving on a landing leg |
| 6 | complete | last stop reached and its hold elapsed (or infinite) |
| 7 | paused | pause command: braking along the path, then at rest; resumes on release |
| 8 | braking (route change) | divert time reached: braking to rest along the current path |
| 9 | join | straight join leg from the stop point to the divert route |

A **stop waypoint is reached** when the target has finished (|s1 − s| < 2 cm and |s'| < 0.02 m/s) **and** the
vehicle is within the acceptance radius (2 m, 3D). This is the same rule as ArduPilot's non-fast waypoints. A
pass-through waypoint is reached when the target passes it. The **next leg** starts once the hold has elapsed and
the reference has settled on the junction (≤ 1 µm, ≤ 10 µm/s). The 1-D state is then restarted at rest on the
junction, so position, velocity and acceleration stay continuous although the legs meet at an angle.

## 3. Equations

### 3.1 Cartesian reference from the path state

The reference point moves along the path with arc length s(t):

    p = P(s),   v = T ṡ,   a = T s̈ + K ṡ²,   j = T s⃛ + 3 K ṡ s̈ + K′ ṡ³,     K = κN,  K′ = dK/ds = κ′N − κ²T.

Bounding the scalar jerk s⃛ alone does **not** bound |j| on a curve. The navigator therefore splits a fixed jerk
budget J (2 m/s³ by default, horizontal and vertical):

| Term | Bound | Enforced by |
|---|---|---|
| tangential T s⃛ | (1 − η) J, η = 0.5 | jerk clamp of the 1-D shaper (constant per leg) |
| geometric K′ ṡ³ | η J / 2 | speed envelope: ṡ ≤ (ηJ / 2 / \|K′\|)^(1/3) |
| coupling 3 K ṡ s̈ | η J / 2 | tangential acceleration limit on curves: \|s̈\| ≤ ηJ / (6 κ ṡ), with look-ahead |

Together they give |j| ≤ J. The lateral acceleration κṡ² ≤ a_c follows from ṡ ≤ √(a_c/κ). The tangential
acceleration is |s̈| ≤ A. For the vertical speed, a leg's ṡ cap is v_cmd/|T_xy| combined with v_up/T_z or
v_dn/(−T_z). Per-leg A and J use ArduPilot's kinematic_limit rule: the largest value whose horizontal and vertical
components both fit.

### 3.2 Target path speed (online, each update)

    v_t = min( v_cap , v_env(s, s + ṡ·A/J_s) , v_fb , v_stop )

- **v_cap**: the speed command (horizontal, a step input from `nav_speed_ts`), capped at 15 m/s, with the vertical
  limits above.
- **v_env**: the precomputed curvature envelope. Each dip of v_geo = min(√(a_c/κ), (ηJ/2/|K′|)^(1/3)) is
  propagated backwards with the exact constant-jerk braking distance (§3.3), using J_s = (1 − η)J and the
  tangential acceleration available at the dip. It is evaluated at s and at a look-ahead point ṡ·A/J_s ahead (the
  time to build up deceleration). It depends only on geometry and limits, not on time. The speed command is
  applied online.
- **v_stop**: speed from which the remaining distance to the leg end can still be covered by a constant-jerk stop.
  The distance is first reduced by the distance needed to ramp a positive acceleration down to zero:

      d_eff = d − (ṡ t_a + ½ a⁺ t_a² − J_s t_a³/6),   t_a = a⁺/J_s
      v_stop = min( V(d_eff),  (p_v/4)·d_eff ),
      V(d) = (J′ d²)^(1/3)                     for d ≤ A³/J′²
      V(d) = A(−A/(2J′) + √(A²/(4J′²) + 2d/A))  otherwise,     J′ = 0.7 J_s

  The linear cap p_v/4 makes the terminal position–velocity cascade critically damped. Without it the reference
  overshoots the stop point. With it, the tail is exponential (time constant A/(2J_s) = 1.25 s horizontal).
- **v_fb**: the **feedback** term (§3.5).

### 3.3 Jerk-limited 1-D shaper

    a_target = clamp( a_ff + sqrtctrl(v_t − ṡ;  p_v = 8 J_s/A,  J_s) ,  ±A_t )
    Δa       = clamp( a_target − s̈ ,  ±J_s Δt ),   s⃛ = Δa/Δt   (exact constant-jerk integration over Δt)

`sqrtctrl` is the square-root controller: gain p near zero, √(2J|e|) shape further out, clamped to |e|/Δt. It is
the same piecewise law as ArduPilot's `sqrt_controller`. a_ff = Δv_t/Δt is the target's own rate of change,
applied only while the same geometric limit (v_env or v_stop) stays binding. It plays the role of
`accel_desired` in ArduPilot's `shape_vel_accel` and removes the lag of the sqrt shape, a²/(2J), which otherwise
gave 14 cm overshoot at the 8 m/s stop. The constant-jerk stop distance for v → v_e is (v + v_e)√(Δv/J) for
Δv ≤ A²/J, and ½(v + v_e)(Δv/A + A/J) otherwise.

### 3.4 Tracking law

The linear design model is ẋ = A x + B u with x = [x vx y vy z vz φ p θ q ψ r]. Velocities are **inertial**:
the plant's states 2, 4, 6 are inertial velocities, and the navigator also works in the inertial frame. The model
has v̇x = gθ, v̇y = −gφ, v̇z = u1/m (without gravity). A reference x_r(t) with input u_r that satisfies
ẋ_r = A x_r + B u_r is

    x_r = [p_x, v_x, p_y, v_y, p_z, v_z, −a_y/g, −j_y/g, a_x/g, j_x/g, ψ_r, 0],
    u_r = [m_d (g + a_z), I_x ṗ_r, I_y q̇_r, 0]

The g in u_r is gravity compensation for the real plant, which the linear model omits. The control law
u = u_r − L (x − x_r) then gives the error dynamics ė = (A − BL) e + B(u_r − u_r,used). With
`M = −(Cref (A − BL)⁻¹ B)⁻¹ = L Cref'` (verified), the existing blocks `M r − L x` already supply −L(x − Cref' r).
The model only **adds**

    u_nav = u_ff + (L + blend·dL) xr_dyn,     xr_dyn = x_r − Cref' r,    u_ff = [m_d (g·g_ff + a_z·a_ff), 0, 0, 0]

The result is u = u_ff − (L + blend·dL)(x − x_r). With `blend` it also holds while the existing controller
switch (L → L_b, M → M_b) is active, because dM = dL·Cref' too. Yaw and the allocation are unchanged.

What is and is not fed forward:
- **Included**: v_r, the attitude references from a_r, the rate references from j_r, and the thrust m_d(g + a_z).
  These are the exact inverse of the LQR's own linear design model. No new or unverified model is introduced.
- **Omitted**: the moments I ṗ_r, I q̇_r. They need snap (d⁴p/dt⁴), which the reference does not bound. The
  residual excites the error dynamics only through the snap of the reference.
- **Omitted**: drag. The plant's form drag (L2) is treated as an unmodelled disturbance, as before. Without
  integral action this leaves a steady along-track lag that grows with speed (linear analysis: 0.05 / 0.28 /
  0.71 m at 2 / 5 / 8 m/s).
- **Gravity feedforward** uses the controller's design mass `m_d` = 1.9 kg, not the true mass. After a payload
  drop the z offset therefore returns, as it would on a real vehicle without an integrator.
- The LQR was **not retuned**: its gains, poles and weights are those of `cfg.q_a`, `cfg.r_a`.

### 3.5 Feedback-dependent progress (guidance law)

The reference is **not** a fixed timeline. It advances only as fast as the shaper allows, and one of the
shaper's limits is the vehicle's own progress:

    e_lead  = Tᵀ (p_ref − p_vehicle)      (reference ahead of the vehicle along the track)
    v_track = Tᵀ v_vehicle
    v_fb    = max(0,  v_track + k_lead (L_lead − e_lead)),   k_lead = 1 /s,  L_lead = 1.5 m

- While the vehicle keeps up (e_lead < L_lead), v_fb > v_track and the command or geometry binds.
- If the vehicle falls behind, the reference slows to the vehicle's own speed. The lead converges to L_lead.
- If the vehicle stops, the reference brakes (jerk-limited) and stops. The lead is bounded by
  **L_lead + the jerk-limited stopping distance** from the speed it had (test `test_stalled_vehicle_reference_stops`).
- The reference is never forced onto the vehicle. Path progress s_ref and vehicle progress s_vehicle = s_ref −
  e_lead are both logged.
- The navigator receives no degradation information: only position, velocity, the speed command and the pause
  flag. A degraded drone flies slower or with larger errors purely through this feedback.

This replaces naive time-based progress. It is the speed-domain counterpart of ArduPilot 4.x's track scaler (§4).

## 4. ArduPilot concepts adopted

| ArduPilot (AC_WPNav, SCurve, control.cpp) | Here |
|---|---|
| Jerk-limited trajectory, separate horizontal/vertical speed and accel limits, `kinematic_limit` for 3D legs | Same limits, same per-leg rule |
| Corner speed `√(accel_c · radius)` for arcs; `WP_ACC_CNR` | √(a_c/κ) on the sampled curvature |
| Fast vs non-fast waypoints; non-fast reached when s finished **and** within `WP_RADIUS` (3D) | Pass-through vertices vs stop waypoints, same rule |
| Track scaler `0.05 + (v_track − kP·e)/v_target`, filtered with τ = A/J, scales the S-curve's dt | v_fb = v_track + k_lead(L_lead − e_lead) as a speed limit of the shaper (§5) |
| Online speed change (`set_speed_NE_ms` → `SCurve::set_speed_max` rebuilds the remaining segments) | Speed command is a target of the online shaper: any time, no rebuild |
| `sqrt_controller`, `shape_vel_accel` (accel_desired feedforward, jerk clamp per dt) | Same structure (`wpnav_sqrt_ctrl.m`, §3.3) |
| Default values WP_ACC 2.5 m/s², WP_ACC_Z 1.0 m/s², WP_JERK 1 m/s³, WP_RADIUS 2 m, PSC_POSXY_P 1 | Used as starting values (§7) |
| 400 Hz main loop | `nav_Ts` = 2.5 ms |

## 5. Deliberate differences from ArduPilot

1. **No precomputed S-curve segments.** ArduPilot builds a 23-segment raised-cosine-jerk profile per straight leg
   and rebuilds it on speed changes. Here a 1-D jerk-limited shaper runs online against a speed envelope. Its jerk
   is piecewise constant, not raised cosine. Speed changes and pauses need no rebuild.
2. **Corners are explicit G2 geometry** (clothoid–arc–clothoid from the planner's fillets), not the overlap of two
   legs' S-curves near the waypoint. The flown corner is therefore known in advance and clearance-checked.
3. **Cartesian jerk is bounded explicitly** by the budget split in §3.1. ArduPilot limits jerk per axis along the
   S-curve.
4. **Feedback acts on the target speed**, with an allowed lead L_lead, instead of scaling the S-curve time with a
   0–1 factor. Both slow the reference when the vehicle cannot follow. Ours leaves the shaper's jerk limits intact
   by construction.
5. **The position controller is the existing LQR**, not AC_PosControl (sqrt position controller + PID velocity
   loop with integrator). There is no integral action and no drag feedforward, so steady errors remain under drag.
6. **No splines, terrain following, yaw control, or object avoidance.** Yaw is held at 0, as before.
7. **Stops settle exponentially** (critically damped terminal law), so a stop takes a few seconds longer than
   ArduPilot's finite-time S-curve end.
8. **Route changes stop first** (brake to rest, join, go) instead of blending into the new route at speed.

## 6. Files

| File | New / changed | What |
|---|---|---|
| `resilience/path.py` | new | G2 geometric path (clothoid corners, sampling, projection) |
| `resilience/navigation.py` | new | NavLimits, legs, Mission, speed envelope, fixed-size tables, NavInput, MATLAB replay |
| `resilience/nav_study.py` | new | closed-loop scenarios, analysis (navigation vs control errors), 9-panel plots |
| `wpnav_step.m`, `wpnav_eval.m`, `wpnav_init.m`, `wpnav_sqrt_ctrl.m`, `wpnav_vstop.m` | new | navigator (codegen-compatible) |
| `wpnav_replay.m` | new | navigator against a kinematic vehicle (tests, preview) |
| `model_edit_wpnav.m` | new | scripted edit of `hexa_waypoints.slx` (reproducible). Backup: `legacy/model_backups/hexa_waypoints_prenav_20261111.slx` |
| `hexa_waypoints.slx` | changed | blocks 'Waypoint navigator', 'Nav pos/vel ZOH', 'Nav clock', 'Nav speed cmd', 'Nav pause', 'Ref select', 'Legacy ref', 'Ref demux', 'Nav L', 'Nav dL', 'Nav blend', 'Nav ff'; `u_cmd` has a third input; model-workspace defaults `nav_*` (off) |
| `resilience/planner.py` | changed | `_fillet` also returns the polygon and radius per vertex; `Plan.corners`. Routes are unchanged (regression test) |
| `resilience/sim.py` | changed | `run(..., nav=NavInput)`, navigator wiring checks, `nav_y` log, cache version 5 (navigator files hashed) |
| `resilience/config.py` | changed | `nav_*` settings [N1–N9] |
| `tests/` | new | `test_path.py`, `test_navigator.py`, `test_simulink_nav.py` (opt-in), fixtures in `tests/data/` |
| `online_navigation.ipynb` | new | runs the closed-loop scenarios and shows tables and figures |

## 7. Parameters (StudyConfig, SI units)

| Field | Default | Unit | Basis |
|---|---|---|---|
| `nav_dt_s` | 0.0025 | s | ArduPilot Copter loop rate (400 Hz); multiple of the 1.25 ms solver step |
| `nav_accel_xy` | 2.5 | m/s² | ArduPilot WP_ACC default; 14° tilt |
| `nav_accel_corner` | 2.5 | m/s² | ArduPilot uses 2×WP_ACC = 5; halved here (more conservative) |
| `nav_jerk_xy`, `nav_jerk_z` | 2.0 | m/s³ | Cartesian bound; (1 − η)J = 1 m/s³ along the track = WP_JERK default. Tilt rate J/g ≈ 12°/s, well inside the attitude loop (poles ≈ 10–14 rad/s) |
| `nav_speed_up`, `nav_speed_down` | 1.0, 1.0 | m/s | Keep R2 (20 m in 20 s). ArduPilot defaults 2.5 / 1.5 |
| `nav_accel_z` | 1.0 | m/s² | ArduPilot WP_ACC_Z default |
| `nav_speed_max` | 15 | m/s | absolute cap on any command |
| `nav_corner_jerk_share` η | 0.5 | – | jerk budget split (§3.1) |
| `nav_transition_ratio` λ | 1.0 | – | clothoid length / arc radius; then the clothoid term κ′ṡ³ equals the arc term κ²ṡ³ and does not lower the corner speed |
| `nav_lead_m` L_lead | 1.5 | m | about 2× the drag-induced lag at 8 m/s (0.71 m, linear analysis) |
| `nav_lead_gain` k_lead | 1.0 | 1/s | ArduPilot PSC_POSXY_P default |
| `nav_wp_radius_m` | 2.0 | m | ArduPilot WP_RADIUS default |
| `nav_finish_tol_m` | 0.02 | m | target "finished" at a stop |
| `nav_lookahead` | 1.0 | × A/J_s | time to build up deceleration |
| `nav_vel_gain_factor` | 8 | – | p_v = 8 J_s/A: with the terminal gain p_v/4, ζ ≥ 1 |
| `nav_stop_jerk_share` | 0.7 | – | stop planned with 0.7 J_s (margin for the shaping lag) |
| `nav_gravity_ff`, `nav_accel_ff` | True | – | §3.4 |

None of these limits is validated against flight data. They are starting values from ArduPilot defaults and from
the linear closed-loop poles (slowest 3.1 rad/s, ζ = 0.71).

## 8. Tests and results (2026-11-11)

### 8.1 Automated tests: all pass

| Suite | Tests | What is checked |
|---|---|---|
| `tests/test_path.py` (Python, 0.2 s) | 9 | straight path; repeated and collinear vertices; degenerate inputs (zero length, one point, 180° reversal, corner without radius, NaN); G2 corner keeps the fillet's tangent points, curvature continuous, dP/ds = T, dT/ds = K; radius floor; projection; **planner routes identical to the pre-change planner** (Main + 2 alternatives); study mission clear of the inflated obstacles |
| `tests/test_navigator.py` (MATLAB engine, 11 s) | 13 | straight line; takeoff + 10 s hover + cruise + hold; multiple waypoints with smooth turns (pass at speed); 120° turn at R = 4 m; 3 m and 0.2 m legs; repeated waypoints (identical output); speed change 5 → 2 → 7 m/s; pause/resume; vehicle limited to 2 m/s (reference slows to it); vehicle stalls (reference stops, lead ≤ L_lead + stopping distance, no false completion); climb and landing limits; invalid inputs; envelope feasibility. In every replay: speed ≤ command, \|s̈\| ≤ A, lateral ≤ 1.02 a_c, \|j\| ≤ J, increments of p, v, a bounded by the next derivative (continuity), finite differences equal the analytic v and a, path speed never negative |
| `tests/test_simulink_nav.py` (Simulink, 3 min, opt-in) | 3 | **legacy reference bit-identical** to the model before the edit (dd2, u_cmd, omega_cmd, m1_friction; 60 s with the M1 wear fault); navigated flight completes, < 1 m error, charge balance (Coulomb) holds, dd2 reference = navigator output; **M1 wear under navigation = Python wear law within 1e-3** |

### 8.2 Closed loop (`online_navigation.ipynb`, `results/navigation/run_20261111/`)

Study mission: climb 20 m, hover 10 s, cruise A → B (563.8 m G2 path), hold at B. Same LQR gains (C1), 1.9 kg,
no wind.

| Scenario | Complete (s) | Mean cruise (m/s) | Max 3D err (m) | Max lag (m) | Max cross (m) | Max \|j_ref\| | M1 cmd at 840 | Energy (Wh) | Final SoC |
|---|---|---|---|---|---|---|---|---|---|
| healthy 2 m/s | 321.7 | 1.99 | 0.06 | 0.05 | 0.05 | 1.00 | 0 % | 21.24 | 78.2 % |
| healthy 5 m/s | 156.6 | 4.81 | 0.28 | 0.27 | 0.07 | 1.23 | 0 % | 11.44 | 88.4 % |
| healthy 8 m/s | 124.8 | 6.67 | 0.63 | 0.62 | 0.17 | 1.23 | 0 % | 10.23 | 89.7 % |
| speed 5 → 3 (70 s) → 6 (120 s) | 169.5 | 4.32 | 0.39 | 0.38 | 0.08 | 1.23 | 0 % | 12.42 | 87.4 % |
| pause 90–105 s at 5 m/s | 171.7 | 4.70 | 0.28 | 0.27 | 0.07 | 1.23 | 0 % | 12.39 | 87.4 % |
| D11 fault (3×, ×2/90 s from 60 s) at 5 m/s | 156.5 | 4.81 | 0.54 | 0.28 | 0.54 | 1.23 | 0 % | 15.29 | 84.2 % |
| D11 fault at 2 m/s | 321.7 | 1.99 | 0.85 | 0.06 | 0.85 | 1.00 | 29 % | 30.70 | **67.3 %** |
| severe fault (6×, ×2/30 s from 50 s) at 5 m/s | 156.5 | 4.81 | 0.95 | 0.29 | 0.95 | 1.23 | 52 % | 18.96 | 80.0 % |
| *old time-scheduled reference, 2 m/s* | (310.7 schedule) | 2.0 | 1.19 | 1.19 | – | steps | 0 % | 21.23 | 78.2 % |

Findings:
- **Feasible speeds.** 2, 5 and 8 m/s are all flyable healthy. No motor command saturates, and the peak reference
  |a| is 2.3 m/s². At 8 m/s the mean cruise speed is 6.7 m/s: the reference is limited by curvature 26 % and by
  the lead feedback 19 % of the cruise time. The feedback mostly acts during the acceleration ramps, when the
  vehicle trails. The 8 m/s lag (0.62 m) matches the linear drag prediction (0.71 m). It is a controller
  limitation (no integral, no drag feedforward), not a navigation error.
- **Better tracking than before.** At 2 m/s the error falls from 1.19 m (old: positions only) to 0.06 m. The
  hover z sag (−0.13 m old) is removed by the gravity feedforward. Energy is the same (21.24 vs 21.23 Wh).
- **Mission time is no longer prescribed.** At 2 m/s the mission completes at 321.7 s: B is reached at 316 s, vs
  310.7 s on the old timeline. The difference comes from jerk-limited ramps, corner slow-downs and settling. The
  study's decision time (P1, 100 s) and deadline (Q1, 330 s) were set on the old timeline. Re-check them before
  using the navigator in the resilience study.
- **Speed changes and pause** are smooth: no velocity step, tangential jerk within ±1 m/s³, no reset, no
  position jump. The vehicle follows within 0.4 m.
- **Degraded drones, same navigation settings.** Degradation shows up as **cross-track** error: M1's weaker
  thrust is not compensated laterally without integral action, giving up to 0.85–0.95 m. It also shows up as
  motor saturation (29 % / 52 % of the time with M1's command at 840 rad/s) and as energy. The vehicle keeps up
  along the track, so the feedback does not slow the reference. The mission therefore completes at the same time
  as healthy. The navigator receives no health information. At 2 m/s the D11 fault ends at 67.3 % SoC, below the
  70 % reserve (Q4), consistent with wear_fault_flight.ipynb (No action → FAIL on SoC).
- **Separation of errors.** At navigation level, the reference stays on the geometric path (deviation of vehicle
  vs path ≈ cross-track to the reference, because the reference is on the path). The binding limit is logged per
  sample ("Cruise limited by"). At control level, lag, cross-track, saturation and energy are reported
  separately, per flight, in `metrics.csv`.

### 8.3 Known limitations and next steps

1. **No integral action** in the LQR: steady lag under drag (grows with v²) and a lateral offset with an
   asymmetric (degraded) rotor. Adding integral states on position is the obvious next controller change. It
   needs a retune and its own validation.
2. Route changes are implemented as stop-and-go (§2, route change; `resilience/nav_options.py`). A blended
   handover at speed is not.
3. **Prognosis and wind need a timeline.** The battery prognosis, `routes.phases` and the wind profile use a
   planned [t x y z]. For navigated flights, use the perfect-tracking preview (`navigation.replay` → `timeline`).
   That is an assumption, not the flown timeline. `HexaSim.run` refuses navigated flights with wind unless a wind
   series is given.
4. Moment (snap) feedforward and drag feedforward are omitted (§3.4). Yaw stays at 0.
5. Stops settle exponentially (a few seconds). The terminal law could be made finite-time.
6. The limits (§7) are ArduPilot defaults and linear-model reasoning, not identified for this airframe.
7. Every flight starts with a rotor-speed transient in its first fraction of a second: up to 549 rad/s navigated
   and healthy, and 673 rad/s on the old reference with 6× friction. It predates the navigator and was not
   investigated.

## 9. Running

```python
from resilience.config import StudyConfig
from resilience import nav_study as ns, navigation as nv
from resilience.sim import HexaSim
cfg = StudyConfig()
ctx = ns.setup(cfg)                                   # planner route -> navigator mission
sim = HexaSim(cfg).start()
f = ns.fly(sim, ctx, speed=[(0, 5.0), (70, 3.0)])     # online speed change at 70 s
an = ns.analyze(f, ctx, t_cmd=70); ns.plot_flight(an, ctx)
# legacy (time-scheduled) flights are unchanged: sim.run(route, L, M, ...)
```

Tests:

```bash
python -m unittest tests.test_path tests.test_navigator -v         # ~15 s (MATLAB engine for the navigator)
RUN_SIMULINK=1 python -m unittest tests.test_simulink_nav -v       # ~3 min, closed loop
```

To redo the model edit from the backup: `copyfile` the backup over `hexa_waypoints.slx`, then run
`model_edit_wpnav('hexa_waypoints')`.
