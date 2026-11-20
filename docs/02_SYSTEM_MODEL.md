# 2. System model: vehicle, controller, allocation, motors, battery, wind

The Simulink model `hexa_waypoints.slx` (not included: it needs MATLAB R2022b) is a closed-loop hexacopter. This
document gives the equations and constants you need to build your own surrogates. The Python copies in
[hexakit/whitebox/](../hexakit/whitebox/) reproduce the steady-state parts exactly. Tags point to
[reference/ASSUMPTIONS.md](reference/ASSUMPTIONS.md).

## 2.1 Signal chain

```
 navigator ──r(t)──► LQR tracker ──u=[T,Mx,My,Mz]──► allocation ──ω²_cmd──► saturation [200², 840²] ──► √ ──► ω_cmd
 (route, speed,      (12 states,                     (pair shares,                                            │
  corners, landing)   no integral action)             moment weights,                       PWM = 0.0008842·ω_cmd + 0.03436
                                                      sat. strategy)                                          │
                                                                                                              ▼
 6-DOF rigid body ◄── thrust/torques F·ω² ◄── actual rotor speed ω_k ◄── BLDC motor k (friction b_k) ◄── PWM_k · V_pack
 + form drag (wind)                                                        │
                                                                           └── motor current I_k ──► pack current ──► battery (SoC, V)
```

Important: **there is no speed loop in the ESC.** PWM follows the command through a fixed affine map. Speed is
closed only by the flight controller (when the vehicle drifts, the LQR raises the wrench and the allocation raises
the commands).

## 2.2 Constants

| Group | Quantity | Value | Tag |
|---|---|---|---|
| Airframe | arm length l | 0.30 m | – |
| | take-off mass | 1.6–2.2 kg in the data (empty with battery 1.60 kg; nominal payload 0.30 kg; LQR designed at 1.90 kg) | M1–M3 |
| | yaw inertia I_zz | 0.0432 kg·m²; **no yaw damping** in the plant | – |
| | form drag | F_D = −½ρC_D·[A_yz·v_x\|v_x\|, A_xz·v_y\|v_y\|, A_xy·v_z\|v_z\|] (body frame, air-relative), C_D = 1, A_xy = 0.40 m², A_xz = A_yz = 0.22 m² | L2 |
| Rotor | radius 0.129 m, C_T = 0.0141, C_Q = 0.0012, ρ = 1.225 kg/m³ | thrust b·ω², drag torque d·ω²·f (f = inflow factor, 1 at hover) | S10 |
| Motor (BLDC) | K_E = K_T = 9.549/920 V·s/rad, R = 0.2 Ω, Coulomb friction T_f = 0.04 N·m, **viscous friction b = 2e-4 N·m·s/rad healthy** | – | D1 |
| | electrical time constant: a few ms; mechanical ≈ 0.08 s | – | – |
| | command limits | ω_cmd ∈ [200, 840] rad/s per motor | – |
| | battery current of motor k | 0.3·PWM_k·I_k (a gain inherited from the model) | L1 |
| Battery | 4S LiPo, 6.2 Ah; 1-RC Thevenin: R0 = 0.03 Ω, Rd = 0.0237 Ω, Cd = 2400 F (τ ≈ 57 s); OCV 16.79 V full, 14.17 V empty | `whitebox/battery.py` (matches Simulink within 7e-6 SoC) | B0 |
| Simulation | ode1, fixed step 1.25 ms; deterministic; export at 50 Hz (block means) | – | S1 |
| Envelope stop | roll or pitch > 60°, or z ≤ −1 m → flight ends | – | S5 |

## 2.3 Motor and rotor (per motor k)

```
I_k = (V_pack·PWM_k − K_E·ω_k) / R
J·dω_k/dt = K_T·I_k − T_f − b_k·ω_k − d·ω_k²·f(ω_k², v_air)
```

At steady state, for a given PWM, a larger b_k gives a **lower ω_k and a higher I_k**. The controller sees the lost
thrust and raises M1's command, so the PWM rises too. Degradation is therefore visible as (i) more current for the
command, (ii) a speed deficit, and (iii) a load shift in the allocation.

Steady-state trim example (`load_model.steady_state`, 1.9 kg, hover, 15.8 V):

| M1 friction | Allocation | ω_cmd (rad/s) M1…M6 | actual ω M1…M6 | pack current |
|---|---|---|---|---|
| 1× | uniform | 527 ×6 | 455 ×6 | 14.3 A |
| 1× | shares (0.15, 0.425, 0.425), S2 | 355, 597, 597, 355, 597, 597 | 311, 512, 512, 311, 512, 512 | 14.4 A |
| 4× | uniform | 706, 578, 578, 410, 578, 578 | 358, 496, 496, 358, 496, 496 | 19.5 A |
| 4× | shares (0.15, 0.425, 0.425), S2 | 453, 623, 623, 263, 623, 623 | 230, 533, 533, 230, 533, 533 | 16.7 A |
| 8× | uniform | 813, 613, 613, 302, 613, 613 | 265, 524, 524, 265, 524, 524 | 23.9 A |
| 8× | shares (0.15, 0.425, 0.425), S2 | 535, 637, 637, 200 (floor), 637, 637 | 174, 543, 543, 174, 543, 543 | 18.9 A |

Note the two structural facts: M1's and M4's **actual** speeds are equal in every case (pair coupling, doc 01 §1.5a),
and at 8× with M1 unloaded, M4's command hits its 200 rad/s floor.

## 2.4 Allocation

**Mode 0 (legacy, switchable in flight):** ω²_cmd = S·pinv(F·S)·u, S = diag(1/√w), then clipped. Uniform w is the
default. The in-flight "allocation" action is w = (4, 1, 1, 1, 1, 1), blended over 0.5 s.

**Mode 1 (G23, per flight):** ω²_cmd = B_T(s)·T + B_M(w_M)·[Mx, My, Mz]ᵀ, then the saturation strategy:
- B_T: each pair gets its share s_ij of the thrust, half per motor; moment-free by construction.
- B_M: a weighted pseudo-inverse of the moment columns only.
- S0 clips each motor. S1 keeps roll and pitch, sheds yaw first, then shifts thrust, and scales roll/pitch only as a
  last resort. S2 fixes saturated motors at their limit and re-allocates the remaining wrench to the free ones.
- M1 ceiling ω̄₁: the allocator treats M1's upper limit as ω̄₁ < 840 rad/s.

Python: `whitebox.allocation.allocate(u, AllocConfig(...))` reproduces Simulink to 7e-16 of full scale.

## 2.5 Controller and navigator

- **LQR**, full-state tracking of position, velocity, attitude and rates (12 states), 4 inputs, designed on the
  linearised hover model at 1.90 kg. Baseline C1: Q = diag(1000, 1, 1000, 1, 2e6, 1, 1000, 1, 1000, 1, 1000, 1),
  R = 60·I₄. In DB_dev, 8 groups of Q and the 2 groups of R are scaled by factors in ×0.1–×10 (`lqr_factors`, order
  in [05_DATA_DICTIONARY.md](05_DATA_DICTIONARY.md)). Designs are screened: closed-loop poles ≤ 3× baseline speed,
  damping ≥ 0.3, plus a 40 s Simulink screening flight.
- **No integral action** (settled design choice): a steady altitude sag of ~13 cm at 1.9 kg and ~1.1 m of
  along-track lag at 2 m/s.
- **Navigator** (ArduPilot-inspired, 400 Hz): the path is geometry only (lines and clothoid-like fillets from the
  planner); the reference advances online with the speed limits: horizontal acceleration 2.5 m/s², cornering
  2.5 m/s², jerk 2 m/s³, vertical speed 1 m/s. Mission: vertical climb, hover, cruise along the route, hold 2 s over
  B, vertical landing at 1 m/s.
- **Divert** (N11): brake to rest on the current path, straight join leg, planned route to the site, hold, land.

## 2.6 Wind

- Mean wind U at cruise altitude, direction from the meteorological convention (clockwise from map north). Log
  profile below the cruise altitude, zero below 10.8 m (climb and landing start in calm air).
- Turbulence: first-order Gauss-Markov (Dryden form), σ = (2.4, 1.9, 1.3)·u*, length scales from MIL-F-8785C.
  σ_u ≈ 1.1 m/s at U = 2.4 m/s.
- Wind enters only through the form drag (and the rotor inflow factor). There is no rotor-aerodynamics gust model.
- DB_dev: U ∈ [0, U_max(altitude)] with the airspeed budget v_max + U + 2σ_u ≤ 10 m/s (G20).

## 2.7 Map, routes and landing sites

- Map `zurich_gablerstrasse`: 1200 × 950 m, 2 m cells, building footprints from OpenStreetMap (26 % occupied).
  Origin at the lower-left corner, x east, y north (map metres). Only buildings are known: streets, parks and water
  are all "free".
- Start/home A = (330, 150). Dataset goal B = (740, 860) (shortest route 975 m). An older short study used
  B = (594, 464).
- Landing sites (placeholders, the most open free points along the corridor): L1 (572, 100), L2 (228, 352),
  L3 (684, 356), L4 (544, 532), plus A (home).
- Route library: 151 routes A → B, via points sampled in an ellipse, length ≤ 1.8 × the shortest, near-duplicates
  removed (15 m), planner clearance 3 m, fillet corners. 113 train routes, 33 test routes (held out), 1 benchmark
  (the shortest), 4 unused.
