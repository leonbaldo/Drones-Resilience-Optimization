# 1. The decision problem: what has to be optimised

This document states the online reconfiguration problem as precisely as the project can today. It is a starting
point for your formalisation, not a finished one. Where the project has a working choice, it is marked as such; where
the question is open, it says so. Tags in brackets (G23, D10, FC3, …) point to rows of
[reference/ASSUMPTIONS.md](reference/ASSUMPTIONS.md).

---

## 1.1 The setting

```
 OFFLINE (done, see docs 03 and 04)                     ONLINE (your part)
 ──────────────────────────────────                     ─────────────────────────────────────────────
 simulated flights DB_dev  ──►  health encoder E   ──►  z(t) every 0.1 s ─┐
                           ──►  forecaster F       ──►  z*(t | a) per     ├─►  choose action a at t_d
                                                        candidate a       │    subject to mission constraints
 white-box physics (allocation, motor, battery) ──►  load, SoC, headroom ─┘    → apply → keep flying → re-decide?
```

- **Vehicle.** Hexacopter, 6 motors M1…M6 on arms 60° apart, 0.30 m arm, take-off mass 1.6–2.2 kg, 4S LiPo
  6.2 Ah. Flown by an LQR tracker (no integral action) that outputs a wrench u = [T, Mx, My, Mz], mapped to six
  rotor-speed commands by an **allocation**. Each rotor command is limited to [200, 840] rad/s.
- **Mission.** Take off at A, climb to 15–40 m, follow a route (one of 151 alternatives over a real city map) at a
  scheduled speed of 1–8 m/s, hold 2 s over B, land at B. Requirements: arrive by the deadline (1.2 × nominal
  duration), keep the SoC ≥ 30 % at touchdown, stay within 1 m of the reference path, fly and land safely.
- **Degradation.** M1's viscous friction b grows (bearing wear): db/dt = λ·b·(ω₁/ω_ref)², with an unknown rate λ that
  switches on at an unknown onset. Higher friction means more current for the same speed, then lower speed, then the
  LQR raises M1's command until it saturates and the vehicle loses authority (see doc 04).
- **Disturbance.** Mean wind 0–6 m/s (direction 0–360°) plus turbulence. The mean wind is assumed forecast (known),
  the turbulence is not.

## 1.2 Time structure of the decision

```
 t = 0           onset t_on         detection        t_d (decision)                 T (touchdown)
 ├── healthy ───────┼── wear grows ──────┼─────────────────┼── action a applied ─────────────┤
                                         z(t) rises        F forecasts z*(t | a) to T
```

- **Today:** one decision per flight at t_d, chosen from a finite set (§1.3, "flown in CF"). The counterfactual set
  in the kit uses t_d = t_on + 0.35·(T − t_on).
- **Natural extension:** receding horizon (re-decide every Δ s), or a trigger rule (when to decide) plus the decision.
  When to act is part of the question: the project wants to compare a **reactive** policy (act when an alarm fires)
  with a **prognostic** one (act on the forecast). See §1.7, item 3.

## 1.3 Decision variables (the levers)

| Lever | Symbol | Domain | Status in the simulator | Effect |
|---|---|---|---|---|
| **Thrust pair shares** | s = (s₁₄, s₂₅, s₃₆) | simplex, s ≥ 0, Σ = 1, restricted to the hover-feasible region (each motor ω² ∈ [200², 840²] at hover for the mass); at 1.9 kg a pair can take 6.4–87.2 % of the thrust [G23] | implemented (alloc mode 1); **sampled per flight in DB_dev, not yet switchable in flight** | moves steady load between motor pairs; M1 and M4 always move together (§1.5) |
| **Moment weights** | w_M ∈ ℝ⁶₊ | log-uniform 0.25–8, normalised to geometric mean 1 | implemented (mode 1), fixed per flight | which motors produce the correcting moments; almost no effect on the data (Z17) |
| **Saturation strategy** | σ ∈ {S0 clip, S1 priority, S2 redistribute} | categorical | implemented (mode 1), fixed per flight | what happens when a command hits a limit; S1 can lose an under-powered vehicle |
| **M1 ceiling (derating)** | ω̄₁ ∈ [500, 840] rad/s | continuous | implemented (mode 1), fixed per flight | the allocator treats M1 as weaker; S1/S2 move work away before it saturates |
| **Legacy allocation weights** | w ∈ ℝ⁶₊, S = diag(1/√w), ω² = S·pinv(F·S)·u | e.g. A2 = (4, 1, 1, 1, 1, 1) | **switchable in flight** (0.5 s blend) | the in-flight "allocation" action of the CF set |
| **Payload drop** | p ∈ {0, 1} | release 0.15 kg (half the payload) | switchable in flight | lower thrust → lower load on every motor; mission value lost |
| **Speed schedule** | v(s) | 1–8 m/s, with the airspeed budget v_max + U_wind + 2σ_u ≤ 10 m/s [G20] | navigator supports it; never used as a decision yet | trades time (deadline) against power and wear rate |
| **Destination / route** | r ∈ {continue to B} ∪ {divert to A, L1, L2, L3, L4}; or another library route to B | finite; a divert = brake on the path, join leg, divert route, land [N11] | implemented | shorter exposure; a divert is a "safe abort" (mission not delivered) |
| **LQR weights** | 8 Q factors + 2 R factors | log-uniform ×0.1–×10 on the baseline (C1) | sampled per flight in DB_dev; switching in flight is implemented but has **never been an option** | tracking vs aggressiveness; Z17: only the position/altitude/attitude loops change the data |

The action space is mixed: discrete (destination, payload, saturation strategy) plus continuous (shares, ceiling,
speed, gains). The simplex constraint and the hover-feasibility region are known in closed form
([hexakit/whitebox/allocation.py](../hexakit/whitebox/allocation.py), `hover_shares_feasible`).

## 1.4 State, information and dynamics available at t_d

| Quantity | Symbol | Known online? | Source / model |
|---|---|---|---|
| Vehicle state (position, velocity, attitude, rates) | x | yes | measured |
| Battery SoC and RC voltage | SoC, V_d | estimated (particle filter, ±1–2 %) | battery model in `whitebox/battery.py` |
| Motor health | z_k(t), k = 1…6 | estimated, every 0.1 s, no friction sensor | encoder E (doc 03); 0 = healthy, larger = worse; concave in log friction |
| True friction | b_k(t) = d_k·2e-4 | **no** (ground truth in the data only) | – |
| Wear rate | λ | **no**: estimated from the z history | forecaster F (doc 04) |
| Mission remaining | route geometry, speed schedule, deadline, distance to B and to each site | yes | map, route library, navigator |
| Mean wind | U, direction | assumed forecast (no error) [W5] | – |
| Turbulence | – | no | – |
| Mass | m | yes | – |

**Dynamics a candidate rollout needs** (from cheapest to most faithful):

1. **Steady-state white-box (in this kit).** For a mass, friction vector, allocation, pack voltage and airspeed,
   `load_model.steady_state` solves the trim: rotor commands, actual speeds, PWM, motor and pack currents, and
   whether any command is at a limit. Piecewise-constant phases (climb, cruise at a given airspeed, hover, descent)
   give the current profile. This feeds the battery model (SoC) and the wear law (friction growth). Accuracy: pack
   current within 0.2 % of Simulink in steady phases; turns, gusts and transients are not modelled.
2. **Learned latent models (project).** The forecaster F gives z*(t | a) from the history and the planned motor load:
   dG(z)/dt = λ̂·h(z)·B(t; a, plan, wind). The decoder of the autoencoder maps (z, command, context) to the motor
   current, PWM and speed, from which a **capability index** follows (the remaining thrust and moment authority, an LP
   over the mixer matrix at full command; Spearman 0.96–0.97 with the true capability, no labels).
3. **Closed-loop Simulink (cluster).** Full 6-DOF, motor and battery dynamics, navigator and LQR, wind and
   turbulence. About 5 min of CPU per flight. This is the ground truth used to grade any policy, not something to
   call inside an online optimiser.

The project's missing block is exactly here: a closed-loop rollout per candidate that predicts tracking error,
arrival time, arrival SoC, health trajectory and actuator headroom (see the status figure, red blocks).

## 1.5 Structural facts that shape the problem

These are properties of the plant that a formulation should exploit or respect.

**(a) Opposite motors are coupled: M1 cannot be unloaded alone.** The mixer matrix F (4 × 6) maps squared rotor
speeds to the wrench, with b the thrust and d the drag coefficient:

```
        M1     M2       M3       M4     M5       M6
T   [   b      b        b        b      b        b     ]
Mx  [   0    √3/2·bl  √3/2·bl    0   −√3/2·bl −√3/2·bl ]
My  [ −bl   −bl/2     bl/2      bl    bl/2    −bl/2    ]
Mz  [   d     −d        d       −d      d       −d     ]
```

F has rank 4, so for a given wrench the allocator has a 2-D freedom (the null space of F). Every null-space
direction changes opposite motors identically: (M1, M4), (M2, M5), (M3, M6). Equivalently, with
Δ₁ = ω₁² − ω₄², Δ₂ = ω₂² − ω₅², Δ₃ = ω₃² − ω₆², the three moment equations involve only the Δ's and are non-singular:
balanced flight (zero moments) forces Δ = 0. The steady thrust distribution is therefore described by **three pair
shares summing to 1** (two numbers). The only ways to break the symmetry are to give up yaw (the plant has no yaw
damping, so the vehicle spins up without bound; excluded) or to move the centre of gravity (not in the plant).

**(b) Friction slows its own motor (a closed loop).** At a given PWM, a motor with more friction settles at a lower
speed and draws more current. The controller compensates, but in steady state M1's load share falls as it degrades:
u₁ / (6·s₁·ū) ≈ 1 − 0.5·z. Since the wear rate grows with speed squared, **wear slows itself down**:
health → load → wear → health. Ignoring this biases the predicted load by +16 % to +27 % (FC12). A forecast must
integrate this loop.

**(c) Derating moves the problem elsewhere.** Unloading the (M1, M4) pair loads M2, M3, M5, M6. With a large fault
and a heavy vehicle, the other motors (or M4 at its 200 rad/s floor) become the binding limit. Whether derating
helps or hurts depends on severity, mass, wind and the remaining route.

**(d) Saturation is not failure, but it is where failure starts.** In the data, divergences happen when commands
sit at 840 rad/s and an additional demand (a gust, a turn, the landing) cannot be met. Headroom is the explanatory
variable; the outcome comes from the dynamics.

**(e) The pass rate falls steeply with constant friction** (DB_dev v2, 1,164 flights, all configurations):

| Friction band | 1× | 1–1.5× | 1.5–2.3× | 2.3–3.5× | 3.5–5.3× | 5.3–8× |
|---|---|---|---|---|---|---|
| MISSION PASS | 89 % | 90 % | 78 % | 66 % | 44 % | 32 % |

Failures are mostly loss of control (F1 envelope/divergence, then a bad landing F7). SoC, corridor and deadline
failures are fewer.

**(f) Actions matter (counterfactual set, 30 families, wear reaching ~6× at landing under no action):**

| Branch from t_d | none | payload drop | allocation (4,1,1,1,1,1) | allocation + payload |
|---|---|---|---|---|
| MISSION PASS | 63 % | 70 % | 90 % | 93 % |
| M1 friction at landing (median) | 4.30× | 4.06× | 2.76× | 2.67× |

## 1.6 Constraints and objectives

### Mission requirements (pre-registered; how the data is graded)

| # | Requirement | Rule | Type |
|---|---|---|---|
| R1 | Deadline | arrival over B (within 1 m, for 2 s) by 1.2 × the nominal healthy duration of the route and speed schedule | hard |
| R2 | Battery reserve | SoC ≥ 30 % at touchdown + 5 s [G26] | hard |
| R3 | Corridor | flown path within 1 m of its own reference, whole flight | hard |
| R4 | Safe flight | F1 no envelope stop or divergence; F2 tilt; F3 attitude tracking; F4 angular rates; F5 altitude control | hard |
| R5 | No collision | F6a obstacle clearance (0.5 m body radius); F6b no ground contact away from the landing site | hard |
| R6 | Landing | F7 at contact: \|v_z\| ≤ 1 m/s, v_xy ≤ 0.5 m/s, tilt ≤ 15°, within 1 m of the site; stays landed | hard |
| – | Actuator limits | time at 840 / 200 rad/s, minimum headroom | **diagnostic only** (project rule) |

Outcome classes, from best to worst: MISSION PASS (B reached, all requirements met) > completed with requirement
violations > SAFE ABORT (diverted and landed safely: the vehicle is saved, the delivery is not) > loss of control.
Exact thresholds: [reference/FLIGHT_SAFETY.md](reference/FLIGHT_SAFETY.md).

### Candidate objective terms (the choice and the weighting are open)

| Term | Meaning | Available from |
|---|---|---|
| P(mission success) or P(any hard violation) | service delivered | outcome surrogate on the manifests; rollout |
| Δwear | friction (or z) added by landing: b(T)/b(t_d) or z(T) − z(t_d); remaining useful life after the mission | wear law + load model; forecaster F |
| Energy / arrival SoC | – | steady-state current + battery model |
| Tracking error | max or RMS cross-track | rollout (not in the white-box) |
| Control authority margin | minimum remaining thrust/moment authority along the plan | capability index; white-box headroom |
| Mission value | payload delivered, destination reached (B > home > site) | discrete |
| Action cost | number of actions, payload lost | discrete |

### Uncertainty

λ (the wear rate) is unknown, and it is estimated from a short, noisy z history. The onset may be recent. The
turbulence is unknown, and the encoder has errors and small leaks. Chance constraints (e.g. P(SoC_T < 30 %) ≤ δ) or
robust margins are natural. The project's existing battery prognosis already computes a reserve-breach risk with a
particle filter and an unscented failure-likelihood (UELF) per (action, route) pair.

## 1.7 A first formalisation (a proposal to criticise)

At decision time t_d, with information I_d = {x, SoC estimate, z history, mission, wind forecast}:

```
  minimise over a = (alloc, p, v(·), r) ∈ A(I_d)

      J(a) = E[ w_F·1{fail} + w_D·D(a) + w_E·E(a) + w_V·V_lost(a) ]

  subject to
      P( T_arr(a) > T_dl )            ≤ δ_1     (deadline, only if r = B)
      P( SoC_T(a) < 0.30 )            ≤ δ_2     (battery reserve)
      P( min_t H(t; a) < H_min )      ≤ δ_3     (authority margin, surrogate for R3–R6)
      a ∈ hover-feasible set; airspeed budget

  with predicted trajectories from
      health:   dG(z_k)/dt = λ̂_k · h(z_k) · B_k(t; a, plan, wind)            (F, per motor)
      load:     B_k(t) = s_k(a) · ū(t; m, v(·), wind) · ρ̂                       (white-box share × mean load)
      battery:  SoC' = −I_pack(t; a, z) / C,  I_pack from the steady state or the decoder
      headroom: H(t) = capability(z(t), a, V_pack(t))                            (LP over F at full command)
```

D is the damage (e.g. log b(T) or z(T)), E the energy, V_lost the mission value lost (payload, destination), and
λ̂ a distribution, not a point.

Points that are deliberately left open:
1. **One-shot vs receding horizon.** A single decision per flight matches the data. A re-decision every 10–30 s
   matches how the forecast improves after the onset (z error at landing 0.15 at 30 s after onset, 0.03 at 240 s).
2. **Lexicographic vs weighted.** The current placeholder policy (`decide_joint` in the project) is lexicographic:
   feasible = SoC risk < δ; prefer delivering to B, then keeping the payload, then fewer actions, then lower risk;
   otherwise home, then the cheapest site.
3. **Reactive vs prognostic.** The paper needs a comparison: a reactive rule (act when z crosses a threshold) against
   the prognostic optimiser. If derating always helps, both choose the same action and foresight is worth nothing.
   Scenarios where an action **hurts** are needed: mild faults where derating costs tracking, gusty legs, turns
   needing authority from the loaded motors, or wider action sets (shares, speed, gains).
4. **Health in z or in friction?** z is label-free but nonlinear (concave in log d) and depends a little on the
   operating point. A monotone map z → d can be fitted on validation flights (isotonic; weak supervision).
5. **Multi-motor faults.** Encoders with one z per motor exist (100 % detection and isolation at 2× and 4×), and a
   dataset with 2–3 motors wearing at once is being flown now. The pair coupling makes multi-motor allocation more
   interesting: e.g. M1 and M4 both worn.

## 1.8 Suggested pilot problems (in increasing order of difficulty)

| # | Problem | Decision | Model | Data to check against | Notebook |
|---|---|---|---|---|---|
| P1 | **Static allocation design.** For mass m, friction vector d, pack voltage V and airspeed, choose the pair shares (and ceiling) that minimise M1's wear rate b₁·(ω₁/ω_ref)² subject to every motor's command ≤ 840 − margin and ≥ 200 + margin, optionally penalising pack current. | s ∈ simplex | white-box trim | DB_dev v2 diagnostics (`diag:` columns) | 02, part A |
| P2 | **Mission-level option ranking.** At t_d of a CF family, rank {none, allocation, payload, both} × {B, A, L1…L4} by predicted damage, arrival SoC and headroom, by integrating the wear law and the battery along the remaining route with steady-state phases. Compare with the 4 flown branches. | discrete | white-box + wear law + battery | `counterfactual_cf` (12 flights in kit; 120 in the full set) | 02, part B |
| P3 | **Chance constraint from data.** Learn P(PASS \| friction or z, mass, wind, shares, speed, …) from the manifests (2,523 flights) and use it as a constraint in P1/P2. | – | classifier | `data/manifests` | – |
| P4 | **Continuous reconfiguration at t_d.** Shares + ceiling + speed + destination, with λ uncertainty, in a receding horizon. | mixed | P2 + F | needs new simulated flights (we fly them) | – |
| P5 | **Reactive vs prognostic policy comparison** in closed loop. | policy | full Simulink | flown on request | – |

## 1.9 Questions we would like your view on

1. A suitable class: stochastic MPC, scenario-based or chance-constrained MINLP, POMDP, or a simpler
   enumerate-and-rank over a small action set?
2. How to handle the mixed action space with expensive, non-smooth outcome models (loss of control is a cliff)?
3. How to propagate the uncertainty of λ̂ and of the encoder into the constraints without a full Monte Carlo of the
   simulator?
4. Which surrogate fidelity is enough for ranking? (For F, ranking correctness, 93 % best-branch hits, mattered
   more than accuracy.)
5. What to optimise offline (e.g. a policy or a lookup table over (z, λ̂, remaining distance, mass, wind)) versus online.
6. How to design the next simulated dataset so that it supports the optimiser (in-flight reconfigurations, physical λ
   per motor, scenarios where actions hurt)?
