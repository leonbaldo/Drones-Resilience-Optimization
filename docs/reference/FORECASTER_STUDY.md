# Latent health forecaster F: options, benchmark, conclusion

**Status:** 2026-11-19. Every value and setting is in ASSUMPTIONS section **FC** (FC1–FC14).
**Code:** `resilience/forecaster.py` (traces, models, benchmark, summary), `resilience/forecast_cf.py` (counterfactual
flights), `resilience/forecast_report.py` (figures). **Notebook:** `forecaster_study.ipynb`.
**Jobs:** `cluster/run_forecast_encode.sbatch`, `run_forecast_cf.sbatch`, `run_forecaster.sbatch`,
`run_forecaster_learned.sbatch`, `run_forecaster_final.sh`.
**Outputs:** `results/forecast/` (traces, `bench_<encoder>/scores_*.csv`, `summary_*.csv`, `figs/`).

## 1. The question

At a decision time t0 the online loop has the latent health history z(≤ t0) from the encoder. For each candidate
reconfiguration C* (allocation, payload, speed, route…) it needs **z*(t) until landing**, to judge the mission and the
damage. F must therefore answer a **counterfactual** question: what will z do *if* we fly C* from now on. Accuracy on
recorded flights is necessary, but **ranking the candidates correctly is what F is for**.

## 2. What the data and the simulator say (checked before designing F)

1. **The wear law is exact and known.** On v3, the realised wear rate equals the manifest rate (ratio 1.000), and log d is
   linear in M1's exposure E = ∫(ω₁/ω_ref)² dt (r > 0.9998). Only λ (the motor's wear rate) is unknown, and in reality
   it is a per-motor unknown too, so F must **estimate λ online from the history**.
2. **z is concave in log d** for all encoders (it flattens above ~3×).
3. **Friction slows its own motor** (FC12): M1's load share falls as it degrades, u₁/(6·s₁·ū) ≈ 1 − 0.5·z (R3). Wear
   therefore slows itself down: health → load → wear → health is a **closed loop**. Ignoring it biases the predicted load
   by +16 % (v3) / +27 % (evalfault_v2). This is where an **ODE solver** is genuinely needed: the forecast integrates
   this loop forward.
4. **DB_dev v3 cannot teach the effect of a reconfiguration.** Its wear rate is solved per flight so that friction reaches
   a sampled end value independent of the configuration (`dataset.doubling_for_target`), and no flight changes its
   configuration. A model trained on v3 sees no link between configuration and damage. → I flew a **counterfactual set**,
   `evalfault_cf` (FC3): 30 held-out families × 4 branches, bit-identical until t_dec, then no action / D11 allocation /
   0.15 kg payload drop / both, with one physical wear rate per family. Friction at landing: ~4.3× (none), ~4.1×
   (payload), ~2.75× (allocation), ~2.67× (both); mission FAIL 11, 9, 3 and 2 of 30.

## 3. Options evaluated

| Family | Model | Idea |
|---|---|---|
| Baselines | P0 persistence, P1 linear in time | no physics, no plan |
| Grey-box D10 ODE (closed form) | G1 linear in exposure; G3 hinge (unknown onset found by grid search); **G5 = G3 + BIC change-point test**; G2/G4 the same in calibrated log d (weak supervision) | z (or log d) linear in the motor's exposure; λ fitted on the history; the future exposure comes from the plan through the load model, integrated in closed loop |
| Black-box learned | L1 GRU seq2seq; L2 latent neural ODE (RK4) | learn everything from v3; input ablation I0 history only, I1 + configuration and forecast wind, I2 + planned profile, I3 + planned motor load, I4 + true future load |
| Physics-structured learned | **L3 structured neural ODE** dz/dt = λ(history)·s(z)·u(t) ≥ 0 | D10 in latent form: a network infers λ from the history, s(z) is learned, the load is a driver |

**What goes into F (the user's question).** The configuration, the plan, the mass and the assumed wind do *not* go in as
raw features. They enter through the **motor's future load** B(t) = ρ̂ · 6·s_k(C*) · ū(t):
- s_k(C*) is the motor's share from the white-box allocation;
- ū(t) is a learned mean-load model of the planned profile Ω*, the planned mass and the **forecast wind** (manifest mean
  wind, W5);
- ρ̂ is an online calibration on the last 180 s, which absorbs per-flight wind and moment errors.

**The ODE:** dG(z)/dt = λ̂ · h(z) · B(t), solved forward in time.

## 4. Benchmark (encoder R3; one seed; numbers from `results/forecast/bench_R3/summary_*.csv`)

Accuracy = |z error| at landing on fault flights, evalfault_v2 (held-out routes, 821 decisions). False growth =
predicted z rise on constant-friction v3 val flights (true +0.004). CF = evalfault_cf. Benefit = z_end(none) −
z_end(allocation), true **0.35**.

| Model | Accuracy | False growth | CF pair order | CF best branch | Predicted allocation benefit |
|---|---|---|---|---|---|
| P0 persistence | 0.336 | 0.000 | 0.00 | 0.00 | 0.00 |
| P1 linear in time | 0.123 | 0.024 | 0.52 | 0.27 | 0.00 |
| G3 grey-box hinge, predicted load | 0.084 | 0.053 | 0.98 | 0.93 | 0.21 |
| **G5 grey-box hinge + BIC, predicted load** | **0.085** | **0.037** | **0.98** | **0.93** | **0.21** |
| G4 hinge in log d (weak supervision) | 0.115 | 0.016 | 0.91 | 0.93 | 0.19 |
| G3 with the true future load (leaky upper bound) | 0.087 | 0.056 | 0.99 | 1.00 | 0.25 |
| L1 seq2seq + configuration + profile (I2) | **0.065** | 0.112 | 0.59 | 0.43 | 0.00 |
| L1 seq2seq + planned load (I3) | 0.068 | 0.129 | 0.36 | 0.17 | −0.01 |
| L2 neural ODE + configuration + profile (I2) | 0.083 | 0.121 | 0.43 | 0.17 | 0.00 |
| L2 neural ODE + true future load (I4) | 0.082 | 0.047 | 0.01 | 0.00 | **−0.30** |
| **L3 structured neural ODE, planned load** | **0.071** | 0.096 | **0.99** | **0.93** | 0.17 |
| L3 structured neural ODE, true load | 0.082 | 0.105 | 1.00 | 1.00 | 0.18 |

Error by decision time after onset (evalfault_v2): 30 s 0.15 (G5) / 0.12 (L3) / 0.09 (L1), 240 s 0.03 for all three.
Figures: `results/forecast/figs/accuracy_R3.png`, `counterfactual_R3.png`, `load_feedback_R3.png`.

**Six motors** (FC14; one motor-symmetric F applied to each z_k with that motor's own load):
- **E1 (symmetric encoder), G5:** M1 accuracy 0.102. CF: 0.98 pair order, 0.97 best branch, predicted allocation
  benefit 0.24 vs a true 0.28 (in E1's z scale). Healthy motors: predicted rise +0.05 vs a true +0.03–0.04. The healthy
  z_k really do drift on M1-fault flights (encoder cross-talk, M4 largest), and F follows that drift.
- **D0, G5:** M1 accuracy 0.21. D0's z saturates and is noisy when healthy (sd 0.054), so the hinge fits noise. The log d
  variant is better (0.13).

## 5. Conclusions

1. **Black-box forecasters are ruled out for the decision loop, whatever their inputs.**
   - Seq2seq and the free neural ODE are the most accurate on recorded flights (0.065).
   - On the counterfactual set they predict **no benefit** from the allocation that halves M1's wear (0.00 vs 0.35), and
     they choose the best action in 3–43 % of families.
   - Given the true future load, the free neural ODE even predicts the allocation **hurts**: on v3, low load goes with high
     friction (the feedback of §2.3), so it learned the dependence backwards.
   - Part of their accuracy is a learned dataset prior: they predict a +0.11 rise on healthy flights, because two-thirds
     of v3 flights have faults that follow the same design rule as evalfault_v2.
2. **Physics structure is what makes F rank candidates correctly.**
   - Every model built on D10 (grey-box G3/G5, structured neural ODE L3) orders 98–100 % of branch pairs correctly and
     picks the best branch in 93–100 % of families.
   - It does so from history that is identical across the branches, so the ranking comes from the candidate's load alone.
3. **Recommended F = latent D10 ODE with a closed-loop load model**, dG(z)/dt = λ̂·h(z)·B(t; C*, Ω*, wind), solved
   forward in time.
   - **G5** (closed form): the prior-free, interpretable reference. It needs no training beyond h(z) and ū, and is not
     fooled by the dataset prior.
   - **L3** (structured neural ODE): the learned upgrade. It is better on recorded faults (0.071 vs 0.085), but carries
     the v3 prior (false growth 0.096).
   - Use G5 now; use L3 once it is trained on data with a realistic fault prevalence.
4. **Inputs:** the plan matters only through the motor load.
   - Configuration: via the white-box allocation share. This is what makes "derate M1" work.
   - Mass and payload drop: via the load model.
   - Profile: via the load model.
   - Assumed wind: the forecast mean wind in the load model; per-flight deviations are absorbed by ρ̂.
   - Feeding the configuration or the profile directly to a black box gave nothing measurable.
5. **One F for all motors.** A motor-symmetric F applied per motor works with E1 (CF ranking as good as with R3). A
   joint six-z model can't be trained anyway: only M1 ever degrades in v3.
6. **The remaining error is mostly in the load under the candidate.**
   - The allocation branch's M1 load is over-predicted by +19 %, so its benefit is under-predicted (0.21 vs 0.35).
   - Cause: the share uses pure thrust, but the D11 weights also re-allocate the moments.
   - With no reconfiguration, the load error is 1–2 %.

## 6. How to proceed (proposed order)

1. **Load share under C\*:** re-allocate the *recorded* wrench (ω_cmd history → FORCES → `allocate` under C*), instead of
   pure thrust. Target: close the +19 % gap on the allocation branches. Small change; needs ω_cmd in the traces.
2. **Uncertainty:** turn λ̂ into a distribution (the regression's posterior, or an L3 ensemble). Push it through the ODE
   to get a predictive band on z*(t), which the constrained optimisation needs (chance constraints).
3. **Couple F to the rollout:** z*(t) → plant decoder (D_nom + z·D_res) → predicted motor commands and headroom →
   feasibility and outcome. In evalfault_cf the action changes the outcome (11 → 3 FAILs), so the set can score the full
   loop end to end.
4. **Data for the learned parts:** for the next DB_dev, set a **physical λ per motor** (not solved to a target), include
   **in-flight reconfigurations** at decision times (the evalfault_cf design), and use a realistic fault prevalence.
   Then L3 (or a learned load model with the configuration) can learn configuration → wear without the v3 confound.
5. **Robustness:** decision times triggered by the H1 detector instead of the true onset; several CF decision times and
   wear rates; mode-1 actions (needs an in-flight switch for mode 1 in the allocation block); 3 seeds for L3.

## 7. Limitations

- One seed for the learned models.
- Decision times are placed relative to the true onset.
- evalfault_v2 shares v3's design rule, which favours models that learned the v3 prior.
- The CF set has one decision time, one wear rate and legacy-weight allocation only.
- The forecast is of z, the encoder's output. Encoder errors (e.g. E1's cross-talk) pass through unchanged.

## 8. Addendum (same day): judged in friction, F works; the remaining gap is the encoder

Mapping F's forecast z through the v3 z→log d calibration (R3, G5, evalfault_cf, median friction at landing):

| Branch | At decision | True at landing | Forecast at landing |
|---|---|---|---|
| no action | 1.8 | 4.30 | 4.49 |
| payload drop | 1.8 | 4.06 | 4.28 |
| allocation | 1.8 | 2.76 | 3.06 |
| allocation + payload | 1.8 | 2.67 | 2.99 |

In friction space, F orders 97 % of the branch pairs correctly (E1: 98 %) and picks the best action in 90 % of
families (E1: 93 %). The large z-space error on the allocation branches (§4: predicted benefit 0.21 vs 0.35) is mostly
the **encoder**:
- Within 20 s of switching to the allocation, R3's z falls by 0.027 (D0: 0.023) while friction keeps growing (+5 %).
  With no action, z rises by 0.038 in the same window.
- So R3 and D0 read part of M1's load as health. E1 is almost unaffected (+0.005).
- The rest of the gap is the load share under the allocation: measured ratio 0.44, pure-thrust prediction 0.50.
- The friction forecast also starts ~10 % high at the decision: an offset of the z→d calibration.

Figures: `results/forecast/figs/rolling_evalfault_v2_R3.png` (forecasts from several decision times) and
`configurations_R3.png` (same history, four configurations). Both are in the notebook.

Added to §6: make the encoder invariant to the allocation (an anchor or consistency term across allocations, or E1),
because F can only be as configuration-correct as the z it is given.

## 9. Which forecaster is best, and the three fixes (2026-11-19 evening; FC16–FC20)

| Forecaster | A error on recorded faults | B false rise, healthy | C picks best action | D orders actions | E predicted allocation benefit (true 0.35) |
|---|---|---|---|---|---|
| linear in time | 0.123 | 0.024 | 27 % | 52 % | 0.00 |
| GRU seq2seq (black box) | **0.065** | 0.112 | 43 % | 59 % | 0.00 |
| neural ODE (black box) | 0.082 | 0.121 | 17 % | 43 % | 0.00 |
| structured neural ODE | 0.071 | 0.096 | **93 %** | **99 %** | 0.17 |
| **grey-box G5** | 0.085 | **0.037** | **93 %** | 98 % | **0.23** |

The GRU wins only column A, and there its forecast is the same for every candidate configuration (E = 0.00): it answers
"how does this flight continue" and cannot answer "what if we change the configuration", which is F's job. The two
physics-structured models answer it. G5 is chosen: same ranking as the structured ODE, 2.6× fewer false rises, larger
(closer) benefit, no training, interpretable; the structured ODE is the learned alternative once trained on data with
a realistic fault prevalence.

Fixes: (1) equilibrium load share FC17 (allocation-branch load bias 19 → 9–13 %); (2) feeding the encoder the true
configuration does not help (FC16); the cause is that z scales with the motor's speed, fixed by the operating-point
normalisation FC19 (allocation forecast 2.91× vs 2.76× true, was 3.06×; ranking 93 % / 98 %); (3) predictive band
FC18 (90 % coverage 0.93 on v2, 0.89 on CF with R3n).
