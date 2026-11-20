# Motor-health autoencoders: every architecture tried so far

*Recap, 2026-11-19. Tags in brackets (Z1, Z30, …) point to ASSUMPTIONS.md, where every setting and result is recorded.
Numbers are means over 2 seeds unless stated. "Spearman" = rank correlation between a flight's median z and its true
M1 friction d, on held-out flights. "Leak" = how much z depends on mass or wind instead of friction.*

## The task in one paragraph

From ordinary onboard data (motor currents, PWM, commanded and actual rotor speeds, pack voltage, vehicle motion),
learn a number z that tracks the friction of motor M1, **without ever seeing the friction**. The only label allowed is
a per-flight "healthy" flag. Data: simulated hexacopter flights. DB_dev v1 is 1,002 flights; v2 is 1,164 flights with
held-out test routes; v3 (999 flights) has friction growing during the flight.

## At a glance

| # | Family | Architecture | Best result | Main problem | Status |
|---|---|---|---|---|---|
| 1 | Generic CVAE | FiLM VAE rebuilding all 40 measurements (the user's guide model) | Spearman 0.90 | z's sign arbitrary, mass leak, best at epoch 1 | superseded |
| 2 | Generic CVAE | + healthy anchor D_nom(W) + z·D_res, encoder sees W | sign fixed in 12/12 runs | mass leak, wind leak | superseded |
| 3 | Generic CVAE | + minimal configuration set, lr 1e-4, onboard wind estimate in W | val error −22 % | best epoch still 1 | superseded |
| 4 | Generic CVAE | Same model on DB_dev v2 | reconstruction 0.094, Spearman 0.93–0.95 | best epoch 1, mass leak 0.3 | superseded |
| 5 | Temporal encoders | TCN, state-space, transformer on top of #4 | none better than a 10 s median of z | same decoder limits | parked |
| 6 | Decomposition study | VAE vs AE, split vs single decoder, no latent, motion as conditioning | identified the decoder as the bottleneck | – | done |
| 7 | **Per-motor plant decoder** | each motor predicted from its own command; z acts on M1 only | motor error **0.001** (20× lower), Spearman **0.994** | early peak at epoch 1 | **current model** |
| 8 | Plant decoder, losses | NLL vs β-NLL vs MSE | NLL kept; leaks 0.03 / 0.02 | early peak remained | done |
| 9 | Plant decoder, no dropout | dropout 0 (+ deterministic latent) | error **0.0004**, no early peak | v1 test slightly worse | **next base** |
| 10 | Six-latent plant (any motor) | one z per motor, shared encoder | 100 % detection/isolation at 2× and 4× | pair-partner alarms; mild faults | parallel line |
| 11 | Six-latent, symmetric | no knowledge of which motor is faulty | as #10, nothing lost | unstable without dropout | parallel line, running |
| 12 | Structure, physics, biases | ladder of structural fixes, physics residual, consistency losses | – | – | **running (Z50)** |

---

## 1. The generic CVAE line (Z1–Z27)

**#1. The guide model (Z1, Z10; 2026-11-16).** The user's FiLM univariate VAE, ported unchanged.
- **Encoder:** X (40) → Dense 256 → 2 × [Dense 256 + Dropout 0.1] → Gaussian q(z | X), z scalar.
- **Decoder:** z → Dense 256 → 2 × [Dense 256 + FiLM(W) + Dropout] → Dense 245 → Gaussian over all 40 X.
- **W:** operating point and the 24 configuration columns.
- **Results:** z orders friction on unseen families (Spearman 0.90–0.91, within-family τ 0.96–0.99) and follows an
  in-flight fault (post-onset Spearman ≈ 0.99).
- **Problems:** z's sign is arbitrary (5 of 8 runs negative), z leaks mass, and the model is best after epoch 1.

**#2. Healthy anchor + encoder sees W (Z11–Z15).**
- **Anchor:** the decoder mean becomes D_nom(W) + z·D_res(z, W). D_nom learns only from healthy flights, and a loss
  term pulls z to 0 on healthy flights.
- **Encoder:** sees [X, W].
- **Results:** the sign is fixed in 12/12 runs, and healthy z is ≈ 0.05. Seeing W halves the mass leak but raises the
  wind leak.

**#3. Configuration, learning rate, wind (Z16–Z26).**
- **Configuration:** a minimal set in W. The "configuration fingerprint" hypothesis was rejected.
- **Learning rate:** 1e-4 instead of 1e-3, which fixed an instability.
- **Wind: the biggest lever.** An onboard wind estimate from the accelerometer, with realistic drag and sensor errors:
  validation error −22 %, and learning continues to epoch ~7.
- **No help:** training only on calm air.
- **Diagnosis:** what is learned after epoch 1 is specific to the training missions (gusts, turbulence).

**#4. The same model on DB_dev v2 (Z27).** v2 has 3× more training missions, its own turbulence per flight, and 30
test families on held-out routes.
- **Results:** reconstruction 0.108 → 0.094, Spearman 0.93–0.95, wind leak 0.13 → 0.03–0.05.
- **Unchanged:** mass leak 0.28–0.35, and still best at epoch 1.
- **Z29 found why:** 58 % of the validation error is in the vehicle-motion channels, which z cannot explain (they are
  gust-driven). The selection metric was measuring the wrong thing.

## 2. Temporal encoders (Z28, Z34, Z35 T1P/T6S)

The idea: give the encoder a window of past data so gusts average out. All are causal, with 50 Hz motor channels and
10 Hz everything else. Trained on v1 + v2, with the #4 decoder.

| Model | v2 test Spearman | z noise | Mass leak |
|---|---|---|---|
| Pointwise #4 | 0.934 | 0.097 | 0.35 |
| + causal 10 s median of z (no training) | 0.934 | **0.086** | 0.35 |
| TCN, 2 / 5 / 10 s | 0.917–0.921 | 0.096 | 0.40 |
| State-space latent (GRU filter, random-walk prior) | **0.946** | 0.088 | **0.29** |
| Causal transformer, 5 s | 0.921 | 0.093 | 0.39 |

No learned temporal encoder beats the free median filter by much. All peak at epoch 1–2, so the problem was the decoder,
not the encoder.

With the plant decoders (Z35):
- **T1P** (state-space encoder + #7): no gain.
- **T6S** (per-motor state-space encoder + #11): better at detecting mild faults (+16 points at band 1), slightly worse
  ranking.

## 3. Finding the bottleneck: the decomposition study (Z30, Z31)

On v2, one change per configuration, all selected on the error of the 18 motor channels:

| Change | Finding |
|---|---|
| VAE vs deterministic AE | **identical**: the variational part does nothing |
| Split decoder (D_nom + z·D_res) vs single FiLM decoder | the split **causes the mass leak** (D_nom sees only healthy flights, z absorbs its mass error), but not the stall |
| No latent at all (W → X regression) | val motor error 0.14; z explains ~82 % of the motor error |
| Vehicle motion + pack voltage moved from targets into conditioning (A1) | val error 0.025 → **0.0135**, still improving to epoch ~13 |
| Student-t likelihood, no upper wall on z, shared embedding | small or no effect |
| **Per-motor plant decoder (A3)** | **0.022 → 0.001**, Spearman 0.994: the decisive change |

**Why the plant decoder works.** Friction is an extra torque proportional to the rotor speed. A decoder that doesn't see
each motor's command must guess the speed from W, and that miss grows with friction.

## 4. The per-motor plant decoder (Z30 A3, Z32, Z36): the current model (#7–#9)

![Per-motor plant decoder](results/cvae/doc/cae_motor_health_architecture.png)

*Full description: MOTOR_HEALTH_CAE.md. Figure with every layer: results/cvae/doc/cvae_plant_architecture.pdf.*

- **φ:** a shared embedding of [W, vehicle motion, pack voltage] (128).
- **Encoder:** [X, φ] → z.
- **g_nom:** a healthy-motor network, the same weights for all six motors. Input [φ, motor identity, that motor's
  command]; output current, PWM and speed with their σ.
- **g_res:** a wear network for M1 only. M1's mean = g_nom + z·r.
- **Training:** M1's copy of g_nom learns from healthy flights only.
- **Size:** 421 k parameters.

| Version | Change | Best val motor error | End / best | v2 test Spearman | Leak per step (mass / wind) |
|---|---|---|---|---|---|
| Z32 P | Gaussian NLL, dropout 0.1 | 0.0012 | 3.9 (early peak) | 0.994 | 0.023 / 0.021 |
| Z32 PB | β-NLL 0.5 | similar | 2.3 | 0.989–0.995 | ~2× P |
| Z32 PM | MSE (σ = 1) | diverges (with the VAE's KL) | – | – | – |
| Z36 R1 | deterministic latent, dropout 0.1 | 0.0013 | 3.6 | 0.993 | 0.016 / 0.004 |
| Z36 R2 | dropout 0 | **0.0004** | **1.04** | 0.991 | 0.008 / 0.004 |
| **Z36 R3** | **dropout 0 + deterministic latent** | **0.0004** | **1.03** | 0.991 | 0.010 / 0.006 |
| Z36 R4 | R3 + MSE | 0.0005 | 1.04 | 0.979 | 0.004 / 0.048 (v1 test: 0.18 / 0.12) |

- **Dropout was the cause of the early peak.** z sampling and σ trading were not needed to explain it.
- **R3 is the new base.** It is slightly weaker than P on the older v1 test flights (Spearman 0.989 vs 0.994; leaks up
  to 0.044 vs 0.028).
- **Known flaw, found in review:** M1's σ correction is not multiplied by z, so z can also act through σ. It is being
  fixed in Z50.

## 5. Faults on any motor: six-latent plant models (Z33, Z35, Z37, Z38)

A parallel line that extends #7 from "health of M1" to "health of each motor".

| Version | Architecture | Key results |
|---|---|---|
| Z33 P6 | one z per motor; the encoder is shared over the motors with no motor identity (input: that motor's channels, its residual against g_nom, φ); z₂…z₆ anchored to 0 | M2–M4 faults detected and isolated 100 % at ~1.9× and ~4.3×, on the same z scale as M1; M1 Spearman 0.98; the pair partner (the opposite motor, coupled by the geometry) false-alarms 23 % at band 4 |
| Z35 S | **motor-symmetric**: nothing in the model or the loss says which motor is faulty; every z_k anchored to 0 on healthy flights only | as good as P6: detection and isolation 100 % for M1–M6 at bands 2 and 4 (M5, M6 never seen faulty in training); mild faults (band 1) only 33–53 % |
| Z35 bottlenecks | a single health number through a trained 6 → 1 → 6 bottleneck (linear, MLP, max, soft max) | all fail or are unstable; a post-hoc max_k z_k loses ≤ 0.01 instead |
| Z35 ablations | β-NLL, lr 3e-5, pair-partner residual, gradient clip, AE, dropout 0 | β-NLL = best detector; lr 3e-5 = best ranker; pair residual halves pair-partner alarms; **dropout 0 diverges on S** |
| **Z37 D0** | non-symmetric six-latent (P6) + dropout 0 | **stable**, test error 0.0002 (6.5× below S), max-z Spearman 0.983, AUROC 0.988, band-1 detection ~77 %. Uses the "M1 is the faulty one" knowledge in the loss |
| Z38 E1–E6 | symmetric S with small, encoder-only, decoder-only or warm-up dropout | **running**: searching for a symmetric model as stable as D0 |

## 6. Running now: structure, physics and inductive biases (Z50)

On the single-latent plant model (base Z36 R3), after two external reviews (spec: EARLY_PEAK_Z50_SPEC.md; code:
`resilience/cvae_z50.py`).

- **Stage I (running): structural ladder, one change per rung.**
  - L1: z removed from g_res's inputs, and no σ route for z;
  - L2: decoder context from exogenous inputs only (W + pack voltage); L2s: W only; L2f: a FiLM version;
  - L3: unit-length r, so z alone carries the size of the deviation;
  - L4: two-stage training (healthy plant first, then frozen);
  - L5: the σ correction back, gated by z.
- **Stage II (next): physics and biases.**
  - Physics: a residual shaped by the simulator's motor law (speed down, current up in proportion to ω, PWM unchanged)
    with 2 learned constants; a version with a small learned correction; a grey-box reference.
  - Biases: sign constraints; within-flight consistency of z; independence from mass and wind.
  - Consistency loss across missions at equal friction, reported as weak supervision.

## 7. What we have learned

1. **The decoder's structure matters more than the encoder's.** Moving from a generic decoder to a per-motor plant
   decoder gave 20×. Temporal encoders, likelihoods and latent types gave a few percent at most.
2. **The yardstick matters.** Selecting on all 40 measurements kept the epoch-1 model. Selecting on the motor channels
   and splitting the error by channel and by flight type exposed the real causes.
3. **Dropout, not over-fitting, caused the early peak** in the plant models. Without it, training improves for 15
   epochs and the error drops 3×. It can make the symmetric six-latent model unstable, though.
4. **The variational part adds nothing so far.** A deterministic latent is as good, and σ_z is not a calibrated
   uncertainty.
5. **Identifiability needs structure.**
   - The anchor fixes z's zero and sign.
   - The healthy-only training rule keeps wear out of g_nom.
   - Leaks come from the decoder's freedoms (the split decoder, φ, the z·r scale), which Z50 addresses.
6. **Supervision has to be stated.** Everything above uses only the healthy flag (plus, in #7, the knowledge that M1 is
   the monitored motor). Any variant that uses the designed friction level is reported as weak supervision.

## Which model to use now

| Question | Model |
|---|---|
| Health of M1, best available | **Z36 R3**: per-motor plant decoder, deterministic latent, dropout 0, Gaussian NLL |
| Fault on any motor, M1 knowledge allowed | **Z37 D0**: six-latent, non-symmetric, dropout 0 |
| Fault on any motor, no fault-location knowledge | **Z35 S** for now (dropout 0.1); Z38 may replace it |
| Online smoothing | causal 10 s median of z |
