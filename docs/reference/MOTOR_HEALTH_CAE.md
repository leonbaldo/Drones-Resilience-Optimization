# The motor-health autoencoder with a per-motor plant decoder, explained simply

**Status:** 2026-11-18. This describes the model run in ASSUMPTIONS Z30 (variant A3) and Z32: `python -m resilience.cvae train
--decoder plant --targets motor ...`; the code is in `resilience/cvae.py` (`_torch_model`, `_loss`, `train`).
**Figures:** `results/cvae/doc/cae_motor_health_architecture.png` (overview) and
`results/cvae/doc/cvae_plant_architecture.pdf` (every layer and dimension, generated from the code).
The earlier model it replaces (Z27, one FiLM decoder for all of X) is described in [CVAE_ARCHITECTURE.md](CVAE_ARCHITECTURE.md).

---

## 1. The idea in five sentences

1. Motor M1 slowly gets more friction. We never measure that friction on the real vehicle; we want a number z that
   tracks it.
2. We train a network that must **rebuild what each motor did** (its current, PWM and speed) from **what it was asked to
   do** (its speed command) and from the flight conditions.
3. The network has a fixed model of a **healthy** motor. The only way it can explain a motor that needs more current
   than a healthy one is through z, and z is only allowed to act on M1.
4. So z becomes "how far M1 is from healthy": 0 when healthy, larger as friction grows. No friction label is ever used.
5. Online, z is read every 0.1 s from the onboard data and smoothed over a few seconds.

## 2. Why "plant" decoder

In the simulator, each rotor is driven like this (ASSUMPTIONS L1):

```
speed command ω_cmd,k  →  PWM map (open loop, affine)  →  PWM_k  →  DC motor k + propeller  →  current I_k, speed ω_k
                          PWM = 0.0008842·ω_cmd + 0.034                 ↑
                                                         friction torque D·ω  (viscous, M1 only)
```

There is **no speed loop inside the ESC**: PWM follows the command through a fixed map. Speed is closed by the flight
controller (LQR + allocation), which raises the commands when the vehicle drifts.

Friction is an extra braking torque on M1. At a given PWM, the motor then settles at a **lower speed** and draws a
**higher current**. Its electrical time constant is a few ms, so at 10 Hz this is almost a static shift, not a lag. The
flight controller notices the missing thrust and raises M1's command, so PWM rises too, but only through the command.
The decoder copies this structure:

- each motor is predicted from **its own command**, the way the real motor only "sees" its own drive signal;
- all six motors use **the same small network**, because they are the same hardware;
- the wear term is added **only on M1**.

No physics equation is written into the network. Both parts are learned from data. What is "physical" is the
**wiring**: which network receives which signal. That wiring is what made the difference (§8).

## 3. The data one sample at a time

One training sample is **one row**: one 10 Hz moment of one airborne flight (the 50 Hz log is thinned by 5; ground
samples are dropped). Every column is standardised with the mean and standard deviation of the training flights and
clipped to ±10 σ.

### X: what the drone measures onboard (40 columns)

| Group | Columns | Symbol | Role in this model |
|---|---|---|---|
| Motor currents | 6 | I₁…I₆ | **target** (rebuilt by the decoder) |
| PWM duty | 6 | PWM₁…PWM₆ | **target** |
| Actual rotor speeds (ESC) | 6 | ω₁…ω₆ | **target** |
| Commanded rotor speeds | 6 | ω_cmd,1…6 | **plant input**: motor k's decoder gets ω_cmd,k |
| Pack voltage | 1 | V_pack | **driver** (conditioning) |
| Roll, pitch | 2 | φ, θ | driver |
| Body rates | 3 | p, q, r | driver |
| Accelerometer | 3 | a_b | driver |
| Inertial velocity | 3 | v | driver |
| Tracking error | 3 | e | driver |
| Pack current | 1 | I_pack | **encoder only** |

All 40 columns go into the encoder. The decoder rebuilds only the 18 motor responses (current, PWM, speed). The vehicle
motion and the pack voltage are **given** to the decoder as context instead of being rebuilt: they are measured, and
they carry almost no information about M1's health (Z29).

### W: known conditions (23 columns)

| Group | Columns | Content |
|---|---|---|
| Operating point | 13 | commanded cruise speed, altitude reference, flight-phase one-hot (5), reference velocity and acceleration (6) |
| Onboard wind estimate | 2 | body air velocity x, y, from the accelerometer, causal 5 s mean (with realistic errors) |
| Flight configuration | 8 | take-off mass, allocation pair shares (3), saturation strategy one-hot (3), M1 speed ceiling |

W never contains the friction, the true wind, the state of charge or the mission outcome. Those are used only to
evaluate z afterwards.

## 4. The three blocks

The model has **421 k parameters** in four trained parts.

### 4.1 Conditioning network → φ (43 k parameters)

```
[W (23), drivers (15)] = 38  →  Dense 256 + ReLU  →  Dense 128 + ReLU  →  φ (128)
```

φ is a compact summary of "the situation the drone is in": the manoeuvre, the wind, the payload, the vehicle's motion.
The encoder and the decoder share the same φ.

### 4.2 Encoder → z (175 k parameters)

```
[X (40), φ (128)] = 168  →  Dense 256 + ReLU  →  2 × [Dense 256 + ReLU + Dropout 0.1]  →  Dense 2  →  μ_z, σ_z
```

- The encoder looks at everything: all six motors, the vehicle state and the context.
- It outputs a Gaussian over the scalar z: a mean μ_z and a spread σ_z (softplus + 1e-5).
- During training, z is sampled as μ_z + σ_z·ε (ε ~ N(0, 1)). At inference, z = μ_z.

### 4.3 Decoder: the per-motor plant model (203 k parameters)

It has two small networks.

**g_nom: the healthy motor (102 k).** It is run six times per row, once per motor, with the same weights:

```
[φ (128), motor identity e_k (one-hot, 6), own command ω_cmd,k (1)] = 135
   →  2 × [Dense 256 + ReLU + Dropout]  →  Dense 6
   →  predicted Î_k, PWM̂_k, ω̂_k  and their 3 log-scales s_k
```

The one-hot lets it learn small differences between motors, for example position on the frame or spin direction.

**g_res: the wear effect on M1 (101 k).** It is run once per row:

```
[φ (128), M1 command ω_cmd,1 (1), z (1)] = 130
   →  2 × [Dense 256 + ReLU + Dropout]  →  Dense 6
   →  r = (r_I, r_PWM, r_ω)  and  Δs
```

**Putting them together**, per channel (current, PWM, speed):

| Motor | Mean | Spread |
|---|---|---|
| M1 | μ₁ = g_nom,1 + **z · r** | σ₁ = softplus(s₁ + Δs) + 1e-4 |
| M2…M6 | μ_k = g_nom,k | σ_k = softplus(s_k) + 1e-4 |

So **z = 0 gives exactly the healthy motor's mean**, and z scales how far M1's behaviour moves away from it. The
direction of that move (r) may depend on the situation and on z itself, because a worn motor does not deviate the same
way in a hover as in a climb.

**Known flaw (found in review, 2026-11-18): Δs is not multiplied by z.** g_res adds Δs to M1's log-σ for every z,
including z = 0, and Δs itself depends on z. Two consequences:
- the healthy statement holds for the mean only: at z = 0, M1's σ is still changed by g_res;
- the model has a second way to explain a worn M1: widen σ₁ instead of shifting μ₁.

Removing this path is the first step of the planned fixes (EARLY_PEAK_Z50_SPEC.md, L1). With the MSE fit, σ is not
used and the flaw has no effect.

The output is a Gaussian per channel (18 channels) for the measured responses Y.

## 5. How it is trained

### 5.1 Two rules that make z mean "M1 wear"

1. **The healthy model learns only from healthy flights.** M1's copy of g_nom receives gradient only from rows of
   flights with 1× friction (stop-gradient on the others). Motors 2–6 are always healthy, so their copies learn from
   every row. Without this rule, g_nom would also learn worn behaviour and z would have nothing left to explain.
2. **Anchor.** On healthy-flight rows, the loss pushes μ_z towards 0 (term 4 below). Together with rule 1, this fixes
   the zero of the scale: 0 = healthy.

The healthy flag comes from the flight's design (friction band 0). It is used in the loss only and is never an input,
so the trained model never needs it.

### 5.2 Loss per batch (mean over rows)

| # | Term | Weight | What it does |
|---|---|---|---|
| 1 | Fit of the 18 motor channels | 1 | Rebuild current, PWM and speed (see the note below) |
| 2 | KL(q(z \| X, φ) ‖ N(0, 1)) | 0.016 | Keeps z's distribution tidy (VAE term) |
| 3 | mean[(relu(−μ_z) + relu(μ_z − 1))²] | 100 | Soft wall: keeps μ_z in [0, 1] |
| 4 | mean[μ_z²] over healthy-flight rows | 100 | Anchor: healthy → z = 0 |

**The fit term is the Gaussian negative log-likelihood** (the decoder learns a mean and a σ per channel). Z32 compared
three options, each with 3 seeds:
- **Gaussian NLL (chosen):** reconstruction error 0.001, Spearman(z, friction) 0.989–0.995, leaks within the limit.
- **β-NLL with β = 0.5** (σ can no longer lower the loss by giving up on the mean): as accurate, with leaks about twice
  as large; it peaks slightly later but still rises afterwards.
- **Plain MSE** (σ fixed at 1): diverges with these anchor and range weights. Rejected.

### 5.3 Settings

| Setting | Value |
|---|---|
| Optimiser | Adam, lr 1e-4, halved after 3 epochs without improvement |
| Batch | 1,024 rows |
| Epochs | 15 (no early stop; patience 100) |
| Model kept | the epoch with the lowest **validation MSE of the 18 motor channels** |
| Dropout | 0.1 in this model. **Z36 found it causes the early peak; the next version uses 0** (§9) |
| Initialisation | Glorot-uniform weights, zero biases |
| Seeds | ≥ 2 per configuration |

## 6. How z is used

1. Every 0.1 s: standardise the new row, compute φ, run the encoder, take z = μ_z. The decoder is not needed online.
2. Smooth z causally, for example with a 10 s running median (Z34): it removes the effect of gusts at no cost.
3. Per flight, the median z is the health estimate used to rank flights and to feed the forecaster F.

The sign is fixed by construction: z ≥ 0, and it grows with friction.

## 7. What is not in the model

- No friction value, true wind, state of charge or outcome as an input.
- No temporal memory: each row is encoded on its own. The temporal encoders (Z28/Z34: TCN at 2, 5 and 10 s, state-space,
  transformer) did not help with the old decoder, and have not yet been tried with this one.
- One z only, for M1. The variant with one z per motor (`--decoder plant6`, Z33) is a separate model.

## 8. Why this design (what the experiments showed)

All numbers are on the DB_dev v2 test flights, the mean of 2 seeds (Z31).

| | Old model (Z27: one decoder rebuilds all 40 X) | This model (Z30 A3) |
|---|---|---|
| Error on the motor channels | 0.022 | **0.001** (20× lower) |
| Spearman(z, friction) | 0.93 | **0.994** |
| Mass leak: rank correlation of the z residual (after the friction trend) with mass | 0.35 | 0.13 |
| Wind leak: same with mean wind | 0.03 | −0.18 |
| Mass / wind leak in size: z shift per 1 σ of mass / wind, over the z step from 1× to 2× friction (Z32, worst case over seeds and test sets) | 0.19 / 0.07 (seed 0) | **0.034 / 0.020** |

The two leak measures answer different questions. The rank correlation says whether *any* trend remains, however
small. The size measure says how much it matters: for this model, mass shifts z by about 3 % of the step from a healthy
motor to a twice-worn one.

**Why the old model stalled.** Friction adds a torque that depends on the motor's instantaneous speed. The old decoder
did not get each motor's command, so it had to guess that speed from W. It missed the gust-driven part, the miss grew with
friction, and the Gaussian loss then shrank σ instead of fixing the mean. Giving each motor its own command removes the
guess.

## 9. Open points and caveats

- **The early peak: solved by dropout 0 (Z36, 2026-11-19).** With dropout 0.1, the validation error was lowest after
  about 1 epoch and then rose ×4, on training flights and healthy flights too. Without dropout it keeps falling to
  epoch 13–15. DB_dev v2, 2 seeds:

  | | Dropout 0.1 (this document's model) | Dropout 0 | Dropout 0 + deterministic latent (next base) |
  |---|---|---|---|
  | Best val motor error | 0.0012 | 0.0004 | 0.0004 |
  | End / best | 3.9 | 1.04 | 1.03 |
  | Spearman(z, d), v2 test / v1 test | 0.994 / 0.994 | 0.991 / 0.982 | 0.991 / 0.989 |
  | Leak per step (mass / wind), worst seed, v2 test | 0.023 / 0.021 | 0.008 / 0.004 | 0.010 / 0.006 |
  | Same, v1 test | 0.028 / 0.028 | 0.052 / 0.033 | 0.044 / 0.027 |

  Neither z sampling (a deterministic latent alone still peaks) nor σ trading was needed to explain it. Plain MSE also
  removes the peak but worsens the latent (Spearman 0.979; v1 leaks 0.18 / 0.12), so the Gaussian NLL stays. The next
  base is therefore **dropout 0 + deterministic latent + Gaussian NLL** (Z36 R3). It is slightly weaker on the older v1
  test flights, which Z50 now tracks as a criterion. Details: EARLY_PEAK_REPORT.md.
- **Leaks: small in size.** The z shift for 1 σ of mass (or wind), divided by the z step from 1× to 2× friction, is at
  worst 0.034 for mass and 0.020 for wind over seeds and test sets (limit 0.10). The rank correlations in §8 show that a
  trend remains. Two structural reasons were pointed out in review:
  - z and r can trade scale as a function of φ;
  - φ contains drivers (pack voltage sag, tracking error, body rates) that react to friction, so the healthy model sees
    health information at inference.

  Both are addressed in Z50.
- **Δs is not multiplied by z** (see §4.3): it gives z a route through σ₁. Planned fix in Z50.
- **The command already contains the controller's reaction.** The LQR and the allocation change ω_cmd when M1 weakens.
  Conditioning on ω_cmd could therefore hide or leak part of the health signal. This needs a dedicated check, and a
  reviewer will ask about it.
- **Hardware assumption.** Each motor's current, PWM and speed must be available onboard (ESC telemetry).
- **The variational part adds nothing.** A plain autoencoder gives the same results (Z31), and σ_z is not a calibrated
  uncertainty. Any uncertainty on z should come from seeds, ensembles or the forecaster.
- **Simulation only.** The decoder's wiring matches the simulator's motor model closely, which flatters
  identifiability. Real motors add effects such as temperature, ESC differences and propeller damage.
- **In-flight faults.** Detection delay and false alarms on progressive faults (evalfault_v2, G27) are not yet measured.


![Overview of the motor-health autoencoder](results/cvae/doc/cae_motor_health_architecture.png)

*Figure 1. Overview. (a) X is split by role: the commands go to the decoder, all 40 columns go to the encoder, and the drivers together with W form φ. (b) Inside the decoder: g_nom (healthy motor, shared by the six motors) and g_res (wear effect, M1 only, scaled by z).*

![Every layer and dimension of the motor-health autoencoder](results/cvae/doc/cvae_plant_architecture.png)

*Figure 2. Detailed version, generated from the code (`python -m resilience.cvae_plant_diagram`): every input column with its role, the layer sizes, the parameter counts, the motor-loop motivation, and the loss terms.*
