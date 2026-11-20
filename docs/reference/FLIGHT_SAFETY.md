# Flight safety under actuator saturation

Status 2026-11-12. Code: `resilience/safety.py`. Notebook: `flight_safety.ipynb`. Results:
`results/safety/run_20261112/`. Assumptions: ASSUMPTIONS.md F1–F15, Q8, S5b, S11–S14.

**Principle.** A mission succeeds or fails only on what the vehicle does in the nonlinear simulation. Motor
saturation is reported as a **diagnostic** of reduced actuator headroom. It is never a failure condition by
itself. Mission outcome and actuator saturation are separate outputs.

**Pre-registration.** The thresholds (F1–F12) and the gust set (F13, F14) were written and hashed before any
graded run (`results/safety/preregistration_F_20261112.md`, sha256 `aaa32c39…`). The B-landing requirements (Q8)
and the landing-phase gusts (F15) were added the same way before any B-landing flight
(`preregistration_Q8_F15_20261112.md`, `024532ef…`). The notebook checks both hashes. No threshold was changed
after the results were seen.

## 1. Is the 840 rad/s limit applied to the plant? (S11)

Signal chain in `hexa_waypoints.slx`: LQR u → pinv allocation ω²_cmd → **Saturation [200², 840²]** → √ →
open-loop ESC map PWM = 0.0008842·ω_ref + 0.03436 → BLDC electrical and mechanical model → F·ω² of the
**simulated** rotor speeds → wrench clamps (S5b) → rigid-body dynamics.

Checked on the logged flights (`safety.actuator_audit`, City D11):

- **The limit is enforced on the plant input.** M1's PWM equals the ESC map of the clipped command exactly (zero
  difference). The logged `omega_cmd` is the request *before* the clip, so it can exceed 840. This is why the
  "headroom" diagnostic can be negative.
- **The plant is driven by the actual rotor speeds.** The wrench applied to the rigid body equals F·ω² of the
  simulated speeds to within 1e-14. The wrench clamps (thrust [9.81, 50] N, moments ±4 N·m) were **never active**
  in any graded flight, with or without gusts.
- **The 840 rad/s limit is a duty-cycle cap at PWM 0.777, not the battery-voltage limit (PWM = 1).** The ESC map
  also does not match the 4S pack: healthy motors run at 0.79 of their speed command.

## 2. Are the motor and ESC models still meaningful under sustained saturation? (S12–S14)

City Divert L1 at the end of the saturated interval (M1 friction 7.8×, 14.5 V):

| Quantity | M1 (saturated) | Healthy motors |
|---|---|---|
| Speed | 255 rad/s (0.31 of its command; about 23 % of the others' thrust) | about 527 rad/s |
| Winding current | 43 A | ≤ 19 A |
| Copper loss I²R | ≈ 370 W | ≤ 75 W |
| At PWM = 1 (steady state) | 331 rad/s, 55 A | |

Missing behaviour:

- **No current limit, no motor or ESC thermal model, no derating and no motor-failure event.** A real
  motor would overheat at 43 A sustained, and a real ESC would current-limit or cut out. Either would remove
  M1 thrust that the simulation still delivers, so the post-saturation flight is **optimistic about M1**.
- **No integrator to wind up:** the LQR is static state feedback. The navigator's lead feedback slows the
  reference when the vehicle lags.
- **The allocation is not saturation-aware:** the wrench lost at the clip is not redistributed. The residual
  u_cmd − F·clip(ω²_cmd) reaches 3.1 N of thrust and 0.92 N·m of roll/pitch moment in City Divert L1. An autopilot
  mixer would give attitude priority over thrust. This residual shows up as a standing tracking offset (§4).
- **The wear law runs past saturation:** the friction reaches 7.8–8.5×, beyond the calibrated range of D10.

The degradation law, the controller and the allocation were **not changed** for this study.

## 3. Failure checks (F1–F10) and diagnostics (F12)

| Check | Threshold | Source |
|---|---|---|
| F1 envelope / divergence | Stop block (roll or pitch > 60°, z ≤ −1 m) or `hexa:diverged` → FAIL | S5 |
| F2 tilt | > 60° for ≥ 0.3 s | PX4 FD_FAIL_R/P, *_TTRI defaults |
| F3 attitude tracking | roll/pitch error to the navigator reference attitude > 30° for ≥ 2 s | ArduPilot crash check |
| F4 angular rates | \|p\|, \|q\| > 220 °/s or \|r\| > 200 °/s for ≥ 0.1 s | PX4 MC_ROLLRATE_MAX, MC_YAWRATE_MAX |
| F5 altitude control | \|z − z_ref\| > 1 m for ≥ 1 s | = corridor (Q5) |
| F6 collision | obstacle check with a 0.5 m body radius; ground impact (z < 0.5 m away from the landing site) | Q6, Q7 |
| F7 landing | at contact (z ≤ 0.05 m): \|v_z\| ≤ 1 m/s, v_xy ≤ 0.5 m/s, tilt ≤ 15°, within 1 m of the site; landed at the end | 2× ArduPilot LAND_SPEED |
| F8 | corridor ≤ 1 m (Q5), SoC ≥ 70 % at the end (Q4) | Q4, Q5 |
| F9 deadlines | arrival over B by 330 s (Q1); divert touchdown by 340 s | Q1, Q2 |

Diagnostics only (F12), recorded while airborne:

- time at each command limit and the first saturation time;
- minimum headroom 840 − ω_cmd;
- allocation residual;
- time with the wrench clamps active;
- M1 winding current, copper loss, speed vs command;
- yaw error.

Rates are the model's Euler-angle rates, from a new 100 Hz log of the 12 controller states (F11, cache version
6). Logging leaves the flights bit-identical: the City Divert L1 flight reproduced the cached one exactly.

## 4. Results

### 4.1 D11 response sets, no disturbance

Outcome classes are the same on both maps and for both B definitions.

| Option | City, B hovers | City, B lands | Darrah, B lands | M1 at 840 while airborne (city, B lands) |
|---|---|---|---|---|
| No action | FAIL (SoC 67.3 %) | FAIL (SoC 64.7 %) | FAIL (65.4 %) | 117 s, from 235 s |
| Allocation | MISSION PASS (73.4 %) | MISSION PASS (71.6 %) | PASS (72.1 %) | never |
| Payload | FAIL (SoC) | FAIL (SoC 66.9 %) | FAIL (67.5 %) | 38 s |
| Allocation + Payload | MISSION PASS | MISSION PASS (73.4 %) | PASS (73.8 %) | never |
| Divert A (home) | SAFE ABORT | SAFE ABORT (80.0 %) | SAFE ABORT (79.7 %) | 1 s, just before touchdown |
| Divert L1 | SAFE ABORT (71.4 %) | SAFE ABORT (71.4 %) | SAFE ABORT (82.3 %) | **56 s, from 240 s** |
| Allocation + divert L1 | SAFE ABORT (76.3 %) | SAFE ABORT (76.3 %) | SAFE ABORT (84.2 %) | never |

- **Every failure is on the SoC reserve.** No flight violates attitude, rates, altitude control, collision or
  landing. Maximum tilt is 8.6°, attitude error 5.4°, altitude error 0.15 m and roll/pitch rate 33 °/s (limit 220).
- **City Divert L1 flies 56 s with M1 at its command limit and lands correctly:** 0.19 m/s at contact, 0.86 m from
  the site.
- **B lands:** the flights end at touchdown + 5 s, about 361 s, so the end SoC is 1.3–2.5 points lower than with
  the hover until 340 s. Saturated No action touches down 0.96 m from B, which is 0.04 m inside the 1 m landing
  radius. The allocation options touch down 0.17–0.19 m from B.
- The divert flights are bit-identical under both B definitions, as expected: the divert replaces the B legs.

### 4.2 Gusts in the saturated interval (City, F13 and F15; 84 disturbed flights)

| Case | Outcomes (disturbed) | Peak 3D error in the gust window | Increase over undisturbed | Peak attitude error |
|---|---|---|---|---|
| Divert L1 (saturated) | 16 SAFE ABORT, **2 FAIL** (corridor 1.02, 1.04 m; +x gust at 262 s and 282 s) | 0.70–1.04 m | 0.00–0.29 m | ≤ 23.3° |
| No action (saturated, lands at B) | 24 FAIL: all on SoC, 3 also on the corridor (+x gusts: 1.05, 1.09, 1.14 m) | 0.70–1.15 m | 0.00–0.33 m | ≤ 24.5° |
| Allocation + divert L1 | 18 SAFE ABORT | 0.18–0.53 m | 0.00–0.35 m | ≤ 22.3° |
| Allocation (lands at B) | 24 MISSION PASS (including the 6 landing-phase gusts) | 0.18–0.58 m | 0.00–0.40 m | ≤ 24.2° |

Findings:

1. **No disturbed flight lost control.** The F1–F5 limits (tilt, attitude, rates, altitude) and the landing check
   F7 were never violated, saturated or not. Peak tilt stayed at or below 24.5° against the 60° limit, and every
   landing passed. That includes the saturated No action landing at B under the descent gusts.
2. **The gust response itself is the same with and without saturation.** For the same gust, tilt and attitude
   peaks agree within about 2°. The extra tracking error peaks at 0.29–0.33 m for the saturated cases and at
   0.32–0.40 m for the allocation cases. During the +x gusts, the saturated M1 request dips towards or below 840
   rad/s.
3. **The saturated cases fail on margin, not on stability.** They fly with a standing tracking offset of
   0.70–0.97 m, against 0.18–0.21 m with allocation. That offset is the unredistributed allocation residual of §2.
   A 6 m/s gust towards +x (M1's arm) then pushes the error past the 1 m corridor. With allocation, no gust takes
   the error above 0.58 m.
4. **Ranking under disturbance:** Allocation and Allocation + divert L1 keep every requirement in every gust.
   Divert L1 loses 2 of 18 runs, on the corridor only. No action fails throughout, on SoC.

## 5. Limitations of the physical model and of the test

- **M1 thrust is optimistic after saturation:** no current, thermal or ESC limits are modelled (S12). A
  motor cut-out, the likely real outcome after tens of seconds at 43 A, is not simulated.
- **The 840 rad/s cap is a duty-cycle limit (S11):** the real ceiling would be higher, and the mismatched ESC map
  makes saturation occur earlier than the motor alone would require.
- **The pinv allocation is not saturation-aware (S13):** an attitude-priority mixer would probably reduce the
  standing offset that causes the corridor failures.
- **Wind acts through form drag and rotor inflow only (S9, S10):** there are no aerodynamic moments, no rotor
  H-force or flapping, and the gusts are spatially uniform. Moment disturbances were therefore **not tested**.
- **One gust level** (6 m/s lateral, 3 m/s vertical, 4 s), three onsets plus one landing onset per B case, one
  fault (D11), one speed (2 m/s), no wind or turbulence otherwise, no sensor noise (S6). It is a controlled test,
  not a statistical one.
- **No ground or gear model (S7):** the landing is judged kinematically at contact.
- **Wear beyond the calibrated range (S14):** the friction values after saturation are an extrapolation.
- **The F3 reference attitude ignores drag and wind:** the reported "attitude error" includes the tilt needed to
  reject the gust. It is conservative, and still at most 24.5° against the 30° limit.
