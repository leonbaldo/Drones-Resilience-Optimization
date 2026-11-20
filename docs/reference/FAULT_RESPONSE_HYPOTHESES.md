# In-flight fault response: hypotheses for step 4 (detection → λ prognosis → risk)

Status 2026-11-11: **hypotheses, not results.** Each one says what it predicts, which existing result supports
it, and how to test it with the code already in place. The numbers come from
`online_navigation_options.ipynb` (navigator, D11 fault: 3× friction, doubling every 90 s from 60 s,
decision at 100 s, 2 m/s, on both maps) and `online_navigation.ipynb`.

## What the navigated runs already show

| | Darrah map | Zurich-Enge |
|---|---|---|
| No action | FAIL (SoC 67.3 %), M1 at 840 rad/s from 240 s | FAIL (67.3 %), M1 at 840 from 235 s |
| Allocation (unload M1) | MISSION PASS (73.4 %), never saturates; ln b rate 16.7 → 6.4 /h | same (73.4 %), same rates |
| Payload drop | FAIL (69.3 %), saturates from 315 s | FAIL (69.3 %), 316 s |
| Divert A (home) | SAFE ABORT 79.7 %, touchdown 219.9 s, M1 at 840 from 219.7 s | SAFE ABORT 80.0 %, touchdown 217.2 s, 840 from 217.0 s |
| Divert L1 | SAFE ABORT 82.3 %, touchdown 194.9 s, 840 after touchdown | SAFE ABORT 71.4 %, touchdown 295.8 s, **840 from 239.7 s: 56 s saturated in the air** |
| Allocation + divert L1 | SAFE ABORT 84.2 % | SAFE ABORT 76.3 % |
| Prognosis on previews (predicted − flown end SoC) | B +4.7 pp, A +1.7, L1 +1.3 | B +4.7, A +1.7, L1 +3.7 |
| Preview end time − flight end | ≤ 0.01 s | ≤ 0.01 s |

The outcome classes on Darrah's map are the same as with the old time-scheduled reference (wear_fault_flight.ipynb,
D11). The city map changes only the divert geometry: its L1 is 100 s further away.

---

### H1. Detection: the abnormal trend is visible within about 15 s of onset
**Claim.** A sliding-window regression of ln b̂, with b̂ the M1 friction estimated from ESC telemetry (rebuilt ω,
current; `load_model.estimate_friction`), detects the D11 fault within ≤ 15 s of onset at < 1 false alarm per
healthy flight. For the ×2/300 s fault the delay is about 3× longer (the slope is 3.3× smaller).
**Basis.** The observed ln b rate before the decision is 18.5 /h, against 0.007 /h for normal aging (D5): a
2600× contrast. The rate is below the 27.7 /h at ω_ref because the friction slows M1 (D10).
**Test.** `fault_study.severity_sweep` doubling times (300, 180, 120, 90, 60 s) plus healthy flights. Window 10–30 s.
Detection delay and false-alarm rate vs window, with the telemetry noise of B2/B3.

### H2. λ can be estimated online, and b(t) projected per option, with option-specific M1 speeds
**Claim.** λ̂ = slope of ln b̂ / ⟨(ω₁/ω_ref)ⁿ⟩ over the window converges to ±20 % of λ_fault within 30 s of
detection. Projecting dx/dt = λ̂ (ω₁,opt(t)/ω_ref)ⁿ, with ω₁,opt from the steady-state load model along each option's
**preview** timeline, predicts the time M1 reaches 840 rad/s within ±10 s.
**Basis.** Allocation lowers the M1 command from 731 to 431 rad/s, and the measured rate falls by 0.386×. That
ratio is what a speed-driven law predicts (n = 2: (431/731)² = 0.35; closer once the speed feedback is
included). The previews match the flight timelines to 0.01 s (N10).
**Test.** Compare the projected saturation time per option with the "M1 at 840 from (s)" column for D11 and the
severity sweep.

### H3. Under fast faults, control headroom, not SoC, is the binding risk, and grading must include it
**Revised 2026-11-12 after the user's decision and the gust tests (FLIGHT_SAFETY.md §4).** Saturation is not
graded. In this plant a saturated M1 did not cause loss of control in any of 84 gust runs. It cost **tracking
margin**: a 0.7–1.0 m standing offset from the unredistributed allocation residual, so two Divert L1 gust runs
and three No action runs broke the 1 m corridor. Allocation options kept every requirement. The headroom risk to
model is therefore "projected corridor/tracking margin", plus the unmodelled motor current and thermal limits
(S12), not "time at the command limit". The original claim is kept below for the record.

**Claim.** For doubling times ≤ 90 s, M1 saturates before the SoC reserve is violated in every option that keeps
the original allocation. A risk that counts only SoC (the current UELF) therefore accepts options that fly with
M1 saturated. Adding "M1 command at its limit while airborne" (or the projected time-to-saturation < time-to-landing)
changes the ranking: Allocation-based options win, and long diverts lose.
**Basis.** City Divert L1 is graded SAFE ABORT but flies 56 s with M1 at 840 rad/s. No action saturates 75–80 s
before the end of the hold. Allocation never saturates in any run.
**Test.** Grade saturation while airborne as FAIL or degraded (a new Q-row). Re-grade both maps. Run
`prognostics.decide_joint` with the headroom term (H2 projection) and compare the chosen responses with the
flown outcomes.

### H4. The SoC prognosis is optimistic because the load model ignores wear growth; projecting b(t) fixes it
**Claim.** The prognosis error (+1.3 to +4.7 pp) grows with the time left after the decision, because the
measured pre-decision current is extrapolated as constant while friction keeps growing. Feeding the projected
b(t) of H2 into the per-case current (`case_loads` friction axis) reduces the error below 1 pp. It also moves the
UELF risk of B from 0.31 to near 1, which is the correct side: B fails.
**Basis.** The timing error is ≤ 0.01 s, so the remaining error is the load. The largest error is on the longest
remaining flights: B (240 s left) and city L1 (190 s left).
**Test.** `nav_options.prognosis_check` with a time-varying load. Report the error vs time left.

### H5. Speed is a resilience action, but only if the mission ends at touchdown
**Claim.** For a mission that **lands at B** (delivery), commanding a higher cruise speed at the decision
(e.g. 2 → 5 m/s) cuts the remaining exposure (time × (ω/ω_ref)ⁿ) and the energy. For D11 that turns No action
from FAIL into PASS without any other action: the remaining ≈ 420 m takes about 90 s instead of 210 s, so the
landing at ≈ 195 s comes before the 235–240 s saturation. For the study's current B definition, **hover over B
until 340 s**, the total time is fixed, so speed changes SoC and saturation very little.
**Basis.** Healthy energy per mission: 21.2 / 11.4 / 10.2 Wh at 2 / 5 / 8 m/s. Mean pack power is 237 / 262 /
295 W, so power rises with speed less than time falls. At 8 m/s the total thrust is about 11 % above hover
(drag), so ω₁ rises about 5 %; at n = 2 the per-second exposure rises 11 %, far less than the time saved. The
navigator already changes speed online without discontinuities.
**Limit to watch.** A degraded M1 near 840 rad/s has little room for the extra thrust and tilt of fast flight
and of the acceleration ramp. An optimum speed is expected, not "as fast as possible". It should be lower with
higher n (bearing wear, n = 7: 1.05⁷ = 1.4 per second).
**Test.** Speed sweep at the decision (2, 3, 4, 5, 6, 8 m/s) × allocation on/off × B mission {hover to 340 s,
land at B}. Measure end SoC, b at the end, time M1 is saturated, and tracking error.

### H6. The map changes the decision mainly through landing-site geometry, not through the street texture
**Claim.** With a matched route length, the city map reproduces the Darrah outcomes for every continue-to-B
option within 0.1 pp SoC and 5 s (observed: 67.27 vs 67.28 %, saturation 235 vs 240 s). Divert outcomes differ
by the divert path length and corner count. The tighter city corners (R′ down to 4 m, 20 vertices) matter only
above about 4 m/s, where the curvature limits bind.
**Test.** Repeat the D11 set at 5 m/s on both maps. Add sites at matched distances on the city map.

### H7. The stop-and-go route change costs little
**Claim.** Braking from 2 m/s takes 5.3 s (observed) and costs < 0.5 Wh and < 0.5 pp SoC compared with a
blended handover. From 5–8 m/s the braking distance (up to about 25 m) becomes relevant to clearance, so the
divert should be planned from the stop point (as now), not from the decision point.
**Test.** Compare with a flight that diverts from rest at the same point. Repeat at 5 and 8 m/s.

---

## Proposed order

H1 (detection) → H2 (λ̂ and the b(t) projection) → H4 (projected load in the prognosis) → H3 (headroom in the risk
and the grading) → H5 (speed as an action, needs a land-at-B mission option). H6 and H7 are side checks that can
reuse the same runs.

## Decisions
1. **Decided 2026-11-12 (user):** M1 at its command limit is **not** a failure, whatever its duration. It is a
   diagnostic of reduced headroom. Mission feasibility comes from the vehicle motion in the nonlinear simulation
   (FLIGHT_SAFETY.md, ASSUMPTIONS F1–F15). Saturation time, first saturation, minimum headroom and the allocation
   residual are reported separately.
2. **Decided 2026-11-12: B lands** (delivery). This replaces the hover until the horizon (Q2) for the navigated
   study, and makes speed a possible resilience action (H5).
3. **Decided 2026-11-12:** the city landing sites L1–L4 are acceptable as placeholders (R9).
