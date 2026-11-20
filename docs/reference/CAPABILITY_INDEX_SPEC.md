# Capability index: one health number from the six latents, without labels

**Status:** spec for review (2026-11-19, ASSUMPTIONS Z40, "Next"). Nothing is coded or run until approved.

## 1. Idea

The six latents z_1…z_6 say how far each motor is from a healthy one. For the drone, what matters is the
consequence: **how much thrust and moment the six motors can still produce together.** Because of the geometry, the
same degradation matters differently depending on where it sits. For example, M1 and M4 form a coupled pair (G23),
so losing both is not the same as losing M1 and M2.

The trained decoder is a learned model of each motor: given the flight conditions, a speed command and z_k, it
predicts that motor's rotor speed, current and PWM. So the decoder itself can tell us **the speed each motor reaches
at full command, given its z_k.** No friction value or other label is needed. Combined with the known mixer
matrix, this gives the remaining control authority of the vehicle.

## 2. Definition

For each flight row (or each flight, using medians):

1. **Per-motor ceiling.** Query the decoder for motor k at the maximum command, ω_cmd = 840 rad/s, with the row's
   conditions φ:
   - ω̂_k = predicted rotor speed with the encoded z_k;
   - ω̂_k⁰ = the same with z_k = 0 (healthy).

   Available thrust fraction: c_k = (ω̂_k / ω̂_k⁰)², clipped to [0, 1].
2. **Vehicle authority.** With the mixer matrix F (`allocation.FORCES`) and the bounds
   ω² ∈ [200², c_k · 840²], compute by linear programming the largest extra wrench the vehicle can still produce
   on top of hover at the flight's mass, along each direction:
   - thrust up;
   - ± roll, ± pitch, ± yaw (yaw weighted low, as the allocation's S2 strategy does).

   This gives a vector of margins m(z).
3. **Capability index.** CI = 1 − min over directions of m(z) / m(0). CI is:
   - 0 for a healthy vehicle;
   - 1 when some direction has no margin left.

   Also reported: the direction that limits.

CI is invariant to renaming the motors (up to the geometry, which is real), never decreases as any motor degrades,
and accounts for the pair coupling by construction. Two degraded motors reduce the margin more than one.

## 3. What it uses

| Input | Labels? |
|---|---|
| encoder and decoder of the trained model (E1 or D0) | none beyond the model's own healthy flag |
| flight conditions of the row (mass, operating point) | none, measured or planned |
| mixer matrix F, speed limits 200 and 840 rad/s | none, known design data |

Friction is used **only to evaluate**, never to build CI.

## 4. Risks, and how they are checked

- **Decoder extrapolation at full command.** Healthy motors rarely run at 840 rad/s in the data; faulty motors at
  high friction often do (§3.5 of the model notebooks). Check, on evaluation only: compare ω̂_k with the simulator's
  actual maximum speed. Run a short simulator sweep (constant full command, friction 1–8×, a few operating points)
  and compare.
- **Thrust ∝ ω²** at the propeller: the simulator uses exactly this law, so it is consistent here. On a real drone
  this is an approximation.
- **Hover point.** It depends on the mass. Mass is in W (configuration), so it is known.

## 5. Read-outs (to be fixed before results)

Same as the Z40 merges, plus:
- the agreement of CI with the **true** remaining authority, computed the same way from the simulator's true
  frictions and speed law (evaluation only): Spearman over all single- and two-motor flights;
- whether CI ranks two-motor faults above single ones, and pair faults (M1+M4, M2+M5) as the geometry predicts.

## 6. Cost

- About half a day of code: decoder query, LP per flight, simulator sweep for the check.
- No retraining.
- The simulator sweep is about 30 short flights (~1 h on the cluster).
