# J3 elbow has an 8° dead band going up and 1° going down

**Date:** 2026-08-10 · **Measured by:** `arm_precision.py <token> 91 backlash 3`
**Arm state at measurement:** shoulder remounted and verified, all four joints
PASS 16/16 on `arm_bench_test.py --delta 15`, camera floor 44 px.

## The number

```
J3 elbow onset backlash, 3 reps each direction, floor 24 px

  UP   (against gravity)   samples [-, 10, 6]    median 8°   spread 4°   1 miss
  DOWN (with gravity)      samples [3, 1, 1]     median 1°   spread 2°   0 misses
```

The onset trace is unambiguous — nothing, then everything:

| commanded ° | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
|---|---|---|---|---|---|---|---|---|---|---|
| changed px | 20 | 16 | 20 | 10 | 32 | 21 | 21 | 11 | 26 | **24169** |

Nine degrees of commanded motion produce noise. The tenth produces 24,169 px.
The slop is taken up all at once, which is what a gear train with directional
lash looks like when gravity holds the output against one flank.

## What it explains

Everything the harness reported tonight, with nothing left over:

- The endurance preflight probe commands J3 **up** by 4°. That is inside the
  8° band, so the probe can never pass. Four separate runs aborted on it.
- The same joint moved 25,700 px on a 15° command minutes earlier — 15 clears
  the band.
- Priming the gear train before the probe did not help, because a 4° prime is
  also inside the band. Two 4° moves in the same direction are still 4°.
- The probe passed on 2026-08-09 only because the adopt angles were wrong that
  night: a 13–17k px snap on enable dragged the train through its lash before
  the probe ran. Once the adopts were correct — 19 to 36 px on 2026-08-10 —
  nothing pre-loaded it and the probe failed every time. **The better the
  adopt, the more reliably that gate false-failed.**

## Relationship to the Phase 3 negative result

`2026-08-09-PHASE3-BACKLASH-NEGATIVE-RESULT.md` measured this joint's onset at
**under 1°** and, on that basis, backlash compensation was correctly not built.
That number matches today's **down** direction exactly. It does not match the
**up** direction, which is now 8°.

Two readings are possible and this document does not choose between them:

1. The joint degraded on 2026-08-10. It spent a long stretch stalling against a
   shoulder servo that had worked loose from its mounting screws and bound up —
   one shoulder servo was hot to the touch. Stalling a gravity-loaded gear train
   is a plausible way to open up lash.
2. Phase 3 did not separate the directions, or measured in a pose where gravity
   loaded the elbow differently, and the 8° was always there.

Deciding this needs the Phase 3 procedure re-run per-direction. **Do not treat
this as licence to rebuild backlash compensation** — Phase 3's reasoning stands
until someone re-measures, and a directional onset number is not the same claim
as the 10,000 px directional pixel split Phase 3 falsified.

## Consequences

- **Any commanded J3 move smaller than ~12° in the up direction is invisible**,
  and a harness that scores it will record a dead move that is a gearbox
  property, not a joint failure. J0's 36%/33% dead-move rate from 2026-08-09
  should be re-examined with this in mind before it is trusted.
- The endurance preflight probe needs a move that clears the band, or J3 is the
  wrong joint to probe with. **This is not a threshold to lower** — the arm
  genuinely does not move 4° up, and a gate that passed it would be lying.
- Mechanically, the elbow gear train wants inspection. 8° of lash on a joint
  carrying the whole forearm is worth a look before an 8-hour burn-in puts
  thousands of cycles through it.

## Not done, deliberately

No overnight endurance run was started. Eight hours against an unmeasured 8°
dead band would produce thousands of "dead moves" that are all the same gearbox
symptom — the exact manufactured finding this harness exists to prevent.

## Reproduce

```bash
cd ~/arm/tests
~/arm-venv/bin/python arm_precision.py "$TOKEN" 91 backlash 3
```

Shoulder adopt 91 is the operator's value from the collapsed rest pose. The arm
falls to that pose whenever the joints detach, which is every time a run ends.
