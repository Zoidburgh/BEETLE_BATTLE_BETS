# Combat smoothing/juggle arc — REWOUND 2026-07-20

UPDATE same day: smooth body pushing (3dfcc2e) was re-applied ALONE per
the redo guidance below — cherry-picked minus the juggle sliders, at
its original day-one tune (PUSH_SMOOTH_SPEED 12, PUSH_MATCH_RATE 0.75,
not the 3.4 that was tuned against the full stack). Everything else in
the arc stays out.

Status: the working tree was reset to a9b4927 (stance brace parked at 0 +
bot-match GUI fixes — those SURVIVE). Two commits and a day of
uncommitted tuning were removed. The ENTIRE arc (both commits + all
uncommitted work) is preserved in `combat_smoothing_attempt.patch` in
this directory (diff a9b4927 -> final state; apply with `git apply`).

## User verdict (the reason for the rewind)

> "overall i just cant tell visually whats knocking me up, the combat
> is not clear now"

Not a single-feature complaint — the PILE of simultaneous smoothing
systems (linear + vertical + rotational velocity matching, volley
gating, caps, taxes, all tuned aggressively in one sitting) made combat
causality unreadable. Forces still happened, but hits stopped LOOKING
like hits. This echoes the physics-feel compass: the user's game is
built on discrete impact EVENTS; every smoothing layer trades event
legibility for continuity, and stacking several crossed the line.

## What was in the removed arc

Commit 1857d99 — Soft juggle J0+J1 (volley-timed aerial lifts):
- `_volley_factor`: lift to an AIRBORNE victim scaled by relative
  vertical closing (timed flick = full, static hold = little)
- --juggletrace instrumentation, JUGGLE_VOLLEY + JUGGLE_REF sliders
- Plan doc: soft_juggle_plan.md (survives, committed in a9b4927)

Commit 3dfcc2e — Smooth body pushing:
- Normalized the contact-normal EMA (fresh contacts had a short/lagged
  normal -> weak diagonal impulses) — CONSIDER KEEPING ON REDO; this
  was an honest bug fix independent of the feel debate
- Body-vs-body velocity matching below PUSH_SMOOTH_SPEED closing
  (bulldoze instead of stick-slip impulse chatter), PUSH_MATCH_RATE
  convergence dial. User at the time: "this really helps a lot".
  Horn/shaft version tried + reverted same day (kept body-only)

Uncommitted (2026-07-19/20, on top of 3dfcc2e):
- Aerial vertical smoothing: vy velocity-matching when BOTH airborne
  below threshold (the vertical half of smooth push)
- TILT_SMOOTH: body-collision tilt injection scaled by closing speed
  (rotational half of smooth push)
- Volley rising-victim rule: victim's UPWARD vy no longer subtracts
  from closing (fixed "no boost lifting under an already-lifted
  beetle") + ref-clamp bugfix (internal max(ref,1.0) floor silently
  overrode slider values below 1)
- LIFT_CAP exposed as slider (the hardcoded 12 anti-carry ceiling)
- JUGGLE_WINDOW/JUGGLE_TAX (J2): height-penalty start + steepness
- Final tune state: JUGGLE_REF 0.5, JUGGLE_VOLLEY 1, HORN_LIFT_STRENGTH
  2.5 (was 1.36), LIFT_CAP 17, PUSH_SMOOTH_SPEED 3.4 (from 12),
  PUSH_MATCH_RATE 0.75, TILT_SMOOTH 1.0

## Notes for a possible redo (user: "maybe i do smooth body pushing
## again, not sure")

1. Smooth body pushing was the piece with a clear positive verdict on
   its own DAY ONE ("this really helps a lot", camera jitter gone).
   The verdict soured only after three more smoothing layers landed on
   top. If redone: bring back ONLY 3dfcc2e (git cherry-pick or the
   first ~third of the patch), play-test alone for a session before
   anything else.
2. The EMA-normalization bugfix inside 3dfcc2e is arguably a
   correctness fix, not a feel change — separable and low-risk.
3. One threshold serving three channels (linear/vertical/tilt) made
   tuning opaque — the 12 -> 3.4 slam was the user trying to regain
   punch globally. Separate thresholds per channel if layering again.
4. The volley ref-clamp bug (max(ref, 1.0)) means all JUGGLE_REF tuning
   below 1 during this arc was placebo — remember if reusing J1 code.
5. Lift force chain reference (still true in current code, values
   hardcoded again after rewind): lift_impulse x HORN_LIFT_STRENGTH x
   height_penalty[1/(1+0.35*(avg_h-2))] / LIFT_STEP_DIV[9], per-tick
   ceiling min(_, 12/LIFT_STEP_DIV), pending_lift drains 20%/tick.
