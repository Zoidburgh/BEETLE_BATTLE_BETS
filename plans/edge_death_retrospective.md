# Edge/Death Physics — Retrospective + Rework Plan

2026-07-19. STATUS: the entire edge/death attempt (10 versions in one
session) was REVERTED at user decision — beetle_physics.py restored to
commit 2b35a26. The full attempt is preserved in
`plans/edge_death_attempt_v10.patch` (apply with `git apply` to
resurrect any piece). Blow-by-blow history: falling_edge_clip_plan.md.
This doc is the distilled experience + the plan for doing it RIGHT.

## What we were trying to fix
Beetles dying off edges clip through the arena (horn through the lip,
body through hole rims), tip in arbitrary directions with weak "tilt",
and fall through holes their stance visibly spans.

## What we shipped, in order (and why each step happened)
1. Lip fulcrum: floor-sampled extremity contact during falls (vy damp,
   voidward slide) — fixed clip-through, caused hole slow-motion.
2. Straddle support: 4 stance samples, opposing-pair = bridge small
   holes; tilt-release; lean.
3. Fulcrum v3/v4/v5: jam constraint, contact torque, roll + side points.
4. Grid-truth over-edge (kernel): floor cache replaces the analytic
   per-mode catalog.
5. V7: body-frame torque projection (kernel never received rotation!).
6. V8: full audit -> falling-body physics gate (_dying).
7. V9/V10: mass-true lever sum -> support-pivot torque (COG levers
   self-cancel), hover/late-fall noise gates, drop cap + sliders.
Each fix was CORRECT in isolation and each exposed the next layer.
The final result was still feel-inconsistent: too many interacting
half-tuned systems patched in sequence.

## THE REAL BUGS FOUND — still present in the restored code
These were diagnosed with evidence (audit agent + --tiptrace) and are
TRUE of the code we reverted to. Any rework starts from these facts:

1. **Tipping kernel is WORLD-frame.** calculate_edge_tipping_kernel
   maps world-space levers directly to body pitch/roll and never
   receives beetle rotation — tip direction is fixed in world axes,
   random relative to facing. (~line 12060s in the restored file.)
2. **Sign conventions** (from the placement transform, the only truth):
   POSITIVE pitch = nose UP; POSITIVE roll = right side DOWN.
   `apply_bowl_tilt` (~8283) is DEAD CODE with an INVERTED pitch
   convention — signs were derived from it twice, wrong both times.
   Never cite it. (Also recorded in collision-system-notes memory.)
3. **Death tumbles run under STANDING physics.** on_ground is sticky
   and host/local deaths never clear it: level-restoring springs
   un-tumble the dying beetle, the 92-deg ground tilt wall freezes
   rotation, the grounded tilt-speed cap throttles spin, and the
   floor-rest vy clamp is one stale flag from freezing falls. Guests
   DO clear on_ground (network death path) => host and guest tumble
   DIFFERENTLY today.
4. **Top-3-farthest levers are direction-biased** (horn hijacks tips
   toward any void it hangs over) — BUT the naive fix (sum about COG)
   self-cancels once the overhang surrounds the COG (--tiptrace:
   torque 441->0 in ~15 substeps). The correct pivot is the SUPPORT
   CENTROID; torque = sum(over-edge) - count*support_centroid, faded
   as support collapses (<~15 columns = remnant noise, one flip
   observed at sup=15).
5. **Support is center-only.** Floor presence = 7x7 scan around the
   center; legs play ZERO role. Holes 8-16 voxels wide swallow beetles
   whose ~20-voxel stance spans them. Straddle sampling (4 stance
   points + per-sample replication of hole/board-break overrides —
   the height cache does NOT know dynamic holes) fixed this and FELT
   RIGHT; it's the most resurrectable piece of the attempt.
6. **Tipping fires for hovering respawners** (they skip the floor loop
   so floor reads void; band-edge voxels give junk micro-torques at
   y=20). Pre-existing, minor, easy gate.
7. The extra downward edge-pull is ~7 vy/substep at full overhang —
   part of the old "snap", but it races the rotation (inconsistent
   tip sizes). Needs a dial, not a constant.

## Why the attempt failed anyway (process lessons)
- **Patch-sequencing a coupled system doesn't converge.** Five writers
  of pitch/roll in three phases; every local fix shifted the blame.
  The audit should have been FIRST, not step 6.
- **Three consecutive theory-only fixes each moved the wrong lever;
  one --tiptrace run found the real one.** Instrument before fixing —
  this project has now learned this lesson in perf AND in feel.
- **Never derive signs from code that might be dead.** Check call
  sites first (apply_bowl_tilt burned us twice).
- Feel-tuning done against a broken foundation (wrong frame, fighting
  restoring springs) is worthless — every constant tuned before V7/V8
  had to be re-tuned after.

## THE REWORK PLAN (when we pick this up)
Design it as ONE system with one owner per phase, built on the fixed
foundation, instrumented from day one:

- **Phase order matters.** Fix the FOUNDATION first, feel later:
  (a) falling-body gate (bug 3) — standalone, host/guest-unifying,
      invisible except during deaths;
  (b) kernel: rotation arg + body-frame + support-centroid pivot +
      hover/depth gates (bugs 1, 4, 6) — with --tiptrace ON and a
      one-session acceptance test: walk off N/S/E/W forward+backward,
      trace must show sign-stable, growing torque each time;
  (c) THEN the death-fall state machine: TEETER (supported partially,
      full collision, tip torque + counterplay) -> COMMIT (-5, leaves
      collision) -> TUMBLE (fulcrum-lite: extremity catches as
      inelastic contacts) -> gone. One doc-defined owner for
      pitch/roll per phase — nothing else writes tilt during a fall;
  (d) straddle support LAST (it was good; resurrect from the patch).
- **Sliders from day one** (Edge Tip Scale / Drop pattern was right),
  in the MAIN physics panel.
- **Acceptance tests, not vibes:** the 4-direction walk-off trace, a
  yin-yang eye crossing, a lives-mode bot canary for death-fall
  duration variance, and host-vs-guest death comparison once v6
  testing runs.
- Budget a full session for (c) alone. Do not stack it onto a ball
  session.
