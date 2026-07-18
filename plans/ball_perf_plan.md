# Ball Performance Plan — measure first, then cut
2026-07-18. Status: PLANNED. Companion: multi_ball_plan.md (MB-PERF history).

## Baselines (user + perf logs)

- No balls / 1 ball, normal combat: ~30-35 FPS (frame ~28-33ms)
- 3 balls, WORST-case scrum (4 bots + 3 balls grinding, 1627 ball-ball
  contacts): ~19 FPS, frame 52ms, physics 34ms, ball_physics 15.5ms,
  substep spiral at 3.3 iters/frame (slow frames double physics costs)
- scene_render ~10ms is the separate long-standing GPU baseline
  (GPU_CPU_OPT_PLAN.md / Vulkan story — not this plan's target)

## THE LESSON THAT SHAPES THIS PLAN: two cost models were wrong

1. Batching the ball voxel checks (12 launch+syncs -> 1): 15 -> 20 FPS. Real
   but partial.
2. MB-PERF hybrid (cluster kernels -> analytic synthesis): ball_physics
   DID NOT MOVE (15.7 -> 15.5 in matched worst-case logs). The cluster
   kernels were NOT the dominant cost. Architecture kept (it's cleaner +
   the safety net stands) but the perf claim failed.
NO MORE GUESSING. Phase P0 instruments before any further optimization.

## Phase P0 — instrument the ball section (15 min + one busy-scrum log)

Sub-timers (perf_counter pairs) inside the per-substep ball region, feeding
new perf-log lines:
- ball_batch_check (fill + kernel + readback)
- ball_midtick_stamps (clear_and_render calls)
- ball_manifold (the _ball_surface_contact calls)
- ball_responses (beetle_collision ball-pair bodies minus manifold)
- ball_ball (MB3 pair loop)
- ball_misc (goal detection, governor, rest latch, dust)
Also count calls per substep for each. Gate: one worst-case scrum log ->
ranked table. THEN pick from P1 by data.

## Phase P1 — candidate cuts (pick by P0 data, not vibes)

- SPIRAL BREAKER (likely biggest lever regardless of ranking): when
  physics_iters climbs past 2, run ball-pair collision every OTHER substep
  (staleness precedent: pair_collision_last). Halves the marginal cost
  exactly when the spiral is feeding on it. Adaptive degradation — full
  quality at healthy framerate.
- Mid-tick stamps: gate to pairs in actual CONTACT (not just close), or
  drop to every other substep (stamp staleness is already accepted).
- Manifold: profile per-call cost; if dominant, cache per (ball, beetle,
  substep) across detection/response consumers, or trim shape count for
  far contacts (legs only matter when low).
- Python response bodies: if dominant, the heavy branches (on-top rest
  logic) can early-out harder for clearly-side contacts.
- Batch check: already one launch; only remaining cut is skipping it when
  no ball moved AND no beetle close-moved (rare win, low priority).

## Success criteria

- Worst-case 3-ball scrum: >= 28 FPS (frame <= 36ms)
- Normal 1-ball play: >= 30 FPS held
- No feel regressions (the v1 analytic-detection incident is the cautionary
  tale: perf changes gate through user feel tests like everything else)
