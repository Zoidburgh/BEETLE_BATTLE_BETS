# Ball Performance Plan — measure first, then cut
2026-07-18. Status: P0 IMPLEMENTED (uncommitted), awaiting worst-case scrum
log. Companion: multi_ball_plan.md (MB-PERF history).

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

IMPLEMENTED 2026-07-18: the six keys live in _physics_timing (auto-reset /
auto-history / auto-ranked in the perf log's Physics Breakdown), counters in
collision_stats, rates in a new "Ball Section P0" log block. ball_responses
subtracts manifold time internally; ball_misc is computed by difference per
substep, so the six sum to ball_physics.

RUN THE SCRUM LOG WITH `--perfsync`. Taichi launches are async: without it
the midtick-stamp timer sees launch cost only (~us) and the GPU cost lands
at the next sync point (misattributed). --perfsync puts a ti.sync() inside
the stamp timer. Diagnostic runs only — it slows the frame slightly, so
strip the flag for A/B comparisons after cuts.

## P0 RESULTS (scrum log 20260718_164432: 4 bots + 3 balls, --perfsync,
## 15 FPS, frame 67ms, physics 48ms, iters 3.8)

Ranked (ms per render frame; sub-timers sum exactly to ball_physics 21.72):
- ball_batch_check    11.72  (54%)  <- fill loop, NOT kernel (see below)
- ball_midtick_stamps  9.00  (41%)  (true GPU-inclusive cost, --perfsync)
- ball_misc            0.66
- ball_responses       0.17   } the feared Python bodies are NOISE —
- ball_manifold        0.15   } P1 candidates 3+4 are dead
- ball_ball            0.03
Rates/substep: batch 1.0 launch / 4.02 rows, stamps 1.54 (only 0.31 in
actual contact), responses 4.02, manifold 0.31, ball-ball checks 2.06.

ROOT CAUSE (verified by standalone benchmark, scratchpad
bench_field_access.py): Python-side Taichi field writes cost ~26us EACH.
The 8-writes-per-row fill loops = 2.5ms at 12 rows; kernel 0.1ms, readback
0.24ms. Benchmark reproduced the in-game 3.1ms/substep exactly. The beetle
batch (batch_check 3.10ms/step, ~12ms/frame inside beetle_collision) has
the IDENTICAL pattern. numpy staging + one from_numpy = 4x faster.

## P1 SHIPPED 2026-07-18 (uncommitted, awaiting A/B scrum log + feel test)

1. FILL FIX (both batches): stage rows in _pair_np_data/_pair_np_colors
   (numpy), ONE from_numpy upload per batch. No behavior change.
2. STAMP GATE: mid-tick re-stamp thresholds are contact-aware — in-contact
   pairs keep 0.35 pos / 0.06 rot; close-not-touching relax to 1.0 / 0.25.
   FEEL RISK: first-touch detection can trail a fast approach by <1 voxel
   (one substep). Feel-test fast aerial hits + beetle-standing-on-ball.
Expected: ball_physics ~21.7 -> ~9, beetle_collision ~18 -> ~10; iters
should fall from 3.8, compounding the win.
NOT taken (dead by data): manifold caching, response early-outs, ball-ball
work. HELD IN RESERVE: spiral breaker (adaptive every-other-substep ball
collision at iters>2) if the A/B lands short of 28 FPS.

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
