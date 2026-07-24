# Perf Plan — ROLLING DOC

Purpose: ONE place that records every perf attempt (win, loss, or no-move) so we
never re-try a known loser or re-guess a known cost model. **Update this file every
time a perf change is attempted**: add a row to the Attempt Log with date, change,
measured before/after, verdict. Candidates move PROPOSED → IN PROGRESS → SHIPPED /
REJECTED (with the number that killed them).

Related docs (detail lives there, verdicts live here): plans/ball_perf_plan.md,
plans/collision_sync_perf_plan.md, GPU_CPU_OPT_PLAN.md, plans/multi_ball_plan.md.

---

## Cost model (established facts — argue with data only)

- Backend: **CPU Taichi** + Vulkan GGUI on Intel Arc 130V iGPU, 1080p.
- Every kernel launch or scalar-return kernel = Python↔Taichi sync **~0.1-0.2ms**.
- Python-side Taichi field element read/write = **~26µs each** (benchmarked).
  Fix: numpy staging + one from_numpy / to_numpy.
- GGUI uploads **full field capacity** every frame (~2.5ms/MB) regardless of
  active count; each `scene.mesh` call ≈ **2ms fixed** on this iGPU.
- Physics is fixed-rate; render at 26 FPS ⇒ **~2.4 physics substeps per render
  frame**. Physics cost per render frame scales with substeps ⇒ every render-side
  ms saved also cuts physics ms (compounding), and vice versa (the spiral).

## Measurement protocol (all learned the hard way)

1. Run-to-run variance is HUGE (combat intensity, 50→61 FPS same scene):
   **3× runs, take median**. Single runs lie.
2. **Check for zombie game instances** before every canary (contaminated ≥2 runs).
3. Back-to-back runs thermal-throttle — decisive numbers need a rested machine.
4. Agent-run canaries must be window-FOCUSED (SetForegroundWindow poll script).
5. Ball-section sub-timer attribution requires **--perfsync** (Taichi launches are
   async; without it stamp cost lands on the next sync = misattributed into
   ball_batch_check). Perfsync runs are diagnostic-only — strip for A/B.
6. User feel-test gates every perf change that touches physics ordering/staleness.

## DO-NOT-RETRY list (measured losers)

- **GPU/Vulkan physics backend**: physics 5ms → 50ms (serialized cleanup loops,
  atomics, O(N²) silk). GPU_CPU_OPT_PLAN.md has the phase plan IF ever revisited;
  parallel-on-GPU has burned us repeatedly — treat as last resort, not a lever.
- **Batching the cache-amortized floor lookup** (July-7): replaced "0 launches
  most steps" with "1 launch every step" — WORSE. Never batch a path that
  usually skips. `check_floor_collision` beetle+ball paths stay as-is.
- **Fix 2 brute-force batched cluster scan** (2026-07-16): replaced the hash
  pipeline with a 77×77×40 box scan per pair — 22.5ms, WORSE. Never replace a
  clever algorithm with a big scan; batch OVERHEAD only.
- **Dropping the secondary point light** (2026-07-08): within noise on Arc 130V.
- **Removing bots**: physics-only ~3ms, fixed render tax unchanged — no FPS move.
- **Shrinking MAX_VOXELS (16000)**: headroom is real — 4P peak ~8813 battle
  voxels + up to 8000 background share the buffer; overflow silently drops.
- **MB-PERF analytic-detection-replaces-kernels** as a perf play: ball_physics
  did not move (15.7→15.5); kept for architecture only. Also FAILED 3× on
  correctness: grid detects, analytics measure.

## Historical attempt log (pre-doc, summarized)

| Date | Change | Result | Verdict |
|---|---|---|---|
| 07-07 | Batch floor lookup kernel | worse (amortized path) | REVERTED |
| 07-12 | Pair GATE batching (mandatory launches) | collision 5→1.5ms | SHIPPED |
| 07-16 | Fix 1: pack ~10 scalar reads → one to_numpy per pair | collision 17→~10ms, 20→35 FPS best | SHIPPED |
| 07-16 | Fix 2: batched brute-force cluster scan | 22.5ms, worse | REVERTED |
| 07-18 | P0 six ball sub-timers + --perfsync flag | attribution fixed | SHIPPED |
| 07-18 | P1 numpy fill fix (both batches) + stamp gate | frame 52→39.4ms, 19→25 FPS | SHIPPED 62ade26 |
| 07-18 | Spiral breaker (--nospiral to disable) | fires 14% of substeps, insurance | SHIPPED 3109e0c |
| 07-08 | Point-light drop A/B | within noise | REJECTED |

---

## BASELINE 2026-07-24 (log perf_20260724_095535_4p, --perfsync OFF)

4 rhinos (1 player + 3 bots), 3 balls, online-host, 1080p. **26 FPS, frame 37.86ms
avg (max 51)**, physics_iters 2.4/render.

| Bucket | ms | Notes |
|---|---|---|
| physics | 19.29 | = ball 7.12 + beetle_collision 7.01 + floor 3.78 + particles/misc ~1.4 |
| scene_render | 9.29 | scene_draw 4.79 (particles 1.71 + floor_mesh 1.55 + shadow 1.36), extract_all 2.12 |
| beetle_render | 3.87 | grid-stamp kernels, not uploads |
| ball_render | 2.17 | grid-stamp, has skip-if-unchanged guard |
| gui | 1.22 | full SETTINGS panel drawn every play frame |
| background/anim/clear/camera | ~1.4 | |

Ball section (sums exactly to ball_physics 7.12): batch_check 4.42,
midtick_stamps 2.12, misc 0.33, responses 0.22, manifold+ball_ball 0.02.
**Attribution caveat: --perfsync OFF ⇒ some stamp cost is hiding inside
batch_check's readback sync.** Collision diags: pairs 12.15/frame checked,
**pairs_culled 0** (cull threshold 77 > arena diameter 64 — structurally dead).

Tracking completeness: top buckets sum to ~37.2 of 37.86; ball sub-timers sum
exactly. **Nothing material is untracked** — the frame is fully accounted for.

---

## CANDIDATE LEDGER (2026-07-24 audit: floor, render, collision-sync agents)

Ranked by expected ms/render-frame ÷ risk. "Pure" = zero behavior change.

### Tier 1 — do these first (~4-6ms combined, all low risk)

| # | Candidate | Where | Est. save | Risk | Status |
|---|---|---|---|---|---|
| 1 | **Bound batch kernel ndrange to pair_count.** `check_collision_pairs_kernel` iterates a FIXED 18×77×77 = 106,722 cells per launch (MAX_COLLISION_PAIRS×PAIR_TILE²) regardless of active rows (~3.18) — **~82% dead iterations**, every substep, both call sites (ball 22651 + beetle 25512). Kernel at `beetle_physics.py:5546-5556`. Caveat: ndrange bounds from a runtime arg may need the arg passed as int (recompile-on-type gotcha); verify Taichi handles dynamic ndrange without per-value recompile — else guard cheaply with an early `continue` restructure. | bp:5555 | **~2-3ms** | Pure | PROPOSED |
| 2 | **Merge shadow mesh into floor-mesh tail.** Shadow discs are 297 verts but a separate scene.mesh call = ~2ms fixed iGPU overhead. Floor mesh already uploads full capacity + has the dome tail-packing precedent (renderer.py:42-114). Shadows rebuild per frame vs cached floor block — write into reserved tail past MAX_FLOOR_VERTS+SKY_DOME_VERTS, bump index_count. | renderer.py:1930→1909 | **~1.0-1.3ms** | Pure (zero visual) | PROPOSED |
| 3 | **Batch the 4 per-slot lowest_point kernels → 1 launch + packed readback.** The one mandatory-every-substep per-beetle launch storm left (bp:25060). MUST keep per-slot geometry — each kernel closure captures a different geo field set (bp:8382-8399); stack slot-indexed or branch on slot, else "everyone uses blue's geometry" bug. | bp:25060 | **~0.8-1.8ms** | Pure if per-slot geo kept | PROPOSED |
| 4 | **Fuse the 3 cluster kernels per colliding pair → 1.** occupied×2 + collision_point (bp:16590/16592/16594) already write one packed field; fuse launches. | bp:16590 | **~0.5-1.0ms** | Pure | PROPOSED |

### Tier 2 — real but smaller / conditional

| # | Candidate | Est. save | Risk | Status |
|---|---|---|---|---|
| 5 | GUI collapse toggle for SETTINGS panel during play (mirror show_beetle_tuning gate; panel spans bp:27586-29800, drawn every frame) | up to ~1.0ms *when collapsed* | HUD hidden while toggled | PROPOSED |
| 6 | Dedicated small-tile kernel for ball rows (77×77 tile sized for horn reach; ball needs ~40×40) | ~0.3-0.6ms | Pure if tile ≥ true reach | PROPOSED |
| 7 | Stage owner_* per-slot field writes through numpy (bp:26306-26327, 26530-26539; ~16-21 × 26µs) | ~0.3-0.5ms | Pure | PROPOSED |
| 8 | MAX_FLOOR_QUADS 4000→~3300 (observed 3038; overflow guard warns, holes if exceeded) | ~0.3ms | Low-mod (visual on overflow) | PROPOSED |
| 9 | Fix dead cull 77→~52 (true max interaction ~48; arena diameter 64). Hygiene — expect <0.2ms | <0.2ms | Pure if ≥48 | PROPOSED |

### Tier 3 — bigger levers, real trade-offs (user decision)

| # | Candidate | Est. save | Trade-off |
|---|---|---|---|
| 10 | `--res 900` at launch (already exists, bp:169-193) — cuts raster share of scene_draw only | variable | sharpness |
| 11 | Narrow extract_voxels Y-scan (1,100)→(1,~60) — 1.52M cells/frame today (renderer.py:1026) | ~0.5-0.9ms | clips anything above cutoff (high throws / tall stacks) — needs height audit first |
| 12 | Dirty-region extract (stop rescanning the whole grid to pull back what beetle_render just stamped) | big, unquantified | large project; only if Tier 1+2 aren't enough |

Note: runtime render-scale slider is NOT cheap in GGUI (window size = framebuffer
size, swapchain recreate) — launch-time --res presets are the lever.

### Next diagnostic before any ball-section work

Re-log the same scenario **with --perfsync** to un-smear batch_check vs
midtick_stamps attribution (candidate 1 is justified either way — the dead
iterations are structural — but honest numbers gate candidate 6 and any further
stamp gating).

## Projection

Tier 1 alone ≈ 4-6ms off a 37.9ms frame → ~32-34ms. Compounding: substeps/render
drop from 2.4 toward 2.0, which cuts physics-per-frame by a further ~8-15% —
realistic landing zone **~30-33ms ≈ 30-33 FPS** for this worst-ish-case scene,
better in normal play. 60 FPS at 1080p on this iGPU is NOT reachable without
Tier 3 #12 or the (do-not-retry) GPU story.

---

## ATTEMPT LOG (append every attempt below)

| Date | Change (candidate #) | Before → After (3× median) | Verdict |
|---|---|---|---|
| | | | |
