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
| 1 | **Bound batch kernel ndrange to pair_count.** `check_collision_pairs_kernel` iterates a FIXED 18×77×77 = 106,722 cells per launch (MAX_COLLISION_PAIRS×PAIR_TILE²) regardless of active rows (~3.18) — **~82% dead iterations**, every substep, both call sites (ball 22651 + beetle 25512). Kernel at `beetle_physics.py:5546-5556`. Recompile caveat resolved: `pair_count` was already a ti.i32 kernel arg (compiled per signature, not per value) and both call sites pass plain Python `len(...)` ints. | bp:5555 | **~2-3ms** | Pure | IMPLEMENTED 2026-07-24, awaiting A/B |
| 2 | **Merge shadow mesh into floor-mesh tail.** Shadow discs are 297 verts but a separate scene.mesh call = ~2ms fixed iGPU overhead. Implemented via FIXED-SIZE shadow index block between dome and floor blocks (index_count can only truncate the LAST block): unused disc slots collapse to a hidden degenerate point (zero-area tris rasterize free), floor stays the variable tail. build_shadow_discs now writes the floor-field tail at SHADOW_VERT_BASE and always runs (clears inactive slots). Watch for: shadows on ice/stepped floors, moving-hole clipping, title→game transitions, sphere-floor + --nodome legacy path. | renderer.py | **~1.0-1.3ms** | Pure (zero visual) | IMPLEMENTED 2026-07-24, awaiting A/B |
| 3 | **Batch the 4 per-slot lowest_point kernels → 1 launch + packed readback.** Implemented as lowest_point_batch_kernel: ti.static-unrolled per-slot blocks each closing over that slot's OWN geometry caches (identical rotation math), results in lowest_point_batch[4], one to_numpy. Dispatch: batch at ≥2 needed slots, old single call at 1 (batch = 2 syncs, single call = 1 — batching a lone call would be a loss). Floor loop split into lookup pass → batch → apply pass; nothing between mutates beetle state. | bp:8517+ / floor loop | **~0.8-1.8ms** | Pure if per-slot geo kept | IMPLEMENTED 2026-07-24, awaiting A/B |
| 4 | **Fuse the 3 cluster kernels per colliding pair → 1.** Implemented as cluster_pair_kernel: bodies extracted to ti.func (_occupied_scan / _collision_point_scan, single copy of logic), legacy kernels kept as thin never-called wrappers, warmup swapped to the fused kernel. | bp:cluster_pair_kernel | **~0.5-1.0ms** | Pure | IMPLEMENTED 2026-07-24, awaiting A/B |

### Tier 2 — real but smaller / conditional

| # | Candidate | Est. save | Risk | Status |
|---|---|---|---|---|
| 5 | GUI collapse toggle: HIDE PANEL button collapses SETTINGS window to FPS box + PANEL restore button (`hide_settings_panel` / `gui_panel_off`); standalone HUDs (LIVES, net debug, beetle tuning, disconnect) stay visible | up to ~1.0ms *when collapsed* | settings hidden while toggled | IMPLEMENTED 2026-07-24, awaiting A/B |
| 6 | Ball-row small-tile kernel `check_collision_pairs_kernel_ball`: 25×25 tile CENTERED ON THE BALL (contacts can only exist in ball-occupied columns → detection-identical, not approximate). BALL_PAIR_TILE=25 covers Ball Radius slider max 10 (+margin); MUST stay > 2*max_radius+2 if the slider cap ever rises | ~0.3-0.6ms | Pure (tile ≥ ball footprint by construction) | IMPLEMENTED 2026-07-24, awaiting A/B |
| 7 | Stage owner_* per-slot field writes through numpy (bp beetle_render/ball_render; ~16-21 × 26µs) | ~0.1-0.2ms net | Pure | REJECTED 2026-07-24: ~0.4ms of writes replaced by ~0.3ms of from_numpy syncs — a wash, not worth the churn |
| 8 | MAX_FLOOR_QUADS 4000→~3300 (observed 3038; overflow guard warns, holes if exceeded) | ~0.3ms | Low-mod (visual on overflow) | REJECTED 2026-07-24: floor-hole risk for 0.3ms is a bad trade |
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
| 2026-07-24 | #3 lowest_point batch + #4 cluster fuse + #5 panel toggle + #6 ball small tile (stacked; logs 113356/114050) | **Cumulative day total: frame 37.86→24.26ms (26→~41 FPS pace), physics 19.3→8.5, iters 2.4→1.6, ball_batch_check 4.42→0.78, floor_collision 3.78→1.70, beetle_collision 7.01→4.36, frame max 51→31.5.** floor_collision beat the substep-scaling expectation (2.33 proportional → 1.70 measured = real #3 gain); scene variance caveat applies to ball numbers. | **WIN (stacked)** |
| 2026-07-24 | #2 shadow mesh merged into floor+dome call (fixed-size degenerate-padded index block; renderer.py) | shadow_draw 1.36→0.21ms, floor_mesh_draw 1.55→1.22, scene_draw 4.79→3.07 (last-frame breakdowns; log 20260724_110620). Frame avg 31.2→32.0 BUT polluted by a 176ms gui hitch + 5× debris vs prior run — mechanism-level win is unambiguous, frame-level ≈ −1ms after noise. User to confirm shadows look right (ice ring, hole, sphere floor). | **WIN (mechanism confirmed)** |
| 2026-07-24 | #1 ndrange(MAX_COLLISION_PAIRS,…) → ndrange(pair_count,…) in check_collision_pairs_kernel (bp:5555; guard `p < pair_count` dropped as redundant). | frame 37.86→31.21ms (26→30 FPS), physics 19.3→14.2, iters 2.4→2.0, batch_check per-launch 2.09→1.46ms, ball_batch_check 4.42→2.67 (log 20260724_101843 vs 20260724_095535). Caveat: single run, lighter ball combat than baseline (ball_ball_hits 245 vs 2189) — per-launch 2.09→1.46 is the robust signal. | **WIN (provisional, 1 run)** |
