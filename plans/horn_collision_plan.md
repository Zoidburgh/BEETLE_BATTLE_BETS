# Robust Horn Collision — Implementation Plan (fixes 1+2+3)

**Status:** planned, NOT implemented. Written 2026-07-12 from the 3-agent clipping research
(segment coverage, response dead zones, voxel-layer timing). Line refs drift — grep anchors.

## Goal

Kill the three root causes of horn clipping WITHOUT per-beetle patching and WITHOUT
changing game feel:
1. Multi-arm horns modeled as one line (stag left pincer / hercules top jaw / rhino
   prong splay / atlas stationary horns have zero anti-clip coverage)
2. Separation pushes are horizontal-only (perpendicular over/under crossings can
   never be resolved; exact intersections excluded by `0.01 < d` gate)
3. Compound rotation tunneling: body turn + collision spin + horn yaw ≈ 5.2 vox/tick
   beats every detection window (4.5–6.5); no swept check since e24ff7b removed the
   predictive gate; controlled turn invisible to contact-velocity math

## Design principles (how we avoid per-beetle patches)

- **The ONLY per-type thing is DATA: the segment list.** `horn_collision_segments()`
  already returns a list (giraffe returns 2). We extend it to return N segments per
  type, each with its own radius — every consumer stays a generic loop. Per-type
  geometry = data rows; response code = universal. (Repo lesson from Nov 2025:
  fix response geometry once, never patch per-type symptoms.)
- **No new hard blocks.** Everything stays proportional/resisted — the 2026-07
  history proved soft responses beat hard blocks on BOTH feel and clip metrics.
- **Tip-vs-tip battle path untouched** in all phases (`calculate_min_horn_distance`
  and the lift-advantage system own combat feel — we don't touch them).
- **Each phase = one commit, user playtest gate between phases**, canary numbers
  before/after. Any phase revertible alone.

## Feel-preservation hard constraints (check at every phase)

- Engagement/burial turn resistance: same response curves, only more accurate inputs.
- Yaw-grind push/lift mechanic unchanged (our vertical separation must stay small
  relative to YAW_GRIND_LIFT so the intentional mechanic dominates).
- At slow speeds, Phase 3's expanded thresholds converge to today's exact values
  (expansion ~ speed, so slow play is bit-identical behavior).
- New constants get physics_params sliders for live tuning.

---

## Phase 0 — Baseline + short-canary harness

The perf log already records `collision_stats` (min_shaft_center_dist, deep-clip %,
max_contact_cluster). Known baselines from history: deep-clips 1.2–1.8% of frames
(best 0.4% giraffe-only canary).

- Canary protocol (user constraint: SHORT runs, results are indicative only):
  **60–90 seconds per run, 3 runs, take the median.** Bots get caught in loops —
  never trust a single run, never run 5 minutes.
- Runs: (a) standard `--local4` bot mix, (b) one stag-heavy run if bot horn types
  can be forced cheaply (stag is the worst-coverage type — its numbers matter most).
- Record baseline BEFORE any change: deep-clip %, min_shaft_center_dist, FPS,
  beetle_collision ms (Phase 1 adds segment-pair math — watch for perf regression).
- Acceptance across the whole plan: deep-clip % trending toward <0.5%, FPS within
  run-to-run noise, and the USER's playtest is the only judge of feel.

## Phase 1 — Multi-segment horn skeletons (data change, response code untouched)

Extend `horn_collision_segments(beetle)` to return a list of
`(x1,y1,z1, x2,y2,z2, base_r, tip_r, articulates)` per type:

| Type | Segments | Source of endpoints | Notes |
|---|---|---|---|
| stag | 2 — base→LEFT tip, base→RIGHT tip | `calculate_stag_pincer_tips` (already models both; currently only used by tip path) | chord per arm first; split at elbow later ONLY if canary still shows elbow clips |
| hercules | 2 — top jaw, bottom jaw | `calculate_hercules_jaw_tips` | top jaw does NOT pitch, bottom does — per-segment articulation flag handles this generically |
| rhino | 3 — shaft base→fork, fork→left prong, fork→right prong | `calculate_rhino_prong_tips` | or 2 overlapping base→tip chords if simpler |
| atlas | 3 — cephalic (articulates), 2 pronotum (STATIC: skip horn pitch/yaw) | pronotum geometry ~7288; cephalic ~7351 | the static flag is data, not special-case code |
| giraffe | 2 (unchanged) | existing | already proven pattern |
| spider/scorpion/bombardier | 1 (unchanged) | existing | |

- **Per-segment radii replace the global backwards taper** (`6.5 - 2.5*max(s,t)`):
  threshold for shaft-vs-shaft becomes `r1(t1) + r2(t2)` from each segment's own
  base_r→tip_r. Pincer arms are thin (~2–2.5); shaft bases thick (~3). This fixes
  "taper narrows where horns physically widen" for every type at once.
- Consumers to convert to the generic loop (all already loop or take segment lists):
  shaft cylinder (b1/b2, ~14590/14617), shaft penetration (~14739),
  shaft-vs-shaft (~14896), `_closest_on_horn_segments`.
- Verification: standalone script asserting each type's segment endpoints coincide
  with the true tip helpers across a pitch/yaw sweep grid (they must match, since
  they come FROM those helpers). Perf check: stag-vs-stag = 2×2 segment pairs
  (16 with elbow split — start without).
- Feel risk & gate: pincers/jaws now physically stop where they visually are —
  stag battles will feel "wider". That's the point, but user playtests before
  Phase 2. Watch: bots wedging pincers (loop artifact) — grain of salt.

## Phase 2 — True 3D separation (perpendicular crossings)

In shaft-vs-shaft response (~14928):
- Use the full 3D connecting vector between closest points (keep X/Z AND Y).
- **Degenerate: exact intersection (d < 0.01)** — currently excluded entirely.
  Fix: separation axis = `cross(dir1, dir2)` normalized (the natural mutual
  perpendicular of two crossing lines), sign from previous-tick relative Y (or
  body height difference). Remove the `0.01 <` exclusion; at d≈0 push full depth.
- **Degenerate: parallel segments** — closest_points pins s=0 (mislabels mid-shaft
  grind as basal); clamp/midpoint handling.
- Feel guardrails:
  - Vertical separation is POSITION-only, same magnitude scale as today's
    horizontal push — never inject vertical velocity (the lift system stays the
    only source of launch vy; no new free-lift exploit).
  - Never push a grounded beetle's position down (floor will fight it — clamp
    the -Y component when on_board).
  - Slider: `SVS_VERTICAL_SCALE` (default 1.0, 0 = today's horizontal-only
    behavior = instant A/B escape hatch).
- Gate: user playtests perpendicular grind battles; yaw-grind lift must still feel
  dominant.

## Phase 3 — Velocity-expanded thresholds + controlled-turn credit (tunneling)

- **3a. Credit the controlled turn.** Track `beetle.controlled_turn_rate`
  (rotation delta written by input this tick / dt — includes the ROTATION_SPEED
  path, hover spin, tornado yaw need not be included). Add it to
  `angular_velocity` wherever contact sweep velocity is computed (~14962-14965),
  so grinding turns get velocity damping, not just position pushes (fixes
  re-penetration every tick).
- **3b. Velocity-expanded windows.** Per pair per tick compute relative tip sweep
  speed: `(|omega_body_1| + |ang_vel_1|) * tip_radius_1 + |horn_yaw_vel_1| * shaft_len_1`
  (+ same for b2), in vox/tick. Expand `_svs_thick` and `SHAFT_CONTACT_DIST` by
  `min(sweep_vox_per_tick, SVS_EXPANSION_CAP)` (cap ~6, slider). Response
  magnitude still scales with true distance, so an "early caught" contact gets a
  gentle nudge — no feel cliff, and at slow speed expansion→0 = today's behavior.
- **3c. Minimal decoupling from the stale voxel gate** (needed for 3b to matter:
  all horn responses currently require `contact_count > 0`, but the voxel grid is
  stamped once per rendered frame from interpolated angles — fast sweeps tunnel
  between snapshots). Change: run the shaft-vs-shaft check when EITHER voxel
  contact fired OR the expanded segment-distance check passes (segments use
  CURRENT angles — this is the fresh data path). Gate it on the existing pair
  distance cull so cost stays bounded. Body/impulse responses stay voxel-gated
  (unchanged).
- Gate: canary especially watches false-positive pushes ("phantom shoves" while
  visually apart) — expansion cap + closing-velocity requirement (only expand when
  segments are approaching) prevent this; verify in playtest.

## Explicitly OUT of scope (don't drift into these)

- Tip-vs-tip battle / lift-advantage system — untouched.
- Re-rasterizing the voxel grid per substep (perf) — 3c sidesteps it for horns.
- The slots-2/3 neighbor-rescue bug + diagonal neighbor gap (real bugs, separate
  small fix, don't bundle — noted for later).
- Reinstating any hard predictive block.

## Order & gates

Phase 0 (baseline) → Phase 1 (segments) → user playtest → Phase 2 (3D separation)
→ user playtest → Phase 3a/3b/3c → user playtest + canary comparison vs Phase 0.
Each phase one commit. If any phase feels wrong: its slider to 0 / revert that
commit alone.
