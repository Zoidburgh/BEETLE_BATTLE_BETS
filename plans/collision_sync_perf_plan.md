# Collision Sync Perf Plan — kill the per-pair round-trip storm

Written 2026-07-16 from focused canaries (stag,stag,hercules,hercules):
beetle_collision 17.0-17.5ms/frame vs ~2ms July-12 baseline, FPS ~20.

## The diagnosis (what the canaries showed)

Per-collision cost did NOT regress — collision FREQUENCY did, and it did
so because the July 14-16 fixes WORK: all-slot neighbor rescue, venom
voxels included, TIP_FACTOR keeping pseudo-tip contact engaged instead of
bouncing apart, more segments producing sustained shaft/svs responses.
Bots now grind in near-constant contact (~1.4 voxel-collisions per
SUBSTEP, ~5.5 per rendered frame; shaft_penetration_fixes 1030,
shaft_vs_shaft 1990 per run).

Each colliding pair pays a ROUND-TRIP STORM on the CPU backend: every
kernel launch and every scalar field read ([None]) is a full Python <->
Taichi synchronization (~0.1-0.2ms). Per pair per substep:
  2x calculate_occupied_voxels_kernel (launches)
  1x calculate_collision_point_kernel (launch)
  ~10 scalar reads (point xyz, count, has_horn_tips, tips_b1, tips_b2,
     hook, + assorted)
= ~13 round trips x ~0.15ms = ~2.4ms per colliding pair — measured, it
matches. 5.5 collisions/frame x 2.4ms = the missing ~13ms. The compute
inside the kernels is trivial; we are paying for the CONVERSATION, not
the work.

PRECEDENTS (memory, both must be honored):
- Pair GATE batching (July 12) took collision 5ms -> 1.5ms: batching WINS
  where launches are mandatory every step. Sustained contact has now made
  the cluster kernels mandatory-every-step too.
- Floor-kernel batching (July 7) made things WORSE because the old path
  was cache-amortized (rarely ran). Do not batch anything that usually
  skips.

## FIX 1 — pack the outputs: ~10 reads -> 1 read per pair (~1 hr)

The cluster kernel writes its ~10 scalar outputs into ONE small packed
field: collision_pack = ti.field(ti.f32, shape=(12,)) with slots
[x, y, z, count, has_tips, tips_b1, tips_b2, hook, spare...].
beetle_collision reads it with a single to_numpy() (one sync) and
unpacks locally. Detection/response logic byte-identical — pure data
movement. Ball pairs (single-pair path) benefit automatically.
EXPECTED: ~8 fewer syncs x 5.5 pairs/frame ~= 5-7ms/frame back.

## FIX 2 — batch the cluster across pairs: N launches -> 1 (~half day)

Replace the per-pair occupied-list + spatial-hash + cluster pipeline with
ONE batched kernel per substep, mirroring check_collision_pairs_kernel's
proven shape: it already knows how to scan each pair's 77x77 intersection
box in a single launch. cluster_pairs_kernel(pair_count) runs the SAME
column scan accumulating per-pair outputs into a (MAX_COLLISION_PAIRS x
12) field; the host launches it once for the pairs whose gate hit, then
does ONE to_numpy() for all pairs.
- The occupied/hash intermediate dies entirely (two launches per pair
  gone; the hash existed only to make the per-pair scan O(N+M), which
  the batched tile scan replaces).
- Responses then consume their pair's row — no other logic changes.
- The BALL pair stays on the single-pair path in this fix (it already
  gets Fix 1's packed read); folding it into the batch is an optional
  follow-up if ball-heavy scenes ever show the same storm.
EXPECTED: response-side sync cost -> ~1-2 launches + 1 read per SUBSTEP
total. Combined target: beetle_collision 17ms -> ~4-6ms (batch_check 3.6
stays), frame ~44ms -> ~30ms, FPS 20 -> ~30+ in the perma-grind worst
case (normal play has far less sustained contact and gains less).

## Invariants & testing

- ZERO physics change intended: identical inputs -> identical outputs,
  only fewer round trips. Quality metrics must match within run variance
  (new baselines: deep_clip 257/341, horn_cross 900/1014, min shaft
  center 0.2 across the two 2026-07-16 focused runs).
- Canary protocol per fix: 3x 75s focused runs, median FPS +
  beetle_collision ms + quality metrics; compare to the 17ms baseline.
  (Agent-run canaries MUST be window-focused — background runs throttle;
  use the SetForegroundWindow poll script.)
- User feel-check after both: any behavior difference is a BUG.
- Taichi gotchas in play: kernel signature changes recompile once
  (offline cache absorbs after); keep arg types stable (int()/float());
  branch-scoped vars declared before conditionals.

## Order

Fix 1 -> canary x1 (sanity) -> Fix 2 -> canary x3 median + user feel
check -> commit (single commit; the plan doc records both baselines).

## OUTCOME 2026-07-16

- FIX 1 SHIPPED (packed read): best observed 35 FPS / 9.95ms collision
  (vs 20-21 FPS / 17ms before). Kept — theoretically strictly better.
- FIX 2 TRIED AND REVERTED: the batched cluster replaced the hash-based
  O(N+M) per-pair scan with a brute-force 77x77x40 box scan per colliding
  pair — measured 22.5ms (worse), echoing the July-7 floor-batching
  lesson: batch mandatory OVERHEAD, never replace a clever algorithm
  with a big scan. (Measurement partly contaminated by a zombie game
  instance — see below — but the mechanism stands; do not retry this
  design. A future fix 2 would need to batch the EXISTING hash pipeline,
  not replace it.)
- MEASUREMENT CAVEATS learned the hard way: (1) run-to-run combat
  intensity varies hugely (horn_cross 900-1446) and directly drives
  collision cost — 3x medians are mandatory, single runs lie; (2) check
  for ZOMBIE game instances before every canary (a leftover 600MB python
  contaminated at least two runs); (3) back-to-back runs accumulate
  thermal throttle — decisive numbers need a rested machine; (4) agent
  canaries must be window-FOCUSED (SetForegroundWindow poll script).
- DECIDING MEASUREMENT still owed: 3x median of the current build on a
  cold machine vs the 17ms pre-fix-1 baseline.
