# Horn Tunneling Plan — fast spins / pitch flicks flashing through other horns
2026-07-16. Companion to plans/smoothness_plan.md (S1/S2 shipped same day) and the
horn_collision_plan.md history.

**STATUS 2026-07-16 (same day): T0.1 + T1 IMPLEMENTED (uncommitted, awaiting user testing).**
- T0.1: the blue/red hardcoding was ALREADY fixed (ownership-based) in the working tree
  before this session; what shipped here is the 8-direction extension (was cardinal-only).
- T0.2: resolved via option (b) — the T1 detector fires inside the tip band (no band change).
- T1 shipped: sign tracking per (slot,slot,arm,arm) in `horn_cross_sign_prev`, frame-
  consecutive flips only, near-parallel state dropped (n through zero is not a crossing),
  45-voxel pair pre-gate, interior params (0.02-0.98) + CROSSING_RANGE (4.0, slider "Cross
  Fix Range", 0 = off). Response: capped positional un-cross (≤1.5/side, XZ positional /
  Y via the S1 capped-velocity pattern, grounded-down blocked), relative-velocity reversal
  (restitution 0.3), horn_burial floored to 4.0 (existing turn clamp owns follow-through),
  one-shot angular_velocity ×0.5. Stat: `horn_crossing_fires` in collision_stats + perf log.
- T2 not built (build only if in-and-out-same-side residue shows in play).
Ghost-push regression check: bots idling horns-raised must produce ZERO fires.

**CANARY RESULTS 2026-07-16 (3x 75s stag,stag,hercules,hercules):**
First cut OVERFIRED: 965 fires/75s — in bot perma-grind, crossed shafts OSCILLATE across
each other's plane (real intersections SVS already owns), each oscillation = a sign flip,
and the response perturbation fed back into more flips. THREE GATES ADDED (committed after):
(1) voxel-contact suppression at fire time (contact steps belong to the normal stack),
(2) sweep magnitude ≥1.2 voxels/step through the plane (true tunnels are fast by definition;
oscillations move 0.2-0.5), (3) 9-frame per-pair refire cooldown.
Gated composition (run 3): **2 fires** (plausible true tunnels), 153 mag-suppressed
(oscillations), 9 contact-suppressed. Frame time identical across all runs (~23ms 4-bot,
detector cost invisible, even at 965 fires/run). deep_clip 129/193/239 across runs = normal
combat-intensity variance (post-change rebaseline; not comparable to older numbers anyway).
Suppression counters stay in the perf log: `horn_crossing_supp: contact=N mag=N`.

## The problem

At max yaw speed (MAX_ANGULAR_SPEED = 8 rad/s) a ~20-voxel horn tip sweeps ~2.7 voxels per
physics step (1/60s). Horn pitch articulation sweeps similarly fast vertically. A horn is 1-2
voxels thick, so a fast perpendicular sweep passes ENTIRELY through the other horn between two
collision checks — at no sampled instant do the segments overlap, so every layer (voxel kernel,
shaft-vs-body, shaft-vs-shaft) reports "no contact." The user sees long horns flash through
each other while spinning or pitching. This is TUNNELING, not a response-strength problem.

## History — the constraint that shapes everything (do not re-walk into this)

The July-12 overhaul's Phase 3 attacked this with a sweep-EXPANDED detection window +
no-voxel-contact SVS path. It was implemented (9fab8ca), patched (05298ae ghost-push fix),
and then REMOVED by user decision (938a137): an expanded window fires on proximity, i.e.
WITHOUT real contact, and produced ghost pushes — beetles shoved apart while visibly not
touching. That failure mode is worse than the clipping it fixed. Verdict on record:
"tunneling clips during fast turn+yaw remain a KNOWN accepted limitation."

Therefore the acceptance bar for anything in this plan:
- **A response may fire ONLY on a certain, actually-occurred crossing** — never on proximity.
- Design rule (user, 2026-07-15): per-type code describes shape/motion only; no special
  forces, no per-type contact regimes.
- Canary hygiene: deep_clip_events is the comparable metric; horn_cross rebaselines on any
  geometry/call-pattern change. 3x medians, no zombie instances, rested machine.

## Phase T0 — the two known dead zones (small, scoped, do first)

**T0.1 Thin-horn voxel neighbor rescue: cardinal-only + hardcoded blue/red.**
The voxel-layer rescue that catches 1-voxel-thin horns is dead for slots 2/3 (checks
blue/red voxel ids only, from the 2P era) and only checks 4 cardinal neighbor directions,
missing diagonal approaches. Fix: is_beetle_voxel()/beetle_owner() lookups (the Phase-4.3
pattern used everywhere else) + include the 4 diagonals. VERIFY FIRST: locate the rescue in
check_collision_kernel and confirm the above is still its current state — this description
is from the July-12 session notes.

**T0.2 Tip-band suppression dead zone.**
Shaft-vs-shaft deliberately skips crossings where BOTH segment params are in the outer
0.7–1.0 band with tips in contact (so tip battles keep original physics). A fast spin
crossing exactly tip-band-vs-tip-band has zero anti-clip coverage. Fix option (a): shrink
the suppressed band to 0.85–1.0. Option (b): keep the band but let the T1 crossing detector
(below) fire inside it — suppression was about damping real tip battles, and a full
pass-through is never a tip battle. Prefer (b): it changes nothing until a certain crossing.

Gate: bot canary, deep_clip_events vs current baseline; user feel check that tip jousts
still feel unchanged (T0.2's risk).

## Phase T1 — sign-flip crossing detection (the real tunneling fix)

**Detection.** Per beetle pair within horn reach (the pairs already gathered for the batched
collision step), per segment-arm combo (horn_collision_segments already returns the arm
lists; memoized per substep): compute the signed side of segment A relative to segment B —
sign of dot(cross(B_dir, A_tip - B_base), reference) or equivalently the sign of the
scalar triple product that closest_points_between_segments already has the ingredients for.
Store last step's sign per (pair, armA, armB). If the sign FLIPS between consecutive steps
AND the segments' closest approach at both endpoints of the step is within a modest range
(e.g. sum of radii + sweep distance), the horns crossed through each other this step.
A sign flip at range is geometrically certain contact — this can never ghost-fire on
proximity, which is exactly what P3 couldn't guarantee.

Care points:
- Sign is computed on the CURRENT poses each step; segments must be compared in a stable
  arm order (slot-sorted) so the sign is well-defined across steps.
- Reset stored signs when a pair leaves proximity, on respawn/death, and on match reset —
  a stale sign from an old engagement must not trigger a phantom crossing.
- Giraffe's chained 2-segment arm and multi-arm types (stag both pincers, hercules two jaws,
  rhino midline+prongs, atlas 3 chords) all come through horn_collision_segments already —
  loop arm combos exactly like the existing SVS block does. No per-type code.
- Articulation (horn pitch/yaw flicks) moves the segments too — detection is pose-based, so
  articulation crossings are caught by the same sign flip. No separate path needed.

**Response** (design-rule-safe, describes motion only):
1. Reconstruct the crossing: binary-search t in [0,1] between prev and current pose for the
   minimum segment-segment distance (2-3 iterations is plenty), giving crossing point and
   penetration direction.
2. Place the swept horn's owner back at the crossing pose for the ROTATIONAL component that
   caused the pass-through: clamp this step's yaw/articulation displacement to the contact
   value (i.e. the spin "hits" instead of passing through). This is a motion clamp, not an
   added force.
3. Then let the EXISTING contact stack own the frame: feed the crossing into horn_burial /
   pair contact so the depth-aware turn clamp and engagement resistance engage, and hand the
   contact point to the normal SVS/momentum-transfer path. Do not invent a new force here.
4. New tunables: CROSSING_ENABLE toggle + "Crossing Range" slider (the closest-approach
   gate). Default ON with conservative range once validated.

**Perf note.** The sign test is a handful of float ops per arm combo per proximate pair —
noise next to the packed-read collision cost. The binary search runs only on actual
crossings (rare). horn_collision_segments would need a pose-parameterized variant for the
search's interpolated poses — keep the memoized fast path for the common case. CPU-backend
lesson applies: no new per-pair kernel launches or [None] reads; this is pure Python-side
math on data already in hand.

## Phase T2 — only if T1 leaves residue: sampled mid-step check

For pairs in proximity with high relative angular speed (body yaw + articulation channel
rate), run the segment distance check at 1-2 interpolated mid-step poses in addition to the
endpoint pose. Catches grazing sweeps that reverse sign twice within one step (in-and-out
through the same side — the one case sign-flip misses). Strictly additive detection; same
response path as T1. Skip unless play shows the in-and-out case actually matters.

## What we are deliberately NOT doing
- No expanded/proximity detection windows (P3's ghost-push failure).
- No raising collision substep rate globally (CPU cost; the sync-storm lesson).
- No per-type pocket/contact regimes (design rule; multi-contact composition is the pattern).
- No thickening horn capsule radii by angular speed — that's the expanded window in disguise.

## Test plan
1. T0: bot canary 3x median (deep_clip_events), user tip-joust feel check.
2. T1 repro before/after: stag or giraffe (longest horns) spinning at full yaw rate through
   an opponent's raised horn; also rapid horn-pitch flicks across a locked horn. Watch
   min_shaft_center_dist and deep-clip % in the perf log, and count crossing-detector fires
   (add to collision_stats).
3. Ghost-push regression watch: bots idling near each other with horns raised must produce
   ZERO crossing fires (it only fires on sign flips — any fire without visible contact is a
   bug by definition, log position when it happens).
4. Networking: pure host-side physics (guests see results via state sync) — no protocol
   impact.
