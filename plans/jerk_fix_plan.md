# Jerk Fix Plan — instant spins + unexplained pop-ups

2026-07-24. Target symptoms (user report): (1) beetle spins hard in 1-2 frames
when hit at certain angles, (2) occasional pop-ups with no visible cause.
Diagnosis mapped to plans/smoothness_audit_2026-07-20.md backlog (the half that
was NOT implemented in smoothing v2); both prime suspects re-verified live in
current code 2026-07-24.

## Design rules (from the combat-smoothing retrospective — non-negotiable)

- **One slider per channel, old-feel-at-0.** The single-shared-knob approach
  made tuning opaque and got rewound. Every phase below adds its OWN dial.
- **Discrete impact events stay discrete.** We smooth DELIVERY jitter and
  boundary strobing, never the existence of the hit. No continuous forces.
- **One phase per feel-test.** User plays each phase alone before the next
  stacks on it. Commit only after the feel pass.
- **Clipping is a hard gate.** Any phase that could weaken separation ships
  with before/after collision-quality metrics (deep_clip_events,
  min_shaft_center_dist, horn_cross_clip_events, min_shaft_shaft_dist — all
  already in the perf log) and reverts on regression beyond run variance.

## Explicitly OUT of scope (measured reverts — do not resurrect)

- Shaft-vs-body momentum smoothing (audit 2d): TRIED, user-reverted 2026-07-19
  ("older horn interactions better").
- Closing-speed scaling of the lift branches (audit 2e): this is the volley-
  factor concept, rewound 2026-07-20 for combat clarity.
- Soft-juggle volley: rewound, stays out.

## Phase 0 — spin/pop trace (attribution before surgery)

The rhino ball-lift lesson: 2 blind fixes failed before the trace found 4
stacked causes. Same discipline here. Add `--jerktrace`: per-substep CSV rows
whenever a beetle's |Δangular_velocity| or Δvy exceeds a threshold in one
substep, tagged with SOURCE (main-torque 19051/19067, tip-yaw 18406, grounded
yaw bias, advantage-branch id + lift_advantage value, pending_lift drain,
separation snaps, clamps). One combat session → ranked table of which source
actually fires on the user's "certain angles" moments. Cheap (mirrors
--balltrace plumbing). GATE: phases 1-3 are already justified by the audit,
but the trace decides whether phase 5/6 are needed and catches surprises.

## Phase 1 — smooth the main collision torque (audit 2a) — TOP suspect, spins

`beetle_physics.py:19051-19067`: raw `angular_velocity +=` every contact
substep, ×TORQUE_MULTIPLIER 1.95, bypasses pending_yaw AND all smoothing.
The linear sibling got Push Smooth; this got nothing. Lever-arm geometry means
glancing off-center hits deliver huge one-substep kicks — exactly "certain
angles".

- **Fix**: scale the per-substep angular impulse by the SAME style of blend
  the linear channel uses (its own EMA/limiter, NOT shared state): new slider
  **TORQUE SMOOTH** (0 = old raw writes). Impulse magnitude unchanged over the
  contact — only delivery is spread over ~2-4 substeps.
- Keep the ±8 MAX_ANGULAR_SPEED clamp as-is in this phase (it's the backstop).
- Optional sub-item if trace shows clamp snaps: lerp the clamp ceiling over
  ~0.1s instead of stepping (audit cat 4).
- **Feel risk**: LOW — same total spin, arrives a hair later. A/B at 0.
- **Clip risk**: none — torque does not separate bodies.
- Tip-yaw sibling (18406-07) gets the same treatment under the same slider IF
  the trace shows it firing (it's tip-gated, already EMA'd via Tip Smooth).

## Phase 2 — pending_lift flush on contact loss (audit 5b) — TOP suspect, pops

`beetle_physics.py:2350-2355`: pending_lift drains 20%/substep UNCONDITIONALLY
— ~15 substeps of leftover lift keep pumping after contact ends (ghost lift);
a second hit stacks onto the residue → unexplained pop.

- **Fix**: when the beetle had NO contact this substep, drain FASTER (not hard
  zero — a hard zero is its own snap): new slider **GHOST FLUSH** = no-contact
  drain rate (default ~0.6, **0.2 = old behavior** — this slider's old-feel
  end is 0.2, not 0; label it in the GUI). Same for pending pitch/roll/yaw
  residues if the trace shows them ghosting (audit 5c notes all four drains
  are hardcoded 0.2).
- "Had contact this substep" signal: the pair loop already knows
  (pair_collision_last / per-beetle contact flag) — set a per-beetle bool in
  the collision pass, read it in the drain. No new detection.
- **Feel risk**: LOW-MED — launches feel tighter-coupled to the hit; scoop
  flick sets vy directly (bat model) so ball scooping is unaffected. Watch
  charge ram pops (18997/19005 feed pending_lift DURING contact — unaffected
  by a no-contact flush).
- **Clip risk**: none — flush only fires when contact is already gone.

## Phase 3 — advantage-branch hysteresis (audit 1c) — both symptoms

lift_advantage ±0.5 picks between branches differing ~3.25x in lift with a
sign-flipped ~9x pitch torque. No hysteresis → contact geometry near the
boundary flips the whole force stack per substep. "Certain angles" is the
branch boundary itself.

- **Fix**: latch the chosen branch per pair; switch only when lift_advantage
  moves **ADV HYST** past the threshold (slider, default ~0.2, 0 = old
  knife-edge). This is the airborne_h/AIR_SLOW_LIFT latch pattern (both
  already shipped and liked). Branches stay discrete — we stop the flicker,
  not the mechanic.
- Needs per-pair state (dict keyed like pair_collision_last), reset on
  contact loss so a NEW engagement re-evaluates fresh.
- **Feel risk**: MED — near-boundary exchanges resolve one way instead of
  oscillating; who-wins can shift slightly at the exact boundary. This is the
  phase most needing the user's horn-fight feel pass (esp. atlas/hercules
  movable horns, which live near the threshold — see LIFT_LEVER history).
- **Clip risk**: none directly; watch that latching the LOW branch doesn't
  keep a buried horn from lifting out (trace + deep_clip_events metric).

## Phase 4 — per-type inertia + fix the 3-axis slider bug (audit 5a)

Amplifier, not cause: all types share moment 18.4 on all axes, so heavies
respond to raw torque like featherweights. AND the Inertia slider writes only
yaw inertia, only slots 0-1 (pitch/roll never update, slots 2-3 never update).

- **Fix A (bug, unconditional)**: slider writes all three axes on ALL slots.
- **Fix B (feature)**: per-type inertia factor in BEETLE TUNING /
  beetle_tuning.json (heavies ~1.3-1.5x, default 1.0 = today). This is a
  DELIBERATE feel change (mass legibility) — user tunes per type in-game.
- **Feel risk**: intended change; per-type dials make it tunable to taste.
- **Clip risk**: slower rotation response could marginally slow rotate-away-
  from-penetration; separation is position-based, not rotation-based, so
  expected nil — but it's in the metrics gate anyway.

## Phase 5 — grounded yaw smoothing (audit 2b) — only if trace demands

Away-from-attacker bias (momentum RATIO, fires at any speed) + geometric
torque floor inject fixed-strength yaw to GROUNDED recipients every substep;
airborne got Air Turn Smooth, grounded got nothing.

- **Fix**: **GROUND TURN SMOOTH** slider, exact analog of Air Turn Smooth
  (own dial, 0 = old). Gate on the phase-0 trace showing these sources in the
  user's actual jerk moments — phases 1+3 may already cover them.

## Phase 6 — separation depth-taper (audit cat 3) — LAST, clipping-sensitive

Position snaps (shaft push-out ≤1.5 vox/step, horn mini-sep ≤1.2, vertical
sep 0.7 un-ramped) read as teleport-pops. The ball got a depth taper
(BALL_SEP_DEPTH); beetles never did. Separations are ANTI-CLIP machinery, so:

- **Fix**: taper separation speed only for SHALLOW overlaps (< ~0.5 vox);
  deep-overlap speed UNCHANGED; keep a minimum-progress floor so any overlap
  still resolves within a few substeps. Slider **SEP TAPER** (0 = old
  full-speed snaps).
- **HARD GATE**: 3× focused canary before/after — deep_clip_events,
  min_shaft_center_dist, horn_cross_clip_events, min_shaft_shaft_dist within
  baseline variance, plus user eyeball for visible clipping. Any regression →
  revert the phase, keep the rest.

## PHASE 0 RESULTS (2026-07-24, jerk_trace.csv: 2278 rows, 3-ball+3-bot session)

The trace CONFIRMS phase 1, DISCOVERS a new pop source the audit missed,
DEMOTES phases 2/5, and adds a tilt finding. Revised order below.

- **SPIN = mtorq, case closed.** Raw main torque present in 74% of spin rows
  and explains >70% of the delta in 300/533; median 1.7 rad/s per substep,
  p90 5.4, MAX 21.5 — 2.7x the entire ±8 speed range in ONE substep. Away-
  bias median contribution 0.01 → **phase 5 DROPPED** (grounded yaw smooth
  not justified). angclamp hit only 5x.
- **POP #1 discovery: beetle-side ball-contact vertical is UNCAPPED.** 733 of
  965 "pop" rows were benign landing arrests (floor stopping a fall — trace
  artifact). Of the 232 REAL launches, `iy` is the strongest tagged channel
  (ratio 1.0), and **12 rows show |iy| up to 19.9 — far above AIR_NUDGE_CAP
  8.25**. Those ride the ball-collision branch: `impulse_y = impulse *
  normal_y * BALL_LOFT` (bp ~18808) has NO cap on the BEETLE side (only
  ×BALL_BEETLE_RECOIL 0.25). Charging into a ball can launch the beetle at
  +20 vy in one substep. NEW PHASE 2A below. (The ball's own loft is the
  feature — only the beetle side needs the cap.)
- **TILT: pending pitch/roll queues deliver up to 9.5-17.6 rad/s in one
  substep.** The 20% drain "smoothing" has no absolute cap, and advantage/
  press-down streams (tumble_mult) pump the queue huge during sustained
  contact — drain then delivers spikes. 960 tilt rows. The user's "spin a
  lot when hit" may be tumble (pitch/roll), not just yaw. NEW PHASE 2B below.
- **Ghost lift DEMOTED**: dlift explains a median 3% of pop deltas (drains
  are 0.05-0.7/substep — float, not pop). Phase 2 (flush) stays as a small
  causality polish, LOW priority.
- **Advantage flips are real but secondary**: 383 branch changes, 80 within
  ≤3 frames. In the hardest events adv was far from the boundary (|adv| 2.7-
  7.0) — branch forces contribute via the pending queues (see 2B), and the
  flip-strobe fix (phase 3) stays justified but after 1/2A/2B.
- **Unknown residue**: 11 launches with zero tags, several at exactly vy
  +6.5 — likely the untagged shaft-vs-shaft vertical (SVS_LIFT_CAP) or
  crossing-detector reversal. Add tags if it persists after the fixes.

## REVISED ORDER

1 (torque smooth — confirmed top spin fix) →
**2A (cap beetle-side ball-contact vertical — slider BALL_POP_CAP, default
~AIR_NUDGE_CAP-ish, old-feel = uncapped/high; ball side untouched)** →
**2B (per-substep delivery cap on pending pitch/roll drains — slider
TILT_DRAIN_CAP, old-feel = uncapped; queue keeps total energy, delivery
stops spiking)** →
user feel checkpoint → 3 (adv hysteresis) → 4 (inertia + slider bug) →
[2 ghost flush as polish] → [6 sep taper, gated] → 5 DROPPED.

## IMPLEMENTATION LOG

- 2026-07-24 **Phase 1 SHIPPED (uncommitted)**: TORQUE_SMOOTH_CAP slider
  "Torque Cap" (default 2.0, 0 = old raw). Direct delivery capped per
  substep, overflow routes through pending_yaw. Balls exempt. First feel
  pass: better, but user still saw the worst jerks ON BIG LIFTS → that is
  the tilt-drain channel, exactly the trace's finding #3 → 2B pulled
  forward ahead of 2A (user: no noticed ball issues, 2A deferred).
- 2026-07-24 **Phase 2B SHIPPED (uncommitted)**: DRAIN_CAP slider "Drain
  Cap" (default 2.0, 0 = old). Per-substep clamp on pending pitch/roll/yaw
  drain DELIVERY; queue keeps the remainder so total tumble is unchanged,
  spread out. Balls exempt. NOTE the two fixes compose: worst-case yaw
  delivery is now direct cap (2.0) + drain cap (2.0) = 4.0/substep, vs
  21.5 observed raw. TRADE-OFF to feel-test: a monster lift's tumble now
  PERSISTS longer at capped rate instead of dumping at 20%/substep — if
  lifts feel like they "keep tumbling too long", raise Drain Cap toward
  3-4 rather than turning it off.
- 2026-07-24 **RETUNE after "super floaty and slow" feel pass**: first
  defaults sat mid-distribution, not on the tail. Second trace session
  (post-phase-1, pre-2B) proved (a) torque cap works (mtorq pinned ≤2.0,
  was max 21.5), (b) big-lift jerk IS the drain channel (roll delivery
  spiked to 51/substep raw), (c) normal combat roll runs ~3.3 at p75, so
  Drain Cap 2.0 throttled ~1/3 of ALL tilt motion. New defaults from the
  percentiles: **Torque Cap 2.0 → 4.0** (raw p90), **Drain Cap 2.0 → 6.0**
  (above pitch-p95/roll-p90). Worst spikes still crushed ~8x; ordinary
  motion delivered in full. LESSON: set delivery caps from tail
  percentiles of a TRACE, never from the median — a mid-distribution cap
  reads as global floatiness, not smoothing.
- 2026-07-24 **PHASES 1 + 2B FULLY REVERTED (user call, immediately after
  the retune)**. What was removed: TORQUE_SMOOTH_CAP (per-substep cap on
  the main collision torque, overflow via pending_yaw) at both b1/b2
  torque sites; DRAIN_CAP (per-substep delivery clamp on the pending
  pitch/roll/yaw drains); both physics_params defaults and both COMBAT
  FEEL sliders. Physics is now byte-identical to pre-phase-1. KEPT: the
  --jerktrace instrumentation (all tags + emit; zero cost when the flag
  is off) and this plan doc. Revert-note comments left at both code
  sites pointing here.
- 2026-07-25 **FINAL OUTCOME: entire jerk-era working tree abandoned; code
  restored byte-identical to commit 5b46576 (the perf state) and user
  confirms feel is good again.** After the cap revert the user STILL
  reported floatiness and then "gravity barely there" — cause never
  isolated, but user is confident it was the jerk-era changes, and the
  clean-commit restore fixed it. IMPORTANT: that means the --jerktrace
  INSTRUMENTATION ITSELF (not just the caps) is suspect — it edited hot
  physics paths (drains, advantage branches, collision sites) and
  something in it may not have been as inert as intended. It lives in a
  git stash ("jerktrace instrumentation + lp dev flags") + a scratchpad
  backup. DO NOT stash-pop it casually — if tracing is ever needed again,
  re-review every insertion against this failure first, or rebuild it
  from scratch with a physics-diff canary (identical seed run with flag
  on/off must produce identical positions).
  WHY (for the next attempt): the delivery-cap CONCEPT reads as floaty/
  slow to this user even at tail-percentile thresholds — it is a
  resistance-shaped mechanic, and the feel compass (see memory/
  physics-feel-compass) consistently rejects those. The DIAGNOSIS stands
  unrefuted: raw one-substep torque spikes to 21.5 rad/s and drain-
  delivered tilt spikes to 51 rad/s are real and measured (jerk_trace
  sessions 11:57 + 12:12). Any future fix should reshape the SOURCE
  (e.g. advantage-branch hysteresis so the queues never get pumped that
  hard, per-type inertia so heavy beetles resist honestly) rather than
  rate-limit the delivery. Phases 3 (adv hysteresis) + 4 (inertia +
  3-axis slider bug) remain open and un-attempted.

## Order + protocol

0 (trace) → 1 (torque) → 2 (flush) → 3 (adv hyst) → user feel checkpoint →
4 (inertia) → [5 if trace demands] → 6 (taper, gated). Each phase: compile
check → user plays with slider at 0 vs default → commit on approval. All new
sliders live in the COMBAT FEEL panel next to their siblings. Perf note:
everything here is Python-side scalar math on plain beetle objects — zero
kernel changes, zero measurable perf cost (guard: no new per-substep field
access / kernel launches — the perf_plan.md cost model applies).

## Success criteria

- The reported moments (angle-hit spin, causeless pop) are visibly gone or
  clearly softened at default sliders, per user play.
- All sliders at old-feel end reproduce today's behavior exactly.
- Collision-quality metrics within run variance of today's baseline.
- No new perf cost (frame ms unchanged in a matched log).
