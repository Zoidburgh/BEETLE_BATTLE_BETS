# Abnormal-Body Collision Plan — bombardier, scorpion, spider

Written 2026-07-14 from a full bombardier audit, extended with scorpion and
spider audits. Revised 2026-07-15: progress marked, and the design rule
made explicit after review with the user.

## DESIGN RULE (the contract)

Venom, silk and acid are the ONLY "special abilities." Everything else is
ONE set of contact physics applied to each beetle's TRUE geometry and TRUE
motion. Per-type code may only ever DESCRIBE shape and motion (where the
voxels are, what pivot they rotate around, how fast) — never add a new
force type, scripted move, or per-type feel knob. If a tail smash or head
scoop feels like a move, it must be emergent from the same
rate-x-lever-arm formula horns already use.

Consequence applied in this revision: the previously proposed per-type
"scoop strength" tunable is CUT. The bombardier scoop rides the same
universal contact tunables (SHAFT_PENETRATION_LIFT etc.) as every horn.

## First principles: the four blanket failure modes

The VOXEL layer (stamped grid) is always right — baked slants, tail
rotation, aim tilts, gait all render into it. The ANALYTIC layer
(capsules/segments) drives ball push-out depth, ball surface normals,
shaft/svs responses, and motion credit — and it was built HORN-FIRST.
Every unique beetle breaks that template the same four ways:

- **F1. Shape coverage** — off-template regions (heads, claws, tails,
  antennae, stilt legs, fat abdomens) have no analytic presence -> ball
  ghosts/slips. Principle: derive shapes from the generation constants,
  declared per type in one place.
- **F2. Motion channels** — DOFs outside horn pitch/yaw (tail swing, aim
  tilts) were invisible: moving geometry carried zero momentum, which was
  the PHYSICALLY WRONG special case. Principle: every shape has a channel
  (pivot + angle + rate); position transform and velocity credit
  (v = rate x lever) come from one mechanism.
- **F3. Combat classification** — tip COLOR flags escalated pseudo-weapons
  (antennae, fangs, claw tips) into full horn-joust physics: an unearned
  hidden special. Principle: per-type TIP_FACTOR scales the escalation
  back toward body contact (bombardier 0.3, spider 0.6, scorpion 0.7,
  horn beetles 1.0). This REMOVES specialness, not adds it.
- **F4. Raised stances / arches** — missing legs only matter where a
  ball-sized gap exists (bombardier front +4, scorpion rear +4, spider
  arches). Principle: generate leg struts from leg-generation constants,
  static stance pose.

## Status

- [x] 0. Commit checkpoint of the ball-collision overhaul (2ea2c0e)
- [x] 1. TIP_FACTOR table + per-side tip attribution in the contact-point
      kernel (bystander tips no longer escalate a pair). Applied at: 0.65
      engagement floor, tip leverage, spin-bias exemption, tip lock
      separation. Geometry gates (svs param limit, ball shaft rescue)
      deliberately untouched.
- [x] 2. Seam normal blending in _ball_surface_contact: penetration-
      weighted average of all shapes within 1 voxel of the deepest
      (winner-takes-all flipped directions at shape junctions).
      GATES 1+2 PASSED (user playtest 2026-07-14: "seems ok").
- [x] 3. Motion channels: _tail_sweep_velocity helper (rate x lever around
      the rear pivot) credited in the shaft path AND horn-vs-horn
      crossings; beetle.tail_vel tracked at input. Spider abdomen capsule
      follows spider_aim around the pedicel pivot (position);
      spider_aim_vel tracked, velocity credit deferred to step 7 (needs
      the abdomen as a segment). Bombardier aim channel was the session-1
      prototype (spray_aim_vel).
      GATE 3 PENDING — checklist below.
- [x] 4. Scorpion claw split: L/R claw chords through the arm + claw mass
      (the old single chord ran along z=0 — the GAP between the claws),
      rotated by horn pitch/yaw around the claw pivot (2,2) (default pitch
      +20 deg). Scorpion is now 4 segments: claws 0-1 (parallel arms,
      articulate), tail 2-3 (chained, tail channel); horn_segment_param /
      articulates / tail-credit gates updated. GATE 4 PENDING.
- [x] 5. Generated leg struts (_leg_strut_capsules, ball layer only) from
      the leg-generation constants + per-slot leg length: bombardier
      front+middle (replaces hand-made), scorpion rear+rear2, spider all
      4 pairs (straight chords; arch refinement is step 7). GATE 5 PENDING.
- [x] 6. Bombardier polish: antennae+head-top merged into one transverse
      bar capsule (killed the three-way blend seam); aim burial damping
      (same horn_burial signal + constants as the turn clamp: 1.5 free,
      5.5 stop); HEAD BLOCK is now a second segment in
      horn_collision_segments (aim-tracked) so beetle-vs-beetle anti-clip
      sees the raised head. [scoop tunable CUT per design rule]
      GATE 6 PENDING — CANARY REQUIRED before committing (the bvb head
      segment rebaselines horn_cross metrics; record the new baseline).
- [ ] 7. Spider refinement (only if gates show residual issues): abdomen
      as a segment (enables its velocity credit via the same channel
      math), fang chord verification, peaked leg arches.

## Testing schedule

User playtests are the feel judge at every gate; canaries only on request.
Each step lands as its own commit (user says "commit" per gate) so feel
regressions bisect to one change.

- GATE 3 (current, ~10 min): scorpion V-strike a resting ball -> visible
  smash (report too-weak/too-strong: it rides the universal
  SHAFT_PENETRATION_LIFT / momentum-transfer tunables); tail strike vs a
  beetle -> real shove; bombardier scoop unchanged; spider aim abdomen
  onto/under a ball -> surface followed, no ghosting; normal horn joust ->
  unchanged.
- GATE 4 (after claw split): ball into each claw from front/side -> clean
  deflection; claw-vs-horn placement looks right. CANARY on request
  (horn_cross rebaselines from new segments; compare deep_clip only).
- GATE 5 (after leg struts): ball under scorpion rear / bombardier front /
  spider arches -> collides, no ghosting; walking beetles don't "carry"
  the ball on invisible legs (static-pose error check).
- GATE 6 (after bombardier polish): head-vs-horn push feels solid; head
  grind into a beetle slows instead of clipping. CANARY required before
  committing the bvb head segment (horn_cross rebaselines — record new
  baseline).
- GATE 7 (spider, if run): ball vs aimed abdomen transfers momentum; ball
  rests on/deflects off fangs and leg arches.
