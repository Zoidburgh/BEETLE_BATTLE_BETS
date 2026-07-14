# Abnormal-Body Collision Plan — bombardier, scorpion, spider

Written 2026-07-14 from a full bombardier audit, extended with scorpion and
spider audits and a first-principles pass on why the "abnormal body"
beetles keep misbehaving. Supersedes bombardier_collision_plan.md.
NO code yet beyond what the session already shipped.

## First principles: the four blanket failure modes

Two representations exist. The VOXEL layer (stamped grid) is always right —
baked slants, tail rotation, aim tilts, leg gait all render into it. The
ANALYTIC layer (capsules/segments) drives ball push-out depth, ball surface
normals, shaft/svs responses, and motion credit — and it was built
HORN-FIRST: one forward chord on a standard body. Every unique beetle
breaks that template the same four ways:

### F1. Shape coverage — off-template regions have no analytic presence
- bombardier: head block, antennae, stilt front legs (patched piecewise)
- scorpion: tail (patched), CLAWS (two claws spread z +-4..7, represented
  by ONE centered chord -> ball ghost/slips on claws), raised-rear stilt
  legs (unpatched)
- spider: fat abdomen barely matched by the thin generic spine capsule;
  peaked arch legs (peak 3-6 vox — ball-sized gaps under the arches);
  fangs only covered by the low prosoma chord (3,2,0)->(11,2,0)
PRINCIPLE: derive shapes from the same constants generation uses, declared
per type in one place — stop discovering them one bug report at a time.

### F2. Motion channels — each type has a DOF the physics can't see
The credit system only knew horn_pitch/horn_yaw.
- bombardier: spray aim tilts the WHOLE body around the rear pivot,
  +-10 deg (position + velocity credit DONE this session — the prototype)
- scorpion: tail_rotation_angle swings the tail 50 deg/s (~16 vox/s at the
  tip). Position tracked; VELOCITY UNCREDITED — a tail smash transfers
  zero momentum to ball or beetles. The signature weapon does nothing.
- spider: spider_aim rotates the ABDOMEN around a front pivot,
  SPIDER_AIM_MAX 0.52 rad (~30 deg!) — the fat rear swings ~7 voxels at
  the tip. ENTIRELY untracked: neither position nor velocity. The biggest
  unmodeled motion of the three.
PRINCIPLE: every analytic shape declares its motion channel (body-static /
horn / tail / spray-aim / spider-aim); each channel = pivot + angle +
rate. Position transform AND velocity credit (v = rate x lever from pivot)
come from ONE mechanism. The bombardier aim credit is the hand-built
instance to generalize.

### F3. Combat classification — "colored like a tip" != "fights like a horn"
has_horn_tips comes from voxel COLOR flags. Bombardier antennae+mandibles,
scorpion claw/tail tips, spider fangs are all flagged, so their contacts
run full horn-joust physics: MIN_TIP_ENGAGEMENT 0.65 floor, full tip
leverage (normal_y += leverage*2.5), HORN_TIP_STRENGTH tipping, lift/
tumble battle at TUMBLE_MULTIPLIER 4.2. That is the "crazy tipping".
PRINCIPLE: per-type TIP_FACTOR table scaling every has_horn_tips
escalation: bombardier 0.3, spider 0.6, scorpion 0.7, horn beetles 1.0.
Pure params; never touch the voxel flags (they drive coloring).

### F4. Raised stances / arches — ball-sized gaps expose the missing legs
Legs have zero analytic presence for all types; it matters where a gap
fits the ball: bombardier front (+4 stance — hand-patched struts),
scorpion rear (+4 — unpatched), spider leg arches (peak 3-6 — unpatched),
giraffe front (+2 — marginal, skip).
PRINCIPLE: generate leg-strut capsules from the leg-generation constants
(attach x, y-offset, multiplier, cascade) for the affected types, static
stance pose (gait swing accepted as unmodeled). Replace the hand-coded
bombardier struts with generated ones.

## Per-beetle specifics beyond the blanket items

### Scorpion
- Tail smash (F2): track beetle.tail_vel at the input site (~19342, same
  pattern as spray_aim_vel), credit in shaft + svs paths. A V-key strike
  should visibly launch the ball / shove a beetle.
- Claw split (F1): L/R claw chords (like stag pincers) to the real claw
  tips. Fixes ball ghost/slip on claws and misplaced beetle-vs-beetle claw
  contact.
- Tail svs tip-band dead zone (tail always tip-flagged, svs gated >0.7):
  accepted; revisit only if tail-vs-horn clipping is visible in play.

### Bombardier
- Simplify shape set: merge antennae + head top into ONE transverse bar
  capsule; drop hand-made middle struts once F4 generates them.
- Aim burial damping: grinding the head down through a beetle/ball should
  slow like horn damping (use horn_burial when bombardier owns contact).
- Scoop tunable: aim-sweep lift rides SHAFT_PENETRATION_LIFT; give it its
  own multiplier so the scoop tunes as a move.
- Beetle-vs-beetle head coverage: append the head capsule as a segment in
  horn_collision_segments (like atlas pronotum chords).

### Spider
- Abdomen shape (F1): dedicated abdomen capsule (fat, from generation
  constants: wide at rear, pedicel waist) on the spider-aim channel.
- Spider-aim channel (F2): track spider_aim_vel (input ~19580), rotate the
  abdomen capsule around the front pivot (spider_pivot_x, 0) — same
  transform the placement kernel uses (~8280) — and credit rate x lever.
  Ball resting on / hit by the swinging abdomen then behaves.
- Fang coverage: prosoma chord is at y=2 fixed — verify against fang
  voxels; widen/raise if ball slips on fangs in testing.
- Leg arches (F4): generated peaked struts (coxa-level + peak + tibia
  slope) — 8 legs, 2 capsules each is 16 shapes; start with ONE capsule
  per leg (attach->foot straight chord) and only refine if playtest shows
  balls resting inside arches.

## Seam stability (global)
`_ball_surface_contact` picks the single DEEPEST shape for depth AND
normal — at junctions (head/antenna, belly/strut, body/claw, abdomen/leg)
the winner flips per substep -> direction flicker = stick pockets and
"weird clipping". Replace with a penetration-weighted blend of all shapes
within ~1 voxel of the max. Small change, benefits every beetle.

## Execution order WITH TESTING SCHEDULE

Testing rules (per project convention): user playtests are the feel judge
and happen at every gate; canaries only run when the user asks — steps
that rebaseline combat metrics call for one and are marked.

0. COMMIT CHECKPOINT of the current session (+337 lines) before any step.

1. TIP_FACTOR table (F3) — params only.
   TEST GATE 1 (user, ~5 min): bombardier head-bump a horn beetle — should
   shove, not tumble; scorpion claw fight — still feels like a weapon;
   rhino/stag joust — UNCHANGED (factor 1.0).
   CANARY (on request): --bot-types stag,hercules,rhino,giraffe before/
   after — deep_clip/horn_cross should be flat (no geometry changed).

2. Normal blending (seams) — global.
   TEST GATE 2 (user, ~5 min): ball on bombardier head/antenna junction
   and under its belly near legs — micro-flicker gone; ball on any horn
   beetle back — unchanged feel.

   >>> MAJOR PLAYTEST after 1+2: do bombardier and scorpion "feel like
   they belong"? If yes, steps 3-7 are enhancement, not repair — schedule
   at leisure. If no, note WHERE and continue.

3. Motion-channel generalization (F2): channel helper extracted from the
   bombardier aim credit; add scorpion tail channel (tail_vel) and spider
   abdomen channel (spider_aim_vel + capsule transform).
   TEST GATE 3 (user, ~10 min): scorpion V-strike a resting ball — visible
   smash/launch; bombardier scoop still works; spider aim its abdomen down
   onto the ball — pushes it, no ghost; aim UP under a ball on its back —
   lifts it.

4. Scorpion claw split (F1).
   TEST GATE 4 (user): roll ball into each claw from front/side — clean
   deflection, no slip-through; claw-vs-horn fight placement looks right.
   CANARY (on request): scorpion bots — horn_cross rebaselines (new
   segments); compare deep_clip only.

5. Generated leg struts (F4): scorpion rear, bombardier front (replace
   hand-made), spider single-chord legs.
   TEST GATE 5 (user): ball under scorpion raised rear, bombardier front,
   spider arches — collides with legs, no ghosting; walking beetles don't
   visibly "carry" the ball on invisible legs (static-pose error check).

6. Bombardier polish: head-bar merge, aim burial damping, scoop tunable,
   bvb head segment.
   TEST GATE 6 (user): scoop tuning session with the new slider; head-vs-
   horn push feels solid.
   CANARY (required before commit of the bvb head segment): horn_cross
   rebaselines — record new baseline.

7. Spider fang/arch refinement ONLY if gates 3/5 showed residual slip.

Each step lands as its own commit (user says "commit" per gate) so any
feel regression bisects to one change.
