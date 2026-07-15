# Ball / Floor / Beetle Consistency — audit + implementation plan

Audited 2026-07-15 after the ball-collision overhaul + moving-surface work;
plan added same day. Same contract as abnormal_body_collision_plan.md:
ONE set of contact physics, per-case code only DESCRIBES geometry/motion,
escape-hatch sliders for feel changes, user playtests every gate, each
step commits on the user's "commit" so regressions bisect.

## Substep execution order (verified)

inputs -> beetle updates -> BALL-vs-BEETLE collisions (net displacement
governor 1.5/substep) -> bowl slide (beetles, ball; rim bounce lives here)
-> beetle-vs-beetle -> particles -> BALL-vs-FLOOR (bounce/settle/grip,
goal pit) -> beetle floor support + edge tipping.
Floor acts only on downward vy, so it cannot eat same-substep beetle
bounces. Ordering is sound; no fix needed.

## The three bounce personalities (as shipped)

| surface | restitution | bounce gate | tangential rule | settle rule |
|---|---|---|---|---|
| floor | 0.8 | none (any vy<0; settle if post vy<2) | world vx*=0.899/bounce | vy=0, y snapped |
| beetle | 0.45 | relative impact > 2-voxel drop | relative, keep 0.6 | vy=surface + capped carry |
| rim band | 0.5 | outward speed > 6 | none (radial reflect) | slide + damped momentum |

Restitution differences are intentional (concrete vs chitin vs boards).
The inconsistencies below are structural, not tuning.

## PLAN — phases with playtest gates

### Phase 0 — commit checkpoint
The tree carries the rim package + moving-surface frame + carry/bounce
grips uncommitted. Commit before starting (user says "commit").

### Phase 1 — slope-aware bowl bounce  [DONE 2026-07-15, gate 1 passed]
FINDING: floor bounce reverses vy only — every surface bounces STRAIGHT
UP including the 22-deg ice slope, so lip bounces pop vertically then get
dragged by the slide (two-phase, unphysical).
IMPLEMENT: in the ball floor-bounce block, when the contact is on the bowl
ring (dist > ARENA_RADIUS, not in a goal lane): reflect the ball's
velocity about the ANALYTIC slope normal instead of vertical. Normal in
the (radial, y) plane from the known geometry: slope rises 0.4/voxel
outward -> unit normal = (-0.4*out_x, 1, -0.4*out_z)/sqrt(1.16) (up and
inward). Restitution stays BALL_GROUND_BOUNCE; grip applies to the
tangential remainder. Flat floor inside the arena: unchanged path.
SLIDER: "Bowl Bnc Normal" 0..1 blend (0 = old vertical-only) — feel
escape hatch, default 1.
GATE 1 (user, ~5 min): drop/roll the ball onto the bowl slope from
inside — bounces deflect up-and-inward, single clean arc, no two-phase
pop-then-drag; flat-floor bounces in mid-arena identical to before;
goal-lane shots unaffected. REGRESSION: dribble run + rim carry.

### Phase 2 — lip double-bounce guard  [DONE 2026-07-15; Rim Bounce default 0.5->0.35 user tune]
FINDING: a fast ball at the lip can collect BOTH the rim reflect (1.5x
outward->inward, in the bowl slide) and a floor bounce (0.8) in the same
substep — independent restitutions, possible super-bounce.
IMPLEMENT: apply_bowl_slide sets a per-substep flag on the ball when the
rim bounce fires (e.g. beetle_ball.rim_bounced_step = physics_frame);
the floor-bounce block skips its RESTITUTION when the flag matches the
current step (keeps the positional push-out and settle logic). One flag,
no tunable.
GATE 2 (user, ~5 min): hammer fast shots into the lip corner from
several angles — lively single rebound, never a rocket; slow rolls onto
the lip settle normally.

### Phase 3 — goal robbery fix (MIN_Y clamp exemption)  [DONE 2026-07-15, gate pending; _in_goal_lane helper shared by slide/squeeze/slope-bounce/clamp]
FINDING: beetle_collision clamps both bodies to y >= 0.5 / vy = 0 — a
floor rule inside the collision system that doesn't know the goal pit has
no floor. A ball entering the pit while a beetle touches it gets snapped
back up and its fall zeroed.
IMPLEMENT: skip the MIN_Y clamp for the BALL when it is inside a goal
lane (abs(z) < 12 and abs(x) >= 30 — same test the bowl slide and rim
squeeze use; factor the lane test into a tiny helper so all three share
it). Beetles keep the clamp everywhere (they never enter the pit).
GATE 3 (user, ~5 min): score repeatedly while a beetle rides/pushes the
ball over the goal lip — ball never hangs at the mouth or pops back out
of the pit; normal beetle-ball contact in open play unchanged.

### Phase 4 — dust on beetle bounces  [CUT 2026-07-15 per user: dust is a FLOOR effect by design, beetle bounces stay dry]
FINDING: floor bounces spawn dust + squash from a shared 2-voxel-drop
gate; beetle bounces squash but never dust — hard bounces off backs look
dry.
IMPLEMENT: call spawn_ball_bounce_dust from both beetle-bounce sites
(BODY BOUNCE + shaft on-top) when the RELATIVE impact clears the same
gate, dust y at the contact height (ball bottom), reusing the existing
0.1s dust cooldown so dribbling can't spam.
GATE 4 (user, ~3 min): heavy drop onto a back -> dust puff at the
contact; normal dribbling -> no dust spam.

### Phase 5 — on-demand items (do NOT implement unless play shows them)
- Floor bounce pre-gate alignment (floor bounces any contact, beetles
  pre-gate at 2-voxel drop): only if floor micro-bounces ever read as
  chatter. Would add BALL_FLOOR_MIN_DROP with the beetle formula.
- Parked-ball stillness on beetle backs (on_ground is floor-only, so the
  anti-vibration re-stamp skip never applies there): only if a ball
  resting on a still beetle visibly buzzes.
- Sloped-floor cache stepping (XZ-threshold caches assume flat floor):
  only if rolling on the bowl looks steppy.
- Split ball gravity (base in Beetle.update + extra in the ball section):
  consolidation is a refactor risk with no observed bug — leave unless a
  gravity bug appears.
- Scorpion tail-tip scan row (+22 grounded scan vs tip at ~23): tiny
  dead zone at full raise; fold into any future scan-range work
  (remember the Nov 2025 scan-trimming lesson before touching).
- Anti-teleport governor dilution in deep pins: revisit only if deep
  multi-contact pins expel visibly slowly.

## Order & cadence
0 (commit) -> 1 -> GATE -> 2 -> GATE -> 3 -> GATE -> 4 -> GATE -> stop.
Phases 1+2 both touch the lip; if gate 1 already shows super-bounces,
note it and continue — phase 2 is the fix, judge the lip at gate 2.
No canaries needed: every change is ball-only (no beetle-vs-beetle
surface touched); combat metrics are unaffected.
