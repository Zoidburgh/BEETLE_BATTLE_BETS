# Collision Geometry Consistency Plan

Written 2026-07-14 from a 2-agent audit of all 8 beetle types' collision
geometry vs their real rendered voxels. Goal: ball/beetle contact resolves
against real shape everywhere, consistently — not a center-to-center fallback
in the common cases. NO code yet; this is the roadmap.

## The core finding (the real consistency story)

**No beetle of ANY type has a body/back/abdomen collision shape.** Every
segment in `horn_collision_segments` (~9292) starts at the horn root on the
FRONT of the shell and projects forward. The entire back/abdomen (the biggest
surface, and where a ball most often lands) is modeled by nothing.

Live ball-on-body path (traced through `beetle_collision`, ball routed at
~19622): the shaft-cylinder push and shaft-vs-shaft are both gated
`if not is_ball_collision`; the shaft-penetration path early-continues at the
horn root (`_st < 0.02`), so a back contact is skipped. Ball-on-body therefore
falls to the **generic center-to-center smoothed normal** (~15066) plus the
one **vertical BODY BOUNCE hack** I added (~15995-16009, `ball.y > beetle.y+3`).
So the common case is "mushy center-to-center + a vertical bounce special
case," identical for all 8 types. (A legacy `ball_radius+10` body sphere
exists in `apply_ball_collision_response` ~9035 but is DEAD — no call sites;
verify at implementation.)

The horn-segment residuals below are all second-order next to this.

## Per-type gap summary (from audit)

| Type | Horn coverage | Residual off-segment | Body? |
|---|---|---|---|
| **scorpion** | claws only, single chord (2,4,0)→(9,4,0) | **ENTIRE TAIL uncovered** — active weapon, tip ~y19 (15 vox above chord, outside any sphere), ~45° strike sweep uncredited; claws spread ±7 in z off the centered chord | none |
| stag | L/R pincer straight chords | L-shape vertical risers + elbow cut off (~8-12 vox/side); inner-face sampler exists but unused by segments | none |
| hercules | top+bottom jaw chords | curved arcs approximated straight: top hook + bow, bottom belly; jaw-gap interior unmodeled (~10-20 vox) | none |
| atlas | cephalic + 2 pronotum chords | pronotum mid-arc bows ±3 off chord; head-piece block unmodeled; **articulation credit inverted** (see below) | none |
| rhino | midline + L/R prong | 5-wide meaty base flanks off all chords (~10 vox) | none |
| giraffe | chained neck+head (the reference model) | 3×3 width vs zero-width chord; odd-step nub (few vox) | none |
| spider | single chord (3,2,0)→(11,2,0) | prosoma+fangs covered; abdomen is body (unmodeled like all); silk separate | none |
| bombardier | single chord (3,5,0)→(10,5,0) | head+mandibles covered; rear butt is body (unmodeled); spray separate | none |

**Also found — articulation-to-ball credit bug (~15190):** uses a hardcoded
`_sseg == _nseg-1` ("last segment moves") instead of the existing
`horn_segment_articulates()` helper (9359). Result: only giraffe is correct;
atlas is INVERTED (credits the stationary pronotum, not the moving cephalic);
rhino/stag/hercules drop motion credit for their non-last arms. Plus `_pt` uses
the generic tip while `so_tip` is the contact arm's tip — different arms for
multi-arm types, so even when credited the velocity delta is wrong.

## Plan — ordered by payoff

### PHASE A — Body capsule for every beetle ★ biggest consistency win (~1 day)
Add a generic BODY segment (a rear→front chord along the shell spine at body
height, or a capsule) derived from body_length + front/back body heights, in
`horn_collision_segments` for ALL types (hornless ones too). Route the ball
path through it so a ball on the back gets a real surface normal + bounce
instead of center-to-center. This:
- fixes the most common ball case for all 8 types at once,
- makes the vertical BODY BOUNCE hack geometry-driven (supersede/clean it),
- is the thing that makes contact FEEL consistent type-to-type.
Feel risk: changes how the ball rolls off every beetle — needs a slider
(BODY_CAPSULE strength / 0=old center-to-center) and playtest gates, like the
SVS_VERTICAL_SCALE escape hatch. Must not touch beetle-vs-beetle (additive on
the ball path only). Verify the dead radius-10 sphere claim first.

### PHASE B — Scorpion tail segment (the one SEVERE horn gap, ~half day)
Give scorpion a chained tail polyline (rear pivot → bulb → stinger tip),
rebuilt at the current `tail_rotation_angle`, exactly like the giraffe's
2-segment model. Uses `get_scorpion_tail_tip_position` (~3363, already exists
for venom). Covers the raised tail AND credits the ~45° strike sweep so the
tail can actually knock the ball / opponents. Also widen claw coverage for the
±z spread (2 claw chords or a fatter cylinder).

### PHASE C — Fix articulation-to-ball credit (~1-2 hrs, pure correctness)
Replace the `_sseg == _nseg-1` hardcode (~15190) with
`horn_segment_articulates(owner, _sseg, _nseg)`, and make `_pt`/`so_tip` both
use the CONTACT arm (rebuild that arm's tip at predicted angles). Fixes atlas
inversion + restores dropped credit on rhino/stag/hercules. Small, high-value,
no feel risk (only affects horn-strike-on-ball transfer).

### PHASE D — Second-order segment accuracy (optional polish, per-type)
Only if playtest shows a ball nestling off-chord: stag L-riser (add elbow →
2 seg/pincer, or wire the existing `get_stag_pincer_inner_points`), hercules
curved hook (add elbow point), atlas mid-arc (low priority, acknowledged in
code). Rhino base flanks + giraffe width are negligible — skip.

## Order & gates
A (body, universal + biggest feel change → playtest gate) → B (scorpion,
severe → playtest) → C (articulation, correctness, safe) → D (polish, only
if needed). Giraffe's chained 2-segment model is the reference pattern for A/B.
Canary (`--canary --bot-types ...`) for regression numbers; user playtest is
the feel judge. Keep the horn_collision_plan.md lessons: fix response geometry
once (data = segment lists), not per-type response hacks.
