# Atlas Low-Horn Ball Detection Plan — grid-side fix for the blind spot

2026-07-19. Status: PLANNED (not started). Owner problem: atlas (and any
horn pitched near the ground) clips through / fails to engage the ball
when turning the horn or the body at low pitch. Companion history:
collision-system-notes memory ("ANALYTIC DETECTION HAS NOW FAILED THREE
TIMES"), 4_player_steam.md MB5 section, commit 516c157 (the response-side
family that already shipped).

## THE HARD CONSTRAINT (read first, it shapes everything)

Analytic contact ASSERTION is banned. Tried three times (MB-PERF v1,
P3 no-voxel-contact SVS, 2026-07-18 low-horn rescue) — all three ghost-
pushed and were reverted. Horn segments are chord approximations that
occupy space the visual horn doesn't; no threshold catches real contacts
without firing on visible daylight. THE GRID DETECTS, ANALYTICS MEASURE.
Therefore this plan fixes DETECTION IN THE GRID KERNEL, nothing else.

## Root cause (established 2026-07-18, verified in _column_pair_contact)

Ball-pair detection requires beetle + ball voxels in the SAME XZ column
within +-2 rows (`ball_adjacent`, beetle_physics.py ~5326/5338). A horn
pitched to the ground is a HORIZONTAL run of voxels at rows ~1-2. The
ball's shell in any column at horizontal distance d from its center sits
at height r - sqrt(r^2 - d^2): ~0.5 voxels at d=2, ~1.4 at d=3, ~2.1 at
d=3.5 (r=4). So at the periphery — exactly where a sweeping horn arrives
first — the vertical gap between horn voxels and shell voxels exceeds 2
rows and the column test is geometrically blind. Legs never hit this:
they are VERTICAL structures (voxels at many heights per column) and the
confirming body mass is always adjacent. The low horn is horizontal AND
body-remote (tip range) — invisible on both counts.

Why the response side is already fine: once ANY column confirms the
pair, the analytic manifold measures every contact and the 2026-07-18
family (sweep bulldozer both flavors, crossing detector, smoothed
gates) handles low-pitch sweeps. Detection is the only missing layer.

## The fix — asymmetric, footprint-gated adjacency (ball pairs only)

In `_column_pair_contact`, for ball pairs only, ADD a second acceptance
rule alongside the existing +-2 adjacency:

    beetle voxel BELOW a ball voxel by <= 5 rows, in a column whose
    horizontal distance to the BALL CENTER is <= r - 0.5

Why this cannot ghost (each condition is load-bearing):
- BELOW only: a horn above the ball with a 3-5 row gap is obvious
  visible daylight — the widened window must never fire there. The
  asymmetric direction is hidden by geometry (next point).
- Footprint gate (d <= r - 0.5): inside the ball's silhouette the shell
  overhang means the "gap" between a ground-level horn and the shell is
  at most ~2.1 voxels of space the CAMERA CANNOT SEE (it's under the
  ball). A horn there is visually touching-or-under the ball. Columns
  outside the footprint keep the strict +-2 rule.
- <= 5 rows: covers ground-row horn (rows 1-2) vs shell heights up to
  the footprint edge with margin; bounded so a horn under a ball that
  is clearly AIRBORNE (shell floor 3+ voxels up in ALL columns) still
  requires the ball to be low. Consider requiring ball y < ~6 (grounded
  or skimming) as a belt-and-suspenders gate — decide in A1 testing.

Implementation notes:
- The kernel already receives both entities' x/z/y per pair
  (pair_check_data) and knows which side is the ball via is_ball_color —
  the column-to-ball-center distance is computable in-kernel from gx/gz
  vs the ball's grid position (mind RENDER_Y_OFFSET conventions and the
  0.5-voxel column center).
- Ball radius: kernel needs it (fields carry positions only today) —
  either a new pair_check_data column or derive from the ball colors
  (radius is uniform across balls today; Ball Count menu doesn't change
  radius... but BIG-BALL support exists — pass it, don't assume).
- Keep the existing +-2 path byte-identical for everything else.
  Beetle-beetle pairs completely untouched.
- Dev flag `--noballwindow` (or param) to disable the new rule for A/B.

## Phases

### A0 — measure the blind spot first (no behavior change)
Cheap Python-side diagnostic in the ball response loop: when the batch
says NO contact for a close pair but any horn segment is within
(radius + 1.5) of the ball center, count it (`ball_blind_events`) into
collision_stats + perf log. This is analytic MEASUREMENT for telemetry
only — it asserts nothing. Gate: one manual atlas-low session + one
canary. Gives: how often the blind spot fires for atlas vs rhino vs legs
(expected: atlas-low dominates), and the baseline the fix must close.

### A1 — the kernel rule
Implement the asymmetric footprint window above. Compile + canary
(--canary with atlas in --bot-types) + manual repro: atlas fully down,
stationary body-turn into grounded ball; horn-yaw sweep at low pitch;
same at the rim. Gate: ball engages (pushes/carries per the bulldozer)
in all repro cases, ball_blind_events collapses vs A0 baseline.

### A2 — ghost audit (the v1 criterion, non-negotiable)
The ball must NEVER move while visible daylight separates it from the
beetle. Manual: walk near the ball at every horn pitch WITHOUT touching;
dribble around a stationary ball; park ball at rim + walk past. Canary:
ball_ball + ball motion with no adjacent beetle would need eyeballing —
this gate is primarily the user's feel session. Any ghost = revert the
rule (flag makes A/B trivial) and shrink the footprint gate (r - 1.0)
before trying again.

### A3 — response tune at the newly-visible contact
With detection landing, low-pitch contacts route: under-center columns →
on-top path (scoop/rest lift — INTENDED, sliders Scoop Lift / Passive
Lift / Ball Surf Normal own it); periphery → side path → sweep
bulldozer pushes laterally. Expected follow-up tuning only: if low
sideways sweeps still lift more than the user likes, that's the July-15
scoop stack (tune sliders, do NOT add mechanics). Watch
ball_crossing_fires — should drop to ~0 once detection precedes the
sweep.

## Success criteria
- Atlas fully-down body-turn and horn-sweep engage the grounded ball
  every time (no click-through), mid-arena and at the rim.
- Zero ghost contacts across the A2 audit (user verdict).
- No canary FPS regression (kernel adds a few compares per column —
  expect noise-level).
- Rhino/stag/legs feel unchanged (their detection never used the new
  window).
