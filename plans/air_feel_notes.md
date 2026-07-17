# Airborne Physics Deep Dive — mechanics + full tuner map
2026-07-17. Reference for "how does air combat actually work and what do I turn."
Verified against code this date. Companion: plans/smoothness_plan.md, air_nerf_plan.md.

## The airborne life cycle of a beetle

### 1. Getting launched
Vertical velocity comes from (all smoothed through the pending drain at 20%/step
unless noted): horn lift (`HORN_LIFT_STRENGTH` 1.36 x leverage, per-step cap
12/`LIFT_STEP_DIV`(9) ≈ 1.33), yaw-grind lift (`YAW_GRIND_LIFT` 80/s via pending),
shaft-under lever (cap `SHAFT_PEN_LIFT_CAP` 1.5), SVS vertical (velocity-capped
`SVS_LIFT_CAP` 8), crossing-detector un-cross. Hard ceiling: MAX_VERTICAL_VELOCITY
= 20 (hardcoded). Tumble torque queues alongside (`TUMBLE_MULTIPLIER` 6.2 through
the same drain).

### 2. Crossing the height gates (air_gap = daylight under lowest geometry)
- **1.0 (`AIR_GRACE_LIFT`)**: hold-speed bonus WIPES (instant); drive fade + cap
  cut begin; board silk stops applying.
- **1.5 (`AIR_SLOW_LIFT`)**: ONE-SHOT horizontal cut `AIR_POP_SLOW` (0.30).
- **1.0→2.5 (`AIR_DEAD_LIFT`)**: drive force ramps 1.0 → `AIR_CONTROL` (0.0 =
  ballistic); speed cap ramps down by `NERF_SPEED_CUT` (0.26). Tilt adds its own
  cap cut (same constant) from `TILT_DRIVE_START` (0.2 of 90°) to 90° — both
  maxed ≈ 45% below base. Turning input stays FULL at all heights (aim the
  landing, not fly to it). `AIR_FRICTION` 0.985/step preserves knockback
  momentum (ground is 0.88).

### 3. Tumbling
pitch/roll velocity: damped `AIRBORNE_DAMPING` 0.95/step (light), pulled level by
`AIR_RESTORING` 35 (spring ∝ angle), capped `AIRBORNE_TILT_SPEED` 10 rad/s
(~9.5°/frame; 900=uncapped Nov 2025 → 14 → 10). Yaw capped MAX_ANGULAR_SPEED 8
(hardcoded). WHY BEETLES RARELY FULL-FLIP (user observation, working as designed):
the restoring spring's torque grows with angle — around 90° (sideways) it fights
hardest, so most launches plateau sideways and right themselves. Full flips need
the tumble to beat the spring through the 90° wall, which only huge hits do.

### 4. Airborne COLLISIONS (what changes vs grounded)
- Full response stack still runs (voxel cluster, impulse, shaft, SVS, crossing
  detector, body-sep, leg cage). Differences:
  - VERTICAL positional separation (`SEPARATION_FORCE` 0.7 along the smoothed
    normal) fires ONLY both-airborne (floor safety) — mid-air untangles faster.
  - Grounded down-blocks (SVS/crossing/scoop "never push a grounded beetle
    down") release — airborne beetles can be spiked downward.
  - Angular damping 0.95 vs ground 0.585: collision torques persist ~8x longer
    airborne — mid-air hits spin dramatically. This asymmetry IS the air feel.
  - HEIGHT PENALTY: horn lift force fades above y≈2 — mid-air re-lifts are
    deliberately weak (no infinite juggles). No slider; in-code fade.
  - Horn articulation credit (predicted-skeleton diff) works airborne — a horn
    flick mid-air transfers real sweep momentum.

### 5. Landing
Touchdown while bouncing (|vy|≥2): `WEAK_RESTORING` 25; landing tilted + slamming
lightens damping (blend→0.78) so the spring can flatten fast. Settled: full
`RESTORING_STRENGTH` 35 + heavy 0.585 damp + close-enough finish (last ~1.5° ease
out over ~5 frames). Ground tilt limit 80° is a VELOCITY WALL (blocks outward,
spring returns) — never a position snap (2026-07-17 fix).

## Ball in the air
Gravity x`BALL_GRAVITY_MULTIPLIER` (1.8 → 162 u/s²), vy cap 50, horizontal cap
`BALL_MAX_SPEED` 40, NO air resistance (arc-hang fix). Contacts use the analytic
surface normal with FULL 3D impulse (2026-07-17 loft fix): `(1+BALL_RESTITUTION
0.2) x closing x MOMENTUM_TRANSFER 0.96`. On-top impacts bounce in the SURFACE
FRAME: `vy = surf_vy + impact x BALL_BEETLE_BOUNCE 0.65` — a rising horn
trampolines (articulation velocity included), gate `BALL_BOUNCE_MIN_DROP` 0.75
voxel-drop. Tangential grip 0.6/bounce. Scoop (`SHAFT_PENETRATION_LIFT` 0.32,
cap 4) is an anti-clip heave, NOT the launcher. Receding-contact guard prevents
carry-pop.

## Tuner cheat sheet (slider name — param — default)
LAUNCH: Horn Lift (HORN_LIFT_STRENGTH 1.36) · Tumble Multiplier (6.2) · Yaw Grind
Lift (80) · Horn Sep Lift Cap (SVS_LIFT_CAP 8) · Shaft Pen Lift Cap (1.5)
FLIGHT: Gravity (90) · Air Pop Slow (0.30) · Air Slow Lift (1.5) · Air Grace/Dead
Lift (1.0/2.5) · Air Drive Floor (AIR_CONTROL 0) · Nerf Speed Cut (0.26) · Tilt
Drive Start/Floor (0.2/0) · Air Friction (0.985)
ROTATION: Air Tilt Speed (10) · Air Damping (0.95) · AIR_RESTORING (35, no
slider) · Ground Tilt Speed (8)
LANDING: Restoring Settled/Bouncing (35/25) · Ground Tilt Max (80, velocity wall)
BALL: Gravity Multiplier (1.8) · Momentum Transfer (0.96) · Restitution (0.2) ·
Beetle Bounce (0.65) · Beetle Bnc Grip (0.6) · Bounce Min Drop (0.75) · Ground
Bounce (0.9) · Ice Bounce (0.6) · Bowl Bnc Normal (0.5) · Shaft Scoop Lift (0.32)
HARDCODED (edit-only): MAX_VERTICAL_VELOCITY 20 · MAX_ANGULAR_SPEED 8 · height
penalty fade y≈2 · pending drain rate 0.2.

## The 30 FPS snap (user report 2026-07-17) — and the fix
Physics caps are per PHYSICS step (60Hz): Air Tilt Speed 10 = 9.5°/step. At 60
FPS each displayed frame shows one step (9.5° max). At 30 FPS each displayed
frame advances TWO steps → 19° of rotation per displayed frame — DOUBLE the
visual snap, same physics. Interpolation can't help; it samples between adjacent
states, it doesn't smooth across them. FIX (approved concept, smoothness plan
S3.3 option): RENDER-SIDE SLEW LIMITER on pitch/roll — cap displayed rotation
per RENDER frame (deg/sec of wall time), bank the remainder in a decaying visual
offset (same pattern as net_vis offsets). Physics untouched, framerate-
independent visual rotation speed.
