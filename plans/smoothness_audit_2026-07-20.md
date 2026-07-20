# Smoothness + readability deep audit — 2026-07-20

Three-agent sweep of beetle_physics.py for remaining physics-side jerk
sources, done AFTER the smoothing panel shipped (Push Smooth 3.7 /
Match Rate 0.35 / Air Match 16 / Tilt Smooth 4 / Air Turn Smooth 6).
Status: REPORT ONLY — nothing below is implemented. Line numbers valid
as of commit 257e04d + uncommitted smoothing sliders.

## Category 1 — threshold strobing (binary gates that flip per substep)

Contact detection strobes (proven: bracetrace showed gates false ~2/3 of
combat substeps), so every hard threshold keyed to contact quantities
flips with it.

1a. **has_horn_tips 0<->1 = 2.5x force swing** (~17710-13, set 9280).
    Tip vs shaft leverage is 1.0 vs 0.4, and horn_leverage feeds
    lift_impulse (=leverage*30) + normal bias + spin. Contact sliding
    along a horn (or strobing) flips this whole stack 2.5x per substep.
    TOP AMPLITUDE OFFENDER. Fix idea: EMA/latch pair_tip_factor
    (smooth the blend over ~0.1s) instead of binary any-tip-voxel.

1b. **The air_gap ~= 1.0 cluster, six systems, zero hysteresis**:
    friction 0.88<->0.985 (2458), AIR NUDGE zeroing (18591), AIR_MATCH
    gate (18645), AIR_TURN gate (18008), ball recoil (18224), on_board
    traction/turn-ramp (21950). A beetle skimming at hover height flips
    ALL simultaneously. Fix idea: shared hysteresis band (enter air at
    1.2, return at 0.8) or per-gate fade zones. The ONE existing
    hysteresis (AIR_SLOW_LIFT 1.5 fire / 1.0 rearm, 2472-78) is the
    pattern to copy.

1c. **lift_advantage +-0.5 branch flip** (17825, tree 17862-17955):
    even branch vs advantage branch is ~3.25x lift AND a sign-flipped
    ~9x pitch torque (tumble_mult + sign). effective_vel strobes via
    LIFT_LEVER contact distance + the per-type dead-zone gates
    (17766-92) + at-cap zero readings. pressing_down -0.3/0.3 windows
    (17842-43) compound it. Fix idea: hysteresis on branch selection
    (hold last branch unless advantage moves >0.2 past threshold) or
    blend forces across a +-0.5..1.0 transition zone.

1d. **Tip-end gate _st > 0.95** (16831): whole shaft side-path
    (carry/bulldozer/faded lift) blinks in/out at tip-to-tip contact.

## Category 2 — raw un-smoothed channels (siblings of fixed ones)

2a. **Main collision torque -> angular_velocity direct** (18744/18751,
    TORQUE_MULTIPLIER 1.95): fires every contact frame, bypasses
    pending_yaw AND all smoothing. Magnitude does scale with impulse
    (so slam vs graze differs) but delivery is raw per-frame writes.
    The linear sibling got velocity matching; this one got nothing.
    Fix idea: route through pending_yaw, or scale by the PUSH_SMOOTH
    blend factor _pss like the linear channel.

2b. **Grounded yaw fixed-strength**: away-from-attacker bias
    (momentum RATIO, not speed, 18019-68) and geometric torque
    baseline (velocity_factor floor 1.0, 17990) inject fixed yaw every
    substep to GROUNDED recipients — airborne got _ay smoothing.
    Fix idea: grounded analog of Air Turn Smooth, own dial.

2c. **Horn tipping torque block** (18086-142, HORN_TIP_STRENGTH,
    0.03s cooldown ~ every other substep): fixed-strength horn-side
    tilt, momentum-ratio distributed. The horn analog of the body tilt
    that Tilt Smooth fixed. Fix idea: same closing-speed scale.

2d. **Shaft-vs-body momentum transfer** (17341-42 + reaction 17352-53):
    raw per-frame; smoothing here was TRIED AND USER-REVERTED
    2026-07-19 ("older horn interactions better") — do NOT redo
    without new evidence. Reaction nudges the player = camera micro-
    jitter while pushing with the horn.

2e. **Lift branches are geometry-fixed strength** (even 0.06 push,
    press-down wedge, advantage stream): no closing-speed term.
    CAUTION: closing-scaling these IS the volley-factor concept that
    was rewound 2026-07-20 (combat clarity). Don't re-add casually.

2f. **Yaw-grind tilt writes pitch_velocity directly** (22258/22298,
    ~1.28 rad/s per grind frame, bypasses pending drain). Dev-defended
    as smooth-by-construction; candidate only if grind "nose pumping"
    is observed.

## Category 3 — position snaps (double-visible on player via camera)

- Shaft-vs-body push-out: up to ~1.5 vox/step (17413-16), burial-scaled
  but not velocity-ramped.
- Horn mini-separation: up to ~1.2 vox/step (18183-86), momentum-scaled,
  no depth taper for beetles (ball got one — BALL_SEP_DEPTH 18073-84).
- VERTICAL SEPARATION both-airborne: 0.7 vox/step un-ramped (18330-31).
- Crossing detector un-cross: rare but hardest single event (±1.5 vox
  each + velocity reversal + spin halved, 16282-311) — event-shaped,
  probably fine.
- Fix idea: give beetle separations the ball's depth-taper treatment
  (small overlaps separate gently, deep ones fast).

## Category 4 — one-step clamps without grace

- Landing tilt clamp 10->8 the instant on_ground flips (2638-41) —
  a KNOWN snap source (see 20151 comment history).
- MAX_ANGULAR_SPEED +-8 hard clamp (2627-28), no grace.
- Directional speed cap executes in one step when cap ceiling drops
  (air_no_traction / tilt nerf transitions) — knockback grace covers
  hits but not these transitions (2617-24).
- Fix idea: lerp clamp ceilings over ~0.1s instead of stepping.

## Category 5 — readability mechanisms (physics, not animation)

5a. **Per-type moment of inertia** — all beetles share 18.4 on all
    three axes (2183/2188/2193). Long/heavy types (hercules/atlas)
    spin like everyone despite 20+ voxel handles — this is the
    turned-around-fast complaint. Per-type factor = mass legibility.
    BUG FOUND: the Inertia slider (29457) writes only
    moment_of_inertia (yaw) on beetles[0..1] — pitch_inertia and
    roll_inertia never update (29522-23).

5b. **Ghost lift** — pending_lift keeps draining ~15 frames after
    contact ends (drain 2329-34 is unconditional; no flush on
    separation). At gravity 90 the launched beetle visibly "sticks up"
    after the cause is gone. Fix: fast-drain or flush pending_lift on
    contact loss = tighter cause->effect.

5c. **Pending drain rate hardcoded 0.2 x4** (lift 2330, pitch 2366,
    roll 2379, yaw 2388): THE cause->effect latency dial, no slider.
    Higher = snappier response to hits, lower = smoother but laggier.

5d. Beetle-beetle restitution is 0.025 (bodies absorb, no bounce);
    no hit-stop, no minimum-impulse deadband exists. Body-sep has a
    1.0-voxel deadzone; horizontal velocity has none.

## Ranked recommendations (best value first)

1. Smooth has_horn_tips / pair_tip_factor over time (1a) — highest-
   amplitude strobe, no feel-philosophy conflict, helps every weapon.
2. air_gap hysteresis band for the 1.0 cluster (1b) — one small fix
   calms six systems at the exact height aerial combat lives at.
3. Per-type inertia + fix the 3-axis slider bug (5a) — readability
   AND directly answers hercules/atlas spinning; physical, honest.
4. pending_lift flush on separation (5b) — kills ghost lift, pure
   causality win, tiny change.
5. Advantage-branch hysteresis (1c) + grounded turn smooth (2b) —
   next tier once 1-4 are felt.
