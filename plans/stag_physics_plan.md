# Stag Natural Physics Plan — ball AND combat

Written 2026-07-15. Goal: the stag feels like actual physics with both the
ball and other beetles — cradle-carry and squeeze-shoot for the ball,
clamp/drag/lift/throw for combat — all under the design rule: per-type
code only DESCRIBES geometry and motion; every behavior emerges from the
shared contact formulas (radial push, Coulomb-capped friction, channel
rate x lever).

## What the stag IS

- Two L-shaped pincers: low horizontal arms (y~1) + vertical RISERS at
  their ends (the real raised tips). 4 collision segments since 0dcf1a7.
- The arms + risers + head form the game's ONLY CONCAVE weapon: a cradle.
- Unique DOF: yaw does not sweep — it OPENS/CLOSES the pincers (squeeze).
  Pitch raises/lowers the whole assembly.
- Hook-interior voxels line the gap (STAG_HOOK_INTERIOR_*).

## Audited current state

- BALL: convex-capsule mitigations shipped (risers, grounded gate,
  segment hysteresis, hook-aura removal). Acceptable feel, but the pocket
  is still convex math in a concave hole: opposing walls cancel blended
  normals, and CLOSING the pincers on the ball has no principled response
  (degenerate pop-out direction).
- COMBAT: the hook interior does NOTHING but detection — wide tolerance
  (+-5 rows) for early contact and an exclusion in the clip metrics.
  There is NO grip: clamp-and-hold/drag/lift/throw does not exist. The
  squeeze reads as early-detected collision mush.

## P1 — MULTI-CONTACT resolution (general; the pocket emerges)
##      [IMPLEMENTED 2026-07-15 (pocket special case deleted same day), gate pending]
## WATCH ITEM: a horn segment can be counted once by the shaft-branch
## primary AND once as a secondary contact when the deepest overall shape
## is a body capsule — caps + the displacement governor bound it; revisit
## only if pushes near horn/body junctions feel doubled.

HISTORY: a dedicated pocket regime (apply_stag_pocket: pocket-frame wall
math + scripted ejection with a magic 2x constant + POCKET_EJECT slider)
was implemented 2026-07-15 and REJECTED the same hour by the design rule
— it was a disguised special move. Replaced by the general mechanism:

THE DEFECT: the ball resolves against ONE shape at a time (deepest
winner; seam-blending of normals was a band-aid on the winner flicker).
Real contact resolution handles every penetrating contact independently.

THE MECHANISM: _ball_surface_contact returns the full contact list (every
shape with real penetration: pen, radial, contact point, that shape's own
articulation velocity). Beyond the deepest contact (which keeps the
existing tuned primary flow), each SECONDARY contact applies its own
capped horizontal push-out + closing-velocity transfer along its own
radial, using the shared tunables (SHAFT_PENETRATION_PUSHOUT/VEL_DAMP).
Vertical stays with the primary paths (on-top branch, grounded gate).

WHAT EMERGES, no stag-specific code at all:
- CRADLE: both arms penetrate -> both push toward the gap center
  simultaneously. Same for hercules jaw gap, rhino fork nest.
- SQUEEZE-SHOT: each arm's inward normal tilts FORWARD by the arm's
  splay angle (perpendicular to a forward-outward line). Closing arms
  transfer their inward wall velocity through their own contacts, and
  the V converts two lateral squeezes into forward ejection — strength
  set by the real arm angle and closing speed. No formula, no slider.

GATE P1 (user): cradle-dribble a lap with turns (ball pocketed, no glue);
squeeze-shot by slamming V with the ball in the cradle (repeatable,
forward, harder with faster close); rest-in-pocket jitter check; hercules
jaw-gap carry as the cross-type check; rhino/others regression.

## P2 — Natural containment (combat) — NO grip mechanic, geometry only

USER DECISION 2026-07-15: no grabbing/coupling mechanic of any kind. If
the physics is right, closed pincers hold a beetle because two real
walls flank it — the design rule taken to its conclusion.

The pieces already exist; this phase VERIFIES they compose:
- CONTAINMENT: the victim between the arms gets pushed inward-off each
  wall by the existing shaft-penetration response (radial out of each
  shaft = toward the gap center from both sides) — being boxed IS the
  hold. Risers (0dcf1a7) close the front. Nothing new to build.
- DRAG: turning with a clamped victim — the arms' turn-sweep is already
  credited at the contact (momentum transfer measures velocity AT the
  contact point) — the victim gets carried around. Exists.
- LIFT: pitching up with a clamped victim — arm articulation credit
  (predicted-skeleton diff) already feeds lift/closing. Exists.
- THROW: opening the pincers removes the walls — the victim leaves with
  whatever velocity the sweep gave it. Pure conservation. Exists.
- ESCAPE: driving out of the open side (backward out of the gap) is
  unobstructed by geometry — escape is a direction, not a stat check.

AUDIT ITEMS (fix only what verification shows broken):
- Does the wall push engage reliably at squeeze depths, or do the tip
  gates (has_horn_tips, _st bands) suppress the arm responses for a
  victim deep in the gap? (The old blanket-tip-gate lesson — basal
  contacts must fire even with tips touching.)
- Is the victim's own horn/body pushed out of the gap by the CORRECT
  wall (segment attribution inside a concave gap — same fork ambiguity
  the ball had; may need the segment-hysteresis treatment for beetle
  intruders too).
GATE P2 (user): clamp a bot mid-charge — it should feel boxed while the
pincers are closed, get dragged through a turn, lifted with pitch,
released by opening; non-clamp fights unchanged. CANARY only if audit
items force response changes.

## P3 — Squeeze-under-load consistency ~1-2 hrs

- Verify yaw-close is burial-damped under load (calculate_horn_damping
  should engage on squeeze contact; if not, apply the standard 1.5/5.5
  burial curve to closing rate) — clamping a beetle should slow the jaws,
  not ghost through.
- Pocket occupancy precedence: ball uses P1, beetle uses P2; single
  deepest occupant wins if both are somehow present.
- Hook wide-detection stays beetle-only (ball adjacency rule from
  0dcf1a7 preserved).

## Order, gates, risk

P1 (ball, self-contained, no combat risk) -> GATE -> P2 (combat AUDIT,
code only where verification shows a gap) -> GATE -> P3 polish. Perf:
pocket test is a point-in-region check on existing segments — negligible.
With no grip mechanic, P2's only risk is discovering suppressed wall
responses in the gap (tip-gate class), which are fixes, not features.
