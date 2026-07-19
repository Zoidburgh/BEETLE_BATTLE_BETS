# Soft Juggle Plan — volley-timed aerial lifts

2026-07-19. Status: PLANNED. Goal: after popping an opponent up, a
TIMED horn flick as they fall re-launches them (a volley); holding the
horn statically under them does almost nothing and no longer feeds
force back into the holder. Juggling becomes a skill (rhythm +
positioning), self-limiting at 2-3 touches, never an infinite tower.

## Constraints and history (respect these)

1. **No phantom forces.** The lift-hold-credit (virtual velocity for a
   held button) was implemented and reverted within an hour on
   2026-07-19 — user: "that's not how physics works". Everything here
   must be real-impulse-shaped: no relative motion, no transfer.
2. **The juggle-bracing variant is REJECTED.** Tried 2026-07-17
   (c6d2c48 era, directional-bracing), user play-tested and reverted:
   "preferred the original feel". Recoverable from git; do not
   re-propose without new evidence.
3. **Sliders with the old feel at one end** (project rule), and
   **instrument before tuning** (this week's twice-learned lesson —
   the stance brace shipped mis-gated and mis-read until --bracetrace).
4. Host-authoritative physics — no protocol impact, but the v6 field
   test is still pending; don't stack this into a network session.

## Current mechanics inventory (verified in code this session)

- **Height penalty** (~line 17745): lift force x 1/(1+0.35*(avg_pair
  height - 2.0)). Starts at avg 2 voxels — so even the FIRST follow-up
  fights it (victim 8 + you 0 -> ~60% force). This is the "no juggle".
- **Advantage battle** (~17689+): pitch-velocity difference, threshold
  0.5. Holder at pitch cap reads vel=0 (input code zeroes clamped
  motion) -> falling victim pitching down wins press-down against a
  static holder: holder takes horizontal push, victim self-wedges.
- **Air-side vertical** (~18465): horn-contact vertical impulse applies
  only to AIRBORNE recipients — the vertical delivery path for volley
  hits already exists.
- **Lift delivery**: pending_lift queue, 20%/tick drain, LIFT_STEP_DIV
  9 continuous stream; TUMBLE_MULTIPLIER 6.2 off-center rotation.
- **Stance brace** (2026-07-19): grounded holder now absorbs a slice of
  incoming tilt per tick (7 standing -> 20 at charge, proximity-faded)
  — already reduces the "holder takes force" problem materially.

## Phase J0 — instrument the aerial exchange (--juggletrace)

Before touching feel: a trace flag logging, for any lift-branch contact
where either beetle has air_gap > 1.5:
  which branch fired (press-down / even / advantage / none),
  attacker horn pitch vel + at-cap flag, victim y/vy,
  RELATIVE VERTICAL CLOSING at the contact (horn-tip upward speed
  [articulation vy + body vy] minus victim vy),
  lift force BEFORE/AFTER height penalty, forces applied to each side.
One session: pop -> attempted follow-ups, both timed flicks and static
holds. Output decides J1's reference speeds and confirms which branch
actually fires mid-air (prediction: press-down against the capped
holder — verify, don't assume).

## Phase J1 — the volley factor

When the RECIPIENT of lift force is airborne (air_gap > ~1.5), scale
the lift force by relative vertical closing speed:

    closing = horn_tip_up_speed - victim_vy   (falling victim => big)
    volley = clamp(closing / JUGGLE_REF, 0.0, 1.0)
    lift_force *= (1-JUGGLE_VOLLEY) + JUGGLE_VOLLEY * volley

- JUGGLE_VOLLEY slider 0..1, DEFAULT 0 ON LANDING THE CODE (0 = old
  behavior exactly — user turns it up to feel it; bake the chosen
  value after tuning, per project convention).
- JUGGLE_REF from J0 data (the closing speed of a well-timed flick).
- Grounded-victim battles UNTOUCHED (factor only when recipient
  airborne) — ground lift feel is play-validated, don't graze it.
- Static hold under a falling victim: closing comes almost entirely
  from their fall speed — so a PERFECTLY still horn still volleys a
  fast-falling victim somewhat (physically honest: they fell onto a
  hard stop). The skill expression is the flick ADDING horn-tip speed
  on top. If static catches feel too strong, J0's numbers set JUGGLE_REF
  higher so unflicked contact lands in the weak zone.
- EXPECTED SIDE BENEFIT: the victim's mid-air press-down against a
  capped holder ALSO has ~zero closing (they're pressing, not falling
  onto a rising horn) -> their wedge-and-shove weakens without any
  special case. Verify in J0->J1 A/B rather than assuming.

## Phase J2 (only if J1 leaves juggles too weak) — penalty window

Expose the height penalty start (NORMAL_HEIGHT 2.0) as JUGGLE_WINDOW
(slider 2..8). Raising it makes the first 1-2 follow-ups full-strength;
decay beyond still kills towers. Quantity-dial, not skill-dial — hence
second, and only if the volley alone reads too weak.

## Gates

- J1 feel session (rhino mirror): pop -> timed flick = clean satisfying
  re-launch; pop -> static hold = victim lands/slides off, holder eats
  ~nothing; ground battles A/B indistinguishable at slider 0 vs pre-J1
  build; ram/charge unchanged.
- Bot canary: lives match, no bot behavior regressions (bots do lifts).
- --bracetrace + --juggletrace on one session: confirm volley-scaled
  forces and brace coexist sanely (no double-punishment of holders).
- Tune -> bake the user's slider values as defaults, same as every
  other feel constant this week.

## Explicitly out of scope
- Any per-type juggle behavior (design rule: shape/motion only).
- Advantage-battle redesign (idle-at-cap is a known quirk; the volley
  factor sidesteps its worst consequence without touching it).
- The rejected juggle-bracing approach.
