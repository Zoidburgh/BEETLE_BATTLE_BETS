# Ground Turn Ramp — turning gets more powerful on the ground
2026-07-18. Status: PLANNED, not started. User design: the turning twin of the
forward-speed ramp — "a charge on the ground and a turn that gets more
powerful on the ground; air turn stays base, no ramp."

## The mechanic

- **Grounded turning ramps +30% over 2 seconds of sustained turning.**
  `turn_mult = 1 + TURN_RAMP_BONUS(0.3) x min(turn_hold_time / TURN_RAMP_TIME(2.0), 1)`
  applied at the rotation_multiplier composition site (multiplies with the
  existing stack: speed-ramp turn penalty, stationary 1.3x pivot boost,
  horn-lock clamp, per-type turn stat).
- **Grounded only**: hold time accrues only while air_gap <= AIR_GRACE_LIFT
  (the same "on board" gate the speed ramp uses). Airborne turn = base speed,
  exactly as today — steer your landing, never ramp it.
- **PURE PIVOT RULE (user, final)**: the ramp accrues ONLY while holding
  exactly one turn key with NO forward/backward input, grounded. This also
  eliminates the circling interaction — you cannot build charge and turn
  ramps simultaneously by design.
- **Wipes** (hard resets, feel compass): key release; DIRECTION CHANGE
  (L<->R); ANY forward/backward press; popped off the board; KNOCKED
  (reuse the knockback-grace impulse>6 trigger).

## Dust escalation over the ramp

The spin-dust path (is_rotating_only branch, spawn_spin_dust_puff on the
0.04s timer) scales with ramp fraction:
- interval 0.04 -> ~0.025 at full ramp (more frequent puffs)
- puff scale/count x(1 + ramp) — mirror the walk dust's ramp-keyed language:
  quiet scuffs at spin start -> visible dust ring at full spin power.
- Turning-while-walking uses the walk-dust path — leave it (the walk ramp
  already owns that language).

## Interactions to verify in test

- Circling: NOT possible to double-ramp (pure pivot rule) — drive input
  wipes the turn ramp instantly.
- Turn ramp + yaw-grind combat: faster grounded turning feeds the crossing
  detector + turn-clamp paths — canary the horn_crossing/deep_clip after.
- Networking: input-derived + deterministic (like the speed ramp) — guests
  predict it from the same code; nothing to sync.

## Implementation sketch (one session)

1. Beetle attrs: turn_hold_time (+ last_turn_dir). Accrue/wipe in the input
   block next to forward_hold_time (same neighborhood, same pattern).
2. turn_mult into rotation_multiplier line (~20927 region).
3. Wipe hook at the knockback-grace site (one line).
4. Dust: ramp fraction into the spin-dust timer + puff args.
5. Params + sliders: TURN_RAMP_BONUS (0-0.8), TURN_RAMP_TIME (0.5-5).
6. Gate: user feel test (committed spins feel escalating; air unchanged;
   knock resets) + one canary for the combat-path stats.
