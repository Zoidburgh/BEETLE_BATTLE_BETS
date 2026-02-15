# Arena Hazards Design Guide

How hazards work in Beetle Battle. Use this as a reference when adding new hazards.

## Core Principles

- **Affects both players equally** — hazards are neutral obstacles, not advantages for either side
- **Avoidable but disruptive** — players can dodge them, but getting caught is punishing
- **Independent of arena mode** — hazards work on any arena (normal, donut, figure8, yinyang, hourglass, square bridge). They float over geometry (gaps, holes, bridges) and don't need floor contact
- **One toggle per hazard** — each hazard is a simple ON/OFF in the menu. Multiple hazards can be active simultaneously

## Arena Space

- **Movement bounds**: Use yin-yang outer radius (38.0) as the reference since it's the largest arena. Set hazard bounds ~2 units inside (36.0) to account for visual width
- **Coordinate system**: Hazard positions are in world XZ space (same as beetle.x, beetle.z). Y axis is vertical
- **Rendering offset**: Arena surface is at Y = 33.5 (RENDER_Y_OFFSET + 0.5). Spawn visual particles from this floor level

## Movement / Pathing

- **Deterministic from game time** — position must be computed identically on host and guest from shared state (e.g. `tornado_time`, `tornado_phase`). No randomness in pathing
- **Variable speed** — use time-varying speed modulators (sin * cos with incommensurate frequencies) so the hazard drifts, surges, and changes pace. Constant speed feels robotic and predictable
- **Multi-harmonic Lissajous** — layer 3+ sine/cosine terms with irrational frequency ratios so the path never repeats. Add phase offsets so it doesn't start centered
- **Accumulated phase** — integrate speed into a phase variable so speed changes are smooth (no teleporting)
- **Clamp to bounds** — normalize position to bounds radius if it exceeds it
- **Max speed cap** — even at fastest, hazard must move at a pace players can react to. Keep max effective speed under ~0.3

## Character Interaction (Physics)

### Body collision (primary)
The body center (beetle.x, beetle.z) is checked against the hazard position. This is the main interaction and should feel natural. Forces applied:

- **Outward push** — radial force away from hazard center (`PUSH_FORCE * falloff * dt`)
- **Upward lift** — pops beetle off ground so tipping torque works (`LIFT_FORCE * falloff * dt`)
- **Tipping torque** — local-frame roll/pitch via the downwash pattern:
  ```
  cos_r = cos(beetle.rotation)
  sin_r = sin(beetle.rotation)
  local_x = dir_x * cos_r + dir_z * sin_r
  local_z = -dir_x * sin_r + dir_z * cos_r
  beetle.roll_velocity += local_x * tip_mag / max(beetle.roll_inertia, 0.1)
  beetle.pitch_velocity += local_z * tip_mag / max(beetle.pitch_inertia, 0.1)
  ```
- **Tangential force** (optional) — push sideways for swirl/spin effects
- **Yaw spin** (optional) — directly rotate the beetle (`beetle.rotation += strength * dt`)
- **Falloff**: linear `1.0 - dist / RADIUS` (stronger when closer, zero at edge)

### Horn interaction
**Do NOT add separate horn physics.** Lessons learned from the tornado:
- Horn pitch/yaw velocity gets overwritten by input handler every frame — modifying it does nothing
- Directly modifying horn_pitch/yaw after input works but feels wrong (horn moving independently of body)
- Using horn tip for range detection with body-direction forces creates ghost wall / forcefield artifacts
- Horn tip pull-toward-hazard fights body push-away-from-hazard during transitions

**Instead**: set the hazard's effect RADIUS large enough to cover horn reach (~16+ voxels). When a beetle's horn is visually inside the hazard, its body center is within the effect radius. This gives the player the intuitive feel that "my horn is in it so I'm affected" without any special horn code.

### Effect radius guidelines
- 10-12: tight, body-only interaction
- 14-16: covers horn reach, feels like full-beetle interaction (recommended)
- 18+: wide area denial, harder to avoid

## Visuals (Debris Particles)

- **Use the existing debris system** (`simulation.debris_*` fields) — no new rendering code needed
- **Local-only rendering** — particles are cosmetic, not synced over network
- **Taichi kernel** — write a `@ti.kernel` function that spawns particles. Must initialize all variables before if/elif/else branches (Taichi scoping rule)
- **Spawn rate**: 50-75Hz with 20-40 particles per burst for visible density
- **Lifetime**: shorter for particles that move fast or are far from center (prevents messy scatter)
- **Height-based lifetime**: particles at the top/edges should die faster than ones at the base/center
- **Height-based spawn bias**: use `ti.sqrt(ti.random())` to compensate for short-lived top particles
- **Inward radial velocity**: add pull toward center for particles at wider radii to prevent centrifugal drift
- **Color palette**: pick a distinct palette per hazard so they're visually distinguishable from each other and from downwash dust (brown/tan)
- **Warmup**: call the kernel once with offscreen coords (e.g. `0.0, -100.0`) during phase 2 loading to pre-compile

## Menu Integration

- All hazards go under the `=== HAZARDS ===` section in the sidebar menu (after arena modes, before arena colors)
- Button format: `HAZARD_NAME: ON` / `HAZARD_NAME: OFF`
- On toggle: reset all hazard state (time, position, phase, dust timer) to 0
- On toggle: send `send_game_options` to sync to guest
- Hazards are independent of arena modes — toggling an arena doesn't disable hazards, toggling a hazard doesn't change arenas

## Network Sync

### Game options packet
- Add one byte per hazard bool to `send_game_options` in `network.py`
- Update the packet format comment and struct pack/unpack
- Add the new param with `=False` default for backwards compatibility
- In receive: check packet length from largest to smallest, defaulting new fields to `False` for old clients

### All call sites
- Every `send_game_options()` call in `beetle_physics.py` (~9-10 sites) must pass the new hazard bool
- Use `replace_all` in editor to update them all at once since they share the same signature

### Guest handler
- In the guest options handler (~line 15349), add a block that checks `opts.get('hazard_name', False)` and toggles the local state + resets timers

### Determinism
- Hazard position is computed from deterministic state (accumulated time/phase with fixed PHYSICS_TIMESTEP)
- Both host and guest compute the same position independently — no position sync packets needed
- Reset state (time, phase) to 0 on toggle and on `reset_match()` so both sides start synchronized

## State Management

### State variables (module level, near other hazard state)
Each hazard needs at minimum:
- `hazard_mode = False` — toggle
- `hazard_time = 0.0` — raw accumulated time
- `hazard_phase = 0.0` — accumulated phase (for variable-speed movement)
- `hazard_x = 0.0` — current position X
- `hazard_z = 0.0` — current position Z
- `hazard_dust_timer = 0.0` — particle spawn timer

### Reset locations (5 places)
1. **`reset_match()`** — add to global declaration AND reset body. Sets mode to False
2. **Guest handler** — toggle on block (reset timers)
3. **Guest handler** — toggle off block (reset timers)
4. **Menu toggle** — enable block (reset timers)
5. **Menu toggle** — disable block (reset timers)

## Constants Location

All hazard constants go near the top of `beetle_physics.py`, grouped after the downwash constants (~line 1050), with a comment header per hazard:
```python
# Arena [hazard name] hazard
HAZARD_RADIUS = ...
HAZARD_PUSH_FORCE = ...
# etc.
```

## Checklist for Adding a New Hazard

1. Add constants (beetle_physics.py, ~line 1050)
2. Add state variables (beetle_physics.py, ~line 2395)
3. Write Taichi debris kernel (beetle_physics.py, ~line 9650)
4. Add warmup call (beetle_physics.py, phase 2 loading, ~line 13210)
5. Add path + physics in main loop (beetle_physics.py, after downwash section ~line 16075)
6. Add guest handler block (beetle_physics.py, ~line 15349)
7. Add menu button under HAZARDS section (beetle_physics.py, ~line 18373)
8. Add to `send_game_options` signature + packet (network.py, send + receive)
9. Update ALL `send_game_options` call sites (beetle_physics.py, ~9-10 sites)
10. Add to `reset_match()` global + body (beetle_physics.py)
11. Test: toggle on/off, both players affected, online sync, arena compat, reset
