# Air-Traction Nerf + Silk-in-Air Factoring — Implementation Plan

**Status:** planned, NOT yet implemented. Written for a fresh (Fable) context to implement.
**File:** everything is in `beetle_physics.py` unless noted. Line numbers are approximate
(as of 2026-07-09, branch `arena_mods`) — grep the anchor strings if they've drifted.

---

## Goal (user's words)

> "if the beetles are lifted any more than a few voxels, they go to base speed and lose all
> their ability to accelerate more again until they are back on ground... when they get popped
> up they lost their 'traction' and can't gain it back until they touch ground again."
> Keep game feel **consistent for small hops** — the nerf only bites on real pop-ups.

Plus the silk factoring:
> "the spider silk slow should NOT affect you on the board when popped up, but the silk STUCK
> ON your body SHOULD still affect you in the air. And spiders should NOT get the floor-silk
> speed boost when popped up above the silk on the board."

---

## Background: what already exists

There is ALREADY a binary air-control nerf. The problem is it's (a) binary on `on_ground`
(fires on even a 1-voxel hop) and (b) only drops to 0.25, not 0 — and turning stays FULL, so
bots re-aim + ride preserved momentum (`AIR_FRICTION = 0.985`) straight to their target.

**Line ~18510** (anchor: `Air control: airborne beetles keep full turning`):
```python
drive_mult = 1.0 if beetle.on_ground else physics_params.get("AIR_CONTROL", 0.25)
```
`drive_mult` multiplies BOTH forward thrust (line ~18517, `forward_force_mult`) and backward
thrust (line ~18524, `backward_force_mult`).

**Height reference:** ground-rest `beetle.y ≈ 1.0`; 1 voxel ≈ 1.0 unit. Existing
`LIFT_THRESHOLD = 5.0` → `is_lifted_high = beetles[slot].y > 5.0` (line ~21389, "4 voxels above
ground", used only for leg-spaz animation). `on_ground` is set True when the beetle's lowest
geometry point is within 0.5 of the floor surface (lines ~20929–20931).

**Params dict** lives at line ~16836 (anchor: `"AIR_CONTROL": 0.25`), with
`AIR_FRICTION: 0.985`, `AIRBORNE_DAMPING: 0.95` beside it.
**Sliders** for these are at line ~24964 (anchor: `physics_params["AIR_CONTROL"] = window.GUI.slider_float("Air Control"`).

---

## Part A — Height-graded drive nerf

Replace the binary `drive_mult` (line ~18510) with a height ramp:

| Beetle `y` | Lift above ground | `drive_mult` | Feel |
|---|---|---|---|
| ≤ `AIR_GRACE_Y` (2.0) | ≤ ~1 voxel | **1.0** | small hops / bumps: unchanged, full traction |
| 2.0 → 3.5 | 1–2.5 voxels | linear fade 1.0 → floor | short ramp, no cliff |
| ≥ `AIR_DEAD_Y` (3.5) | > ~2.5 voxels | **AIR_CONTROL floor (0.0)** | popped up: ballistic, no thrust |

### New params (add to physics_params dict ~16836)
```python
"AIR_GRACE_Y": 2.0,   # at/below this y, full drive (small-hop grace); ~1 voxel of lift
"AIR_DEAD_Y":  3.5,   # at/above this y, drive == AIR_CONTROL floor; ~2.5 voxels of lift
# REPURPOSE existing AIR_CONTROL: now the MINIMUM drive floor above AIR_DEAD_Y.
# Change default 0.25 -> 0.0 (set back to 0.25 to restore old behavior for A/B).
"AIR_CONTROL": 0.0,
```

### New drive_mult (replace line ~18510)
```python
# Air traction: full drive for small hops, fading to the AIR_CONTROL floor once
# lifted more than a few voxels. Regained only on landing (on_ground below).
_grace_y = physics_params.get("AIR_GRACE_Y", 2.0)
_dead_y  = physics_params.get("AIR_DEAD_Y", 3.5)
_air_floor = physics_params.get("AIR_CONTROL", 0.0)
if beetle.on_ground or beetle.y <= _grace_y:
    drive_mult = 1.0
elif beetle.y >= _dead_y:
    drive_mult = _air_floor
else:
    t = (beetle.y - _grace_y) / max(1e-6, _dead_y - _grace_y)  # 0..1
    drive_mult = 1.0 + (_air_floor - 1.0) * t
```
Note: keep the `beetle.on_ground` short-circuit so a grounded-but-geometrically-tall pose
(e.g. reared up) never loses drive.

### "Go to base speed" while high (matches "they go to base speed")
`forward_bonus`/`backward_bonus` already only accrue on ground (lines ~18494–18506, holds pause
mid-air). The remaining cheese is a beetle that BUILT the bonus, then gets popped and keeps its
boosted top-speed cap. Neutralize the bonus in the speed cap while high.

Cleanest spot: the max-speed cap at lines ~2147–2148 (`Beetle.update_physics`):
```python
forward_max = base_forward * self.silk_speed_mult * (1.0 + self.forward_bonus) * ice_speed_mult
backward_max = base_backward * self.silk_speed_mult * (1.0 + self.backward_bonus) * ice_speed_mult
```
`update_physics` doesn't currently know the grace/dead thresholds. Two options:
- **(preferred)** add a per-beetle flag `self.air_no_traction` (default False), set it in the
  main loop right after computing `drive_mult` (`beetle.air_no_traction = (drive_mult < 1.0 and not beetle.on_ground)`),
  and in update_physics use `bonus_mult = 0.0 if self.air_no_traction else 1.0` →
  `(1.0 + self.forward_bonus * bonus_mult)`. Clamp-DOWN only (never a gain); small hops
  (drive_mult == 1.0) keep their bonus.
- (simpler, cruder) skip this and accept that a boosted beetle keeps boosted momentum through a
  pop. Ship Part A's drive ramp first, playtest, add the cap clamp only if it still feels cheesy.

### Turning stays full (recommend)
Do NOT nerf yaw. With drive at 0 the beetle can still rotate to face its landing but can't move
there — that's what kills bot tracking while preserving flip/recovery feel. (Optional future
knob: reduce air-turn authority for full helplessness — leave OFF by default.)

### Sliders (add near line ~24964, beside "Air Control")
```python
physics_params["AIR_GRACE_Y"] = window.GUI.slider_float("Air Grace Y", physics_params["AIR_GRACE_Y"], 1.0, 4.0)
physics_params["AIR_DEAD_Y"]  = window.GUI.slider_float("Air Dead Y",  physics_params["AIR_DEAD_Y"], 2.0, 8.0)
# existing "Air Control" slider now reads as the min air-drive floor (0.0..1.0)
```

---

## Part B — Silk factoring in the air

### The two silk effects (both computed each frame at lines ~18479–18490)
```python
if silk_might_exist and silk_counts is not None:
    silk_slowdown[slot] = max(0.0, 1.0 - 0.01 * silk_counts[slot * 2])      # (1) BODY silk
    floor_silk_count = silk_counts[slot * 2 + 1]                            # (2) FLOOR/BOARD silk
    if beetle.horn_type_id == 6:  # Spider
        floor_modifier[slot] = 1.0 + 0.05 * floor_silk_count               #   spider: +5%/silk BOOST
    else:
        floor_modifier[slot] = max(0.0, 1.0 - 0.01 * floor_silk_count)     #   others: -1%/silk slow
    speed_mult[slot] = silk_slowdown[slot] * floor_modifier[slot]
else:
    speed_mult[slot] = 1.0
beetle.silk_speed_mult = speed_mult[slot]
```
- **(1) `silk_slowdown` = silk STUCK ON THE BODY** (`silk_counts[slot*2]`, counted via
  `silk_on[slot]`). This is silk physically wrapped on the beetle → **must STILL apply in air.**
- **(2) `floor_modifier` = silk on the BOARD under the beetle** (`silk_counts[slot*2+1]`, counted
  by `count_floor_silk_under` — a 2D x/z radius check that does NOT care about the beetle's
  height, so a popped-up beetle still "counts" floor silk under it). Spider gets +5%/silk boost,
  others get −1%/silk slow. → **must be NEUTRALIZED when popped up** (no board contact), which
  also removes the spider's boost above the silk, exactly as requested.

`speed_mult[slot]` flows into both the thrust force (lines ~18517/18524) and the max-speed cap
(via `beetle.silk_speed_mult`, lines ~2147–2148), so gating it here fixes both.

### The change (replace the floor_modifier branch, ~18481–18487)
Gate the floor effect on the same grace height as Part A — board silk only affects you while
you're touching/near the board:
```python
    silk_slowdown[slot] = max(0.0, 1.0 - 0.01 * silk_counts[slot * 2])  # body silk: always applies
    floor_silk_count = silk_counts[slot * 2 + 1]
    # Board silk only grips while grounded / in a small hop. Popped up = off the board:
    # no floor slow for others AND no floor boost for spiders.
    on_board = beetle.on_ground or beetle.y <= physics_params.get("AIR_GRACE_Y", 2.0)
    if not on_board:
        floor_modifier[slot] = 1.0
    elif beetle.horn_type_id == 6:  # Spider
        floor_modifier[slot] = 1.0 + 0.05 * floor_silk_count
    else:
        floor_modifier[slot] = max(0.0, 1.0 - 0.01 * floor_silk_count)
    speed_mult[slot] = silk_slowdown[slot] * floor_modifier[slot]
```
Body silk (`silk_slowdown`) is left untouched, so a beetle webbed while airborne is still slowed
by the silk on its body — including the base-speed cap while high (Part A's bonus clamp and this
body-silk factor stack correctly, both are clamp-downs).

Use the SAME `AIR_GRACE_Y` gate as Part A (not a separate threshold) so "on the board" is one
consistent concept: a small hop over silk still feels it; a real pop-up escapes it.

---

## Defaults summary

| Param | Old | New | Meaning |
|---|---|---|---|
| `AIR_CONTROL` | 0.25 | **0.0** | drive floor once fully airborne (was the whole air mult) |
| `AIR_GRACE_Y` | — | **2.0** | ≤ this y = full drive + board silk applies (~1 voxel lift) |
| `AIR_DEAD_Y` | — | **3.5** | ≥ this y = drive at floor, no traction (~2.5 voxel lift) |

Set `AIR_CONTROL` back to 0.25 (and it'll behave close to today, minus the height grace) for A/B.

---

## Verification

- **Headless / logic:** import `simulation` (NOT `beetle_physics`, which launches the game).
  For the drive ramp, unit-test the pure `drive_mult` formula at y = 1.0 (→1.0), 2.0 (→1.0),
  2.75 (→~0.5 with floor 0.0), 3.5 (→floor), 6.0 (→floor).
- **Silk:** confirm with a spider on floor silk — boost present at y≤2.0, gone (mult→1.0×body)
  when lifted above `AIR_GRACE_Y`; a non-spider on floor silk loses the slow when popped;
  a body-webbed beetle stays slowed at all heights.
- **Live:** `--local4` smoke run; watch that bots can no longer steer to target mid-launch and
  that small hops still feel normal. Big run-to-run FPS variance on this machine — this is a
  physics-only change, expect ~0 FPS impact.
- **Feel risk:** this nerfs the HUMAN identically (it's physics, not bot logic). If pops feel
  too punishing, raise `AIR_DEAD_Y` or lift the `AIR_CONTROL` floor off 0 via the sliders.

## Do NOT
- Don't nerf turning/yaw by default.
- Don't touch `silk_slowdown` (body silk) — it must keep working in air.
- Don't push to GitHub (project rule: only on explicit "push"). Commits are fine.

---

## AUDIT ADDENDUM (Fable, 2026-07-09) — one showstopper, plan otherwise sound

**`beetle.on_ground` is STICKY-TRUE and must not be trusted for "airborne".** Verified every
assignment site: it is set True in the floor block (anchor `elif lowest_point < floor_surface + 0.5`)
and set False ONLY at death-fall (x3), respawn (x2), and hover-drop (x1). **No gameplay event —
horn launch, wedge lift, yaw-grind pop — ever clears it.** The floor block has no else branch.
Consequences:

1. The plan's short-circuit `if beetle.on_ground or beetle.y <= _grace_y: drive_mult = 1.0`
   makes the entire height ramp DEAD CODE for horn pop-ups (the target case): a launched beetle
   still has on_ground == True all flight.
2. The existing binary nerf (commit 1725bea) is mostly INERT for the same reason — AIR_CONTROL,
   AIR_FRICTION, and the boost-ramp pause only ever fired between respawn/hover-drop and first
   landing. Don't A/B against it expecting to feel a difference. Replace it wholesale.
3. Do NOT "fix" on_ground globally (adding an else-False) — tilt damping, restoring torque,
   both-airborne collision checks, and ball logic all read it, and the airborne-tumbling sliders
   were user-tuned under the sticky semantics. Too blast-radius-y for this change.

**The fix — use height-above-floor gap instead of on_ground/absolute y.** The floor block already
computes `lowest_point_kernels[slot](...)` and `floor_surface = floor_y_by_slot[slot] + 0.5`
every step (zero added kernel cost). Cache it per beetle:
```python
# in the floor block (floor detected branch):
beetles[slot].air_gap = max(0.0, lowest_point - floor_surface)
# in the no-floor branch (floor_y <= -100: over edge / hole / board-break):
beetles[slot].air_gap = 999.0
# Beetle.__init__ (next to self.on_ground): self.air_gap = 0.0  (reset at respawn too)
```
The input block runs BEFORE the floor block in a step, so it reads a 1-tick-stale gap — fine at 60Hz.
Then everywhere the plan says `beetle.on_ground or beetle.y <= AIR_GRACE_Y`, use
`beetle.air_gap <= AIR_GRACE_LIFT` instead. This is strictly better than absolute y:
- build-independent (rest y varies with leg length/geometry; gap is always ~0 at rest),
- reared-up poses keep drive automatically (lowest point still touches floor) — no on_ground
  short-circuit needed,
- falling off the edge / into a hole / through board-break → no floor → gap 999 → nerfed
  (absolute-y wrongly RESTORES full drive once y drops below 2.0 mid-fall),
- raised/varying floor handled for free.

**Threshold conversion** (plan's y-values assumed rest y ≈ 1.0): `AIR_GRACE_LIFT = 1.0`,
`AIR_DEAD_LIFT = 2.5` (voxels of daylight under the beetle's lowest point). Slider ranges 0.5–3.0
and 1.5–7.0.

**Gap gate must also replace on_ground at these three other sites** (all currently inert):
- boost-ramp pause (anchor `Ramp pauses (doesn't reset) while airborne`): pause when
  `air_gap > AIR_GRACE_LIFT`,
- `AIR_FRICTION` selection in `update_physics` (anchor `linear_friction = ICE_LINEAR_FRICTION`):
  `self.air_gap > grace` → AIR_FRICTION, else ground FRICTION (self.air_gap is on the beetle, and
  update_physics already reads physics_params),
- Part B's `on_board` gate: `on_board = beetle.air_gap <= AIR_GRACE_LIFT`.

The `air_no_traction` flag option for the bonus cap works as written — verified the input block
runs before `update_physics(...)` (anchor `beetles[_slot].update_physics(PHYSICS_TIMESTEP`) in the
same tick, so the flag is fresh.

**Bots/players coverage confirmed:** the drive/silk code is inside the per-slot input loop
(`p_inputs = frame_inputs[slot]`) which bots (local `get_bot_inputs` slots 2/3, host bots via
`_handle_packet`) and net guests all flow through — physics-level, no special-casing. Everything
else in the plan (Part B structure, defaults, turning untouched, verification recipe) checks out
against the code.
