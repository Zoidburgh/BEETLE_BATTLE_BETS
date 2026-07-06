# Networking Rework Plan: Lockstep -> Host-Authoritative + 4-Player Support

## Goal
Switch from lockstep (both players wait for each other's input) to host-authoritative (host runs physics, guests receive state). This removes freezes, cuts host input lag to zero, and is the foundation for 4-player support.

## STATUS (2026-07-06)
**Phase 1 (host-authoritative 2P) is IMPLEMENTED** — commits `adbd49c`..`e69219b` on `arena_mods`:
- Host + guest never block (InputBuffer.host_authoritative, last-known input fallback), input delay 0 both sides
- State sync v2: 88B @ 20Hz with velocities + active/is_falling flags; deadzone/lerp/snap correction scheme (tunables at the guest sync-apply block in beetle_physics.py)
- Wall-clock disconnect detection; PROTOCOL_VERSION=2 handshake (legacy packet compat deleted)
- Net debug HUD (N key online), `--simlag <ms>` / `--simloss <pct>` network simulator
- NEXT: 2-computer test session, then Phase 2 (beetles[] array refactor)

## CORRECTIONS to this doc (found during implementation)
- **Voxel IDs for P3/P4 must start at 51+**, not 20-31 as written below — 20-50 are taken (SHADOW=20, SLIPPERY=21, GOAL=22, ... ICE_PATCH=50). Also the enumerated blue/red voxel lists (UFO beam, comet, spray, silk kernels) must be replaced with `is_beetle_voxel()`/`beetle_owner()` ti.func helpers or P3/P4 are invisible to hazards.
- **"Kernels already work for N beetles" is too optimistic**: `place_animated_beetle_blue/red` are ~560-line duplicate kernels compile-time bound to per-color Taichi field sets (`blue_body_cache_*` etc.). Phase 4 needs a kernel factory over per-player fields — it is the largest single work item.
- The beetle_blue/beetle_red reference count is ~849, not 762.
- Disconnect detection had to move to wall-clock (done) — `frames_waited` only advanced while lockstep-blocked.

---

## PART 1: Current Architecture (What We Have)

### Networking Model: Lockstep
- Transport: Steam P2P via `py_steam_net` (NAT traversal handled by Steam)
- Both host and guest run identical physics simulations
- Neither side can advance a frame without the other's input
- 8-frame artificial input delay (133ms) on BOTH players, on top of real network latency
- If either player's packet is late, BOTH freeze until it arrives (or 3-second disconnect timeout)

### Key Files
- **network.py** — Steam lobby, P2P messaging, packet pack/unpack
- **beetle_physics.py** — Physics, rendering, input buffer, game logic (~20k lines)

### Message Protocol (network.py:39-56)
| Message | ID | Size | Reliability | Frequency |
|---------|-----|------|-------------|-----------|
| MSG_INPUT | 0x01 | 6B | Unreliable | Every frame (60Hz) |
| MSG_STATE_SYNC | 0x08 | 50B | Unreliable | Every 5 physics frames |
| MSG_FRAME_SYNC | 0x0B | 5B | Unreliable | Every 30 frames |
| MSG_GAME_OPTIONS | 0x0E | 11B | Reliable | On toggle |
| MSG_SCORE | 0x0D | 11B | Reliable | On death/goal |
| MSG_BEETLE_CONFIG | 0x0C | 24B | Reliable | In lobby |
| MSG_RECONNECT_STATE | 0x10 | 112B | Reliable | On reconnect |
| MSG_PING/PONG | 0x06/07 | 5B | Unreliable | Periodic |
| MSG_GO | 0x0A | 1B | Reliable (3x) | Match start |
| MSG_BALL_EXPLODE | 0x12 | 13B | Reliable | On ball explode |

### Current Bandwidth (~7.7 kbps total for 2 players)
- Input: 6 bytes x 60fps = 360 B/s per player
- State sync: 50 bytes x 12/sec = 600 B/s (host only)

### InputBuffer Class (beetle_physics.py:740-880)
```
NETWORK_INPUT_DELAY = 8       # Fixed 133ms delay on both players
local_inputs = {}             # frame_num -> input_bits (our inputs)
remote_inputs = {}            # frame_num -> input_bits (opponent inputs)
local_player_id = 0 or 1     # 0 = host (blue), 1 = guest (red)
```
- `can_simulate()` (line 807): Returns True ONLY if both local AND remote inputs exist for the frame. This is the lockstep gate.
- `get_frame_inputs()` (line 848): Returns (blue_inputs, red_inputs) by swapping local/remote based on player_id.
- Input prediction (line 825-834): If exact frame missing but newer frame exists, copies latest known input.

### State Sync (Already Exists)
- **Host sends** every 5 frames (line 14412): positions + rotations for both beetles + ball (50 bytes)
- **Guest applies** with 50% lerp (line 14421-14501): blends toward host positions, wraps rotations
- **Reconnect state** (network.py:610-645): Full snapshot with velocities, scores, timers (112 bytes)
- **Frame sync** (line 14408): Host sends frame counter every 30 frames, guest speeds up/slows down to match

### Physics Loop Stall Point (beetle_physics.py:14578-14585)
```python
if not input_buffer.can_simulate():
    network_stalled = True
    accumulator = 0   # Drain - prevents catch-up after stall
    break             # Exit physics loop entirely
```
This is where both players freeze when waiting for input.

### Hardcoded 2-Player Assumptions

**Network layer (network.py):**
- `peer_steam_id = None` (line 109) — single opponent, not a list
- `create_lobby(max_players=2)` (line 226) — defaults to 2
- `local_player_id = 0 or 1` only (line 763)
- State sync packet: named `blue_x, blue_y, ... red_x, red_y` (line 484-488)
- Score: `scorer = 0` (blue scores) or `1` (red scores) (line 552)
- Reconnect: explicit `blue_state`, `red_state` dicts (line 624-642)

**Game logic (beetle_physics.py):**
- Global variables `beetle_blue` and `beetle_red` (line 1781-1782) — referenced ~762 times total
- `InputBuffer.local_inputs` + `remote_inputs` — exactly 2 streams
- `get_frame_inputs()` returns `(blue_inputs, red_inputs)` — hardcoded swap logic

**Taichi kernels (beetle_physics.py):**
- Voxel types: `BEETLE_BLUE` (5) and `BEETLE_RED` (6) with sub-types (legs=7/8, tips=9/10, etc.)
- Collision kernel `check_collision_kernel()` (line 3383): takes color1/color2 params, checks blue vs red voxels
- `beetle_collision()` (line 12532): called once for blue-vs-red, twice for ball-vs-beetle
- All rendering kernels reference specific beetle color IDs

**Collision (beetle_physics.py):**
- Currently: 1 beetle-beetle pair per tick
- 4 players would be: 6 pairs per tick (n*(n-1)/2)
- Each pair: early distance rejection -> voxel overlap scan -> spatial hash collision point
- GPU-accelerated, O(overlap_area) per pair, NOT naive O(n^2) on voxels
- Performance: should handle 6 pairs fine (early rejection saves most work when beetles are apart)

---

## PART 2: Target Architecture (What We Want)

### Model: Host-Authoritative
- **Host** runs physics at 60Hz unconditionally, never waits for anyone
- **Host** applies guest inputs when available, uses last-known input if late
- **Host** broadcasts state to all guests every 3 frames (~50ms)
- **Guests** send their input to host, run local prediction, lerp toward host state
- **No freezes** — late packets just mean stale input for a few frames

### What Each Player Experiences

| | Host (Player 1) | Guest (Player 2/3/4) |
|---|---|---|
| Input lag | ~0ms (immediate) | ~80-100ms (variable, ping-dependent) |
| Freezes | Never | Never |
| Own beetle | Authoritative | Local prediction + lerp correction |
| Other beetles | Authoritative | State sync + interpolation |
| Physics | Runs locally | Display only (driven by host state) |

### Lobby / Connection Changes (network.py)

**Current:**
```python
self.peer_steam_id = None          # Single opponent
```

**Target:**
```python
self.peers = {}                    # {steam_id: player_slot} — up to 3 guests
self.peer_names = {}               # {steam_id: display_name}
self.player_count = 1              # 1 (host) to 4
```

- `create_lobby(max_players=4)` — already parameterized (line 226)
- Lobby member detection (line 313-317): loop assigns slots instead of single `peer_steam_id`
- Host assigns player_slot (0=host, 1-3=guests in join order)
- Send slot assignment to each guest on join

### Player ID System

**Current:** `local_player_id = 0` (host/blue) or `1` (guest/red)

**Target:** `local_player_id = 0..3` (host=0, guests=1-3)

Colors: Player 0=Blue, Player 1=Red, Player 2=Green, Player 3=Yellow (or user-chosen presets)

### Input Handling Changes

**Current (lockstep):**
- Both players send MSG_INPUT every frame
- `can_simulate()` blocks until both arrive
- 8-frame artificial delay on everyone

**Target (host-auth):**
- Guests send MSG_INPUT to host (same packet, 6 bytes)
- Host never waits — uses last-known input for any guest whose packet hasn't arrived
- Host has 0 input delay
- Remove `can_simulate()` gate on host
- Guest input delay is natural network latency only (no artificial 8-frame delay)

**InputBuffer rework:**
```python
# Old
self.local_inputs = {}       # frame -> bits
self.remote_inputs = {}      # frame -> bits

# New
self.player_inputs = {0: {}, 1: {}, 2: {}, 3: {}}  # player_slot -> {frame -> bits}
self.last_known_input = {0: 0, 1: 0, 2: 0, 3: 0}   # fallback when frame missing
```

`get_frame_inputs()` returns a list of N inputs instead of (blue, red).

### New Packet Format: MSG_STATE_SYNC

**Current (50 bytes, 2 beetles):**
```
[type:1][frame:4]
[blue_x:4][blue_y:4][blue_z:4][blue_rot:4]
[red_x:4][red_y:4][red_z:4][red_rot:4]
[ball_x:4][ball_y:4][ball_z:4][ball_active:1]
```

**Target (variable length, N beetles):**
```
[type:1][frame:4][player_count:1]
[p0_x:4][p0_y:4][p0_z:4][p0_rot:4]  — 16 bytes per beetle
[p1_x:4][p1_y:4][p1_z:4][p1_rot:4]
[p2_x:4][p2_y:4][p2_z:4][p2_rot:4]  — only if 3+ players
[p3_x:4][p3_y:4][p3_z:4][p3_rot:4]  — only if 4 players
[ball_x:4][ball_y:4][ball_z:4][ball_active:1]
```

Size: 6 + (16 x N) + 13 = **83 bytes for 4 players** (vs 50 now for 2)

Increase frequency from every 5 frames to every 3 frames (~50ms interval).

### New Packet Format: MSG_RECONNECT_STATE

Same idea — variable number of beetle states. Include velocities for each.

### New Packet Format: MSG_SCORE

**Current:** `scorer = 0 or 1`

**Target:** `scorer = 0..3` (which player scored), `victim = 0..3` (who died/got scored on)

### New Packet Format: MSG_GAME_OPTIONS

No structural change needed — hazards and arena modes are global, not per-player.

### Physics Loop Changes (beetle_physics.py)

**Host physics loop (remove lockstep gate):**
```python
# OLD (line 14578-14585):
if not input_buffer.can_simulate():
    network_stalled = True
    accumulator = 0
    break

# NEW:
# Host always simulates. For any missing guest input, use last_known_input.
# Guest: still runs local prediction, but no hard block either.
```

**Guest behavior:**
- Guest runs local prediction for its own beetle (responsive feel)
- Other beetles: interpolate between received state sync positions
- No local physics for other beetles — just smooth visual interpolation
- Apply state sync corrections with lerp (already exists, increase frequency + factor)

### Beetle Array Refactor (beetle_physics.py)

**Current:**
```python
beetle_blue = Beetle(-20.0, 0.0, 0.0, simulation.BEETLE_BLUE)
beetle_red = Beetle(20.0, 0.0, math.pi, simulation.BEETLE_RED)
```

**Target:**
```python
beetles = [
    Beetle(-20.0, -10.0, 0.0, BEETLE_COLORS[0]),        # Player 0 (host)
    Beetle(20.0, 10.0, math.pi, BEETLE_COLORS[1]),       # Player 1
    Beetle(-20.0, 10.0, math.pi/2, BEETLE_COLORS[2]),    # Player 2 (if present)
    Beetle(20.0, -10.0, -math.pi/2, BEETLE_COLORS[3]),   # Player 3 (if present)
]
active_player_count = 2  # or 3 or 4
```

Then replace all `beetle_blue.x` with `beetles[0].x` and `beetle_red.x` with `beetles[1].x`.

This is the biggest single refactor — ~762 references to `beetle_blue.` / `beetle_red.` across the file.

### Taichi Voxel Type Expansion

**Current voxel types (per beetle):**
```
BEETLE_BLUE = 5,  BEETLE_RED = 6       # Body
BEETLE_BLUE_LEGS = 7, BEETLE_RED_LEGS = 8
BEETLE_BLUE_LEG_TIPS = 9, BEETLE_RED_LEG_TIPS = 10
BEETLE_BLUE_STRIPE = 11, BEETLE_RED_STRIPE = 12
BEETLE_BLUE_HORN_TIP = 13, BEETLE_RED_HORN_TIP = 14
BEETLE_BLUE_STAG = 18, BEETLE_RED_STAG = 19
```

**Target:** Add 2 more sets (6 types each = 12 new IDs) for players 3 and 4.
```
BEETLE_P3 = 20, BEETLE_P3_LEGS = 21, ... BEETLE_P3_STAG = 25
BEETLE_P4 = 26, BEETLE_P4_LEGS = 27, ... BEETLE_P4_STAG = 31
```

Collision kernel `check_collision_kernel()` already takes `color1, color2` params — it should work with new IDs without structural changes.

### Collision Scaling

**Current:** 1 pair checked per tick (blue vs red)

**4 players:** 6 pairs per tick. Loop over all combinations:
```python
for i in range(active_player_count):
    for j in range(i + 1, active_player_count):
        if beetles[i].active and beetles[j].active:
            beetle_collision(beetles[i], beetles[j], physics_params)
```

Performance: fine. Each pair has early distance rejection. GPU kernels handle the heavy lifting. The spatial hash collision point finder is O(N+M) per pair, not O(N*M).

Ball collision: loop over all beetles instead of hardcoded blue/red.

### Spawn Positions (4 players)

Current: Blue at (-20, 0), Red at (20, 0), facing each other.

4-player layout (corners of the arena):
```
Player 0: (-18, -18), facing center (45 degrees)
Player 1: (18, 18), facing center (225 degrees)
Player 2: (-18, 18), facing center (315 degrees)
Player 3: (18, -18), facing center (135 degrees)
```

Arena radius is 32 — these positions keep everyone inside with room to maneuver.

### Scoring / Win Condition

**Current:** First to N kills wins. Blue score vs Red score.

**4 players — options:**
- Free-for-all: each player has a kill count, first to N wins
- Last-beetle-standing: eliminated on ring-out, last one alive wins the round
- Team 2v2: pairs share a score

Start with free-for-all (simplest). `scores = [0, 0, 0, 0]` indexed by player slot.

### UI Changes

**Lobby:**
- Show up to 4 player slots with names
- Host can start when 2+ players are connected

**In-match HUD:**
- Score display for all active players
- Skin selection per player (already indexed by preset)

**Menus:**
- Host controls (hazards, arena, referee) stay the same — they're global

---

## PART 3: Implementation Order

### Phase 1: Host-Authoritative for 2 Players (smallest useful change)
This improves the existing 1v1 experience without any player count changes.

1. **Remove lockstep gate on host** — host always runs physics, uses last-known input for late guest input
2. **Remove 8-frame artificial delay on host** — host input is immediate
3. **Guest keeps local prediction** — runs own physics for its beetle, lerps toward host for opponent
4. **Increase state sync frequency** — from every 5 frames to every 3 frames
5. **Increase lerp factor** — from 0.5 to 0.7 for snappier guest correction
6. **Test thoroughly** — should feel better for both players, especially host

### Phase 2: Beetle Array Refactor
Replace `beetle_blue`/`beetle_red` globals with `beetles[]` array. This is purely structural and doesn't change behavior.

1. Create `beetles = [beetle_blue, beetle_red]` list, keep old names as aliases initially
2. Refactor input application to use `beetles[player_id]` in a loop
3. Refactor collision to loop over pairs
4. Refactor rendering to loop over beetles
5. Refactor scoring to use player index
6. Remove old `beetle_blue`/`beetle_red` aliases once everything uses the array

### Phase 3: Network Protocol for N Players
Extend the protocol to handle variable player counts.

1. `peer_steam_id` -> `peers` dict with slot assignment
2. Variable-length state sync packets
3. Input routing: each guest sends to host, host broadcasts state to all
4. Score packets with player indices
5. Lobby UI for 2-4 player slots

### Phase 4: Taichi + Rendering for 4 Beetles
Add voxel types and rendering support for players 3 and 4.

1. Add BEETLE_P3 / BEETLE_P4 voxel type constants
2. Add color palettes for players 3 and 4
3. Spawn position logic for 2/3/4 player layouts
4. 4-way collision loop
5. Camera adjustments for wider arena view

### Phase 5: Game Mode + Polish
Free-for-all scoring, UI for 4 players, playtesting.

---

## PART 4: Risk Assessment

| Risk | Severity | Mitigation |
|------|----------|------------|
| Guest beetle feels "slippery" from lerp corrections | Medium | Tune lerp factor + sync frequency. Can also send velocities in state sync for better interpolation |
| Collision desync (host and guest disagree) | Low | Only host runs authoritative collision — guests display host result |
| Bandwidth with 4 players | Low | 83B state sync x 20/sec x 3 guests = ~5 KB/s total. Trivial |
| Taichi recompilation with new voxel types | Low | Just add new constants, kernels already parameterized by color |
| beetle_blue/beetle_red refactor breaks things | High | Do it incrementally — alias first, then replace. Test after each file section |
| 762 references to beetle_blue/beetle_red | High | Use find-replace carefully. Many are in physics, rendering, collision, UI — each section needs individual attention |
| Input prediction feels different per-player | Medium | Host feels great (0 delay). Guests may notice slight difference from lockstep. Overall better than 133ms fixed delay |

---

## PART 5: Quick Reference — Key Line Numbers

All line numbers are approximate (will shift as code changes).

**beetle_physics.py:**
| What | Line | Notes |
|------|------|-------|
| InputBuffer class | 740-880 | Needs per-player input dicts |
| NETWORK_INPUT_DELAY = 8 | 755 | Remove for host, reduce for guests |
| can_simulate() | 807-846 | Remove lockstep gate on host |
| get_frame_inputs() | 848-871 | Return N inputs instead of 2 |
| beetle_blue / beetle_red creation | 1781-1782 | Replace with beetles[] array |
| check_collision_kernel | 3383 | Already parameterized by color — works for N beetles |
| beetle_collision() | 12532 | Call in pair loop instead of once |
| Physics loop stall | 14578-14585 | Remove accumulator drain, host always runs |
| State sync send | 14412-14418 | Variable-length packet, increase frequency |
| State sync receive + lerp | 14421-14501 | Per-beetle lerp loop |
| Frame sync | 14408-14409 | Keep as-is |
| Blue input application | 14643-14755 | Refactor to beetles[i] loop |
| Red input application | 14987-15107 | Merge into same loop as blue |
| Collision call | 17070 | Loop over all pairs |

**network.py:**
| What | Line | Notes |
|------|------|-------|
| peer_steam_id | 109 | Replace with peers dict |
| create_lobby | 226 | Already has max_players param |
| send_input | 431-441 | No change (guests still send 6B) |
| send_state_sync | 473-489 | Variable-length for N beetles |
| send_score | 552-566 | Add victim field |
| send_game_options | 568-586 | No change (global settings) |
| send_reconnect_state | 610-645 | Variable-length for N beetles |
| MSG_STATE_SYNC unpack | 845-872 | Variable-length parse |
| MSG_INPUT receive | 786-790 | Route to correct player slot |
