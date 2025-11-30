# Beetle Battle Networking Strategy

> **AI CONTEXT DOCUMENT**: This file explains the networking implementation plan for Beetle Battle. Read this first when working on multiplayer features.

## TL;DR for AI

This game is **easy to network** because:
1. **Input-based sync** - Only send 8 bits of button presses per frame, not game state
2. **Deterministic physics** - Physics runs in Python (not GPU), fixed 60Hz timestep
3. **Static environment** - Voxel arena is identical on both clients, no sync needed
4. **Small state** - Beetles have ~16 floats, but we don't sync state, just inputs

**Architecture**: Peer-to-peer with 4-frame input delay using Steam networking.

---

## Why This Is Simpler Than It Looks

### Common Networking Fears vs. Reality

| Worry | Reality |
|-------|---------|
| "128³ voxel grid = 2.1MB to sync" | **NO** - Arena is static, beetle voxels computed from position. Sync nothing. |
| "Taichi GPU = non-deterministic" | **NO** - Physics is Python code, Taichi only renders voxels. |
| "Complex beetle state" | **NO** - Just sync inputs (8 bits), not state (16 floats). |
| "Rollback netcode required" | **NO** - Delay-based works fine for casual 1v1. |

### The Magic of Input Synchronization

```
Frame 100 on BOTH machines:
├── Blue inputs: forward + horn_up = 0x11
├── Red inputs: left + horn_down = 0x24
├── Physics.update(blue, 0x11)  ← Identical code
├── Physics.update(red, 0x24)   ← Identical inputs
└── Result: Identical game state ← No state sync needed!
```

Both clients run identical code with identical inputs = identical results.

---

## Architecture Overview

```
┌─────────────────────────────────────────────────┐
│  LOCAL MACHINE                                   │
│  ┌─────────┐   ┌──────────────┐   ┌──────────┐ │
│  │Keyboard │──>│ InputBuffer  │──>│ Physics  │ │
│  └─────────┘   │ (4-frame     │   │ Loop     │ │
│       ↓        │  delay)      │   │          │ │
│  pack as       └──────────────┘   └──────────┘ │
│  6 bytes             ↑                          │
│       ↓              │                          │
│  ┌─────────────────────────────────────────────┤
│  │ Steam P2P (py_steam_net)                    │
│  │ - NAT traversal handled automatically       │
│  │ - Reliable message delivery                 │
│  │ - 6 bytes/frame = 360 bytes/sec            │
└──┴─────────────────────────────────────────────┘
         ↕ Internet (50-150ms typical)
┌────────────────────────────────────────────────┐
│  REMOTE MACHINE (mirror of above)              │
└────────────────────────────────────────────────┘
```

---

## Implementation Phases

### Phase 1: Input Abstraction (REQUIRED FIRST)
**File**: `beetle_physics.py` lines 6242-6515

Current code reads keyboard directly in physics loop. Refactor to:

```python
# Input bit flags
INPUT_FORWARD    = 0x01
INPUT_BACKWARD   = 0x02
INPUT_LEFT       = 0x04
INPUT_RIGHT      = 0x08
INPUT_HORN_UP    = 0x10
INPUT_HORN_DOWN  = 0x20
INPUT_HORN_LEFT  = 0x40
INPUT_HORN_RIGHT = 0x80

def get_local_inputs(window, player='blue'):
    """Read keyboard, return 8-bit input state"""
    keys = BLUE_KEYS if player == 'blue' else RED_KEYS
    inputs = 0
    if window.is_pressed(keys['forward']): inputs |= INPUT_FORWARD
    if window.is_pressed(keys['backward']): inputs |= INPUT_BACKWARD
    # ... etc
    return inputs

def apply_inputs(beetle, inputs, dt, physics_params):
    """Apply 8-bit input state to beetle physics"""
    if inputs & INPUT_FORWARD:
        # existing forward movement code
    if inputs & INPUT_LEFT:
        # existing left turn code
    # ... etc
```

### Phase 2: Input Delay Buffer
**File**: `beetle_physics.py`

```python
class InputBuffer:
    def __init__(self, delay_frames=4):
        self.delay = delay_frames
        self.local_inputs = {}   # frame_num -> input_bits
        self.remote_inputs = {}  # frame_num -> input_bits
        self.current_frame = 0

    def add_local(self, inputs):
        self.local_inputs[self.current_frame] = inputs

    def add_remote(self, frame, inputs):
        self.remote_inputs[frame] = inputs

    def get_frame_inputs(self, frame):
        """Get inputs for physics frame (delayed by self.delay)"""
        target = frame - self.delay
        local = self.local_inputs.get(target, 0)
        remote = self.remote_inputs.get(target, 0)
        return local, remote
```

### Phase 3: Steam SDK
**File**: New `network.py`

```python
import py_steam_net as steam  # or steamworks

class NetworkManager:
    def __init__(self):
        steam.init()  # Requires steam_appid.txt with "480" for testing
        self.is_host = False
        self.peer_id = None
        self.lobby_id = None
        self.connected = False

    def host_lobby(self):
        self.is_host = True
        self.lobby_id = steam.create_lobby(2, steam.LOBBY_PUBLIC)

    def join_lobby(self, lobby_id):
        self.is_host = False
        steam.join_lobby(lobby_id, self._on_joined)

    def send_input(self, frame, inputs):
        data = struct.pack('>IB', frame, inputs)  # 5 bytes
        steam.send_reliable(self.peer_id, data)

    def poll_inputs(self, input_buffer):
        while msg := steam.receive():
            frame, inputs = struct.unpack('>IB', msg)
            input_buffer.add_remote(frame, inputs)
```

### Phase 4: Integrate Into Game Loop
**File**: `beetle_physics.py` main loop (~line 6054)

```python
# Modified physics loop
physics_frame = 0
input_buffer = InputBuffer(delay_frames=4)
network = NetworkManager() if online_mode else None

while running:
    # Read local inputs
    local_inputs = get_local_inputs(window)
    input_buffer.add_local(local_inputs)

    # Network: send local, receive remote
    if network:
        network.send_input(physics_frame, local_inputs)
        network.poll_inputs(input_buffer)

    # Physics with delayed inputs
    while accumulator >= PHYSICS_TIMESTEP:
        blue_inputs, red_inputs = input_buffer.get_frame_inputs(physics_frame)

        if network and not network.is_host:
            blue_inputs, red_inputs = red_inputs, blue_inputs  # Swap for guest

        apply_inputs(beetle_blue, blue_inputs, PHYSICS_TIMESTEP, physics_params)
        apply_inputs(beetle_red, red_inputs, PHYSICS_TIMESTEP, physics_params)

        # Rest of physics unchanged
        beetle_collision(beetle_blue, beetle_red, physics_params)

        physics_frame += 1
        accumulator -= PHYSICS_TIMESTEP
```

---

## Key Files Reference

| File | What's There | Network Changes Needed |
|------|--------------|----------------------|
| `beetle_physics.py:261-375` | Beetle class | None - state is internal |
| `beetle_physics.py:382-505` | `update_physics()` | None - called by apply_inputs |
| `beetle_physics.py:6054-6058` | Accumulator loop | Add frame counter |
| `beetle_physics.py:6242-6515` | Input handling | Refactor to abstraction |
| `beetle_physics.py:531-582` | `reset_match()` | Sync initial state |
| `simulation.py` | Voxel grid | None - computed locally |
| `renderer.py` | Rendering | None |
| New: `network.py` | Networking | All networking code |

---

## Bandwidth Math

```
Per frame:   5 bytes (4 frame + 1 input)
Per second:  5 × 60 = 300 bytes
Both ways:   600 bytes/sec = 4.8 kbps

This is NOTHING. Voice chat uses 30-50 kbps.
```

---

## Steam Testing Setup

### Option A: Quick Testing with Spacewar (No Account Needed)

Use Steam's public test App ID `480`:

1. Create `steam_appid.txt` in game folder containing just: `480`
2. Have Steam running on both test machines
3. Both players must be Steam friends (for lobby discovery)
4. Run the game on both machines

**Limitation:** Shares lobby pool with other devs using 480. Filter lobbies by name.

---

### Option B: Private Testing with Steamworks (Recommended)

#### One-Time Setup (Developer)

1. **Create Steamworks account** (free)
   - Go to: https://partner.steamgames.com
   - Sign in with your Steam account
   - Accept the developer agreement

2. **Create your app** (free, no $100 fee needed)
   - Steamworks dashboard → "Create new app"
   - Choose "Game"
   - Fill in basic info (name, etc.)
   - You'll get an **App ID** (e.g., `2847590`)

3. **Add testers**
   - Steamworks → Your App → "Users & Permissions" → "Manage Testers"
   - Add your friends' Steam account names or emails
   - They'll receive access instantly

#### Tester Setup (Your Friends)

**If running from source (no Steam upload):**

Your friends need these files (send via zip, Google Drive, GitHub, etc.):

```
BeetleBattle/
├── beetle_physics.py
├── beetle_battle_main.py
├── simulation.py
├── renderer.py
├── network.py              ← (once implemented)
├── steam_appid.txt         ← Contains YOUR App ID (not 480)
├── requirements.txt
└── ... (other game files)
```

**Requirements for testers:**
1. Python 3.10+ installed
2. Run `pip install -r requirements.txt`
3. Steam client running & logged in
4. Be added as tester in your Steamworks dashboard
5. Run `python beetle_physics.py`

**If uploading build to Steam (more polished):**

1. You upload build via Steamworks SDK (SteamPipe)
2. Friends see "Beetle Battle" in their Steam library
3. They just click "Play" - no Python install needed
4. Requires packaging game as executable (PyInstaller)

---

### Required Files for Testing

#### steam_appid.txt
Create this file in your game folder:
```
YOUR_APP_ID_HERE
```
Replace with `480` for quick tests or your actual Steamworks App ID.

#### requirements.txt
```
taichi
py_steam_net
```
Or if using SteamworksPy:
```
taichi
steamworkspy
```

---

### Testing Checklist

| Step | Developer | Tester |
|------|-----------|--------|
| Steam running | ✓ | ✓ |
| Steamworks tester access | Grant it | Receive it |
| Game files | Have them | Get from dev |
| steam_appid.txt | Create | Included in files |
| Python + deps | Install | Install |
| Run game | Host lobby | Join lobby |

---

### Network Testing on One Machine

For solo testing without a friend:

1. Have two Steam accounts (use Family Sharing for second)
2. Run Steam on main account
3. Run game instance 1 → Host lobby
4. Log into second Steam account (via Steam website or second PC)
5. Run game instance 2 → Join lobby

Or use two computers on same network - easier.

---

### The $100 Fee - When Do You Pay?

| What You Want | Cost |
|---------------|------|
| Private testing with friends | **Free** |
| Steamworks App ID | **Free** |
| Upload builds for testers | **Free** |
| Public store page | **$100** (refundable after $1000 revenue) |

You only pay when ready to release publicly.

---

## Desync Detection (Optional Enhancement)

```python
def compute_checksum(beetle_blue, beetle_red):
    """Quick hash for desync detection - send every 60 frames"""
    data = struct.pack('>ffffff',
        round(beetle_blue.x, 2), round(beetle_blue.z, 2),
        round(beetle_red.x, 2), round(beetle_red.z, 2),
        round(beetle_blue.rotation, 2), round(beetle_red.rotation, 2)
    )
    return zlib.crc32(data)
```

If checksums differ, you have a desync bug to fix.

---

## Common Pitfalls to Avoid

1. **DON'T sync voxel grid** - It's computed from beetle positions
2. **DON'T sync particle effects** - Visual only, regenerate locally
3. **DON'T use random()** in physics - Seed it or results will differ
4. **DON'T skip the input abstraction** - It's the foundation
5. **DO use fixed timestep** - Already have this, don't change it
6. **DO use Steam's reliable messaging** - Handles retransmits for you

---

## Game Events That Need Syncing

Beyond inputs, these discrete events must be synchronized:

### Match Start Synchronization

Both players must start physics on the same frame:

```python
# Host sends START signal with initial frame number
def start_match(network):
    if network.is_host:
        # Wait for guest to be ready
        network.send_event('READY_CHECK')
        # Guest responds with 'READY'
        # Host sends 'START', frame=0, seed=random_seed
        network.send_event('START', frame=0, seed=12345)

    # Both players now start physics_frame = 0
```

### Events to Sync (not inputs)

| Event | Data | When |
|-------|------|------|
| `READY` | none | Player loaded, waiting |
| `START` | frame, rng_seed | Match begins |
| `DEATH` | which_beetle, frame | Beetle dies (falls off) |
| `RESET` | frame | Rematch requested |
| `PAUSE` | frame | Player paused |
| `DISCONNECT` | none | Player left |

### Victory/Death Handling

```python
# When beetle falls off arena (y < -10)
if beetle.y < DEATH_Y_THRESHOLD:
    # Don't process locally - send event
    network.send_event('DEATH', beetle='blue', frame=physics_frame)

# On receiving DEATH event
def on_death_event(beetle_id, frame):
    # Both clients process death on same frame
    trigger_victory(winner='red' if beetle_id == 'blue' else 'blue')
```

---

## Horn Type Selection

Players choose their horn type before match starts:

```python
# Sync horn selection in lobby
HORN_TYPES = ['rhino', 'stag', 'hercules', 'scorpion', 'atlas']

# During lobby phase
def select_horn(horn_type):
    network.send_event('HORN_SELECT', horn=horn_type)
    my_horn = horn_type

def on_horn_select(peer_horn):
    opponent_horn = peer_horn
    # Both players now know horn types before START
```

Include horn types in START event:
```python
network.send_event('START',
    frame=0,
    seed=12345,
    host_horn='rhino',
    guest_horn='stag'
)
```

---

## Lobby UI Flow

### Game States

```
MAIN_MENU
    ├── "Host Game" → HOSTING (waiting for player)
    ├── "Join Game" → BROWSING (lobby list)
    └── "Local Play" → existing single-machine mode

HOSTING
    ├── Show lobby code / "Waiting for player..."
    ├── Player joins → HORN_SELECT
    └── Cancel → MAIN_MENU

BROWSING
    ├── Show available lobbies
    ├── Select lobby → HORN_SELECT
    └── Back → MAIN_MENU

HORN_SELECT
    ├── Both players pick horn type
    ├── Both ready → COUNTDOWN
    └── Player leaves → MAIN_MENU

COUNTDOWN
    └── 3... 2... 1... FIGHT! → PLAYING

PLAYING
    ├── Normal gameplay (physics loop)
    ├── Beetle dies → VICTORY
    └── Disconnect → DISCONNECTED

VICTORY
    ├── "BLUE WINS!" / "RED WINS!"
    ├── Rematch → COUNTDOWN
    └── Leave → MAIN_MENU

DISCONNECTED
    └── "Opponent disconnected" → MAIN_MENU
```

### Simple Lobby UI (Taichi GUI)

```python
# During HOSTING state
gui.text(f"Lobby Code: {lobby_id}", pos=(0.5, 0.6))
gui.text("Waiting for opponent...", pos=(0.5, 0.5))
if gui.button("Cancel", pos=(0.5, 0.3)):
    network.leave_lobby()
    state = MAIN_MENU
```

---

## Disconnect Handling

### Detection

```python
# Steam provides disconnect callbacks
def on_peer_disconnect(peer_id):
    if game_state == PLAYING:
        # Mid-match disconnect
        show_message("Opponent disconnected")
        # Option: count as win, or just return to menu
    game_state = MAIN_MENU
```

### Timeout for Missing Inputs

```python
TIMEOUT_FRAMES = 180  # 3 seconds at 60fps

def check_input_timeout(input_buffer, physics_frame):
    # If we haven't received remote input for too long
    last_remote = max(input_buffer.remote_inputs.keys(), default=0)
    if physics_frame - last_remote > TIMEOUT_FRAMES:
        # Connection probably dead
        handle_disconnect()
```

### Graceful Pause

If remote inputs are late but not timed out, pause physics:

```python
while accumulator >= PHYSICS_TIMESTEP:
    target_frame = physics_frame - input_buffer.delay

    if target_frame not in input_buffer.remote_inputs:
        # Missing input - wait (don't advance physics)
        break  # Exit physics loop, keep accumulator

    # Have inputs, proceed normally
    ...
```

---

## Troubleshooting Guide

### Common Issues

| Problem | Cause | Fix |
|---------|-------|-----|
| "Steam not initialized" | Steam client not running | Start Steam first |
| "Lobby not found" | Wrong App ID or not friends | Check steam_appid.txt, add as Steam friends |
| Can't see friend's lobby | NAT issues | Steam Relay should handle, but try restarting Steam |
| Beetles move differently | Desync | Check for random() calls, verify same game version |
| Input feels laggy | 4-frame delay (~67ms) | Normal - can reduce to 3 frames for LAN |
| Game freezes | Waiting for remote input | Check opponent connection, add timeout |
| "py_steam_net not found" | Missing dependency | `pip install py_steam_net` |

### Debug Mode

Add debug overlay for network testing:

```python
DEBUG_NETWORK = True

if DEBUG_NETWORK:
    gui.text(f"Frame: {physics_frame}", pos=(0.02, 0.98))
    gui.text(f"Ping: {network.ping_ms}ms", pos=(0.02, 0.95))
    gui.text(f"Remote buffer: {len(input_buffer.remote_inputs)}", pos=(0.02, 0.92))
    gui.text(f"Local buffer: {len(input_buffer.local_inputs)}", pos=(0.02, 0.89))
```

### Logging for Desync Debugging

```python
# Log inputs for replay/debugging
if DEBUG_NETWORK:
    with open('netlog.txt', 'a') as f:
        f.write(f"{physics_frame},{local_inputs},{remote_inputs}\n")
```

Compare logs between players to find where desync started.

---

## Future Upgrade Path: Rollback Netcode

If delay-based feels too sluggish for competitive play, upgrade to rollback:

### What Changes

| Delay-Based (Current Plan) | Rollback (Future) |
|---------------------------|-------------------|
| 4-frame input delay | 0-frame delay (instant response) |
| Wait for remote inputs | Predict remote inputs |
| Simple implementation | Complex state management |
| ~67ms added latency | Feels like offline |

### Rollback Requirements

1. **Save/load game state** - Must snapshot and restore beetle positions
2. **Re-simulate frames** - When prediction wrong, rewind and replay
3. **State serialization** - Quick save/load of all beetle state
4. **GGPO library** - Consider using existing rollback library

### Estimated Additional Work

- Rollback adds ~500-1000 lines of code
- Requires careful state management
- Test thoroughly - rollback bugs are subtle

### Recommendation

Start with delay-based. If players complain about lag feel, then invest in rollback. Many successful indie fighting games ship with delay-based netcode.

---

## Summary

This is a well-structured game for networking. The hard parts (deterministic physics, fixed timestep, small state) are already solved. The implementation is mostly plumbing:

1. Extract input reading into functions
2. Add delay buffer
3. Hook up Steam SDK
4. Send/receive 5 bytes per frame
5. Handle game events (start, death, rematch)
6. Add lobby UI flow

Total new code: ~300-500 lines across 2-3 files.
