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

## Testing Locally

1. Create `steam_appid.txt` in game folder containing just: `480`
2. Run two instances of the game
3. One hosts, one joins via lobby browser
4. Both Steam accounts must be logged in (can use Steam Family Sharing)

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

## Summary

This is a well-structured game for networking. The hard parts (deterministic physics, fixed timestep, small state) are already solved. The implementation is mostly plumbing:

1. Extract input reading into functions
2. Add delay buffer
3. Hook up Steam SDK
4. Send/receive 5 bytes per frame

Total new code: ~200-300 lines across 2 files.
