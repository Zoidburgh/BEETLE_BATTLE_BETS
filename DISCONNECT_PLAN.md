# Disconnect & Reconnection System Plan

## Overview

Simple, trust-based disconnect handling for casual multiplayer between friends. No pause screen - game keeps running, opponent can rejoin anytime.

---

## Design Philosophy

- **Trust-based:** Assume players are friends who won't abuse disconnects
- **Non-blocking:** Remaining player can keep playing
- **Simple:** Minimal UI, minimal complexity
- **Seamless:** Reconnecting player snaps to current state and continues

---

## How It Works

### When Opponent Disconnects

1. Detect no inputs for 3 seconds (180 frames)
2. Show small banner: "Opponent disconnected - waiting for reconnect..."
3. **Game keeps running** - physics, movement, everything
4. Opponent's beetle freezes in place (no inputs = no movement)
5. Their beetle is still affected by physics (can be pushed, can fall)

### When Opponent Reconnects

1. Opponent's game sends MSG_RECONNECT_REQUEST
2. Host sends MSG_RECONNECT_STATE (full game snapshot)
3. Opponent applies snapshot, snaps to current state
4. Show banner: "Opponent reconnected!"
5. Game continues seamlessly

### If Opponent Never Returns

- After 60 seconds, show option to exit
- Or player can keep playing solo indefinitely
- No automatic timeout/kick

---

## New Message Types

```python
MSG_RECONNECT_REQUEST = 0x0E   # "I'm back, send me current state"
MSG_RECONNECT_STATE = 0x0F    # Full game snapshot (host → guest)
MSG_DISCONNECT = 0x10         # "I'm leaving intentionally" (optional)
```

---

## State Snapshot (MSG_RECONNECT_STATE)

```python
# ~70 bytes total
struct.pack('>B I fffffffff fffffffff ffffff BBBB ff B',
    MSG_RECONNECT_STATE,        # 1 byte - message type
    physics_frame,              # 4 bytes - current frame

    # Blue beetle (36 bytes)
    blue_x, blue_y, blue_z,
    blue_vx, blue_vy, blue_vz,
    blue_rotation, blue_pitch, blue_roll,

    # Red beetle (36 bytes)
    red_x, red_y, red_z,
    red_vx, red_vy, red_vz,
    red_rotation, red_pitch, red_roll,

    # Ball (24 bytes)
    ball_x, ball_y, ball_z,
    ball_vx, ball_vy, ball_vz,

    # Scores & flags (4 bytes)
    blue_score, red_score, ball_active, referee_active,

    # Timers (8 bytes)
    blue_respawn_timer, red_respawn_timer,

    # Extra (1 byte)
    input_delay  # So guest uses same delay
)
```

---

## UI Elements

### Disconnect Banner (non-blocking)

```
+------------------------------------------+
|  ⚠ Opponent disconnected - waiting...   |
+------------------------------------------+
```
- Small banner at top of screen
- Semi-transparent, doesn't block gameplay
- Disappears when opponent reconnects

### Reconnect Banner (temporary)

```
+------------------------------------------+
|  ✓ Opponent reconnected!                 |
+------------------------------------------+
```
- Shows for 2 seconds then fades

### Exit Option (after 60 sec)

```
+------------------------------------------+
|  ⚠ Opponent disconnected (60s)          |
|  [Continue Waiting]  [Exit to Menu]      |
+------------------------------------------+
```
- Only shows after extended disconnect
- Player can choose to keep waiting or leave

---

## Implementation Checklist

### Phase 1: Detection & UI
- [ ] Track `frames_since_remote_input` counter
- [ ] Detect disconnect (counter > 180)
- [ ] Set `opponent_disconnected = True` flag
- [ ] Draw disconnect banner (non-blocking)
- [ ] Keep physics running normally

### Phase 2: Reconnection
- [ ] Add MSG_RECONNECT_REQUEST to network.py
- [ ] Add MSG_RECONNECT_STATE to network.py
- [ ] Guest sends REQUEST when reconnecting
- [ ] Host builds and sends full state snapshot
- [ ] Guest applies snapshot and resets input buffer
- [ ] Clear `opponent_disconnected` flag
- [ ] Show "Reconnected!" banner for 2 seconds

### Phase 3: Polish
- [ ] Add exit option after 60 seconds
- [ ] Add MSG_DISCONNECT for graceful exits
- [ ] Handle edge cases (reconnect during death, etc.)

---

## Code Locations

### beetle_physics.py

```python
# New globals
opponent_disconnected = False
disconnect_timer = 0.0
reconnect_banner_timer = 0.0

# In main loop - detection
if input_buffer.frames_waited > 180 and not opponent_disconnected:
    opponent_disconnected = True
    disconnect_timer = 0.0
    print("[Network] Opponent disconnected")

# In main loop - reconnect detection
if opponent_disconnected and input_buffer.frames_waited == 0:
    opponent_disconnected = False
    reconnect_banner_timer = 2.0  # Show banner for 2 sec
    print("[Network] Opponent reconnected")

# In render - draw banner
if opponent_disconnected:
    draw_disconnect_banner(disconnect_timer)
if reconnect_banner_timer > 0:
    draw_reconnect_banner()
    reconnect_banner_timer -= dt
```

### network.py

```python
MSG_RECONNECT_REQUEST = 0x0E
MSG_RECONNECT_STATE = 0x0F
MSG_DISCONNECT = 0x10

def send_reconnect_request(self):
    """Guest calls this when reconnecting"""
    data = struct.pack('>B', MSG_RECONNECT_REQUEST)
    self._send_packet(data, reliable=True)

def send_reconnect_state(self, game_state_dict):
    """Host sends full game snapshot"""
    data = struct.pack('>B I fff fff fff fff fff fff fff fff BBBB ff',
        MSG_RECONNECT_STATE,
        game_state_dict['frame'],
        # ... all the state fields
    )
    self._send_packet(data, reliable=True)
```

---

## Edge Cases

| Scenario | Behavior |
|----------|----------|
| Opponent pushed off while disconnected | They respawn, score counts normally |
| Ball scores while opponent disconnected | Goal counts, game continues |
| Opponent disconnects mid-death | Death completes, they respawn |
| Host disconnects | Guest waits, host reconnects and sends state |
| Both disconnect | Both wait, first to reconnect becomes authority |
| Disconnect during countdown | Not possible (countdown is pre-game) |

---

## What We're NOT Doing

- ❌ Pausing the game
- ❌ Freezing physics
- ❌ Blocking remaining player
- ❌ Automatic forfeits
- ❌ Ranked/competitive protections
- ❌ Anti-cheat for disconnect abuse

This is a casual friends game - we trust players not to abuse it.

---

## Testing

- [ ] Disconnect by closing opponent's game
- [ ] Disconnect by disabling network
- [ ] Reconnect after 5 seconds
- [ ] Reconnect after 30 seconds
- [ ] Push opponent's beetle while they're disconnected
- [ ] Score goal while opponent disconnected
- [ ] Opponent beetle falls off while disconnected
- [ ] Exit to menu after 60 seconds
- [ ] Both players disconnect and reconnect

---

## Version History

| Date | Changes |
|------|---------|
| 2024-01-08 | Initial draft - complex pause-based system |
| 2024-01-08 | Simplified to trust-based keep-playing system |
