# Beetle Battle Steam Networking Plan

> **AI CONTEXT DOCUMENT**: Read this first when working on multiplayer. Everything runs through Steam - no workarounds, no IP sharing.

---

## Game Modes

| Mode | Players | Beetles | Description |
|------|---------|---------|-------------|
| **1v1** | 2 | 2 | Classic duel |
| **1v1v1** | 3 | 3 | Free-for-all |
| **2v2** | 4 | 4 | Team battle |

All modes use the same networking code - just different player counts.

---

## Current Status

| Component | Status | Notes |
|-----------|--------|-------|
| py_steam_net library | **DONE** | Python 3.12 required |
| Steam lobby create/join | **DONE** | No IP addresses needed |
| Steam P2P messaging | **DONE** | NAT traversal automatic |
| Input abstraction (8-bit) | **DONE** | In beetle_physics.py |
| Input buffer (frame sync) | **DONE** | In beetle_physics.py |
| Game integration | **TODO** | Hook network into game loop |
| Multi-player (3-4) | **TODO** | Expand for 1v1v1 and 2v2 |
| Matchmaking UI | **TODO** | Lobby browser, invites |

---

## Architecture: Host-Based P2P

All game modes use the same pattern - one player is host, others are guests.

```
1v1 (2 players):
    Host ←——————→ Guest

1v1v1 (3 players):
         Guest 1
            ↕
    Host ←——————→ Guest 2

2v2 (4 players):
         Guest 1
            ↕
    Host ←——————→ Guest 2
            ↕
         Guest 3
```

**How it works:**
1. Host creates Steam lobby (public/friends/private)
2. Guests join lobby via ID or friend invite
3. All players select beetles/horns
4. Host starts match when everyone ready
5. Each frame: everyone sends inputs to host, host broadcasts all inputs to everyone
6. Deterministic physics = everyone stays in sync

**Why this works:**
- Input-based sync: Only 8 bits per player per frame
- Deterministic physics: Fixed 60Hz timestep, same result everywhere
- Steam P2P: NAT traversal handled automatically
- Tiny bandwidth: ~360 bytes/sec per player

---

## Network Protocol

### Message Types

| Type | ID | Format | Description |
|------|-----|--------|-------------|
| MSG_INPUT | 0x01 | `[type:1][frame:4][player_id:1][inputs:1]` | Player input for frame |
| MSG_INPUTS_ALL | 0x02 | `[type:1][frame:4][count:1][inputs:N]` | Host broadcasts all inputs |
| MSG_READY | 0x03 | `[type:1][player_id:1]` | Player ready to start |
| MSG_START | 0x04 | `[type:1][frame:4][seed:4][mode:1]` | Match start signal |
| MSG_BEETLE_SELECT | 0x05 | `[type:1][player_id:1][beetle:1][horn:1]` | Beetle/horn choice |
| MSG_PING | 0x06 | `[type:1][timestamp:4]` | Latency measurement |
| MSG_PONG | 0x07 | `[type:1][timestamp:4]` | Ping response |

### Player Assignment

| Mode | Host | Guest 1 | Guest 2 | Guest 3 |
|------|------|---------|---------|---------|
| 1v1 | Player 0 (Blue) | Player 1 (Red) | - | - |
| 1v1v1 | Player 0 | Player 1 | Player 2 | - |
| 2v2 | Team A, P0 | Team A, P1 | Team B, P0 | Team B, P1 |

---

## Implementation Plan

### Phase 1: 2-Player Online (1v1) - NEXT

Get basic online 1v1 working through Steam.

**1.1 Menu Integration**
```python
# Game states
LOBBY_HOST = "lobby_host"      # Created lobby, waiting for opponent
LOBBY_JOIN = "lobby_join"      # Entering lobby ID
LOBBY_WAITING = "lobby_wait"   # In lobby, waiting for ready

# Menu flow
"Host Game" → create_lobby() → show lobby ID → wait for guest
"Join Game" → enter lobby ID → join_lobby() → wait for host
```

**1.2 Lobby Screen**
- Show lobby ID (for sharing with friends)
- Show connected players and their Steam names
- Beetle/horn selection for each player
- "Ready" button
- Match starts when all players ready

**1.3 Game Loop Integration**
```python
# In main game loop
if online_mode:
    # Send our inputs
    network.send_input(frame, local_inputs)

    # Receive inputs from network
    network.poll_messages(input_buffer)

    # Wait for all inputs before running physics
    if input_buffer.has_all_inputs(frame):
        run_physics_frame(input_buffer.get_all_inputs(frame))
```

**1.4 Input Buffer Upgrade**
```python
class InputBuffer:
    def __init__(self, num_players, delay_frames=4):
        self.inputs = {pid: {} for pid in range(num_players)}
        self.delay = delay_frames

    def add_input(self, player_id, frame, inputs):
        self.inputs[player_id][frame] = inputs

    def has_all_inputs(self, frame):
        target = frame - self.delay
        return all(target in self.inputs[pid] for pid in self.inputs)

    def get_all_inputs(self, frame):
        target = frame - self.delay
        return [self.inputs[pid].get(target, 0) for pid in sorted(self.inputs)]
```

**1.5 Match Flow**
```
LOBBY_HOST/JOIN
    ↓ (opponent joins)
BEETLE_SELECT
    ↓ (both ready)
COUNTDOWN (3-2-1)
    ↓
PLAYING
    ↓ (beetle dies)
VICTORY
    ↓ (rematch or leave)
LOBBY or MENU
```

---

### Phase 2: Multi-Player (1v1v1 and 2v2)

Expand to support 3-4 players. Same architecture, just more players.

**2.1 Lobby Changes**
```python
# When creating lobby
network.create_lobby("public", max_players=4)  # For 2v2
network.create_lobby("public", max_players=3)  # For 1v1v1

# Lobby tracks all connected players
network.players = {
    steam_id: {
        "player_id": 0,  # 0-3
        "name": "PlayerName",
        "beetle": "rhino",
        "horn": "rhino",
        "ready": False,
        "team": 0  # For 2v2: 0 or 1
    }
}
```

**2.2 Host Broadcasts All Inputs**
```python
# Host collects inputs from all guests
def on_input_received(player_id, frame, inputs):
    input_buffer.add_input(player_id, frame, inputs)

    # When we have all inputs for this frame, broadcast to everyone
    if input_buffer.has_all_inputs(frame):
        all_inputs = input_buffer.get_all_inputs(frame)
        broadcast_all_inputs(frame, all_inputs)
```

**2.3 Team Assignment (2v2)**
```python
# Teams assigned by join order or player choice
TEAMS = {
    0: [0, 1],  # Team A: players 0 and 1
    1: [2, 3],  # Team B: players 2 and 3
}

# Win condition: all beetles on one team dead
def check_victory():
    team_alive = {0: False, 1: False}
    for pid, beetle in enumerate(beetles):
        if beetle.health > 0:
            team = 0 if pid in TEAMS[0] else 1
            team_alive[team] = True

    if not team_alive[0]: return 1  # Team B wins
    if not team_alive[1]: return 0  # Team A wins
    return None  # No winner yet
```

**2.4 Arena Spawns**
```python
# Spawn positions by player count
SPAWNS = {
    2: [(x1, y1), (x2, y2)],                          # 1v1: opposite sides
    3: [(x1, y1), (x2, y2), (x3, y3)],                # 1v1v1: triangle
    4: [(x1, y1), (x2, y2), (x3, y3), (x4, y4)],      # 2v2: corners
}
```

---

### Phase 3: Matchmaking UI

Make finding games easy.

**3.1 Lobby Browser**
```python
# List public lobbies
lobbies = network.get_public_lobbies()
for lobby in lobbies:
    print(f"{lobby.host_name}'s game - {lobby.player_count}/{lobby.max_players}")
```

**3.2 Steam Friend Invites**
```python
# Invite Steam friend to lobby
network.invite_friend(friend_steam_id)

# Handle incoming invite
def on_invite_received(lobby_id, inviter_name):
    show_invite_popup(f"{inviter_name} invited you to play!")
```

**3.3 Quick Match**
```python
# Auto-join first available lobby or create new one
def quick_match(mode="1v1"):
    lobbies = network.get_public_lobbies(mode=mode)
    available = [l for l in lobbies if not l.is_full]

    if available:
        network.join_lobby(available[0].id)
    else:
        network.create_lobby("public", max_players_for_mode(mode))
```

---

## Files

| File | Purpose |
|------|---------|
| `network.py` | NetworkManager - Steam lobbies, P2P messaging |
| `beetle_physics.py` | InputBuffer, input flags, get_local_inputs() |
| `steam_appid.txt` | Steam App ID (480 for testing) |
| `steam_api64.dll` | Valve's Steam API (specific version for py_steam_net) |

**Requirements:**
- Python 3.12 (not 3.13+)
- py_steam_net (from GitHub releases)
- Steam running

---

## Testing

### Test Lobby Creation
```bash
py -3.12 network.py
# Creates lobby, prints ID
```

### Test Join
```bash
py -3.12 network.py <lobby_id>
# Joins existing lobby
```

### Test with Friend
1. You: Run game, click "Host Game"
2. Share the lobby ID with friend (Discord, etc.)
3. Friend: Run game, click "Join Game", enter lobby ID
4. Both select beetles, click Ready
5. Play!

---

## Bandwidth

```
1v1 (2 players):
  Input packet: 7 bytes
  Per second: 420 bytes each way
  Total: ~1 KB/s

1v1v1 (3 players):
  Each player sends: 420 bytes/sec
  Host broadcasts: 630 bytes/sec to each guest
  Total: ~2 KB/s

2v2 (4 players):
  Each player sends: 420 bytes/sec
  Host broadcasts: 840 bytes/sec to each guest
  Total: ~4 KB/s

For comparison: Voice chat uses 30+ KB/s
```

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| py_steam_net import fails | Use Python 3.12, not 3.13+ |
| DLL load failed | Use correct steam_api64.dll (300KB version) |
| Steam init failed | Make sure Steam is running |
| Lobby not created | Check callback signature (1 arg, not 2) |
| Beetles desync | Check for random() calls in physics - must use seeded RNG |
| High latency | Normal for internet; consider adding input delay |

---

## Summary

**What's Done:**
- Steam P2P networking via py_steam_net
- Lobby create/join (no IP addresses!)
- Input abstraction and buffering

**What's Next:**
1. Hook network into game loop (1v1 first)
2. Add lobby UI and beetle selection
3. Expand to 3-4 players (1v1v1, 2v2)
4. Add lobby browser and friend invites

The code is designed so all modes (1v1, 1v1v1, 2v2) use the same networking - just different player counts.
