# Beetle Battle Steam Networking Plan

> **AI CONTEXT DOCUMENT**: Read this first when working on multiplayer. Everything runs through Steam - no workarounds, no IP sharing.

---

## Steam App ID

| App ID | Description |
|--------|-------------|
| **3998620** | Beetle Battle (our registered app) |

**Note:** Using our real App ID. Friends need developer access or beta key from Steamworks to test.

---

## Game Modes

| Mode | Players | Beetles | Description |
|------|---------|---------|-------------|
| **1v1** | 2 | 2 | Classic duel |
| **1v1v1** | 3 | 3 | Free-for-all |
| **2v2** | 4 | 4 | Team battle |

All modes use the same networking code - just different player counts.

---

## py_steam_net API Reality Check

**What py_steam_net provides:**
- `create_lobby()` / `join_lobby()` / `leave_lobby()` ✓
- `get_lobby_members()` ✓
- `send_message_to()` / `receive_messages()` ✓
- Callbacks for lobby changes, messages, connection failures ✓

**What py_steam_net does NOT provide:**
- ❌ `get_public_lobbies()` - No lobby browser
- ❌ `invite_friend()` - No in-game friend invites
- ❌ `get_friends_list()` - No friend list access

**Steam features that work anyway (no code needed):**
- ✓ Steam Overlay invites (Shift+Tab → right-click friend → Invite)
- ✓ `steam://joinlobby/3998620/LOBBY_ID` clickable links
- ✓ NAT traversal handled automatically

---

## Current Status

| Component | Status | Notes |
|-----------|--------|-------|
| py_steam_net library | **DONE** | Python 3.12 required |
| Steam lobby create/join | **DONE** | No IP addresses needed |
| Steam P2P messaging | **TESTING** | Bidirectional handshake fix applied |
| Input abstraction (8-bit) | **DONE** | In beetle_physics.py |
| Input buffer (frame sync) | **DONE** | In beetle_physics.py |
| Game state integration | **DONE** | Host/Join/Waiting/Playing states |
| P2P bidirectional fix | **DONE** | Guest sends MSG_READY on join |
| Basic 1v1 online | **TESTING** | Need to verify P2P fix works |
| Multi-player (3-4) | **TODO** | Expand for 1v1v1 and 2v2 |
| Steam Overlay invites | **READY** | Works automatically! |
| Copy invite link button | **TODO** | Easy addition |

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

### Phase 1: 2-Player Online (1v1) - CURRENT

Get basic online 1v1 working through Steam. **Almost complete!**

| Step | Status | Notes |
|------|--------|-------|
| 1.1 Menu Integration | ✅ DONE | Host/Join buttons, game states |
| 1.2 Lobby Screen | ✅ DONE | Shows lobby ID, copy/paste buttons |
| 1.3 Game Loop Integration | ✅ DONE | Network polling in main loop |
| 1.4 Input Buffer | ✅ DONE | Frame-synced with delay |
| 1.5 P2P Messaging | 🔄 TESTING | Bidirectional fix applied |
| 1.6 Match Start Flow | 🔄 TESTING | Host starts, guest receives |

**What's left:** Verify P2P fix works with two computers, then gameplay sync.

---

### Phase 2: Polish & Friend Invites

Make it easy for friends to join.

| Feature | Difficulty | How |
|---------|------------|-----|
| Steam Overlay invites | ✅ FREE | Already works! Shift+Tab → Invite |
| Copy invite link button | Easy | `steam://joinlobby/3998620/{lobby_id}` |
| Handle join-from-invite | Medium | Detect launch args from Steam |
| Rematch button | Easy | Reset match, stay in lobby |

---

### Phase 3: Multi-Player (1v1v1 and 2v2)

Same architecture, just more players.

**3.1 Lobby Changes**
```python
network.create_lobby("public", max_players=4)  # For 2v2
network.create_lobby("public", max_players=3)  # For 1v1v1
```

**3.2 Host Broadcasts All Inputs**
```python
# Host collects inputs from all guests, broadcasts to everyone
def on_input_received(player_id, frame, inputs):
    input_buffer.add_input(player_id, frame, inputs)
    if input_buffer.has_all_inputs(frame):
        broadcast_all_inputs(frame, all_inputs)
```

**3.3 Team Assignment (2v2)**
```python
TEAMS = {0: [0, 1], 1: [2, 3]}  # Team A and Team B
```

**3.4 Arena Spawns**
```python
SPAWNS = {
    2: [(x1, y1), (x2, y2)],                    # 1v1: opposite sides
    3: [(x1, y1), (x2, y2), (x3, y3)],          # 1v1v1: triangle
    4: [(x1, y1), (x2, y2), (x3, y3), (x4, y4)] # 2v2: corners
}
```

---

### Phase 4: Advanced Matchmaking (OPTIONAL)

These features require either:
- Contributing lobby browser/friend list to py_steam_net
- Switching to a more complete Steam library
- Using external matchmaking (Discord, etc.)

| Feature | Blocked By |
|---------|------------|
| Lobby browser | py_steam_net missing `get_public_lobbies()` |
| In-game friend list | py_steam_net missing `get_friends_list()` |
| In-game friend invite | py_steam_net missing `invite_friend()` |

**Workaround:** Steam Overlay handles all of this! Not critical for release.

---

## Files

| File | Purpose |
|------|---------|
| `network.py` | NetworkManager - Steam lobbies, P2P messaging |
| `beetle_physics.py` | InputBuffer, input flags, get_local_inputs() |
| `steam_appid.txt` | Steam App ID (3998620) |
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
- ✅ Steam P2P networking via py_steam_net
- ✅ Lobby create/join (no IP addresses!)
- ✅ Input abstraction and buffering
- ✅ Game state system (menu → lobby → playing)
- ✅ Host/Join UI with copy/paste lobby ID
- ✅ Bidirectional P2P handshake (guest sends MSG_READY)

**Currently Testing:**
- 🔄 P2P message reception (MSG_START from host to guest)
- 🔄 Match start flow

**What's Next:**
1. ✅ Verify P2P fix with two computers
2. Complete gameplay sync (inputs flowing both ways)
3. Add invite link button (`steam://joinlobby/3998620/{id}`)
4. Expand to 3-4 players (1v1v1, 2v2)

**Friend Invites:** Steam Overlay (Shift+Tab) works! Add friend as developer in Steamworks or send beta key.

The code is designed so all modes (1v1, 1v1v1, 2v2) use the same networking - just different player counts.
