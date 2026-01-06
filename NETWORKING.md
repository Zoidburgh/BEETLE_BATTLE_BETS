# Beetle Battle Networking Reference

## Overview

The game uses **lockstep networking** - both players must have inputs for a frame before it simulates. This ensures identical gameplay on both machines but adds input delay.

```
[Player presses key] --> [Input sent over network] --> [Both inputs received] --> [Physics simulates]
```

---

## Key Variables

### 1. Input Delay (`self.delay`)

**Location:** `beetle_physics.py` line 452, set by `set_network_delay()` at line 588

**What it does:** Delays your inputs by N frames before applying them. This gives time for opponent's inputs to arrive.

**Current behavior:**
```python
def set_network_delay(self, ping_ms):
    one_way_ms = ping_ms / 2.0
    delay_frames = int(one_way_ms / 16.67) + 1
    self.delay = max(2, min(8, delay_frames))  # Clamped 2-8 frames
```

**How it affects feel:**
| Delay | Latency Added | Feel |
|-------|---------------|------|
| 2 frames | ~33ms | Responsive, may stutter on bad connection |
| 4 frames | ~67ms | Balanced |
| 6 frames | ~100ms | Smooth but sluggish |

**To tune:** Change the `max(2, ...)` minimum or `min(..., 8)` maximum in `set_network_delay()`

---

### 2. Guest State Lerp Factor

**Location:** `beetle_physics.py` line 11397

**What it does:** When guest receives host's authoritative positions, lerp toward them by this factor.

**Current value:** `lerp_factor = 0.5`

```python
beetle_blue.x += (sync['blue_x'] - beetle_blue.x) * lerp_factor
```

**How it affects feel:**
| Value | Effect |
|-------|--------|
| 0.25 | Floaty, slow corrections, smooth |
| 0.50 | Balanced (current) |
| 0.75 | Snappy corrections, may jitter |
| 1.00 | Instant snap to host position |

**Trade-off:** Higher = more responsive but can cause visible "rubber-banding" if positions diverge

---

### 3. State Sync Interval

**Location:** `beetle_physics.py` line 11382

**What it does:** How often host sends authoritative positions to guest.

**Current value:** Every 10 physics frames (~167ms)

```python
if network_manager.is_host and physics_frame % 10 == 0:
    network_manager.send_state_sync(...)
```

**How it affects feel:**
| Interval | Effect |
|----------|--------|
| 5 frames | More accurate, more bandwidth |
| 10 frames | Balanced (current) |
| 30 frames | Less bandwidth, more drift |

---

### 4. Frame Sync Interval

**Location:** `beetle_physics.py` line 11378

**What it does:** How often host tells guest what frame they should be on.

**Current value:** Every 30 physics frames (~500ms)

```python
if network_manager.is_host and physics_frame % 30 == 0:
    network_manager.send_frame_sync(input_buffer.current_frame)
```

---

### 5. Frame Sync Tolerance

**Location:** `beetle_physics.py` lines 11489-11498

**What it does:** How many frames guest can drift before adjusting.

**Current value:** 2 frames tolerance

```python
if frame_diff < -2:    # Guest is AHEAD by 3+ frames
    # Skip physics step to slow down
elif frame_diff > 2:   # Guest is BEHIND by 3+ frames
    # Run extra physics step to catch up
```

**How it affects feel:**
| Tolerance | Effect |
|-----------|--------|
| 1 frame | Tight sync, more adjustments |
| 2 frames | Balanced (current) |
| 4 frames | Loose sync, fewer adjustments |

---

### 6. Input Prediction

**Location:** `beetle_physics.py` lines 513-522

**What it does:** If we're missing opponent's input for frame N but have frame N+1, use that instead.

```python
if not has_remote and self.remote_frame_received >= sim_frame:
    # Use newer input for this frame
    self.remote_inputs[sim_frame] = self.remote_inputs[f]
```

**Effect:** Reduces waiting when opponent is ahead, but can cause slightly wrong inputs to be used.

---

### 7. NETWORK_INPUT_DELAY (UNUSED!)

**Location:** `beetle_physics.py` line 449

**Status:** DEAD CODE - defined but never referenced

```python
NETWORK_INPUT_DELAY = 3  # This does NOTHING!
```

---

## Data Flow Diagram

```
HOST MACHINE                           GUEST MACHINE
============                           =============

1. Read keyboard input                 1. Read keyboard input
      |                                      |
      v                                      v
2. Store in local_inputs[frame]        2. Store in local_inputs[frame]
      |                                      |
      v                                      v
3. Send input packet ----[network]---> 3. Receive in remote_inputs[frame]
      |                                      |
      v                                      v
3. Receive in remote_inputs[frame] <--[network]---- 3. Send input packet
      |                                      |
      v                                      v
4. can_simulate()?                     4. can_simulate()?
   - Need local_inputs[frame] YES         - Need local_inputs[frame] YES
   - Need remote_inputs[frame] ???        - Need remote_inputs[frame] ???
      |                                      |
      v                                      v
5. If both YES: run physics            5. If both YES: run physics
   If NO: WAIT                            If NO: WAIT
      |                                      |
      v                                      v
6. Both beetles move identically       6. Both beetles move identically
      |                                      |
      v                                      v
7. Host sends state_sync ----[network]---> 7. Guest lerps toward host state
```

---

## Asymmetry Issues

### Why Guest Might Feel Different

1. **State Sync is One-Way:** Host -> Guest only. Guest lerps BOTH beetles toward host's positions.

2. **Frame Sync Adjustment:** Guest adjusts their frame counter to match host. Host never adjusts.

3. **Input Prediction Asymmetry:** If guest runs faster (higher FPS), they send inputs early. Host can use "future" guest inputs. Guest can't use "future" host inputs.

### Debug Output

Enable debug logging (already added) to see:
```
[HOST] Frame:1200 RemoteFrame:1202 Diff:+2 | Waits:5 (4.0%) Predicts:15 (12.5%)
[GUEST] Frame:1202 RemoteFrame:1198 Diff:-4 | Waits:25 (17.0%) Predicts:0 (0.0%)
```

- **Diff positive** = opponent is ahead of you
- **Diff negative** = opponent is behind you
- **High Waits%** = you're waiting for opponent often (feels laggy)
- **High Predicts%** = you're using opponent's future inputs (feels smooth)

---

## Tuning Recommendations

### For More Responsive Feel (riskier)

```python
# Lower minimum delay
self.delay = max(1, min(6, delay_frames))  # Was max(2, ...)

# Higher lerp factor
lerp_factor = 0.7  # Was 0.5

# More frequent state sync
if physics_frame % 5 == 0:  # Was % 10
```

### For More Stable Feel (safer)

```python
# Higher minimum delay
self.delay = max(3, min(8, delay_frames))  # Was max(2, ...)

# Lower lerp factor
lerp_factor = 0.3  # Was 0.5

# Less frequent state sync
if physics_frame % 15 == 0:  # Was % 10
```

### For Perfect Fairness

Add equal fixed delay to BOTH players regardless of who's host:

```python
# In set_network_delay():
self.delay = 3  # Fixed 3 frames for everyone, ignore ping
```

---

## Quick Reference

| Variable | Location | Current | Effect |
|----------|----------|---------|--------|
| `self.delay` | set_network_delay() | 2-8 (ping-based) | Input lag |
| `lerp_factor` | line 11397 | 0.5 | Guest correction speed |
| State sync interval | line 11382 | 10 frames | Position accuracy |
| Frame sync interval | line 11378 | 30 frames | Frame alignment |
| Frame sync tolerance | line 11489 | 2 frames | Drift allowed |
| `NETWORK_INPUT_DELAY` | line 449 | 3 | **UNUSED** |

---

## Files

- `beetle_physics.py` - All gameplay networking logic
- `network.py` - Steam networking, packet sending/receiving
