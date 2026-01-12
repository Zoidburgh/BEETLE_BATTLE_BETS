# GPU/CPU Optimization Plan for Beetle Battle

## The Problem

**Symptoms:**
- CPU: 1%, GPU: 27%, but FPS is low (20-30)
- Hardware is idle but game is slow
- The bottleneck is CPU↔GPU data transfer, not compute

**Root Cause:**
- Physics runs on CPU (fast: 5ms)
- Rendering uses Vulkan GPU
- Every frame: 5000+ particles copied CPU→GPU (slow: 12ms)
- Both CPU and GPU spend most time WAITING for transfer

**Goal:**
- Run physics on GPU (Vulkan backend)
- Rendering already on GPU → no transfer needed
- Target: 60+ FPS on any GPU from last 5 years

---

## Why GPU Physics is Currently Slow

When we tried Vulkan backend, physics jumped from 5ms to 50ms because of these code patterns:

| Issue | Impact | Current Time |
|-------|--------|--------------|
| 3 serialized cleanup loops | CRITICAL | ~20ms |
| O(N²) silk collision | HIGH | ~15ms |
| Atomic contention | HIGH | ~10ms |
| Full grid scans | MEDIUM | ~5ms |

**Total: ~50ms** (vs 5ms on CPU)

---

## The Fix: 4 Phases

### Phase 1: Eliminate Serialized Loops (BIGGEST WIN)

**Current Problem:**
```python
# beetle_physics.py lines 7987, 8340, 8961
ti.loop_config(serialize=True)  # Forces ONE thread to do all work
for read_idx in range(20000):   # 20,000 iterations on 1 GPU thread
    # cleanup logic...
```

**Why it's slow:** GPU has 2000+ threads, but serialize=True uses only 1.

**The Fix: Free List Pattern (No Compaction Needed)**

Instead of removing dead particles and compacting the array, we:
1. Mark dead particles
2. Reuse their slots when spawning new particles
3. Never compact - just skip dead particles in physics

```python
# New fields
debris_active = ti.field(ti.i32, shape=MAX_DEBRIS)  # 1=alive, 0=dead
debris_free_head = ti.field(ti.i32, shape=())  # First free slot

@ti.kernel
def update_debris_parallel():
    """Update all particles in parallel - dead ones just get skipped"""
    for idx in range(MAX_DEBRIS):
        if debris_active[idx] == 0:
            continue  # Skip dead - no compaction needed

        # Update physics...
        debris_lifetime[idx] -= dt
        if debris_lifetime[idx] <= 0:
            debris_active[idx] = 0  # Mark dead (will be reused)

@ti.kernel
def spawn_debris_reuse(spawn_x: ti.f32, spawn_y: ti.f32, spawn_z: ti.f32, ...):
    """Spawn into first available slot"""
    for idx in range(MAX_DEBRIS):
        if debris_active[idx] == 0:
            # Found dead slot - reuse it
            debris_active[idx] = 1
            debris_pos[idx] = ti.Vector([spawn_x, spawn_y, spawn_z])
            debris_lifetime[idx] = 2.0
            # ...
            break  # Only spawn one per call
```

**Files to modify:**
- `simulation.py` - Add `debris_active`, `spray_active`, `silk_active` fields
- `beetle_physics.py` - Remove `cleanup_dead_debris/spray/silk`, modify `spawn_*` and `update_*`

**Expected improvement:** 20ms → 2ms (10x faster)

**Gameplay impact:** NONE - particles behave identically, just managed differently

---

### Phase 2: Spatial Hashing for Silk Collision

**Current Problem:**
```python
# beetle_physics.py lines 8778-8882
for silk_idx in range(600):           # Each silk particle
    for voxel_idx in range(800):      # Check ALL beetle voxels
        # Distance calculation...
# Total: 600 × 800 = 480,000 checks
```

**The Fix: Only Check Nearby Cells**

Already have `silk_grid_particles` for floor silk. Extend to beetle collision:

```python
# Beetle voxel spatial hash (8-unit cells)
beetle_cell_count = ti.field(ti.i32, shape=(16, 16, 16))  # 16x16x16 grid
beetle_cell_voxels = ti.field(ti.i32, shape=(16, 16, 16, 64))  # Up to 64 voxels per cell

@ti.kernel
def build_beetle_spatial_hash(beetle_x: ti.f32, beetle_z: ti.f32, rotation: ti.f32):
    """Build hash once per frame, before silk collision check"""
    # Clear counts
    for i, j, k in ti.ndrange(16, 16, 16):
        beetle_cell_count[i, j, k] = 0

    # Insert voxels
    for vi in range(body_cache_size[None]):
        world_pos = transform_voxel_to_world(vi, beetle_x, beetle_z, rotation)
        cell_x = int((world_pos.x + 64) / 8) % 16
        cell_y = int((world_pos.y) / 8) % 16
        cell_z = int((world_pos.z + 64) / 8) % 16

        slot = ti.atomic_add(beetle_cell_count[cell_x, cell_y, cell_z], 1)
        if slot < 64:
            beetle_cell_voxels[cell_x, cell_y, cell_z, slot] = vi

@ti.kernel
def check_silk_beetle_collision_fast():
    """O(N × 27) instead of O(N × M)"""
    for silk_idx in range(num_silk[None]):
        if silk_stuck[silk_idx] != 0:
            continue

        pos = silk_pos[silk_idx]
        cell_x = int((pos.x + 64) / 8) % 16
        cell_y = int((pos.y) / 8) % 16
        cell_z = int((pos.z + 64) / 8) % 16

        # Only check 27 neighboring cells
        for di, dj, dk in ti.static(ti.ndrange((-1, 2), (-1, 2), (-1, 2))):
            cx = (cell_x + di) % 16
            cy = (cell_y + dj) % 16
            cz = (cell_z + dk) % 16

            for slot in range(beetle_cell_count[cx, cy, cz]):
                vi = beetle_cell_voxels[cx, cy, cz, slot]
                # Check distance to this voxel only
```

**Files to modify:**
- `simulation.py` - Add spatial hash fields
- `beetle_physics.py` - New `build_beetle_spatial_hash`, modify `check_silk_beetle_collision`

**Expected improvement:** 15ms → 1-2ms (10x faster)

**Gameplay impact:** NONE - same collisions detected, just found faster

---

### Phase 3: Fix Anti-Stacking O(N²)

**Current Problem:**
```python
# Inside silk collision check (lines 8800-8805)
for other in range(num_silk[None]):  # Check ALL other silk
    if silk_stuck_voxel_idx[other] == best_voxel:
        voxel_taken = True
# 600 × 600 = 360,000 checks
```

**The Fix: Voxel Occupancy Lookup**

```python
# Track which beetle voxels have silk attached
silk_voxel_occupied_blue = ti.field(ti.i32, shape=MAX_BODY_VOXELS)
silk_voxel_occupied_red = ti.field(ti.i32, shape=MAX_BODY_VOXELS)

@ti.kernel
def mark_occupied_voxels():
    """Run once at start of silk collision check"""
    # Clear
    for i in range(MAX_BODY_VOXELS):
        silk_voxel_occupied_blue[i] = 0
        silk_voxel_occupied_red[i] = 0

    # Mark occupied
    for idx in range(num_silk[None]):
        if silk_stuck[idx] == 2:  # Stuck to beetle
            voxel = silk_stuck_voxel_idx[idx]
            if silk_stuck_beetle[idx] == 0:
                silk_voxel_occupied_blue[voxel] = 1
            else:
                silk_voxel_occupied_red[voxel] = 1

# Then in collision check:
if silk_voxel_occupied_blue[best_voxel] == 0:
    # Safe to stick (O(1) instead of O(N))
```

**Expected improvement:** 5ms → 0.1ms (50x faster)

**Gameplay impact:** NONE - same anti-stacking behavior

---

### Phase 4: Reduce Atomic Contention

**Current Problem:**
```python
# Every voxel placement does an atomic
idx = ti.atomic_add(dirty_voxel_count[None], 1)  # Called 800+ times
```

**The Fix: Batch by Beetle**

Since we place one beetle at a time, we know the count beforehand:

```python
@ti.kernel
def place_beetle_batched(start_idx: ti.i32, voxel_count: ti.i32, ...):
    """Place all voxels with pre-allocated range (no atomics in loop)"""
    for local_idx in range(voxel_count):
        global_idx = start_idx + local_idx
        # Place voxel without atomic
        dirty_voxel_x[global_idx] = ...

# Before calling:
start_idx = dirty_voxel_count[None]
dirty_voxel_count[None] += expected_voxel_count  # Single atomic
place_beetle_batched(start_idx, expected_voxel_count, ...)
```

**Expected improvement:** 5ms → 1ms (5x faster)

**Gameplay impact:** NONE

---

## Implementation Order

| Phase | Task | Time | Risk | Test |
|-------|------|------|------|------|
| 1a | Add active flags to debris | 30 min | Low | Debris still spawns/dies |
| 1b | Modify debris update to skip dead | 30 min | Low | Debris physics works |
| 1c | Modify debris spawn to reuse slots | 1 hr | Medium | No memory growth |
| 1d | Remove cleanup_dead_debris | 15 min | Low | Game doesn't crash |
| 1e | Repeat for spray | 1 hr | Low | Spray works |
| 1f | Repeat for silk | 1 hr | Low | Silk works |
| **Test Point** | Run with --vulkan, check physics time | | | |
| 2a | Add beetle spatial hash fields | 30 min | Low | Fields exist |
| 2b | Implement build_beetle_spatial_hash | 1 hr | Medium | Hash builds |
| 2c | Modify silk collision to use hash | 2 hr | Medium | Silk sticks correctly |
| **Test Point** | Silk collision still works | | | |
| 3a | Add voxel occupancy fields | 15 min | Low | Fields exist |
| 3b | Implement mark_occupied_voxels | 30 min | Low | Marks correctly |
| 3c | Use O(1) lookup in anti-stack | 30 min | Low | No double-sticking |
| **Test Point** | Full gameplay test | | | |
| 4 | Batch atomics in beetle placement | 2 hr | Medium | Beetles render |

**Total estimated time: 10-12 hours**

---

## Expected Results

### Before (Current Vulkan)
```
physics: 50ms
scene_particles: 3ms
FPS: 17
```

### After (Optimized Vulkan)
```
physics: 8-12ms (serialized loops: 0ms, collision: 2ms, other: 6-10ms)
scene_particles: 3ms
FPS: 60-80
```

### Fallback (CPU Backend)
```
physics: 5ms
scene_particles: 12ms (transfer)
FPS: 35-50
```

---

## How to Test

### After Phase 1 (Serialized Loops):
```bash
py -3.12 beetle_physics.py --vulkan
```
- Expected physics time: 30ms → should drop to ~15ms
- Check: Particles still spawn/die correctly

### After Phase 2 (Spatial Hash):
- Enable spider beetles (silk)
- Spray silk at beetles
- Check: Silk still sticks, no visual difference
- Expected physics time: ~15ms → ~8ms

### After Phase 3 (Anti-Stacking):
- Spray lots of silk at one beetle
- Check: Silk doesn't stack in same voxel
- Expected: No visible change, just faster

### After Phase 4 (Atomics):
- Full gameplay
- Check: Beetles render correctly
- Expected: Additional 2-3ms improvement

---

## Rollback Plan

If any phase breaks gameplay:

1. Each phase is independent - can skip broken phase
2. CPU backend still works as fallback
3. Git commit before each phase for easy revert

---

## Files to Modify

| File | Changes |
|------|---------|
| `simulation.py` | Add `debris_active`, `spray_active`, `silk_active` fields |
| `simulation.py` | Add `beetle_cell_count`, `beetle_cell_voxels` fields |
| `simulation.py` | Add `silk_voxel_occupied_blue/red` fields |
| `beetle_physics.py` | Remove 3 `cleanup_dead_*` functions |
| `beetle_physics.py` | Modify `spawn_debris/spray/silk` to reuse slots |
| `beetle_physics.py` | Modify `update_debris/spray/silk` to skip dead |
| `beetle_physics.py` | New `build_beetle_spatial_hash` function |
| `beetle_physics.py` | Modify `check_silk_beetle_collision` to use hash |
| `beetle_physics.py` | New `mark_occupied_voxels` function |
| `beetle_physics.py` | Modify anti-stack logic to use O(1) lookup |

---

## Success Criteria

1. **Performance:** 60+ FPS on RTX 3060/3070 with Vulkan backend
2. **Gameplay:** Beetles push, particles fly, silk sticks - exactly as before
3. **Stability:** No crashes, no memory growth, no visual glitches

---

## Questions Before Starting

1. Should we do Phase 1 first and test, or implement all phases then test?
2. Any beetle types or features I should test specifically?
3. Want me to create a separate branch for this work?
