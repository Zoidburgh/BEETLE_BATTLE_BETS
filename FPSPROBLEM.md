# FPS Performance Problem Investigation

## The Problem

Massive FPS variance across systems with no clear pattern:

| System | GPU | Expected FPS | Actual FPS |
|--------|-----|--------------|------------|
| Some users | Various | 60+ | **130-140** |
| Friend 1 | RTX 3070 | 100+ | **38** |
| Friend 2 | RTX 3070 Ti | 100+ | **25-30** |
| Other friends | Good GPUs | 60+ | **~30-40** |
| Dev laptop | Integrated Intel | 30-40 | **70** |

**The RTX 3070 getting 38 FPS while an integrated laptop gets 70 FPS makes no sense.**

---

## Root Cause Theory

The game uses Taichi for physics and rendering. Taichi can run on different backends:
- **CPU** - Uses CPU for compute
- **CUDA** - Uses NVIDIA GPU (native)
- **Vulkan** - Uses GPU (cross-platform)

The rendering (`scene.particles()`) always uses Vulkan/GPU regardless of backend.

### The Core Issue: CPU-GPU Data Transfer

When using **CPU backend**:
1. All voxel data (positions, colors, radii) lives in CPU RAM
2. `scene.particles()` must copy 5000+ particles to GPU every frame
3. This transfer takes **8-12ms per frame** (the bottleneck!)
4. Physics runs fast on CPU (~5ms)

When using **GPU backend** (Vulkan/CUDA):
1. All voxel data lives on GPU
2. `scene.particles()` reads directly from GPU memory (<1ms)
3. BUT physics may run slower on some GPUs

### The Dilemma

| Backend | Physics | Render Transfer | Total |
|---------|---------|-----------------|-------|
| CPU | Fast (5ms) | Slow (12ms) | ~25ms (40 FPS) |
| GPU (ideal) | Fast (5ms) | None (1ms) | ~8ms (120 FPS) |
| GPU (broken) | Slow (34ms) | None (1ms) | ~40ms (25 FPS) |

Some GPUs have slow Vulkan/CUDA compute, making physics terrible even though rendering is fast.

---

## What We've Tried

### Attempt 1: Force CPU Backend (Original)
```python
ti.init(arch=ti.cpu)
```
- **Result:** Consistent but slow (38-48 FPS on good GPUs)
- **Problem:** 12ms wasted on CPU→GPU transfer every frame

### Attempt 2: Force Vulkan Backend
```python
ti.init(arch=ti.vulkan)
```
- **Result on dev laptop:** Physics 72ms, unplayable
- **Result on 3070:** Physics 34ms, worse than CPU!
- **Problem:** Vulkan compute is slow on some systems

### Attempt 3: Auto-detect Discrete GPU → Vulkan
```python
if detect_discrete_gpu():
    ti.init(arch=ti.vulkan)
else:
    ti.init(arch=ti.cpu)
```
- **Result:** 3070 user dropped from 38 FPS to 17 FPS
- **Problem:** Vulkan compute still slow

### Attempt 4: NVIDIA → CUDA, AMD → Vulkan, else → CPU (Current)
```python
if gpu_type == 'nvidia':
    ti.init(arch=ti.cuda)
elif gpu_type == 'amd':
    ti.init(arch=ti.vulkan)
else:
    ti.init(arch=ti.cpu)
```
- **Result:** Untested, waiting for friend feedback
- **Theory:** CUDA is NVIDIA's native API, might work better than Vulkan

---

## Performance Logs

### Friend with RTX 3070 (CPU Backend) - 38 FPS
```
frame_total: 24.35ms
physics: 5.10ms          <- Fast
scene_render: 15.64ms
  scene_particles: 12.03ms  <- THE BOTTLENECK (CPU→GPU transfer)
```

### Friend with RTX 3070 (Vulkan Backend) - 17 FPS
```
frame_total: 55.31ms
physics: 34.31ms         <- Vulkan compute is SLOW
scene_render: 13.73ms
  scene_particles: 10.26ms  <- Slightly better
```

### Dev Laptop (CPU Backend) - 70 FPS
```
frame_total: ~14ms
physics: ~5ms
scene_particles: ~3ms    <- Why is this faster??
```

---

## Unanswered Questions

1. **Why does dev laptop get 3ms scene_particles while 3070 gets 12ms?**
   - Same CPU backend, same data transfer
   - Laptop should be slower, not faster

2. **Why is Vulkan compute slow on RTX 3070?**
   - 3070 should crush Taichi physics kernels
   - Is it driver issue? Taichi bug? Our code?

3. **Who's getting 130+ FPS and what's their setup?**
   - Need perf log from fast systems to compare
   - What backend are they using?

4. **Would CUDA fix the 3070 issue?**
   - CUDA is NVIDIA native, should be optimized
   - Testing pending

---

## Potential Fixes Not Yet Tried

### 1. Optimize the Extract Voxels Kernel
Current code scans entire 128x128x128 grid (2M voxels) to find ~5000 active voxels.
```python
for i, j, k in ti.ndrange((2, 126), (1, 100), (2, 126)):
    # Check every voxel
```
Could maintain a list of active voxels instead.

### 2. Reduce Particle Count
Currently rendering 5000-7000 particles. Could:
- Use lower LOD for distant beetles
- Reduce debris particles
- Simplify beetle geometry

### 3. Batch Data Differently
Currently 3 separate arrays (positions, colors, radii) = 3 transfers.
Could pack into single struct array = 1 transfer.

### 4. Profile Specific Kernels
Use Taichi's kernel profiler to find which specific kernel is slow:
```python
ti.init(kernel_profiler=True)
# ... run game ...
ti.profiler.print_kernel_profiler_info()
```

### 5. Try Different Taichi Versions
Maybe a Taichi bug? Current version: 1.7.4

---

## Command Line Flags

Users can override auto-detection:
```
--cpu      Force CPU backend
--cuda     Force CUDA backend (NVIDIA)
--vulkan   Force Vulkan backend
--gpu      Same as --vulkan
```

---

## Files Involved

- `simulation.py` - Backend selection, ti.init(), physics kernels
- `renderer.py` - scene.particles() call, voxel extraction
- `beetle_physics.py` - Main loop, timing code
- `perf_log.txt` - Performance data output

---

## Status

**Current approach:** Auto-detect GPU and use native backend (CUDA for NVIDIA, Vulkan for AMD, CPU for integrated)

---

## Deep Analysis Results (2026-01-10)

### CUDA Test Results - WORSE!

| Backend | Physics | scene_particles | FPS |
|---------|---------|-----------------|-----|
| CPU | 5ms | 12ms | 38 |
| Vulkan | 34ms | 10ms | 17 |
| CUDA | 18ms | **27ms** | **20** |

**Why CUDA scene_particles is 27ms:** Taichi GGUI always uses Vulkan for rendering. CUDA data must be copied CUDA→Vulkan which is slower than CPU→Vulkan!

### Root Cause: Serialized Loops on GPU

Found 3 kernels with `ti.loop_config(serialize=True)`:
- `cleanup_dead_debris()` - line 7992 - up to 20,000 particles sequentially
- `cleanup_dead_spray()` - line 8344 - up to 500 particles sequentially
- `cleanup_dead_silk()` - line 8966 - up to 600 particles sequentially

**On CPU:** Sequential is fine, it's fast
**On GPU:** One thread does all work, thousands sit idle = DISASTER

### Other Issues Found

1. **Heavy atomics** - Vulkan lacks CUDA's 100x TLS optimization for atomics
2. **O(N²) silk collision** - Nested loops over 800 voxels × 600 silk particles
3. **Excessive ti.sync() calls** - 8 sync points causing pipeline stalls

### Fix Applied: Reduced Cleanup Frequency on GPU

```python
# simulation.py
CLEANUP_FREQUENCY_DEBRIS = 2 if BACKEND == 'cpu' else 30  # Every 0.5s on GPU
CLEANUP_FREQUENCY_SPRAY = 2 if BACKEND == 'cpu' else 30
CLEANUP_FREQUENCY_SILK = 5 if BACKEND == 'cpu' else 60
```

This reduces serialized loop impact by 15x. Tradeoff: dead particles accumulate longer.

### The Ideal Solution

**Vulkan backend with parallel-safe compaction:**
1. Replace serialized loops with double-buffered parallel compaction
2. Use atomic counters for parallel-safe writes
3. Would give: Fast physics (parallel) + Fast rendering (no transfer)

But this requires significant code refactoring.

### Current Recommendation

1. Test the reduced cleanup frequency fix on GPU backends
2. If physics is still slow, fall back to CPU backend (consistent 38 FPS)
3. Long-term: Refactor particle systems for parallel-safe compaction

---

## Next Steps

1. Have friend test with new cleanup frequency settings
2. If GPU physics is now faster, we found the main issue
3. If not, the atomics and O(N²) loops are also contributing
