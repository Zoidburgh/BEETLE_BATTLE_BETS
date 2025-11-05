# THERMITE: Vibe-Coding Implementation Guide
## From Zero to Playable Demo in 2 Weeks

---

## Philosophy: Build Fast, Optimize As You Go

**Traditional approach:**
Build everything → Optimize later → Discover it's fundamentally broken → Rewrite

**Our approach:**
Build minimal → Test performance → Optimize immediately → Add next feature → Repeat

**The secret:** With Taichi + GPU, you're STARTING with performance. But you need to verify it stays fast as you add complexity.

---

## Phase 0: Installation & Verification (30 Minutes)

### Step 1: Install the Stack

```bash
# Create project folder
mkdir thermite
cd thermite

# Create virtual environment (recommended)
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install Taichi
pip install taichi

# Install rendering
pip install pygame

# Verify GPU
python -c "import taichi as ti; ti.init(arch=ti.gpu); print('GPU Ready!')"
```

**Expected output:**
```
[Taichi] version 1.x.x, llvm 15.0.1, commit xxxxxxx, linux, python 3.x.x
[Taichi] Starting on arch=cuda
GPU Ready!
```

If you see `arch=cpu` instead, that's okay - it's still fast enough for prototyping. We'll optimize for GPU later.

### Step 2: Create Minimal Structure

```bash
# Create files
touch main.py
touch simulation.py
touch renderer.py
```

### Step 3: Verify Hot Reload Works

**main.py:**
```python
import pygame
import importlib
import simulation
import time

pygame.init()
screen = pygame.display.set_mode((800, 600))
clock = pygame.time.Clock()

running = True
last_reload = time.time()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    # Hot reload every 0.5 seconds
    if time.time() - last_reload > 0.5:
        importlib.reload(simulation)
        last_reload = time.time()
        print("Reloaded simulation.py")
    
    # Clear screen
    screen.fill((20, 20, 30))
    
    # Your simulation will go here
    simulation.step()
    
    pygame.display.flip()
    clock.tick(60)

pygame.quit()
```

**simulation.py:**
```python
import taichi as ti

ti.init(arch=ti.gpu)

@ti.kernel
def step():
    print("Simulation running!")
```

**Run it:**
```bash
python main.py
```

You should see a black window and "Reloaded simulation.py" every 0.5 seconds.

**Test hot reload:** While the window is open, edit `simulation.py` and change the print message. Save. Within 0.5 seconds, you'll see the new message.

**If this works → You're ready to vibe-code.**

---

## Day 1 Morning: Minimal Voxel World (2-3 hours)

### Goal: See voxels on screen, measure baseline performance

### Step 1: Create Basic Voxel Grid

**simulation.py:**
```python
import taichi as ti

ti.init(arch=ti.gpu)

# Start SMALL to verify performance
n_grid = 64  # 64x64x64 = 262,144 voxels
voxel_type = ti.field(dtype=ti.i32, shape=(n_grid, n_grid, n_grid))

# Voxel types
EMPTY = 0
STEEL = 1
CONCRETE = 2

@ti.kernel
def init_test_building():
    """Create a simple test structure"""
    # Floor
    for i, j in ti.ndrange(n_grid, n_grid):
        voxel_type[i, 0, j] = CONCRETE
    
    # Four corner columns
    for y in range(1, 20):
        voxel_type[10, y, 10] = STEEL
        voxel_type[50, y, 10] = STEEL
        voxel_type[10, y, 50] = STEEL
        voxel_type[50, y, 50] = STEEL

@ti.kernel
def step():
    """Physics step - empty for now"""
    pass

# Initialize
init_test_building()
```

### Step 2: Render Voxels (Simple First)

**renderer.py:**
```python
import taichi as ti
import pygame
import numpy as np

def render_voxels_2d(screen, voxel_field, n_grid):
    """
    Ultra-simple 2D projection
    Just draw a top-down view first
    """
    pixel_size = 800 // n_grid
    
    # Get voxel data from GPU to CPU (slow, but okay for now)
    voxels = voxel_field.to_numpy()
    
    # Draw top layer
    for i in range(n_grid):
        for j in range(n_grid):
            # Find highest non-empty voxel in column
            for k in range(n_grid-1, -1, -1):
                if voxels[i, k, j] != 0:
                    # Color based on type
                    if voxels[i, k, j] == 1:  # STEEL
                        color = (100, 100, 150)
                    elif voxels[i, k, j] == 2:  # CONCRETE
                        color = (150, 150, 150)
                    else:
                        color = (255, 255, 255)
                    
                    x = i * pixel_size
                    y = j * pixel_size
                    pygame.draw.rect(screen, color, (x, y, pixel_size, pixel_size))
                    break
```

### Step 3: Connect Everything

**main.py:**
```python
import pygame
import importlib
import simulation
import renderer
import time

pygame.init()
screen = pygame.display.set_mode((800, 600))
clock = pygame.time.Clock()

running = True
last_reload = time.time()

# Performance tracking
frame_times = []

while running:
    frame_start = time.time()
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    # Hot reload
    if time.time() - last_reload > 0.5:
        importlib.reload(simulation)
        importlib.reload(renderer)
        last_reload = time.time()
    
    # Clear screen
    screen.fill((20, 20, 30))
    
    # Simulation step
    step_start = time.time()
    simulation.step()
    ti.sync()  # Wait for GPU
    step_time = time.time() - step_start
    
    # Render
    render_start = time.time()
    renderer.render_voxels_2d(screen, simulation.voxel_type, simulation.n_grid)
    render_time = time.time() - render_start
    
    # FPS counter
    font = pygame.font.Font(None, 36)
    fps_text = font.render(f"FPS: {int(clock.get_fps())}", True, (255, 255, 255))
    step_text = font.render(f"Sim: {step_time*1000:.1f}ms", True, (255, 255, 0))
    render_text = font.render(f"Render: {render_time*1000:.1f}ms", True, (0, 255, 255))
    
    screen.blit(fps_text, (10, 10))
    screen.blit(step_text, (10, 50))
    screen.blit(render_text, (10, 90))
    
    pygame.display.flip()
    
    frame_time = time.time() - frame_start
    frame_times.append(frame_time)
    if len(frame_times) > 60:
        frame_times.pop(0)
    
    clock.tick(60)

# Print performance summary
avg_frame = sum(frame_times) / len(frame_times)
print(f"\nAverage frame time: {avg_frame*1000:.2f}ms ({1/avg_frame:.1f} FPS)")

pygame.quit()
```

### Step 4: Run & Verify Performance

```bash
python main.py
```

**What you should see:**
- A window with a top-down view
- Four steel columns (darker blue)
- Concrete floor (gray)
- FPS counter showing 60 FPS
- Sim time: < 1ms (almost nothing happening yet)
- Render time: 10-20ms (this is the slow part)

**CRITICAL CHECKPOINT:**
- If FPS < 30 → Something is wrong
- If Sim > 5ms → GPU not being used
- If Render > 50ms → Need optimization now

**If everything is 60 FPS → Perfect. Move on.**

---

## Day 1 Afternoon: Add Temperature (2-3 hours)

### Goal: See heat spread through voxels in real-time

⚠️ **PERFORMANCE WARNING:** Heat diffusion is expensive! Each voxel checks 6 neighbors. At 64³ that's 1.5M+ checks per frame. Watch your FPS counter closely. If it drops below 45 FPS, you'll need to:
- Update heat less frequently (every 2-3 frames)
- Use sparse updates (only hot voxels)
- Reduce grid size temporarily

### Iteration 1: Add Temperature Field

**Tell AI:**
> "I have a voxel grid in Taichi. Add a temperature field (float, one per voxel). Add a kernel that diffuses heat to neighboring voxels. Heat should spread faster through steel (high conductivity) than concrete (low conductivity)."

**AI will give you something like:**

```python
# Add to simulation.py
temperature = ti.field(dtype=ti.f32, shape=(n_grid, n_grid, n_grid))

# Material properties
thermal_conductivity = ti.field(dtype=ti.f32, shape=10)  # Indexed by voxel type

@ti.func
def get_conductivity(voxel_type: ti.i32) -> ti.f32:
    conductivities = [
        0.0,    # EMPTY
        50.0,   # STEEL (high)
        1.0,    # CONCRETE (low)
    ]
    return conductivities[voxel_type]

@ti.kernel
def diffuse_heat(dt: ti.f32):
    for i, j, k in ti.ndrange(n_grid, n_grid, n_grid):
        if voxel_type[i, j, k] == EMPTY:
            continue
        
        heat_transfer = 0.0
        my_conductivity = get_conductivity(voxel_type[i, j, k])
        
        # Check 6 neighbors
        for di, dj, dk in ti.static([
            (1,0,0), (-1,0,0), (0,1,0), (0,-1,0), (0,0,1), (0,0,-1)
        ]):
            ni, nj, nk = i+di, j+dj, k+dk
            
            # Bounds check
            if 0 <= ni < n_grid and 0 <= nj < n_grid and 0 <= nk < n_grid:
                if voxel_type[ni, nj, nk] != EMPTY:
                    neighbor_conductivity = get_conductivity(voxel_type[ni, nj, nk])
                    avg_conductivity = (my_conductivity + neighbor_conductivity) * 0.5
                    
                    temp_diff = temperature[ni, nj, nk] - temperature[i, j, k]
                    heat_transfer += temp_diff * avg_conductivity * dt * 0.01
        
        temperature[i, j, k] += heat_transfer

@ti.kernel
def place_heat_source(x: ti.i32, y: ti.i32, z: ti.i32, heat: ti.f32):
    """Add heat at position"""
    if 0 <= x < n_grid and 0 <= y < n_grid and 0 <= z < n_grid:
        temperature[x, y, z] = heat
```

**Update step function:**
```python
dt = 0.016  # ~60 FPS

@ti.kernel
def step():
    diffuse_heat(dt)
```

**Save file. Watch it reload.**

### Iteration 2: Visualize Temperature

**Tell AI:**
> "Update my renderer to color voxels based on temperature. Cold = original color. Hot (>400°C) = red. Very hot (>800°C) = orange. Extremely hot (>1400°C) = yellow-white."

```python
# Add to renderer.py
def temp_to_color(base_color, temp):
    """Blend base color with heat glow"""
    if temp < 400:
        return base_color
    elif temp < 800:
        # Red glow
        heat_factor = (temp - 400) / 400
        return (
            int(base_color[0] + (255 - base_color[0]) * heat_factor),
            int(base_color[1] * (1 - heat_factor)),
            int(base_color[2] * (1 - heat_factor))
        )
    elif temp < 1400:
        # Orange glow
        heat_factor = (temp - 800) / 600
        return (
            255,
            int(100 + 155 * heat_factor),
            0
        )
    else:
        # White hot
        return (255, 255, 200)

def render_voxels_2d(screen, voxel_field, temp_field, n_grid):
    pixel_size = 800 // n_grid
    voxels = voxel_field.to_numpy()
    temps = temp_field.to_numpy()
    
    for i in range(n_grid):
        for j in range(n_grid):
            for k in range(n_grid-1, -1, -1):
                if voxels[i, k, j] != 0:
                    # Base color
                    if voxels[i, k, j] == 1:  # STEEL
                        base = (100, 100, 150)
                    elif voxels[i, k, j] == 2:  # CONCRETE
                        base = (150, 150, 150)
                    else:
                        base = (255, 255, 255)
                    
                    # Apply heat
                    color = temp_to_color(base, temps[i, k, j])
                    
                    x = i * pixel_size
                    y = j * pixel_size
                    pygame.draw.rect(screen, color, (x, y, pixel_size, pixel_size))
                    break
```

**Update main.py renderer call:**
```python
renderer.render_voxels_2d(screen, simulation.voxel_type, simulation.temperature, simulation.n_grid)
```

### Iteration 3: Add Mouse Interaction

**Tell AI:**
> "Let me click on the screen to add heat at that voxel position. Convert screen coordinates to voxel grid coordinates."

```python
# Add to main.py
def screen_to_voxel(mouse_x, mouse_y, n_grid, screen_size=800):
    """Convert screen coords to voxel coords (top-down view)"""
    voxel_x = int((mouse_x / screen_size) * n_grid)
    voxel_z = int((mouse_y / screen_size) * n_grid)
    return voxel_x, voxel_z

# In main loop, add to event handling:
for event in pygame.event.get():
    if event.type == pygame.QUIT:
        running = False
    elif event.type == pygame.MOUSEBUTTONDOWN:
        mx, my = pygame.mouse.get_pos()
        vx, vz = screen_to_voxel(mx, my, simulation.n_grid)
        # Heat column of voxels
        for vy in range(simulation.n_grid):
            if simulation.voxel_type.to_numpy()[vx, vy, vz] != 0:
                simulation.place_heat_source(vx, vy, vz, 1500.0)
                break
```

### Step 5: Test Heat Spreading

**Run the game.**
**Click on a steel column.**

**What you should see:**
- Column turns red-orange (hot)
- Over next few seconds, heat spreads
- Steel columns conduct heat to each other
- Concrete floor barely heats up (low conductivity)

**Performance check:**
- FPS should still be 60
- Sim time should now be 2-5ms

**If sim time > 10ms:**
```python
# Reduce update frequency
heat_steps_per_frame = 1

def step():
    global heat_steps_per_frame
    heat_steps_per_frame += 1
    if heat_steps_per_frame >= 2:  # Only update heat every 2 frames
        diffuse_heat(dt * 2)
        heat_steps_per_frame = 0
```

**If this works → You have thermal simulation running.**

---

## Day 2 Morning: Structural Integrity (3 hours)

### Goal: Voxels that can't support weight should fall

### Iteration 1: Add Structural Physics

**Tell AI:**
> "Add structural integrity simulation. Each voxel should calculate the weight above it. If weight exceeds material strength, the voxel should be marked as 'failed' and removed. Steel is strong, concrete is medium, wood is weak."

```python
# Add to simulation.py
structural_health = ti.field(dtype=ti.f32, shape=(n_grid, n_grid, n_grid))
velocity = ti.Vector.field(3, dtype=ti.f32, shape=(n_grid, n_grid, n_grid))

# Material strengths (arbitrary units)
STEEL_STRENGTH = 1000.0
CONCRETE_STRENGTH = 100.0

@ti.func
def get_strength(voxel_type: ti.i32) -> ti.f32:
    strengths = [
        0.0,           # EMPTY
        1000.0,        # STEEL
        100.0,         # CONCRETE
    ]
    return strengths[voxel_type]

@ti.kernel
def compute_structural_stress():
    """Calculate load on each voxel"""
    # Pass 1: Calculate weight above each voxel
    for i, j, k in ti.ndrange(n_grid, n_grid, n_grid):
        if voxel_type[i, j, k] == EMPTY:
            continue
        
        # Count solid voxels above
        weight_above = 0.0
        for y in range(j+1, n_grid):
            if voxel_type[i, y, k] != EMPTY:
                weight_above += 1.0
        
        # Get material strength
        strength = get_strength(voxel_type[i, j, k])
        
        # Temperature weakens materials
        temp_factor = 1.0
        if temperature[i, j, k] > 400:
            # At 400°C, steel loses 50% strength
            # At 800°C, steel loses 90% strength
            if temperature[i, j, k] < 800:
                temp_factor = 1.0 - 0.5 * ((temperature[i, j, k] - 400) / 400)
            else:
                temp_factor = 0.1  # 90% strength loss
        
        effective_strength = strength * temp_factor
        
        # Check failure
        if weight_above > effective_strength:
            structural_health[i, j, k] = 0.0  # Failed
        else:
            structural_health[i, j, k] = 1.0  # Intact

@ti.kernel
def apply_structural_failures():
    """Remove failed voxels"""
    for i, j, k in ti.ndrange(n_grid, n_grid, n_grid):
        if structural_health[i, j, k] == 0.0:
            voxel_type[i, j, k] = EMPTY
            temperature[i, j, k] = 0.0

@ti.kernel
def apply_gravity(dt: ti.f32):
    """Make unsupported voxels fall"""
    for i, j, k in ti.ndrange(n_grid, n_grid, n_grid):
        if voxel_type[i, j, k] != EMPTY:
            # Check if voxel below is empty
            if j > 0 and voxel_type[i, j-1, k] == EMPTY:
                # Apply gravity
                velocity[i, j, k][1] -= 9.8 * dt
                
                # Move voxel down if velocity is enough
                if velocity[i, j, k][1] < -0.5:
                    # Swap with voxel below
                    voxel_type[i, j-1, k] = voxel_type[i, j, k]
                    temperature[i, j-1, k] = temperature[i, j, k]
                    velocity[i, j-1, k] = velocity[i, j, k]
                    
                    voxel_type[i, j, k] = EMPTY
                    temperature[i, j, k] = 0.0
                    velocity[i, j, k] = ti.Vector([0.0, 0.0, 0.0])
```

**Update step:**
```python
@ti.kernel
def step():
    diffuse_heat(dt)
    compute_structural_stress()
    apply_structural_failures()
    apply_gravity(dt)
```

### Step 6: Test Structural Collapse

**Run the game.**
**Heat a bottom corner column until it glows yellow-white.**
**Wait.**

**What should happen:**
- Column weakens (still visible but red-hot)
- After ~5-10 seconds at >800°C, column fails
- Voxels above lose support
- Structure begins collapsing
- Falling voxels create cascade

**Performance check:**
- FPS might drop to 45-50 during active collapse
- This is OKAY for now
- Sim time: 5-10ms

**If FPS < 30 during collapse:**
```python
# Optimize: Only check voxels that might be affected
# We'll do this in Day 3 optimization pass
```

---

## Day 2 Afternoon: Add Thermite (2 hours)

### Goal: Placeable heat sources that burn for duration

### Iteration 1: Thermite Mechanic

**Tell AI:**
> "Add a thermite system. Player can place thermite charges on voxels. When ignited, thermite burns at 2500°C for 30 seconds, then goes out. Thermite should be placed, then player clicks 'ignite' to start all charges."

```python
# Add to simulation.py
THERMITE = 3  # New voxel type

thermite_active = ti.field(dtype=ti.i32, shape=(n_grid, n_grid, n_grid))
thermite_timer = ti.field(dtype=ti.f32, shape=(n_grid, n_grid, n_grid))

thermite_burn_duration = 30.0  # seconds
thermite_temperature = 2500.0

@ti.kernel
def place_thermite(x: ti.i32, y: ti.i32, z: ti.i32):
    """Place thermite charge"""
    if voxel_type[x, y, z] != EMPTY:
        voxel_type[x, y, z] = THERMITE
        thermite_active[x, y, z] = 0  # Not burning yet

@ti.kernel
def ignite_all_thermite():
    """Start all thermite charges"""
    for i, j, k in ti.ndrange(n_grid, n_grid, n_grid):
        if voxel_type[i, j, k] == THERMITE:
            thermite_active[i, j, k] = 1
            thermite_timer[i, j, k] = thermite_burn_duration

@ti.kernel
def update_thermite(dt: ti.f32):
    """Update burning thermite"""
    for i, j, k in ti.ndrange(n_grid, n_grid, n_grid):
        if thermite_active[i, j, k] == 1:
            # Generate heat
            temperature[i, j, k] = thermite_temperature
            
            # Count down
            thermite_timer[i, j, k] -= dt
            
            # Burn out
            if thermite_timer[i, j, k] <= 0:
                thermite_active[i, j, k] = 0
                voxel_type[i, j, k] = EMPTY

@ti.kernel
def step():
    update_thermite(dt)
    diffuse_heat(dt)
    compute_structural_stress()
    apply_structural_failures()
    apply_gravity(dt)
```

### Iteration 2: UI for Thermite

**Add to main.py:**
```python
# Game state
mode = "PLACE"  # PLACE or IGNITE
thermite_budget = 5

# UI
font = pygame.font.Font(None, 24)

# In event loop:
elif event.type == pygame.KEYDOWN:
    if event.key == pygame.K_SPACE:
        if mode == "PLACE":
            mode = "IGNITE"
            simulation.ignite_all_thermite()
        else:
            mode = "PLACE"
    elif event.key == pygame.K_r:
        # Reset level
        simulation.init_test_building()
        mode = "PLACE"
        thermite_budget = 5

elif event.type == pygame.MOUSEBUTTONDOWN and mode == "PLACE":
    if thermite_budget > 0:
        mx, my = pygame.mouse.get_pos()
        vx, vz = screen_to_voxel(mx, my, simulation.n_grid)
        # Find top voxel in column
        for vy in range(simulation.n_grid-1, -1, -1):
            if simulation.voxel_type.to_numpy()[vx, vy, vz] != 0:
                simulation.place_thermite(vx, vy, vz)
                thermite_budget -= 1
                break

# Render UI
mode_text = font.render(f"Mode: {mode} (SPACE to toggle)", True, (255, 255, 255))
budget_text = font.render(f"Thermite: {thermite_budget}", True, (255, 165, 0))
screen.blit(mode_text, (10, 130))
screen.blit(budget_text, (10, 160))
```

### Step 7: Test Complete Loop

**Run game.**

**Flow:**
1. Click on voxels to place thermite (orange markers)
2. Press SPACE to ignite
3. Watch thermite burn at 2500°C
4. Heat spreads to nearby steel
5. Steel weakens
6. Structure collapses
7. Press R to reset

**This is your MVP game loop.**

**Performance check:**
- With 5 burning thermite charges: FPS should be 45-60
- Sim time: 8-15ms

---

## Day 3 Morning: Optimization Pass (3 hours)

### Goal: Get back to solid 60 FPS even with complex collapses

### Optimization 1: Sparse Simulation

**Problem:** We're updating ALL voxels, even empty ones.

**Tell AI:**
> "Optimize my simulation to only update non-empty voxels. Use Taichi's sparse grid feature or iterate only over active voxels."

```python
# Rebuild with active voxel tracking
active_voxels = ti.field(dtype=ti.i32, shape=n_grid * n_grid * n_grid)
num_active = ti.field(dtype=ti.i32, shape=())

@ti.kernel
def rebuild_active_list():
    """Find all non-empty voxels"""
    num_active[None] = 0
    for i, j, k in ti.ndrange(n_grid, n_grid, n_grid):
        if voxel_type[i, j, k] != EMPTY:
            idx = ti.atomic_add(num_active[None], 1)
            # Pack coordinates into single int
            active_voxels[idx] = (i << 16) | (j << 8) | k

@ti.kernel
def diffuse_heat_sparse(dt: ti.f32):
    """Heat diffusion - only active voxels"""
    n = num_active[None]
    for idx in range(n):
        packed = active_voxels[idx]
        i = (packed >> 16) & 0xFF
        j = (packed >> 8) & 0xFF
        k = packed & 0xFF
        
        # Same heat diffusion logic as before
        # ... (same neighbor loop)
```

**Rebuild active list periodically:**
```python
frame_count = 0

def step():
    global frame_count
    frame_count += 1
    
    # Rebuild active list every 10 frames
    if frame_count % 10 == 0:
        simulation.rebuild_active_list()
    
    simulation.update_thermite(simulation.dt)
    simulation.diffuse_heat_sparse(simulation.dt)
    simulation.compute_structural_stress()
    simulation.apply_structural_failures()
    simulation.apply_gravity(simulation.dt)
```

**Expected improvement:** 2-3x faster sim time

### Optimization 2: Reduce Heat Diffusion Updates

**Problem:** Heat diffusion every frame is overkill

```python
# Only update heat every N frames
heat_update_interval = 3  # Update every 3 frames
heat_update_counter = 0

def step():
    global heat_update_counter
    heat_update_counter += 1
    
    simulation.update_thermite(simulation.dt)
    
    if heat_update_counter >= heat_update_interval:
        simulation.diffuse_heat_sparse(simulation.dt * heat_update_interval)
        heat_update_counter = 0
    
    simulation.compute_structural_stress()
    simulation.apply_structural_failures()
    simulation.apply_gravity(simulation.dt)
```

**Expected improvement:** Another 2x faster

### Optimization 3: Faster Rendering

**Problem:** Copying data from GPU to CPU every frame is slow

```python
# Cache voxel data, only update when it changes
voxel_cache = None
temp_cache = None
cache_dirty = True
cache_frame_count = 0

def render_voxels_2d_cached(screen, voxel_field, temp_field, n_grid):
    global voxel_cache, temp_cache, cache_dirty, cache_frame_count
    
    cache_frame_count += 1
    
    # Update cache every 5 frames (or when dirty)
    if cache_dirty or cache_frame_count >= 5:
        voxel_cache = voxel_field.to_numpy()
        temp_cache = temp_field.to_numpy()
        cache_dirty = False
        cache_frame_count = 0
    
    # Render from cache (same logic as before, but using cached data)
    # ...
```

**Expected improvement:** Render time drops from 20ms to 5ms

### Performance Target After Optimization

**Goal metrics:**
- FPS: 60 (even during collapse)
- Sim time: < 5ms
- Render time: < 5ms
- Total frame time: < 16ms (60 FPS = 16.67ms per frame)

**If you hit these numbers → Ready for content creation**

---

## Day 3 Afternoon: Better Rendering (3 hours)

### Goal: 3D isometric view instead of top-down

### Iteration 1: Isometric Projection

**Tell AI:**
> "Convert my 2D top-down renderer to isometric 3D. Use simple isometric projection (no perspective). Render front-to-back for proper occlusion."

```python
def render_voxels_isometric(screen, voxel_field, temp_field, n_grid):
    """Isometric projection"""
    voxels = voxel_field.to_numpy()
    temps = temp_field.to_numpy()
    
    # Isometric projection parameters
    tile_width = 10
    tile_height = 5
    
    # Offset to center on screen
    offset_x = 400
    offset_y = 100
    
    # Render back-to-front for proper occlusion
    for i in range(n_grid):
        for k in range(n_grid):
            for j in range(n_grid):
                if voxels[i, j, k] != 0:
                    # Isometric projection
                    screen_x = offset_x + (i - k) * tile_width
                    screen_y = offset_y + (i + k) * tile_height // 2 - j * tile_height
                    
                    # Get color
                    if voxels[i, j, k] == 1:  # STEEL
                        base = (100, 100, 150)
                    elif voxels[i, j, k] == 2:  # CONCRETE
                        base = (150, 150, 150)
                    elif voxels[i, j, k] == 3:  # THERMITE
                        base = (255, 100, 0)
                    else:
                        base = (255, 255, 255)
                    
                    color = temp_to_color(base, temps[i, j, k])
                    
                    # Draw isometric tile (diamond shape)
                    points = [
                        (screen_x, screen_y),
                        (screen_x + tile_width, screen_y + tile_height // 2),
                        (screen_x, screen_y + tile_height),
                        (screen_x - tile_width, screen_y + tile_height // 2)
                    ]
                    pygame.draw.polygon(screen, color, points)
```

**This looks 10x better immediately.**

### Iteration 2: Add Simple Lighting

```python
def render_voxels_isometric_lit(screen, voxel_field, temp_field, n_grid):
    """Isometric with ambient occlusion"""
    voxels = voxel_field.to_numpy()
    temps = temp_field.to_numpy()
    
    # Pre-compute occlusion
    occlusion = compute_ambient_occlusion(voxels, n_grid)
    
    # Same rendering loop, but darken based on occlusion
    for i in range(n_grid):
        for k in range(n_grid):
            for j in range(n_grid):
                if voxels[i, j, k] != 0:
                    # ... (same projection)
                    
                    color = temp_to_color(base, temps[i, j, k])
                    
                    # Apply occlusion
                    ao = occlusion[i, j, k]
                    color = (
                        int(color[0] * ao),
                        int(color[1] * ao),
                        int(color[2] * ao)
                    )
                    
                    pygame.draw.polygon(screen, color, points)

def compute_ambient_occlusion(voxels, n_grid):
    """Simple AO: darken based on neighbor count"""
    occlusion = np.ones((n_grid, n_grid, n_grid))
    
    for i in range(n_grid):
        for j in range(n_grid):
            for k in range(n_grid):
                if voxels[i, j, k] != 0:
                    # Count neighbors
                    neighbor_count = 0
                    for di in [-1, 0, 1]:
                        for dj in [-1, 0, 1]:
                            for dk in [-1, 0, 1]:
                                ni, nj, nk = i+di, j+dj, k+dk
                                if (0 <= ni < n_grid and 
                                    0 <= nj < n_grid and 
                                    0 <= nk < n_grid):
                                    if voxels[ni, nj, nk] != 0:
                                        neighbor_count += 1
                    
                    # Occlusion factor (0.4 to 1.0)
                    occlusion[i, j, k] = 0.4 + 0.6 * (1.0 - neighbor_count / 27.0)
    
    return occlusion
```

### Iteration 3: Add Camera Controls (CRITICAL - DO THIS NOW)

**Problem:** You can't see the structure from different angles. Camera control is not optional.

**Tell AI:**
> "Add camera rotation to my isometric renderer. Arrow keys should rotate the view 90 degrees around the structure. Make it so the building stays centered but the viewing angle changes."

```python
# Add to main.py
camera_angle = 0  # 0, 90, 180, 270
camera_zoom = 1.0

# In event loop
elif event.type == pygame.KEYDOWN:
    if event.key == pygame.K_LEFT:
        camera_angle = (camera_angle - 90) % 360
    elif event.key == pygame.K_RIGHT:
        camera_angle = (camera_angle + 90) % 360
    elif event.key == pygame.K_UP:
        camera_zoom *= 1.2
    elif event.key == pygame.K_DOWN:
        camera_zoom /= 1.2

# Update renderer to use camera_angle
def render_voxels_isometric_with_camera(screen, voxel_field, temp_field, n_grid, camera_angle, camera_zoom):
    # Rotate voxel coordinates based on camera_angle
    # 0° = view from front
    # 90° = view from right
    # 180° = view from back
    # 270° = view from left

    # Apply rotation matrix to i, k coordinates before projection
    for i in range(n_grid):
        for k in range(n_grid):
            for j in range(n_grid):
                if voxels[i, j, k] != 0:
                    # Rotate around center
                    center = n_grid // 2
                    i_centered = i - center
                    k_centered = k - center

                    # Rotate based on camera_angle
                    if camera_angle == 90:
                        i_rot = -k_centered
                        k_rot = i_centered
                    elif camera_angle == 180:
                        i_rot = -i_centered
                        k_rot = -k_centered
                    elif camera_angle == 270:
                        i_rot = k_centered
                        k_rot = -i_centered
                    else:  # 0
                        i_rot = i_centered
                        k_rot = k_centered

                    # Add center back
                    i_rot += center
                    k_rot += center

                    # Now do isometric projection with i_rot, k_rot
                    screen_x = offset_x + (i_rot - k_rot) * tile_width * camera_zoom
                    screen_y = offset_y + (i_rot + k_rot) * tile_height // 2 - j * tile_height * camera_zoom
                    # ... rest of rendering
```

**Test it:**
- Run game
- Press LEFT/RIGHT arrow keys
- Building should rotate 90° each press
- You can now see all sides

**This is ESSENTIAL.** Don't skip this. You can't make levels without seeing the structure from all angles.

---

## Day 4: First Real Level (Full Day)

### Goal: Create "Tutorial 1: The Shed" from design doc

### Step 1: Level Definition

```python
# levels.py
import taichi as ti
from simulation import voxel_type, temperature, EMPTY, STEEL, CONCRETE, WOOD

WOOD = 4  # Add wood type

@ti.kernel
def load_level_1():
    """Tutorial 1: The Shed"""
    # Clear everything
    for i, j, k in ti.ndrange(64, 64, 64):
        voxel_type[i, j, k] = EMPTY
        temperature[i, j, k] = 20.0  # Ambient temp
    
    # Wooden floor (5x5)
    for i in range(25, 35):
        for k in range(25, 35):
            voxel_type[i, 0, k] = WOOD
    
    # Four corner posts (height 8)
    for j in range(1, 9):
        voxel_type[25, j, 25] = WOOD
        voxel_type[34, j, 25] = WOOD
        voxel_type[25, j, 34] = WOOD
        voxel_type[34, j, 34] = WOOD
    
    # Roof (flat)
    for i in range(25, 35):
        for k in range(25, 35):
            voxel_type[i, 8, k] = WOOD

# Level metadata
level_1_data = {
    "name": "The Shed",
    "objective": "Demolish completely",
    "thermite_budget": 2,
    "par_time": 30,
    "load_function": load_level_1
}
```

### Step 2: Add Wood Physics

```python
# Update simulation.py
WOOD = 4

# Wood properties
WOOD_STRENGTH = 20.0
WOOD_CONDUCTIVITY = 0.1
WOOD_IGNITION_TEMP = 300.0

wood_on_fire = ti.field(dtype=ti.i32, shape=(n_grid, n_grid, n_grid))

@ti.kernel
def update_fire(dt: ti.f32):
    """Wood catches fire and burns"""
    for i, j, k in ti.ndrange(n_grid, n_grid, n_grid):
        if voxel_type[i, j, k] == WOOD:
            # Check if hot enough to ignite
            if temperature[i, j, k] > WOOD_IGNITION_TEMP:
                wood_on_fire[i, j, k] = 1
            
            # If on fire, generate heat
            if wood_on_fire[i, j, k] == 1:
                temperature[i, j, k] = 800.0  # Burning wood temp
                
                # Spread fire to adjacent wood
                for di, dj, dk in ti.static([
                    (1,0,0), (-1,0,0), (0,1,0), (0,-1,0), (0,0,1), (0,0,-1)
                ]):
                    ni, nj, nk = i+di, j+dj, k+dk
                    if (0 <= ni < n_grid and 0 <= nj < n_grid and 0 <= nk < n_grid):
                        if voxel_type[ni, nj, nk] == WOOD:
                            # Heat adjacent wood
                            temperature[ni, nj, nk] += 100.0 * dt

# Add to step()
update_fire(dt)
```

### Step 3: Win Condition

```python
# Add to main.py
level_complete = False
objective_text = "Objective: Demolish the shed"

@ti.kernel
def check_level_complete() -> ti.i32:
    """Check if any structure remains"""
    remaining = 0
    for i, j, k in ti.ndrange(simulation.n_grid, simulation.n_grid, simulation.n_grid):
        if simulation.voxel_type[i, j, k] == WOOD:
            remaining += 1
    return remaining

# In main loop
if mode == "IGNITE" and not level_complete:
    remaining = check_level_complete()
    if remaining == 0:
        level_complete = True
        print("LEVEL COMPLETE!")

# Render win screen
if level_complete:
    win_text = font.render("LEVEL COMPLETE!", True, (0, 255, 0))
    screen.blit(win_text, (300, 300))
```

### Step 4: Playtest & Iterate

**Run level 1.**

**Expected flow:**
1. Player places 2 thermite on opposite corners
2. Ignites
3. Wood ignites at 300°C
4. Fire spreads
5. Structure burns and collapses
6. "LEVEL COMPLETE!" appears

**Iterate on feel:**
- Is fire spreading too fast? Adjust spread rate
- Is collapse too slow? Reduce wood strength
- Is it too easy? Reduce thermite budget

**Keep iterating until it FEELS good.**

---

## Week 2: ONLY IF WEEK 1 WORKED

**STOP. Check Week 1 success criteria first:**
- Does simulation run at 60 FPS?
- Does heat spread visibly?
- Do structures collapse when heated?
- Can you rotate the camera?
- Does placing and igniting thermite work?

**If NO to any of above → Don't start Week 2. Fix Week 1 first.**

**If YES → Proceed carefully:**

### Day 1-2: Make Collapse Feel Better

The technical part works, but does it FEEL satisfying?

- Adjust collapse timing (not instant, not glacial)
- Tune gravity strength
- Make sure camera shows the action
- Add simple screen shake when big pieces fall
- Test, iterate, test, iterate

**Goal:** You want to keep watching collapses. They're satisfying.

### Day 3-4: Make 2-3 More Test Levels

**Not 5 levels. Not 10 levels. Just 2-3.**

Use simple structures:
- Level 2: Steel frame building (test steel heating)
- Level 3: Multi-floor structure (test cascade)

**Goal:** Learn what's fun. What's frustrating. What needs changing.

### Day 5-7: Fix What's Broken

You'll discover issues:
- Fire spreads too fast/slow
- Structural calculation is wrong
- Camera is annoying
- Performance drops with large collapses

**Fix these before making more content.**

### DO NOT:

- Add 10 materials
- Make particle systems
- Add sound effects
- Add menus and polish
- Think about ratings or scoring

**Those are for Week 4+ IF the core is proven fun.**

---

## The Instant Feedback Loop Mastered

### The Pattern You'll Repeat 100+ Times

```
1. Identify what needs improvement
   "Fire spreads too slow"

2. Describe to AI
   "Make fire spread 3x faster by increasing heat transfer to adjacent wood"

3. AI generates code
   [Paste code snippet]

4. Save file
   [Hot reload kicks in]

5. Test immediately (< 5 seconds)
   [Click, place, ignite, watch]

6. Evaluate
   "Better, but still too slow" OR "Perfect!"

7. If not perfect → Go to step 2
   If perfect → Move to next thing
```

**This loop takes 30-60 seconds per iteration.**

**You can do 50+ iterations per hour.**

**That's why you can ship in weeks, not months.**

---

## Performance Targets Throughout Development

### Day 1:
- 64³ grid
- Basic heat + structure
- **Target:** 60 FPS

### Day 3:
- 64³ grid
- Full physics + thermite
- **Target:** 60 FPS (after optimization)

### Week 2:
- 128³ grid (8x more voxels!)
- All materials
- Particles
- **Target:** 45-60 FPS

**How to scale up:**

```python
# Start at 64
n_grid = 64

# After optimization, bump to 96
n_grid = 96

# If still fast, go to 128
n_grid = 128

# Stop when FPS drops below 45
```

**The beauty:** You can adjust resolution without changing ANY game logic.

---

## Common Issues & Instant Fixes

### "My simulation is unstable/exploding"

**Symptoms:** Voxels flying everywhere, temperatures going to infinity

**Fix:**
```python
# Clamp temperature
@ti.kernel
def diffuse_heat(dt: ti.f32):
    # ... heat logic ...
    # At the end:
    temperature[i, j, k] = ti.min(temperature[i, j, k], 5000.0)  # Cap at 5000°C
```

### "Heat isn't spreading"

**Debug:**
```python
# Print temperatures
@ti.kernel
def debug_print():
    for i, j, k in ti.ndrange(n_grid, n_grid, n_grid):
        if temperature[i, j, k] > 100:
            print(f"Hot voxel at ({i},{j},{k}): {temperature[i, j, k]}")

# Call once per second
if time.time() % 1.0 < 0.1:
    debug_print()
```

### "Structure won't collapse"

**Debug:**
```python
# Visualize structural health
# In renderer, color voxels by health:
if structural_health[i, j, k] < 0.5:
    color = (255, 0, 0)  # Red = about to fail
```

### "FPS dropping"

**Profile:**
```python
import time

# Around each kernel call
start = time.time()
diffuse_heat(dt)
ti.sync()
print(f"Heat: {(time.time()-start)*1000:.1f}ms")

start = time.time()
compute_structural_stress()
ti.sync()
print(f"Structure: {(time.time()-start)*1000:.1f}ms")
```

**Then optimize the slowest part first.**

---

## Reality Check Checklist (Not "Shipping")

### Week 1 End - Tech Validation:
- [ ] Simulation runs at 60 FPS (45+ minimum)
- [ ] Heat visibly spreads through materials
- [ ] Structures collapse when heated
- [ ] Camera can rotate/zoom
- [ ] You can place and ignite thermite
- [ ] No critical bugs or crashes

**If you check all these → Week 1 was a success. Proceed.**

### Week 2 End - Fun Validation:
- [ ] You want to try "just one more" demolition
- [ ] Collapse timing feels satisfying (not too fast/slow)
- [ ] You've shown it to one other person
- [ ] They understood it without explanation
- [ ] They played for 10+ minutes voluntarily
- [ ] Performance still good during collapse

**If you check most of these → The core is fun. Worth continuing.**

### Week 3 End - Proof of Concept:
- [ ] 3-5 levels that teach the mechanics
- [ ] Each level feels different/teaches something new
- [ ] Friends have played and given feedback
- [ ] You've fixed the top 3 complaints
- [ ] You still want to work on it

**If you check these → You have something. Decide: keep going or pivot?**

### Beyond Week 3 - TBD

Don't plan this far ahead. See what you learn first.

---

## The Power of This Approach

**Traditional indie game dev:**
- Months learning Unity
- Fighting engine limitations
- Compromise vision
- Ship something generic

**Your approach:**
- Days learning basics
- Physics does what you tell it
- Iterate in SECONDS
- Ship something unique

**Example iteration counts:**

**Traditional (Unity):**
- Change physics behavior: 30 minutes (recompile, test, debug)
- Try new material: 2 hours (write scripts, debug, integrate)
- Balance level: 10 iterations x 5 minutes = 50 minutes

**Your approach (Taichi + AI):**
- Change physics behavior: 2 minutes (ask AI, paste, test)
- Try new material: 10 minutes (ask AI, paste, test, iterate)
- Balance level: 50 iterations x 1 minute = 50 minutes

**You're iterating 10-20x faster.**

**That's how you ship in weeks.**

---

## Final Mindset

### Performance First, Always

If it doesn't run at 45+ FPS, nothing else matters.
Optimize immediately when FPS drops. Don't accumulate tech debt.

### Fun Is Not Guaranteed

The physics might work perfectly and still be boring.
Be ready to pivot or stop if Week 2 validation fails.
Don't get attached to the idea - get attached to what works.

### Camera Is Not Optional

You can't test levels if you can't see the structure.
Do camera on Day 3, not "later."

### Test With Other Humans Early

You'll be blind to problems. Others won't.
Week 2: Show someone. Watch them play. Don't explain anything.
Their confusion is data.

### The Real Goal

**Week 1 Goal:**
Prove the simulation can work with good performance.

**Week 2 Goal:**
Prove the core loop is actually fun (or discover it's not).

**Week 3 Goal:**
Have something worth showing publicly, or have learned enough to pivot.

**There is no Week 6 plan. See what Week 1-3 teaches you first.**

**Now stop reading and start building.**
