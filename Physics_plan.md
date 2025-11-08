# PATH 1: IMPACT - Physics Goal Plan

## 🎯 CORE PHYSICS GOALS

### Goal 1: **Satisfying Impact Destruction**
**What it means:** When projectile hits structure, instant visible destruction with appropriate scale
- Explosion radius scales with projectile velocity (faster = bigger boom)
- Destroyed voxels immediately spawn debris particles
- Impact feels powerful and immediate (< 100ms from hit to explosion)

**Physics parameters to tune:**
- Base destruction radius: 3-8 voxels
- Velocity multiplier: speed/50 = extra radius (100 m/s projectile = +2 voxels)
- Material resistance multiplier (see Goal 2)

**Success looks like:**
- Hit tower at 50 m/s → 5 voxel radius explosion, 400 voxels destroyed
- Hit tower at 150 m/s → 10 voxel radius explosion, 4000 voxels destroyed
- Player thinks: "HELL YEAH" not "why so little damage?"

---

### Goal 2: **Material-Based Destruction Feedback**
**What it means:** Different materials respond differently to impact - clear visual/numeric feedback

**Material properties:**
```
STEEL (type=1):
  - Hardness: 2.0 (takes half damage)
  - Color: Blue-gray (0.6, 0.65, 0.7)
  - Destruction behavior: Small craters, less debris

CONCRETE (type=2):
  - Hardness: 1.0 (baseline)
  - Color: Warm gray (0.65, 0.6, 0.55)
  - Destruction behavior: Medium craters, lots of debris

WOOD (type=3):
  - Hardness: 0.5 (takes double damage)
  - Color: Brown (0.4, 0.3, 0.2)
  - Destruction behavior: Large craters, splinters everywhere
```

**Physics formula:**
```
effective_radius = base_radius / material_hardness
steel:    radius / 2.0 = smaller explosion
concrete: radius / 1.0 = normal explosion
wood:     radius / 0.5 = 2x larger explosion
```

**Success looks like:**
- Hit steel beam → small dent, 200 voxels destroyed
- Hit concrete wall → medium hole, 800 voxels destroyed
- Hit wooden structure → massive crater, 1600 voxels destroyed
- Player learns: "I need to hit weak points"

---

### Goal 3: **Debris Physics (Simple & Fast)**
**What it means:** Destroyed voxels become flying debris that bounces and settles

**Debris behavior:**
- Spawn at destroyed voxel position
- Initial velocity: away from impact center (radial explosion)
- Gravity: 9.8 m/s² downward
- Bounce once with elasticity 0.4 (loses 60% energy)
- Fade after 5 seconds (or convert to permanent debris)

**Physics implementation:**
```
For each destroyed voxel:
  debris_pos = voxel_position
  direction = normalize(voxel_pos - impact_center)
  debris_vel = direction * explosion_force * (1.0 - distance/radius)

Each frame:
  debris_vel.y -= 9.8 * dt
  debris_pos += debris_vel * dt

  if hits_ground:
    debris_vel.y *= -0.4  (bounce)
    debris_vel.xz *= 0.8  (friction)
```

**Success looks like:**
- Impact → debris flies in all directions
- Chunks arc through air realistically
- Bounce once or twice, then settle
- Creates satisfying "debris pile" on ground
- Runs at 60+ FPS with 5000 debris particles

---

### Goal 4: **Simple "Unsupported Falls" Check**
**What it means:** Voxels floating in air after explosion should fall down

**Detection method (SIMPLE VERSION):**
```
After destruction:
  For each voxel in affected region:
    if voxel is non-empty:
      count = count_neighbors_below(voxel)
      if count == 0:  // Nothing supporting it
        convert_to_debris(voxel)
```

**NOT doing (too complex for PATH 1):**
- Full connectivity/flood-fill (that's PATH 2)
- Horizontal support propagation
- Structural load calculations

**Just checking:** Is there literally ANY voxel directly below? No → fall.

**Success looks like:**
- Blast hole through middle of tower
- Top half of tower has nothing below it
- Top half converts to debris and falls
- Simple, fast, feels good enough

---

## 🔧 TECHNICAL IMPLEMENTATION PLAN

### Taichi Kernels to Write:

**1. Sphere Destruction Kernel**
```python
@ti.kernel
def destroy_sphere_at_impact(
    center_x: ti.f32, center_y: ti.f32, center_z: ti.f32,
    base_radius: ti.f32,
    projectile_speed: ti.f32
) -> ti.i32:
    """
    Destroy voxels in sphere, accounting for material hardness
    Returns: number of voxels destroyed
    """
    # Implementation: bounding box optimization + distance check
```

**2. Material Property Functions**
```python
@ti.func
def get_material_hardness(voxel_type: ti.i32) -> ti.f32:
    # Returns: 0.5 (wood), 1.0 (concrete), 2.0 (steel)

@ti.func
def get_material_weight(voxel_type: ti.i32) -> ti.f32:
    # Returns: kg per voxel (for future use)
```

**3. Simple Unsupported Check**
```python
@ti.kernel
def mark_unsupported_in_region(
    min_x: ti.i32, max_x: ti.i32,
    min_y: ti.i32, max_y: ti.i32,
    min_z: ti.i32, max_z: ti.i32
) -> ti.i32:
    """
    Check voxels in region, mark floating ones as DEBRIS
    Returns: count of newly-falling voxels
    """
```

**4. Debris Spawn Helper**
```python
@ti.kernel
def spawn_debris_particles(
    impact_center: ti.math.vec3,
    explosion_force: ti.f32
):
    """
    For each destroyed voxel, create debris particle
    with velocity away from impact center
    """
```

---

## 📊 TUNABLE PARAMETERS (We'll Adjust These)

### Impact Parameters:
```python
BASE_DESTRUCTION_RADIUS = 5.0        # voxels, baseline
VELOCITY_RADIUS_SCALE = 0.05         # radius += speed * this
MIN_DESTRUCTION_RADIUS = 2.0         # never smaller than this
MAX_DESTRUCTION_RADIUS = 15.0        # never larger than this

EXPLOSION_FORCE = 20.0               # m/s debris initial velocity
EXPLOSION_FORCE_FALLOFF = 0.8        # edge debris = force * this
```

### Material Parameters:
```python
STEEL_HARDNESS = 2.0
CONCRETE_HARDNESS = 1.0
WOOD_HARDNESS = 0.5

STEEL_WEIGHT = 7.8      # kg per voxel (for future)
CONCRETE_WEIGHT = 2.4
WOOD_WEIGHT = 0.6
```

### Debris Parameters:
```python
DEBRIS_GRAVITY = 9.8                 # m/s² (realistic)
DEBRIS_BOUNCE_ELASTICITY = 0.4       # loses 60% energy
DEBRIS_FRICTION = 0.8                # XZ velocity *= this on bounce
DEBRIS_LIFETIME = 5.0                # seconds before cleanup
DEBRIS_RENDER_RADIUS = 0.15          # voxels (half normal size)
```

### Physics Timestep:
```python
PHYSICS_DT = 0.01                    # 10ms = 100Hz physics
PHYSICS_SUBSTEPS = 1                 # can increase if unstable
```

---

## ✅ SUCCESS CRITERIA

### Milestone 1: Basic Impact (Day 1)
- [ ] Projectile hits voxel grid
- [ ] Sphere destruction kernel works
- [ ] Correct number of voxels destroyed (count matches expected)
- [ ] Hole appears in structure immediately
- [ ] No crashes, no NaN values
- [ ] Runs at 100+ FPS

### Milestone 2: Material Response (Day 2)
- [ ] Steel takes half damage (smaller crater)
- [ ] Wood takes double damage (bigger crater)
- [ ] Player can see/feel difference
- [ ] Cathedral steel beams are noticeably harder to destroy
- [ ] Impact radius formula: `radius = base_radius * (1 + speed/50) / hardness`

### Milestone 3: Debris Particles (Day 3)
- [ ] Destroyed voxels spawn debris
- [ ] Debris flies away from impact radially
- [ ] Debris falls with gravity
- [ ] Debris bounces once, then settles
- [ ] Can see 5000+ debris particles at 60 FPS
- [ ] Debris rendered smaller than voxels

### Milestone 4: Unsupported Falls (Day 4)
- [ ] Blast hole through tower base
- [ ] Top of tower has nothing below it
- [ ] Top converts to debris and falls
- [ ] Falling triggers more debris spawning
- [ ] Chain reaction works (falling voxels trigger neighbors)

### Milestone 5: Polish & Feel (Day 5)
- [ ] Impact feels punchy (screen shake?)
- [ ] Destruction radius feels appropriate
- [ ] Material differences are clear
- [ ] Fun to shoot different structures
- [ ] No physics explosions or crashes
- [ ] Consistent 60+ FPS

---

## 🎮 GAMEPLAY VALIDATION TESTS

### Test 1: The Tower Test
```
Build: Vertical tower, 10x10x50 voxels, all concrete
Shoot: Base with 100 m/s projectile
Expected:
  - 8 voxel radius destruction
  - ~1600 voxels destroyed at base
  - Top 30 voxels fall as debris
  - Debris shower on ground
  - Satisfying to watch
```

### Test 2: The Material Test
```
Build: Three towers side-by-side (steel, concrete, wood)
Shoot: All three with identical 80 m/s projectile
Expected:
  - Steel: Small dent, ~500 voxels destroyed
  - Concrete: Medium hole, ~1000 voxels destroyed
  - Wood: Large crater, ~2000 voxels destroyed
  - Player sees: "Wood is weak, steel is strong"
```

### Test 3: The Cathedral Test
```
Build: Your existing cathedral
Shoot: Various points with varying speeds
Expected:
  - Steel spires resist damage (dramatic effect)
  - Concrete nave crumbles nicely
  - Buttresses can be destroyed to trigger falls
  - Fun to find weak points
  - 60 FPS throughout
```

### Test 4: The Debris Test
```
Destroy: Large concrete wall (50x50x5)
Expected:
  - 12,500 voxels destroyed
  - 12,500 debris particles spawn
  - FPS stays above 45 (acceptable)
  - Debris fills screen dramatically
  - Bounces and settles within 3 seconds
```

---

## 🚫 OUT OF SCOPE (Not in PATH 1)

These are intentionally NOT included (save for PATH 2/3):

- ❌ Stress calculation (weight above / material strength)
- ❌ Flood-fill connectivity (ground-connected analysis)
- ❌ Load path propagation (forces through structure)
- ❌ Progressive failure (overstressed voxels breaking over time)
- ❌ Horizontal support (bridges, arches staying up)
- ❌ Material deformation (bending, cracking)
- ❌ Temperature/heat physics
- ❌ Penetration (projectile going through multiple voxels)

PATH 1 is pure **trigger-based destruction** - when projectile hits, things explode. Simple, fast, feels good.

---

## 📈 EXPECTED PERFORMANCE

### Performance Budget:
```
Voxel extraction:        0.5ms  (unchanged)
Rendering 50K voxels:    2.0ms  (unchanged)
Projectile physics:      0.1ms  (unchanged)
Impact destruction:      1.0ms  (new - sphere check + destroy)
Debris physics (5000):   2.0ms  (new - update positions)
Unsupported check:       0.5ms  (new - simple neighbor check)
---
Total frame time:        6.1ms  = 163 FPS
Target:                  16.6ms = 60 FPS minimum
Headroom:                10.5ms (plenty!)
```

### Worst Case (massive explosion):
```
20 voxel radius sphere
20,000 voxels destroyed
20,000 debris particles spawned

Impact destruction:      3.0ms  (larger sphere)
Debris physics:         8.0ms  (4x particles)
Total spike:            15ms   = 66 FPS (acceptable)
```

---

## 🔄 ITERATION PLAN

### Week 1 Workflow:
```
Day 1:
  - Code sphere destruction kernel
  - Test with fixed radius (no materials yet)
  - Verify voxel count matches expected

Day 2:
  - Add material hardness system
  - Test all three materials
  - Tune radius formula until it feels right

Day 3:
  - Add debris spawning
  - Add debris physics (gravity + bounce)
  - Tune explosion force

Day 4:
  - Add simple unsupported check
  - Test tower collapse
  - Fix any falling edge cases

Day 5:
  - Polish + playtest
  - Adjust ALL parameters based on feel
  - Fix any performance issues
```

### Parameter Tuning Sessions:
After each milestone, spend 30 minutes just shooting structures and adjusting:
- Is destruction radius too big/small?
- Do materials feel different enough?
- Is debris velocity too fast/slow?
- Does unsupported check work correctly?

**Goal: 80% code, 20% feel tuning**

---

## 🎯 DEFINITION OF DONE

PATH 1 is complete when:

1. ✅ Can shoot projectile at cathedral
2. ✅ Impact creates appropriately-sized explosion
3. ✅ Steel resists, concrete breaks, wood crumbles
4. ✅ Debris flies everywhere dramatically
5. ✅ Unsupported voxels fall down
6. ✅ Runs at 60+ FPS sustained
7. ✅ No crashes or physics explosions
8. ✅ **Fun to play with for 10+ minutes**

If #8 is true, we succeeded. If not, we tune parameters until it is.
