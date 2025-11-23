# Bombardier Beetle Implementation Plan

## Overview
Transform the Stag beetle into a Bombardier beetle with rear-mounted projectile weapon system.

### Key Features
- **Shotgun-style firing**: 5 pellets spread over 0.3s in a cone pattern
- **5 directional controls**: R+Y (forward), R (30° left), Y (30° right), V (rear-left), B (rear-right)
- **Visual ammo system**: Back stripe changes color to show charges (rows of voxels)
- **Charge-based cooldown**: 1 second per shot, charges regenerate every 3 seconds
- **Front-elevated body tilt**: Opposite of scorpion - front legs higher so rear-mounted weapon can fire forward
- **Spray surfing**: Firing applies recoil for mobility tricks

---

## Research Summary

### Biological Foundation
- **Real bombardier beetles**: Fire in rapid pulses (500-1000/sec) at 100°C, reaching 10 m/s
- **Range**: 20-30 cm effective range
- **Precision aiming**: 270° articulated spray nozzle - can aim over back, sides, or rear
- **Limited shots**: ~20-30 times before depleting chemical reserves
- **Key insight**: "Reverse turret" gameplay - precision directional control from rear-mounted weapon

### Recommended Mechanics (from research)
- **Projectile type**: Rapid-fire voxel stream (burst of 3-5 voxels)
- **Cooldown**: 0.25s recommended (we're using 1.0s for charge system)
- **Knockback formula**: `Force = Base Impulse × Velocity Multiplier × Mass Ratio × Hit Location Modifier`
- **Visual**: Bright orange/yellow voxels (RGB: 255, 180, 0) with trails
- **Sound**: Short "hiss" or "pssht" with pitch variation

---

## Implementation Steps

### 1. Replace Stag with Bombardier in Type System
**Files**: `beetle_physics.py`
**Location**: Lines 5971-6165 (type selection UI)

**Changes**:
- Update type rotation: rhino → **bombardier** → hercules → scorpion → atlas
- Replace all "stag" string references with "bombardier"
- Update button labels and cycling logic
- Current stag index in rotation: position 1 (after rhino)

**Code locations**:
- Blue beetle type button: Lines 5971-6026
- Red beetle type button: Lines 6111-6165
- Type storage: `blue_horn_type` and `red_horn_type` (lines 5057-5058)

---

### 2. Create Bombardier Beetle Geometry

#### 2A. Remove Stag Pincers
**Location**: Lines 1662-1723 (`generate_stag_pincers()`)
**Action**: Delete entire function or gate with `if horn_type != "bombardier"`

#### 2B. Create Rear Weapon Mount
**Location**: Lines 917-1046 (horn generation section)
**Add new case**:
```python
elif horn_type == "bombardier":
    # Rear-mounted nozzle/cannon
    weapon_x = -body_length  # Rear of abdomen
    weapon_y = back_body_height + 2  # Elevated above body
    weapon_z = 0  # Centered

    # Generate simple elevated nozzle (3x3x3 box/cylinder)
    # Pointing upward/rearward for clear firing line
    # Color: Use horn_tip color for the weapon
```

#### 2C. Front-Elevated Body Tilt
**Location**: Lines 869-915 (body generation)
**Reference**: Scorpion tilt at lines 869-876 (rear UP by 4 voxels)

**Add for bombardier** (OPPOSITE tilt - front UP):
```python
if horn_type == "bombardier":
    # Calculate front_progress (0.0 at rear, 1.0 at front)
    body_total_length = body_length + 4  # Total X span
    front_progress = (dx - (-body_length)) / body_total_length
    y_offset = int(front_progress * 4)  # Front 4 voxels higher
elif horn_type == "scorpion":
    # Existing scorpion code (rear UP)
    rear_progress = (dx - (-1)) / (-body_length - (-1))
    y_offset = int(rear_progress * 4)
else:
    y_offset = 0
```

**Apply to sections**:
- Abdomen (lines 841-884): Add `+ y_offset` to Y coordinates
- Thorax (lines 886-902): Add `+ y_offset` to Y coordinates
- Head (lines 904-915): Add `+ y_offset` to Y coordinates

**Alternative approach** (simpler):
- Increase front leg attachment Y from 1 to 4-5 (lines 1121-1152)
- Keeps body geometry unchanged, just raises front

---

### 3. Add Beetle Class Fields
**Location**: Lines 102-218 (Beetle class definition)

**Add after line 167** (`self.lift_cooldown = 0.0`):
```python
# Bombardier beetle firing system
self.fire_cooldown = 0.0          # Time until next shot allowed (seconds)
self.ammo_charges = 5             # Current charges available (max 5)
self.charge_recharge_timer = 0.0  # Countdown to next charge regeneration (seconds)
```

**Add timer decrement** in `update_timers()` section (around lines 216-218):
```python
# Update fire cooldown
if self.fire_cooldown > 0.0:
    self.fire_cooldown = max(0.0, self.fire_cooldown - dt)

# Update charge regeneration
if self.ammo_charges < 5:
    self.charge_recharge_timer -= dt
    if self.charge_recharge_timer <= 0.0:
        self.ammo_charges += 1
        self.charge_recharge_timer = 3.0  # 3 seconds per charge
```

---

### 4. Implement Projectile System

#### 4A. Projectile Spawn Function
**Location**: Add new function around line 4400 (near debris functions)

```python
@ti.kernel
def spawn_bombardier_shot(origin_x: ti.f32, origin_y: ti.f32, origin_z: ti.f32,
                          direction_x: ti.f32, direction_y: ti.f32, direction_z: ti.f32,
                          burst_index: ti.i32):
    """Spawn single projectile particle in bombardier shot burst

    Call this 5 times with burst_index 0-4 over 0.3 seconds for shotgun effect
    """
    import taichi as ti

    PROJECTILE_SPEED = 50.0      # Units per second
    CONE_SPREAD = 0.175          # ±10° in radians (0.175 rad = 10°)

    # Add random spread based on burst index for cone effect
    spread_factor = (burst_index - 2.0) / 2.0  # Maps 0-4 to -1.0 to +1.0
    spread_x = spread_factor * CONE_SPREAD * ti.random()
    spread_z = spread_factor * CONE_SPREAD * ti.random()

    # Calculate velocity with spread
    vx = direction_x * PROJECTILE_SPEED + spread_x * 10.0
    vy = direction_y * PROJECTILE_SPEED
    vz = direction_z * PROJECTILE_SPEED + spread_z * 10.0

    # Spawn particle using debris system
    idx = ti.atomic_add(simulation.num_debris[None], 1)
    if idx < simulation.MAX_DEBRIS:
        simulation.debris_pos[idx] = ti.math.vec3(origin_x, origin_y, origin_z)
        simulation.debris_vel[idx] = ti.math.vec3(vx, vy, vz)
        simulation.debris_material[idx] = ti.math.vec3(1.0, 0.7, 0.1)  # Yellow-orange
        simulation.debris_lifetime[idx] = 1.5  # 1.5 second lifetime
```

#### 4B. Burst Controller (Python side)
**Location**: Add after projectile spawn function

```python
def fire_bombardier_burst(beetle, direction_x, direction_y, direction_z):
    """Fire shotgun burst of 5 pellets over 0.3 seconds

    This should be called once, then subsequent particles spawned in update loop
    """
    import math

    # Calculate firing position (rear of beetle, elevated)
    facing_angle = beetle.rotation
    fire_offset_x = -math.cos(facing_angle) * 8  # 8 voxels behind center
    fire_offset_z = -math.sin(facing_angle) * 8

    fire_pos_x = beetle.x + fire_offset_x
    fire_pos_y = beetle.y + 5  # Elevated above body
    fire_pos_z = beetle.z + fire_offset_z

    # Spawn all 5 particles with slight delay between them (handled in main loop)
    for i in range(5):
        spawn_bombardier_shot(
            fire_pos_x, fire_pos_y, fire_pos_z,
            direction_x, direction_y, direction_z,
            i  # Burst index for spread calculation
        )
```

---

### 5. Firing Controls (Replace R/Y Horn Controls)
**Location**: Lines 5265-5356 (Red beetle control section)

**Replace existing U/O/N/M horn controls with R/Y/V/B firing**:

```python
# Bombardier beetle firing controls (Red beetle only)
if red_horn_type == "bombardier" and beetle_red.active:
    import math

    # Check fire cooldown and ammo
    if beetle_red.fire_cooldown <= 0.0 and beetle_red.ammo_charges > 0:
        r_pressed = window.is_pressed('r')
        y_pressed = window.is_pressed('y')
        v_pressed = window.is_pressed('v')
        b_pressed = window.is_pressed('b')

        firing_angle = None

        # Determine firing direction
        if r_pressed and y_pressed:
            # R+Y together: Fire forward (0°)
            firing_angle = 0.0
        elif r_pressed:
            # R alone: Fire 30° left from front
            firing_angle = math.radians(30)
        elif y_pressed:
            # Y alone: Fire 30° right from front
            firing_angle = math.radians(-30)
        elif v_pressed:
            # V: Fire 30° left of rear (150° left)
            firing_angle = math.radians(150)
        elif b_pressed:
            # B: Fire 30° right of rear (150° right)
            firing_angle = math.radians(-150)

        if firing_angle is not None:
            # Calculate world direction
            world_angle = beetle_red.rotation + firing_angle
            direction_x = math.cos(world_angle)
            direction_z = math.sin(world_angle)
            direction_y = 0.2  # Slight upward angle

            # Fire the shot
            fire_bombardier_burst(beetle_red, direction_x, direction_y, direction_z)

            # Apply spray surfing recoil (10% of projectile momentum)
            recoil_magnitude = 5.0  # Tunable
            beetle_red.vx -= direction_x * recoil_magnitude
            beetle_red.vz -= direction_z * recoil_magnitude

            # Consume resources
            beetle_red.ammo_charges -= 1
            beetle_red.fire_cooldown = 1.0  # 1 second cooldown

            # Start charge regeneration timer if needed
            if beetle_red.charge_recharge_timer <= 0.0:
                beetle_red.charge_recharge_timer = 3.0
```

---

### 6. Ammo Charge Visual System

#### 6A. Stripe-Based Ammo Indicator
**Concept**: Use existing back stripe voxels to show ammo charges
- 5 horizontal rows on beetle's rear abdomen
- Each row represents 1 charge
- Full charge: bright yellow (stripe color)
- Empty: dark red/brown (depleted color)

#### 6B. Implementation in Rendering
**Location**: Lines 2835+ (`place_animated_beetle_red()`)

**Modify stripe rendering logic**:
```python
# Inside beetle rendering, when drawing stripe voxels:
# Check if this is a bombardier beetle and adjust stripe color by Y level

if horn_type == "bombardier":
    # Divide abdomen rear into 5 horizontal zones (Y levels)
    # Each zone represents 1 charge
    # Calculate which charge zone this voxel belongs to

    stripe_zones = 5  # 5 charges
    zone_height = back_body_height / stripe_zones
    voxel_zone = int(local_y / zone_height)

    # Check if this zone has a charge
    if voxel_zone < beetle.ammo_charges:
        # Full charge: Use bright stripe color
        use_color = stripe_color
    else:
        # Empty charge: Use dark depleted color
        use_color = ti.math.vec3(0.3, 0.1, 0.1)  # Dark red-brown
else:
    # Normal stripe rendering for other beetle types
    use_color = stripe_color
```

**Alternative simpler approach**:
- Store charge colors in beetle class
- Update colors when charges change
- No conditional rendering needed

---

### 7. Projectile Collision & Knockback

#### 7A. Collision Detection Function
**Location**: Add new function around line 4400

```python
@ti.kernel
def check_projectile_beetle_collision(b1: Beetle, b2: Beetle):
    """Check if debris particles hit beetles and apply knockback

    Only checks particles that are projectiles (bright yellow color)
    """
    PROJECTILE_KNOCKBACK = 40.0   # Base knockback impulse (N·s)
    HIT_RADIUS = 4.0              # Collision radius (voxel units)
    UPWARD_KICK = 3.0             # Slight upward component

    for idx in range(simulation.num_debris[None]):
        if simulation.debris_lifetime[idx] > 0.0:
            # Check if this is a projectile (yellow/orange color)
            is_projectile = (simulation.debris_material[idx].x > 0.9 and
                           simulation.debris_material[idx].y > 0.6)

            if not is_projectile:
                continue

            px = simulation.debris_pos[idx].x
            py = simulation.debris_pos[idx].y
            pz = simulation.debris_pos[idx].z

            # Check hit on beetle 1
            if b1.active:
                dx = px - b1.x
                dz = pz - b1.z
                dist = ti.sqrt(dx*dx + dz*dz)

                if dist < HIT_RADIUS and abs(py - b1.y) < 6.0:  # Height check
                    # Calculate knockback direction
                    if dist > 0.1:
                        knockback_x = (dx / dist) * PROJECTILE_KNOCKBACK
                        knockback_z = (dz / dist) * PROJECTILE_KNOCKBACK
                    else:
                        knockback_x = 0.0
                        knockback_z = 0.0

                    # Apply linear knockback
                    b1.vx += knockback_x
                    b1.vz += knockback_z
                    b1.vy += UPWARD_KICK

                    # Apply angular knockback (spin based on hit location)
                    # Offset from center creates torque
                    angular_impulse = (dx * knockback_z - dz * knockback_x) * 0.3
                    b1.angular_velocity += angular_impulse / b1.moment_of_inertia

                    # Destroy projectile
                    simulation.debris_lifetime[idx] = 0.0

                    # Optional: Spawn impact particles
                    # spawn_impact_effect(px, py, pz)

            # Check hit on beetle 2 (same logic)
            if b2.active:
                dx = px - b2.x
                dz = pz - b2.z
                dist = ti.sqrt(dx*dx + dz*dz)

                if dist < HIT_RADIUS and abs(py - b2.y) < 6.0:
                    if dist > 0.1:
                        knockback_x = (dx / dist) * PROJECTILE_KNOCKBACK
                        knockback_z = (dz / dist) * PROJECTILE_KNOCKBACK
                    else:
                        knockback_x = 0.0
                        knockback_z = 0.0

                    b2.vx += knockback_x
                    b2.vz += knockback_z
                    b2.vy += UPWARD_KICK

                    angular_impulse = (dx * knockback_z - dz * knockback_x) * 0.3
                    b2.angular_velocity += angular_impulse / b2.moment_of_inertia

                    simulation.debris_lifetime[idx] = 0.0
```

#### 7B. Call Collision Check in Physics Loop
**Location**: Lines 5500-5600 (main physics update loop)

**Add after beetle-beetle collision**:
```python
# Check projectile collisions with beetles
if red_horn_type == "bombardier":
    check_projectile_beetle_collision(beetle_blue, beetle_red)
```

---

### 8. Spray Surfing (Recoil Mobility)
**Already implemented in Step 5 firing controls**

**Mechanism**:
```python
# Apply spray surfing recoil (10% of projectile momentum)
recoil_magnitude = 5.0  # Tunable - adjust for feel
beetle_red.vx -= direction_x * recoil_magnitude
beetle_red.vz -= direction_z * recoil_magnitude
```

**Effect**:
- Firing backward (V, B) propels beetle forward
- Firing forward (R+Y, R, Y) creates backward recoil
- Skilled players can chain shots for movement combos
- Recoil is 10% of knockback applied to enemy

**Balance notes**:
- Too strong: Becomes primary mobility tool (may be fun!)
- Too weak: Doesn't feel impactful
- Recommended: 5-10 units/sec impulse

---

### 9. Update Rendering Functions

#### 9A. Remove Stag Pincer Rendering
**Location**: `place_animated_beetle_red()` function

**Find stag pincer rendering calls**:
- Search for "stag" in rendering section
- Gate with `if horn_type != "bombardier"` or remove entirely

#### 9B. Add Bombardier Weapon Rendering
**Location**: Same function, horn rendering section

```python
elif horn_type == "bombardier":
    # Render rear-mounted weapon
    # Position at abdomen rear, elevated
    # Use horn_tip color
    # Simple geometry: 3x3x3 box or cylinder
```

#### 9C. Update Stripe Color Rendering
**Implemented in Step 6B**

---

### 10. Balance Tuning Values

#### Core Parameters
```python
# Cooldown & Charges
FIRE_COOLDOWN = 1.0                # Seconds between shots
CHARGE_REGEN_TIME = 3.0            # Seconds per charge regeneration
MAX_CHARGES = 5                    # Maximum ammo charges

# Projectile Properties
PARTICLES_PER_SHOT = 5             # Shotgun pellets
SPREAD_DURATION = 0.3              # Seconds to spawn all pellets
CONE_ANGLE = 10.0                  # Degrees of spread (±10°)
PROJECTILE_SPEED = 50.0            # Units per second
PROJECTILE_LIFETIME = 1.5          # Seconds before despawn

# Knockback
KNOCKBACK_PER_PARTICLE = 40.0      # N·s linear impulse
ANGULAR_KNOCKBACK_FACTOR = 0.3     # Torque multiplier
UPWARD_KICK = 3.0                  # Vertical component

# Spray Surfing
SELF_RECOIL_MAGNITUDE = 5.0        # Units/sec backward impulse
RECOIL_PERCENTAGE = 0.10           # 10% of projectile momentum

# Collision
HIT_RADIUS = 4.0                   # Voxel units collision detection
HEIGHT_TOLERANCE = 6.0             # Vertical collision tolerance
```

#### Tuning Guidelines
- **Too spammy**: Increase cooldown or reduce charges
- **Too weak**: Increase knockback or projectile speed
- **Too strong**: Reduce knockback or add charge cost
- **Spray surfing OP**: Reduce recoil magnitude
- **Hard to hit**: Increase hit radius or cone spread

---

## Files to Modify

### Primary File: `beetle_physics.py`
**Sections**:
1. Beetle class definition (lines 102-218)
   - Add fire_cooldown, ammo_charges, charge_recharge_timer fields
   - Add timer decrement logic

2. Geometry generation (lines 815-1287)
   - Remove/modify stag pincer function (lines 1662-1723)
   - Add bombardier weapon geometry (lines 917-1046)
   - Add front body tilt (lines 869-915)

3. Physics functions (around line 4400)
   - Add spawn_bombardier_shot() kernel
   - Add fire_bombardier_burst() function
   - Add check_projectile_beetle_collision() kernel

4. Control input (lines 5265-5356)
   - Replace R/Y horn controls with firing logic
   - Add V/B rear firing controls
   - Implement spray surfing recoil

5. Physics update loop (lines 5500-5600)
   - Call projectile collision check

6. Rendering (lines 2835+)
   - Modify place_animated_beetle_red()
   - Update stripe color based on charges
   - Remove stag pincer rendering
   - Add bombardier weapon rendering

7. Type selection UI (lines 5971-6165)
   - Replace "stag" with "bombardier"
   - Update button labels

### Secondary File: `simulation.py`
**Likely no changes needed** - debris system already supports projectiles

**Potential additions**:
- New voxel type for projectiles (optional)
- Currently using debris system which is sufficient

---

## Technical Implementation Notes

### Reusing Debris System
- **MAX_DEBRIS**: 20,000 particles available
- **Fields**: position, velocity, material (RGB color), lifetime
- **Update**: Handled by `update_debris_particles()` (lines 4439-4459)
- **Cleanup**: `cleanup_dead_debris()` compacts array (lines 4462-4476)
- **Advantage**: No new system needed, GPU-optimized

### Projectile Identification
- Use **color** to distinguish projectiles from debris
- Projectiles: RGB(1.0, 0.7, 0.1) - bright yellow/orange
- Debris: Varies (beetle body colors)
- Collision check filters by color

### Collision Detection Approach
- **Simple sphere-vs-point**: Fast, works for most cases
- **Distance check**: `sqrt(dx² + dz²) < HIT_RADIUS`
- **Height tolerance**: `abs(py - beetle.y) < 6.0`
- **No raycast needed**: Projectiles move slow enough

### Front Tilt Implementation
- **Similar to scorpion**: Uses Y offset based on X position
- **Opposite direction**: Front up instead of rear up
- **Formula**: `y_offset = int(front_progress * 4)`
- **Alternative**: Lengthen front legs instead

### Performance Considerations
- **Particle count**: 5 pellets × 1.5sec lifetime × 1sec cooldown = ~8 particles/beetle max
- **Collision checks**: O(particles × beetles) = O(20 × 2) = 40 checks/frame
- **Negligible impact**: Well within Taichi GPU capabilities

---

## Testing Checklist

### Functionality Tests
- [ ] Bombardier appears in type selection UI (both blue and red)
- [ ] Body tilts correctly (front elevated, rear low)
- [ ] Rear weapon geometry renders properly
- [ ] R+Y fires forward
- [ ] R fires 30° left
- [ ] Y fires 30° right
- [ ] V fires rear-left
- [ ] B fires rear-right
- [ ] Shotgun spread creates 5 particles in cone
- [ ] Particles travel in correct directions
- [ ] Particles despawn after lifetime

### Ammo System Tests
- [ ] Ammo charges start at 5
- [ ] Firing consumes 1 charge
- [ ] Cannot fire with 0 charges
- [ ] Charges regenerate every 3 seconds
- [ ] Visual stripe indicator updates correctly
- [ ] Full charges show bright yellow
- [ ] Empty charges show dark color

### Combat Tests
- [ ] Projectiles hit enemy beetle
- [ ] Knockback applies on hit
- [ ] Knockback creates linear motion
- [ ] Knockback creates angular spin
- [ ] Multiple hits stack knockback
- [ ] Projectiles despawn on hit
- [ ] Hit detection works at various ranges

### Spray Surfing Tests
- [ ] Firing creates recoil on shooter
- [ ] Shooting backward moves beetle forward
- [ ] Shooting forward creates backward push
- [ ] Recoil magnitude feels balanced
- [ ] Can chain shots for movement

### Balance Tests
- [ ] Cooldown prevents spam (1 second feels right)
- [ ] Knockback strength feels satisfying
- [ ] Not overpowered vs. horn beetles
- [ ] Spray surfing useful but not broken
- [ ] Charge regeneration timing balanced

### Edge Cases
- [ ] Works when beetle is airborne
- [ ] Works when beetle is tilted
- [ ] Handles max particle limit gracefully
- [ ] No crashes when rapid firing
- [ ] Proper cleanup when beetle dies

---

## Future Enhancement Ideas

### From Research (Not in Initial Implementation)

1. **Chemical Puddles** (Area Denial)
   - Missed shots leave hazards on ground
   - 5 second duration
   - Slow + damage effect
   - Affects both beetles (friendly fire)

2. **Overheat Mechanic** (Resource Management)
   - Rapid firing builds heat meter
   - At 100%: 2 second cooldown penalty
   - Risk/reward: Burst vs. sustained fire
   - Visual: Beetle glows with heat

3. **Directional Tells** (Mindgames)
   - Rear nozzle rotates before firing
   - Creates prediction gameplay
   - Can feint (cancel rotation)
   - Adds depth without complexity

4. **Impact Effects** (Polish)
   - Particle explosion on hit
   - Screen shake on impact
   - Sound effects with pitch variation
   - Muzzle flash when firing

5. **Charge Shot Variant** (Alternative Mode)
   - Hold button to charge (0.3s)
   - Releases larger, stronger burst
   - Consumes 2 charges instead of 1
   - Creates skill-based timing

---

## References

### Research Documents
- **Lateral Research Report**: Comprehensive bombardier beetle biology, game design inspiration, physics formulas
- **Planning Agent Report**: Current codebase analysis, implementation locations, technical details

### Key Code Locations
- **Beetle class**: beetle_physics.py lines 102-218
- **Debris system**: simulation.py lines 11-16, beetle_physics.py lines 4439-4523
- **Scorpion tilt reference**: beetle_physics.py lines 869-876
- **Control input**: beetle_physics.py lines 5116-5356
- **Type selection**: beetle_physics.py lines 5971-6165

### Biological References
- Bombardier beetles fire in pulses (500-1000/sec)
- 270° articulated spray nozzle
- 20-30 shots before depletion
- Chemical reaction reaches 100°C
- Range: 20-30 cm effective

---

## Design Philosophy

### Core Identity
"Precision rear-mounted artillery with high skill ceiling"

### Gameplay Pillars
1. **Directional mastery**: 5 firing angles reward spatial awareness
2. **Resource management**: Charge system creates strategic decisions
3. **Mobility tech**: Spray surfing separates casual from advanced players
4. **Risk/reward**: Close range more effective but more dangerous

### Differentiation from Horn Beetles
- **Range**: Bombardier fights at distance, horn beetles close
- **Positioning**: Bombardier rear-facing, horn beetles front-facing
- **Timing**: Bombardier cooldown-based, horn beetles contact-based
- **Skill floor**: Bombardier easier to land hits (projectiles)
- **Skill ceiling**: Bombardier spray surfing tech, horn beetles lift timing

### Balance Goals
- **Not overpowered**: Cooldown + charges prevent spam
- **Not underpowered**: Knockback feels impactful
- **Distinct playstyle**: Encourages kiting and zoning
- **Fun to play**: Satisfying feedback, skill expression
- **Fun to fight**: Counterplay exists (rush down, dodge)

---

## Implementation Estimate

### Time Breakdown
1. Type replacement & UI: **15 minutes**
2. Geometry (remove stag, add weapon, tilt): **45 minutes**
3. Beetle class fields & timers: **10 minutes**
4. Projectile spawn functions: **30 minutes**
5. Firing controls: **30 minutes**
6. Ammo visual system: **30 minutes**
7. Collision & knockback: **45 minutes**
8. Testing & tuning: **60 minutes**

**Total estimated time**: ~4 hours for full implementation

### Incremental Milestones
1. **Milestone 1** (30 min): Bombardier type exists, basic geometry renders
2. **Milestone 2** (1 hour): Can fire projectiles in one direction
3. **Milestone 3** (1.5 hours): All 5 directions work, ammo system functional
4. **Milestone 4** (2.5 hours): Collision and knockback working
5. **Milestone 5** (3 hours): Spray surfing, visual polish
6. **Milestone 6** (4 hours): Balanced and play-tested

---

## Success Criteria

### Must Have (MVP)
- [x] Bombardier replaces stag in type rotation
- [x] Front-elevated body tilt
- [x] Can fire projectiles in all 5 directions
- [x] Shotgun spread (5 pellets)
- [x] Charge-based ammo system
- [x] Visual stripe charge indicator
- [x] Projectiles apply knockback on hit
- [x] Spray surfing recoil

### Nice to Have (Polish)
- [ ] Impact particle effects
- [ ] Sound effects (firing, impact)
- [ ] Muzzle flash visual
- [ ] Projectile trail effects
- [ ] Camera shake on impact

### Future Features (V2)
- [ ] Chemical puddles
- [ ] Overheat mechanic
- [ ] Directional tells/feints
- [ ] Charge shot variant

---

## Known Limitations & Risks

### Performance
- **Risk**: Too many particles lag game
- **Mitigation**: Limited to 5 particles/shot, 1.5s lifetime, shared 20k pool

### Balance
- **Risk**: Ranged weapon overpowered vs melee
- **Mitigation**: Cooldown, charge limits, moderate knockback

### Usability
- **Risk**: 5 directional controls confusing
- **Mitigation**: Clear visual indicators, practice mode

### Technical
- **Risk**: Collision detection misses fast projectiles
- **Mitigation**: Projectile speed moderate (50 units/sec), large hit radius

### Gameplay
- **Risk**: Spray surfing becomes exploitable
- **Mitigation**: Tunable recoil magnitude, costs ammo

---

## Conclusion

This plan provides a complete roadmap for implementing a bombardier beetle that:
- **Respects biology**: Rear-mounted, pulsed firing, limited shots
- **Creates unique gameplay**: Directional control, charge management, spray surfing
- **Balances with existing beetles**: Different playstyle, clear strengths/weaknesses
- **Leverages existing systems**: Debris particles, impulse physics, cooldown timers
- **Achievable scope**: ~4 hours implementation, incremental milestones

The design is backed by extensive research into real beetle biology, game design patterns from fighting games/shooters, and thorough codebase analysis. All systems needed already exist - this is primarily integration work rather than building from scratch.

Ready to implement when approved!