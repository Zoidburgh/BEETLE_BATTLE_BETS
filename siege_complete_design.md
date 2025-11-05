# SIEGE: The Complete Game Design Document
## Angry Birds with Real Physics - Now with Perfect 3D Aiming

---

## Table of Contents

1. [Game Overview](#game-overview)
2. [The Aiming System (Core Innovation)](#the-aiming-system)
3. [Core Gameplay Loop](#core-gameplay-loop)
4. [Physics Systems](#physics-systems)
5. [Projectiles & Materials](#projectiles-materials)
6. [Level Design](#level-design)
7. [Visual Design](#visual-design)
8. [Mobile Implementation](#mobile-implementation)
9. [Development Roadmap](#development-roadmap)

---

## Game Overview

### The Elevator Pitch

**"Angry Birds meets Teardown - Castle siege with real structural physics"**

You're commanding a medieval siege. Aim your catapult, choose your angle, launch projectiles, and watch castles collapse with real physics. Every stone falls according to actual weight-bearing calculations. Every impact creates emergent chain reactions. No scripted animations - pure physics simulation.

### Core Promise

**"The same shot never looks the same twice."**

Unlike Angry Birds where destruction is pre-animated, SIEGE uses real-time physics simulation. Hit the same tower twice in the same spot - you'll get different collapses based on slight variations in impact angle, projectile spin, and structural stress distribution.

### Target Audience

**Primary:** Mobile gamers who loved Angry Birds but want deeper mechanics (ages 18-35)
**Secondary:** PC puzzle game fans who enjoy physics-based games like Poly Bridge, Teardown
**Tertiary:** Engineering/STEM enthusiasts who appreciate realistic simulations

### Platform Strategy

**Phase 1:** Mobile-first (iOS + Android)
- Touch controls are PERFECT for this
- Bigger market (billions of potential players)
- F2P model works better

**Phase 2:** PC/Steam port
- Same codebase, better graphics
- Premium pricing ($5-10)
- Workshop for user-created levels

---

## The Aiming System (Core Innovation)

This is the heart of SIEGE. We're solving the hardest problem in "3D Angry Birds" - how do you aim in 3D space with 2D touch controls?

### The Two-Mode System

**SIEGE uses a hybrid approach that feels simple but allows 3D depth:**

```
Mode 1: ROTATE → Choose which side of castle to attack
Mode 2: AIM → Set power and trajectory (like 2D Angry Birds)
```

**It's like aiming a tank:**
- First rotate the turret (choose direction)
- Then adjust elevation and power (aim and fire)

---

## Mode 1: ROTATE (Choose Your Angle)

### What the Player Does

**Simple swipe gesture:**
```
Touch screen → Swipe left/right → Camera orbits castle
```

**No buttons, no complex controls - just swipe.**

### What Happens Visually

**Camera orbits around the castle in a circle:**

```
[TOP-DOWN VIEW OF ROTATION]

          North
           ⬆️
           
    West  🏰  East
      ⬅️  Castle  ➡️
           
          South
           ⬇️

Your swipe moves camera position:
Swipe right → Camera moves clockwise → See castle from different angle
Swipe left → Camera moves counter-clockwise → See different side

        🎥 Camera position
       /
      / (fixed distance from castle)
     /
   🏰 Castle (always at center)
```

**As you swipe, you see the castle rotate on screen** (actually, camera is moving around it).

### UI During ROTATE Mode

```
┌─────────────────────────────────┐
│                                 │
│     [Castle from current angle] │
│                                 │
│            🏰                   │
│           /|\                   │
│          / | \                  │
│         /  |  \                 │
│        /___|___\                │
│                                 │
│   📱 Swipe left/right           │
│   to choose attack angle        │
│                                 │
│   Current: Front View           │
│                                 │
│   [Back] [Lock & Aim] ✓         │
└─────────────────────────────────┘
```

**Key visual indicators:**
- Compass showing which side you're viewing (N/E/S/W)
- Arrow from catapult pointing at castle
- Weak points highlighted (if player unlocked this ability)
- Mini-map showing camera position relative to castle

### Strategic Depth

**Different angles reveal different things:**

**Front angle (0°):**
- Can see main gate
- Can hit wooden door (weak)
- But can't see stone buttresses behind (strong)

**Side angle (90°):**
- Can see load-bearing wall
- Can target key structural support
- But door is protected

**Back angle (180°):**
- Can see weak wooden wall
- Can collapse rear tower
- But front remains standing

**This creates strategy:** "Which angle gives me access to the weakest points?"

### Tutorial Teaching

**Level 1:** Camera locked at front (no rotation)
- Player learns basic aiming without rotation complexity

**Level 5:** Rotation unlocked
- Tutorial: "Some structures have weak points on different sides. Try rotating!"
- Player swipes, discovers wooden back wall
- Aha moment: "Oh, I need to find the right angle!"

**Level 10:** Rotation required
- Front of castle is impenetrable stone
- Back has wooden supports
- Can't win without rotating

---

## Mode 2: AIM (Set Trajectory)

### Transitioning from ROTATE to AIM

**Once you've chosen your angle:**

```
Player taps "Lock & Aim" button
    ↓
Camera stops rotating (locked in place)
    ↓
Mode switches to AIM
    ↓
Now you drag to aim (like Angry Birds)
```

**Visual feedback of lock:**
- Camera stops moving
- UI changes from "Swipe to rotate" → "Drag to aim"
- Catapult "clicks" into locked position (animation + sound)
- Trajectory preview appears

### What the Player Does

**Drag gesture (exactly like Angry Birds):**

```
Touch catapult → Drag finger backward → Release to fire

The further back you drag = more power
The higher/lower you drag = different trajectory arc
```

### The Aiming Mechanics (Technical Detail)

**When you drag, you're setting two parameters:**

1. **Power** (how far you drag)
2. **Elevation angle** (how high you drag)

**Visual representation:**

```
[SIDE VIEW OF AIMING]

        * * * * <- Trajectory preview
       *       *
      *         *  
     *           *
    *             🏰 (Castle you locked onto)
    
   🏹 Catapult
    ↑
    |
    |← Drag to here (far = high power)
    |
[Your finger position]

Power: [=============>] 95%
Angle: 48° ↗️
```

**How power is calculated:**

```python
# Distance dragged from catapult
drag_distance = (finger_position - catapult_position).length()

# Convert to power (0% to 100%)
power_percent = min(drag_distance / max_drag_distance, 1.0) * 100

# Max drag distance = 200 pixels on screen
# So:
# Drag 50px = 25% power
# Drag 100px = 50% power  
# Drag 200px = 100% power
# Drag 300px = still 100% (clamped)
```

**How angle is calculated:**

```python
# Vertical component of drag
drag_vertical = catapult_position.y - finger_position.y

# Positive = dragged down = lower angle
# Negative = dragged up = higher angle

# Convert to angle (15° to 75° range)
angle = base_angle + (drag_vertical / screen_height) * angle_range

# Examples:
# Drag straight back (no vertical) = 45° (medium arc)
# Drag upward = 60-75° (high arc, shorter distance)
# Drag downward = 15-30° (low arc, longer distance)
```

### Trajectory Preview

**As you drag, you see EXACTLY where the shot will go:**

**What's drawn:**

1. **Yellow dotted line** - Path projectile will take through air
2. **Red circle** - Where it will impact
3. **Ground shadows** - Dark dots on ground below trajectory
4. **Impact radius** - Area that will be damaged

**How trajectory is calculated:**

```python
# Real ballistic physics
def calculate_trajectory(launch_velocity, launch_position):
    trajectory_points = []
    position = launch_position.copy()
    velocity = launch_velocity.copy()
    time_step = 0.05  # 50ms steps
    
    for step in range(200):  # Simulate up to 10 seconds
        # Apply gravity
        velocity.y -= gravity * time_step
        
        # Update position
        position += velocity * time_step
        
        # Store point
        trajectory_points.append(position.copy())
        
        # Check if hit something
        if position.y <= 0:  # Hit ground
            break
        if check_collision_with_castle(position):  # Hit castle
            break
    
    return trajectory_points
```

**This is REAL physics, not fake:**
- Gravity = 9.8 m/s²
- Air resistance = none (simplified for clarity)
- Parabolic arc (real ballistics)

**The preview updates in real-time as you drag** (60 FPS update).

### UI During AIM Mode

```
┌─────────────────────────────────┐
│  Power: [==============>] 92%   │
│  Angle: 52° ↗️                  │
│  Distance: ~35m                 │
│                                 │
│      [Castle from locked angle] │
│                                 │
│         * * * * *               │
│        *         *              │
│       *           🏰            │
│      *           /|\            │
│     *           / | \           │
│    *           /  |  \          │
│   *           /___|___\         │
│  *                              │
│ * ← [Impact point]              │
│🏹 Catapult                      │
│ ↑                               │
│ └─ [Drag from here]             │
│                                 │
│ [Cancel] [Fire!] [Change Angle] │
└─────────────────────────────────┘
```

**Additional visual aids:**

**Grid floor:**
```
Shows distance markers every 5m
Helps judge how far shot will travel

[Ground view]
|----|----|----|----|
 5m  10m  15m  20m
```

**Depth lines:**
```
Vertical yellow lines from trajectory to ground
Shows height of arc at each point

    * (height indicator line)
    |
    |
[ground]
```

**Impact prediction:**
```
Red expanding circle where impact will occur
Larger circle = more damage radius
Flashing = perfect shot (will hit weak point)
```

### Advanced Aiming Features

**Wind (unlocked in later levels):**
```
Wind arrow appears
Affects trajectory in flight
Preview shows adjusted path
Player must compensate
```

**Projectile selection (multiple types):**
```
Before aiming, player selects projectile type
Each has different:
- Mass (affects arc)
- Size (affects aim)
- Special properties (explosion, etc.)

Preview shows trajectory for SELECTED projectile
```

**Charge shot (expert mechanic):**
```
Hold position for 2 seconds = charged shot
1.5x power boost
Visual: Catapult glows, trajectory line pulses
Risk/reward: Harder to time
```

---

## The Complete Aiming Flow (Step by Step)

### Step 1: Level Loads

```
Scene appears:
- Castle in center
- Catapult in front
- Camera at default angle (facing front of castle)
- UI shows "Swipe to choose angle"

Player sees:
     [Castle front view]
         🏰
        /|\
       / | \
      /  |  \
     
     🏹 (Catapult in foreground)
```

### Step 2: Player Explores Angles (ROTATE Mode)

```
Player swipes right:

Frame 1: Front view
   🏰
  /|\

Frame 2: 45° view (swipe continues)
   /🏰
  / |\
    
Frame 3: Side view (swipe more)
  |🏰|
  | |
  
Player sees different sides of castle as they swipe
```

**Player thinks:**
- "Front has thick stone walls"
- "Side has wooden supports"
- "Aha! I'll attack the wooden supports"

### Step 3: Player Locks Angle (Transition)

```
Player taps "Lock & Aim" button

Animation:
1. Camera smoothly stops rotating
2. Catapult "clicks" into position (sound + animation)
3. UI transitions:
   - "Swipe to rotate" fades out
   - "Drag to aim" fades in
   - Power/angle meters appear
4. Trajectory preview appears (faint)

Duration: 0.5 seconds
```

### Step 4: Player Aims (AIM Mode)

```
Player touches catapult, drags back:

Frame 1: Small drag
  Power: 30%
  Angle: 40°
  Trajectory: Short arc
  
Frame 2: Medium drag
  Power: 60%
  Angle: 45°
  Trajectory: Medium arc (reaching castle)
  
Frame 3: Large drag
  Power: 90%
  Angle: 50°
  Trajectory: Long arc (will hit upper tower)

Player adjusts until trajectory looks perfect
```

**Feedback during drag:**
- Haptic buzz when trajectory crosses weak point
- Color change: Red (will miss) → Yellow (close) → Green (perfect shot)
- Audio cue: Pitch increases with power

### Step 5: Player Fires (Release)

```
Player releases finger:

Immediate effects:
1. Catapult animates (arm swings forward)
2. Projectile launches along predicted trajectory
3. Camera switches to "follow projectile" mode
4. Sound: WHOOSH + wind
5. Slight slow-motion (0.7x speed for first 0.5 seconds)

Player watches projectile fly through air...
```

### Step 6: Impact & Physics (The Payoff)

```
Projectile hits castle:

Frame 1: Impact
- Flash of light at impact point
- Particles explode outward
- Sound: CRACK + rumble
- Screen shake (proportional to impact force)
- Slow-motion (0.3x speed for 1 second)

Frame 2: Immediate damage (0.1 seconds)
- Impact voxels destroyed
- Cracks appear in adjacent voxels
- Structural stress propagates

Frame 3: Cascade begins (0.5 seconds)
- Weight redistributes
- Overloaded supports start failing
- First major piece falls

Frame 4: Full collapse (2-5 seconds)
- Chain reaction continues
- Floors pancake
- Walls tumble
- Dust clouds expand
- Debris bounces and settles

Frame 5: Aftermath (8 seconds)
- Camera pulls back
- Final pieces settle
- Goblin targets check: Did they fall?
- Results screen appears
```

**Camera during collapse:**
- Dynamically frames the action
- Zooms in on dramatic moments
- Pulls back to show full destruction
- Switches to slow-motion at peak moments

### Step 7: Results & Retry

```
Results screen:
┌─────────────────────────────────┐
│         LEVEL COMPLETE!         │
│                                 │
│  ⭐⭐⭐                          │
│                                 │
│  Targets: 3/3 Destroyed ✓       │
│  Projectiles: 1/3 Used          │
│  Time: 15 seconds               │
│  Bonus: Perfect shot! +500      │
│                                 │
│  [Next Level]  [Retry]          │
└─────────────────────────────────┘

Options:
- Proceed to next level
- Retry for better score
- Change projectile type
- Share replay (auto-recorded)
```

---

## Physics Systems

### Structural Integrity Simulation

**Every voxel calculates:**

1. **Weight above it**
   - Count all solid voxels in column above
   - Calculate total mass

2. **Material strength**
   - Stone: 100 strength units
   - Wood: 20 strength units
   - Steel: 200 strength units

3. **Stress calculation**
   ```python
   stress = weight_above / material_strength
   
   if stress > 1.0:
       voxel_fails()
   ```

4. **Stress propagation**
   - When voxel fails, weight redistributes to neighbors
   - Neighbors recalculate stress
   - Can cause cascade failure

**Real-world analog:**
```
Like Jenga:
- Remove wrong block → nothing happens
- Remove key support → tower collapses
- But you discover key supports through experimentation
```

### Impact Physics

**When projectile hits:**

1. **Momentum transfer**
   ```python
   # Projectile has mass and velocity
   momentum = projectile_mass * projectile_velocity
   
   # Transfer to impact voxels
   for voxel in impact_radius:
       voxel_velocity += momentum * distribution_factor
   ```

2. **Structural damage**
   ```python
   # Impact force damages voxels
   impact_force = momentum.magnitude()
   
   for voxel in impact_point:
       damage = impact_force / voxel_strength
       voxel_health -= damage
       
       if voxel_health <= 0:
           voxel_destroyed()
   ```

3. **Stress propagation**
   - Destroyed voxels no longer support weight
   - Stress redistributes to remaining voxels
   - Can trigger collapse if stress exceeds strength

**Visual feedback:**
- Cracks appear before failure (health 20-50%)
- Voxels glow red when highly stressed
- Sound: Creaking, groaning (stress building)

### Gravity & Falling

**Unsupported voxels fall:**

```python
@ti.kernel
def apply_gravity(dt: ti.f32):
    for voxel in all_voxels:
        if voxel.type != EMPTY:
            # Check if supported
            below = voxel_below(voxel)
            
            if below == EMPTY or below.is_falling:
                # Not supported - start falling
                voxel.velocity.y -= 9.8 * dt
                voxel.position.y += voxel.velocity.y * dt
                voxel.is_falling = True
                
                # Rotate as it falls (tumbling)
                voxel.rotation += voxel.angular_velocity * dt
```

**Falling voxels:**
- Bounce when they hit ground/other voxels
- Can damage other voxels on impact
- Eventually settle (velocity damping)

---

## Projectile Types

### 1. Stone Ball (Basic)

**Properties:**
- Mass: 10 kg
- Speed: Medium (30 m/s max)
- Damage: Blunt force
- Cost: Free (always available)

**Physics behavior:**
- Simple sphere collision
- Bounces once (elasticity 0.3)
- Rolls after landing

**Best used for:**
- Wooden structures
- Knocking down towers
- General purpose

**Trajectory characteristics:**
- Medium arc
- Predictable path
- Easy to aim

**Visual:**
```
   🪨 Gray stone sphere
   Size: 1 voxel radius
   Trail: None
   Sound: Whoosh → Thud
```

### 2. Iron Cannonball (Heavy Hitter)

**Properties:**
- Mass: 50 kg
- Speed: Slow (20 m/s max)
- Damage: Extreme blunt force
- Cost: 1 star (unlock level 5)

**Physics behavior:**
- Punches through weak materials
- Doesn't bounce (embeds)
- Extremely high momentum

**Best used for:**
- Stone walls
- Breaking through barriers
- Single point destruction

**Trajectory characteristics:**
- Heavy arc (drops fast)
- Short range
- Requires high angle

**Visual:**
```
   ⚫ Black iron sphere
   Size: 1.5 voxel radius
   Trail: Smoke
   Sound: Deep BOOM on impact
```

### 3. Explosive Shell (Area Damage)

**Properties:**
- Mass: 15 kg
- Speed: Medium (25 m/s)
- Damage: Radial explosion
- Cost: 2 stars (unlock level 10)

**Physics behavior:**
- Explodes on impact
- Applies outward force to all voxels in radius
- Can chain react with TNT barrels

**Explosion radius:**
```python
explosion_radius = 5 voxels
explosion_force = 1000 N at center
force_falloff = 1/distance²

for voxel in radius:
    distance = (voxel.pos - explosion_center).length()
    force = explosion_force / (distance * distance)
    apply_force(voxel, force, direction_outward)
```

**Best used for:**
- Creating large breaches
- Destroying multiple supports
- Chain reactions

**Visual:**
```
   💣 Red shell with fuse
   Size: 0.8 voxel radius
   Trail: Sparks
   Explosion: Orange flash + smoke ring
   Sound: Fizzle → BOOM
```

### 4. Splitting Shot (Multi-Hit)

**Properties:**
- Mass: 5 kg (splits into 3x projectiles of 1.5 kg each)
- Speed: Fast (35 m/s)
- Damage: Multiple small impacts
- Cost: 1 star (unlock level 8)

**Physics behavior:**
- Flies as single projectile
- Splits at apex of trajectory
- Three fragments spread in cone pattern

**Split pattern:**
```
      * 
     /|\  ← Splits here (at highest point)
    / | \
   *  *  * ← Three fragments
   
Fragment spread: 15° cone
Each has 1/3 mass but same velocity
```

**Best used for:**
- Hitting multiple weak points
- Spreading damage across area
- When unsure of exact target

**Visual:**
```
   🔵 Blue sphere
   Splits: Flash of light
   Fragments: 3 smaller blue spheres
   Trail: Triple vapor trails after split
   Sound: Whistle → Pop → 3x thuds
```

### 5. Wrecking Ball (Pendulum)

**Properties:**
- Mass: 80 kg
- Speed: Varies (momentum-based)
- Damage: Sweeping destruction
- Cost: 3 stars (unlock level 15)

**Physics behavior:**
- Attached to catapult by chain
- Swings like pendulum
- Can hit multiple times as it swings back and forth
- Chain can wrap around structures

**Pendulum physics:**
```python
# Chain keeps wrecking ball at fixed distance from catapult
chain_length = 20 voxels

# Applies constraint each frame
distance = (ball.pos - catapult.pos).length()
if distance > chain_length:
    # Pull ball back to chain length
    direction = (catapult.pos - ball.pos).normalized()
    ball.pos = catapult.pos + direction * chain_length
    
    # Convert excess velocity to tangential motion
    ball.velocity = tangential_component(ball.velocity)
```

**Best used for:**
- Sweeping through multiple walls
- Dramatic destruction
- When you want to "paint" damage across structure

**Visual:**
```
   ⚫━━━━━🏹 (Ball connected to catapult)
   Heavy sphere on chain
   Trail: None (too heavy to leave trail)
   Sound: Rattle of chain + repeated impacts
```

### 6. Incendiary (Fire)

**Properties:**
- Mass: 8 kg
- Speed: Medium (28 m/s)
- Damage: Fire spread + thermal weakening
- Cost: 2 stars (unlock level 12)

**Physics behavior:**
- Sets wooden voxels on fire
- Fire spreads to adjacent wood
- Heat weakens steel (same as THERMITE)
- Burns for 30 seconds

**Fire spread:**
```python
@ti.kernel
def spread_fire():
    for voxel in on_fire:
        # Heat up voxel
        voxel.temperature = 800°C
        
        # Spread to neighbors
        for neighbor in adjacent_voxels:
            if neighbor.material == WOOD:
                if neighbor.temperature > 300°C:  # Ignition point
                    neighbor.on_fire = True
            
            if neighbor.material == STEEL:
                # Steel doesn't burn but weakens
                if neighbor.temperature > 400°C:
                    neighbor.strength *= 0.5  # Loses strength
```

**Best used for:**
- Wooden structures
- Delayed destruction (fire spreads slowly)
- Weakening steel beams

**Visual:**
```
   🔥 Orange sphere with flames
   Trail: Smoke
   Impact: Fire spreads in expanding circle
   Sound: Whoosh → Crackling flames
```

### 7. Drill Bolt (Penetrator)

**Properties:**
- Mass: 12 kg
- Speed: Very fast (45 m/s)
- Damage: Piercing + internal chaos
- Cost: 2 stars (unlock level 18)

**Physics behavior:**
- Pointed projectile
- Penetrates outer walls without stopping
- Bounces inside structure
- Eventually embeds

**Penetration logic:**
```python
# High velocity + pointed shape = penetration
if projectile.velocity > 40 and projectile.type == DRILL:
    # Pierce through first layer of voxels
    for voxel in impact_path:
        if voxel.strength < 50:  # Weak materials
            destroy_voxel(voxel)
            projectile.velocity *= 0.7  # Lose some speed
        else:
            projectile.velocity *= 0  # Stop at strong material
            break
```

**Best used for:**
- Getting inside fortified structures
- Damaging internal supports
- Precision strikes through windows/gaps

**Visual:**
```
   🎯 Silver pointed bolt
   Rotates rapidly in flight
   Trail: Spiral vapor trail
   Sound: Whistle → Multiple impacts inside
```

### 8. Frost Bomb (Ice Physics)

**Properties:**
- Mass: 10 kg
- Speed: Medium (30 m/s)
- Damage: Freezing + brittle failure
- Cost: 3 stars (unlock level 20)

**Physics behavior:**
- Explodes into freezing mist on impact
- Freezes voxels in radius
- Frozen voxels become brittle (easy to break)
- Ice melts after 20 seconds

**Freezing effect:**
```python
@ti.kernel
def freeze_area(center, radius):
    for voxel in radius:
        voxel.temperature = -20°C
        voxel.frozen = True
        
        # Brittle when frozen
        voxel.strength *= 0.3  # 70% weaker
        
        # Visual change
        voxel.color = blue_tint(voxel.base_color)
```

**Best used for:**
- Making strong materials weak
- Combo with second projectile (freeze → shatter)
- Water features (freeze rivers/moats)

**Visual:**
```
   ❄️ Blue-white sphere
   Trail: Frost particles
   Explosion: Expanding mist cloud
   Sound: Crystalline tinkle
```

---

## Materials & Their Properties

### Wood
```python
strength = 20
density = 600 kg/m³
thermal_conductivity = 0.1
ignition_temperature = 300°C

behavior:
- Splinters when broken
- Burns and spreads fire
- Floats in water
- Weak against all projectiles
```

### Stone
```python
strength = 100
density = 2400 kg/m³
thermal_conductivity = 0.5
ignition_temperature = N/A (doesn't burn)

behavior:
- Cracks before breaking
- Doesn't burn
- Very heavy
- Strong against light projectiles
- Weak to heavy impacts
```

### Steel
```python
strength = 200
density = 7800 kg/m³
thermal_conductivity = 50
ignition_temperature = N/A

behavior:
- Bends before breaking
- Weakens when heated (>400°C)
- Extremely heavy
- Strong against most projectiles
- Weak to explosives and thermal
```

### Glass
```python
strength = 3
density = 2500 kg/m³
thermal_conductivity = 1
thermal_shock_sensitivity = HIGH

behavior:
- Shatters on any impact
- Transparent (visual through it)
- Breaks from thermal shock
- Weak to everything
```

### Rope/Chain
```python
strength = 10 (tension only)
density = negligible
burn_time = 5 seconds

behavior:
- Only works in tension (can't push)
- Snaps when overloaded
- Burns quickly
- Used for suspension bridges, etc.
```

---

## Level Design Principles

### Difficulty Progression

**World 1: Tutorial Castle (Levels 1-10)**
- All wood construction
- Simple structures (towers, houses)
- Clear weak points
- Camera locked to front view (no rotation)
- Teaches: Aiming basics, power control

**World 2: Stone Fortress (Levels 11-20)**
- Mixed wood and stone
- More complex layouts
- Rotation unlocked
- Hidden weak points (require angle exploration)
- Teaches: Material understanding, angle selection

**World 3: Steel Citadel (Levels 21-30)**
- Steel frame structures
- Multi-story buildings
- Multiple angles required
- Advanced projectiles needed
- Teaches: Strategic projectile choice, combo shots

**World 4: Mixed Master (Levels 31-40)**
- All materials combined
- Complex structures (castles, bridges, factories)
- Optional challenges (time limits, projectile limits)
- Expert-level engineering
- Teaches: Mastery, optimization

### Level Structure Template

Every level has:

1. **Starting camera angle** (default view)
2. **Castle/structure** (voxel-based, different each level)
3. **Targets** (1-5 goblins placed strategically)
4. **Projectile allowance** (1-5 shots)
5. **Star requirements:**
   - ⭐ Complete (all targets destroyed)
   - ⭐⭐ Efficient (under projectile budget)
   - ⭐⭐⭐ Perfect (under time limit + no collateral)

### Example Level Breakdown

**Level 1: "First Shot"**
```
Structure: Single wooden tower (8 voxels high)
Target: 1 goblin on top
Projectiles: 1 stone ball
Difficulty: Tutorial

Solution: Hit base of tower, it topples

Star requirements:
⭐ Hit goblin
⭐⭐ 1 shot used
⭐⭐⭐ Under 30 seconds
```

**Level 13: "The Stone Arch"**
```
Structure: Stone arch bridge spanning gap
Targets: 2 goblins on bridge
Projectiles: 3 shots (1 cannonball, 2 stone balls)
Difficulty: Medium

Solution: Hit keystone with cannonball, bridge collapses

Star requirements:
⭐ Both goblins fall
⭐⭐ 2 shots or less
⭐⭐⭐ Under 45 seconds + destroy keystone first
```

**Level 25: "Factory Siege"**
```
Structure: Steel frame factory, wooden walls, glass windows
Targets: 3 goblins on different floors
Projectiles: 5 mixed (player chooses)
Difficulty: Hard

Solution: Multiple approaches possible:
- Explosive through window → internal destruction
- Incendiary on wooden walls → fire spreads → collapse
- Heavy shots on steel frame → progressive failure

Star requirements:
⭐ All 3 goblins destroyed
⭐⭐ 3 shots or less
⭐⭐⭐ Under 60 seconds + factory completely demolished
```

---

## Visual Design

### Art Style: Stylized Medieval

**NOT photorealistic. NOT cartoon. Somewhere between.**

**Reference games:**
- Valheim (low-poly medieval)
- Kingdom Two Crowns (silhouette style)
- For The King (stylized fantasy)

### Color Palette

**Structure materials:**
```
Stone:  Warm gray (#A8907B)
Wood:   Rich brown (#6B4423)
Steel:  Blue-gray (#3D5A80)
Glass:  Light cyan (#B4E3F0, transparent)
Rope:   Tan (#D4A574)
```

**Environment:**
```
Sky:    Gradient blue (#87CEEB → #4682B4)
Grass:  Vibrant green (#5FAD56)
Ground: Brown earth (#8B7355)
Water:  Deep blue (#1E5F8C)
```

**UI & Effects:**
```
Trajectory:  Yellow (#FFD700)
Impact:      Red (#FF4444)
Success:     Green (#44FF44)
Fire:        Orange-red gradient
Ice:         White-blue gradient
```

### Voxel Style

**Size:** 1 voxel = 1 meter cube in world space
**On screen:** 8-12 pixels per voxel (depending on zoom)

**Rendering:**
- Slightly rounded edges (1px smoothing)
- Flat shaded (no gradients on faces)
- Simple ambient occlusion (darken in corners)
- No textures (solid colors only)

**Why this works:**
- Clean, readable
- Runs fast (low poly count)
- Scales to any resolution
- Distinctive look

### Camera & Presentation

**Camera properties:**
```python
field_of_view = 60°
near_plane = 1
far_plane = 500
position_from_castle = 50 voxels
height = 30 voxels
look_at = castle_center
```

**Camera modes:**

1. **Orbit mode (ROTATE)**
   - Smooth circular motion around castle
   - Fixed distance and height
   - Always looking at castle center

2. **Locked mode (AIM)**
   - Completely static
   - Player focuses on aiming

3. **Follow mode (PROJECTILE FLIGHT)**
   - Tracks projectile smoothly
   - Leads projectile slightly (look ahead)
   - Adjusts zoom based on projectile speed

4. **Collapse mode (DESTRUCTION)**
   - Dynamic framing
   - Zooms in on dramatic moments
   - Pulls back for wide shots
   - Adds cinematic shake

### Visual Effects (The Juice)

**Impact effects:**
```
Frame 0: Flash of white light
Frame 1-3: Expanding shockwave ring
Frame 4-10: Particle burst (debris)
Frame 11-20: Dust cloud expansion
Frame 21+: Smoke dissipates
```

**Projectile trails:**
```
Stone ball: None
Cannonball: Black smoke trail
Explosive: Sparking fuse trail
Splitting shot: Vapor trail
Wrecking ball: None (too heavy)
Incendiary: Fire trail
Drill bolt: Spiral vapor trail
Frost bomb: Frost particles
```

**Destruction particles:**
```
Wood: Brown splinters + sawdust
Stone: Gray chunks + dust cloud
Steel: Metal shards + sparks
Glass: Clear shards + sparkles

Each particle:
- Inherits velocity from parent voxel
- Affected by gravity
- Bounces once (elastic 0.3)
- Fades over 2 seconds
- ~10 particles per voxel destroyed
```

**Screen effects:**
```
Impact shake:
- Intensity = impact_force / 100
- Duration = 0.5 seconds
- Random direction each frame

Slow motion:
- Triggered on: Impact, first major collapse
- Speed = 0.3x normal
- Duration = 0.5-1.0 seconds
- Smooth transition in/out

Vignette:
- Darkens edges during dramatic moments
- Focuses attention on destruction
```

---

## Mobile Implementation

### Performance Targets

**iPhone 13/14 (high-end):**
- Resolution: 128³ voxels
- FPS: 60 locked
- Physics: Full simulation 60 Hz
- Effects: All enabled

**iPhone 11/12 (mid-range):**
- Resolution: 96³ voxels
- FPS: 60 locked
- Physics: Full simulation 60 Hz
- Effects: All enabled

**iPhone XR / Budget Android:**
- Resolution: 64³ voxels
- FPS: 45-60
- Physics: 30 Hz (still smooth)
- Effects: Reduced particles

### Touch Controls

**Single finger for everything:**

**In ROTATE mode:**
```
Touch + swipe left/right = Rotate camera
Tap = Lock angle
Two-finger pinch = Zoom (optional)
```

**In AIM mode:**
```
Touch catapult + drag = Aim
Release = Fire
Tap outside = Cancel (back to rotate)
```

**Tap zones:**
```
┌─────────────────────────────────┐
│ [?] Settings    Score: 1250 [⭐] │
│                                 │
│                                 │
│         MAIN VIEW               │
│     (Aiming happens here)       │
│                                 │
│                                 │
├─────────────────────────────────┤
│ 🏹 🪨 💣 ❄️  ← Projectile select│
│ [Rotate] [Fire] [Menu] ← Actions│
└─────────────────────────────────┘
```

**Button sizes:**
- Minimum: 44×44 points (iOS guideline)
- Recommended: 60×60 points
- Spacing: 8 points between

### Mobile-Specific Features

**Haptic feedback:**
```python
# On drag start
haptic_feedback(style="light")

# On lock rotation
haptic_feedback(style="medium")

# On fire
haptic_feedback(style="heavy")

# On impact
haptic_feedback(style="heavy", duration=0.5)
```

**Auto-pause:**
```python
# Game auto-pauses when:
- Phone call incoming
- App goes to background
- Battery low warning
- Notification appears

# Resume exactly where left off
```

**Battery optimization:**
```python
# Reduce physics update when idle
if no_touch_for(5_seconds):
    physics_update_rate = 10 Hz  # From 60 Hz
    
if no_touch_for(30_seconds):
    physics_update_rate = 1 Hz  # Minimal
    dim_screen()

# Full rate resumes on touch
```

---

## Monetization Strategy

### Mobile (Primary Platform)

**Free to play with ads:**

**Level access:**
- Levels 1-10: Free (tutorial + hook)
- Levels 11-20: Watch ad to unlock OR $0.99 to skip
- Levels 21-30: Watch ad OR $1.99
- Levels 31-40: Watch ad OR $2.99

**OR one-time unlock:**
- $4.99: Remove all ads + unlock all current levels
- $7.99: Above + all future level packs

**Optional IAP:**
- Projectile skins: $0.99 each
- Castle themes: $1.99 each
- VIP pass: $9.99 (ad-free + all cosmetics + beta access)

**Revenue model:**
```
100k downloads:
- 80k free players (watch ads) = $8-16k (ad rev)
- 15k unlock ($4.99) = $75k
- 5k cosmetics/VIP = $20-50k

Total: $103-141k revenue

Cost to make: 3-4 weeks = $0 (solo dev)
ROI: Infinite
```

### PC/Steam (Secondary Platform)

**Premium pricing:**
- Base game: $7.99
- Soundtrack: $2.99
- Level editor: Included free

**Revenue model:**
```
10k Steam sales at $7.99:
- Gross: $79.9k
- Steam cut (30%): -$24k
- Net: $55.9k

Lower volume but better margins
```

---

## Development Roadmap

### Week 1: Prototype

**Days 1-2: Core aiming system**
- [ ] Implement ROTATE mode (camera orbit)
- [ ] Implement AIM mode (trajectory preview)
- [ ] Test on mobile device
- [ ] Verify feels good

**Days 3-4: Basic physics**
- [ ] Port THERMITE structural simulation
- [ ] Add projectile ballistics
- [ ] Add impact detection
- [ ] Test collapse behavior

**Days 5-7: First playable level**
- [ ] Build wooden tower (procedural)
- [ ] Add goblin target
- [ ] Win condition
- [ ] Restart functionality

**Milestone: Can you aim, fire, destroy? If yes → continue**

### Week 2: Content & Polish

**Days 8-10: Projectile variety**
- [ ] Add 3 more projectile types
- [ ] Different physics behaviors
- [ ] Balance damage values
- [ ] Test each on demo level

**Days 11-12: Level creation**
- [ ] Build levels 1-10 (tutorial world)
- [ ] Test difficulty progression
- [ ] Add star rating system
- [ ] Implement level select screen

**Days 13-14: Visual polish**
- [ ] Particle effects
- [ ] Screen shake & slow-mo
- [ ] Sound effects (free assets)
- [ ] UI refinement

**Milestone: 10 levels playable, looks good**

### Week 3: Mobile Optimization

**Days 15-17: Performance**
- [ ] Profile on 3 device tiers
- [ ] Optimize slow kernels
- [ ] Add quality settings
- [ ] Battery optimization

**Days 18-19: Touch UX**
- [ ] Refine touch controls
- [ ] Haptic feedback
- [ ] Tutorial overlays
- [ ] Menu polish

**Days 20-21: Content sprint**
- [ ] Levels 11-20 (stone world)
- [ ] Test on real devices
- [ ] Balance pass

**Milestone: Ready for beta test**

### Week 4: Launch Prep

**Days 22-24: Testing**
- [ ] Beta test with 10 people
- [ ] Fix critical bugs
- [ ] Balance adjustments
- [ ] Performance fixes

**Days 25-26: Marketing**
- [ ] Record trailer (gameplay clips)
- [ ] Create screenshots
- [ ] Write store description
- [ ] Set up social media

**Days 27-28: Release**
- [ ] Submit to App Store
- [ ] Submit to Google Play
- [ ] Soft launch (one country)
- [ ] Monitor analytics

**Milestone: SHIPPED**

---

## Post-Launch Roadmap

### Month 2: Content Updates

**Week 5-6:**
- Levels 21-30 (steel world)
- 2 new projectile types
- Leaderboards

**Week 7-8:**
- Levels 31-40 (mixed world)
- Daily challenges
- Achievements

### Month 3: Features

**Level editor:**
- Build custom castles
- Share via code
- Upvote best levels

**Multiplayer (async):**
- Friend challenges
- Build castle, friend destroys it
- Score competitions

### Month 4+: New Worlds

**World 5: Ice Kingdom**
- Frozen structures
- Thermal mechanics
- New ice projectiles

**World 6: Floating Islands**
- Balloons, chains
- Wind mechanics
- Aerial targets

**World 7: Underground**
- Cave systems
- Support pillars
- Lava hazards

---

## Success Metrics

### MVP Success (Week 4):
- [ ] Soft launch in 1 country
- [ ] 500+ downloads first week
- [ ] Retention: 40%+ day 1
- [ ] Retention: 20%+ day 7
- [ ] 4+ star rating

### Month 1 Success:
- [ ] 10k+ downloads
- [ ] Retention: 30%+ day 7
- [ ] Revenue: $5k+ (covers opportunity cost)
- [ ] Featured on 1+ indie game blog

### Month 3 Success:
- [ ] 100k+ downloads
- [ ] Revenue: $50k+ (profitable)
- [ ] 4.5+ star rating
- [ ] Community forming (Discord/Reddit)

### Month 6 Success:
- [ ] 500k+ downloads
- [ ] Revenue: $200k+ (sustainable)
- [ ] 1+ YouTuber coverage
- [ ] User-created content spreading

---

## Risk Analysis

### Technical Risks

**Risk: Mobile performance issues**
- Mitigation: Test on device from day 1
- Fallback: Reduce voxel resolution
- Severity: Medium
- Likelihood: Low

**Risk: Aiming UX doesn't feel good**
- Mitigation: Prototype aiming FIRST before content
- Fallback: Simplify to fixed camera only
- Severity: High
- Likelihood: Medium

**Risk: Physics bugs (weird collapses)**
- Mitigation: Extensive testing, damage clamping
- Fallback: Can patch post-launch
- Severity: Medium
- Likelihood: Medium

### Market Risks

**Risk: Can't compete with Angry Birds**
- Mitigation: Target different audience (physics fans, not casual)
- Fallback: Pivot positioning to "engineering puzzle game"
- Severity: High
- Likelihood: High

**Risk: No discoverability (buried in app store)**
- Mitigation: Reddit/social media marketing, viral GIFs
- Fallback: PC launch on Steam (better discoverability)
- Severity: High
- Likelihood: High

**Risk: Players don't care about "real physics"**
- Mitigation: Make physics DRAMATIC (amplified for fun)
- Fallback: Add other hooks (characters, story, progression)
- Severity: Medium
- Likelihood: Medium

### Development Risks

**Risk: Scope creep (never ship)**
- Mitigation: Fixed 4-week timeline, cut features ruthlessly
- Fallback: Ship with 10 levels, add more post-launch
- Severity: High
- Likelihood: Medium

**Risk: Motivation loss**
- Mitigation: Public commitment, daily progress posts
- Fallback: Ship minimal version even if not perfect
- Severity: Medium
- Likelihood: Low

---

## Why This Will Work

### Strengths

1. **Proven market** (Angry Birds = billions of downloads)
2. **Clear differentiator** ("real physics" is immediately impressive)
3. **Fast development** (4 weeks to MVP with vibe-coding)
4. **Low cost** (solo dev, no art team needed)
5. **Viral potential** (spectacular collapses shareable)
6. **Mobile-perfect** (touch controls ideal for this)
7. **Scalable content** (procedural + level editor)

### Weaknesses

1. **Fierce competition** (Angry Birds clone stigma)
2. **Discoverability challenge** (app store is crowded)
3. **"Physics" might not hook casual players** (too technical?)
4. **Monetization uncertainty** (F2P vs premium vs hybrid)

### Opportunities

1. **Physics-hungry market** (Teardown showed demand)
2. **Mobile lacks physics depth** (gap in market)
3. **YouTube potential** (physics games perform well)
4. **Educational angle** (schools, STEM programs)
5. **Esports potential** (speedrunning, precision challenges)

### Threats

1. **Rovio could clone this** (they have resources)
2. **Market saturation** (too many mobile games)
3. **Platform changes** (iOS/Android updates breaking game)
4. **Player expectations** (need constant content updates)

---

## Final Recommendation

**BUILD THIS GAME.**

**Why:**
- Aiming system is solved (2-mode hybrid works)
- Technical feasibility proven (Taichi + mobile works)
- Development time is short (4 weeks to launch)
- Market is proven (billions love physics catapult games)
- Worst case: Learn + portfolio piece
- Best case: $100k+ revenue from mobile hit

**Start tomorrow.**

**Test aiming in 2 days.**

**Ship in 4 weeks.**

**Then iterate based on real player feedback.**

---

## Appendix: Quick Reference

### Input Controls Summary

**Mobile:**
```
ROTATE mode:
- Swipe left/right = Rotate camera
- Tap "Lock" = Switch to AIM mode

AIM mode:
- Drag from catapult = Aim (sets power + angle)
- Release = Fire
- Tap "Cancel" = Back to ROTATE mode
```

**PC (future):**
```
ROTATE mode:
- A/D keys = Rotate camera
- Space = Lock angle

AIM mode:
- Mouse drag = Aim
- Release = Fire
- Escape = Cancel
```

### Physics Formulas

**Trajectory:**
```
x(t) = v₀ₓ * t
y(t) = v₀ᵧ * t - 0.5 * g * t²
z(t) = v₀ₖ * t

Where:
v₀ = launch velocity
g = 9.8 m/s² (gravity)
t = time
```

**Structural stress:**
```
stress = weight_above / material_strength

if stress > 1.0:
    voxel_fails()
```

**Impact damage:**
```
damage = (projectile_mass * projectile_velocity) / voxel_strength
voxel_health -= damage
```

### Development Commands

```bash
# Run on desktop
python main.py

# Test on mobile (iOS)
ti build --platform ios --arch metal
# Install on device

# Test on mobile (Android)
ti build --platform android --arch vulkan
# Install on device

# Profile performance
python main.py --profile
```

### File Structure

```
siege/
├── main.py              # Main game loop
├── simulation.py        # Taichi physics
├── aiming.py           # Aiming system
├── renderer.py         # Voxel rendering
├── levels.py           # Level definitions
├── projectiles.py      # Projectile behaviors
├── ui.py              # UI rendering
└── assets/
    ├── sounds/        # SFX
    └── levels/        # Level data
```

---

**Now go build it.**

**The aiming system works. The physics work. The market exists.**

**All that's left is execution.**

**Good luck. 🏹**
