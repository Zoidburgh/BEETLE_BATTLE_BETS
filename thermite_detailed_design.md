# THERMITE: Complete Design Document
## The Pyromaniac's Puzzle Game

---

## Executive Summary

**Elevator Pitch:** 
"You're a demolition expert with thermite charges. Melt steel beams to collapse buildings. Every material has real thermal properties. Plan your burns, watch physics do the rest."

**Core Loop (30 seconds):**
1. Rotate camera to study building structure
2. Place thermite charges on key beams
3. Ignite sequence
4. Watch heat spread (rotate camera to see different angles)
5. Steel weakens → Structure collapses
6. Did it work? Reset and try again or move to next level

**The Hook:**
This is the ONLY game where buildings collapse from actual thermal weakening of materials, not scripted destruction. Every playthrough is different because physics.

---

## Why This Is Actually Fun

### 1. **The "Aha!" Moment**

**Bad player (first attempt):**
- Places thermite randomly
- Burns everything
- Building falls... eventually
- Messy, expensive, slow

**Good player (after learning):**
- Identifies load-bearing columns
- Heats ONE critical beam
- Beam fails → load shifts to adjacent beam
- Adjacent beam overloads → fails
- CASCADE FAILURE
- Building pancakes perfectly
- Uses 3 charges instead of 20

**That learning curve = FUN.**

### 2. **Emergent Chaos You Caused**

You're not watching a cutscene. You're watching YOUR decisions play out through physics:

- "I heated that corner beam..."
- "...which made the floor sag..."
- "...which put torsional stress on THAT beam..."
- "...which I didn't even touch..."
- "...and now the whole wing is collapsing sideways!"

**Every mistake teaches you about structural engineering.**
**Every success feels earned.**

### 3. **The Planning → Execution → "Oh Shit" → Satisfaction Arc**

**Planning phase:** Calm, strategic. "If I heat here, then this will..."

**Ignition:** Committed. No undo. "Here we go..."

**Heat spreading:** Building tension. "Come on, melt faster..."

**First failure:** "YES! It's starting!"

**Cascade begins:** "Oh god it's going THAT way—"

**Final collapse:** "HOLY SHIT THAT WORKED" or "well, fuck"

**That emotional rollercoaster = engagement.**

### 4. **Optimization & Mastery**

**Casual players:** Trial and error until it works
**Learning players:** Start understanding load paths and heat spread
**Master players:** Discover insane optimizations (cascade failures, minimal thermite, heat spreading exploits)

**[LATER: Rating/scoring system - decide after core is fun]**

### 5. **Visceral Satisfaction**

**There's something primal about:**
- Watching steel glow red-hot
- Hearing the creak of buckling metal
- Seeing a 40-story tower pancake
- Creating a perfectly controlled collapse

**It's the same dopamine hit as:**
- Popping bubble wrap
- Peeling protective film off electronics
- Watching dominos fall
- Satisfying ASMR destruction videos

**But YOU made it happen through puzzle-solving.**

---

## The Physics That Make It Work

### Material Properties (Simplified but Believable)

**Steel Beams:**
- Melting point: 1400°C
- Weakening starts: 400°C (loses 50% strength)
- Critical failure: 800°C (loses 90% strength)
- Thermal conductivity: HIGH (heat spreads through steel)
- Behavior: Bends before breaking, plastic deformation

**Concrete:**
- Doesn't melt, but cracks at 600°C
- Thermal conductivity: LOW (insulator)
- Contains steel rebar (weak point when rebar heats)
- Behavior: Shatters/crumbles under stress

**Wood:**
- Ignition: 300°C
- Burns: Creates MORE heat (fire spreads)
- Thermal conductivity: VERY LOW
- Behavior: Burns, collapses when structural integrity lost

**Thermite:**
- Reaction temperature: 2500°C (melts through anything)
- Burn duration: 15-30 seconds
- Localized (small radius, intense heat)
- Behavior: One-time use, point source of extreme heat

**Glass:**
- Melting point: 1500°C
- Shatters from thermal shock (rapid heating)
- Thermal conductivity: LOW
- Behavior: Brittle failure

### Heat Transfer Rules

**Conduction:** Heat spreads through touching materials
- Fast through steel (beams spread heat to connected beams)
- Slow through concrete (walls insulate)
- Very slow through air gaps

**Convection:** Hot air rises
- Fire/heat creates updraft
- Upper floors get hotter faster
- Smoke fills upper rooms

**Radiation:** Intense heat affects nearby objects
- Thermite radiates heat to close materials
- Red-hot steel heats adjacent wood (can ignite)

### ⚠️ PERFORMANCE CRITICAL

**Heat Diffusion is Expensive:**
- 128³ grid = 2+ million voxels
- Checking 6 neighbors per voxel per frame = 12M+ checks
- **Solution:** Only update active voxels, skip every N frames, use sparse grid
- **Start with 64³ grid and scale up only after optimization**

**Structural Calculation is O(n×height):**
- Calculating "weight above" for each voxel means checking all voxels above it
- For a 30-floor building, that's checking 30+ voxels per column
- **Solution:** Cache load paths, only recalculate when structure changes, or use beam elements instead of per-voxel stress

**Wood Fire Can Explode Exponentially:**
- Each burning voxel can ignite 6 neighbors → 6^n growth
- Can lag the simulation or burn too fast to see
- **Solution:** Rate-limit spread, require sustained heat, add "burn through" timer

### Structural Physics

**Load-bearing logic:**
- Each voxel knows weight above it
- Beams distribute load horizontally
- When beam fails, load redistributes to remaining beams
- Overloaded beams fail faster

**Failure modes:**
- **Buckling:** Vertical columns bend under compression
- **Shear:** Horizontal beams snap from sideways force
- **Torsion:** Twisting forces (when one side fails first)
- **Pancaking:** Floors collapse onto floors below

---

## Example Puzzles (Progressive Difficulty)

### Tutorial Puzzles (Levels 1-5)

#### **Level 1: "The Shed"**
**Structure:** Simple wooden shack, 4 corner posts, flat roof
**Objective:** Demolish completely
**Thermite budget:** 2 charges
**Par time:** 30 seconds

**Teaching:** Basic placement, ignition, fire spread
**Solution:** Place thermite on 2 opposite corners, wood catches fire, spreads, entire structure burns and collapses

**Why it works:** Impossible to fail. Builds confidence.

---

#### **Level 2: "The Garage"**
**Structure:** Steel frame, concrete walls on 3 sides, open front
**Objective:** Collapse roof
**Thermite budget:** 3 charges
**Par time:** 45 seconds
**New mechanic:** Steel doesn't burn, must heat to weaken

**Teaching:** Steel requires sustained heat, concrete doesn't conduct
**Solution:** Place thermite on steel support columns (not concrete walls), heat until columns buckle, roof falls

**Learning curve:** Players who put thermite on concrete walls learn they're wasting charges

---

#### **Level 3: "The Billboard"**
**Structure:** Tall steel frame billboard, concrete foundation
**Objective:** Topple billboard FORWARD (specific direction)
**Thermite budget:** 2 charges
**Par time:** 40 seconds
**New mechanic:** Directional collapse

**Teaching:** Heat one side to make it fall that direction
**Solution:** Heat front two support legs, back legs hold, billboard tips forward

**Aha moment:** "Oh, I can CONTROL where it falls!"

---

#### **Level 4: "The Warehouse"**
**Structure:** Large steel frame, 6 support columns, flat roof, valuable cargo inside
**Objective:** Collapse roof without destroying cargo
**Thermite budget:** 4 charges
**Par time:** 60 seconds
**Constraint:** Cargo must survive

**Teaching:** Partial demolition, controlled collapse
**Solution:** Heat top of columns only, roof collapses inward, cargo (lower level) undamaged

**Difficulty spike:** First puzzle that's actually hard. Some players will fail multiple times.

---

#### **Level 5: "The Condemned House"**
**Structure:** 2-story wood + steel hybrid, mixed materials
**Objective:** Complete demolition
**Thermite budget:** 5 charges
**Par time:** 90 seconds
**New mechanic:** Heat spreading between different materials

**Teaching:** Steel conducts heat to wood (can ignite), planning chain reactions
**Solution:** Heat steel beams near wood, let conduction ignite wood, fire spreads, minimal thermite needed

**Graduation:** Players who beat this understand the core mechanics

---

### Mid-Game Puzzles (Levels 6-15)

#### **Level 8: "The Bridge"**
**Structure:** Steel truss bridge over river, 30m span
**Objective:** Collapse center span into river
**Thermite budget:** 3 charges
**Par time:** 45 seconds
**Constraint:** Don't damage shore supports (reusable for new bridge)

**Challenge:** Identify critical truss members
**Solution:** Heat bottom chord members at center, top chord can't support alone, center collapses, ends remain standing

**Why it's interesting:** Real engineering problem. Many solutions, but optimal one requires understanding truss load paths.

---

#### **Level 10: "The Insurance Job"**
**Structure:** 3-story office building, steel frame
**Objective:** Demolish completely
**Thermite budget:** 8 charges
**Par time:** 2 minutes
**Constraint:** Must fall into its own footprint (buildings on all sides)
**Bonus objective:** Perfectly vertical collapse (pancaking)

**Challenge:** Even distribution of heat/failure
**Solution:** Heat bottom floor columns evenly, bottom floor fails first, upper floors pancake straight down

**Why it's iconic:** This is THE classic controlled demolition. Getting three stars feels like being a pro.

---

#### **Level 12: "The Grain Silo"**
**Structure:** Tall cylindrical concrete silo, steel reinforcement, full of grain (granular physics!)
**Objective:** Empty silo by creating breach
**Thermite budget:** 4 charges
**Par time:** 60 seconds
**New mechanic:** Grain flows out breach, additional weight stress on structure

**Challenge:** Structural integrity changes as grain flows
**Solution:** Heat bottom ring of steel reinforcement, breach forms, grain flows out, added stress collapses silo

**Visual spectacle:** Thousands of grain particles flowing out, then silo crumbling

---

#### **Level 14: "The Ice Hotel"**
**Structure:** Ice blocks reinforced with steel frame, subfreezing environment
**Objective:** Demolish structure
**Thermite budget:** 6 charges
**Par time:** 90 seconds
**New mechanic:** Ice melts into water, water flows, refreezes in some areas
**Environmental:** Cold air slows heat spread

**Challenge:** Heat spreads slowly, ice melts and refreezes
**Solution:** Focus heat on steel frame (ice doesn't load-bear anyway), let structure collapse into ice blocks

**Why it's cool:** Totally different feel from other levels, phase changes visible

---

### Late-Game Puzzles (Levels 16-25)

#### **Level 18: "The Refinery"**
**Structure:** Chemical plant with tanks, pipes, steel catwalks
**Objective:** Demolish but DON'T rupture chemical tanks (explosive)
**Thermite budget:** 10 charges
**Par time:** 3 minutes
**Constraint:** Chemical tanks must remain intact
**Hazard:** If tank ruptures = mission fail + massive explosion

**Challenge:** Surgical precision, avoid collateral damage
**Solution:** Carefully heat support structures AWAY from tanks, guide collapse direction away, tanks settle gently

**Tension:** One wrong move = spectacular failure

---

#### **Level 20: "The Skyscraper"**
**Structure:** 15-story steel frame building, glass facade
**Objective:** Controlled demolition
**Thermite budget:** 15 charges
**Par time:** 4 minutes
**Constraint:** Neighboring buildings must not be damaged
**Bonus:** Perfect pancake collapse

**Challenge:** Largest structure yet, complex load paths
**Solution:** Floors 1-3: heat all columns evenly, stagger timing for perfect pancake, upper floors slam down sequentially

**Epic:** This is your final exam. Beat this with 3 stars = you're a master

---

#### **Level 23: "The Suspension Bridge"**
**Structure:** Golden Gate-style suspension bridge, cables + towers + deck
**Objective:** Drop center span only, preserve towers
**Thermite budget:** 6 charges
**Par time:** 2 minutes
**Constraint:** Towers must survive (historic landmarks)

**Challenge:** Suspension cables redistribute load weirdly
**Solution:** Heat cable anchors at center span, cables fail, deck falls, tension releases from towers (they survive)

**Iconic:** Everyone knows this bridge type, collapsing it feels momentous

---

#### **Level 25: "The Castle"**
**Structure:** Medieval stone castle, some steel reinforcement (modern restoration)
**Objective:** Breach the wall (create 10m wide opening)
**Thermite budget:** 8 charges
**Par time:** 2 minutes
**Constraint:** Don't collapse whole castle (war crime!)
**Historical:** Wall is 3m thick stone + steel rebar

**Challenge:** Precise breach, control collapse extent
**Solution:** Heat steel rebar inside wall at key points, thermal expansion cracks stone, section collapses, rest remains

**Satisfying:** Sieging a castle with thermite feels badass

---

### Post-Game Challenges (Endless Mode)

**Procedurally generated buildings:**
- Random height (5-30 floors)
- Random materials mix
- Random constraints
- Thermite budget scales with difficulty

**Leaderboards:**
- Fastest demolition
- Fewest thermite charges
- Most spectacular collapse (measured by debris spread?)

---

## Asset Generation Strategy

### The Beautiful Part: You Don't Need Artists

**Everything is voxels. Voxels are procedural.**

### Building Generation (Procedural)

#### **Method 1: Structural Templates**

Define buildings as structural skeletons + fill:

```python
class Building:
    def __init__(self):
        self.floors = 5
        self.width = 20  # voxels
        self.depth = 15
        
    def generate(self):
        # Steel columns at corners and every 5 voxels
        for floor in range(self.floors):
            for x in [0, 5, 10, 15, 20]:
                for z in [0, 5, 10, 15]:
                    place_voxel(x, floor*4, z, STEEL_COLUMN)
        
        # Horizontal beams connecting columns
        for floor in range(self.floors):
            for x in range(0, 20):
                place_voxel(x, floor*4, 0, STEEL_BEAM)
                # ... etc
        
        # Concrete floor slabs
        for floor in range(self.floors):
            for x in range(0, 20):
                for z in range(0, 15):
                    place_voxel(x, floor*4-1, z, CONCRETE_FLOOR)
        
        # Glass facade
        for floor in range(self.floors):
            for y in range(floor*4, floor*4+3):
                place_voxel(0, y, z, GLASS)  # Outer walls
```

**Ask AI to generate these templates:**
> "Generate a procedural building generator for THERMITE. Buildings should have steel frame with columns, horizontal beams, concrete floors, and glass facades. Make buildings look realistic."

AI gives you the code. Tweak numbers until buildings look good.

#### **Method 2: L-Systems for Complex Structures**

For bridges, towers, complex shapes:

```python
# Bridge truss generator
def generate_truss(length, height):
    # Top chord
    for i in range(length):
        place_voxel(i, height, 0, STEEL_BEAM)
    
    # Bottom chord
    for i in range(length):
        place_voxel(i, 0, 0, STEEL_BEAM)
    
    # Vertical members every 5 units
    for i in range(0, length, 5):
        for y in range(height):
            place_voxel(i, y, 0, STEEL_COLUMN)
    
    # Diagonal bracing (Warren truss pattern)
    for i in range(0, length-5, 10):
        for y in range(height):
            place_voxel(i + y/height*5, y, 0, STEEL_BEAM)
```

**Again, ask AI:**
> "Generate procedural bridge trusses, Warren and Pratt patterns, parameterized by length and height."

#### **Method 3: Voxel Sculpture (Manual)**

For unique levels (castle, iconic buildings):

**Tools:**
- MagicaVoxel (free voxel editor)
- Export as .vox file
- Write importer (or ask AI to write it)
- Load into game

**Workflow:**
1. Sketch building in MagicaVoxel (30 minutes)
2. Export
3. Game loads it as voxel grid
4. Play level

**Pro tip:** You only need 25-30 handcrafted levels. That's maybe 15-20 hours of voxel sculpting total.

### Visual Style: Make Limits Look Good

**Go for minimalist/stylized:**

#### **Color Palette (Critical for voxel games)**

**Option 1: Industrial Realism**
- Steel: Blue-gray (#4A5568)
- Concrete: Light gray (#CBD5E0)
- Wood: Warm brown (#8B4513)
- Glass: Cyan tint (#81E6D9 with transparency)
- Thermite: Bright orange (#FF6B35)
- Heat glow: Red → Orange → Yellow gradient

**Option 2: Vaporwave Aesthetic**
- Steel: Purple (#9D4EDD)
- Concrete: Pink (#FF006E)
- Wood: Cyan (#06FFA5)
- Glass: Magenta with transparency
- Thermite: Neon blue
- Heat: Purple → Pink → White gradient

**Pick ONE palette and commit.** Consistency makes voxels look intentional, not lazy.

#### **Lighting**

**Don't overthink this:**
```python
# Simple ambient occlusion
@ti.kernel
def compute_ao():
    for i, j, k in voxels:
        if voxel_type[i,j,k] != EMPTY:
            # Count solid neighbors
            neighbor_count = 0
            for di, dj, dk in neighbors:
                if voxel_type[i+di,j+dj,k+dk] != EMPTY:
                    neighbor_count += 1
            
            # Darken based on occlusion
            brightness[i,j,k] = 1.0 - (neighbor_count / 26.0) * 0.6
```

**That's it. Looks good enough.**

**Heat glow (the important one):**
```python
@ti.kernel
def render_heat():
    for i, j, k in voxels:
        if temperature[i,j,k] > 400:
            # Red hot glow
            heat_color = map_temp_to_color(temperature[i,j,k])
            voxel_color[i,j,k] = mix(base_color, heat_color, heat_intensity)
```

**Steel glowing red-hot is 90% of your visual appeal.**

### Particle Effects (The Juice)

**When voxels break:**
```python
def spawn_debris(position, velocity, material):
    for i in range(10):  # 10 debris particles per broken voxel
        particle = Particle(
            pos=position + random_offset(),
            vel=velocity + random_velocity(),
            color=material.color,
            lifetime=2.0  # seconds
        )
        particles.append(particle)
```

**Smoke when things heat up:**
```python
def spawn_smoke(position, intensity):
    particle = Particle(
        pos=position,
        vel=Vector(0, 5, 0) + random_horizontal(),  # Rises
        color=lerp(gray, black, intensity),
        lifetime=3.0,
        size_over_time=grows  # Smoke expands as it rises
    )
```

**Sparks when thermite ignites:**
```python
def spawn_sparks(position):
    for i in range(50):
        particle = Particle(
            pos=position,
            vel=random_direction() * 10,  # Fast
            color=orange,
            lifetime=0.5,  # Brief
            gravity=True
        )
```

**Ask AI to implement particle systems. It's good at this.**

### Camera & Presentation [CORE FEATURE - NOT OPTIONAL]

**Camera is ESSENTIAL for gameplay:**
Players need to see structure from multiple angles to plan demolition. This isn't polish, it's core functionality.

**Phase 1 (MVP - Simple but functional):**
- Fixed isometric view
- Arrow keys / WASD to rotate 90° increments (N/E/S/W)
- Q/E to rotate camera around structure
- Scroll wheel to zoom in/out
- Mouse drag to pan (if needed)

**Phase 2 (After core physics work):**
- Free rotation with mouse drag
- Smooth camera transitions (not instant snaps)
- Double-click to focus on a point

**Phase 3 (Polish - LATER):**
- Slow-motion toggle (SPACE to slow to 0.3x speed)
- Auto-camera during collapse (optional dramatic angle)
- Replay system with free camera

**Implementation notes:**
```python
# Camera state
camera_angle = 0  # 0, 90, 180, 270 degrees
camera_distance = 300
camera_height = 100

# Rotation on key press
if key == K_RIGHT:
    camera_angle = (camera_angle + 90) % 360

# Zoom
if mouse_wheel_up:
    camera_distance -= 20
```

**DO THIS EARLY.** Day 2 or 3 at latest. Can't test levels without seeing the structure.

### UI (Keep It Minimal)

**MVP HUD:**
- Thermite charges remaining
- Current mode (PLACE / IGNITED)
- FPS counter (for performance testing)

**Nice to have (add after core works):**
- Temperature readout (on hover/click)
- Objective text
- Simple timer

**Post-level (LATER - decide after core is fun):**
- "Level Complete!" message
- Button to reset / next level
- Stats if we want them

**Main menu (MUCH LATER):**
- Play button
- Level select
- Quit button

**That's it. Focus on simulation working first.**

---

## Technical Architecture

### File Structure

```
thermite/
├── main.py              # Main game loop
├── simulation.py        # Taichi physics kernels
├── renderer.py          # Voxel rendering
├── buildings.py         # Procedural generation
├── levels.py            # Level definitions
├── ui.py                # Pygame UI
└── assets/
    ├── sounds/
    └── levels/          # .vox files for handcrafted levels
```

### Core Systems

**simulation.py:**
```python
import taichi as ti

ti.init(arch=ti.gpu)

# Voxel grid
n_grid = 128
voxel_type = ti.field(dtype=ti.i32, shape=(n_grid, n_grid, n_grid))
temperature = ti.field(dtype=ti.f32, shape=(n_grid, n_grid, n_grid))
structural_integrity = ti.field(dtype=ti.f32, shape=(n_grid, n_grid, n_grid))

@ti.kernel
def simulate_heat_diffusion():
    # Heat spreads through materials
    pass

@ti.kernel
def simulate_structural_stress():
    # Check load bearing, fail weak voxels
    pass

@ti.kernel
def simulate_thermite():
    # Active thermite generates heat
    pass
```

**Ask AI to flesh out each kernel as you need it.**

### Development Phases (REVISED - Focus on Tech First)

**Week 1: Core Physics + Performance**
- Day 1-2: Heat diffusion working (start 64³ grid)
- Day 3: Basic structural integrity
- Day 4-5: Optimize! Get 60 FPS with active simulation
- Day 6-7: Camera rotation/zoom (ESSENTIAL)
- Material: Steel only for now

**Week 2: Thermite + Basic Gameplay Loop**
- Day 1-2: Place thermite, ignition, UI
- Day 3: Add concrete + wood materials
- Day 4-5: Make collapse FEEL good (timing, gravity)
- Day 6-7: First test level (The Shed) - iterate until fun
- **Checkpoint:** Can place thermite, ignite, structure collapses, feels satisfying

**Week 3: Content Foundation (IF tech is solid)**
- 5 tutorial levels
- Test with friends - is it actually fun?
- Fix whatever feels bad
- Add simple level complete/reset

**Week 4+: TBD Based on What We Learn**
- More levels if core is fun
- Polish only after core is proven
- Sound/particles only if they enhance what already works

**Don't think about Week 5-6 until Week 3 proves this is worth finishing.**

---

## Monetization [WAY LATER - IGNORE FOR NOW]

**Don't think about this until you have a fun game.**

When the time comes:
- Free on itch.io initially (build audience, get feedback)
- Maybe premium later if people love it
- Or keep it free and make something bigger next

**Focus: Make it work, make it fun, make people want to play it.**

---

## Marketing Hook

**Trailer structure (60 seconds):**

1. **Opening (5s):** "What if buildings collapsed realistically?"
2. **Gameplay (20s):** Show planning, placement, ignition, collapse sequence
3. **Variety (15s):** Quick cuts of different structures (bridge, skyscraper, castle)
4. **Mastery (10s):** Three-star perfect run, speedrun footage
5. **Hook (5s):** "Every demolition is different. Physics, not scripts."
6. **CTA (5s):** Logo, "THERMITE - Coming [date]"

**GIF for Twitter/Reddit:**
- 15-story building pancaking perfectly
- Bridge collapse into river
- Castle wall breach

**Post it with:** "Spent 20 minutes planning this demolition. Buildings in THERMITE collapse from real physics, not animations."

**Subreddits:**
- r/IndieDev
- r/DestroyedTanks (they love destruction)
- r/SimulationGaming
- r/EngineeringPorn (yes really, structural stuff does well there)

---

## Why This Will Stand Out

**What else is out there:**
- Teardown: Beautiful, but scripted voxel fracture, not thermal physics
- Besiege: Great, but medieval, not modern demolition
- Hardspace Shipbreaker: Close! But space salvage, not demolition

**What you have:**
- Real thermodynamics
- Structural engineering
- Modern setting (relatable)
- Puzzle focus (not just chaos)

**You're in a unique niche.**

---

## Success Metrics (Realistic Checkpoints)

**Week 1 Success:**
- Simulation runs at 60 FPS
- Heat spreads visibly through materials
- Structure collapses when heated
- You can rotate camera to see what's happening

**Week 2 Success:**
- You can place thermite and ignite it
- Collapse feels satisfying (not instant, not too slow)
- You want to try different configurations
- Performance still good (45+ FPS during collapse)

**Week 3 Success:**
- One person who isn't you plays it for 30+ minutes
- They understand the mechanics without you explaining
- They say something like "oh cool!" when it collapses
- They want to try "just one more"

**IF THOSE WORK → Keep going. If not → Pivot or stop.**

---

## Final Thoughts: Why Start Here

1. **Scope is controllable** - Can ship with 20 levels
2. **Core loop is proven** - Puzzle games work
3. **Physics is achievable** - Thermal + structural is well-understood
4. **Visuals are cheap** - Voxels = no 3D modeling needed
5. **Marketing writes itself** - "Buildings collapse realistically"
6. **It's actually fun** - I want to play this RIGHT NOW

**You can have a playable prototype in 2 weeks.**
**You can ship a full game in 6-8 weeks.**
**It will look like nothing else on Steam.**

Now go melt some steel beams.
