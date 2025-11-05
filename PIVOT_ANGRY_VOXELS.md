# Project Pivot: 3D Voxel Destruction Game

**Date:** November 3, 2025
**Status:** Active Development

---

## Executive Summary

### Original Concept
THERMITE was originally envisioned as a thermal-based demolition game where players place thermite charges that heat and weaken structures through realistic heat propagation physics, causing structural collapse.

### New Concept: "Angry Voxels" (working title)
A 3D projectile-based destruction game where players rotate a camera/catapult around iconic voxel structures and launch various projectiles to cause maximum destruction. Think "Angry Birds meets Teardown" in 3D with realistic voxel physics.

### Reasons for Pivot
1. **Faster to playable prototype** - Core mechanics can be implemented in days vs weeks
2. **Proven gameplay loop** - Projectile physics games are understood and fun
3. **Leverages existing work** - 80% of our voxel rendering and camera system carries over
4. **More accessible** - Easier to understand than heat propagation mechanics
5. **Mobile-friendly** - Touch controls map naturally to aiming and shooting

---

## Core Gameplay

### Player Actions
1. **Aim** - Rotate camera horizontally around structure (A/D keys or drag)
2. **Angle** - Adjust vertical trajectory (W/S keys or vertical drag)
3. **Power** - Hold spacebar/tap to charge shot power
4. **Fire** - Release to launch projectile
5. **Watch** - See voxels explode and structure collapse

### Win Conditions
- Destroy X% of structure (e.g., 80%)
- Knock down specific targets (columns, towers, etc.)
- Complete within limited shots (3-5 attempts)

### Projectile Types
1. **Cannonball** - Standard projectile, medium damage, medium radius
2. **Explosive** - Large destruction radius, slower
3. **Piercing** - Goes through multiple voxels, small radius
4. **Heavy Ball** - High impact force, small radius, causes collapse

---

## What We Already Have (Built)

✅ **GPU-Accelerated Voxel Rendering**
- Taichi GGUI for 60 FPS rendering
- 128×128×128 voxel grid (2M voxels)
- Handles 100K visible voxels smoothly

✅ **Camera System**
- Smooth rotation (A/D keys)
- Vertical panning (W/S keys)
- Zoom in/out (Q/E keys)
- Mouse wheel support

✅ **Multiple Iconic Structures**
- Test building (10-story asymmetric tower)
- Roman Colosseum (elliptical, 4 tiers, arches)
- Golden Gate Bridge (suspension cables, towers, deck)

✅ **Material System**
- Steel voxels (blue tint)
- Concrete voxels (gray)
- Easy to extend for more materials

---

## Development Roadmap

### Phase 1: Core Mechanics (1-2 days) - PRIORITY

**Goal:** Get projectile shooting and basic destruction working

**Tasks:**
1. ✅ Camera system (already done)
2. 🔨 Lock camera at fixed orbital distance from structure center
3. 🔨 Add aiming crosshair/reticle UI
4. 🔨 Projectile launching system
   - Hold space to charge power
   - Release to fire
   - Show power meter
5. 🔨 Ballistic physics (gravity + initial velocity)
6. 🔨 Trajectory preview line
7. 🔨 Collision detection (projectile vs voxels)
8. 🔨 Voxel destruction on impact (sphere radius removal)

**Success Criteria:** Can aim, shoot projectile, and blow holes in structures

---

### Phase 2: Structural Physics (2-3 days)

**Goal:** Make destruction satisfying with realistic collapse

**Tasks:**
1. 🔨 Gravity simulation for voxels
2. 🔨 Connectivity checking (flood fill from ground)
3. 🔨 Unsupported voxels fall
4. 🔨 Chain reactions (falling voxels damage others)
5. 🔨 Multiple projectile types
6. 🔨 Impact force (knockback/momentum transfer)

**Success Criteria:** Bridges sag and collapse, towers topple, realistic physics

---

### Phase 3: Mobile Support (2-3 days)

**Goal:** Make game playable on mobile devices

**Tasks:**
1. 🔨 Touch input detection
2. 🔨 Single-finger drag for horizontal rotation
3. 🔨 Two-finger pinch/drag for vertical angle
4. 🔨 Tap-and-hold for power charge
5. 🔨 UI scaling for different screen sizes
6. 🔨 Portrait and landscape support
7. 🔨 Performance optimization for mobile GPUs

**Success Criteria:** Playable on phone with touch controls, 30+ FPS

---

### Phase 4: Game Polish (1-2 days)

**Goal:** Make it feel like a real game

**Tasks:**
1. 🔨 HUD with shot counter
2. 🔨 Destruction percentage display
3. 🔨 Victory/failure screens
4. 🔨 Level selection menu
5. 🔨 Star rating (1-3 stars based on efficiency)
6. 🔨 Camera shake on impact
7. 🔨 Particle effects on destruction
8. 🔨 Sound effects (impact, collapse)

**Success Criteria:** Feels polished and complete

---

### Phase 5: Content & Levels (Ongoing)

**Goal:** Add variety and replayability

**Tasks:**
1. 🔨 Easy levels (simple towers)
2. 🔨 Medium levels (bridge, multi-tower)
3. 🔨 Hard levels (Colosseum, complex structures)
4. 🔨 More projectile types
5. 🔨 Environmental hazards
6. 🔨 Moving structures
7. 🔨 Combo/scoring system

---

## Technical Architecture

### Core Systems

**Voxel Rendering (DONE)**
- File: `renderer.py`
- GPU particle rendering via Taichi GGUI
- 100K max voxels, 50+ FPS

**Voxel Physics (TODO)**
- File: `simulation.py`
- Currently: Static structures only
- Need: Gravity, collision, connectivity checking

**Camera System (MOSTLY DONE)**
- File: `renderer.py` + `main.py`
- Currently: Free rotation/pan/zoom
- Need: Lock to orbital mode, aiming mode

**Projectile System (TODO)**
- File: New `projectiles.py`
- Ballistic physics (gravity)
- Collision detection
- Multiple projectile types

**Input Handling (PARTIAL)**
- File: `main.py` + `renderer.py`
- Currently: Keyboard only
- Need: Touch support, power charging

### File Structure
```
THERMITE/
├── main.py              # Game loop, input, window
├── simulation.py        # Voxel physics, structures
├── renderer.py          # GPU rendering, camera
├── projectiles.py       # (NEW) Projectile physics
├── ui.py                # (NEW) HUD, menus
├── levels.py            # (NEW) Level definitions
└── PIVOT_ANGRY_VOXELS.md  # This document
```

---

## Immediate Next Steps

### TODAY: Camera Refinement + Aiming System

**Step 1: Lock Camera to Orbital Mode**
- Camera orbits at fixed distance from structure center
- Horizontal rotation (A/D) moves around structure
- Vertical angle control (W/S) for trajectory

**Step 2: Add Aiming UI**
- Draw crosshair at screen center
- Show trajectory preview line (parabolic arc)
- Display angle and power indicators

**Step 3: Power Charging**
- Hold spacebar to charge (0-100%)
- Visual power meter
- Release to fire at charged power

### TOMORROW: Projectile Physics + Destruction

**Step 4: Implement Projectile**
- Spawn projectile at camera position
- Apply initial velocity based on angle + power
- Update position with gravity each frame
- Draw projectile as sphere/voxel

**Step 5: Collision Detection**
- Check projectile position against voxel grid each frame
- On collision: destroy voxels in radius (sphere)
- Remove projectile after impact
- Visual impact effect

**Step 6: Test & Iterate**
- Tune physics constants (gravity, power scaling)
- Test on all three structures
- Adjust destruction radius for fun

---

## Design Principles

1. **Feel over realism** - Physics should be fun, not perfectly realistic
2. **Instant feedback** - Player sees immediate results from their actions
3. **Easy to learn, hard to master** - Simple controls, skill-based optimization
4. **Satisfying destruction** - Big explosions, dramatic collapses
5. **Mobile-first** - Design for touch, keyboard is bonus

---

## Success Metrics

**Minimum Viable Product (MVP):**
- ✅ Can rotate camera around structure
- ❌ Can fire projectile with adjustable angle/power
- ❌ Projectile destroys voxels on impact
- ❌ At least one structure works well

**Version 1.0:**
- ❌ 3+ levels with different structures
- ❌ 3+ projectile types
- ❌ Realistic collapse physics
- ❌ Victory conditions and star rating
- ❌ Works on mobile with touch controls

**Stretch Goals:**
- ❌ Level editor
- ❌ User-created structures
- ❌ Multiplayer (async or real-time)
- ❌ Procedurally generated structures

---

## Future Considerations

### Could We Add Thermite Back?
**Yes!** Thermite could become one of several projectile types:
- **Thermite Projectile** - Sticks to surface, burns for 5 seconds, weakens nearby steel
- Adds unique gameplay: delayed destruction, strategic placement
- Combines both concepts

### Monetization (If Successful)
- Mobile: Ads + optional IAP for projectile packs
- Steam: $5-10 one-time purchase
- Free web version with limited levels

---

## Notes & Lessons Learned

**What Worked:**
- GPU-accelerated voxel rendering is fast enough
- Taichi handles large grids well
- Building recognizable structures pays off

**What Didn't Work:**
- Heat propagation is too complex for initial prototype
- Pygame rendering was too slow (fixed by switching to Taichi GGUI)

**Key Insight:**
Start with the simplest fun mechanic (shooting things), then add complexity (thermite) later if it makes sense.

---

**Last Updated:** November 3, 2025
**Next Review:** After Phase 1 completion
