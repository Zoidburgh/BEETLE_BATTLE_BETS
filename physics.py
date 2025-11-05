"""
THERMITE Physics Engine - PATH 1 Implementation
Trigger-based voxel destruction with debris particles
"""

import taichi as ti
import simulation

# ============================================================
# TUNABLE PARAMETERS (adjust these for feel)
# ============================================================

# Impact destruction parameters
BASE_DESTRUCTION_RADIUS = 8.0        # voxels, baseline explosion size (increased for drama)
VELOCITY_RADIUS_SCALE = 0.03         # radius += speed * this (more velocity scaling)
MIN_DESTRUCTION_RADIUS = 3.0         # never smaller
MAX_DESTRUCTION_RADIUS = 20.0        # never larger (performance cap, increased)

# Debris physics parameters
EXPLOSION_FORCE = 30.0               # m/s initial debris velocity (increased for dramatic effect)
EXPLOSION_FORCE_FALLOFF = 0.8        # edge velocity = force * this
DEBRIS_GRAVITY = 9.8                 # m/s² downward acceleration
DEBRIS_BOUNCE_ELASTICITY = 0.4       # loses 60% energy on bounce
DEBRIS_FRICTION = 0.8                # XZ velocity *= this on ground hit
DEBRIS_LIFETIME_MAX = 5.0            # seconds before cleanup
DEBRIS_MIN_VELOCITY = 0.1            # stop particles moving slower than this

# Ground level
GROUND_Y = 0.5                       # Y coordinate of ground plane

# Projectile parameters
PROJECTILE_RADIUS = 1.0              # Cannonball collision radius
PROJECTILE_GRAVITY = 9.8             # Same as debris

# ============================================================
# PROJECTILE KERNELS
# ============================================================

def spawn_projectile(pos, velocity):
    """
    Spawn a new projectile (Python helper function).

    Args:
        pos: ti.math.vec3 starting position
        velocity: ti.math.vec3 initial velocity

    Returns:
        Index of spawned projectile, or -1 if pool full
    """
    # Find first inactive slot
    for i in range(simulation.MAX_PROJECTILES):
        if simulation.projectile_active[i] == 0:
            simulation.projectile_pos[i] = pos
            simulation.projectile_vel[i] = velocity
            simulation.projectile_active[i] = 1
            simulation.projectile_radius[i] = PROJECTILE_RADIUS
            simulation.num_projectiles[None] += 1
            return i
    return -1  # Pool full

@ti.kernel
def update_projectiles(dt: ti.f32):
    """
    Update all active projectiles - apply gravity, update positions.

    Args:
        dt: Delta time in seconds
    """
    for idx in range(simulation.MAX_PROJECTILES):
        if simulation.projectile_active[idx] == 1:
            # Apply gravity
            simulation.projectile_vel[idx].y -= PROJECTILE_GRAVITY * dt

            # Update position
            simulation.projectile_pos[idx] += simulation.projectile_vel[idx] * dt

            # Despawn if below ground or too far away
            if simulation.projectile_pos[idx].y < -10.0:
                simulation.projectile_active[idx] = 0

@ti.kernel
def check_projectile_collisions() -> ti.i32:
    """
    Check all projectiles for collision with voxel grid.
    Returns index of first colliding projectile, or -1 if none.
    """
    result = -1  # Default: no collision

    for idx in range(simulation.MAX_PROJECTILES):
        if simulation.projectile_active[idx] == 1 and result == -1:
            pos = simulation.projectile_pos[idx]
            radius = simulation.projectile_radius[idx]

            # Convert to grid coordinates
            grid_x = int(pos.x + simulation.n_grid / 2.0)
            grid_y = int(pos.y)
            grid_z = int(pos.z + simulation.n_grid / 2.0)

            # Check bounding box around projectile
            check_radius = int(radius) + 1
            has_collision = 0

            for dx in range(-check_radius, check_radius + 1):
                for dy in range(-check_radius, check_radius + 1):
                    for dz in range(-check_radius, check_radius + 1):
                        if has_collision == 0:  # Only check if no collision yet
                            check_x = grid_x + dx
                            check_y = grid_y + dy
                            check_z = grid_z + dz

                            # Bounds check
                            if 0 <= check_x < simulation.n_grid and \
                               0 <= check_y < simulation.n_grid and \
                               0 <= check_z < simulation.n_grid:

                                voxel_type = simulation.voxel_type[check_x, check_y, check_z]

                                if voxel_type != simulation.EMPTY:
                                    # Collision found!
                                    has_collision = 1
                                    result = idx

    return result

# ============================================================
# KERNEL 1: Sphere Destruction
# ============================================================

@ti.kernel
def destroy_sphere_at_impact(
    center: ti.math.vec3,
    base_radius: ti.f32,
    projectile_speed: ti.f32
) -> ti.i32:
    """
    Destroy voxels in sphere around impact point.
    Accounts for material hardness and projectile velocity.

    Args:
        center: Impact point in world coordinates
        base_radius: Base explosion radius (tunable parameter)
        projectile_speed: Projectile velocity in m/s

    Returns:
        Number of voxels destroyed
    """
    destroyed_count = 0

    # Calculate effective radius (scales with projectile speed)
    velocity_bonus = projectile_speed * VELOCITY_RADIUS_SCALE
    radius = ti.min(base_radius + velocity_bonus, MAX_DESTRUCTION_RADIUS)
    radius = ti.max(radius, MIN_DESTRUCTION_RADIUS)

    # Bounding box optimization - only check affected region
    # Convert world coordinates to grid coordinates
    center_grid_x = int(center.x + simulation.n_grid / 2.0)
    center_grid_y = int(center.y)
    center_grid_z = int(center.z + simulation.n_grid / 2.0)

    # Calculate bounding box
    min_x = ti.max(0, int(center_grid_x - radius))
    max_x = ti.min(simulation.n_grid, int(center_grid_x + radius + 1))
    min_y = ti.max(0, int(center_grid_y - radius))
    max_y = ti.min(simulation.n_grid, int(center_grid_y + radius + 1))
    min_z = ti.max(0, int(center_grid_z - radius))
    max_z = ti.min(simulation.n_grid, int(center_grid_z + radius + 1))

    # Iterate only within bounding box
    for i, j, k in ti.ndrange((min_x, max_x), (min_y, max_y), (min_z, max_z)):
        voxel_type = simulation.voxel_type[i, j, k]

        # Skip empty voxels
        if voxel_type != simulation.EMPTY:
            # Convert grid coordinates to world coordinates
            world_x = float(i) - simulation.n_grid / 2.0
            world_y = float(j)
            world_z = float(k) - simulation.n_grid / 2.0
            voxel_pos = ti.math.vec3(world_x, world_y, world_z)

            # Calculate distance from impact center
            dist = (voxel_pos - center).norm()

            # Simple sphere check (no material hardness for now - all same material)
            # Check if within destruction sphere
            if dist < radius:
                # Mark voxel as destroyed (temporarily as DEBRIS type)
                simulation.voxel_type[i, j, k] = simulation.DEBRIS
                destroyed_count += 1

    return destroyed_count

# ============================================================
# KERNEL 2: Spawn Debris Particles
# ============================================================

@ti.kernel
def spawn_debris_from_destroyed(
    impact_center: ti.math.vec3,
    explosion_force: ti.f32
):
    """
    Convert DEBRIS voxels in grid to flying debris particles.
    Creates radial explosion pattern from impact center.

    Args:
        impact_center: Center of explosion in world coordinates
        explosion_force: Initial velocity magnitude for debris
    """
    # Iterate through entire grid looking for DEBRIS voxels
    for i, j, k in simulation.voxel_type:
        voxel_type = simulation.voxel_type[i, j, k]

        if voxel_type == simulation.DEBRIS:
            # Convert to world coordinates
            world_x = float(i) - simulation.n_grid / 2.0
            world_y = float(j)
            world_z = float(k) - simulation.n_grid / 2.0
            voxel_pos = ti.math.vec3(world_x, world_y, world_z)

            # Atomic add to get unique debris index (thread-safe)
            debris_idx = ti.atomic_add(simulation.num_debris[None], 1)

            # Only add if we have space in debris pool
            if debris_idx < simulation.MAX_DEBRIS:
                # Calculate explosion direction (radial from impact center)
                direction = voxel_pos - impact_center
                distance = direction.norm()

                # Normalize direction (handle zero-distance case)
                if distance > 0.001:
                    direction = direction / distance
                else:
                    direction = ti.math.vec3(0.0, 1.0, 0.0)  # Default up

                # Velocity falloff - stronger near center, weaker at edges
                falloff = 1.0 - ti.min(distance / (MAX_DESTRUCTION_RADIUS * 2.0), EXPLOSION_FORCE_FALLOFF)
                velocity = direction * explosion_force * falloff

                # Store debris particle data
                simulation.debris_pos[debris_idx] = voxel_pos
                simulation.debris_vel[debris_idx] = velocity
                simulation.debris_material[debris_idx] = simulation.CONCRETE  # All concrete for now
                simulation.debris_lifetime[debris_idx] = 0.0

            # Clear voxel from grid (particle replaces voxel)
            simulation.voxel_type[i, j, k] = simulation.EMPTY

# ============================================================
# KERNEL 3: Update Debris Physics
# ============================================================

@ti.kernel
def update_debris_physics(dt: ti.f32):
    """
    Update all active debris particles.
    Applies gravity, updates positions, handles ground collision.

    Args:
        dt: Delta time in seconds
    """
    n = simulation.num_debris[None]

    # Parallel update of all debris particles
    for idx in range(n):
        # Apply gravity
        simulation.debris_vel[idx].y -= DEBRIS_GRAVITY * dt

        # Update position (simple Euler integration)
        simulation.debris_pos[idx] += simulation.debris_vel[idx] * dt

        # Ground collision detection and response
        if simulation.debris_pos[idx].y < GROUND_Y:
            # Snap to ground
            simulation.debris_pos[idx].y = GROUND_Y

            # Bounce (invert Y velocity with energy loss)
            simulation.debris_vel[idx].y *= -DEBRIS_BOUNCE_ELASTICITY

            # Friction on horizontal velocity
            simulation.debris_vel[idx].x *= DEBRIS_FRICTION
            simulation.debris_vel[idx].z *= DEBRIS_FRICTION

            # Stop if barely moving (prevent infinite micro-bounces)
            if ti.abs(simulation.debris_vel[idx].y) < DEBRIS_MIN_VELOCITY:
                simulation.debris_vel[idx].y = 0.0
                # Also stop horizontal movement when settled
                if simulation.debris_vel[idx].x * simulation.debris_vel[idx].x + \
                   simulation.debris_vel[idx].z * simulation.debris_vel[idx].z < DEBRIS_MIN_VELOCITY * DEBRIS_MIN_VELOCITY:
                    simulation.debris_vel[idx].x = 0.0
                    simulation.debris_vel[idx].z = 0.0

        # Increment lifetime
        simulation.debris_lifetime[idx] += dt

# ============================================================
# KERNEL 4: Simple Unsupported Check
# ============================================================

@ti.kernel
def mark_unsupported_in_region(
    min_x: ti.i32, max_x: ti.i32,
    min_y: ti.i32, max_y: ti.i32,
    min_z: ti.i32, max_z: ti.i32,
    required_support: ti.i32
) -> ti.i32:
    """
    Check voxels in region for structural support.
    Improved rule: Check 3x3 region below each voxel and require adequate support.

    Args:
        min/max_x/y/z: Bounding box in grid coordinates
        required_support: Minimum number of support voxels needed (out of 9)

    Returns:
        Count of voxels marked as falling
    """
    falling_count = 0

    # Clamp bounds to grid
    actual_min_x = ti.max(0, min_x)
    actual_max_x = ti.min(simulation.n_grid, max_x)
    actual_min_y = ti.max(0, min_y)
    actual_max_y = ti.min(simulation.n_grid, max_y)
    actual_min_z = ti.max(0, min_z)
    actual_max_z = ti.min(simulation.n_grid, max_z)

    # Check from top to bottom (so upper voxels fall when lower ones are gone)
    # Note: Can't use negative step in Taichi range, so iterate and reverse index
    height = actual_max_y - actual_min_y
    for idx in range(height):
        j = actual_max_y - 1 - idx  # Start from top, go down
        for i, k in ti.ndrange((actual_min_x, actual_max_x),
                                (actual_min_z, actual_max_z)):
            voxel_type = simulation.voxel_type[i, j, k]

            # Only check solid voxels (not empty, not already debris)
            if voxel_type != simulation.EMPTY and voxel_type != simulation.DEBRIS:
                # Count solid voxels in 3x3 region below
                support_count = 0

                if j > 0:
                    # Check 3x3 region below this voxel
                    for dx in range(-1, 2):
                        for dz in range(-1, 2):
                            check_x = i + dx
                            check_z = k + dz

                            # Stay within grid bounds
                            if 0 <= check_x < simulation.n_grid and 0 <= check_z < simulation.n_grid:
                                below_type = simulation.voxel_type[check_x, j - 1, check_z]
                                if below_type != simulation.EMPTY and below_type != simulation.DEBRIS:
                                    support_count += 1
                else:
                    # Ground level always has full support
                    support_count = 9

                # Check if support meets the required threshold
                if support_count < required_support:
                    simulation.voxel_type[i, j, k] = simulation.DEBRIS
                    falling_count += 1

    return falling_count

# ============================================================
# KERNEL 5: Cleanup Old Debris
# ============================================================

@ti.kernel
def cleanup_old_debris():
    """
    Remove debris particles older than DEBRIS_LIFETIME_MAX.
    Uses compaction pattern to keep array dense.
    """
    write_idx = 0
    n = simulation.num_debris[None]

    # Compact array - keep only particles younger than lifetime limit
    for read_idx in range(n):
        if simulation.debris_lifetime[read_idx] < DEBRIS_LIFETIME_MAX:
            # Keep this particle
            if write_idx != read_idx:
                # Copy to compacted position
                simulation.debris_pos[write_idx] = simulation.debris_pos[read_idx]
                simulation.debris_vel[write_idx] = simulation.debris_vel[read_idx]
                simulation.debris_material[write_idx] = simulation.debris_material[read_idx]
                simulation.debris_lifetime[write_idx] = simulation.debris_lifetime[read_idx]
            write_idx += 1

    # Update active count
    simulation.num_debris[None] = write_idx

# ============================================================
# HELPER FUNCTIONS (Python-side utilities)
# ============================================================

def trigger_impact(impact_pos, projectile_speed=100.0, base_radius=None, explosion_force=None, support_threshold=None):
    """
    Convenience function to trigger full impact sequence with cascade collapse.

    Args:
        impact_pos: ti.math.vec3 impact position in world coords
        projectile_speed: Projectile velocity in m/s
        base_radius: Override BASE_DESTRUCTION_RADIUS (optional)
        explosion_force: Override EXPLOSION_FORCE (optional)
        support_threshold: Override support check threshold (optional)

    Returns:
        dict with destruction statistics
    """
    # Use defaults if not provided
    if base_radius is None:
        base_radius = BASE_DESTRUCTION_RADIUS
    if explosion_force is None:
        explosion_force = EXPLOSION_FORCE
    if support_threshold is None:
        support_threshold = 2

    # 1. Destroy voxels in sphere
    destroyed = destroy_sphere_at_impact(
        impact_pos,
        base_radius,
        projectile_speed
    )

    # 2. Spawn debris particles from destroyed voxels
    if destroyed > 0:
        spawn_debris_from_destroyed(impact_pos, explosion_force)

    # 3. Check for unsupported voxels - MULTIPLE ITERATIONS for cascade collapse
    # Check a larger vertical column (entire height above impact)
    region_radius = int(MAX_DESTRUCTION_RADIUS + 10)
    center_grid_x = int(impact_pos.x + simulation.n_grid / 2.0)
    center_grid_y = int(impact_pos.y)
    center_grid_z = int(impact_pos.z + simulation.n_grid / 2.0)

    total_falling = 0
    max_iterations = 5  # Allow cascade effects

    for iteration in range(max_iterations):
        # Check from impact point to top of grid (entire column above)
        falling = mark_unsupported_in_region(
            center_grid_x - region_radius,
            center_grid_x + region_radius,
            center_grid_y,  # Start from impact height
            simulation.n_grid,  # Go all the way to top
            center_grid_z - region_radius,
            center_grid_z + region_radius,
            support_threshold  # Use slider value
        )

        if falling > 0:
            total_falling += falling
            # Spawn debris from this iteration (gentler force for falling voxels)
            spawn_debris_from_destroyed(impact_pos, EXPLOSION_FORCE * 0.2)
        else:
            # No more voxels falling, stop iterating
            break

    return {
        'destroyed': destroyed,
        'falling': total_falling,
        'total_debris': simulation.num_debris[None]
    }
