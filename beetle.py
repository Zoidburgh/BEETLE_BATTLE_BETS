"""
Beetle Battle - Beetle Entity System
Manages beetle physics, AI, and rendering state
"""

import taichi as ti
import simulation

# Maximum beetles in arena at once
MAX_BEETLES = 10

# Beetle entity data (Structure of Arrays for GPU efficiency)
beetle_active = ti.field(dtype=ti.i32, shape=MAX_BEETLES)  # 1 = active, 0 = inactive
num_beetles = ti.field(dtype=ti.i32, shape=())  # Active beetle count

# Physics state
beetle_pos = ti.Vector.field(3, dtype=ti.f32, shape=MAX_BEETLES)  # Position (world coords)
beetle_vel = ti.Vector.field(3, dtype=ti.f32, shape=MAX_BEETLES)  # Velocity (m/s)
beetle_rotation = ti.field(dtype=ti.f32, shape=MAX_BEETLES)  # Rotation angle (radians, 0 = +X axis)
beetle_angular_vel = ti.field(dtype=ti.f32, shape=MAX_BEETLES)  # Rotation speed (rad/s)

# Physical properties
beetle_radius = ti.field(dtype=ti.f32, shape=MAX_BEETLES)  # Collision radius
beetle_mass = ti.field(dtype=ti.f32, shape=MAX_BEETLES)  # Mass (kg)
beetle_size = ti.field(dtype=ti.f32, shape=MAX_BEETLES)  # Visual size multiplier

# Combat state
beetle_health = ti.field(dtype=ti.f32, shape=MAX_BEETLES)  # HP (0-100)
beetle_stamina = ti.field(dtype=ti.f32, shape=MAX_BEETLES)  # Energy for special moves
beetle_is_charging = ti.field(dtype=ti.i32, shape=MAX_BEETLES)  # 1 = charging attack

# Animation state (for future use)
beetle_leg_phase = ti.field(dtype=ti.f32, shape=MAX_BEETLES)  # Leg animation time
beetle_last_collision_time = ti.field(dtype=ti.f32, shape=MAX_BEETLES)  # Time of last hit

# Arena parameters
ARENA_RADIUS = 40.0  # Arena radius in world units
ARENA_CENTER = ti.math.vec3(0.0, 0.0, 0.0)

# Physics constants
BEETLE_BASE_MASS = 10.0  # kg
BEETLE_BASE_RADIUS = 8.0  # collision radius
FRICTION = 0.85  # Velocity decay per second
ROTATION_FRICTION = 0.9  # Angular velocity decay
GRAVITY = 9.8  # m/s²

# Movement forces (tunable)
MOVE_FORCE = 80.0  # Forward/backward thrust
TURN_TORQUE = 5.0  # Rotation speed
MAX_SPEED = 15.0  # Speed cap
MAX_ANGULAR_SPEED = 3.0  # Rotation speed cap

# ============================================================
# INITIALIZATION
# ============================================================

@ti.kernel
def init_beetles():
    """Clear all beetles and reset arena"""
    for i in range(MAX_BEETLES):
        beetle_active[i] = 0
    num_beetles[None] = 0

def spawn_beetle(pos, rotation=0.0, size=1.0):
    """
    Spawn a new beetle in the arena (Python helper).

    Args:
        pos: ti.math.vec3 starting position
        rotation: Initial rotation angle (radians)
        size: Size multiplier (affects mass and radius)

    Returns:
        Index of spawned beetle, or -1 if arena full
    """
    # Find first inactive slot
    for i in range(MAX_BEETLES):
        if beetle_active[i] == 0:
            beetle_active[i] = 1
            beetle_pos[i] = pos
            beetle_vel[i] = ti.math.vec3(0.0, 0.0, 0.0)
            beetle_rotation[i] = rotation
            beetle_angular_vel[i] = 0.0

            beetle_radius[i] = BEETLE_BASE_RADIUS * size
            beetle_mass[i] = BEETLE_BASE_MASS * (size ** 3)  # Mass scales with volume
            beetle_size[i] = size

            beetle_health[i] = 100.0
            beetle_stamina[i] = 100.0
            beetle_is_charging[i] = 0

            beetle_leg_phase[i] = 0.0
            beetle_last_collision_time[i] = 0.0

            num_beetles[None] += 1
            return i
    return -1  # Arena full

# ============================================================
# PHYSICS UPDATE
# ============================================================

@ti.kernel
def update_beetle_physics(dt: ti.f32):
    """
    Update all beetle physics - movement, rotation, friction, arena bounds.

    Args:
        dt: Delta time in seconds
    """
    for idx in range(MAX_BEETLES):
        if beetle_active[idx] == 1:
            # Apply friction to velocity
            beetle_vel[idx] *= ti.pow(FRICTION, dt)
            beetle_angular_vel[idx] *= ti.pow(ROTATION_FRICTION, dt)

            # Update position
            beetle_pos[idx] += beetle_vel[idx] * dt

            # Update rotation
            beetle_rotation[idx] += beetle_angular_vel[idx] * dt

            # Normalize rotation to [0, 2π]
            while beetle_rotation[idx] > 3.14159 * 2.0:
                beetle_rotation[idx] -= 3.14159 * 2.0
            while beetle_rotation[idx] < 0.0:
                beetle_rotation[idx] += 3.14159 * 2.0

            # Arena boundary collision (push back if outside)
            dist_from_center = (beetle_pos[idx] - ARENA_CENTER).norm()
            if dist_from_center > ARENA_RADIUS - beetle_radius[idx]:
                # Push back to arena edge
                direction = (beetle_pos[idx] - ARENA_CENTER).normalized()
                beetle_pos[idx] = ARENA_CENTER + direction * (ARENA_RADIUS - beetle_radius[idx])

                # Reflect velocity (bounce off wall)
                normal = direction
                beetle_vel[idx] -= normal * (beetle_vel[idx].dot(normal)) * 1.5  # Bounce

            # Clamp to ground level (beetles don't fly)
            if beetle_pos[idx].y < 0.5:
                beetle_pos[idx].y = 0.5
                if beetle_vel[idx].y < 0.0:
                    beetle_vel[idx].y = 0.0

# ============================================================
# MOVEMENT CONTROLS
# ============================================================

@ti.kernel
def apply_beetle_thrust(beetle_idx: ti.i32, forward: ti.f32):
    """
    Apply forward/backward thrust to beetle.

    Args:
        beetle_idx: Index of beetle to control
        forward: Thrust amount (-1.0 to 1.0, negative = backward)
    """
    if beetle_active[beetle_idx] == 1:
        # Get forward direction from rotation
        rot = beetle_rotation[beetle_idx]
        forward_dir = ti.math.vec3(ti.cos(rot), 0.0, ti.sin(rot))

        # Apply force
        force = forward_dir * forward * MOVE_FORCE
        acceleration = force / beetle_mass[beetle_idx]
        beetle_vel[beetle_idx] += acceleration

        # Clamp speed
        speed = beetle_vel[beetle_idx].norm()
        if speed > MAX_SPEED:
            beetle_vel[beetle_idx] = beetle_vel[beetle_idx].normalized() * MAX_SPEED

@ti.kernel
def apply_beetle_turn(beetle_idx: ti.i32, turn: ti.f32):
    """
    Apply turning torque to beetle.

    Args:
        beetle_idx: Index of beetle to control
        turn: Turn amount (-1.0 to 1.0, positive = left, negative = right)
    """
    if beetle_active[beetle_idx] == 1:
        # Apply angular acceleration
        beetle_angular_vel[beetle_idx] += turn * TURN_TORQUE

        # Clamp angular speed
        if beetle_angular_vel[beetle_idx] > MAX_ANGULAR_SPEED:
            beetle_angular_vel[beetle_idx] = MAX_ANGULAR_SPEED
        elif beetle_angular_vel[beetle_idx] < -MAX_ANGULAR_SPEED:
            beetle_angular_vel[beetle_idx] = -MAX_ANGULAR_SPEED

# ============================================================
# COLLISION DETECTION
# ============================================================

@ti.kernel
def check_beetle_collisions():
    """
    Check and resolve collisions between all beetles.
    Simple sphere-sphere collision with impulse response.
    """
    for i, j in ti.ndrange(MAX_BEETLES, MAX_BEETLES):
        if i < j and beetle_active[i] == 1 and beetle_active[j] == 1:
            # Calculate distance between beetles
            diff = beetle_pos[i] - beetle_pos[j]
            dist = diff.norm()
            min_dist = beetle_radius[i] + beetle_radius[j]

            if dist < min_dist and dist > 0.001:
                # COLLISION DETECTED
                collision_normal = diff / dist
                overlap = min_dist - dist

                # Separate beetles (push apart)
                mass_total = beetle_mass[i] + beetle_mass[j]
                push_i = overlap * (beetle_mass[j] / mass_total)
                push_j = overlap * (beetle_mass[i] / mass_total)

                beetle_pos[i] += collision_normal * push_i
                beetle_pos[j] -= collision_normal * push_j

                # Calculate relative velocity
                rel_vel = beetle_vel[i] - beetle_vel[j]
                vel_along_normal = rel_vel.dot(collision_normal)

                # Only resolve if beetles moving toward each other
                if vel_along_normal < 0.0:
                    # Impulse magnitude (elastic collision)
                    restitution = 0.5  # Bounciness (0 = stick, 1 = perfect bounce)
                    impulse_magnitude = -(1.0 + restitution) * vel_along_normal / mass_total

                    impulse = collision_normal * impulse_magnitude

                    # Apply impulse to both beetles
                    beetle_vel[i] += impulse * beetle_mass[j]
                    beetle_vel[j] -= impulse * beetle_mass[i]

# ============================================================
# DEBUG / UTILITY
# ============================================================

@ti.kernel
def get_beetle_forward_vector(beetle_idx: ti.i32) -> ti.math.vec3:
    """Get the forward direction vector for a beetle"""
    rot = beetle_rotation[beetle_idx]
    return ti.math.vec3(ti.cos(rot), 0.0, ti.sin(rot))

# Initialize on module load
init_beetles()
print("Beetle Battle system initialized")
print(f"- Max beetles: {MAX_BEETLES}")
print(f"- Arena radius: {ARENA_RADIUS}m")
print(f"- Physics ready")
