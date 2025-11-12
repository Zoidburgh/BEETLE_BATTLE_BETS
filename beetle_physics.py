"""
Beetle combat with rotation and pushing physics
TGHF for blue beetle, IKJL for red beetle
"""

import taichi as ti
import simulation
import renderer
import time
import math

# Physics constants
BEETLE_RADIUS = 16.0  # Back to original scale - lean and mean
MOVE_FORCE = 120.0  # Forward movement force
BACKWARD_MOVE_FORCE = 85.0  # Backward movement force (slower retreat)
FRICTION = 0.88  # Slightly higher friction
MAX_SPEED = 7.0  # Forward top speed
BACKWARD_MAX_SPEED = 5.0  # Backward top speed (slower)
ROTATION_SPEED = 3.0  # Even slower rotation (6x slower than original) for more controlled turning
ANGULAR_FRICTION = 0.85  # How quickly spin slows down (lower = more spin)
MAX_ANGULAR_SPEED = 8.0  # Max spin speed (radians/sec)
TORQUE_MULTIPLIER = 1.95  # Rotational forces on collision
RESTITUTION = 0.0  # Bounce coefficient (0 = no bounce, 1 = full bounce)
IMPULSE_MULTIPLIER = 0.7  # Linear momentum transfer
MOMENT_OF_INERTIA_FACTOR = 1.15  # Resistance to rotation
ARENA_RADIUS = 32.0  # 25% smaller for closer combat

# Horn control constants
HORN_TILT_SPEED = 2.0  # Radians per second
HORN_DEFAULT_PITCH = math.radians(10)  # Start at +10 degrees (raised)
HORN_MAX_PITCH = math.radians(30)  # +30 degrees vertical (up) - 20° range from default
HORN_MIN_PITCH = math.radians(-10)  # -10 degrees vertical (down) - 20° range from default

# Fixed timestep physics constants
PHYSICS_TIMESTEP = 1.0 / 60.0  # 60 Hz physics update rate (16.67ms per step)
MAX_TIMESTEP_ACCUMULATOR = 0.25  # Max accumulator to prevent spiral of death

# Rendering offset - allows beetles to be visible while falling below arena
RENDER_Y_OFFSET = 33.0  # Shift voxel rendering up so Y=0 maps to grid Y=33 (128 grid, center at 64)

# Beetle state with physics
class Beetle:
    def __init__(self, x, z, rotation, color):
        self.x = x
        self.z = z
        self.vx = 0.0  # Velocity
        self.vz = 0.0
        self.y = 1.0  # Vertical position (floor level)
        self.vy = 0.0  # Vertical velocity
        self.on_ground = True  # Ground contact state
        # Yaw rotation (spin around vertical axis)
        self.rotation = rotation
        self.target_rotation = rotation
        self.angular_velocity = 0.0  # Spin speed (radians/sec)
        self.moment_of_inertia = BEETLE_RADIUS * MOMENT_OF_INERTIA_FACTOR  # Resistance to rotation (higher = more stable, less flinging)

        # Pitch rotation (forward/backward tilt)
        self.pitch = 0.0  # Body pitch angle (radians, positive = front up)
        self.pitch_velocity = 0.0  # Pitch angular velocity (radians/sec)
        self.pitch_inertia = BEETLE_RADIUS * MOMENT_OF_INERTIA_FACTOR  # Same inertia as yaw

        # Roll rotation (side-to-side tilt)
        self.roll = 0.0  # Body roll angle (radians, positive = right side up)
        self.roll_velocity = 0.0  # Roll angular velocity (radians/sec)
        self.roll_inertia = BEETLE_RADIUS * MOMENT_OF_INERTIA_FACTOR  # Same inertia as yaw
        self.color = color
        self.radius = BEETLE_RADIUS
        self.occupied_voxels = set()  # Track voxel positions for accurate collision

        # Animation state for procedural leg movement
        self.walk_phase = 0.0  # Current walk cycle phase (0 to 2π)
        self.is_moving = False  # Whether beetle is actively walking

        # Horn control state
        self.horn_pitch = HORN_DEFAULT_PITCH  # Vertical angle in radians - starts at +10°
        self.horn_yaw = 0.0    # Horizontal angle in radians (Phase 2, keep at 0.0 for now)
        self.horn_pitch_velocity = 0.0  # Rate of change of horn pitch (radians/sec)

        # Collision cooldown timers
        self.lift_cooldown = 0.0  # Time remaining before next lift can be applied (seconds)

        # Beetle active state (for fall death)
        self.active = True  # False when beetle has fallen off arena
        self.is_falling = False  # True when beetle has passed point of no return
        self.is_lifted_high = False  # True when lifted significantly above ground (for leg spaz animation)

        # Previous state for fixed timestep interpolation
        self.prev_x = x
        self.prev_y = 1.0
        self.prev_z = z
        self.prev_rotation = rotation
        self.prev_pitch = 0.0
        self.prev_roll = 0.0
        self.prev_horn_pitch = HORN_DEFAULT_PITCH

    def save_previous_state(self):
        """Save current state as previous for interpolation"""
        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_z = self.z
        self.prev_rotation = self.rotation
        self.prev_pitch = self.pitch
        self.prev_roll = self.roll
        self.prev_horn_pitch = self.horn_pitch

    def apply_force(self, fx, fz, dt):
        """Add force to velocity"""
        self.vx += fx * dt
        self.vz += fz * dt

    def update_physics(self, dt):
        """Apply friction and update position"""
        # === COLLISION COOLDOWN TIMER ===
        # Decrement lift cooldown timer (prevents multi-frame lift application)
        if self.lift_cooldown > 0.0:
            self.lift_cooldown = max(0.0, self.lift_cooldown - dt)

        # === GRAVITY SYSTEM (Phase 1) ===
        # Apply gravity (always on) - use global adjustable gravity
        self.vy -= physics_params["GRAVITY"] * dt

        # Apply vertical velocity
        self.y += self.vy * dt

        # Ground contact will be determined by floor collision detection in main loop
        # (No hardcoded floor level here - floor is detected via voxel checking)

        # === HORIZONTAL PHYSICS (existing) ===
        # Apply linear friction
        self.vx *= FRICTION
        self.vz *= FRICTION

        # Apply angular friction (yaw)
        self.angular_velocity *= ANGULAR_FRICTION

        # Apply angular friction (pitch/roll) - stronger damping for stability
        self.pitch_velocity *= ANGULAR_FRICTION * 0.9  # Extra damping
        self.roll_velocity *= ANGULAR_FRICTION * 0.9

        # Ground restoring torque - automatically level out when on ground
        if self.on_ground:
            # Apply torque to bring pitch/roll back to zero (level)
            RESTORING_STRENGTH = 15.0  # How quickly beetle levels out
            self.pitch_velocity -= self.pitch * RESTORING_STRENGTH * dt
            self.roll_velocity -= self.roll * RESTORING_STRENGTH * dt

        # Clamp linear speed (different max for forward vs backward)
        speed = math.sqrt(self.vx**2 + self.vz**2)
        if speed > 0.01:  # Avoid division by zero
            # Calculate direction beetle is moving relative to facing direction
            forward_x = math.cos(self.rotation)
            forward_z = math.sin(self.rotation)
            dot_product = self.vx * forward_x + self.vz * forward_z

            # Apply different speed caps based on direction
            if dot_product >= 0:  # Moving forward
                if speed > MAX_SPEED:
                    self.vx = (self.vx / speed) * MAX_SPEED
                    self.vz = (self.vz / speed) * MAX_SPEED
            else:  # Moving backward
                if speed > BACKWARD_MAX_SPEED:
                    self.vx = (self.vx / speed) * BACKWARD_MAX_SPEED
                    self.vz = (self.vz / speed) * BACKWARD_MAX_SPEED

        # Clamp angular speeds (yaw, pitch, roll)
        if abs(self.angular_velocity) > MAX_ANGULAR_SPEED:
            self.angular_velocity = MAX_ANGULAR_SPEED if self.angular_velocity > 0 else -MAX_ANGULAR_SPEED

        MAX_TILT_SPEED = 8.0  # Slower than yaw to prevent violent flipping
        if abs(self.pitch_velocity) > MAX_TILT_SPEED:
            self.pitch_velocity = MAX_TILT_SPEED if self.pitch_velocity > 0 else -MAX_TILT_SPEED
        if abs(self.roll_velocity) > MAX_TILT_SPEED:
            self.roll_velocity = MAX_TILT_SPEED if self.roll_velocity > 0 else -MAX_TILT_SPEED

        # Update horizontal position
        self.x += self.vx * dt
        self.z += self.vz * dt

        # Update rotation angles (yaw, pitch, roll)
        self.pitch += self.pitch_velocity * dt
        self.roll += self.roll_velocity * dt

        # Clamp pitch/roll to prevent full flips (±45 degrees)
        MAX_TILT_ANGLE = math.radians(45)
        self.pitch = max(-MAX_TILT_ANGLE, min(MAX_TILT_ANGLE, self.pitch))
        self.roll = max(-MAX_TILT_ANGLE, min(MAX_TILT_ANGLE, self.roll))

    def arena_collision(self):
        """Bounce off arena walls"""
        dist = math.sqrt(self.x**2 + self.z**2)
        if dist > ARENA_RADIUS - self.radius:
            # Push back
            angle = math.atan2(self.z, self.x)
            self.x = math.cos(angle) * (ARENA_RADIUS - self.radius)
            self.z = math.sin(angle) * (ARENA_RADIUS - self.radius)

            # Bounce velocity
            normal_x = self.x / dist
            normal_z = self.z / dist
            dot = self.vx * normal_x + self.vz * normal_z
            self.vx -= 2 * dot * normal_x * 0.5
            self.vz -= 2 * dot * normal_z * 0.5

# Game state
match_winner = None  # "BLUE" or "RED" when a beetle dies

def reset_match():
    """Reset beetles to starting positions for new match"""
    global beetle_blue, beetle_red, match_winner
    beetle_blue = Beetle(-20.0, 0.0, 0.0, simulation.BEETLE_BLUE)
    beetle_red = Beetle(20.0, 0.0, math.pi, simulation.BEETLE_RED)
    match_winner = None
    print("\n" + "="*50)
    print("NEW MATCH STARTED!")
    print("="*50 + "\n")

# Create beetles - closer together for smaller arena
beetle_blue = Beetle(-20.0, 0.0, 0.0, simulation.BEETLE_BLUE)  # Facing right (toward red)
beetle_red = Beetle(20.0, 0.0, math.pi, simulation.BEETLE_RED)  # Facing left (toward blue)

@ti.kernel
def clear_beetles():
    """Remove all beetle voxels"""
    for i, j, k in ti.ndrange(simulation.n_grid, simulation.n_grid, simulation.n_grid):
        if simulation.voxel_type[i, j, k] == simulation.BEETLE_BLUE or \
           simulation.voxel_type[i, j, k] == simulation.BEETLE_RED:
            simulation.voxel_type[i, j, k] = simulation.EMPTY

@ti.kernel
def check_collision_kernel(x1: ti.f32, z1: ti.f32, x2: ti.f32, z2: ti.f32) -> ti.i32:
    """Fast GPU-based collision check - returns 1 if collision, 0 otherwise"""
    collision = 0

    # Convert to grid coordinates
    center1_x = int(x1 + simulation.n_grid / 2.0)
    center1_z = int(z1 + simulation.n_grid / 2.0)
    center2_x = int(x2 + simulation.n_grid / 2.0)
    center2_z = int(z2 + simulation.n_grid / 2.0)

    # Early rejection: If beetles too far apart, skip voxel scanning
    dx = center1_x - center2_x
    dz = center1_z - center2_z
    dist_sq = dx * dx + dz * dz
    too_far = 0
    if dist_sq > 2704:  # (52 voxels)^2 = beyond 2x collision margin
        too_far = 1

    # Tighter collision bounds - only check where beetles can actually overlap
    # Beetles are max 26 voxels from center (max horn reach: shaft 14 + prong 7 = 21, plus diagonal tilt extension)
    x_min = ti.max(ti.max(0, center1_x - 26), ti.max(0, center2_x - 26))
    x_max = ti.min(ti.min(simulation.n_grid, center1_x + 26),
                   ti.min(simulation.n_grid, center2_x + 26))
    z_min = ti.max(ti.max(0, center1_z - 26), ti.max(0, center2_z - 26))
    z_max = ti.min(ti.min(simulation.n_grid, center1_z + 26),
                   ti.min(simulation.n_grid, center2_z + 26))

    # Scan for colliding voxels - TRUE 3D collision detection (only if not too far)
    # Track Y-ranges for each beetle in each XZ column
    for gx in range(x_min, x_max):
        for gz in range(z_min, z_max):
            # Skip remaining checks if collision already found OR beetles too far apart
            if collision == 0 and too_far == 0:
                # Track Y ranges for both beetles in this XZ column
                beetle1_y_min = 999
                beetle1_y_max = -1
                beetle2_y_min = 999
                beetle2_y_max = -1

                # Find Y-ranges for both beetles in this column (includes legs and tips!)
                # Scan around the render offset where beetles are actually placed
                # Optimized range: beetles max ~15 voxels tall (8 body + 10 legs)
                y_start = ti.max(0, int(RENDER_Y_OFFSET) - 3)
                y_end = ti.min(simulation.n_grid, int(RENDER_Y_OFFSET) + 20)
                for gy in range(y_start, y_end):
                    voxel = simulation.voxel_type[gx, gy, gz]
                    # Blue beetle voxels (body, legs, and tips)
                    if voxel == simulation.BEETLE_BLUE or voxel == simulation.BEETLE_BLUE_LEGS or voxel == simulation.LEG_TIP_BLUE:
                        if gy < beetle1_y_min:
                            beetle1_y_min = gy
                        if gy > beetle1_y_max:
                            beetle1_y_max = gy
                    # Red beetle voxels (body, legs, and tips)
                    elif voxel == simulation.BEETLE_RED or voxel == simulation.BEETLE_RED_LEGS or voxel == simulation.LEG_TIP_RED:
                        if gy < beetle2_y_min:
                            beetle2_y_min = gy
                        if gy > beetle2_y_max:
                            beetle2_y_max = gy

                # Check if Y-ranges overlap or are adjacent (within 1 voxel)
                if beetle1_y_max >= 0 and beetle2_y_max >= 0:  # Both beetles present
                    # Check if Y ranges are within 1 voxel of each other
                    if beetle1_y_min <= beetle2_y_max + 1 and beetle2_y_min <= beetle1_y_max + 1:
                        collision = 1

    return collision

@ti.kernel
def place_beetle_rotated(world_x: ti.f32, world_y: ti.f32, world_z: ti.f32, rotation: ti.f32, color_type: ti.i32, front_body_height: ti.i32, back_body_height: ti.i32):
    """LEAN 1/3 SCALE BEETLE - Exaggerated legs & horn for animation"""
    center_x = int(world_x + simulation.n_grid / 2.0)
    center_z = int(world_z + simulation.n_grid / 2.0)
    base_y = int(world_y)  # Use dynamic Y position

    cos_r = ti.cos(rotation)
    sin_r = ti.sin(rotation)

    # ========== ABDOMEN (rear) - LEAN ellipse ==========
    # Height adjusted by back_body_height parameter (range 2-6)
    for dx in range(-12, -1):
        for dy in range(0, back_body_height):
            for dz in range(-3, 4):
                # Lean elliptical shape
                length_pos = (dx + 6.5) / 5.0
                center_height = float(back_body_height - 1) / 2.0
                height_factor = 1.0 - ((dy - center_height) / center_height) ** 2
                # Use minimum width for top/bottom layers (when height_factor = 0)
                width_at_height = 3.0 * ti.sqrt(ti.max(height_factor, 0.01)) if height_factor >= 0 else 0
                length_taper = 1.0 - (length_pos * length_pos)
                max_width = width_at_height * ti.sqrt(length_taper) if length_taper > 0 else 0

                if ti.abs(dz) <= max_width:
                    local_x = float(dx)
                    local_z = float(dz)
                    rotated_x = local_x * cos_r - local_z * sin_r
                    rotated_z = local_x * sin_r + local_z * cos_r
                    grid_x = center_x + int(ti.round(rotated_x))
                    grid_y = base_y + dy
                    grid_z = center_z + int(ti.round(rotated_z))
                    if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                        simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

    # ========== THORAX (middle) - LEAN with small pronotum bump ==========
    # Height adjusted by front_body_height parameter (range 2-6)
    for dx in range(-1, 2):
        for dy in range(0, front_body_height):
            for dz in range(-2, 3):
                # Lean rounded shape
                width_at_height = 2.0 if dy < 1 else 1.5
                length_factor = 1.0 - ti.abs(dx) / 1.5
                max_width = width_at_height * length_factor

                if ti.abs(dz) <= max_width:
                    local_x = float(dx)
                    local_z = float(dz)
                    rotated_x = local_x * cos_r - local_z * sin_r
                    rotated_z = local_x * sin_r + local_z * cos_r
                    grid_x = center_x + int(ti.round(rotated_x))
                    grid_y = base_y + dy
                    grid_z = center_z + int(ti.round(rotated_z))
                    if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                        simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

    # Small thoracic bump (removed - body too tall)

    # ========== HEAD (front) - LEAN ==========
    # 2 voxels long × 3 wide × 1 tall (reduced from 2)
    for dx in range(2, 4):
        for dy in range(0, 1):
            for dz in range(-1, 2):
                width_at_height = 1.5
                if ti.abs(dz) <= width_at_height:
                    local_x = float(dx)
                    local_z = float(dz)
                    rotated_x = local_x * cos_r - local_z * sin_r
                    rotated_z = local_x * sin_r + local_z * cos_r
                    grid_x = center_x + int(ti.round(rotated_x))
                    grid_y = base_y + dy
                    grid_z = center_z + int(ti.round(rotated_z))
                    if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                        simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

    # ========== EXAGGERATED Y-SHAPED HORN - EARLY SPLIT ==========
    # Shorter main shaft - 8 voxels, curves upward, splits earlier
    for i in range(8):
        dx = 3 + i
        height_curve = int(i * 0.3)
        dy = 1 + height_curve

        # Thick solid horn
        thickness = 2

        for dz in range(-thickness, thickness + 1):
            local_x = float(dx)
            local_z = float(dz)
            rotated_x = local_x * cos_r - local_z * sin_r
            rotated_z = local_x * sin_r + local_z * cos_r
            grid_x = center_x + int(ti.round(rotated_x))
            grid_y = 2 + dy
            grid_z = center_z + int(ti.round(rotated_z))
            if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

    # LONG LEFT PRONG - 9 voxels, starts earlier, wider at tip
    for i in range(9):
        dx = 9 + i  # Start at dx=9 (overlaps shaft for solid connection)
        dy = 3 + i  # Continue upward curve
        center_dz = -i  # Angles left

        # Widen at the tip (last 3 voxels)
        if i >= 6:
            # Extra wide tip for catching
            for dz_offset in range(-2, 3):
                dz = center_dz + dz_offset
                local_x = float(dx)
                local_z = float(dz)
                rotated_x = local_x * cos_r - local_z * sin_r
                rotated_z = local_x * sin_r + local_z * cos_r
                grid_x = center_x + int(ti.round(rotated_x))
                grid_y = 2 + dy
                grid_z = center_z + int(ti.round(rotated_z))
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = color_type
        else:
            # Normal thickness for base/middle
            for dz_offset in range(-1, 2):
                dz = center_dz + dz_offset
                local_x = float(dx)
                local_z = float(dz)
                rotated_x = local_x * cos_r - local_z * sin_r
                rotated_z = local_x * sin_r + local_z * cos_r
                grid_x = center_x + int(ti.round(rotated_x))
                grid_y = 2 + dy
                grid_z = center_z + int(ti.round(rotated_z))
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

    # LONG RIGHT PRONG - 9 voxels, starts earlier, wider at tip
    for i in range(9):
        dx = 9 + i  # Start at dx=9 (overlaps shaft for solid connection)
        dy = 3 + i  # Continue upward curve
        center_dz = i  # Angles right

        # Widen at the tip (last 3 voxels)
        if i >= 6:
            # Extra wide tip for catching
            for dz_offset in range(-2, 3):
                dz = center_dz + dz_offset
                local_x = float(dx)
                local_z = float(dz)
                rotated_x = local_x * cos_r - local_z * sin_r
                rotated_z = local_x * sin_r + local_z * cos_r
                grid_x = center_x + int(ti.round(rotated_x))
                grid_y = 2 + dy
                grid_z = center_z + int(ti.round(rotated_z))
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = color_type
        else:
            # Normal thickness for base/middle
            for dz_offset in range(-1, 2):
                dz = center_dz + dz_offset
                local_x = float(dx)
                local_z = float(dz)
                rotated_x = local_x * cos_r - local_z * sin_r
                rotated_z = local_x * sin_r + local_z * cos_r
                grid_x = center_x + int(ti.round(rotated_x))
                grid_y = 2 + dy
                grid_z = center_z + int(ti.round(rotated_z))
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

    # ========== EXAGGERATED LEGS (6 total) - THICK SEGMENTS ==========
    # Front legs (longest) - attach to front thorax
    for side in ti.static([-1, 1]):
        # COXA (thick body attachment) - 2 voxels
        for i in range(2):
            dx = 1
            dy = 1
            dz = side * (2 + i)
            for extra_y in range(2):  # Make it 2 voxels tall
                local_x = float(dx)
                local_z = float(dz)
                rotated_x = local_x * cos_r - local_z * sin_r
                rotated_z = local_x * sin_r + local_z * cos_r
                grid_x = center_x + int(ti.round(rotated_x))
                grid_y = 2 + dy + extra_y
                grid_z = center_z + int(ti.round(rotated_z))
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

        # FEMUR (thick upper leg) - 3 voxels extending out
        for i in range(3):
            dx = 1
            dy = 0
            dz = side * (4 + i)
            for extra_y in range(2):  # Make it 2 voxels tall
                local_x = float(dx)
                local_z = float(dz)
                rotated_x = local_x * cos_r - local_z * sin_r
                rotated_z = local_x * sin_r + local_z * cos_r
                grid_x = center_x + int(ti.round(rotated_x))
                grid_y = 2 + dy + extra_y
                grid_z = center_z + int(ti.round(rotated_z))
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

        # TIBIA (lower leg) - 2 voxels angling down
        for i in range(2):
            dx = 1 + i
            dy = -i
            dz = side * 7
            local_x = float(dx)
            local_z = float(dz)
            rotated_x = local_x * cos_r - local_z * sin_r
            rotated_z = local_x * sin_r + local_z * cos_r
            grid_x = center_x + int(ti.round(rotated_x))
            grid_y = 2 + dy
            grid_z = center_z + int(ti.round(rotated_z))
            if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

    # Middle legs - attach to middle thorax
    for side in ti.static([-1, 1]):
        # COXA - 2 voxels
        for i in range(2):
            dx = -1
            dy = 1
            dz = side * (2 + i)
            for extra_y in range(2):
                local_x = float(dx)
                local_z = float(dz)
                rotated_x = local_x * cos_r - local_z * sin_r
                rotated_z = local_x * sin_r + local_z * cos_r
                grid_x = center_x + int(ti.round(rotated_x))
                grid_y = 2 + dy + extra_y
                grid_z = center_z + int(ti.round(rotated_z))
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

        # FEMUR - 3 voxels
        for i in range(3):
            dx = -1
            dy = 0
            dz = side * (4 + i)
            for extra_y in range(2):
                local_x = float(dx)
                local_z = float(dz)
                rotated_x = local_x * cos_r - local_z * sin_r
                rotated_z = local_x * sin_r + local_z * cos_r
                grid_x = center_x + int(ti.round(rotated_x))
                grid_y = 2 + dy + extra_y
                grid_z = center_z + int(ti.round(rotated_z))
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

        # TIBIA - 2 voxels angling down
        for i in range(2):
            dx = -1 - i
            dy = -i
            dz = side * 7
            local_x = float(dx)
            local_z = float(dz)
            rotated_x = local_x * cos_r - local_z * sin_r
            rotated_z = local_x * sin_r + local_z * cos_r
            grid_x = center_x + int(ti.round(rotated_x))
            grid_y = 2 + dy
            grid_z = center_z + int(ti.round(rotated_z))
            if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

    # Rear legs - attach to front abdomen
    for side in ti.static([-1, 1]):
        # COXA - 2 voxels
        for i in range(2):
            dx = -4
            dy = 1
            dz = side * (2 + i)
            for extra_y in range(2):
                local_x = float(dx)
                local_z = float(dz)
                rotated_x = local_x * cos_r - local_z * sin_r
                rotated_z = local_x * sin_r + local_z * cos_r
                grid_x = center_x + int(ti.round(rotated_x))
                grid_y = 2 + dy + extra_y
                grid_z = center_z + int(ti.round(rotated_z))
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

        # FEMUR - 3 voxels angling back
        for i in range(3):
            dx = -4 - i
            dy = 0
            dz = side * (4 + i)
            for extra_y in range(2):
                local_x = float(dx)
                local_z = float(dz)
                rotated_x = local_x * cos_r - local_z * sin_r
                rotated_z = local_x * sin_r + local_z * cos_r
                grid_x = center_x + int(ti.round(rotated_x))
                grid_y = 2 + dy + extra_y
                grid_z = center_z + int(ti.round(rotated_z))
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

        # TIBIA - 2 voxels angling down
        for i in range(2):
            dx = -7 - i
            dy = -i
            dz = side * 7
            local_x = float(dx)
            local_z = float(dz)
            rotated_x = local_x * cos_r - local_z * sin_r
            rotated_z = local_x * sin_r + local_z * cos_r
            grid_x = center_x + int(ti.round(rotated_x))
            grid_y = 2 + dy
            grid_z = center_z + int(ti.round(rotated_z))
            if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

# ========== PERFORMANCE OPTIMIZATION: GEOMETRY CACHING ==========
def generate_beetle_geometry(horn_shaft_len=12, horn_prong_len=5, front_body_height=4, back_body_height=5, body_length=12, body_width=7, leg_length=10):
    """Generate beetle geometry separated into body and legs for animation

    Args:
        horn_shaft_len: Length of main horn shaft (default 12 voxels)
        horn_prong_len: Length of each Y-fork prong (default 5 voxels)
        front_body_height: Height of thorax (front body) in voxel layers (default 4, range 2-6)
        back_body_height: Height of abdomen (back body) base layers (default 5, range 2-6)
        body_length: Length of abdomen in voxels (default 12, range 8-16)
        body_width: Width of body in voxels (default 7, range 5-9)
        leg_length: Total length of legs in voxels (default 10, range 6-14)
    """
    # Constrain all parameters to integers to prevent floating voxels from float arithmetic
    horn_shaft_len = int(horn_shaft_len)
    horn_prong_len = int(horn_prong_len)
    front_body_height = int(front_body_height)
    back_body_height = int(back_body_height)
    body_length = int(body_length)
    body_width = int(body_width)
    leg_length = int(leg_length)

    body_voxels = []
    leg_voxels = []  # Will be organized as list of 6 legs, each containing voxel offsets

    # ABDOMEN (rear) - LEAN ellipse
    # Height directly controlled by back_body_height parameter
    # Width directly controlled by body_width parameter
    # Pre-calculate height-dependent values for performance
    center_height = (back_body_height - 1) / 2.0
    base_width_radius = body_width / 1.75  # Scale base radius proportionally (7→4.0, 5→2.86, 9→5.14)
    width_at_height_cache = []
    for dy in range(0, back_body_height):
        height_factor = 1.0 - ((dy - center_height) / center_height) ** 2
        if height_factor >= 0:
            width_at_height_cache.append(base_width_radius * math.sqrt(max(height_factor, 0.01)))
        else:
            width_at_height_cache.append(0.0)

    # Dynamic width range based on body_width parameter
    width_half = body_width // 2
    dz_min = -width_half
    dz_max = width_half + 1

    for dx in range(-body_length, -1):
        length_pos = (dx + 6.5) / 5.0
        length_taper = 1.0 - (length_pos * length_pos)
        if length_taper > 0:
            length_taper_sqrt = math.sqrt(length_taper)  # Calculate sqrt once per dx
            for dy in range(0, back_body_height):
                width_at_height = width_at_height_cache[dy]
                if width_at_height > 0:
                    max_width = width_at_height * length_taper_sqrt
                    for dz in range(dz_min, dz_max):
                        if abs(dz) <= max_width:
                            body_voxels.append((dx, dy, dz))

    # THORAX (middle) - LEAN with small pronotum bump, scales with body_width
    # Height directly controlled by front_body_height parameter
    # Width proportional to abdomen (slightly narrower for taper)
    thorax_width = max(3, int(body_width * 5 / 7))  # 71% of abdomen width, min 3
    thorax_half = thorax_width // 2
    thorax_dz_min = -thorax_half
    thorax_dz_max = thorax_half + 1
    thorax_base_width = thorax_width / 2.0  # Scale from constrained thorax_width to prevent floating voxels

    for dx in range(-1, 2):
        for dy in range(0, front_body_height):
            for dz in range(thorax_dz_min, thorax_dz_max):
                width_at_height = thorax_base_width if dy < 1 else thorax_base_width * 0.83
                length_factor = 1.0 - abs(dx) / 1.5
                max_width = width_at_height * length_factor
                if abs(dz) <= max_width:
                    body_voxels.append((dx, dy, dz))

    # HEAD (front) - LEAN, scales with body_width (narrower taper)
    head_width = max(2, int(body_width * 3 / 7))  # 43% of abdomen width, min 2
    head_half = head_width // 2
    head_dz_min = -head_half
    head_dz_max = head_half + 1
    head_base_width = head_width / 1.25  # Scale from constrained head_width to prevent floating voxels

    for dx in range(2, 4):
        for dy in range(0, 1):
            for dz in range(head_dz_min, head_dz_max):
                if abs(dz) <= head_base_width:
                    body_voxels.append((dx, dy, dz))

    # EXAGGERATED Y-SHAPED HORN - Main shaft with overlapping layers
    shaft_segments = int(horn_shaft_len)
    for i in range(shaft_segments):
        dx = 3 + i
        height_curve = int(i * 0.3)
        dy = 1 + height_curve
        thickness = 1  # Changed from 2 to 1 for 3-voxel width
        for dz in range(-thickness, thickness + 1):
            # Add voxel at current height
            body_voxels.append((dx, dy, dz))
            # Add overlapping voxel below (unless at bottom)
            if dy > 1:
                body_voxels.append((dx, dy - 1, dz))

    # EXAGGERATED Y-fork - Left prong with overlapping layers
    prong_segments = int(horn_prong_len)
    prong_start_x = 3 + shaft_segments  # Start where shaft ends
    prong_start_y = 1 + int(shaft_segments * 0.3)  # Continue from shaft height curve

    for i in range(prong_segments):
        dx = prong_start_x + i
        dy = prong_start_y + i
        center_dz = -i
        # Use range(-1, 1) to create 2-voxel width prong on left side
        for dz_offset in range(-1, 1):
            dz = center_dz + dz_offset
            # Add voxel at current height
            body_voxels.append((dx, dy, dz))
            # Add overlapping voxel below
            body_voxels.append((dx, dy - 1, dz))

    # Right prong with overlapping layers
    for i in range(prong_segments):
        dx = prong_start_x + i
        dy = prong_start_y + i
        center_dz = i
        # Use range(0, 2) to create 2-voxel width prong on right side
        for dz_offset in range(0, 2):
            dz = center_dz + dz_offset
            # Add voxel at current height
            body_voxels.append((dx, dy, dz))
            # Add overlapping voxel below
            body_voxels.append((dx, dy - 1, dz))

    # IMPROVED LEGS (6 total) - Separated for animation
    # Leg order: [front_left, front_right, middle_left, middle_right, rear_left, rear_right]

    # Calculate leg segment lengths based on leg_length parameter
    # Total leg = coxa (20%) + femur (40%) + tibia (40%)
    coxa_len = max(1, int(leg_length * 0.2))   # Minimum 1 voxel
    femur_len = max(2, int(leg_length * 0.4))  # Minimum 2 voxels
    tibia_len = max(2, int(leg_length * 0.4))  # Minimum 2 voxels

    # Calculate Z positions for each segment
    coxa_start = 3
    femur_start = coxa_start + coxa_len
    tibia_start = femur_start + femur_len
    tibia_end = tibia_start + tibia_len

    # Front left leg (leg 0) - MOVED FORWARD 1 VOXEL
    front_left = []
    front_left_tips = []  # Separate tips for black coloring
    side = -1
    # COXA
    for i in range(coxa_len):
        for extra_y in range(2):
            front_left.append((2, 1 + extra_y, side * (coxa_start + i)))
    # FEMUR
    for i in range(femur_len):
        for extra_y in range(2):
            front_left.append((2, 0 + extra_y, side * (femur_start + i)))
    # TIBIA (tips - will be rendered black)
    for i in range(tibia_len):
        tip_x = 2 + min(i // 2, 2)  # Extend forward gradually
        front_left_tips.append((tip_x, 0, side * (tibia_start + i)))
    leg_voxels.append(front_left)

    # Front right leg (leg 1) - MOVED FORWARD 1 VOXEL
    front_right = []
    front_right_tips = []
    side = 1
    for i in range(coxa_len):
        for extra_y in range(2):
            front_right.append((2, 1 + extra_y, side * (coxa_start + i)))
    for i in range(femur_len):
        for extra_y in range(2):
            front_right.append((2, 0 + extra_y, side * (femur_start + i)))
    for i in range(tibia_len):
        tip_x = 2 + min(i // 2, 2)
        front_right_tips.append((tip_x, 0, side * (tibia_start + i)))
    leg_voxels.append(front_right)

    # Middle left leg (leg 2)
    middle_left = []
    middle_left_tips = []
    side = -1
    for i in range(coxa_len):
        for extra_y in range(2):
            middle_left.append((-2, 1 + extra_y, side * (coxa_start + i)))
    for i in range(femur_len):
        for extra_y in range(2):
            middle_left.append((-2, 0 + extra_y, side * (femur_start + i)))
    for i in range(tibia_len):
        tip_x = -2 - min(i // 2, 2)  # Extend backward gradually
        middle_left_tips.append((tip_x, 0, side * (tibia_start + i)))
    leg_voxels.append(middle_left)

    # Middle right leg (leg 3)
    middle_right = []
    middle_right_tips = []
    side = 1
    for i in range(coxa_len):
        for extra_y in range(2):
            middle_right.append((-2, 1 + extra_y, side * (coxa_start + i)))
    for i in range(femur_len):
        for extra_y in range(2):
            middle_right.append((-2, 0 + extra_y, side * (femur_start + i)))
    for i in range(tibia_len):
        tip_x = -2 - min(i // 2, 2)
        middle_right_tips.append((tip_x, 0, side * (tibia_start + i)))
    leg_voxels.append(middle_right)

    # Rear left leg (leg 4) - Position proportional to body length
    rear_leg_attach_x = -(body_length // 2)  # Proportional to abdomen length
    rear_left = []
    rear_left_tips = []
    side = -1
    for i in range(coxa_len):
        for extra_y in range(2):
            rear_left.append((rear_leg_attach_x, 1 + extra_y, side * (coxa_start + i)))
    for i in range(femur_len):
        for extra_y in range(2):
            rear_left.append((rear_leg_attach_x - i, 0 + extra_y, side * (femur_start + i)))
    for i in range(tibia_len):
        tip_x = rear_leg_attach_x - femur_len - min(i // 2, 2)  # Continue backward
        rear_left_tips.append((tip_x, 0, side * (tibia_start + i)))
    leg_voxels.append(rear_left)

    # Rear right leg (leg 5) - Position proportional to body length
    rear_right = []
    rear_right_tips = []
    side = 1
    for i in range(coxa_len):
        for extra_y in range(2):
            rear_right.append((rear_leg_attach_x, 1 + extra_y, side * (coxa_start + i)))
    for i in range(femur_len):
        for extra_y in range(2):
            rear_right.append((rear_leg_attach_x - i, 0 + extra_y, side * (femur_start + i)))
    for i in range(tibia_len):
        tip_x = rear_leg_attach_x - femur_len - min(i // 2, 2)
        rear_right_tips.append((tip_x, 0, side * (tibia_start + i)))
    leg_voxels.append(rear_right)

    # Collect all leg tips into single list (6 legs × 4 tips = 24 tip voxels)
    leg_tips = [front_left_tips, front_right_tips, middle_left_tips,
                middle_right_tips, rear_left_tips, rear_right_tips]

    return body_voxels, leg_voxels, leg_tips

# Generate beetle geometry ONCE at startup (with default params)
BEETLE_BODY, BEETLE_LEGS, BEETLE_LEG_TIPS = generate_beetle_geometry()
print(f"Beetle geometry cached: {len(BEETLE_BODY)} body voxels + {sum(len(leg) for leg in BEETLE_LEGS)} leg voxels + {sum(len(tips) for tips in BEETLE_LEG_TIPS)} tip voxels")

# Create OVERSIZED Taichi fields for body geometry cache (to allow dynamic resizing)
# Max horn: shaft=15 + prong=12 = ~27 voxels * 3 width * 2 layers = ~162 horn voxels
# Body is ~300 voxels, so max total ~500 voxels
MAX_BODY_VOXELS = 600
body_cache_size = ti.field(ti.i32, shape=())  # Track actual size
body_cache_size[None] = len(BEETLE_BODY)
body_cache_x = ti.field(ti.i32, shape=MAX_BODY_VOXELS)
body_cache_y = ti.field(ti.i32, shape=MAX_BODY_VOXELS)
body_cache_z = ti.field(ti.i32, shape=MAX_BODY_VOXELS)

# Create OVERSIZED Taichi fields for leg geometry cache (6 legs)
# Store all leg voxels flattened with offsets to know where each leg starts
# Max leg length=14: coxa(3) + femur(6) + tibia(6) = 15 voxels per segment
# Each segment has 2 height layers, so 15*2 = 30 voxels max per leg
# 6 legs × 30 voxels = 180 voxels max
MAX_LEG_VOXELS = 180
leg_cache_x = ti.field(ti.i32, shape=MAX_LEG_VOXELS)
leg_cache_y = ti.field(ti.i32, shape=MAX_LEG_VOXELS)
leg_cache_z = ti.field(ti.i32, shape=MAX_LEG_VOXELS)

# Leg start indices for each of the 6 legs
leg_start_idx = ti.field(ti.i32, shape=6)
leg_end_idx = ti.field(ti.i32, shape=6)

# Create OVERSIZED Taichi fields for leg tip geometry cache
# Max leg length=14: tibia(6) tips per leg × 6 legs = 36 tip voxels max
MAX_LEG_TIP_VOXELS = 40
leg_tip_cache_x = ti.field(ti.i32, shape=MAX_LEG_TIP_VOXELS)
leg_tip_cache_y = ti.field(ti.i32, shape=MAX_LEG_TIP_VOXELS)
leg_tip_cache_z = ti.field(ti.i32, shape=MAX_LEG_TIP_VOXELS)

# Leg tip start indices for each of the 6 legs
leg_tip_start_idx = ti.field(ti.i32, shape=6)
leg_tip_end_idx = ti.field(ti.i32, shape=6)

# Collision detection fields - store occupied voxels for each beetle (GPU-resident)
# Each beetle can occupy up to ~1600 voxels in a 40x40 area
MAX_OCCUPIED_VOXELS = 1600
beetle1_occupied_x = ti.field(ti.i32, shape=MAX_OCCUPIED_VOXELS)
beetle1_occupied_z = ti.field(ti.i32, shape=MAX_OCCUPIED_VOXELS)
beetle1_occupied_count = ti.field(ti.i32, shape=())

beetle2_occupied_x = ti.field(ti.i32, shape=MAX_OCCUPIED_VOXELS)
beetle2_occupied_z = ti.field(ti.i32, shape=MAX_OCCUPIED_VOXELS)
beetle2_occupied_count = ti.field(ti.i32, shape=())

# Collision point calculation fields
collision_point_x = ti.field(ti.f32, shape=())
collision_point_y = ti.field(ti.f32, shape=())
collision_point_z = ti.field(ti.f32, shape=())
collision_contact_count = ti.field(ti.i32, shape=())

# Dirty voxel tracking for efficient clearing
# Instead of scanning 250K voxels, track only the ~600 voxels we actually place
# Max voxels per beetle: body(600) + legs(180) + tips(40) = 820
# Two beetles: 1640, use 2000 for safety
MAX_DIRTY_VOXELS = 2000
dirty_voxel_x = ti.field(ti.i32, shape=MAX_DIRTY_VOXELS)
dirty_voxel_y = ti.field(ti.i32, shape=MAX_DIRTY_VOXELS)
dirty_voxel_z = ti.field(ti.i32, shape=MAX_DIRTY_VOXELS)
dirty_voxel_count = ti.field(ti.i32, shape=())

# Animation parameters
WALK_CYCLE_SPEED = 0.8  # Radians per second at normal walk speed (slowed down 5X)

# Copy body geometry to GPU
for i, (dx, dy, dz) in enumerate(BEETLE_BODY):
    body_cache_x[i] = dx
    body_cache_y[i] = dy
    body_cache_z[i] = dz

# Copy leg geometry to GPU
offset = 0
for leg_id, leg_voxels in enumerate(BEETLE_LEGS):
    leg_start_idx[leg_id] = offset
    for i, (dx, dy, dz) in enumerate(leg_voxels):
        leg_cache_x[offset + i] = dx
        leg_cache_y[offset + i] = dy
        leg_cache_z[offset + i] = dz
    offset += len(leg_voxels)
    leg_end_idx[leg_id] = offset

# Copy leg tip geometry to GPU
offset = 0
for leg_id, leg_tip_voxels in enumerate(BEETLE_LEG_TIPS):
    leg_tip_start_idx[leg_id] = offset
    for i, (dx, dy, dz) in enumerate(leg_tip_voxels):
        leg_tip_cache_x[offset + i] = dx
        leg_tip_cache_y[offset + i] = dy
        leg_tip_cache_z[offset + i] = dz
    offset += len(leg_tip_voxels)
    leg_tip_end_idx[leg_id] = offset

# Function to rebuild beetle geometry with new parameters
def rebuild_beetle_geometry(shaft_len, prong_len, front_body_height=4, back_body_height=5, body_length=12, body_width=7, leg_length=10):
    """Rebuild beetle geometry cache with new horn, body, and leg parameters"""
    global BEETLE_BODY, BEETLE_LEGS, BEETLE_LEG_TIPS

    # Generate new geometry
    BEETLE_BODY, BEETLE_LEGS, BEETLE_LEG_TIPS = generate_beetle_geometry(shaft_len, prong_len, front_body_height, back_body_height, body_length, body_width, leg_length)

    # Update body cache
    body_cache_size[None] = len(BEETLE_BODY)
    if len(BEETLE_BODY) > MAX_BODY_VOXELS:
        print(f"WARNING: Body voxels ({len(BEETLE_BODY)}) exceeds max ({MAX_BODY_VOXELS})!")
        return

    for i, (dx, dy, dz) in enumerate(BEETLE_BODY):
        body_cache_x[i] = dx
        body_cache_y[i] = dy
        body_cache_z[i] = dz

    # Update leg cache
    offset = 0
    for leg_id, leg_voxels in enumerate(BEETLE_LEGS):
        leg_start_idx[leg_id] = offset
        for i, (dx, dy, dz) in enumerate(leg_voxels):
            leg_cache_x[offset + i] = dx
            leg_cache_y[offset + i] = dy
            leg_cache_z[offset + i] = dz
        offset += len(leg_voxels)
        leg_end_idx[leg_id] = offset

    # Update leg tip cache
    offset = 0
    for leg_id, leg_tip_voxels in enumerate(BEETLE_LEG_TIPS):
        leg_tip_start_idx[leg_id] = offset
        for i, (dx, dy, dz) in enumerate(leg_tip_voxels):
            leg_tip_cache_x[offset + i] = dx
            leg_tip_cache_y[offset + i] = dy
            leg_tip_cache_z[offset + i] = dz
        offset += len(leg_tip_voxels)
        leg_tip_end_idx[leg_id] = offset

    print(f"Rebuilt beetle: {len(BEETLE_BODY)} body voxels (shaft={shaft_len:.0f}, prong={prong_len:.0f}, front={front_body_height:.0f}, back={back_body_height:.0f}, legs={leg_length:.0f})")

@ti.kernel
def check_floor_collision(world_x: ti.f32, world_z: ti.f32) -> ti.f32:
    """Check if beetle would collide with floor - returns highest floor Y coordinate in WORLD space"""
    center_x = int(world_x + simulation.n_grid / 2.0)
    center_z = int(world_z + simulation.n_grid / 2.0)

    # Check area around beetle (beetles are ~20 voxels wide)
    check_radius = 3
    highest_floor_y = -1000.0  # Start very low in world space

    floor_base_y = int(RENDER_Y_OFFSET)  # Floor is at offset position in grid

    for i in range(center_x - check_radius, center_x + check_radius + 1):
        for k in range(center_z - check_radius, center_z + check_radius + 1):
            if 0 <= i < simulation.n_grid and 0 <= k < simulation.n_grid:
                # Check around the expected floor height (with some margin for variations)
                for j in range(floor_base_y - 5, floor_base_y + 5):
                    if 0 <= j < simulation.n_grid:
                        if simulation.voxel_type[i, j, k] == simulation.CONCRETE:
                            # Convert grid Y back to world Y for return value
                            world_floor_y = float(j) - RENDER_Y_OFFSET
                            if world_floor_y > highest_floor_y:
                                highest_floor_y = world_floor_y

    return highest_floor_y

@ti.kernel
def calculate_beetle_lowest_point(world_y: ti.f32, rotation: ti.f32, pitch: ti.f32, roll: ti.f32, horn_pitch: ti.f32) -> ti.f32:
    """Calculate the lowest Y coordinate the beetle's geometry would occupy after rotation"""
    base_y = int(world_y)

    # Rotation matrices
    cos_yaw = ti.cos(rotation)
    sin_yaw = ti.sin(rotation)
    cos_pitch_body = ti.cos(pitch)
    sin_pitch_body = ti.sin(pitch)
    cos_roll = ti.cos(roll)
    sin_roll = ti.sin(roll)

    # Horn pitch rotation
    cos_pitch = ti.cos(horn_pitch)
    sin_pitch = ti.sin(horn_pitch)
    horn_pivot_x = 3.0
    horn_pivot_y = 1

    lowest_y = 9999.0  # Start with very high value

    # Check all body voxels
    for i in range(body_cache_size[None]):
        local_x = float(body_cache_x[i])
        local_y = body_cache_y[i]
        local_z = float(body_cache_z[i])

        # Apply horn pitch rotation if this is a horn voxel
        if body_cache_x[i] >= 3:
            rel_x = local_x - horn_pivot_x
            rel_y = float(local_y - horn_pivot_y)
            pitched_x = rel_x * cos_pitch - rel_y * sin_pitch
            pitched_y = rel_x * sin_pitch + rel_y * cos_pitch
            local_x = pitched_x + horn_pivot_x
            local_y = int(ti.round(pitched_y + float(horn_pivot_y)))

        # Apply 3D rotation (yaw -> pitch -> roll)
        ly = float(local_y)

        # Yaw
        temp_x = local_x * cos_yaw - local_z * sin_yaw
        temp_z = local_x * sin_yaw + local_z * cos_yaw
        temp_y = ly

        # Pitch
        temp2_x = temp_x * cos_pitch_body - temp_y * sin_pitch_body
        temp2_y = temp_x * sin_pitch_body + temp_y * cos_pitch_body
        temp2_z = temp_z

        # Roll
        final_y = temp2_y * cos_roll - temp2_z * sin_roll

        grid_y = float(base_y) + final_y
        if grid_y < lowest_y:
            lowest_y = grid_y

    # Check all leg voxels
    for leg_id in range(6):
        start_idx = leg_start_idx[leg_id]
        end_idx = leg_end_idx[leg_id]

        for i in range(start_idx, end_idx):
            local_x = float(leg_cache_x[i])
            local_y = float(leg_cache_y[i])
            local_z = float(leg_cache_z[i])

            # Apply 3D rotation (same as body)
            ly = local_y

            # Yaw
            temp_x = local_x * cos_yaw - local_z * sin_yaw
            temp_z = local_x * sin_yaw + local_z * cos_yaw
            temp_y = ly

            # Pitch
            temp2_x = temp_x * cos_pitch_body - temp_y * sin_pitch_body
            temp2_y = temp_x * sin_pitch_body + temp_y * cos_pitch_body
            temp2_z = temp_z

            # Roll
            final_y = temp2_y * cos_roll - temp2_z * sin_roll

            grid_y = float(base_y) + final_y
            if grid_y < lowest_y:
                lowest_y = grid_y

    # Check all leg tip voxels (these are typically the lowest points)
    for leg_id in range(6):
        tip_start_idx = leg_tip_start_idx[leg_id]
        tip_end_idx = leg_tip_end_idx[leg_id]

        for i in range(tip_start_idx, tip_end_idx):
            local_x = float(leg_tip_cache_x[i])
            local_y = float(leg_tip_cache_y[i])
            local_z = float(leg_tip_cache_z[i])

            # Apply 3D rotation (same as legs)
            ly = local_y

            # Yaw
            temp_x = local_x * cos_yaw - local_z * sin_yaw
            temp_z = local_x * sin_yaw + local_z * cos_yaw
            temp_y = ly

            # Pitch
            temp2_x = temp_x * cos_pitch_body - temp_y * sin_pitch_body
            temp2_y = temp_x * sin_pitch_body + temp_y * cos_pitch_body
            temp2_z = temp_z

            # Roll
            final_y = temp2_y * cos_roll - temp2_z * sin_roll

            grid_y = float(base_y) + final_y
            if grid_y < lowest_y:
                lowest_y = grid_y

    return lowest_y

@ti.kernel
def place_animated_beetle(world_x: ti.f32, world_y: ti.f32, world_z: ti.f32, rotation: ti.f32, pitch: ti.f32, roll: ti.f32, horn_pitch: ti.f32, body_color: ti.i32, leg_color: ti.i32, leg_tip_color: ti.i32, walk_phase: ti.f32, is_lifted_high: ti.i32):
    """Beetle placement with 3D rotation (yaw/pitch/roll) and animated legs"""
    center_x = int(world_x + simulation.n_grid / 2.0)
    center_z = int(world_z + simulation.n_grid / 2.0)
    base_y = int(world_y + RENDER_Y_OFFSET)  # Apply Y offset for rendering below floor

    # Rotation matrices for yaw, pitch, roll
    cos_yaw = ti.cos(rotation)
    sin_yaw = ti.sin(rotation)
    cos_pitch_body = ti.cos(pitch)
    sin_pitch_body = ti.sin(pitch)
    cos_roll = ti.cos(roll)
    sin_roll = ti.sin(roll)

    # Pitch rotation for horn (rotation around Z-axis in local space)
    cos_pitch = ti.cos(horn_pitch)
    sin_pitch = ti.sin(horn_pitch)

    # Horn pivot point in local coordinates (where horn attaches to head)
    horn_pivot_x = 3.0
    horn_pivot_y = 1

    # 1. Place body with horn pitch applied
    for i in range(body_cache_size[None]):
        local_x = float(body_cache_x[i])
        local_y = body_cache_y[i]
        local_z = float(body_cache_z[i])

        # Apply horn pitch rotation ONLY to horn voxels (dx >= 3)
        if body_cache_x[i] >= 3:
            # This is a horn voxel - apply pitch rotation around pivot point
            # Translate to pivot, rotate, translate back
            rel_x = local_x - horn_pivot_x
            rel_y = float(local_y - horn_pivot_y)

            # Pitch rotation (around Z-axis in beetle local space, before body rotation)
            # Positive pitch = horn rotates up
            pitched_x = rel_x * cos_pitch - rel_y * sin_pitch
            pitched_y = rel_x * sin_pitch + rel_y * cos_pitch

            # Translate back from pivot
            local_x = pitched_x + horn_pivot_x
            local_y = int(ti.round(pitched_y + float(horn_pivot_y)))

        # 3D rotation: Apply yaw → pitch → roll (standard rotation order)
        # Convert local_y to float for rotation
        ly = float(local_y)

        # Step 1: Yaw rotation (around Y-axis)
        temp_x = local_x * cos_yaw - local_z * sin_yaw
        temp_z = local_x * sin_yaw + local_z * cos_yaw
        temp_y = ly

        # Step 2: Pitch rotation (around Z-axis) - nose up/down
        temp2_x = temp_x * cos_pitch_body - temp_y * sin_pitch_body
        temp2_y = temp_x * sin_pitch_body + temp_y * cos_pitch_body
        temp2_z = temp_z

        # Step 3: Roll rotation (around X-axis) - tilt left/right
        final_x = temp2_x
        final_y = temp2_y * cos_roll - temp2_z * sin_roll
        final_z = temp2_y * sin_roll + temp2_z * cos_roll

        grid_x = center_x + int(ti.round(final_x))
        grid_y = base_y + int(ti.round(final_y))
        grid_z = center_z + int(ti.round(final_z))

        if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid and 0 <= grid_y < simulation.n_grid:
            # Don't overwrite floor (CONCRETE) voxels
            if simulation.voxel_type[grid_x, grid_y, grid_z] != simulation.CONCRETE:
                simulation.voxel_type[grid_x, grid_y, grid_z] = body_color
                # Track this voxel for efficient clearing later
                idx = ti.atomic_add(dirty_voxel_count[None], 1)
                if idx < MAX_DIRTY_VOXELS:
                    dirty_voxel_x[idx] = grid_x
                    dirty_voxel_y[idx] = grid_y
                    dirty_voxel_z[idx] = grid_z

    # 2. Place legs (animated with tripod gait) - DIFFERENT COLOR
    # Tripod gait: legs 0, 3, 4 move together (Group A), legs 1, 2, 5 move together (Group B)
    # leg_id: 0=front_left, 1=front_right, 2=middle_left, 3=middle_right, 4=rear_left, 5=rear_right

    for leg_id in range(6):
        # Determine leg phase offset for tripod gait
        # Group A (0, 3, 4): phase_offset = 0
        # Group B (1, 2, 5): phase_offset = π
        phase_offset = 0.0
        if leg_id == 1 or leg_id == 2 or leg_id == 5:
            phase_offset = 3.14159265359  # π

        leg_phase = walk_phase + phase_offset

        # Calculate animation transforms (vertical lift and minimal forward/back sweep)
        # sin(leg_phase) ranges from -1 to 1
        # We want: lift when sin > 0 (leg in air), on ground when sin <= 0
        lift = ti.max(0.0, ti.sin(leg_phase)) * 2.5  # Lift up to 2.5 voxels

        # Subtle forward/back sweep (1 voxel range) - inverted so legs push backward when grounded
        sweep = -ti.cos(leg_phase) * 0.5  # ±0.5 voxel sweep (negative = correct walk direction)

        # SPAZ WIGGLE: When beetle is lifted high, add chaotic leg movement
        if is_lifted_high == 1:
            # High-frequency wiggle with per-leg variation for chaos
            wiggle_freq = 5.0  # Slowed down 3x (was 15.0)
            wiggle_phase = leg_phase * wiggle_freq + float(leg_id)  # Each leg different

            # Add erratic movement to lift and sweep
            wiggle_lift = ti.sin(wiggle_phase) * 2.0  # ±2 voxels extra lift
            wiggle_sweep = ti.cos(wiggle_phase * 1.3) * 0.8  # ±0.8 voxels extra sweep

            lift += wiggle_lift
            sweep += wiggle_sweep

        # Place all voxels for this leg
        start_idx = leg_start_idx[leg_id]
        end_idx = leg_end_idx[leg_id]

        for i in range(start_idx, end_idx):
            local_x = float(leg_cache_x[i]) + sweep  # Apply sweep
            local_y = leg_cache_y[i] + int(lift)  # Apply vertical lift
            local_z = float(leg_cache_z[i])

            # 3D rotation (same as body)
            ly = float(local_y)

            # Yaw
            temp_x = local_x * cos_yaw - local_z * sin_yaw
            temp_z = local_x * sin_yaw + local_z * cos_yaw
            temp_y = ly

            # Pitch
            temp2_x = temp_x * cos_pitch_body - temp_y * sin_pitch_body
            temp2_y = temp_x * sin_pitch_body + temp_y * cos_pitch_body
            temp2_z = temp_z

            # Roll
            final_x = temp2_x
            final_y = temp2_y * cos_roll - temp2_z * sin_roll
            final_z = temp2_y * sin_roll + temp2_z * cos_roll

            grid_x = center_x + int(ti.round(final_x))
            grid_y = base_y + int(ti.round(final_y))
            grid_z = center_z + int(ti.round(final_z))

            if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid and 0 <= grid_y < simulation.n_grid:
                # Don't overwrite floor (CONCRETE) voxels
                if simulation.voxel_type[grid_x, grid_y, grid_z] != simulation.CONCRETE:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = leg_color
                    # Track this voxel for efficient clearing later
                    idx = ti.atomic_add(dirty_voxel_count[None], 1)
                    if idx < MAX_DIRTY_VOXELS:
                        dirty_voxel_x[idx] = grid_x
                        dirty_voxel_y[idx] = grid_y
                        dirty_voxel_z[idx] = grid_z

        # Place leg tips (same animation as legs, but BLACK color for visibility)
        tip_start_idx = leg_tip_start_idx[leg_id]
        tip_end_idx = leg_tip_end_idx[leg_id]

        for i in range(tip_start_idx, tip_end_idx):
            local_x = float(leg_tip_cache_x[i]) + sweep  # Apply same animation
            local_y = leg_tip_cache_y[i] + int(lift)
            local_z = float(leg_tip_cache_z[i])

            # 3D rotation (same as legs)
            ly = float(local_y)

            # Yaw
            temp_x = local_x * cos_yaw - local_z * sin_yaw
            temp_z = local_x * sin_yaw + local_z * cos_yaw
            temp_y = ly

            # Pitch
            temp2_x = temp_x * cos_pitch_body - temp_y * sin_pitch_body
            temp2_y = temp_x * sin_pitch_body + temp_y * cos_pitch_body
            temp2_z = temp_z

            # Roll
            final_x = temp2_x
            final_y = temp2_y * cos_roll - temp2_z * sin_roll
            final_z = temp2_y * sin_roll + temp2_z * cos_roll

            grid_x = center_x + int(ti.round(final_x))
            grid_y = base_y + int(ti.round(final_y))
            grid_z = center_z + int(ti.round(final_z))

            if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid and 0 <= grid_y < simulation.n_grid:
                # Don't overwrite floor (CONCRETE) voxels
                if simulation.voxel_type[grid_x, grid_y, grid_z] != simulation.CONCRETE:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = leg_tip_color
                    # Track this voxel for efficient clearing later
                    idx = ti.atomic_add(dirty_voxel_count[None], 1)
                    if idx < MAX_DIRTY_VOXELS:
                        dirty_voxel_x[idx] = grid_x
                        dirty_voxel_y[idx] = grid_y
                        dirty_voxel_z[idx] = grid_z

@ti.kernel
def reset_dirty_voxels():
    """Reset dirty voxel counter before placing beetles"""
    dirty_voxel_count[None] = 0

@ti.kernel
def clear_dirty_voxels():
    """Clear only the voxels we tracked during placement (400x faster than scanning 250K voxels)"""
    for i in range(dirty_voxel_count[None]):
        vtype = simulation.voxel_type[dirty_voxel_x[i], dirty_voxel_y[i], dirty_voxel_z[i]]
        # Only clear beetle voxels (not floor/concrete)
        if vtype >= simulation.BEETLE_BLUE:  # All beetle colors are >= BEETLE_BLUE
            simulation.voxel_type[dirty_voxel_x[i], dirty_voxel_y[i], dirty_voxel_z[i]] = simulation.EMPTY

@ti.kernel
def clear_beetles_bounded(x1: ti.f32, y1: ti.f32, z1: ti.f32, x2: ti.f32, y2: ti.f32, z2: ti.f32):
    """Clear beetles using SEPARATE bounding boxes for each beetle"""
    # Max horn reach: 22 voxels (shaft 14 + prong 7 = 21, plus diagonal extension)
    # Need margin to cover full horn reach in all directions
    margin = 26  # Covers max reach (same as collision detection margin)

    # === BEETLE 1 BOUNDING BOX ===
    center_x1 = int(x1 + simulation.n_grid / 2.0)
    center_z1 = int(z1 + simulation.n_grid / 2.0)

    min_x1 = ti.max(0, center_x1 - margin)
    max_x1 = ti.min(simulation.n_grid, center_x1 + margin)
    min_z1 = ti.max(0, center_z1 - margin)
    max_z1 = ti.min(simulation.n_grid, center_z1 + margin)

    y_margin = 23  # Covers beetle height (~15 voxels) + safety
    min_y1 = ti.max(0, int(y1 + RENDER_Y_OFFSET) - y_margin)
    max_y1 = ti.min(simulation.n_grid, int(y1 + RENDER_Y_OFFSET) + y_margin)

    # Clear beetle 1's area
    for i in range(min_x1, max_x1):
        for j in range(min_y1, max_y1):
            for k in range(min_z1, max_z1):
                vtype = simulation.voxel_type[i, j, k]
                if vtype == simulation.BEETLE_BLUE or vtype == simulation.BEETLE_RED or \
                   vtype == simulation.BEETLE_BLUE_LEGS or vtype == simulation.BEETLE_RED_LEGS or \
                   vtype == simulation.LEG_TIP_BLUE or vtype == simulation.LEG_TIP_RED:
                    simulation.voxel_type[i, j, k] = simulation.EMPTY

    # === BEETLE 2 BOUNDING BOX ===
    center_x2 = int(x2 + simulation.n_grid / 2.0)
    center_z2 = int(z2 + simulation.n_grid / 2.0)

    min_x2 = ti.max(0, center_x2 - margin)
    max_x2 = ti.min(simulation.n_grid, center_x2 + margin)
    min_z2 = ti.max(0, center_z2 - margin)
    max_z2 = ti.min(simulation.n_grid, center_z2 + margin)

    min_y2 = ti.max(0, int(y2 + RENDER_Y_OFFSET) - y_margin)
    max_y2 = ti.min(simulation.n_grid, int(y2 + RENDER_Y_OFFSET) + y_margin)

    # Clear beetle 2's area
    for i in range(min_x2, max_x2):
        for j in range(min_y2, max_y2):
            for k in range(min_z2, max_z2):
                vtype = simulation.voxel_type[i, j, k]
                if vtype == simulation.BEETLE_BLUE or vtype == simulation.BEETLE_RED or \
                   vtype == simulation.BEETLE_BLUE_LEGS or vtype == simulation.BEETLE_RED_LEGS or \
                   vtype == simulation.LEG_TIP_BLUE or vtype == simulation.LEG_TIP_RED:
                    simulation.voxel_type[i, j, k] = simulation.EMPTY

@ti.kernel
def calculate_occupied_voxels_kernel(world_x: ti.f32, world_z: ti.f32, beetle_color: ti.i32,
                                     occupied_x: ti.template(), occupied_z: ti.template(),
                                     occupied_count: ti.template()):
    """GPU-accelerated calculation of occupied voxels for collision detection"""
    center_x = int(world_x + simulation.n_grid / 2.0)
    center_z = int(world_z + simulation.n_grid / 2.0)

    # Reset count
    occupied_count[None] = 0

    # Scan area around beetle (40x40 area, height range based on render offset)
    y_base = int(RENDER_Y_OFFSET)
    x_min = ti.max(0, center_x - 20)
    x_max = ti.min(simulation.n_grid, center_x + 20)
    z_min = ti.max(0, center_z - 20)
    z_max = ti.min(simulation.n_grid, center_z + 20)
    y_min = ti.max(0, y_base - 5)
    y_max = ti.min(simulation.n_grid, y_base + 30)

    # Scan voxel grid on GPU
    for i in range(x_min, x_max):
        for k in range(z_min, z_max):
            found_in_column = 0
            for j in range(y_min, y_max):
                vtype = simulation.voxel_type[i, j, k]
                # Check if voxel belongs to target beetle
                if beetle_color == simulation.BEETLE_BLUE:
                    if vtype == simulation.BEETLE_BLUE or vtype == simulation.BEETLE_BLUE_LEGS or vtype == simulation.LEG_TIP_BLUE:
                        found_in_column = 1
                else:  # BEETLE_RED
                    if vtype == simulation.BEETLE_RED or vtype == simulation.BEETLE_RED_LEGS or vtype == simulation.LEG_TIP_RED:
                        found_in_column = 1

            # Add to occupied list if found (atomic increment to avoid race conditions)
            if found_in_column == 1:
                idx = ti.atomic_add(occupied_count[None], 1)
                if idx < MAX_OCCUPIED_VOXELS:
                    occupied_x[idx] = i
                    occupied_z[idx] = k

@ti.kernel
def calculate_collision_point_kernel(overlap_count: ti.i32):
    """GPU-accelerated calculation of 3D collision point from overlapping voxels"""
    # Reset collision data
    collision_point_x[None] = 0.0
    collision_point_y[None] = 0.0
    collision_point_z[None] = 0.0
    collision_contact_count[None] = 0

    # Scan through overlapping voxel columns
    y_scan_start = ti.max(0, int(RENDER_Y_OFFSET) - 5)
    y_scan_end = ti.min(simulation.n_grid, int(RENDER_Y_OFFSET) + 35)

    # Check overlap between beetles using stored occupied voxels
    for i in range(beetle1_occupied_count[None]):
        vx1 = beetle1_occupied_x[i]
        vz1 = beetle1_occupied_z[i]

        for j in range(beetle2_occupied_count[None]):
            vx2 = beetle2_occupied_x[j]
            vz2 = beetle2_occupied_z[j]

            # If same (x, z) position, we have overlap
            if vx1 == vx2 and vz1 == vz2:
                # Scan vertically to find all contact heights
                for vy_grid in range(y_scan_start, y_scan_end):
                    vtype = simulation.voxel_type[vx1, vy_grid, vz1]
                    if vtype != 0:  # Non-empty voxel
                        # Accumulate collision position
                        ti.atomic_add(collision_point_x[None], float(vx1) - simulation.n_grid / 2.0)
                        ti.atomic_add(collision_point_z[None], float(vz1) - simulation.n_grid / 2.0)
                        ti.atomic_add(collision_point_y[None], float(vy_grid) - RENDER_Y_OFFSET)
                        ti.atomic_add(collision_contact_count[None], 1)


def beetle_collision(b1, b2, params):
    """Handle collision with voxel-perfect detection, pushing, and horn leverage"""
    # Fast GPU-based collision check
    has_collision = check_collision_kernel(b1.x, b1.z, b2.x, b2.z)

    if has_collision:
        # GPU-ACCELERATED: Calculate occupied voxels on GPU (no CPU transfer!)
        calculate_occupied_voxels_kernel(b1.x, b1.z, simulation.BEETLE_BLUE,
                                        beetle1_occupied_x, beetle1_occupied_z, beetle1_occupied_count)
        calculate_occupied_voxels_kernel(b2.x, b2.z, simulation.BEETLE_RED,
                                        beetle2_occupied_x, beetle2_occupied_z, beetle2_occupied_count)

        # GPU-ACCELERATED: Calculate collision point on GPU (no CPU transfer!)
        calculate_collision_point_kernel(0)  # overlap_count not used in kernel

        # Read results from GPU (minimal data transfer - just 4 values!)
        collision_x = collision_point_x[None]
        collision_y = collision_point_y[None]
        collision_z = collision_point_z[None]
        contact_count = collision_contact_count[None]

        # Collision detected! Calculate 3D collision geometry
        dx = b1.x - b2.x
        dz = b1.z - b2.z
        dist = math.sqrt(dx**2 + dz**2)

        if dist > 0.001:
            # Use GPU-calculated collision point or fallback to midpoint
            if contact_count > 0:
                collision_x /= contact_count
                collision_z /= contact_count
                collision_y /= contact_count
            else:
                collision_x = (b1.x + b2.x) / 2.0
                collision_z = (b1.z + b2.z) / 2.0
                collision_y = (b1.y + b2.y) / 2.0

            # Calculate 3D collision normal (from b2 to b1)
            # Include vertical component based on contact geometry
            dy = b1.y - b2.y
            dist_3d = math.sqrt(dx**2 + dy**2 + dz**2)

            if dist_3d > 0.001:
                normal_x = dx / dist_3d
                normal_y = dy / dist_3d
                normal_z = dz / dist_3d
            else:
                # Fallback to horizontal normal
                normal_x = dx / dist
                normal_y = 0.0
                normal_z = dz / dist

            # HORN LEVERAGE: Strong vertical lift when contact is high (horn collision)
            center_y = (b1.y + b2.y) / 2.0
            contact_height_above_center = collision_y - center_y
            is_horn_contact = contact_height_above_center > 2.0  # Horn contact (above body center)

            if is_horn_contact:
                # Scale up vertical component based on height - logarithmic scaling for diminishing returns
                raw_leverage = contact_height_above_center / 3.0
                horn_leverage = min(math.log(raw_leverage + 1.0) * 2.0, 2.5)

                # Add MASSIVE upward bias to the collision normal
                normal_y += horn_leverage * 2.5  # 5x stronger than before!

                # Re-normalize
                norm_len = math.sqrt(normal_x**2 + normal_y**2 + normal_z**2)
                if norm_len > 0.001:
                    normal_x /= norm_len
                    normal_y /= norm_len
                    normal_z /= norm_len

                # SIMPLIFIED SYMMETRIC LIFTING - PURE VELOCITY (for debugging)
                # Whoever is pressing their lift key (R/U) lifts the opponent
                lift_impulse = horn_leverage * 30.0  # Upward kick

                # Calculate where the collision is relative to each beetle's center
                collision_above_b1 = collision_y - b1.y
                collision_above_b2 = collision_y - b2.y

                # PURE VELOCITY ADVANTAGE - who's actively pressing R or U?
                # Positive velocity = tilting horn UP = you lift opponent
                # Blue pressing R: b1.horn_pitch_velocity = +2.0
                # Red pressing U: b2.horn_pitch_velocity = +2.0
                lift_advantage = b1.horn_pitch_velocity - b2.horn_pitch_velocity

                # DEBUG: Print collision info
                # print(f"HORN COLLISION:")
                # print(f"  Blue: vel={b1.horn_pitch_velocity:.2f}, y={b1.y:.1f}")
                # print(f"  Red:  vel={b2.horn_pitch_velocity:.2f}, y={b2.y:.1f}")
                # print(f"  Lift advantage: {lift_advantage:.2f} (>0 = Blue lifts Red, <0 = Red lifts Blue)")

                # Very low threshold - almost any velocity difference triggers
                ADVANTAGE_THRESHOLD = 0.5
                LIFT_COOLDOWN_DURATION = 0.1  # Seconds between lift applications

                # Check if both beetles are off cooldown before applying lift forces
                if b1.lift_cooldown <= 0.0 and b2.lift_cooldown <= 0.0:
                    # Cooldown expired - can apply lift force
                    if lift_advantage > ADVANTAGE_THRESHOLD:
                        # Blue has advantage - lifts red
                        # print(f"  -> BLUE lifts RED!")
                        lift_force = lift_impulse * 0.195
                        b2.vy += lift_force  # Red gets lifted HIGHER
                        b1.vy -= lift_impulse * 0.03  # Blue pushes down (reaction)

                        # TORQUE: Apply rotation from off-center force
                        # Calculate lever arm from red's center to collision point
                        lever_x = collision_x - b2.x  # X offset (causes roll)
                        lever_z = collision_z - b2.z  # Z offset (causes pitch)

                        # Pitch torque: force in +Y at position +Z causes nose-up pitch
                        pitch_torque = -lever_z * lift_force  # Negative Z = positive pitch
                        b2.pitch_velocity += pitch_torque / b2.pitch_inertia

                        # Roll torque: force in +Y at position +X causes right-side-up roll
                        roll_torque = lever_x * lift_force
                        b2.roll_velocity += roll_torque / b2.roll_inertia

                        # Set cooldown for both beetles
                        b1.lift_cooldown = LIFT_COOLDOWN_DURATION
                        b2.lift_cooldown = LIFT_COOLDOWN_DURATION

                    elif lift_advantage < -ADVANTAGE_THRESHOLD:
                        # Red has advantage - lifts blue
                        # print(f"  -> RED lifts BLUE!")
                        lift_force = lift_impulse * 0.195
                        b1.vy += lift_force  # Blue gets lifted HIGHER
                        b2.vy -= lift_impulse * 0.03  # Red pushes down (reaction)

                        # TORQUE: Apply rotation from off-center force
                        lever_x = collision_x - b1.x  # X offset (causes roll)
                        lever_z = collision_z - b1.z  # Z offset (causes pitch)

                        # Pitch torque
                        pitch_torque = -lever_z * lift_force
                        b1.pitch_velocity += pitch_torque / b1.pitch_inertia

                        # Roll torque
                        roll_torque = lever_x * lift_force
                        b1.roll_velocity += roll_torque / b1.roll_inertia

                        # Set cooldown for both beetles
                        b1.lift_cooldown = LIFT_COOLDOWN_DURATION
                        b2.lift_cooldown = LIFT_COOLDOWN_DURATION

                    else:
                        # Evenly matched - both get pushed (with torque)
                        # print(f"  -> BOTH beetles pushed!")
                        push_force = lift_impulse * 0.06
                        b1.vy += push_force
                        b2.vy += push_force

                        # Apply torque to both
                        lever_x1 = collision_x - b1.x
                        lever_z1 = collision_z - b1.z
                        b1.pitch_velocity += (-lever_z1 * push_force) / b1.pitch_inertia
                        b1.roll_velocity += (lever_x1 * push_force) / b1.roll_inertia

                        lever_x2 = collision_x - b2.x
                        lever_z2 = collision_z - b2.z
                        b2.pitch_velocity += (-lever_z2 * push_force) / b2.pitch_inertia
                        b2.roll_velocity += (lever_x2 * push_force) / b2.roll_inertia

                        # Set cooldown for both beetles
                        b1.lift_cooldown = LIFT_COOLDOWN_DURATION
                        b2.lift_cooldown = LIFT_COOLDOWN_DURATION
                else:
                    # Cooldown active - skip lift force but still print debug info
                    pass  # print(f"  -> COOLDOWN ACTIVE (Blue: {b1.lift_cooldown:.3f}s, Red: {b2.lift_cooldown:.3f}s)")

                # Add horizontal spin based on where horn hit BOTH beetles
                # Beetle being lifted spins
                dx_to_b1 = collision_x - b1.x
                dz_to_b1 = collision_z - b1.z
                torque_b1 = dx_to_b1 * normal_z - dz_to_b1 * normal_x
                angular_impulse_b1 = (torque_b1 / b1.moment_of_inertia) * horn_leverage * 2.0
                b1.angular_velocity += angular_impulse_b1

                dx_to_b2 = collision_x - b2.x
                dz_to_b2 = collision_z - b2.z
                torque_b2 = dx_to_b2 * normal_z - dz_to_b2 * normal_x
                angular_impulse_b2 = (torque_b2 / b2.moment_of_inertia) * horn_leverage * 2.0
                b2.angular_velocity -= angular_impulse_b2  # Opposite direction

            # STRONG separation to prevent stuck collisions (now 3D!)
            separation_force = params["SEPARATION_FORCE"]  # Tunable via slider
            b1.x += normal_x * separation_force
            b1.z += normal_z * separation_force
            b2.x -= normal_x * separation_force
            b2.z -= normal_z * separation_force

            # VERTICAL SEPARATION - only when both beetles are airborne
            # This prevents floor voxel destruction and maintains symmetry
            if not b1.on_ground and not b2.on_ground:
                b1.y += normal_y * separation_force
                b2.y -= normal_y * separation_force

            # Safety clamp to prevent going below floor voxel layer
            # (Main floor collision handles proper positioning above floor)
            MIN_Y = 0.5  # Minimum Y to prevent center going below floor at y=0
            if b1.y < MIN_Y:
                b1.y = MIN_Y
                b1.vy = 0.0
            if b2.y < MIN_Y:
                b2.y = MIN_Y
                b2.vy = 0.0

            # Calculate 3D relative velocity
            rel_vx = b1.vx - b2.vx
            rel_vy = b1.vy - b2.vy
            rel_vz = b1.vz - b2.vz
            vel_along_normal = rel_vx * normal_x + rel_vy * normal_y + rel_vz * normal_z

            # Only apply impulse if moving toward each other
            if vel_along_normal < 0:
                # Impulse magnitude (very low restitution for minimal bouncing)
                impulse = -(1 + params["RESTITUTION"]) * vel_along_normal * params["IMPULSE_MULTIPLIER"]

                # Apply 3D linear impulse
                impulse_x = impulse * normal_x
                impulse_z = impulse * normal_z

                # For horn contact, lifting logic handles vertical forces explicitly
                # Only apply vertical impulse for non-horn collisions
                if is_horn_contact:
                    impulse_y = 0.0  # Disable vertical impulse - lifting logic handles it
                else:
                    impulse_y = impulse * normal_y

                b1.vx += impulse_x
                b1.vy += impulse_y
                b1.vz += impulse_z
                b2.vx -= impulse_x
                b2.vy -= impulse_y
                b2.vz -= impulse_z

                # Calculate and apply torque (angular impulse)
                # Torque = r × F (cross product in 2D: rx*Fz - rz*Fx)

                # For beetle 1: collision point relative to its center
                r1_x = collision_x - b1.x
                r1_z = collision_z - b1.z
                torque1 = r1_x * impulse_z - r1_z * impulse_x
                angular_impulse1 = (torque1 / b1.moment_of_inertia) * params["TORQUE_MULTIPLIER"]
                b1.angular_velocity += angular_impulse1

                # For beetle 2: collision point relative to its center
                r2_x = collision_x - b2.x
                r2_z = collision_z - b2.z
                torque2 = r2_x * (-impulse_z) - r2_z * (-impulse_x)
                angular_impulse2 = (torque2 / b2.moment_of_inertia) * params["TORQUE_MULTIPLIER"]
                b2.angular_velocity += angular_impulse2

def normalize_angle(angle):
    """Normalize angle to [0, 2π)"""
    while angle < 0:
        angle += 2 * math.pi
    while angle >= 2 * math.pi:
        angle -= 2 * math.pi
    return angle

def shortest_rotation(current, target):
    """Find shortest rotation from current to target"""
    current = normalize_angle(current)
    target = normalize_angle(target)
    diff = target - current
    if diff > math.pi:
        diff -= 2 * math.pi
    elif diff < -math.pi:
        diff += 2 * math.pi
    return diff

# Window
window = ti.ui.Window("Beetle Physics", (1920, 1080), vsync=False)
canvas = window.get_canvas()
scene = window.get_scene()

camera = renderer.Camera()
camera.pos_x = 0.0
camera.pos_y = 60.0
camera.pos_z = 0.0
camera.pitch = -70.0
camera.yaw = 0.0

print("\n=== BEETLE PHYSICS ===")
print("BLUE BEETLE (TFGH + RY) - Tank Controls:")
print("  T - Move Forward")
print("  G - Move Backward")
print("  F - Rotate Left")
print("  H - Rotate Right")
print("  R - Tilt Horn Up (+20°)")
print("  Y - Tilt Horn Down (-20°)")
print("")
print("RED BEETLE (IJKL + UO) - Tank Controls:")
print("  I - Move Forward")
print("  K - Move Backward")
print("  J - Rotate Left")
print("  L - Rotate Right")
print("  U - Tilt Horn Up (+20°)")
print("  O - Tilt Horn Down (-20°)")
print("")
print("Push beetles together!")
print("Camera: WASD/Mouse/Q/E")
print("="*40 + "\n")

# Physics parameters dictionary (mutable for sliders)
physics_params = {
    "TORQUE_MULTIPLIER": TORQUE_MULTIPLIER,
    "IMPULSE_MULTIPLIER": IMPULSE_MULTIPLIER,
    "RESTITUTION": RESTITUTION,
    "MOMENT_OF_INERTIA_FACTOR": MOMENT_OF_INERTIA_FACTOR,
    "GRAVITY": 20.0,  # Adjustable gravity for testing horn lifting
    "SEPARATION_FORCE": 1.15  # Instant position separation on collision (set to 0 for smooth)
}

last_time = time.time()
accumulator = 0.0  # Time accumulator for fixed timestep physics

while window.running:
    current_time = time.time()
    frame_dt = current_time - last_time
    last_time = current_time

    # Calculate actual FPS before capping (for accurate display)
    actual_fps = 1.0 / frame_dt if frame_dt > 0 else 0

    frame_dt = min(frame_dt, 0.02)  # Cap at 20ms to prevent accumulator spikes

    # Add frame time to accumulator
    accumulator += frame_dt

    # Cap accumulator to prevent spiral during geometry rebuilds (max 3 physics steps per frame)
    MAX_ACCUMULATOR = PHYSICS_TIMESTEP * 3  # ~50ms worth of simulation
    accumulator = min(accumulator, MAX_ACCUMULATOR)

    # Camera (runs every frame at frame rate)
    renderer.handle_camera_controls(camera, window, frame_dt)
    renderer.handle_mouse_look(camera, window)

    # ===== FIXED TIMESTEP PHYSICS LOOP =====
    # Run physics in fixed increments (may run 0+ times per frame)
    while accumulator >= PHYSICS_TIMESTEP:
        # Save previous state for interpolation
        beetle_blue.save_previous_state()
        beetle_red.save_previous_state()

        # === BLUE BEETLE CONTROLS (TFGH) - TANK STYLE ===
        if beetle_blue.active and not beetle_blue.is_falling:
            # Rotation controls (F/H)
            if window.is_pressed('f'):
                # Rotate left (counterclockwise)
                beetle_blue.rotation -= ROTATION_SPEED * PHYSICS_TIMESTEP
            if window.is_pressed('h'):
                # Rotate right (clockwise)
                beetle_blue.rotation += ROTATION_SPEED * PHYSICS_TIMESTEP

            # Movement controls (T/G) - move in facing direction
            if window.is_pressed('t'):
                # Move forward in facing direction
                move_x = math.cos(beetle_blue.rotation)
                move_z = math.sin(beetle_blue.rotation)
                beetle_blue.apply_force(move_x * MOVE_FORCE, move_z * MOVE_FORCE, PHYSICS_TIMESTEP)
            if window.is_pressed('g'):
                # Move backward in facing direction
                move_x = -math.cos(beetle_blue.rotation)
                move_z = -math.sin(beetle_blue.rotation)
                beetle_blue.apply_force(move_x * BACKWARD_MOVE_FORCE, move_z * BACKWARD_MOVE_FORCE, PHYSICS_TIMESTEP)

            # Horn controls (R/Y) - tilt up/down
            if window.is_pressed('r'):
                # Tilt horn up
                beetle_blue.horn_pitch += HORN_TILT_SPEED * PHYSICS_TIMESTEP
                beetle_blue.horn_pitch = min(HORN_MAX_PITCH, beetle_blue.horn_pitch)
                beetle_blue.horn_pitch_velocity = HORN_TILT_SPEED  # Positive = tilting up
            elif window.is_pressed('y'):
                # Tilt horn down
                beetle_blue.horn_pitch -= HORN_TILT_SPEED * PHYSICS_TIMESTEP
                beetle_blue.horn_pitch = max(HORN_MIN_PITCH, beetle_blue.horn_pitch)
                beetle_blue.horn_pitch_velocity = -HORN_TILT_SPEED  # Negative = tilting down
            else:
                # Not pressing horn keys - no velocity
                beetle_blue.horn_pitch_velocity = 0.0

            # Always add physics-driven rotation from collisions
            beetle_blue.rotation += beetle_blue.angular_velocity * PHYSICS_TIMESTEP
            beetle_blue.rotation = normalize_angle(beetle_blue.rotation)

        # === RED BEETLE CONTROLS (IJKL) - TANK STYLE ===
        if beetle_red.active and not beetle_red.is_falling:
            # Rotation controls (J/L)
            if window.is_pressed('j'):
                # Rotate left (counterclockwise)
                beetle_red.rotation -= ROTATION_SPEED * PHYSICS_TIMESTEP
            if window.is_pressed('l'):
                # Rotate right (clockwise)
                beetle_red.rotation += ROTATION_SPEED * PHYSICS_TIMESTEP

            # Movement controls (I/K) - move in facing direction
            if window.is_pressed('i'):
                # Move forward in facing direction
                move_x = math.cos(beetle_red.rotation)
                move_z = math.sin(beetle_red.rotation)
                beetle_red.apply_force(move_x * MOVE_FORCE, move_z * MOVE_FORCE, PHYSICS_TIMESTEP)
            if window.is_pressed('k'):
                # Move backward in facing direction
                move_x = -math.cos(beetle_red.rotation)
                move_z = -math.sin(beetle_red.rotation)
                beetle_red.apply_force(move_x * BACKWARD_MOVE_FORCE, move_z * BACKWARD_MOVE_FORCE, PHYSICS_TIMESTEP)

            # Horn controls (U/O) - tilt up/down
            if window.is_pressed('u'):
                # Tilt horn up
                beetle_red.horn_pitch += HORN_TILT_SPEED * PHYSICS_TIMESTEP
                beetle_red.horn_pitch = min(HORN_MAX_PITCH, beetle_red.horn_pitch)
                beetle_red.horn_pitch_velocity = HORN_TILT_SPEED  # Positive = tilting up
            elif window.is_pressed('o'):
                # Tilt horn down
                beetle_red.horn_pitch -= HORN_TILT_SPEED * PHYSICS_TIMESTEP
                beetle_red.horn_pitch = max(HORN_MIN_PITCH, beetle_red.horn_pitch)
                beetle_red.horn_pitch_velocity = -HORN_TILT_SPEED  # Negative = tilting down
            else:
                # Not pressing horn keys - no velocity
                beetle_red.horn_pitch_velocity = 0.0

            # Always add physics-driven rotation from collisions
            beetle_red.rotation += beetle_red.angular_velocity * PHYSICS_TIMESTEP
            beetle_red.rotation = normalize_angle(beetle_red.rotation)

        # Physics update
        beetle_blue.update_physics(PHYSICS_TIMESTEP)
        beetle_red.update_physics(PHYSICS_TIMESTEP)

        # Fall death detection - two-stage system
        POINT_OF_NO_RETURN = -5.0  # Once below this, can't recover (5 voxels below floor)
        FALL_DEATH_Y = -30.0  # Fully removed at this point (30 voxels below floor)

        # Stage 1: Point of no return - disable controls but keep rendering
        if beetle_blue.active and not beetle_blue.is_falling and beetle_blue.y < POINT_OF_NO_RETURN:
            beetle_blue.is_falling = True
            print("BLUE BEETLE IS FALLING!")
        if beetle_red.active and not beetle_red.is_falling and beetle_red.y < POINT_OF_NO_RETURN:
            beetle_red.is_falling = True
            print("RED BEETLE IS FALLING!")

        # Stage 2: Full removal - deactivate completely
        if beetle_blue.active and beetle_blue.y < FALL_DEATH_Y:
            beetle_blue.active = False
            print("BLUE BEETLE FELL INTO THE ABYSS!")
            if match_winner is None:
                match_winner = "RED"
                print("\n" + "="*50)
                print("RED BEETLE WINS!")
                print("="*50 + "\n")
        if beetle_red.active and beetle_red.y < FALL_DEATH_Y:
            beetle_red.active = False
            print("RED BEETLE FELL INTO THE ABYSS!")
            if match_winner is None:
                match_winner = "BLUE"
                print("\n" + "="*50)
                print("BLUE BEETLE WINS!")
                print("="*50 + "\n")

        # Floor collision - prevent penetration by pushing beetles upward
        # Don't check floor collision if beetle is falling
        if beetle_blue.active and not beetle_blue.is_falling:
            floor_y_blue = check_floor_collision(beetle_blue.x, beetle_blue.z)
            if floor_y_blue > -100.0:  # Floor detected under beetle (world space, floor is at Y=0)
                # Calculate lowest point of beetle geometry after rotation
                lowest_point_blue = calculate_beetle_lowest_point(
                    beetle_blue.y, beetle_blue.rotation, beetle_blue.pitch,
                    beetle_blue.roll, beetle_blue.horn_pitch
                )

                # Check if beetle penetrates floor (lowest point goes into or below floor)
                floor_surface = floor_y_blue + 1.0  # Top of floor is at floor_y + 1
                if lowest_point_blue < floor_surface:
                    # Penetration detected! Push beetle upward
                    penetration_depth = floor_surface - lowest_point_blue
                    beetle_blue.y += penetration_depth

                    # Apply bounce/resistance
                    if beetle_blue.vy < 0:  # Moving downward
                        beetle_blue.vy = 0.0  # Stop downward motion

                    beetle_blue.on_ground = True
                elif lowest_point_blue < floor_surface + 0.5:  # Close to ground
                    beetle_blue.on_ground = True

        if beetle_red.active and not beetle_red.is_falling:
            floor_y_red = check_floor_collision(beetle_red.x, beetle_red.z)
            if floor_y_red >= 0:  # Floor detected under beetle
                lowest_point_red = calculate_beetle_lowest_point(
                    beetle_red.y, beetle_red.rotation, beetle_red.pitch,
                    beetle_red.roll, beetle_red.horn_pitch
                )

                floor_surface = floor_y_red + 1.0
                if lowest_point_red < floor_surface:
                    penetration_depth = floor_surface - lowest_point_red
                    beetle_red.y += penetration_depth

                    if beetle_red.vy < 0:
                        beetle_red.vy = 0.0

                    beetle_red.on_ground = True
                elif lowest_point_red < floor_surface + 0.5:
                    beetle_red.on_ground = True

        # Beetle collision (voxel-perfect) - only if both beetles are active
        if beetle_blue.active and beetle_red.active:
            beetle_collision(beetle_blue, beetle_red, physics_params)

        # Subtract fixed timestep from accumulator
        accumulator -= PHYSICS_TIMESTEP

    # ===== END FIXED TIMESTEP PHYSICS LOOP =====

    # Calculate interpolation alpha for smooth rendering between physics states
    alpha = accumulator / PHYSICS_TIMESTEP

    # Helper function for angle interpolation (shortest path)
    def lerp_angle(a, b, t):
        """Interpolate between angles using shortest path (radians)"""
        diff = (b - a) % (2 * math.pi)
        if diff > math.pi:
            diff -= 2 * math.pi
        return a + diff * t

    # Interpolate blue beetle state for rendering
    blue_render_x = beetle_blue.prev_x + (beetle_blue.x - beetle_blue.prev_x) * alpha
    blue_render_y = beetle_blue.prev_y + (beetle_blue.y - beetle_blue.prev_y) * alpha
    blue_render_z = beetle_blue.prev_z + (beetle_blue.z - beetle_blue.prev_z) * alpha
    blue_render_rotation = lerp_angle(beetle_blue.prev_rotation, beetle_blue.rotation, alpha)
    blue_render_pitch = lerp_angle(beetle_blue.prev_pitch, beetle_blue.pitch, alpha)
    blue_render_roll = lerp_angle(beetle_blue.prev_roll, beetle_blue.roll, alpha)
    blue_render_horn_pitch = lerp_angle(beetle_blue.prev_horn_pitch, beetle_blue.horn_pitch, alpha)

    # Interpolate red beetle state for rendering
    red_render_x = beetle_red.prev_x + (beetle_red.x - beetle_red.prev_x) * alpha
    red_render_y = beetle_red.prev_y + (beetle_red.y - beetle_red.prev_y) * alpha
    red_render_z = beetle_red.prev_z + (beetle_red.z - beetle_red.prev_z) * alpha
    red_render_rotation = lerp_angle(beetle_red.prev_rotation, beetle_red.rotation, alpha)
    red_render_pitch = lerp_angle(beetle_red.prev_pitch, beetle_red.pitch, alpha)
    red_render_roll = lerp_angle(beetle_red.prev_roll, beetle_red.roll, alpha)
    red_render_horn_pitch = lerp_angle(beetle_red.prev_horn_pitch, beetle_red.horn_pitch, alpha)

    # Update walk animation based on velocity (uses real-time frame_dt)
    blue_speed = math.sqrt(beetle_blue.vx**2 + beetle_blue.vz**2)
    if blue_speed > 0.5:  # Only animate if moving
        beetle_blue.walk_phase += blue_speed * WALK_CYCLE_SPEED * frame_dt
        beetle_blue.walk_phase = beetle_blue.walk_phase % (2 * math.pi)  # Keep in [0, 2π]
        beetle_blue.is_moving = True
    else:
        beetle_blue.is_moving = False

    red_speed = math.sqrt(beetle_red.vx**2 + beetle_red.vz**2)
    if red_speed > 0.5:
        beetle_red.walk_phase += red_speed * WALK_CYCLE_SPEED * frame_dt
        beetle_red.walk_phase = beetle_red.walk_phase % (2 * math.pi)
        beetle_red.is_moving = True
    else:
        beetle_red.is_moving = False

    # Detect if beetles are lifted high (for leg spaz animation)
    LIFT_THRESHOLD = 5.0  # 4 voxels above normal ground
    beetle_blue.is_lifted_high = (beetle_blue.y > LIFT_THRESHOLD)
    beetle_red.is_lifted_high = (beetle_red.y > LIFT_THRESHOLD)

    # Render - ANIMATED with leg walking cycles and interpolated smooth positions!
    # OPTIMIZATION: Use dirty voxel tracking instead of scanning 250K voxels
    clear_dirty_voxels()  # Clear voxels from previous frame (only ~600 voxels)
    reset_dirty_voxels()  # Reset counter for this frame's tracking
    if beetle_blue.active:
        place_animated_beetle(blue_render_x, blue_render_y, blue_render_z, blue_render_rotation, blue_render_pitch, blue_render_roll, blue_render_horn_pitch, simulation.BEETLE_BLUE, simulation.BEETLE_BLUE_LEGS, simulation.LEG_TIP_BLUE, beetle_blue.walk_phase, 1 if beetle_blue.is_lifted_high else 0)
    if beetle_red.active:
        place_animated_beetle(red_render_x, red_render_y, red_render_z, red_render_rotation, red_render_pitch, red_render_roll, red_render_horn_pitch, simulation.BEETLE_RED, simulation.BEETLE_RED_LEGS, simulation.LEG_TIP_RED, beetle_red.walk_phase, 1 if beetle_red.is_lifted_high else 0)

    canvas.set_background_color((0.13, 0.35, 0.13))  # Forest green
    renderer.render(camera, canvas, scene, simulation.voxel_type, simulation.n_grid)
    canvas.scene(scene)

    # HUD
    window.GUI.begin("Beetle Physics", 0.01, 0.01, 0.35, 0.52)
    window.GUI.text(f"FPS: {actual_fps:.0f}")
    window.GUI.text("")

    # Blue beetle status
    if beetle_blue.active:
        window.GUI.text("BLUE BEETLE (TFGH + RY)")
        window.GUI.text(f"  Pos: ({beetle_blue.x:.1f}, {beetle_blue.y:.1f}, {beetle_blue.z:.1f})")
        window.GUI.text(f"  Speed: {math.sqrt(beetle_blue.vx**2 + beetle_blue.vz**2):.1f}")
        window.GUI.text(f"  Facing: {math.degrees(beetle_blue.rotation):.0f}°")
        window.GUI.text(f"  Horn: {math.degrees(beetle_blue.horn_pitch):.1f}°")
    else:
        window.GUI.text("BLUE BEETLE: FALLEN")

    window.GUI.text("")

    # Red beetle status
    if beetle_red.active:
        window.GUI.text("RED BEETLE (IJKL + UO)")
        window.GUI.text(f"  Pos: ({beetle_red.x:.1f}, {beetle_red.y:.1f}, {beetle_red.z:.1f})")
        window.GUI.text(f"  Speed: {math.sqrt(beetle_red.vx**2 + beetle_red.vz**2):.1f}")
        window.GUI.text(f"  Facing: {math.degrees(beetle_red.rotation):.0f}°")
        window.GUI.text(f"  Horn: {math.degrees(beetle_red.horn_pitch):.1f}°")
    else:
        window.GUI.text("RED BEETLE: FALLEN")
    window.GUI.text("")

    # Distance display - only if both beetles are active
    if beetle_blue.active and beetle_red.active:
        dx = beetle_blue.x - beetle_red.x
        dz = beetle_blue.z - beetle_red.z
        distance = math.sqrt(dx**2 + dz**2)
        window.GUI.text(f"Distance: {distance:.1f}")

    # Active beetles count
    active_count = (1 if beetle_blue.active else 0) + (1 if beetle_red.active else 0)
    window.GUI.text(f"Active beetles: {active_count}/2")

    window.GUI.text("")
    window.GUI.text("=== GENETICS TEST ===")

    # Initialize persistent slider values
    if not hasattr(window, 'horn_shaft_value'):
        window.horn_shaft_value = 12
        window.horn_prong_value = 5
        window.back_body_height_value = 5
        window.body_length_value = 12
        window.body_width_value = 7
        window.leg_length_value = 10

    # Front body (thorax) is fixed at 4 layers
    front_body_height = 4

    # Use slider_int for discrete voxel values (max 18 voxel reach: shaft 12 + prong 6)
    new_shaft = window.GUI.slider_int("Horn Shaft", window.horn_shaft_value, 4, 12)
    new_prong = window.GUI.slider_int("Horn Prong", window.horn_prong_value, 3, 6)
    new_back_body = window.GUI.slider_int("Back Body", window.back_body_height_value, 4, 8)
    new_body_length = window.GUI.slider_int("Body Length", window.body_length_value, 8, 16)
    new_body_width = window.GUI.slider_int("Body Width", window.body_width_value, 5, 9)
    new_leg_length = window.GUI.slider_int("Leg Length", window.leg_length_value, 6, 10)

    # Rebuild geometry if sliders changed
    if new_shaft != window.horn_shaft_value or new_prong != window.horn_prong_value or new_back_body != window.back_body_height_value or new_body_length != window.body_length_value or new_body_width != window.body_width_value or new_leg_length != window.leg_length_value:
        rebuild_beetle_geometry(new_shaft, new_prong, front_body_height, new_back_body, new_body_length, new_body_width, new_leg_length)
        ti.sync()  # Ensure CPU writes complete before GPU kernels read
        window.horn_shaft_value = new_shaft
        window.horn_prong_value = new_prong
        window.back_body_height_value = new_back_body
        window.body_length_value = new_body_length
        window.body_width_value = new_body_width
        window.leg_length_value = new_leg_length

    window.GUI.text(f"Shaft: {window.horn_shaft_value} voxels")
    window.GUI.text(f"Prong: {window.horn_prong_value} voxels")
    window.GUI.text(f"Back Body: {window.back_body_height_value} layers")
    window.GUI.text(f"Body Length: {window.body_length_value} voxels")
    window.GUI.text(f"Body Width: {window.body_width_value} voxels")
    window.GUI.text(f"Leg Length: {window.leg_length_value} voxels")
    total_reach = window.horn_shaft_value + window.horn_prong_value
    total_body = 3 + window.body_length_value  # Thorax (3) + abdomen (variable)
    window.GUI.text(f"Total Horn: {total_reach} voxels")
    window.GUI.text(f"Total Body: {total_body} voxels")

    # Winner announcement and restart button
    if match_winner is not None:
        window.GUI.text("")
        window.GUI.text("="*30)
        if match_winner == "BLUE":
            window.GUI.text("*** BLUE BEETLE WINS! ***")
        else:
            window.GUI.text("*** RED BEETLE WINS! ***")
        window.GUI.text("="*30)
        window.GUI.text("")
        if window.GUI.button("RESTART MATCH"):
            reset_match()

    # Physics parameter sliders
    window.GUI.text("")
    window.GUI.text("=== PHYSICS TUNING ===")
    physics_params["GRAVITY"] = window.GUI.slider_float("Gravity", physics_params["GRAVITY"], 0.5, 40.0)
    physics_params["TORQUE_MULTIPLIER"] = window.GUI.slider_float("Torque", physics_params["TORQUE_MULTIPLIER"], 0.0, 1.5)
    physics_params["IMPULSE_MULTIPLIER"] = window.GUI.slider_float("Impulse", physics_params["IMPULSE_MULTIPLIER"], 0.0, 1.0)
    physics_params["SEPARATION_FORCE"] = window.GUI.slider_float("Separation", physics_params["SEPARATION_FORCE"], 0.0, 5.0)
    physics_params["RESTITUTION"] = window.GUI.slider_float("Bounce", physics_params["RESTITUTION"], 0.0, 0.5)
    new_inertia_factor = window.GUI.slider_float("Inertia", physics_params["MOMENT_OF_INERTIA_FACTOR"], 0.5, 5.0)

    # Update beetle inertia if factor changed
    if abs(new_inertia_factor - physics_params["MOMENT_OF_INERTIA_FACTOR"]) > 0.001:
        physics_params["MOMENT_OF_INERTIA_FACTOR"] = new_inertia_factor
        beetle_blue.moment_of_inertia = BEETLE_RADIUS * physics_params["MOMENT_OF_INERTIA_FACTOR"]
        beetle_red.moment_of_inertia = BEETLE_RADIUS * physics_params["MOMENT_OF_INERTIA_FACTOR"]

    window.GUI.end()

    window.show()

print("Done!")
