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
ROTATION_SPEED = 2.5  # Rotation speed for controlled turning
ANGULAR_FRICTION = 0.85  # How quickly spin slows down (lower = more spin)
MAX_ANGULAR_SPEED = 8.0  # Max spin speed (radians/sec) from collision impacts
TORQUE_MULTIPLIER = 1.95  # Rotational forces on collision
RESTITUTION = 0.0  # Bounce coefficient (0 = no bounce, 1 = full bounce)
IMPULSE_MULTIPLIER = 0.7  # Linear momentum transfer
MOMENT_OF_INERTIA_FACTOR = 1.15  # Resistance to rotation
ARENA_RADIUS = 32.0  # 25% smaller for closer combat
MAX_VERTICAL_VELOCITY = 20.0  # Maximum upward velocity to prevent outlier launches
AIR_RESISTANCE = 0.02  # Air drag coefficient for vertical movement

# Horn control constants
HORN_TILT_SPEED = 2.0  # Radians per second
HORN_DEFAULT_PITCH = math.radians(10)  # Start at +10 degrees (raised) - for rhino
HORN_DEFAULT_PITCH_STAG = math.radians(18)  # Start at +18 degrees (raised) - for stag pincers
HORN_DEFAULT_PITCH_HERCULES = math.radians(15)  # Start at +15 degrees (jaws slightly open) - for hercules
HORN_DEFAULT_PITCH_SCORPION = math.radians(20)  # Start at +20 degrees (raised) - for scorpion claws
HORN_MAX_PITCH = math.radians(30)  # +30 degrees vertical (up) - 20° range from default
HORN_MIN_PITCH = math.radians(-10)  # -10 degrees vertical (down) - 20° range from default
# Hercules-specific limits (±15° range from default)
HORN_MAX_PITCH_HERCULES = math.radians(30)  # +30 degrees (jaws fully open)
HORN_MIN_PITCH_HERCULES = math.radians(0)   # 0 degrees (jaws fully closed)

# Horn yaw control (Phase 2 - pincer spread for stag, yaw for rhino)
HORN_YAW_SPEED = 2.0  # Radians per second
HORN_MAX_YAW = math.radians(15)  # +15 degrees horizontal
HORN_MIN_YAW = math.radians(-15)  # -15 degrees horizontal
HORN_YAW_MIN_DISTANCE = 1.0  # Minimum distance between horn tips (voxels) to allow yaw rotation
HORN_PITCH_MIN_DISTANCE = 1.0  # Minimum distance between horn tips (voxels) to allow pitch rotation

# Edge tipping constants
EDGE_TIPPING_STRENGTH = 0.45  # Force multiplier per over-edge voxel
ARENA_CENTER_X = 64.0  # Arena center X coordinate
ARENA_CENTER_Z = 64.0  # Arena center Z coordinate
ARENA_EDGE_RADIUS = 30.0  # Arena edge radius (voxels beyond this trigger tipping)

# Fixed timestep physics constants
PHYSICS_TIMESTEP = 1.0 / 60.0  # 60 Hz physics update rate (16.67ms per step)
MAX_TIMESTEP_ACCUMULATOR = 0.25  # Max accumulator to prevent spiral of death

# Horn rotation damping constants (Phase 2: Horn Clipping Prevention)
HORN_DAMPING_INTO = 0.85      # Damping when rotating into collision (0.85 = 15% speed)
HORN_DAMPING_AWAY = 0.15      # Damping when rotating away from collision (0.15 = 85% speed)
HORN_DAMPING_NEUTRAL = 0.30   # Damping when not actively rotating
HORN_DAMPING_SMOOTHING = 0.3  # Smoothing rate for damping changes (0-1, higher = faster)
HORN_DAMPING_DECAY_RATE = 5.0 # Damping decay rate when no contact (per second)
HORN_ENGAGEMENT_CONTACT_THRESHOLD = 20.0  # Voxel count for full engagement
HORN_ENGAGEMENT_HEIGHT_WEIGHT = 0.4  # Weight of height vs contact count (0-1)

# Body rotation damping constants (Phase 3: Directional Rotation Prevention)
BODY_ROTATION_DAMPING_STRENGTH = 0.95  # 95% damping (5% speed) when rotating into spin direction
BODY_ROTATION_DAMPING_DECAY = 2.0      # Decay rate per second (~0.5s duration at full strength)

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
        self.horn_yaw = 0.0    # Horizontal angle (stag=pincer spread, rhino=horn yaw) in radians
        self.horn_pitch_velocity = 0.0  # Rate of change of horn pitch (radians/sec)
        self.horn_yaw_velocity = 0.0  # Rate of change of horn yaw (radians/sec)
        self.horn_rotation_damping = 0.0  # Horn rotation resistance during collision (0.0-1.0)

        # Horn geometry (set during geometry generation)
        self.horn_shaft_len = 12  # Default horn shaft length in voxels
        self.horn_prong_len = 5   # Default horn prong length in voxels
        self.horn_length = 17.0   # Cached total horn reach (updated when geometry changes)
        self.horn_type = "rhino"  # Horn type: "rhino" (single horn), "stag" (dual pincers), "hercules" (dual jaws), or "scorpion"
        self.body_pitch_offset = 0.0  # Static body tilt angle (radians) - calculated from leg geometry for scorpion

        # Scorpion stinger control (VB/NM keys)
        self.stinger_curvature = 0.0  # Stinger curvature offset (-1.0 to +1.0): negative=dart forward, positive=pull back, 0=neutral
        self.tail_rotation_angle = 0.0  # Tail rotation angle in degrees (VB=down, NM=up)

        # Center of gravity tracking (for edge tipping physics)
        self.cog_x = x  # Center of gravity X
        self.cog_y = 1.0 + 3.0  # Center of gravity Y (approximate beetle center)
        self.cog_z = z  # Center of gravity Z

        # Collision cooldown timers
        self.lift_cooldown = 0.0  # Time remaining before next lift can be applied (seconds)

        # Body rotation damping state (Phase 3: Directional Rotation Prevention)
        self.body_rotation_damping = 0.0  # Rotation resistance after collision (0.0-1.0)
        self.collision_spin_direction = 0  # Which way collision made us spin (-1=CCW, 0=none, 1=CW)
        self.in_horn_collision = False  # True when actively in horn-to-horn collision

        # Beetle active state (for fall death)
        self.active = True  # False when beetle has fallen off arena
        self.is_falling = False  # True when beetle has passed point of no return
        self.has_exploded = False  # True when death particle explosion has been triggered
        self.is_lifted_high = False  # True when lifted significantly above ground (for leg spaz animation)

        # Previous state for fixed timestep interpolation
        self.prev_x = x
        self.prev_y = 1.0
        self.prev_z = z
        self.prev_rotation = rotation
        self.prev_pitch = 0.0
        self.prev_roll = 0.0
        self.prev_horn_pitch = HORN_DEFAULT_PITCH
        self.prev_horn_yaw = 0.0

    def save_previous_state(self):
        """Save current state as previous for interpolation"""
        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_z = self.z
        self.prev_rotation = self.rotation
        self.prev_pitch = self.pitch
        self.prev_roll = self.roll
        self.prev_horn_pitch = self.horn_pitch
        self.prev_horn_yaw = self.horn_yaw

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

        # Decay horn rotation damping when not in contact (Phase 2: Horn Clipping Prevention)
        if self.horn_rotation_damping > 0.0:
            self.horn_rotation_damping = max(0.0, self.horn_rotation_damping - HORN_DAMPING_DECAY_RATE * dt)

        # Clear horn collision flag (will be reset each frame if still colliding)
        self.in_horn_collision = False

        # Decay body rotation damping after collision (Phase 3: Directional Rotation Prevention)
        if self.body_rotation_damping > 0.0:
            self.body_rotation_damping = max(0.0, self.body_rotation_damping - BODY_ROTATION_DAMPING_DECAY * dt)
            # Clear spin direction when damping reaches zero
            if self.body_rotation_damping == 0.0:
                self.collision_spin_direction = 0

        # === GRAVITY SYSTEM (Phase 1) ===
        # Air resistance (quadratic drag - lightweight calculation)
        if abs(self.vy) > 1.0:
            air_drag = self.vy * abs(self.vy) * AIR_RESISTANCE
            self.vy -= air_drag * dt

        # Apply gravity (always on) - use global adjustable gravity
        self.vy -= physics_params["GRAVITY"] * dt

        # Apply vertical velocity
        self.y += self.vy * dt

        # Cap vertical velocity to prevent outlier launches (hard safety limit)
        if self.vy > MAX_VERTICAL_VELOCITY:
            self.vy = MAX_VERTICAL_VELOCITY

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

# Create beetles - closer together for smaller arena (horn dimensions set later)
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
def check_collision_kernel(x1: ti.f32, z1: ti.f32, y1: ti.f32, x2: ti.f32, z2: ti.f32, y2: ti.f32) -> ti.i32:
    """Fast GPU-based collision check - returns 1 if collision, 0 otherwise"""
    collision = 0

    # Convert to grid coordinates
    center1_x = int(x1 + simulation.n_grid / 2.0)
    center1_z = int(z1 + simulation.n_grid / 2.0)
    center2_x = int(x2 + simulation.n_grid / 2.0)
    center2_z = int(z2 + simulation.n_grid / 2.0)

    # Early rejection: If beetles too far apart, skip voxel scanning
    # Optimized: Beetle max radius ~18 voxels (body + horn), so 2x radius + margin = 38
    dx = center1_x - center2_x
    dz = center1_z - center2_z
    dist_sq = dx * dx + dz * dz
    too_far = 0
    if dist_sq > 5776:  # (76 voxels)^2 = 2 * 38 voxel max reach for collision detection
        too_far = 1

    # Tighter collision bounds - only check where beetles can actually overlap
    # Beetles are max 38 voxels from center (max body back: 16 + max horn forward: 21 = 37 + 1 safety margin)
    x_min = ti.max(ti.max(0, center1_x - 38), ti.max(0, center2_x - 38))
    x_max = ti.min(ti.min(simulation.n_grid, center1_x + 38),
                   ti.min(simulation.n_grid, center2_x + 38))
    z_min = ti.max(ti.max(0, center1_z - 38), ti.max(0, center2_z - 38))
    z_max = ti.min(ti.min(simulation.n_grid, center1_z + 38),
                   ti.min(simulation.n_grid, center2_z + 38))

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
                # Calculate independent ranges for each beetle (asymmetric: legs down, body/horn up)
                beetle_max_reach_down = 12  # Legs (max 10) + safety margin
                beetle_max_reach_up = 28    # Body (8) + horn vertical reach (20) + safety margin

                # Find grid Y for each beetle
                beetle1_grid_y = int(RENDER_Y_OFFSET + y1)
                beetle2_grid_y = int(RENDER_Y_OFFSET + y2)

                # Take union of both beetles' ranges
                y_start = ti.max(0, ti.min(beetle1_grid_y - beetle_max_reach_down,
                                            beetle2_grid_y - beetle_max_reach_down))
                y_end = ti.min(simulation.n_grid, ti.max(beetle1_grid_y + beetle_max_reach_up,
                                                          beetle2_grid_y + beetle_max_reach_up))
                for gy in range(y_start, y_end):
                    voxel = simulation.voxel_type[gx, gy, gz]
                    # Optimized: Range-based checks (all blue beetle types are 5-13, red are 6-14)
                    # Blue beetle voxels (body=5, legs=7, tips=9, stripe=11, horn_tip=13)
                    if voxel >= 5 and voxel <= 13 and (voxel == 5 or voxel == 7 or voxel == 9 or voxel == 11 or voxel == 13):
                        if gy < beetle1_y_min:
                            beetle1_y_min = gy
                        if gy > beetle1_y_max:
                            beetle1_y_max = gy
                    # Red beetle voxels (body=6, legs=8, tips=10, stripe=12, horn_tip=14)
                    elif voxel >= 6 and voxel <= 14 and (voxel == 6 or voxel == 8 or voxel == 10 or voxel == 12 or voxel == 14):
                        if gy < beetle2_y_min:
                            beetle2_y_min = gy
                        if gy > beetle2_y_max:
                            beetle2_y_max = gy

                # Check if Y-ranges overlap or are adjacent (within 1 voxel for tighter collision)
                if beetle1_y_max >= 0 and beetle2_y_max >= 0:  # Both beetles present
                    # Reduced tolerance from 2 to 1 voxel to reduce horn clipping
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
def generate_beetle_geometry(horn_shaft_len=12, horn_prong_len=5, front_body_height=4, back_body_height=5, body_length=12, body_width=7, leg_length=10, horn_type="rhino", stinger_curvature=0.0, tail_rotation_angle=0.0):
    """Generate beetle geometry separated into body and legs for animation

    Args:
        horn_shaft_len: Length of main horn shaft (default 12 voxels)
        horn_prong_len: Length of each Y-fork prong (default 5 voxels)
        horn_type: Type of horn to generate - "rhino" or "stag" (default "rhino")
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
        # Scale taper formula to body_length so abdomen grows properly
        taper_center = -body_length / 2.0  # Center of abdomen
        taper_scale = body_length / 2.5     # Scale factor for smooth taper
        length_pos = (dx - taper_center) / taper_scale
        length_taper = 1.0 - (length_pos * length_pos)
        if length_taper > 0:
            length_taper_sqrt = math.sqrt(length_taper)  # Calculate sqrt once per dx

            # SCORPION: Add Y offset that increases toward the rear (tail up)
            if horn_type == "scorpion":
                # Calculate how far back we are (0.0 at front, 1.0 at rear)
                rear_progress = (dx - (-1)) / (-body_length - (-1))  # 0.0 at dx=-1, 1.0 at dx=-body_length
                # Rear should be 4 voxels higher than front
                y_offset = int(rear_progress * 4)
            else:
                y_offset = 0

            for dy in range(0, back_body_height):
                width_at_height = width_at_height_cache[dy]
                if width_at_height > 0:
                    max_width = width_at_height * length_taper_sqrt
                    for dz in range(dz_min, dz_max):
                        if abs(dz) <= max_width:
                            body_voxels.append((dx, dy + y_offset, dz))

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

    # HORN GENERATION - Choose between rhino, stag, hercules, or scorpion (claws)
    if horn_type == "scorpion":
        # SCORPION - Dense, curved claws with thick arms (pedipalps)
        # Arms extend forward and upward from front of body
        arm_length = 6

        # LEFT ARM + CLAW
        # Dense arm that extends forward, upward, and outward
        for i in range(arm_length):
            dx = 2 + i
            dy_base = 2 + (i // 2)  # Rises from 2 to 4
            dz_base = -(i + 2)  # Extends leftward

            # 2x2 cross-section for thick arms (less bulky than 3x3)
            for dy_off in range(0, 2):
                for dz_off in range(0, 2):
                    body_voxels.append((dx, dy_base + dy_off, dz_base - dz_off))

        # Left claw - dense curved pincers at end of arm
        claw_base_x = 2 + arm_length
        claw_base_z = -(arm_length + 2)

        # Upper pincer - curves down and inward naturally
        upper_pincer = [
            # Base (wide)
            (0, 5, 0), (0, 5, -1), (0, 6, 0), (0, 6, -1),
            (0, 4, 0), (0, 4, -1),
            # Mid (curving)
            (1, 5, 0), (1, 5, 1), (1, 4, 0), (1, 4, 1),
            (2, 5, 1), (2, 4, 1), (2, 4, 2),
            # Curve
            (3, 4, 2), (3, 4, 3), (3, 3, 2), (3, 3, 3),
            # Tip (narrower, pointing inward)
            (4, 4, 3), (4, 3, 3), (4, 3, 4),
            (5, 3, 4), (5, 3, 5),
        ]
        for offset in upper_pincer:
            body_voxels.append((claw_base_x + offset[0], offset[1], claw_base_z + offset[2]))

        # Lower pincer - curves up and inward naturally
        lower_pincer = [
            # Base (wide) - raised to Y=2 minimum to avoid ground clipping
            (0, 2, 0), (0, 2, -1), (0, 2, 0), (0, 2, -1),
            (0, 3, 0), (0, 3, -1),
            # Mid (curving)
            (1, 2, 0), (1, 2, 1), (1, 3, 0), (1, 3, 1),
            (2, 2, 1), (2, 3, 1), (2, 3, 2),
            # Curve
            (3, 3, 2), (3, 3, 3), (3, 4, 2), (3, 4, 3),
            # Tip (narrower, pointing inward)
            (4, 3, 3), (4, 4, 3), (4, 4, 4),
            (5, 4, 4), (5, 4, 5),
        ]
        for offset in lower_pincer:
            body_voxels.append((claw_base_x + offset[0], offset[1], claw_base_z + offset[2]))

        # RIGHT ARM + CLAW (mirror of left)
        # Dense arm that extends forward, upward, and outward
        for i in range(arm_length):
            dx = 2 + i
            dy_base = 2 + (i // 2)
            dz_base = (i + 2)  # Extends rightward

            # 2x2 cross-section for thick arms (less bulky than 3x3)
            for dy_off in range(0, 2):
                for dz_off in range(0, 2):
                    body_voxels.append((dx, dy_base + dy_off, dz_base + dz_off))

        # Right claw - mirror the left claw
        claw_base_z_right = (arm_length + 2)

        # Upper pincer (mirrored)
        for offset in upper_pincer:
            body_voxels.append((claw_base_x + offset[0], offset[1], claw_base_z_right - offset[2]))

        # Lower pincer (mirrored)
        for offset in lower_pincer:
            body_voxels.append((claw_base_x + offset[0], offset[1], claw_base_z_right - offset[2]))

        # SCORPION STINGER/TAIL - Curved tail extending from rear
        # NOTE: tail_rotation_angle is now applied at runtime, not baked into geometry
        stinger_voxels = generate_scorpion_stinger(
            shaft_len=horn_shaft_len,
            prong_len=horn_prong_len,
            body_length=body_length,
            back_body_height=back_body_height,
            stinger_curvature=stinger_curvature,
            tail_rotation_angle=0.0  # Runtime rotation applied in rendering kernel
        )
        body_voxels.extend(stinger_voxels)
    elif horn_type == "stag":
        # STAG BEETLE PINCERS - Horizontal mandibles
        pincer_voxels = generate_stag_pincers(horn_shaft_len - 2, horn_prong_len)
        body_voxels.extend(pincer_voxels)
    elif horn_type == "hercules":
        # HERCULES BEETLE HORNS - Vertical dual-jaw pincer system
        hercules_voxels = generate_hercules_horns(horn_shaft_len, horn_prong_len, front_body_height, back_body_height)
        body_voxels.extend(hercules_voxels)
    else:
        # RHINOCEROS BEETLE HORN - Y-shaped vertical horn
        # Main shaft with overlapping layers
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

        # Y-fork - Left prong with overlapping layers
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

    # IMPROVED LEGS (6 for beetles, 8 for scorpion) - Separated for animation
    # Leg order: [front_left, front_right, middle_left, middle_right, rear_left, rear_right, rear2_left*, rear2_right*]
    # *Only for scorpion type

    # Helper function to calculate leg segment lengths with optional length multiplier
    def calc_leg_segments(base_leg_length, length_multiplier=1.0):
        """Calculate coxa, femur, tibia lengths for a leg with optional multiplier"""
        total_length = int(base_leg_length * length_multiplier)
        coxa = max(1, total_length // 5)  # ~20%
        femur = max(1, (total_length * 2) // 5)  # ~40%
        tibia = max(1, total_length - coxa - femur)  # Remainder ensures exact total
        return coxa, femur, tibia

    # For scorpion: graduated leg lengths (front shortest, rear longest)
    # For beetles: all legs same length (multiplier = 1.0)
    if horn_type == "scorpion":
        leg_multipliers = [1.0, 1.0, 1.1, 1.1, 1.2, 1.2, 1.3, 1.3]  # Graduated lengths
    else:
        leg_multipliers = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]  # All same length for beetles

    # Calculate segment lengths for front legs (legs 0, 1) - use multiplier[0]
    coxa_len, femur_len, tibia_len = calc_leg_segments(leg_length, leg_multipliers[0])
    coxa_start = 3
    femur_start = coxa_start + coxa_len
    tibia_start = femur_start + femur_len

    # Front left leg (leg 0) - MOVED BACK 1 VOXEL
    front_left = []
    front_left_tips = []  # Separate tips for black coloring
    side = -1
    # COXA
    for i in range(coxa_len):
        for extra_y in range(2):
            front_left.append((1, 1 + extra_y, side * (coxa_start + i)))
    # FEMUR
    for i in range(femur_len):
        for extra_y in range(2):
            front_left.append((1, 0 + extra_y, side * (femur_start + i)))
    # TIBIA (tips - will be rendered black)
    for i in range(tibia_len):
        tip_x = 1 + min(i // 2, 2)  # Extend forward gradually
        front_left_tips.append((tip_x, 0, side * (tibia_start + i)))
    leg_voxels.append(front_left)

    # Front right leg (leg 1) - MOVED BACK 1 VOXEL
    front_right = []
    front_right_tips = []
    side = 1
    for i in range(coxa_len):
        for extra_y in range(2):
            front_right.append((1, 1 + extra_y, side * (coxa_start + i)))
    for i in range(femur_len):
        for extra_y in range(2):
            front_right.append((1, 0 + extra_y, side * (femur_start + i)))
    for i in range(tibia_len):
        tip_x = 1 + min(i // 2, 2)
        front_right_tips.append((tip_x, 0, side * (tibia_start + i)))
    leg_voxels.append(front_right)

    # Calculate segment lengths for middle legs (legs 2, 3) - use multiplier[2]
    coxa_len, femur_len, tibia_len = calc_leg_segments(leg_length, leg_multipliers[2])
    coxa_start = 3
    femur_start = coxa_start + coxa_len
    tibia_start = femur_start + femur_len

    # Middle left leg (leg 2) - MOVED BACK 1 VOXEL
    middle_left = []
    middle_left_tips = []
    side = -1
    for i in range(coxa_len):
        for extra_y in range(2):
            middle_left.append((-3, 1 + extra_y, side * (coxa_start + i)))
    for i in range(femur_len):
        for extra_y in range(2):
            middle_left.append((-3, 0 + extra_y, side * (femur_start + i)))
    for i in range(tibia_len):
        tip_x = -3 - min(i // 2, 2)  # Extend backward gradually
        middle_left_tips.append((tip_x, 0, side * (tibia_start + i)))
    leg_voxels.append(middle_left)

    # Middle right leg (leg 3) - MOVED BACK 1 VOXEL
    middle_right = []
    middle_right_tips = []
    side = 1
    for i in range(coxa_len):
        for extra_y in range(2):
            middle_right.append((-3, 1 + extra_y, side * (coxa_start + i)))
    for i in range(femur_len):
        for extra_y in range(2):
            middle_right.append((-3, 0 + extra_y, side * (femur_start + i)))
    for i in range(tibia_len):
        tip_x = -3 - min(i // 2, 2)
        middle_right_tips.append((tip_x, 0, side * (tibia_start + i)))
    leg_voxels.append(middle_right)

    # Calculate segment lengths for rear legs (legs 4, 5) - use multiplier[4]
    coxa_len, femur_len, tibia_len = calc_leg_segments(leg_length, leg_multipliers[4])
    coxa_start = 3
    femur_start = coxa_start + coxa_len
    tibia_start = femur_start + femur_len

    # Rear left leg (leg 4) - Position proportional to body length - MOVED BACK 1 VOXEL
    rear_leg_attach_x = -(body_length // 2) - 1  # Proportional to abdomen length
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

    # SCORPION ONLY: Add two extra rear legs (legs 6, 7) - furthest back, longest, raised attachment
    if horn_type == "scorpion":
        # Calculate segment lengths for extra rear legs (legs 6, 7) - use multiplier[6]
        coxa_len, femur_len, tibia_len = calc_leg_segments(leg_length, leg_multipliers[6])
        coxa_start = 3
        femur_start = coxa_start + coxa_len
        tibia_start = femur_start + femur_len

        # Rear-2 left leg (leg 6) - Position furthest back, attach higher (Y=2)
        rear2_leg_attach_x = -(body_length * 3 // 4) - 2  # 75% back along abdomen, plus 2 voxels
        rear2_left = []
        rear2_left_tips = []
        side = -1
        # Add extra connector voxels to bridge from body to leg attachment
        for bridge_z in range(1, coxa_start + 1):  # Z from 1 to leg start (skip z=0 next to body)
            for extra_y in range(3):  # Y=2,3,4 - connects to tilted body
                rear2_left.append((rear2_leg_attach_x, 2 + extra_y, side * bridge_z))
        for i in range(coxa_len):
            for extra_y in range(2):
                rear2_left.append((rear2_leg_attach_x, 2 + extra_y, side * (coxa_start + i)))  # Y=2 (raised)
        for i in range(femur_len):
            for extra_y in range(2):
                rear2_left.append((rear2_leg_attach_x - i, 1 + extra_y, side * (femur_start + i)))  # Y=1
        for i in range(tibia_len):
            tip_x = rear2_leg_attach_x - femur_len - min(i // 2, 2)
            rear2_left_tips.append((tip_x, 0, side * (tibia_start + i)))
        leg_voxels.append(rear2_left)

        # Rear-2 right leg (leg 7) - Position furthest back, attach higher (Y=2)
        rear2_right = []
        rear2_right_tips = []
        side = 1
        # Add extra connector voxels to bridge from body to leg attachment
        for bridge_z in range(1, coxa_start + 1):  # Z from 1 to leg start (skip z=0 next to body)
            for extra_y in range(3):  # Y=2,3,4 - connects to tilted body
                rear2_right.append((rear2_leg_attach_x, 2 + extra_y, side * bridge_z))
        for i in range(coxa_len):
            for extra_y in range(2):
                rear2_right.append((rear2_leg_attach_x, 2 + extra_y, side * (coxa_start + i)))  # Y=2 (raised)
        for i in range(femur_len):
            for extra_y in range(2):
                rear2_right.append((rear2_leg_attach_x - i, 1 + extra_y, side * (femur_start + i)))  # Y=1
        for i in range(tibia_len):
            tip_x = rear2_leg_attach_x - femur_len - min(i // 2, 2)
            rear2_right_tips.append((tip_x, 0, side * (tibia_start + i)))
        leg_voxels.append(rear2_right)

        # Collect all leg tips into single list (8 legs for scorpion)
        leg_tips = [front_left_tips, front_right_tips, middle_left_tips,
                    middle_right_tips, rear_left_tips, rear_right_tips,
                    rear2_left_tips, rear2_right_tips]
    else:
        # Collect all leg tips into single list (6 legs for beetles)
        leg_tips = [front_left_tips, front_right_tips, middle_left_tips,
                    middle_right_tips, rear_left_tips, rear_right_tips]

    return body_voxels, leg_voxels, leg_tips

# Generate beetle geometry ONCE at startup (with default params)
BEETLE_BODY, BEETLE_LEGS, BEETLE_LEG_TIPS = generate_beetle_geometry()
print(f"Beetle geometry cached: {len(BEETLE_BODY)} body voxels + {sum(len(leg) for leg in BEETLE_LEGS)} leg voxels + {sum(len(tips) for tips in BEETLE_LEG_TIPS)} tip voxels")

# Create OVERSIZED Taichi fields for body geometry cache (to allow dynamic resizing)
# Hercules beetle with max settings can reach ~606 voxels
# Scorpion with Y-overlap for gap-free tail can reach ~2000 voxels at max length (3x multiplier in Y)
MAX_BODY_VOXELS = 2500
body_cache_size = ti.field(ti.i32, shape=())  # Track actual size
body_cache_size[None] = len(BEETLE_BODY)
body_cache_x = ti.field(ti.i32, shape=MAX_BODY_VOXELS)
body_cache_y = ti.field(ti.i32, shape=MAX_BODY_VOXELS)
body_cache_z = ti.field(ti.i32, shape=MAX_BODY_VOXELS)

# Create OVERSIZED Taichi fields for leg geometry cache (6 legs)
# Store all leg voxels flattened with offsets to know where each leg starts
# Max leg length=14: coxa(3) + femur(6) + tibia(6) = 15 voxels per segment
# Each segment has 2 height layers, so 15*2 = 30 voxels max per leg
# 8 legs × 30 voxels = 240 voxels max (scorpion has 8 legs)
MAX_LEG_VOXELS = 240
leg_cache_x = ti.field(ti.i32, shape=MAX_LEG_VOXELS)
leg_cache_y = ti.field(ti.i32, shape=MAX_LEG_VOXELS)
leg_cache_z = ti.field(ti.i32, shape=MAX_LEG_VOXELS)

# Leg start indices for each of the 8 legs (beetles have 6, scorpion has 8)
leg_start_idx = ti.field(ti.i32, shape=8)
leg_end_idx = ti.field(ti.i32, shape=8)

# Create OVERSIZED Taichi fields for leg tip geometry cache
# Max leg length=14: tibia tips per leg × 8 legs
# Increased to 65 to provide headroom for scorpion's 8 legs with thicker tip rendering
MAX_LEG_TIP_VOXELS = 65
leg_tip_cache_x = ti.field(ti.i32, shape=MAX_LEG_TIP_VOXELS)
leg_tip_cache_y = ti.field(ti.i32, shape=MAX_LEG_TIP_VOXELS)
leg_tip_cache_z = ti.field(ti.i32, shape=MAX_LEG_TIP_VOXELS)

# Leg tip start indices for each of the 8 legs (beetles have 6, scorpion has 8)
leg_tip_start_idx = ti.field(ti.i32, shape=8)
leg_tip_end_idx = ti.field(ti.i32, shape=8)

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
collision_has_horn_tips = ti.field(ti.i32, shape=())  # 1 if horn tip voxels involved, 0 otherwise

# Edge tipping calculation fields
edge_tipping_vy = ti.field(ti.f32, shape=())  # Downward velocity to apply
edge_tipping_pitch_vel = ti.field(ti.f32, shape=())  # Pitch angular velocity
edge_tipping_roll_vel = ti.field(ti.f32, shape=())  # Roll angular velocity

# Dirty voxel tracking for efficient clearing
# Instead of scanning 250K voxels, track only the ~600 voxels we actually place
# Max voxels per beetle: body(600) + legs(180) + tips(40) = 820
# Two beetles: 1640, use 2000 for safety
MAX_DIRTY_VOXELS = 5000  # Increased to handle 2 beetles at max size (back_body_height=8, with horns/tail/legs)
dirty_voxel_x = ti.field(ti.i32, shape=MAX_DIRTY_VOXELS)
dirty_voxel_y = ti.field(ti.i32, shape=MAX_DIRTY_VOXELS)
dirty_voxel_z = ti.field(ti.i32, shape=MAX_DIRTY_VOXELS)
dirty_voxel_count = ti.field(ti.i32, shape=())

# Animation parameters
WALK_CYCLE_SPEED = 1.0  # Radians per second at normal walk speed

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
def rebuild_beetle_geometry(shaft_len, prong_len, front_body_height=4, back_body_height=5, body_length=12, body_width=7, leg_length=10, horn_type="rhino", stinger_curvature=0.0, tail_rotation_angle=0.0):
    """Rebuild beetle geometry cache with new horn, body, and leg parameters"""
    global BEETLE_BODY, BEETLE_LEGS, BEETLE_LEG_TIPS

    # Generate new geometry
    BEETLE_BODY, BEETLE_LEGS, BEETLE_LEG_TIPS = generate_beetle_geometry(shaft_len, prong_len, front_body_height, back_body_height, body_length, body_width, leg_length, horn_type, stinger_curvature, tail_rotation_angle)

    # Update body cache
    if len(BEETLE_BODY) > MAX_BODY_VOXELS:
        print(f"WARNING: Body voxels ({len(BEETLE_BODY)}) exceeds max ({MAX_BODY_VOXELS})!")
        return

    # Clear old cache entries to prevent voxel pollution when switching beetle types
    old_size = body_cache_size[None]
    new_size = len(BEETLE_BODY)

    # Write new geometry
    for i, (dx, dy, dz) in enumerate(BEETLE_BODY):
        body_cache_x[i] = dx
        body_cache_y[i] = dy
        body_cache_z[i] = dz

    # Zero out any leftover voxels from previous geometry
    for i in range(new_size, old_size):
        body_cache_x[i] = 0
        body_cache_y[i] = 0
        body_cache_z[i] = 0

    # Update size LAST to prevent race conditions
    body_cache_size[None] = new_size

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

    # Clear extra legs (6 & 7) when switching from scorpion to 6-legged beetles
    num_legs = len(BEETLE_LEGS)
    for leg_id in range(num_legs, 8):
        leg_start_idx[leg_id] = 0
        leg_end_idx[leg_id] = 0

    # Zero out any leftover leg voxels from previous geometry (offset = total new leg voxels)
    for i in range(offset, MAX_LEG_VOXELS):
        leg_cache_x[i] = 0
        leg_cache_y[i] = 0
        leg_cache_z[i] = 0

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

    # Clear extra leg tips (6 & 7) when switching from scorpion to 6-legged beetles
    for leg_id in range(num_legs, 8):
        leg_tip_start_idx[leg_id] = 0
        leg_tip_end_idx[leg_id] = 0

    # Zero out any leftover leg tip voxels from previous geometry (offset = total new leg tip voxels)
    for i in range(offset, MAX_LEG_TIP_VOXELS):
        leg_tip_cache_x[i] = 0
        leg_tip_cache_y[i] = 0
        leg_tip_cache_z[i] = 0

    # Update beetle horn dimensions for accurate collision detection
    horn_length = calculate_horn_length(shaft_len, prong_len, horn_type)

    # Update both beetles (they share the same geometry)
    beetle_blue.horn_shaft_len = shaft_len
    beetle_blue.horn_prong_len = prong_len
    beetle_blue.horn_length = horn_length
    beetle_blue.horn_type = horn_type

    beetle_red.horn_shaft_len = shaft_len
    beetle_red.horn_prong_len = prong_len
    beetle_red.horn_length = horn_length
    beetle_red.horn_type = horn_type

    # Scorpion doesn't use body_pitch_offset - its tilt is built into the geometry
    beetle_blue.body_pitch_offset = 0.0
    beetle_red.body_pitch_offset = 0.0

    print(f"Rebuilt beetle: {len(BEETLE_BODY)} body voxels (shaft={shaft_len:.0f}, prong={prong_len:.0f}, front={front_body_height:.0f}, back={back_body_height:.0f}, legs={leg_length:.0f})")

def generate_stag_pincers(shaft_length, curve_length):
    """Generate stag beetle pincers with dynamic sizing (horizontal mandibles)

    Args:
        shaft_length: Length of straight horizontal segment close to body
        curve_length: Length of curved inward segment at tips

    Returns list of (x, y, z) voxel coordinates for both pincers.
    Pivot point: (x=3, y=1, z=0) same as rhinoceros horn
    Angled 10° upward from horizontal
    """
    pincer_voxels = []

    # Convert to int for voxel generation
    shaft_length = int(shaft_length)
    curve_length = int(curve_length)

    # LEFT PINCER (extends in -Z direction, curves inward toward +Z)
    # Straight shaft section extending left, angled 10° upward
    for i in range(shaft_length):
        dx = 3 + i  # Extends forward
        dy = 1 + int(i * 0.176)  # 10° upward angle (tan(10°) ≈ 0.176)
        dz = -i - 1 # Extends left (-Z direction), offset by -1 to avoid center overlap

        # Make it 2-3 voxels thick for solidity
        for dy_offset in range(2):  # Add vertical thickness
            for dz_offset in range(-1, 1):  # Add some width
                pincer_voxels.append((dx, dy + dy_offset, dz + dz_offset))

    # Curved tip section (curves inward)
    curve_start_x = 3 + shaft_length
    curve_start_z = -shaft_length - 1  # Match the offset
    base_height = 1 + int(shaft_length * 0.176)  # Continue from shaft's final height
    for i in range(curve_length):
        dx = curve_start_x + i
        dy = base_height + int(i * 0.176)  # Continue 10° upward angle
        dz = curve_start_z + int(i * 0.36)  # Curves back toward center at 110° angle (+Z direction)

        # Maintain thickness
        for dy_offset in range(2):
            for dz_offset in range(-1, 1):
                pincer_voxels.append((dx, dy + dy_offset, dz + dz_offset))

    # RIGHT PINCER (extends in +Z direction, curves inward toward -Z)
    # Straight shaft section extending right, angled 10° upward
    for i in range(shaft_length):
        dx = 3 + i  # Extends forward
        dy = 1 + int(i * 0.176)  # 10° upward angle (tan(10°) ≈ 0.176)
        dz = i + 1  # Extends right (+Z direction), offset by +1 to avoid center overlap

        # Make it 2-3 voxels thick for solidity
        for dy_offset in range(2):
            for dz_offset in range(-1, 1):
                pincer_voxels.append((dx, dy + dy_offset, dz + dz_offset))

    # Curved tip section (curves inward)
    curve_start_x = 3 + shaft_length
    curve_start_z = shaft_length + 1  # Match the offset
    base_height = 1 + int(shaft_length * 0.176)  # Continue from shaft's final height
    for i in range(curve_length):
        dx = curve_start_x + i
        dy = base_height + int(i * 0.176)  # Continue 10° upward angle
        dz = curve_start_z - int(i * 0.36)  # Curves back toward center at 110° angle (-Z direction)

        # Maintain thickness
        for dy_offset in range(2):
            for dz_offset in range(-1, 1):
                pincer_voxels.append((dx, dy + dy_offset, dz + dz_offset))

    return pincer_voxels

def generate_hercules_horns(top_horn_len, bottom_horn_len, front_body_height, back_body_height):
    """Generate Hercules beetle dual-jaw horns (vertical pincer system)

    Args:
        top_horn_len: Length of thoracic horn (upper jaw) - long curved hook
        bottom_horn_len: Length of cephalic horn (lower jaw) - short straight spike
        front_body_height: Height of thorax (needed for horn attachment positioning)
        back_body_height: Height of abdomen (needed for proper scaling)

    Returns list of (x, y, z) voxel coordinates for both horns.
    Pivot point: (x=3, y=1, z=0) same as other horn types
    """
    horn_voxels = []

    # Convert to int for voxel generation
    top_horn_len = int(top_horn_len)
    bottom_horn_len = int(bottom_horn_len)

    # TOP HORN (Thoracic - upper jaw from thorax)
    # Long curved arc that rises then hooks downward like excavator claw
    # Origin: Y=7 (top of body), extends forward with upward curve then downward hook
    # Slow reduction formula: max length at slider 12, but reduces slowly when decreasing slider
    # Angled 20° upward (tan(20°) ≈ 0.364)
    # Then rotated up 10° from base position
    MAX_TOP_LENGTH = 17  # Length at max slider (12)
    MAX_SLIDER = 12
    actual_top_len = int(MAX_TOP_LENGTH - (MAX_SLIDER - top_horn_len) * 0.5)

    import math
    top_rotation_angle = math.radians(10)  # Rotate up 10 degrees
    cos_top_rot = math.cos(top_rotation_angle)
    sin_top_rot = math.sin(top_rotation_angle)
    top_pivot_x = 3.0
    top_pivot_y = 7.0

    for i in range(actual_top_len):
        progress = i / float(actual_top_len) if actual_top_len > 0 else 0.0

        dx = 3 + i  # Extends forward from pivot

        # Base upward angle: 20 degrees
        base_angle_rise = int(i * 0.364)

        # Height curve: rises in first 40%, levels 40-70%, hooks down 70-100%
        if progress < 0.4:
            # Rising section
            dy = 7 + base_angle_rise + int(i * 0.4)  # Base angle + additional rise
        elif progress < 0.7:
            # Level section at peak
            dy = 7 + base_angle_rise + int(actual_top_len * 0.4 * 0.4)
        else:
            # Downward hook at tip
            hook_progress = (progress - 0.7) / 0.3
            peak_height = 7 + int(actual_top_len * 0.4 * 0.4)
            dy = base_angle_rise + peak_height - int(hook_progress * actual_top_len * 0.15)

        # Apply +10° rotation around pivot (3, 7)
        # Rotate center of 2x2 block
        rel_x = dx - top_pivot_x
        rel_y = dy - top_pivot_y

        rotated_x = rel_x * cos_top_rot - rel_y * sin_top_rot
        rotated_y = rel_x * sin_top_rot + rel_y * cos_top_rot

        center_x = rotated_x + top_pivot_x
        center_y = rotated_y + top_pivot_y

        # Place 2x2 block around rotated center, using floor to avoid gaps
        base_x = int(center_x)
        base_y = int(center_y)

        for dx_off in [0, 1]:
            for dy_off in [0, 1]:
                for dz in [-1, 0]:
                    horn_voxels.append((base_x + dx_off, base_y + dy_off, dz))

    # Fill gap at top horn attachment point (where horn meets body)
    # Minimal attachment - just bridge from thorax to rotated horn base
    # Thorax top is at Y=(front_body_height-1), horn base (after rotation) starts around X=3, Y=7+
    # Only fill if there's an actual gap (when body is short)
    if front_body_height < 7:
        # Body is shorter than horn base - need a bridge
        # Bridge from top of thorax to horn base
        attach_y_start = front_body_height  # Start at top of thorax
        attach_y_end = 7  # End just below horn base at Y=7
        for attach_y in range(attach_y_start, attach_y_end):
            for attach_x in range(3, 5):  # Just X=3,4 near horn pivot
                for dz in [-1, 0]:
                    horn_voxels.append((attach_x, attach_y, dz))

    # BOTTOM HORN (Cephalic - lower jaw from head)
    # Clean, smooth upward curve from below head to meet top horn
    # Origin: Y=1 (head level), extends forward with upward arc
    # Slow reduction formula: max length at slider 6, but reduces slowly when decreasing slider
    # Angled 20° upward base + additional upward curve
    # Then rotated down 30° from base position
    MAX_BOTTOM_LENGTH = 13  # Length at max slider (6)
    MAX_PRONG_SLIDER = 6
    actual_bottom_len = int(MAX_BOTTOM_LENGTH - (MAX_PRONG_SLIDER - bottom_horn_len) * 1.0)

    import math
    rotation_angle = math.radians(-30)  # Rotate down 30 degrees
    cos_rot = math.cos(rotation_angle)
    sin_rot = math.sin(rotation_angle)
    pivot_x = 3.0
    pivot_y = 1.0

    for i in range(actual_bottom_len):
        # Generate initial position
        dx = 3 + i  # Extends forward from pivot

        # Simple, clean upward curve formula
        # Base 20° angle + quadratic upward curve (accelerating rise)
        base_angle_rise = int(i * 0.364)  # 20 degree base angle
        curve_rise = int((i * i) / float(actual_bottom_len) * 0.5)  # Accelerating upward curve

        dy = 1 + base_angle_rise + curve_rise

        # Apply -30° rotation around pivot (3, 1)
        # Rotate center of 2x2 block
        rel_x = dx - pivot_x
        rel_y = dy - pivot_y

        rotated_x = rel_x * cos_rot - rel_y * sin_rot
        rotated_y = rel_x * sin_rot + rel_y * cos_rot

        center_x = rotated_x + pivot_x
        center_y = rotated_y + pivot_y

        # Place 2x2 block around rotated center, using floor to avoid gaps
        base_x = int(center_x)
        base_y = int(center_y)

        for dx_off in [0, 1]:
            for dy_off in [0, 1]:
                for dz in [-1, 0]:
                    horn_voxels.append((base_x + dx_off, base_y + dy_off, dz))

    return horn_voxels

def generate_scorpion_stinger(shaft_len=12, prong_len=5, body_length=12, back_body_height=5, stinger_curvature=0.0, tail_rotation_angle=0.0):
    """
    Generate scorpion tail/stinger - Simple version for debugging

    Args:
        shaft_len: Total tail arc length (6-18) - controls reach
        prong_len: Curvature amount (2-10) - controls curl tightness
        body_length: Body length for parametric attachment
        back_body_height: Rear body height for parametric attachment
        stinger_curvature: Dynamic curvature offset (-1.0 to +1.0) controlled by VB/NM keys
        tail_rotation_angle: Dynamic tail rotation angle in degrees (-15 to +15) controlled by VB/NM keys

    Returns:
        List of (dx, dy, dz) voxel positions for stinger
    """
    import math

    stinger_voxels = []

    # ===== SCORPION TAIL SHAFT (CLEAN REBUILD) =====
    # Attach to top of butt, smooth curve, 3 voxels thick
    # shaft_len slider controls length

    # ATTACHMENT POINT - Top of abdomen (butt)
    start_x = -body_length + 1
    start_y = back_body_height
    start_z = 0

    # SHAFT LENGTH - Directly controlled by shaft_len slider
    # More samples = smoother curve
    num_samples = int(shaft_len * 8)  # 8 samples per voxel length = smooth curve

    # CURVE SHAPE - Simple 4-point smooth arc (fewer points = smoother curve)
    # Scale EVERYTHING proportionally with shaft_len for smooth curves at all sizes
    height = int(shaft_len * 1.0)  # How high the tail arcs
    forward = int(shaft_len * 2.0)  # How far forward it reaches (LONGER, not taller)

    # Curvature adjustment (VB/NM keys)
    curl = int(stinger_curvature * shaft_len * 0.4)

    # Simple 4-point arc - NO SHARP CORNERS, gentle curve
    control_points = [
        (start_x, start_y),  # Base attachment
        (start_x + int(shaft_len * 0.4) - curl, start_y + int(shaft_len * 0.95)),  # Rising (angled up ~10deg more)
        (start_x + int(shaft_len * 1.2) - curl, start_y + height),  # Peak
        (start_x + forward - curl, start_y + height - int(shaft_len * 0.2)),  # End slightly lower
    ]

    num_control = len(control_points)
    prev_x, prev_y = None, None

    # Generate smooth curve using Catmull-Rom spline
    for i in range(num_samples):
        # Progress along curve (0.0 to 1.0)
        t = i / float(max(num_samples - 1, 1))

        # Find which segment we're interpolating
        segment_t = t * (num_control - 1)
        segment_idx = int(segment_t)
        if segment_idx >= num_control - 1:
            segment_idx = num_control - 2

        # Local t within segment
        local_t = segment_t - segment_idx

        # Get 4 control points for Catmull-Rom spline
        if segment_idx == 0:
            p0 = control_points[0]
        else:
            p0 = control_points[segment_idx - 1]

        p1 = control_points[segment_idx]
        p2 = control_points[segment_idx + 1]

        if segment_idx + 2 < num_control:
            p3 = control_points[segment_idx + 2]
        else:
            p3 = control_points[-1]

        # Catmull-Rom interpolation formula
        t2 = local_t * local_t
        t3 = t2 * local_t

        x = int(0.5 * ((2 * p1[0]) +
                       (-p0[0] + p2[0]) * local_t +
                       (2*p0[0] - 5*p1[0] + 4*p2[0] - p3[0]) * t2 +
                       (-p0[0] + 3*p1[0] - 3*p2[0] + p3[0]) * t3))

        y = int(0.5 * ((2 * p1[1]) +
                       (-p0[1] + p2[1]) * local_t +
                       (2*p0[1] - 5*p1[1] + 4*p2[1] - p3[1]) * t2 +
                       (-p0[1] + 3*p1[1] - 3*p2[1] + p3[1]) * t3))

        z = start_z

        # Fill gaps between consecutive voxels for solid shaft
        if prev_x is not None:
            dx = x - prev_x
            dy = y - prev_y
            steps = max(abs(dx), abs(dy)) + 1

            for step in range(steps):
                inter_t = step / float(steps)
                inter_x = int(prev_x + dx * inter_t)
                inter_y = int(prev_y + dy * inter_t)

                # 3 VOXELS THICK (in Z direction) + Y-overlap for continuity at all angles
                for dz in range(-1, 2):  # -1, 0, 1 = 3 layers in Z
                    for dy_overlap in range(-1, 2):  # -1, 0, 1 = 3 layers in Y for overlap
                        stinger_voxels.append((inter_x, inter_y + dy_overlap, z + dz))

        prev_x, prev_y = x, y

    # ===== SCORPION STINGER TIP (PRONG) =====
    # Pointy tip that curves down and forward like real scorpion stingers
    # prong_len slider controls length (2-10)

    if prong_len > 0:
        # Tip starts from the end of the shaft (prev_x, prev_y)
        tip_start_x = prev_x
        tip_start_y = prev_y
        tip_start_z = start_z

        # Tip length controlled by prong_len
        tip_segments = int(prong_len) * 5  # 5x segments for smooth, gap-free stinger at all lengths

        # Track previous position for interpolation
        last_tip_x = tip_start_x
        last_tip_y = tip_start_y

        # Create pointy tip - tapers from 3 voxels thick to 1 voxel (point)
        # AND curves downward and forward
        for i in range(tip_segments):
            progress = i / float(max(tip_segments - 1, 1))  # 0.0 at base, 1.0 at tip

            # Curve down and forward (like scorpion stinger)
            # Forward: progress-based so angle stays consistent regardless of segment density
            tip_x = tip_start_x + int(progress * prong_len * 1.2)

            # Downward: drops down smoothly (parabolic curve)
            drop = int(progress * progress * prong_len * 0.8)  # Use prong_len, not tip_segments
            tip_y = tip_start_y - drop

            # === INTERPOLATION: Fill gaps ONLY when needed ===
            # Calculate the distance we need to travel
            dx_step = tip_x - last_tip_x
            dy_step = tip_y - last_tip_y
            steps = max(abs(dx_step), abs(dy_step))

            # Only interpolate if there's an actual gap (steps > 1)
            # If positions are adjacent (steps <= 1), just use the current position
            if steps > 1:
                # Fill intermediate positions to close the gap
                for step in range(1, steps):  # Skip 0 (already done) and steps (will be done next iter)
                    t = step / float(steps)
                    inter_x = last_tip_x + int(dx_step * t)
                    inter_y = last_tip_y + int(dy_step * t)

                    # Taper: starts at 5 voxels thick, narrows to 1 voxel at the tip
                    if progress < 0.5:
                        # Base section: 5 voxels thick (fuller stinger) + Y-overlap
                        for dz in range(-2, 3):  # -2, -1, 0, 1, 2
                            for dy_overlap in range(-1, 2):  # Y-overlap for continuity
                                stinger_voxels.append((inter_x, inter_y + dy_overlap, tip_start_z + dz))
                    elif progress < 0.8:
                        # Middle section: 3 voxels thick + Y-overlap
                        for dz in range(-1, 2):  # -1, 0, 1
                            for dy_overlap in range(-1, 2):  # Y-overlap for continuity
                                stinger_voxels.append((inter_x, inter_y + dy_overlap, tip_start_z + dz))
                    else:
                        # Tip section: 1 voxel thick (point) + minimal Y-overlap
                        for dy_overlap in range(-1, 2):
                            stinger_voxels.append((inter_x, inter_y + dy_overlap, tip_start_z))

            # Always add the current position voxels
            if progress < 0.5:
                # Base section: 5 voxels thick (fuller stinger) + Y-overlap
                for dz in range(-2, 3):  # -2, -1, 0, 1, 2
                    for dy_overlap in range(-1, 2):  # Y-overlap for continuity
                        stinger_voxels.append((tip_x, tip_y + dy_overlap, tip_start_z + dz))
            elif progress < 0.8:
                # Middle section: 3 voxels thick + Y-overlap
                for dz in range(-1, 2):  # -1, 0, 1
                    for dy_overlap in range(-1, 2):  # Y-overlap for continuity
                        stinger_voxels.append((tip_x, tip_y + dy_overlap, tip_start_z + dz))
            else:
                # Tip section: 1 voxel thick (point) + minimal Y-overlap
                for dy_overlap in range(-1, 2):
                    stinger_voxels.append((tip_x, tip_y + dy_overlap, tip_start_z))

            # Update last position for next iteration
            last_tip_x = tip_x
            last_tip_y = tip_y

    # Rotate entire tail upward by 15 degrees (base) + dynamic tail_rotation_angle around attachment point
    # Rotation in X-Y plane (around Z-axis)
    # tail_rotation_angle: -15 (down) to +15 (up)
    angle_deg = 15.0 + tail_rotation_angle
    angle_rad = math.radians(angle_deg)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)

    rotated_voxels = []
    for (vx, vy, vz) in stinger_voxels:
        # Translate to origin (attachment point)
        rel_x = vx - start_x
        rel_y = vy - start_y

        # Rotate in X-Y plane (around Z-axis) - tilts tail upward
        new_x = rel_x * cos_a - rel_y * sin_a
        new_y = rel_x * sin_a + rel_y * cos_a

        # Translate back
        final_x = int(round(new_x + start_x))
        final_y = int(round(new_y + start_y))

        rotated_voxels.append((final_x, final_y, vz))

    return rotated_voxels

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

    # Check all leg voxels (up to 8 for scorpion, 6 for others)
    for leg_id in range(8):
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

    # Check all leg tip voxels (these are typically the lowest points) (up to 8 for scorpion)
    for leg_id in range(8):
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
def place_animated_beetle(world_x: ti.f32, world_y: ti.f32, world_z: ti.f32, rotation: ti.f32, pitch: ti.f32, roll: ti.f32, horn_pitch: ti.f32, horn_yaw: ti.f32, tail_pitch: ti.f32, horn_type_id: ti.i32, body_pitch_offset: ti.f32, body_color: ti.i32, leg_color: ti.i32, leg_tip_color: ti.i32, walk_phase: ti.f32, is_lifted_high: ti.i32, default_horn_pitch: ti.f32, body_length: ti.i32, back_body_height: ti.i32):
    """Beetle placement with 3D rotation (yaw/pitch/roll) and animated legs

    Args:
        horn_yaw: Horizontal horn rotation (stag=pincer spread, rhino/hercules=horn yaw)
        tail_pitch: Scorpion tail rotation angle (degrees, -15 to +15)
        horn_type_id: 0=rhino, 1=stag, 2=hercules, 3=scorpion
        body_pitch_offset: Static body tilt angle for scorpion (radians)
    """
    center_x = int(world_x + simulation.n_grid / 2.0)
    center_z = int(world_z + simulation.n_grid / 2.0)
    base_y = int(world_y + RENDER_Y_OFFSET)  # Apply Y offset for rendering below floor

    # Rotation matrices for yaw, pitch, roll
    cos_yaw = ti.cos(rotation)
    sin_yaw = ti.sin(rotation)
    # Combine physics pitch with static body tilt offset (for scorpion stance)
    total_pitch = pitch + body_pitch_offset
    cos_pitch_body = ti.cos(total_pitch)
    sin_pitch_body = ti.sin(total_pitch)
    cos_roll = ti.cos(roll)
    sin_roll = ti.sin(roll)

    # Pitch rotation for horn (rotation around Z-axis in local space)
    cos_pitch = ti.cos(horn_pitch)
    sin_pitch = ti.sin(horn_pitch)

    # Horn/claw pivot point in local coordinates (where horn/claw attaches to head)
    # Scorpion claws pivot at X=2, beetle horns pivot at X=3
    horn_pivot_x = 2.0 if horn_type_id == 3 else 3.0
    horn_pivot_y = 1

    # 1. Place body with horn pitch applied
    for i in range(body_cache_size[None]):
        local_x = float(body_cache_x[i])
        local_y = body_cache_y[i]
        local_z = float(body_cache_z[i])

        # Apply horn pitch and yaw rotation ONLY to horn voxels (dx >= 2 for scorpion claws, dx >= 3 for others)
        # Scorpion claws start at dx=2, beetle horns start at dx=3
        should_rotate = False
        if horn_type_id == 3:  # Scorpion - rotate claws (dx >= 2 AND |dz| > 2 to exclude centered tail)
            should_rotate = body_cache_x[i] >= 2 and abs(local_z) > 2.0
        elif body_cache_x[i] >= 3:  # Beetles - rotate horns (dx >= 3)
            should_rotate = True

        if should_rotate:
            # This is a horn voxel - apply pitch and yaw rotations around pivot point
            # Translate to pivot
            rel_x = local_x - horn_pivot_x
            rel_y = float(local_y - horn_pivot_y)
            rel_z = local_z  # Z is already centered at 0

            # STEP 1: Pitch rotation (around Z-axis in beetle local space)
            # Positive pitch = horn rotates up
            # For Hercules: ONLY bottom horn (y < 3) rotates, top horn (y >= 3) stays fixed
            cos_horn_pitch = ti.cos(horn_pitch)
            sin_horn_pitch = ti.sin(horn_pitch)

            # Initialize pitched results
            pitched_x = rel_x
            pitched_y = rel_y
            pitched_z = rel_z

            # Apply pitch rotation based on beetle type and horn position
            if horn_type_id == 3:
                # SCORPION: Both claws start at default angle, then alternate when R/Y pressed
                # Calculate deviation from default (neutral) position
                pitch_deviation = horn_pitch - default_horn_pitch

                # First apply default pitch to both claws equally
                cos_default = ti.cos(default_horn_pitch)
                sin_default = ti.sin(default_horn_pitch)
                temp_x = rel_x * cos_default - rel_y * sin_default
                temp_y = rel_x * sin_default + rel_y * cos_default

                # Then apply alternating deviation
                if pitch_deviation != 0.0:
                    cos_deviation = ti.cos(pitch_deviation)
                    sin_deviation = ti.sin(pitch_deviation)

                    if rel_z < -1.0:  # Left claw - invert deviation
                        pitched_x = temp_x * cos_deviation + temp_y * sin_deviation
                        pitched_y = -temp_x * sin_deviation + temp_y * cos_deviation
                    elif rel_z > 1.0:  # Right claw - normal deviation
                        pitched_x = temp_x * cos_deviation - temp_y * sin_deviation
                        pitched_y = temp_x * sin_deviation + temp_y * cos_deviation
                    else:  # Center voxels use default only
                        pitched_x = temp_x
                        pitched_y = temp_y
                else:  # No deviation, just use default
                    pitched_x = temp_x
                    pitched_y = temp_y
                # else: center voxels don't rotate
            elif horn_type_id == 2:
                # HERCULES: Only rotate bottom horn, top horn stays fixed
                # After -30 degree rotation, bottom horn is mostly Y < 3, with tip reaching ~Y=5
                # Top horn: After +10 degree rotation, starts at Y~8, goes up to Y=20+
                # Simple conservative rule: if Y < 5, it's bottom horn for sure
                # If Y >= 5 and Y < 8, only rotate if far forward (bottom horn tip area)
                if local_y < 5:
                    # Definitely bottom horn (rotated down 30 degrees)
                    pitched_x = rel_x * cos_horn_pitch - rel_y * sin_horn_pitch
                    pitched_y = rel_x * sin_horn_pitch + rel_y * cos_horn_pitch
                elif local_y < 8 and rel_x >= 10.0:
                    # Mid-height but far forward - bottom horn tip only
                    pitched_x = rel_x * cos_horn_pitch - rel_y * sin_horn_pitch
                    pitched_y = rel_x * sin_horn_pitch + rel_y * cos_horn_pitch
                # else: Y >= 8 or near base - top horn stays fixed
            else:
                # STAG/RHINO: Rotate entire horn
                pitched_x = rel_x * cos_horn_pitch - rel_y * sin_horn_pitch
                pitched_y = rel_x * sin_horn_pitch + rel_y * cos_horn_pitch

            # Z unchanged by pitch regardless of beetle type
            pitched_z = rel_z

            # STEP 2: Yaw rotation (around Y-axis in beetle local space)
            # Behavior differs based on beetle type
            cos_horn_yaw = ti.cos(horn_yaw)
            sin_horn_yaw = ti.sin(horn_yaw)

            # Initialize yaw results (required for Taichi)
            yawed_x = pitched_x
            yawed_y = pitched_y
            yawed_z = pitched_z

            if horn_type_id == 1:
                # STAG BEETLE: Apply opposite rotations to left vs right pincers
                # Left pincer (z < 0): positive horn_yaw opens outward (more negative Z)
                # Right pincer (z > 0): positive horn_yaw opens outward (more positive Z)
                if pitched_z < -0.1:
                    # Left pincer - apply rotation as-is
                    yawed_x = pitched_x * cos_horn_yaw + pitched_z * sin_horn_yaw
                    yawed_z = -pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
                elif pitched_z > 0.1:
                    # Right pincer - apply inverse rotation
                    yawed_x = pitched_x * cos_horn_yaw - pitched_z * sin_horn_yaw
                    yawed_z = pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
                # else: Center voxels - keep initialized values (no yaw rotation)
            elif horn_type_id == 2:
                # HERCULES BEETLE: Roll/twist both horns around X-axis (forward axis)
                # Like twisting a ball held between top and bottom hands
                # Both horns roll in same direction (B key = both twist right, V key = both twist left)
                yawed_x = pitched_x  # X unchanged (axis of rotation)
                yawed_y = pitched_y * cos_horn_yaw - pitched_z * sin_horn_yaw
                yawed_z = pitched_y * sin_horn_yaw + pitched_z * cos_horn_yaw
            else:
                # RHINO BEETLE (horn_type_id == 0): Apply uniform rotation to entire horn
                yawed_x = pitched_x * cos_horn_yaw + pitched_z * sin_horn_yaw
                yawed_z = -pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw

            # Translate back from pivot
            local_x = yawed_x + horn_pivot_x
            local_y = int(ti.round(yawed_y + float(horn_pivot_y)))
            local_z = yawed_z  # Already centered at 0

        # SCORPION TAIL ROTATION: Apply runtime tail rotation to tail voxels (not claws or body)
        # Tail starts at x = -body_length + 1 and extends forward, curving UPWARD
        if horn_type_id == 3:  # Scorpion only
            # Detect tail voxels: X range at rear, Y starting 3 voxels above attachment, Z near centerline
            # Tail is at centerline (z = -1, 0, 1) while claws are at sides (larger |z|)
            # This combination isolates tail from body/claws regardless of back_body_height
            is_tail_voxel = (body_cache_x[i] >= -body_length) and (body_cache_x[i] <= -body_length + 30) and (body_cache_y[i] >= back_body_height + 3) and (abs(body_cache_z[i]) <= 2)

            if is_tail_voxel:
                # Tail attachment point in body-local coordinates (from generate_scorpion_stinger)
                # Attachment is at top of abdomen: x = -body_length + 1, y = back_body_height, z = 0
                # Body cache uses positive X forward, so attachment is at negative X
                tail_pivot_x = -body_length + 1.0
                tail_pivot_y = float(back_body_height)

                # Translate to tail pivot
                tail_rel_x = local_x - tail_pivot_x
                tail_rel_y = float(local_y) - tail_pivot_y

                # Apply tail rotation in X-Y plane (around Z-axis)
                # tail_pitch is already in radians and includes base 15 degrees + dynamic adjustment
                cos_tail = ti.cos(tail_pitch)
                sin_tail = ti.sin(tail_pitch)

                rotated_tail_x = tail_rel_x * cos_tail - tail_rel_y * sin_tail
                rotated_tail_y = tail_rel_x * sin_tail + tail_rel_y * cos_tail

                # Translate back from tail pivot
                local_x = rotated_tail_x + tail_pivot_x
                local_y = int(ti.round(rotated_tail_y + tail_pivot_y))
                # local_z unchanged (rotation around Z-axis)

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
                # Determine if this voxel should be a racing stripe (top centerline of body)
                # Scorpions don't have racing stripes (neither on body, claws, nor tail)
                is_stripe = 0
                if horn_type_id != 3 and body_cache_x[i] < 3 and local_y >= 3 and ti.abs(local_z) <= 1.0:
                    # Top layer centerline of body (not horn, not scorpion)
                    is_stripe = 1

                # Determine if this voxel is a horn prong tip (far end of horn)
                # For stag: color the curved prong section
                # For rhino: color the Y-fork prongs
                # For hercules: color the tips of both jaws
                # For scorpion: NO tip coloring (all same body color)
                is_horn_tip = 0
                if horn_type_id == 1:
                    # Stag pincers: horns start at x=3, curved section is further out
                    # Color tips for x >= 9 (covers curved sections even at minimum size)
                    if body_cache_x[i] >= 9:
                        is_horn_tip = 1
                elif horn_type_id == 2:
                    # Hercules horns: color the far tips of both top and bottom jaws
                    # Top horn (y > 3) and bottom horn (y < 3) tips start at x >= 10
                    if body_cache_x[i] >= 10:
                        is_horn_tip = 1
                elif horn_type_id == 3:
                    # Scorpion: keep all stinger/tail voxels same body color (no tip coloring)
                    is_horn_tip = 0
                else:
                    # Rhino horn: prongs start around x=8+ (shaft + prongs)
                    # Color tips for x >= 13 (covers just the outer prong sections)
                    if body_cache_x[i] >= 13:
                        is_horn_tip = 1

                # Use appropriate color: horn tip > stripe > body color
                voxel_color = body_color
                if is_horn_tip == 1:
                    if body_color == simulation.BEETLE_BLUE:
                        voxel_color = simulation.BEETLE_BLUE_HORN_TIP
                    elif body_color == simulation.BEETLE_RED:
                        voxel_color = simulation.BEETLE_RED_HORN_TIP
                elif is_stripe == 1:
                    if body_color == simulation.BEETLE_BLUE:
                        voxel_color = simulation.BEETLE_BLUE_STRIPE
                    elif body_color == simulation.BEETLE_RED:
                        voxel_color = simulation.BEETLE_RED_STRIPE

                simulation.voxel_type[grid_x, grid_y, grid_z] = voxel_color
                # Track this voxel for efficient clearing later
                idx = ti.atomic_add(dirty_voxel_count[None], 1)
                if idx < MAX_DIRTY_VOXELS:
                    dirty_voxel_x[idx] = grid_x
                    dirty_voxel_y[idx] = grid_y
                    dirty_voxel_z[idx] = grid_z

    # 2. Place legs (animated with tripod gait) - DIFFERENT COLOR
    # Tripod gait: legs 0, 3, 4 move together (Group A), legs 1, 2, 5 move together (Group B)
    # leg_id: 0=front_left, 1=front_right, 2=middle_left, 3=middle_right, 4=rear_left, 5=rear_right
    # Scorpion adds: 6=rear2_left, 7=rear2_right

    for leg_id in range(8):  # Up to 8 legs for scorpion, 6 for beetles
        # Determine leg phase offset for gait pattern
        # SCORPION (horn_type_id == 3): Alternating wave gait
        #   Even legs (0,2,4,6): phase_offset = 0
        #   Odd legs (1,3,5,7): phase_offset = π
        # BEETLE: Tripod gait
        #   Group A (0, 3, 4): phase_offset = 0
        #   Group B (1, 2, 5): phase_offset = π
        phase_offset = 0.0
        if horn_type_id == 3:  # Scorpion: alternating gait
            if leg_id % 2 == 1:  # Odd legs
                phase_offset = 3.14159265359  # π
        else:  # Beetle: tripod gait
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
                   vtype == simulation.LEG_TIP_BLUE or vtype == simulation.LEG_TIP_RED or \
                   vtype == simulation.BEETLE_BLUE_STRIPE or vtype == simulation.BEETLE_RED_STRIPE or \
                   vtype == simulation.BEETLE_BLUE_HORN_TIP or vtype == simulation.BEETLE_RED_HORN_TIP:
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
                   vtype == simulation.LEG_TIP_BLUE or vtype == simulation.LEG_TIP_RED or \
                   vtype == simulation.BEETLE_BLUE_STRIPE or vtype == simulation.BEETLE_RED_STRIPE or \
                   vtype == simulation.BEETLE_BLUE_HORN_TIP or vtype == simulation.BEETLE_RED_HORN_TIP:
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
                    if vtype == simulation.BEETLE_BLUE or vtype == simulation.BEETLE_BLUE_LEGS or vtype == simulation.LEG_TIP_BLUE or vtype == simulation.BEETLE_BLUE_STRIPE or vtype == simulation.BEETLE_BLUE_HORN_TIP:
                        found_in_column = 1
                else:  # BEETLE_RED
                    if vtype == simulation.BEETLE_RED or vtype == simulation.BEETLE_RED_LEGS or vtype == simulation.LEG_TIP_RED or vtype == simulation.BEETLE_RED_STRIPE or vtype == simulation.BEETLE_RED_HORN_TIP:
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
    collision_has_horn_tips[None] = 0  # Reset horn tip detection

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
                        # Check if this is a horn tip voxel
                        if vtype == simulation.BEETLE_BLUE_HORN_TIP or vtype == simulation.BEETLE_RED_HORN_TIP:
                            collision_has_horn_tips[None] = 1

                        # Accumulate collision position
                        ti.atomic_add(collision_point_x[None], float(vx1) - simulation.n_grid / 2.0)
                        ti.atomic_add(collision_point_z[None], float(vz1) - simulation.n_grid / 2.0)
                        ti.atomic_add(collision_point_y[None], float(vy_grid) - RENDER_Y_OFFSET)
                        ti.atomic_add(collision_contact_count[None], 1)


def calculate_horn_length(shaft_len, prong_len, horn_type):
    """Calculate actual horn reach based on geometry and beetle type

    Args:
        shaft_len: Horn shaft length parameter
        prong_len: Horn prong length parameter
        horn_type: "rhino", "stag", or "hercules"
    """
    if horn_type == "stag":
        # Stag pincers: shaft is reduced by 2 for pivot, then prong extends
        return float((shaft_len - 2) + prong_len)
    elif horn_type == "hercules":
        # Hercules dual-jaw: top horn is shaft_len, bottom horn is prong_len
        # Return the longer top horn length as the reach
        return float(shaft_len)
    else:  # rhino
        # Rhino horn: full shaft + prong
        return float(shaft_len + prong_len)

# Initialize beetle horn dimensions with default geometry values (must be after calculate_horn_length definition)
default_shaft = 12
default_prong = 5
default_horn_type = "rhino"
initial_horn_length = calculate_horn_length(default_shaft, default_prong, default_horn_type)
beetle_blue.horn_shaft_len = default_shaft
beetle_blue.horn_prong_len = default_prong
beetle_blue.horn_length = initial_horn_length
beetle_red.horn_shaft_len = default_shaft
beetle_red.horn_prong_len = default_prong
beetle_red.horn_length = initial_horn_length

def calculate_horn_tip_position(beetle):
    """Calculate world position of horn tip using exact voxel placement transform chain"""
    # Horn tip in local coordinates (furthest voxel + safety margin)
    # For both rhino and stag: max_x = 3 + shaft_len + prong_len (approx)
    tip_local_x = beetle.horn_shaft_len + beetle.horn_prong_len + 3.0  # Actual furthest voxel + margin
    tip_local_y = 1.0  # Base height at pivot
    tip_local_z = 0.0 if beetle.horn_type == "rhino" else beetle.horn_prong_len  # Stag pincers extend sideways

    # Add safety buffer for tip width (2-3 voxels wide) + voxel edge + rotation margin
    if beetle.horn_type == "stag":
        tip_local_z += 3.0  # Stag tips extend further in Z (was 1.5)
    else:
        tip_local_x += 3.0  # Rhino tips extend further in X (was 1.0)

    # Step 1: Translate to horn pivot (3.0, 1, 0)
    rel_x = tip_local_x - 3.0
    rel_y = tip_local_y - 1.0
    rel_z = tip_local_z

    # Step 2: Apply horn pitch rotation (around Z-axis)
    cos_horn_pitch = math.cos(beetle.horn_pitch)
    sin_horn_pitch = math.sin(beetle.horn_pitch)
    pitched_x = rel_x * cos_horn_pitch - rel_y * sin_horn_pitch
    pitched_y = rel_x * sin_horn_pitch + rel_y * cos_horn_pitch
    pitched_z = rel_z

    # Step 3: Apply horn yaw rotation - behavior differs by beetle type
    cos_horn_yaw = math.cos(beetle.horn_yaw)
    sin_horn_yaw = math.sin(beetle.horn_yaw)

    if beetle.horn_type == "stag":
        # Stag: Y-axis rotation (use right pincer position - maximum extent)
        yawed_x = pitched_x * cos_horn_yaw - pitched_z * sin_horn_yaw
        yawed_z = pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
        yawed_y = pitched_y
    elif beetle.horn_type == "hercules":
        # Hercules: X-axis rotation (roll/twist around forward axis)
        yawed_x = pitched_x
        yawed_y = pitched_y * cos_horn_yaw - pitched_z * sin_horn_yaw
        yawed_z = pitched_y * sin_horn_yaw + pitched_z * cos_horn_yaw
    else:
        # Rhino: Y-axis rotation (horizontal sweep)
        yawed_x = pitched_x * cos_horn_yaw + pitched_z * sin_horn_yaw
        yawed_z = -pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
        yawed_y = pitched_y

    # Step 4: Translate back from pivot
    local_x = yawed_x + 3.0
    local_y = yawed_y + 1.0
    local_z = yawed_z

    # Step 5: Apply beetle body yaw rotation (around Y-axis)
    cos_rotation = math.cos(beetle.rotation)
    sin_rotation = math.sin(beetle.rotation)
    rotated_x = local_x * cos_rotation - local_z * sin_rotation
    rotated_z = local_x * sin_rotation + local_z * cos_rotation
    rotated_y = local_y

    # Step 6: Translate to world position
    world_x = beetle.x + rotated_x
    world_y = beetle.y + rotated_y
    world_z = beetle.z + rotated_z

    return world_x, world_y, world_z

def calculate_horn_tip_position_with_yaw(beetle, yaw_angle):
    """Calculate horn tip position with a specific yaw angle (for predictive collision checking)"""
    # Horn tip in local coordinates (same as base function)
    tip_local_x = beetle.horn_shaft_len + beetle.horn_prong_len + 3.0
    tip_local_y = 1.0
    tip_local_z = 0.0 if beetle.horn_type == "rhino" else beetle.horn_prong_len

    # Add safety buffer for tip width + voxel edge + rotation margin
    if beetle.horn_type == "stag":
        tip_local_z += 3.0  # (was 1.5)
    else:
        tip_local_x += 3.0  # (was 1.0)

    # Step 1: Translate to horn pivot
    rel_x = tip_local_x - 3.0
    rel_y = tip_local_y - 1.0
    rel_z = tip_local_z

    # Step 2: Apply horn pitch rotation (around Z-axis) - use current pitch
    cos_horn_pitch = math.cos(beetle.horn_pitch)
    sin_horn_pitch = math.sin(beetle.horn_pitch)
    pitched_x = rel_x * cos_horn_pitch - rel_y * sin_horn_pitch
    pitched_y = rel_x * sin_horn_pitch + rel_y * cos_horn_pitch
    pitched_z = rel_z

    # Step 3: Apply horn yaw rotation - behavior differs by beetle type
    cos_horn_yaw = math.cos(yaw_angle)
    sin_horn_yaw = math.sin(yaw_angle)

    if beetle.horn_type == "stag":
        # Stag: Y-axis rotation (left/right pincers spread)
        yawed_x = pitched_x * cos_horn_yaw - pitched_z * sin_horn_yaw
        yawed_z = pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
        yawed_y = pitched_y
    elif beetle.horn_type == "hercules":
        # Hercules: X-axis rotation (roll/twist around forward axis)
        yawed_x = pitched_x
        yawed_y = pitched_y * cos_horn_yaw - pitched_z * sin_horn_yaw
        yawed_z = pitched_y * sin_horn_yaw + pitched_z * cos_horn_yaw
    else:
        # Rhino: Y-axis rotation (horizontal sweep)
        yawed_x = pitched_x * cos_horn_yaw + pitched_z * sin_horn_yaw
        yawed_z = -pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
        yawed_y = pitched_y

    # Step 4: Translate back from pivot
    local_x = yawed_x + 3.0
    local_y = yawed_y + 1.0
    local_z = yawed_z

    # Step 5: Apply beetle body yaw rotation
    cos_rotation = math.cos(beetle.rotation)
    sin_rotation = math.sin(beetle.rotation)
    rotated_x = local_x * cos_rotation - local_z * sin_rotation
    rotated_z = local_x * sin_rotation + local_z * cos_rotation
    rotated_y = local_y

    # Step 6: Translate to world position
    world_x = beetle.x + rotated_x
    world_y = beetle.y + rotated_y
    world_z = beetle.z + rotated_z

    return world_x, world_y, world_z

def calculate_horn_tip_position_with_pitch(beetle, pitch_angle):
    """Calculate horn tip position with a specific pitch angle (for predictive collision checking)"""
    # Horn tip in local coordinates (same as base function)
    tip_local_x = beetle.horn_shaft_len + beetle.horn_prong_len + 3.0
    tip_local_y = 1.0
    tip_local_z = 0.0 if beetle.horn_type == "rhino" else beetle.horn_prong_len

    # Add safety buffer for tip width + voxel edge + rotation margin
    if beetle.horn_type == "stag":
        tip_local_z += 3.0  # (was 1.5)
    else:
        tip_local_x += 3.0  # (was 1.0)

    # Step 1: Translate to horn pivot
    rel_x = tip_local_x - 3.0
    rel_y = tip_local_y - 1.0
    rel_z = tip_local_z

    # Step 2: Apply horn pitch rotation (around Z-axis) - use PROPOSED pitch_angle
    cos_horn_pitch = math.cos(pitch_angle)
    sin_horn_pitch = math.sin(pitch_angle)
    pitched_x = rel_x * cos_horn_pitch - rel_y * sin_horn_pitch
    pitched_y = rel_x * sin_horn_pitch + rel_y * cos_horn_pitch
    pitched_z = rel_z

    # Step 3: Apply horn yaw rotation - behavior differs by beetle type (use current yaw)
    cos_horn_yaw = math.cos(beetle.horn_yaw)
    sin_horn_yaw = math.sin(beetle.horn_yaw)

    if beetle.horn_type == "stag":
        # Stag: Y-axis rotation (left/right pincers spread)
        yawed_x = pitched_x * cos_horn_yaw - pitched_z * sin_horn_yaw
        yawed_z = pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
        yawed_y = pitched_y
    elif beetle.horn_type == "hercules":
        # Hercules: X-axis rotation (roll/twist around forward axis)
        yawed_x = pitched_x
        yawed_y = pitched_y * cos_horn_yaw - pitched_z * sin_horn_yaw
        yawed_z = pitched_y * sin_horn_yaw + pitched_z * cos_horn_yaw
    else:
        # Rhino: Y-axis rotation (horizontal sweep)
        yawed_x = pitched_x * cos_horn_yaw + pitched_z * sin_horn_yaw
        yawed_z = -pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
        yawed_y = pitched_y

    # Step 4: Translate back from pivot
    local_x = yawed_x + 3.0
    local_y = yawed_y + 1.0
    local_z = yawed_z

    # Step 5: Apply beetle body yaw rotation
    cos_rotation = math.cos(beetle.rotation)
    sin_rotation = math.sin(beetle.rotation)
    rotated_x = local_x * cos_rotation - local_z * sin_rotation
    rotated_z = local_x * sin_rotation + local_z * cos_rotation
    rotated_y = local_y

    # Step 6: Translate to world position
    world_x = beetle.x + rotated_x
    world_y = beetle.y + rotated_y
    world_z = beetle.z + rotated_z

    return world_x, world_y, world_z

def calculate_horn_tip_position_with_both(beetle, pitch_angle, yaw_angle):
    """Calculate horn tip position with both pitch and yaw (optimized for combined movements)"""
    # Horn tip in local coordinates (same as base function)
    tip_local_x = beetle.horn_shaft_len + beetle.horn_prong_len + 3.0
    tip_local_y = 1.0
    tip_local_z = 0.0 if beetle.horn_type == "rhino" else beetle.horn_prong_len

    # Add safety buffer for tip width + voxel edge + rotation margin
    if beetle.horn_type == "stag":
        tip_local_z += 3.0  # (was 1.5)
    else:
        tip_local_x += 3.0  # (was 1.0)

    # Step 1: Translate to horn pivot
    rel_x = tip_local_x - 3.0
    rel_y = tip_local_y - 1.0
    rel_z = tip_local_z

    # Step 2: Apply horn pitch rotation (around Z-axis) - use PROPOSED pitch_angle
    cos_horn_pitch = math.cos(pitch_angle)
    sin_horn_pitch = math.sin(pitch_angle)
    pitched_x = rel_x * cos_horn_pitch - rel_y * sin_horn_pitch
    pitched_y = rel_x * sin_horn_pitch + rel_y * cos_horn_pitch
    pitched_z = rel_z

    # Step 3: Apply horn yaw rotation - behavior differs by beetle type
    cos_horn_yaw = math.cos(yaw_angle)
    sin_horn_yaw = math.sin(yaw_angle)

    if beetle.horn_type == "stag":
        # Stag: Y-axis rotation (left/right pincers spread)
        yawed_x = pitched_x * cos_horn_yaw - pitched_z * sin_horn_yaw
        yawed_z = pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
        yawed_y = pitched_y
    elif beetle.horn_type == "hercules":
        # Hercules: X-axis rotation (roll/twist around forward axis)
        yawed_x = pitched_x
        yawed_y = pitched_y * cos_horn_yaw - pitched_z * sin_horn_yaw
        yawed_z = pitched_y * sin_horn_yaw + pitched_z * cos_horn_yaw
    else:
        # Rhino: Y-axis rotation (horizontal sweep)
        yawed_x = pitched_x * cos_horn_yaw + pitched_z * sin_horn_yaw
        yawed_z = -pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
        yawed_y = pitched_y

    # Step 4: Translate back from pivot
    local_x = yawed_x + 3.0
    local_y = yawed_y + 1.0
    local_z = yawed_z

    # Step 5: Apply beetle body yaw rotation
    cos_rotation = math.cos(beetle.rotation)
    sin_rotation = math.sin(beetle.rotation)
    rotated_x = local_x * cos_rotation - local_z * sin_rotation
    rotated_z = local_x * sin_rotation + local_z * cos_rotation
    rotated_y = local_y

    # Step 6: Translate to world position
    world_x = beetle.x + rotated_x
    world_y = beetle.y + rotated_y
    world_z = beetle.z + rotated_z

    return world_x, world_y, world_z

def calculate_stag_pincer_tips(beetle, pitch_angle, yaw_angle):
    """Calculate both left and right pincer tip positions for stag beetles"""
    # Stag pincer tips in local coordinates
    tip_local_x = beetle.horn_shaft_len + beetle.horn_prong_len + 3.0
    tip_local_y = 1.0
    tip_local_z = beetle.horn_prong_len + 3.0  # Pincers extend sideways + safety buffer (was 1.5)

    # === LEFT PINCER (extends in -Z direction) ===
    # Step 1: Translate to horn pivot
    rel_x = tip_local_x - 3.0
    rel_y = tip_local_y - 1.0
    rel_z = -tip_local_z  # Negative Z for left pincer

    # Step 2: Apply horn pitch rotation
    cos_horn_pitch = math.cos(pitch_angle)
    sin_horn_pitch = math.sin(pitch_angle)
    pitched_x = rel_x * cos_horn_pitch - rel_y * sin_horn_pitch
    pitched_y = rel_x * sin_horn_pitch + rel_y * cos_horn_pitch
    pitched_z = rel_z

    # Step 3: Apply horn yaw rotation (positive yaw opens left pincer outward)
    cos_horn_yaw = math.cos(yaw_angle)
    sin_horn_yaw = math.sin(yaw_angle)
    yawed_x_left = pitched_x * cos_horn_yaw + pitched_z * sin_horn_yaw
    yawed_z_left = -pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
    yawed_y_left = pitched_y

    # Step 4: Translate back from pivot
    local_x_left = yawed_x_left + 3.0
    local_y_left = yawed_y_left + 1.0
    local_z_left = yawed_z_left

    # Step 5: Apply beetle body yaw rotation
    cos_rotation = math.cos(beetle.rotation)
    sin_rotation = math.sin(beetle.rotation)
    rotated_x_left = local_x_left * cos_rotation - local_z_left * sin_rotation
    rotated_z_left = local_x_left * sin_rotation + local_z_left * cos_rotation
    rotated_y_left = local_y_left

    # Step 6: Translate to world position
    left_tip_x = beetle.x + rotated_x_left
    left_tip_y = beetle.y + rotated_y_left
    left_tip_z = beetle.z + rotated_z_left

    # === RIGHT PINCER (extends in +Z direction) ===
    # Step 1: Translate to horn pivot
    rel_z = tip_local_z  # Positive Z for right pincer

    # Step 2: Pitch rotation (same as left)
    pitched_z = rel_z

    # Step 3: Apply horn yaw rotation (inverse rotation for right pincer)
    yawed_x_right = pitched_x * cos_horn_yaw - pitched_z * sin_horn_yaw
    yawed_z_right = pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
    yawed_y_right = pitched_y

    # Step 4: Translate back from pivot
    local_x_right = yawed_x_right + 3.0
    local_y_right = yawed_y_right + 1.0
    local_z_right = yawed_z_right

    # Step 5: Apply beetle body yaw rotation
    rotated_x_right = local_x_right * cos_rotation - local_z_right * sin_rotation
    rotated_z_right = local_x_right * sin_rotation + local_z_right * cos_rotation
    rotated_y_right = local_y_right

    # Step 6: Translate to world position
    right_tip_x = beetle.x + rotated_x_right
    right_tip_y = beetle.y + rotated_y_right
    right_tip_z = beetle.z + rotated_z_right

    return (left_tip_x, left_tip_y, left_tip_z), (right_tip_x, right_tip_y, right_tip_z)

def calculate_hercules_jaw_tips(beetle, pitch_angle, yaw_angle):
    """Calculate both top and bottom jaw tip positions for Hercules beetles"""
    import math

    # Calculate actual horn lengths using slow-reduction formula (same as geometry generation)
    MAX_TOP_LENGTH = 17
    MAX_SLIDER = 12
    actual_top_len = int(MAX_TOP_LENGTH - (MAX_SLIDER - beetle.horn_shaft_len) * 0.5)

    MAX_BOTTOM_LENGTH = 13
    MAX_PRONG_SLIDER = 6
    actual_bottom_len = int(MAX_BOTTOM_LENGTH - (MAX_PRONG_SLIDER - beetle.horn_prong_len) * 1.0)

    # === TOP JAW (extends from Y=7, rotated +10° around pivot (3,7)) ===
    # Calculate EXACT position of the last voxel (using same formula as geometry generation)
    i = actual_top_len - 1  # Last voxel index
    progress = float(i) / float(actual_top_len) if actual_top_len > 0 else 0.0

    dx = 3 + i  # Extends forward from pivot
    base_angle_rise = int(i * 0.364)  # 20 degree base angle

    # Height curve: rises in first 40%, levels 40-70%, hooks down 70-100%
    if progress < 0.4:
        # Rising section
        dy = 7 + base_angle_rise + int(i * 0.4)
    elif progress < 0.7:
        # Level section at peak
        dy = 7 + base_angle_rise + int(actual_top_len * 0.4 * 0.4)
    else:
        # Downward hook at tip
        hook_progress = (progress - 0.7) / 0.3
        peak_height = 7 + int(actual_top_len * 0.4 * 0.4)
        dy = base_angle_rise + peak_height - int(hook_progress * actual_top_len * 0.15)

    # Step 1: Translate to top horn pivot (3, 7)
    top_pivot_x = 3.0
    top_pivot_y = 7.0
    top_rel_x = float(dx) - top_pivot_x
    top_rel_y = float(dy) - top_pivot_y
    top_rel_z = 0.0

    # Step 2: Apply +10° rotation around top pivot (same as geometry generation)
    top_rotation_angle = math.radians(10)
    cos_top_rot = math.cos(top_rotation_angle)
    sin_top_rot = math.sin(top_rotation_angle)
    top_rotated_x = top_rel_x * cos_top_rot - top_rel_y * sin_top_rot
    top_rotated_y = top_rel_x * sin_top_rot + top_rel_y * cos_top_rot
    top_rotated_z = top_rel_z

    # Get center position after rotation
    center_x = top_rotated_x + top_pivot_x
    center_y = top_rotated_y + top_pivot_y

    # Account for 2x2x2 block placement (dx_off: [0,1], dy_off: [0,1], dz: [-1,0])
    # The furthest voxel is at int(center_x) + 1 in X, int(center_y) + 1 in Y
    # Z voxels at -1 and 0 can rotate to extend Y when roll applied
    top_tip_local_x = float(int(center_x) + 1) + 1.5  # +1 for block, +1.5 for width
    top_tip_local_y = float(int(center_y) + 1) + 1.5  # +1 for block, +1.5 for width + rotation
    top_tip_local_z = 0.0

    # Recalculate rotation from the buffered tip position
    top_rel_x = top_tip_local_x - 3.0
    top_rel_y = top_tip_local_y - 7.0
    top_rel_z = top_tip_local_z

    top_rotated_x = top_rel_x
    top_rotated_y = top_rel_y
    top_rotated_z = top_rel_z

    # Step 3: Apply yaw/roll (X-axis rotation) - top horn does NOT apply pitch
    cos_horn_yaw = math.cos(yaw_angle)
    sin_horn_yaw = math.sin(yaw_angle)
    top_yawed_x = top_rotated_x
    top_yawed_y = top_rotated_y * cos_horn_yaw - top_rotated_z * sin_horn_yaw
    top_yawed_z = top_rotated_y * sin_horn_yaw + top_rotated_z * cos_horn_yaw

    # Step 4: Translate back from top pivot (add 7 to Y, 3 to X)
    top_local_x = top_yawed_x + 3.0
    top_local_y = top_yawed_y + 7.0
    top_local_z = top_yawed_z

    # Step 5: Apply beetle body rotation
    cos_rotation = math.cos(beetle.rotation)
    sin_rotation = math.sin(beetle.rotation)
    top_rotated_x_body = top_local_x * cos_rotation - top_local_z * sin_rotation
    top_rotated_z_body = top_local_x * sin_rotation + top_local_z * cos_rotation
    top_rotated_y_body = top_local_y

    # Step 6: Translate to world position
    top_tip_x = beetle.x + top_rotated_x_body
    top_tip_y = beetle.y + top_rotated_y_body
    top_tip_z = beetle.z + top_rotated_z_body

    # === BOTTOM JAW (extends from Y=1, rotated -30° around pivot (3,1)) ===
    # Calculate EXACT position of the last voxel (using same formula as geometry generation)
    i_bottom = actual_bottom_len - 1  # Last voxel index

    dx_bottom = 3 + i_bottom  # Extends forward from pivot
    base_angle_rise_bottom = int(i_bottom * 0.364)  # 20 degree base angle
    curve_rise_bottom = int((i_bottom * i_bottom) / float(actual_bottom_len) * 0.5)  # Accelerating upward curve
    dy_bottom = 1 + base_angle_rise_bottom + curve_rise_bottom

    # Step 1: Translate to bottom horn pivot (3, 1)
    bottom_pivot_x = 3.0
    bottom_pivot_y = 1.0
    bottom_rel_x_pre = float(dx_bottom) - bottom_pivot_x
    bottom_rel_y_pre = float(dy_bottom) - bottom_pivot_y
    bottom_rel_z_pre = 0.0

    # Step 2: Apply -30° rotation around bottom pivot (same as geometry generation)
    bottom_rotation_angle = math.radians(-30)
    cos_bottom_rot = math.cos(bottom_rotation_angle)
    sin_bottom_rot = math.sin(bottom_rotation_angle)
    bottom_pre_rotated_x = bottom_rel_x_pre * cos_bottom_rot - bottom_rel_y_pre * sin_bottom_rot
    bottom_pre_rotated_y = bottom_rel_x_pre * sin_bottom_rot + bottom_rel_y_pre * cos_bottom_rot
    bottom_pre_rotated_z = bottom_rel_z_pre

    # Get center position after -30° rotation
    center_x_bottom = bottom_pre_rotated_x + bottom_pivot_x
    center_y_bottom = bottom_pre_rotated_y + bottom_pivot_y

    # Account for 2x2x2 block placement - furthest voxel is at int(center_x) + 1, int(center_y) + 1
    # Z voxels at -1 and 0 can rotate to extend Y when roll applied
    bottom_tip_local_x = float(int(center_x_bottom) + 1) + 1.5  # +1 for block, +1.5 for width
    bottom_tip_local_y = float(int(center_y_bottom) + 1) + 1.5  # +1 for block, +1.5 for width + rotation
    bottom_tip_local_z = 0.0

    # Recalculate from buffered tip position for pitch/yaw application
    bottom_rel_x = bottom_tip_local_x - 3.0
    bottom_rel_y = bottom_tip_local_y - 1.0
    bottom_rel_z = bottom_tip_local_z

    # Step 3: Apply pitch rotation (R/Y keys) - ONLY bottom jaw rotates with pitch
    cos_horn_pitch = math.cos(pitch_angle)
    sin_horn_pitch = math.sin(pitch_angle)
    bottom_pitched_x = bottom_rel_x * cos_horn_pitch - bottom_rel_y * sin_horn_pitch
    bottom_pitched_y = bottom_rel_x * sin_horn_pitch + bottom_rel_y * cos_horn_pitch
    bottom_pitched_z = bottom_rel_z

    # Don't re-apply the -30° rotation - it's already baked into the tip position
    bottom_rotated_x = bottom_pitched_x
    bottom_rotated_y = bottom_pitched_y
    bottom_rotated_z = bottom_pitched_z

    # Step 4: Apply yaw/roll (X-axis rotation)
    bottom_yawed_x = bottom_rotated_x
    bottom_yawed_y = bottom_rotated_y * cos_horn_yaw - bottom_rotated_z * sin_horn_yaw
    bottom_yawed_z = bottom_rotated_y * sin_horn_yaw + bottom_rotated_z * cos_horn_yaw

    # Step 5: Translate back from bottom pivot (add 1 to Y, 3 to X)
    bottom_local_x = bottom_yawed_x + 3.0
    bottom_local_y = bottom_yawed_y + 1.0
    bottom_local_z = bottom_yawed_z

    # Step 6: Apply beetle body rotation
    bottom_rotated_x_body = bottom_local_x * cos_rotation - bottom_local_z * sin_rotation
    bottom_rotated_z_body = bottom_local_x * sin_rotation + bottom_local_z * cos_rotation
    bottom_rotated_y_body = bottom_local_y

    # Step 7: Translate to world position
    bottom_tip_x = beetle.x + bottom_rotated_x_body
    bottom_tip_y = beetle.y + bottom_rotated_y_body
    bottom_tip_z = beetle.z + bottom_rotated_z_body

    return (top_tip_x, top_tip_y, top_tip_z), (bottom_tip_x, bottom_tip_y, bottom_tip_z)

def calculate_min_horn_distance(beetle1, beetle2, pitch1, yaw1, pitch2, yaw2):
    """Calculate minimum distance between horn tips (handles stag, rhino, and hercules)"""

    # CASE 1: Both rhino - simple single tip collision
    if beetle1.horn_type == "rhino" and beetle2.horn_type == "rhino":
        tip1_x, tip1_y, tip1_z = calculate_horn_tip_position_with_both(beetle1, pitch1, yaw1)
        tip2_x, tip2_y, tip2_z = calculate_horn_tip_position_with_both(beetle2, pitch2, yaw2)
        dx = tip1_x - tip2_x
        dy = tip1_y - tip2_y
        dz = tip1_z - tip2_z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    # CASE 2: Both stag - check all 4 pincer tip combinations
    if beetle1.horn_type == "stag" and beetle2.horn_type == "stag":
        (b1_left_x, b1_left_y, b1_left_z), (b1_right_x, b1_right_y, b1_right_z) = calculate_stag_pincer_tips(beetle1, pitch1, yaw1)
        (b2_left_x, b2_left_y, b2_left_z), (b2_right_x, b2_right_y, b2_right_z) = calculate_stag_pincer_tips(beetle2, pitch2, yaw2)

        # Calculate all 4 distances inline
        dx = b1_left_x - b2_left_x
        dy = b1_left_y - b2_left_y
        dz = b1_left_z - b2_left_z
        dist_ll = math.sqrt(dx*dx + dy*dy + dz*dz)

        dx = b1_left_x - b2_right_x
        dy = b1_left_y - b2_right_y
        dz = b1_left_z - b2_right_z
        dist_lr = math.sqrt(dx*dx + dy*dy + dz*dz)

        dx = b1_right_x - b2_left_x
        dy = b1_right_y - b2_left_y
        dz = b1_right_z - b2_left_z
        dist_rl = math.sqrt(dx*dx + dy*dy + dz*dz)

        dx = b1_right_x - b2_right_x
        dy = b1_right_y - b2_right_y
        dz = b1_right_z - b2_right_z
        dist_rr = math.sqrt(dx*dx + dy*dy + dz*dz)

        return min(dist_ll, dist_lr, dist_rl, dist_rr)

    # CASE 3: Both hercules - check all 4 jaw tip combinations
    if beetle1.horn_type == "hercules" and beetle2.horn_type == "hercules":
        (b1_top_x, b1_top_y, b1_top_z), (b1_bot_x, b1_bot_y, b1_bot_z) = calculate_hercules_jaw_tips(beetle1, pitch1, yaw1)
        (b2_top_x, b2_top_y, b2_top_z), (b2_bot_x, b2_bot_y, b2_bot_z) = calculate_hercules_jaw_tips(beetle2, pitch2, yaw2)

        # Calculate all 4 distances: top-top, top-bottom, bottom-top, bottom-bottom
        dx = b1_top_x - b2_top_x
        dy = b1_top_y - b2_top_y
        dz = b1_top_z - b2_top_z
        dist_tt = math.sqrt(dx*dx + dy*dy + dz*dz)

        dx = b1_top_x - b2_bot_x
        dy = b1_top_y - b2_bot_y
        dz = b1_top_z - b2_bot_z
        dist_tb = math.sqrt(dx*dx + dy*dy + dz*dz)

        dx = b1_bot_x - b2_top_x
        dy = b1_bot_y - b2_top_y
        dz = b1_bot_z - b2_top_z
        dist_bt = math.sqrt(dx*dx + dy*dy + dz*dz)

        dx = b1_bot_x - b2_bot_x
        dy = b1_bot_y - b2_bot_y
        dz = b1_bot_z - b2_bot_z
        dist_bb = math.sqrt(dx*dx + dy*dy + dz*dz)

        return min(dist_tt, dist_tb, dist_bt, dist_bb)

    # CASE 4: Mixed types - check dual-tip beetle against single or other dual-tip
    # Handle: stag vs rhino, hercules vs rhino, stag vs hercules

    # Get tips for beetle1
    if beetle1.horn_type == "stag":
        (b1_tip1_x, b1_tip1_y, b1_tip1_z), (b1_tip2_x, b1_tip2_y, b1_tip2_z) = calculate_stag_pincer_tips(beetle1, pitch1, yaw1)
    elif beetle1.horn_type == "hercules":
        (b1_tip1_x, b1_tip1_y, b1_tip1_z), (b1_tip2_x, b1_tip2_y, b1_tip2_z) = calculate_hercules_jaw_tips(beetle1, pitch1, yaw1)
    else:  # rhino
        b1_tip1_x, b1_tip1_y, b1_tip1_z = calculate_horn_tip_position_with_both(beetle1, pitch1, yaw1)
        b1_tip2_x, b1_tip2_y, b1_tip2_z = b1_tip1_x, b1_tip1_y, b1_tip1_z  # Same tip twice for rhino

    # Get tips for beetle2
    if beetle2.horn_type == "stag":
        (b2_tip1_x, b2_tip1_y, b2_tip1_z), (b2_tip2_x, b2_tip2_y, b2_tip2_z) = calculate_stag_pincer_tips(beetle2, pitch2, yaw2)
    elif beetle2.horn_type == "hercules":
        (b2_tip1_x, b2_tip1_y, b2_tip1_z), (b2_tip2_x, b2_tip2_y, b2_tip2_z) = calculate_hercules_jaw_tips(beetle2, pitch2, yaw2)
    else:  # rhino
        b2_tip1_x, b2_tip1_y, b2_tip1_z = calculate_horn_tip_position_with_both(beetle2, pitch2, yaw2)
        b2_tip2_x, b2_tip2_y, b2_tip2_z = b2_tip1_x, b2_tip1_y, b2_tip1_z  # Same tip twice for rhino

    # Calculate all 4 distances (even if some are duplicates for rhino)
    dx = b1_tip1_x - b2_tip1_x
    dy = b1_tip1_y - b2_tip1_y
    dz = b1_tip1_z - b2_tip1_z
    dist_11 = math.sqrt(dx*dx + dy*dy + dz*dz)

    dx = b1_tip1_x - b2_tip2_x
    dy = b1_tip1_y - b2_tip2_y
    dz = b1_tip1_z - b2_tip2_z
    dist_12 = math.sqrt(dx*dx + dy*dy + dz*dz)

    dx = b1_tip2_x - b2_tip1_x
    dy = b1_tip2_y - b2_tip1_y
    dz = b1_tip2_z - b2_tip1_z
    dist_21 = math.sqrt(dx*dx + dy*dy + dz*dz)

    dx = b1_tip2_x - b2_tip2_x
    dy = b1_tip2_y - b2_tip2_y
    dz = b1_tip2_z - b2_tip2_z
    dist_22 = math.sqrt(dx*dx + dy*dy + dz*dz)

    return min(dist_11, dist_12, dist_21, dist_22)

@ti.kernel
def calculate_edge_tipping_kernel(world_x: ti.f32, world_z: ti.f32, beetle_color: ti.i32, dt: ti.f32, pitch_inertia: ti.f32, roll_inertia: ti.f32):
    """GPU-accelerated edge tipping calculation"""
    # Reset outputs
    edge_tipping_vy[None] = 0.0
    edge_tipping_pitch_vel[None] = 0.0
    edge_tipping_roll_vel[None] = 0.0

    # Scan beetle voxels to calculate COG (reduced scan area for performance)
    center_x = int(world_x + simulation.n_grid / 2.0)
    center_z = int(world_z + simulation.n_grid / 2.0)
    y_base = int(RENDER_Y_OFFSET)

    x_min = ti.max(0, center_x - 15)
    x_max = ti.min(simulation.n_grid, center_x + 15)
    z_min = ti.max(0, center_z - 15)
    z_max = ti.min(simulation.n_grid, center_z + 15)
    y_min = ti.max(0, y_base - 3)
    y_max = ti.min(simulation.n_grid, y_base + 20)

    # First pass: calculate center of gravity
    sum_x = 0.0
    sum_y = 0.0
    sum_z = 0.0
    voxel_count = 0

    for i in range(x_min, x_max):
        for k in range(z_min, z_max):
            for j in range(y_min, y_max):
                vtype = simulation.voxel_type[i, j, k]
                is_beetle = 0

                if beetle_color == simulation.BEETLE_BLUE:
                    if vtype == simulation.BEETLE_BLUE or vtype == simulation.BEETLE_BLUE_LEGS or vtype == simulation.LEG_TIP_BLUE or vtype == simulation.BEETLE_BLUE_STRIPE or vtype == simulation.BEETLE_BLUE_HORN_TIP:
                        is_beetle = 1
                else:
                    if vtype == simulation.BEETLE_RED or vtype == simulation.BEETLE_RED_LEGS or vtype == simulation.LEG_TIP_RED or vtype == simulation.BEETLE_RED_STRIPE or vtype == simulation.BEETLE_RED_HORN_TIP:
                        is_beetle = 1

                if is_beetle == 1:
                    world_x_v = float(i) - simulation.n_grid / 2.0
                    world_y_v = float(j) - RENDER_Y_OFFSET
                    world_z_v = float(k) - simulation.n_grid / 2.0

                    sum_x += world_x_v
                    sum_y += world_y_v
                    sum_z += world_z_v
                    voxel_count += 1

    # Only continue if voxels were found
    if voxel_count > 0:
        cog_x = sum_x / float(voxel_count)
        cog_y = sum_y / float(voxel_count)
        cog_z = sum_z / float(voxel_count)

        # Second pass: find max lever distance for over-edge voxels
        over_edge_count = 0
        max_lever_dist = 0.0
        arena_center_x = ARENA_CENTER_X - simulation.n_grid / 2.0
        arena_center_z = ARENA_CENTER_Z - simulation.n_grid / 2.0

        for i in range(x_min, x_max):
            for k in range(z_min, z_max):
                for j in range(y_min, y_max):
                    vtype = simulation.voxel_type[i, j, k]
                    is_beetle = 0

                    if beetle_color == simulation.BEETLE_BLUE:
                        if vtype == simulation.BEETLE_BLUE or vtype == simulation.BEETLE_BLUE_LEGS or vtype == simulation.LEG_TIP_BLUE or vtype == simulation.BEETLE_BLUE_STRIPE or vtype == simulation.BEETLE_BLUE_HORN_TIP:
                            is_beetle = 1
                    else:
                        if vtype == simulation.BEETLE_RED or vtype == simulation.BEETLE_RED_LEGS or vtype == simulation.LEG_TIP_RED or vtype == simulation.BEETLE_RED_STRIPE or vtype == simulation.BEETLE_RED_HORN_TIP:
                            is_beetle = 1

                    if is_beetle == 1:
                        world_x_v = float(i) - simulation.n_grid / 2.0
                        world_z_v = float(k) - simulation.n_grid / 2.0

                        dist_from_center = ti.sqrt((world_x_v - arena_center_x)**2 + (world_z_v - arena_center_z)**2)

                        if dist_from_center > ARENA_EDGE_RADIUS:
                            over_edge_count += 1
                            lever_dist = ti.sqrt((world_x_v - cog_x)**2 + (world_z_v - cog_z)**2)
                            if lever_dist > max_lever_dist:
                                max_lever_dist = lever_dist

        # Only continue if voxels are over edge
        if over_edge_count > 0:
            tipping_force = float(over_edge_count) * EDGE_TIPPING_STRENGTH

            # Third pass: apply torque at farthest voxels (within 90% of max distance)
            total_vy = 0.0
            total_pitch = 0.0
            total_roll = 0.0
            num_far_voxels = 0

            for i in range(x_min, x_max):
                for k in range(z_min, z_max):
                    for j in range(y_min, y_max):
                        vtype = simulation.voxel_type[i, j, k]
                        is_beetle = 0

                        if beetle_color == simulation.BEETLE_BLUE:
                            if vtype == simulation.BEETLE_BLUE or vtype == simulation.BEETLE_BLUE_LEGS or vtype == simulation.LEG_TIP_BLUE or vtype == simulation.BEETLE_BLUE_STRIPE or vtype == simulation.BEETLE_BLUE_HORN_TIP:
                                is_beetle = 1
                        else:
                            if vtype == simulation.BEETLE_RED or vtype == simulation.BEETLE_RED_LEGS or vtype == simulation.LEG_TIP_RED or vtype == simulation.BEETLE_RED_STRIPE or vtype == simulation.BEETLE_RED_HORN_TIP:
                                is_beetle = 1

                        if is_beetle == 1:
                            world_x_v = float(i) - simulation.n_grid / 2.0
                            world_z_v = float(k) - simulation.n_grid / 2.0

                            dist_from_center = ti.sqrt((world_x_v - arena_center_x)**2 + (world_z_v - arena_center_z)**2)

                            if dist_from_center > ARENA_EDGE_RADIUS:
                                lever_dist = ti.sqrt((world_x_v - cog_x)**2 + (world_z_v - cog_z)**2)

                                if lever_dist > max_lever_dist * 0.9 and lever_dist > 0.0 and num_far_voxels < 3:
                                    lever_x = world_x_v - cog_x
                                    lever_z = world_z_v - cog_z

                                    lever_x_norm = lever_x / lever_dist
                                    lever_z_norm = lever_z / lever_dist

                                    total_vy -= tipping_force * dt
                                    total_pitch += (lever_z_norm * tipping_force * lever_dist / pitch_inertia) * dt
                                    total_roll += (lever_x_norm * tipping_force * lever_dist / roll_inertia) * dt
                                    num_far_voxels += 1

            edge_tipping_vy[None] = total_vy
            edge_tipping_pitch_vel[None] = total_pitch
            edge_tipping_roll_vel[None] = total_roll

@ti.kernel
def update_debris_particles(dt: ti.f32):
    """Update debris particle physics - position, velocity, lifetime (GPU parallel)"""
    gravity = 20.0  # Gravity acceleration

    # Update all active particles in parallel
    for idx in range(simulation.num_debris[None]):
        # Skip dead particles early to save GPU cycles
        if simulation.debris_lifetime[idx] <= 0.0:
            continue

        # Age the particle
        simulation.debris_lifetime[idx] -= dt

        # Update physics (gravity + position)
        simulation.debris_vel[idx].y -= gravity * dt
        simulation.debris_pos[idx] += simulation.debris_vel[idx] * dt

@ti.kernel
def cleanup_dead_debris():
    """Remove dead particles by compacting array (serial due to compaction)"""
    write_idx = 0

    for read_idx in range(simulation.num_debris[None]):
        if simulation.debris_lifetime[read_idx] > 0.0:
            # Particle still alive, keep it
            if write_idx != read_idx:
                simulation.debris_pos[write_idx] = simulation.debris_pos[read_idx]
                simulation.debris_vel[write_idx] = simulation.debris_vel[read_idx]
                simulation.debris_material[write_idx] = simulation.debris_material[read_idx]
                simulation.debris_lifetime[write_idx] = simulation.debris_lifetime[read_idx]
            write_idx += 1

    simulation.num_debris[None] = write_idx

@ti.kernel
def spawn_death_explosion(pos_x: ti.f32, pos_y: ti.f32, pos_z: ti.f32, beetle_color: ti.i32):
    """Dramatic particle burst when beetle dies - optimized"""
    num_particles = 100  # Optimized count for performance (was 180)

    # Pre-compute golden angle constant
    golden_angle = 3.14159 * (1.0 + ti.sqrt(5.0))

    for i in range(num_particles):
        # Simplified spherical distribution
        t = (i + 0.5) / num_particles
        phi = ti.acos(1.0 - 2.0 * t)  # Vertical angle
        theta = golden_angle * i  # Golden angle for horizontal

        # Higher speed to compensate for fewer particles (more spread)
        speed = 90.0 + ti.random() * 110.0  # 90-200 units/sec - faster burst

        # Convert spherical to Cartesian with upward bias
        sin_phi = ti.sin(phi)
        vx = sin_phi * ti.cos(theta) * speed
        vy = ti.abs(ti.cos(phi)) * speed * 1.5  # Upward bias (1.5x vertical)
        vz = sin_phi * ti.sin(theta) * speed

        # Add particle to debris system
        idx = ti.atomic_add(simulation.num_debris[None], 1)
        if idx < simulation.MAX_DEBRIS:
            simulation.debris_pos[idx] = ti.math.vec3(pos_x, pos_y, pos_z)
            simulation.debris_vel[idx] = ti.math.vec3(vx, vy, vz)
            simulation.debris_material[idx] = beetle_color
            simulation.debris_lifetime[idx] = 0.4 + ti.random() * 0.3  # 0.4-0.7 sec - faster cleanup

def calculate_horn_rotation_damping(beetle, collision_x, collision_y, collision_z, engagement_factor):
    """Calculate damping factor based on horn rotation direction (Phase 2: Horn Clipping Prevention)"""
    # Get horn tip position
    horn_tip_x, horn_tip_y, horn_tip_z = calculate_horn_tip_position(beetle)

    # Vector from horn tip to collision point
    to_collision_y = collision_y - horn_tip_y

    # Determine rotation direction relative to collision
    rotating_up = beetle.horn_pitch_velocity > 0.1
    rotating_down = beetle.horn_pitch_velocity < -0.1
    collision_above_tip = to_collision_y > 0.0

    # Calculate if rotating toward or away from collision
    rotating_toward = (rotating_up and collision_above_tip) or (rotating_down and not collision_above_tip)
    rotating_away = (rotating_up and not collision_above_tip) or (rotating_down and collision_above_tip)

    # Select damping amount based on direction
    if rotating_toward:
        target_damping = HORN_DAMPING_INTO * engagement_factor
    elif rotating_away:
        target_damping = HORN_DAMPING_AWAY * engagement_factor
    else:
        target_damping = HORN_DAMPING_NEUTRAL * engagement_factor

    # Smooth interpolation to avoid jerky feel
    current_damping = beetle.horn_rotation_damping
    new_damping = current_damping + (target_damping - current_damping) * HORN_DAMPING_SMOOTHING

    return new_damping


def beetle_collision(b1, b2, params):
    """Handle collision with voxel-perfect detection, pushing, and horn leverage"""
    # Fast GPU-based collision check
    has_collision = check_collision_kernel(b1.x, b1.z, b1.y, b2.x, b2.z, b2.y)

    if has_collision:
        # GPU-ACCELERATED: Calculate occupied voxels on GPU (no CPU transfer!)
        calculate_occupied_voxels_kernel(b1.x, b1.z, simulation.BEETLE_BLUE,
                                        beetle1_occupied_x, beetle1_occupied_z, beetle1_occupied_count)
        calculate_occupied_voxels_kernel(b2.x, b2.z, simulation.BEETLE_RED,
                                        beetle2_occupied_x, beetle2_occupied_z, beetle2_occupied_count)

        # GPU-ACCELERATED: Calculate collision point on GPU (no CPU transfer!)
        calculate_collision_point_kernel(0)  # overlap_count not used in kernel

        # Read results from GPU (minimal data transfer - just 5 values!)
        collision_x = collision_point_x[None]
        collision_y = collision_point_y[None]
        collision_z = collision_point_z[None]
        contact_count = collision_contact_count[None]
        has_horn_tips = collision_has_horn_tips[None]

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
                # Mark both beetles as in horn collision (blocks rotation during contact)
                b1.in_horn_collision = True
                b2.in_horn_collision = True

                # === PHASE 2: HORN ROTATION DAMPING ===
                # Calculate engagement factor (how much horn contact exists)
                normalized_contact_count = min(contact_count / HORN_ENGAGEMENT_CONTACT_THRESHOLD, 1.0)
                height_factor = min((contact_height_above_center - 2.0) / 8.0, 1.0)
                engagement_factor = (normalized_contact_count * (1.0 - HORN_ENGAGEMENT_HEIGHT_WEIGHT) +
                                    height_factor * HORN_ENGAGEMENT_HEIGHT_WEIGHT)

                # Boost engagement for tip collisions - tips should feel solid even with few voxels
                MIN_TIP_ENGAGEMENT = 0.65  # Minimum engagement when horn tips are involved
                if has_horn_tips == 1:
                    engagement_factor = max(engagement_factor, MIN_TIP_ENGAGEMENT)

                # Calculate damping for both beetles based on their rotation direction
                b1.horn_rotation_damping = calculate_horn_rotation_damping(
                    b1, collision_x, collision_y, collision_z, engagement_factor)
                b2.horn_rotation_damping = calculate_horn_rotation_damping(
                    b2, collision_x, collision_y, collision_z, engagement_factor)
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

                # Calculate height penalty (reduces lift when beetles are already airborne)
                NORMAL_HEIGHT = 2.0  # Height where lifts start weakening (lower = earlier penalty)
                HEIGHT_PENALTY_FACTOR = 0.35  # How quickly lift weakens with height (higher = steeper)
                avg_height = (b1.y + b2.y) / 2.0
                height_penalty = 1.0
                if avg_height > NORMAL_HEIGHT:
                    height_above_normal = avg_height - NORMAL_HEIGHT
                    height_penalty = 1.0 / (1.0 + height_above_normal * HEIGHT_PENALTY_FACTOR)

                # Check if both beetles are off cooldown before applying lift forces
                if b1.lift_cooldown <= 0.0 and b2.lift_cooldown <= 0.0:
                    # Cooldown expired - can apply lift force
                    if lift_advantage > ADVANTAGE_THRESHOLD:
                        # Blue has advantage - lifts red
                        # print(f"  -> BLUE lifts RED!")
                        lift_force = lift_impulse * 0.195 * height_penalty
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
                        lift_force = lift_impulse * 0.195 * height_penalty
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
                        push_force = lift_impulse * 0.06 * height_penalty
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

            # === PHASE 3: BODY ROTATION DAMPING ===
            # Track collision-induced spin direction and set damping
            # This prevents immediate rotation into collision direction, preventing horn clipping

            # For beetle 1: if angular_velocity is significant, apply damping
            if abs(b1.angular_velocity) > 0.3:  # Lower threshold for more responsiveness
                # Store spin direction: 1 = clockwise (positive), -1 = counterclockwise (negative)
                b1.collision_spin_direction = 1 if b1.angular_velocity > 0 else -1
                # Apply full damping strength on any significant collision
                b1.body_rotation_damping = BODY_ROTATION_DAMPING_STRENGTH

            # For beetle 2: same logic
            if abs(b2.angular_velocity) > 0.3:
                b2.collision_spin_direction = 1 if b2.angular_velocity > 0 else -1
                # Apply full damping strength on any significant collision
                b2.body_rotation_damping = BODY_ROTATION_DAMPING_STRENGTH

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

# Initialize gradient background for forest atmosphere
renderer.init_gradient_background()

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
horn_type = "rhino"  # Current horn type: "rhino" or "stag"
physics_frame = 0  # Frame counter for periodic cleanup
previous_stinger_curvature = 0.0  # Track stinger curvature for scorpion tail animation
previous_tail_rotation = 0.0  # Track tail rotation angle for scorpion tail animation

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
            # Rotation controls (F/H) - BLOCKED during horn collision
            if not beetle_blue.in_horn_collision:
                if window.is_pressed('f'):
                    beetle_blue.rotation -= ROTATION_SPEED * PHYSICS_TIMESTEP
                if window.is_pressed('h'):
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

            # Horn controls - OPTIMIZED for combined pitch+yaw movements
            # Calculate proposed pitch and yaw changes
            pitch_pressed = window.is_pressed('r') or window.is_pressed('y')
            yaw_pressed = window.is_pressed('v') or window.is_pressed('b')

            new_pitch = beetle_blue.horn_pitch
            new_yaw = beetle_blue.horn_yaw
            pitch_speed = 0.0
            yaw_speed = 0.0

            # Calculate new pitch if pitch keys pressed
            # Use hercules-specific limits for hercules beetle
            max_pitch_limit = HORN_MAX_PITCH_HERCULES if horn_type == "hercules" else HORN_MAX_PITCH
            min_pitch_limit = HORN_MIN_PITCH_HERCULES if horn_type == "hercules" else HORN_MIN_PITCH

            if window.is_pressed('r'):
                effective_speed = HORN_TILT_SPEED * (1.0 - beetle_blue.horn_rotation_damping)
                new_pitch = beetle_blue.horn_pitch + effective_speed * PHYSICS_TIMESTEP
                new_pitch = min(max_pitch_limit, new_pitch)
                pitch_speed = effective_speed
            elif window.is_pressed('y'):
                effective_speed = HORN_TILT_SPEED * (1.0 - beetle_blue.horn_rotation_damping)
                new_pitch = beetle_blue.horn_pitch - effective_speed * PHYSICS_TIMESTEP
                new_pitch = max(min_pitch_limit, new_pitch)
                pitch_speed = -effective_speed

            # Calculate new yaw if yaw keys pressed
            # For scorpion type, V/B control tail rotation angle instead of horn yaw
            if beetle_blue.horn_type == "scorpion":
                # SCORPION: V/B control blue beetle's tail rotation
                TAIL_ROTATION_SPEED = 30.0  # Degrees per second
                if window.is_pressed('v'):
                    # V = Rotate tail up
                    beetle_blue.tail_rotation_angle += TAIL_ROTATION_SPEED * PHYSICS_TIMESTEP
                    beetle_blue.tail_rotation_angle = min(15.0, beetle_blue.tail_rotation_angle)
                elif window.is_pressed('b'):
                    # B = Rotate tail down
                    beetle_blue.tail_rotation_angle -= TAIL_ROTATION_SPEED * PHYSICS_TIMESTEP
                    beetle_blue.tail_rotation_angle = max(-15.0, beetle_blue.tail_rotation_angle)
                # Don't set yaw_pressed for scorpion (skip horn collision checks)
                yaw_pressed = False
            else:
                # OTHER TYPES: V/B control horn yaw (claws)
                if window.is_pressed('v'):
                    effective_speed = HORN_YAW_SPEED * (1.0 - beetle_blue.horn_rotation_damping)
                    new_yaw = beetle_blue.horn_yaw - effective_speed * PHYSICS_TIMESTEP
                    new_yaw = max(HORN_MIN_YAW, new_yaw)
                    yaw_speed = -effective_speed
                elif window.is_pressed('b'):
                    effective_speed = HORN_YAW_SPEED * (1.0 - beetle_blue.horn_rotation_damping)
                    new_yaw = beetle_blue.horn_yaw + effective_speed * PHYSICS_TIMESTEP
                    new_yaw = min(HORN_MAX_YAW, new_yaw)
                    yaw_speed = effective_speed

            # Predictive collision check (optimized for combined movements + dual-pincer tracking)
            if (pitch_pressed or yaw_pressed) and beetle_red.active:
                # Calculate minimum distance between horn tips (handles both stag and rhino)
                distance = calculate_min_horn_distance(
                    beetle_blue, beetle_red,
                    new_pitch, new_yaw,
                    beetle_red.horn_pitch, beetle_red.horn_yaw
                )

                # Determine minimum distance threshold (use smaller of the two for safety)
                min_distance = min(HORN_PITCH_MIN_DISTANCE, HORN_YAW_MIN_DISTANCE) if (pitch_pressed and yaw_pressed) else (HORN_PITCH_MIN_DISTANCE if pitch_pressed else HORN_YAW_MIN_DISTANCE)

                # Apply changes if safe, otherwise block
                if distance >= min_distance:
                    beetle_blue.horn_pitch = new_pitch
                    beetle_blue.horn_yaw = new_yaw
                    beetle_blue.horn_pitch_velocity = pitch_speed
                    beetle_blue.horn_yaw_velocity = yaw_speed
                else:
                    # Blocked by collision
                    beetle_blue.horn_pitch_velocity = 0.0
                    beetle_blue.horn_yaw_velocity = 0.0
            elif pitch_pressed or yaw_pressed:
                # No other beetle - allow rotation freely
                beetle_blue.horn_pitch = new_pitch
                beetle_blue.horn_yaw = new_yaw
                beetle_blue.horn_pitch_velocity = pitch_speed
                beetle_blue.horn_yaw_velocity = yaw_speed
            else:
                # Not pressing horn keys - no velocity
                beetle_blue.horn_pitch_velocity = 0.0
                beetle_blue.horn_yaw_velocity = 0.0

            # Always add physics-driven rotation from collisions
            beetle_blue.rotation += beetle_blue.angular_velocity * PHYSICS_TIMESTEP
            beetle_blue.rotation = normalize_angle(beetle_blue.rotation)

        # === RED BEETLE CONTROLS (IJKL) - TANK STYLE ===
        if beetle_red.active and not beetle_red.is_falling:
            # Rotation controls (J/L) - BLOCKED during horn collision
            if not beetle_red.in_horn_collision:
                if window.is_pressed('j'):
                    beetle_red.rotation -= ROTATION_SPEED * PHYSICS_TIMESTEP
                if window.is_pressed('l'):
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

            # Horn controls - OPTIMIZED for combined pitch+yaw movements
            # Calculate proposed pitch and yaw changes
            pitch_pressed = window.is_pressed('u') or window.is_pressed('o')
            yaw_pressed = window.is_pressed('n') or window.is_pressed('m')

            new_pitch = beetle_red.horn_pitch
            new_yaw = beetle_red.horn_yaw
            pitch_speed = 0.0
            yaw_speed = 0.0

            # Calculate new pitch if pitch keys pressed
            # Use hercules-specific limits for hercules beetle
            max_pitch_limit = HORN_MAX_PITCH_HERCULES if horn_type == "hercules" else HORN_MAX_PITCH
            min_pitch_limit = HORN_MIN_PITCH_HERCULES if horn_type == "hercules" else HORN_MIN_PITCH

            if window.is_pressed('u'):
                effective_speed = HORN_TILT_SPEED * (1.0 - beetle_red.horn_rotation_damping)
                new_pitch = beetle_red.horn_pitch + effective_speed * PHYSICS_TIMESTEP
                new_pitch = min(max_pitch_limit, new_pitch)
                pitch_speed = effective_speed
            elif window.is_pressed('o'):
                effective_speed = HORN_TILT_SPEED * (1.0 - beetle_red.horn_rotation_damping)
                new_pitch = beetle_red.horn_pitch - effective_speed * PHYSICS_TIMESTEP
                new_pitch = max(min_pitch_limit, new_pitch)
                pitch_speed = -effective_speed

            # Calculate new yaw if yaw keys pressed
            # For scorpion type, N/M control tail rotation angle instead of horn yaw
            if beetle_red.horn_type == "scorpion":
                # SCORPION: N/M control red beetle's tail rotation
                TAIL_ROTATION_SPEED = 30.0  # Degrees per second
                if window.is_pressed('n'):
                    # N = Rotate tail up
                    beetle_red.tail_rotation_angle += TAIL_ROTATION_SPEED * PHYSICS_TIMESTEP
                    beetle_red.tail_rotation_angle = min(15.0, beetle_red.tail_rotation_angle)
                elif window.is_pressed('m'):
                    # M = Rotate tail down
                    beetle_red.tail_rotation_angle -= TAIL_ROTATION_SPEED * PHYSICS_TIMESTEP
                    beetle_red.tail_rotation_angle = max(-15.0, beetle_red.tail_rotation_angle)
                # Don't set yaw_pressed for scorpion (skip horn collision checks)
                yaw_pressed = False
            else:
                # OTHER TYPES: N/M control horn yaw (claws)
                if window.is_pressed('n'):
                    effective_speed = HORN_YAW_SPEED * (1.0 - beetle_red.horn_rotation_damping)
                    new_yaw = beetle_red.horn_yaw - effective_speed * PHYSICS_TIMESTEP
                    new_yaw = max(HORN_MIN_YAW, new_yaw)
                    yaw_speed = -effective_speed
                elif window.is_pressed('m'):
                    effective_speed = HORN_YAW_SPEED * (1.0 - beetle_red.horn_rotation_damping)
                    new_yaw = beetle_red.horn_yaw + effective_speed * PHYSICS_TIMESTEP
                    new_yaw = min(HORN_MAX_YAW, new_yaw)
                    yaw_speed = effective_speed

            # Predictive collision check (optimized for combined movements + dual-pincer tracking)
            if (pitch_pressed or yaw_pressed) and beetle_blue.active:
                # Calculate minimum distance between horn tips (handles both stag and rhino)
                distance = calculate_min_horn_distance(
                    beetle_red, beetle_blue,
                    new_pitch, new_yaw,
                    beetle_blue.horn_pitch, beetle_blue.horn_yaw
                )

                # Determine minimum distance threshold (use smaller of the two for safety)
                min_distance = min(HORN_PITCH_MIN_DISTANCE, HORN_YAW_MIN_DISTANCE) if (pitch_pressed and yaw_pressed) else (HORN_PITCH_MIN_DISTANCE if pitch_pressed else HORN_YAW_MIN_DISTANCE)

                # Apply changes if safe, otherwise block
                if distance >= min_distance:
                    beetle_red.horn_pitch = new_pitch
                    beetle_red.horn_yaw = new_yaw
                    beetle_red.horn_pitch_velocity = pitch_speed
                    beetle_red.horn_yaw_velocity = yaw_speed
                else:
                    # Blocked by collision
                    beetle_red.horn_pitch_velocity = 0.0
                    beetle_red.horn_yaw_velocity = 0.0
            elif pitch_pressed or yaw_pressed:
                # No other beetle - allow rotation freely
                beetle_red.horn_pitch = new_pitch
                beetle_red.horn_yaw = new_yaw
                beetle_red.horn_pitch_velocity = pitch_speed
                beetle_red.horn_yaw_velocity = yaw_speed
            else:
                # Not pressing horn keys - no velocity
                beetle_red.horn_pitch_velocity = 0.0
                beetle_red.horn_yaw_velocity = 0.0

            # Always add physics-driven rotation from collisions
            beetle_red.rotation += beetle_red.angular_velocity * PHYSICS_TIMESTEP
            beetle_red.rotation = normalize_angle(beetle_red.rotation)

        # Physics update
        beetle_blue.update_physics(PHYSICS_TIMESTEP)
        beetle_red.update_physics(PHYSICS_TIMESTEP)

        # Update debris particles (physics, aging)
        update_debris_particles(PHYSICS_TIMESTEP)

        # Cleanup dead debris every 2 frames (aggressive cleanup for better performance)
        if physics_frame % 2 == 0:
            cleanup_dead_debris()

        # Fall death detection - two-stage system
        POINT_OF_NO_RETURN = -5.0  # Once below this, can't recover (5 voxels below floor)
        FALL_DEATH_Y = -25.0  # Fully removed at this point (25 voxels below floor)

        # Stage 1: Point of no return - disable controls but keep rendering
        if beetle_blue.active and not beetle_blue.is_falling and beetle_blue.y < POINT_OF_NO_RETURN:
            beetle_blue.is_falling = True
            print("BLUE BEETLE IS FALLING!")
        if beetle_red.active and not beetle_red.is_falling and beetle_red.y < POINT_OF_NO_RETURN:
            beetle_red.is_falling = True
            print("RED BEETLE IS FALLING!")

        # Stage 1.5: Death explosion - dramatic particle burst midway through fall
        EXPLOSION_TRIGGER_Y = -20.0  # Trigger explosion after falling 20 voxels
        if beetle_blue.is_falling and not beetle_blue.has_exploded and beetle_blue.y < EXPLOSION_TRIGGER_Y:
            spawn_death_explosion(beetle_blue.x, beetle_blue.y + 21.0, beetle_blue.z, simulation.BEETLE_BLUE)
            beetle_blue.has_exploded = True
            print("BLUE BEETLE EXPLODED!")
        if beetle_red.is_falling and not beetle_red.has_exploded and beetle_red.y < EXPLOSION_TRIGGER_Y:
            spawn_death_explosion(beetle_red.x, beetle_red.y + 21.0, beetle_red.z, simulation.BEETLE_RED)
            beetle_red.has_exploded = True
            print("RED BEETLE EXPLODED!")

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

        # Apply edge tipping physics (GPU-accelerated)
        if beetle_blue.active and not beetle_blue.is_falling:
            floor_y_blue_check = check_floor_collision(beetle_blue.x, beetle_blue.z)
            if floor_y_blue_check <= -100.0:  # No floor support
                calculate_edge_tipping_kernel(beetle_blue.x, beetle_blue.z, simulation.BEETLE_BLUE,
                                              PHYSICS_TIMESTEP, beetle_blue.pitch_inertia, beetle_blue.roll_inertia)
                beetle_blue.vy += edge_tipping_vy[None]
                beetle_blue.pitch_velocity += edge_tipping_pitch_vel[None]
                beetle_blue.roll_velocity += edge_tipping_roll_vel[None]

        if beetle_red.active and not beetle_red.is_falling:
            floor_y_red_check = check_floor_collision(beetle_red.x, beetle_red.z)
            if floor_y_red_check <= -100.0:  # No floor support
                calculate_edge_tipping_kernel(beetle_red.x, beetle_red.z, simulation.BEETLE_RED,
                                              PHYSICS_TIMESTEP, beetle_red.pitch_inertia, beetle_red.roll_inertia)
                beetle_red.vy += edge_tipping_vy[None]
                beetle_red.pitch_velocity += edge_tipping_pitch_vel[None]
                beetle_red.roll_velocity += edge_tipping_roll_vel[None]

        # Beetle collision (voxel-perfect) - only if both beetles are active
        if beetle_blue.active and beetle_red.active:
            beetle_collision(beetle_blue, beetle_red, physics_params)

        # Subtract fixed timestep from accumulator
        accumulator -= PHYSICS_TIMESTEP
        physics_frame += 1  # Increment frame counter

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
    blue_render_horn_yaw = lerp_angle(beetle_blue.prev_horn_yaw, beetle_blue.horn_yaw, alpha)
    blue_render_tail_pitch = math.radians(15.0 + beetle_blue.tail_rotation_angle)  # Convert degrees to radians (base 15 + dynamic)

    # Interpolate red beetle state for rendering
    red_render_x = beetle_red.prev_x + (beetle_red.x - beetle_red.prev_x) * alpha
    red_render_y = beetle_red.prev_y + (beetle_red.y - beetle_red.prev_y) * alpha
    red_render_z = beetle_red.prev_z + (beetle_red.z - beetle_red.prev_z) * alpha
    red_render_rotation = lerp_angle(beetle_red.prev_rotation, beetle_red.rotation, alpha)
    red_render_pitch = lerp_angle(beetle_red.prev_pitch, beetle_red.pitch, alpha)
    red_render_roll = lerp_angle(beetle_red.prev_roll, beetle_red.roll, alpha)
    red_render_tail_pitch = math.radians(15.0 + beetle_red.tail_rotation_angle)  # Convert degrees to radians (base 15 + dynamic)
    red_render_horn_pitch = lerp_angle(beetle_red.prev_horn_pitch, beetle_red.horn_pitch, alpha)
    red_render_horn_yaw = lerp_angle(beetle_red.prev_horn_yaw, beetle_red.horn_yaw, alpha)

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

    # Convert horn_type string to horn_type_id: 0=rhino, 1=stag, 2=hercules, 3=scorpion
    horn_type_id = 1 if horn_type == "stag" else (2 if horn_type == "hercules" else (3 if horn_type == "scorpion" else 0))

    # Get default horn pitch for current beetle type
    if horn_type == "scorpion":
        default_horn_pitch = HORN_DEFAULT_PITCH_SCORPION
    elif horn_type == "stag":
        default_horn_pitch = HORN_DEFAULT_PITCH_STAG
    elif horn_type == "hercules":
        default_horn_pitch = HORN_DEFAULT_PITCH_HERCULES
    else:
        default_horn_pitch = HORN_DEFAULT_PITCH

    # Initialize persistent slider values (needed for kernel parameters)
    if not hasattr(window, 'horn_shaft_value'):
        window.horn_shaft_value = 12
        window.horn_prong_value = 5
        window.back_body_height_value = 5
        window.body_length_value = 12
        window.body_width_value = 7
        window.leg_length_value = 10

    if beetle_blue.active:
        place_animated_beetle(blue_render_x, blue_render_y, blue_render_z, blue_render_rotation, blue_render_pitch, blue_render_roll, blue_render_horn_pitch, blue_render_horn_yaw, blue_render_tail_pitch, horn_type_id, beetle_blue.body_pitch_offset, simulation.BEETLE_BLUE, simulation.BEETLE_BLUE_LEGS, simulation.LEG_TIP_BLUE, beetle_blue.walk_phase, 1 if beetle_blue.is_lifted_high else 0, default_horn_pitch, window.body_length_value, window.back_body_height_value)
    if beetle_red.active:
        place_animated_beetle(red_render_x, red_render_y, red_render_z, red_render_rotation, red_render_pitch, red_render_roll, red_render_horn_pitch, red_render_horn_yaw, red_render_tail_pitch, horn_type_id, beetle_red.body_pitch_offset, simulation.BEETLE_RED, simulation.BEETLE_RED_LEGS, simulation.LEG_TIP_RED, beetle_red.walk_phase, 1 if beetle_red.is_lifted_high else 0, default_horn_pitch, window.body_length_value, window.back_body_height_value)

    canvas.set_background_color((0.18, 0.40, 0.22))  # Lighter forest green
    renderer.render(camera, canvas, scene, simulation.voxel_type, simulation.n_grid)
    canvas.scene(scene)

    # HUD
    window.GUI.begin("Beetle Physics", 0.01, 0.01, 0.35, 0.52)
    window.GUI.text(f"FPS: {actual_fps:.0f}")
    window.GUI.text("")

    # Blue beetle status
    if beetle_blue.active:
        window.GUI.text("BLUE BEETLE (TFGH + RY + VB)")
        window.GUI.text(f"  Pos: ({beetle_blue.x:.1f}, {beetle_blue.y:.1f}, {beetle_blue.z:.1f})")
        window.GUI.text(f"  Speed: {math.sqrt(beetle_blue.vx**2 + beetle_blue.vz**2):.1f}")
        window.GUI.text(f"  Facing: {math.degrees(beetle_blue.rotation):.0f}°")
        window.GUI.text(f"  Horn pitch: {math.degrees(beetle_blue.horn_pitch):.1f}°")
        window.GUI.text(f"  Horn yaw: {math.degrees(beetle_blue.horn_yaw):.1f}°")
    else:
        window.GUI.text("BLUE BEETLE: FALLEN")

    window.GUI.text("")

    # Red beetle status
    if beetle_red.active:
        window.GUI.text("RED BEETLE (IJKL + UO + NM)")
        window.GUI.text(f"  Pos: ({beetle_red.x:.1f}, {beetle_red.y:.1f}, {beetle_red.z:.1f})")
        window.GUI.text(f"  Speed: {math.sqrt(beetle_red.vx**2 + beetle_red.vz**2):.1f}")
        window.GUI.text(f"  Facing: {math.degrees(beetle_red.rotation):.0f}°")
        window.GUI.text(f"  Horn pitch: {math.degrees(beetle_red.horn_pitch):.1f}°")
        window.GUI.text(f"  Horn yaw: {math.degrees(beetle_red.horn_yaw):.1f}°")
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

    # Front body (thorax) is fixed at 4 layers
    front_body_height = 4

    # Use slider_int for discrete voxel values (max 18 voxel reach: shaft 12 + prong 6)
    new_shaft = window.GUI.slider_int("Horn Shaft", window.horn_shaft_value, 5, 12)
    new_prong = window.GUI.slider_int("Horn Prong", window.horn_prong_value, 3, 6)
    new_back_body = window.GUI.slider_int("Back Body", window.back_body_height_value, 4, 8)
    new_body_length = window.GUI.slider_int("Body Length", window.body_length_value, 9, 14)
    new_body_width = window.GUI.slider_int("Body Width", window.body_width_value, 5, 9)
    new_leg_length = window.GUI.slider_int("Leg Length", window.leg_length_value, 6, 10)

    # Rebuild geometry if sliders changed OR if scorpion tail curvature changed
    # Only check scorpion-specific values when horn_type is scorpion (optimization: skip for other beetles)
    if (new_shaft != window.horn_shaft_value or new_prong != window.horn_prong_value or
        new_back_body != window.back_body_height_value or new_body_length != window.body_length_value or
        new_body_width != window.body_width_value or new_leg_length != window.leg_length_value or
        (horn_type == "scorpion" and abs(beetle_blue.stinger_curvature - previous_stinger_curvature) > 0.01)):

        # Map horn shaft slider to shorter geometry range with slower decay (only when rebuilding)
        # Slider 5→5, Slider 12→9, with power curve for slower decay at high end
        slider_normalized = (new_shaft - 5) / 7.0  # Normalize to 0-1
        curved = slider_normalized ** 0.6  # Power < 1 gives slower decay
        mapped_shaft = int(round(5 + curved * 4))  # Map to 5-9 range

        # For scorpion type, use blue beetle's values (both beetles share same geometry, only blue controls it)
        current_stinger_curvature = 0.0
        current_tail_rotation = 0.0
        if horn_type == "scorpion":
            current_stinger_curvature = beetle_blue.stinger_curvature
            current_tail_rotation = beetle_blue.tail_rotation_angle

        rebuild_beetle_geometry(mapped_shaft, new_prong, front_body_height, new_back_body, new_body_length, new_body_width, new_leg_length, horn_type, current_stinger_curvature, current_tail_rotation)
        ti.sync()  # Ensure CPU writes complete before GPU kernels read
        window.horn_shaft_value = new_shaft
        window.horn_prong_value = new_prong
        window.back_body_height_value = new_back_body
        window.body_length_value = new_body_length
        window.body_width_value = new_body_width
        window.leg_length_value = new_leg_length
        previous_stinger_curvature = current_stinger_curvature
        previous_tail_rotation = current_tail_rotation

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

    # Horn type cycle button (rhino → stag → hercules → rhino)
    window.GUI.text("")
    if horn_type == "rhino":
        button_text = "Horn Type: RHINO (click for STAG)"
    elif horn_type == "stag":
        button_text = "Horn Type: STAG (click for HERCULES)"
    elif horn_type == "hercules":
        button_text = "Horn Type: HERCULES (click for SCORPION)"
    else:  # scorpion
        button_text = "Horn Type: SCORPION (click for RHINO)"

    if window.GUI.button(button_text):
        # Cycle horn type: rhino → stag → hercules → scorpion → rhino
        if horn_type == "rhino":
            horn_type = "stag"
        elif horn_type == "stag":
            horn_type = "hercules"
        elif horn_type == "hercules":
            horn_type = "scorpion"
        else:
            horn_type = "rhino"

        print(f"Switching to {horn_type.upper()} beetle...")

        # Update horn pitch based on beetle type
        if horn_type == "scorpion":
            # Scorpion claws angled up 20 degrees by default
            beetle_blue.horn_pitch = HORN_DEFAULT_PITCH_SCORPION
            beetle_red.horn_pitch = HORN_DEFAULT_PITCH_SCORPION
            beetle_blue.prev_horn_pitch = HORN_DEFAULT_PITCH_SCORPION
            beetle_red.prev_horn_pitch = HORN_DEFAULT_PITCH_SCORPION
        elif horn_type == "stag":
            beetle_blue.horn_pitch = HORN_DEFAULT_PITCH_STAG
            beetle_red.horn_pitch = HORN_DEFAULT_PITCH_STAG
            beetle_blue.prev_horn_pitch = HORN_DEFAULT_PITCH_STAG
            beetle_red.prev_horn_pitch = HORN_DEFAULT_PITCH_STAG
        elif horn_type == "hercules":
            beetle_blue.horn_pitch = HORN_DEFAULT_PITCH_HERCULES
            beetle_red.horn_pitch = HORN_DEFAULT_PITCH_HERCULES
            beetle_blue.prev_horn_pitch = HORN_DEFAULT_PITCH_HERCULES
            beetle_red.prev_horn_pitch = HORN_DEFAULT_PITCH_HERCULES
        else:  # rhino
            beetle_blue.horn_pitch = HORN_DEFAULT_PITCH
            beetle_red.horn_pitch = HORN_DEFAULT_PITCH
            beetle_blue.prev_horn_pitch = HORN_DEFAULT_PITCH
            beetle_red.prev_horn_pitch = HORN_DEFAULT_PITCH

        # Rebuild geometry with current slider values and new horn type
        # Apply horn shaft mapping (slider 5-12 → geometry 5-9)
        slider_normalized = (window.horn_shaft_value - 5) / (12 - 5)
        curved = slider_normalized ** 0.6
        mapped_shaft = int(round(5 + curved * (9 - 5)))

        rebuild_beetle_geometry(
            mapped_shaft,
            window.horn_prong_value,
            front_body_height,
            window.back_body_height_value,
            window.body_length_value,
            window.body_width_value,
            window.leg_length_value,
            horn_type,
            stinger_curvature=0.0  # Reset tail to neutral when switching types
        )
        ti.sync()

        print(f"{horn_type.upper()} beetle activated! Shaft={window.horn_shaft_value}, Prong={window.horn_prong_value}")

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
