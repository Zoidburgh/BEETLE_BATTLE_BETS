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
MOVE_FORCE = 120.0  # Increased for more responsive feel
FRICTION = 0.88  # Slightly higher friction
MAX_SPEED = 7.0  # Slightly higher top speed
ROTATION_SPEED = 3.0  # Even slower rotation (6x slower than original) for more controlled turning
ANGULAR_FRICTION = 0.85  # How quickly spin slows down (lower = more spin)
MAX_ANGULAR_SPEED = 8.0  # Max spin speed (radians/sec)
TORQUE_MULTIPLIER = 1.5  # Rotational forces on collision
RESTITUTION = 0.0  # Bounce coefficient (0 = no bounce, 1 = full bounce)
IMPULSE_MULTIPLIER = 0.32  # Linear momentum transfer
MOMENT_OF_INERTIA_FACTOR = 1.15  # Resistance to rotation
ARENA_RADIUS = 42.5  # Half size for closer combat

# Beetle state with physics
class Beetle:
    def __init__(self, x, z, rotation, color):
        self.x = x
        self.z = z
        self.vx = 0.0  # Velocity
        self.vz = 0.0
        self.rotation = rotation
        self.target_rotation = rotation
        self.angular_velocity = 0.0  # Spin speed (radians/sec)
        self.moment_of_inertia = BEETLE_RADIUS * MOMENT_OF_INERTIA_FACTOR  # Resistance to rotation (higher = more stable, less flinging)
        self.color = color
        self.radius = BEETLE_RADIUS
        self.occupied_voxels = set()  # Track voxel positions for accurate collision

        # Animation state for procedural leg movement
        self.walk_phase = 0.0  # Current walk cycle phase (0 to 2π)
        self.is_moving = False  # Whether beetle is actively walking

    def apply_force(self, fx, fz, dt):
        """Add force to velocity"""
        self.vx += fx * dt
        self.vz += fz * dt

    def update_physics(self, dt):
        """Apply friction and update position"""
        # Apply linear friction
        self.vx *= FRICTION
        self.vz *= FRICTION

        # Apply angular friction
        self.angular_velocity *= ANGULAR_FRICTION

        # Clamp linear speed
        speed = math.sqrt(self.vx**2 + self.vz**2)
        if speed > MAX_SPEED:
            self.vx = (self.vx / speed) * MAX_SPEED
            self.vz = (self.vz / speed) * MAX_SPEED

        # Clamp angular speed
        if abs(self.angular_velocity) > MAX_ANGULAR_SPEED:
            self.angular_velocity = MAX_ANGULAR_SPEED if self.angular_velocity > 0 else -MAX_ANGULAR_SPEED

        # Update position
        self.x += self.vx * dt
        self.z += self.vz * dt

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

# Create beetles - closer together for smaller arena
beetle_blue = Beetle(-20.0, 0.0, math.pi, simulation.BEETLE_BLUE)
beetle_red = Beetle(20.0, 0.0, 0.0, simulation.BEETLE_RED)

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

    # Tighter collision bounds - only check where beetles can actually overlap
    # Beetles are max 18 voxels from center (horn extends to ~15, body to ~12)
    x_min = ti.max(ti.max(0, center1_x - 18), ti.max(0, center2_x - 18))
    x_max = ti.min(ti.min(simulation.n_grid, center1_x + 18),
                   ti.min(simulation.n_grid, center2_x + 18))
    z_min = ti.max(ti.max(0, center1_z - 18), ti.max(0, center2_z - 18))
    z_max = ti.min(ti.min(simulation.n_grid, center1_z + 18),
                   ti.min(simulation.n_grid, center2_z + 18))

    # Scan for colliding voxels - TRUE 3D collision detection
    # Track Y-ranges for each beetle in each XZ column
    for gx in range(x_min, x_max):
        for gz in range(z_min, z_max):
            # Track Y ranges for both beetles in this XZ column
            beetle1_y_min = 999
            beetle1_y_max = -1
            beetle2_y_min = 999
            beetle2_y_max = -1

            # Find Y-ranges for both beetles in this column (includes legs!)
            for gy in range(0, 15):
                voxel = simulation.voxel_type[gx, gy, gz]
                if voxel == simulation.BEETLE_BLUE or voxel == simulation.BEETLE_BLUE_LEGS:
                    if gy < beetle1_y_min:
                        beetle1_y_min = gy
                    if gy > beetle1_y_max:
                        beetle1_y_max = gy
                elif voxel == simulation.BEETLE_RED or voxel == simulation.BEETLE_RED_LEGS:
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

def calculate_occupied_voxels(world_x, world_z, rotation):
    """Calculate which voxels this beetle occupies - for torque calculation only"""
    center_x = int(world_x + simulation.n_grid / 2.0)
    center_z = int(world_z + simulation.n_grid / 2.0)

    occupied = set()

    # Read actual voxel grid - only called during collision
    voxels = simulation.voxel_type.to_numpy()

    # Check beetle's range
    x_min = max(0, center_x - 20)
    x_max = min(simulation.n_grid, center_x + 20)
    z_min = max(0, center_z - 20)
    z_max = min(simulation.n_grid, center_z + 20)

    for grid_x in range(x_min, x_max):
        for grid_z in range(z_min, z_max):
            for grid_y in range(0, 15):
                voxel = voxels[grid_x, grid_y, grid_z]
                if voxel == simulation.BEETLE_BLUE or voxel == simulation.BEETLE_RED:
                    occupied.add((grid_x, grid_z))
                    break

    return occupied

@ti.kernel
def place_beetle_rotated(world_x: ti.f32, world_z: ti.f32, rotation: ti.f32, color_type: ti.i32):
    """LEAN 1/3 SCALE BEETLE - Exaggerated legs & horn for animation"""
    center_x = int(world_x + simulation.n_grid / 2.0)
    center_z = int(world_z + simulation.n_grid / 2.0)

    cos_r = ti.cos(rotation)
    sin_r = ti.sin(rotation)

    # ========== ABDOMEN (rear) - LEAN ellipse ==========
    # 10 voxels long × 6 wide × 5 tall (reduced from 8)
    # Position: X from -12 to -2
    for dx in range(-12, -1):
        for dy in range(0, 5):
            for dz in range(-3, 4):
                # Lean elliptical shape
                length_pos = (dx + 6.5) / 5.0
                height_factor = 1.0 - ((dy - 2.5) / 2.5) ** 2
                width_at_height = 3.0 * ti.sqrt(height_factor) if height_factor > 0 else 0
                length_taper = 1.0 - (length_pos * length_pos)
                max_width = width_at_height * ti.sqrt(length_taper) if length_taper > 0 else 0

                if ti.abs(dz) <= max_width:
                    local_x = float(dx)
                    local_z = float(dz)
                    rotated_x = local_x * cos_r - local_z * sin_r
                    rotated_z = local_x * sin_r + local_z * cos_r
                    grid_x = center_x + int(ti.round(rotated_x))
                    grid_y = 2 + dy
                    grid_z = center_z + int(ti.round(rotated_z))
                    if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                        simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

    # ========== THORAX (middle) - LEAN with small pronotum bump ==========
    # 3 voxels long × 4 wide × 2 tall (reduced from 4)
    for dx in range(-1, 2):
        for dy in range(0, 2):
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
                    grid_y = 2 + dy
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
                    grid_y = 2 + dy
                    grid_z = center_z + int(ti.round(rotated_z))
                    if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                        simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

    # ========== EXAGGERATED Y-SHAPED HORN ==========
    # Main shaft - 12 voxels, curves upward, DENSE/SOLID
    for i in range(12):
        dx = 3 + i
        height_curve = int(i * 0.3)  # Slightly lower curve due to body height reduction
        dy = 1 + height_curve

        # Thick solid horn - no thin tip
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

    # EXAGGERATED Y-fork - THICK SOLID prongs spreading wide
    # Left prong - 5 voxels angling left and up, NOW THICK
    for i in range(5):
        dx = 13 + i
        dy = 4 + i  # Adjusted for body height reduction
        center_dz = -i  # Angles left

        # Make prong thick (2 voxels radius)
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

    # Right prong - 5 voxels angling right and up, NOW THICK
    for i in range(5):
        dx = 13 + i
        dy = 4 + i  # Adjusted for body height reduction
        center_dz = i  # Angles right

        # Make prong thick (2 voxels radius)
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
def generate_beetle_geometry():
    """Generate beetle geometry separated into body and legs for animation"""
    body_voxels = []
    leg_voxels = []  # Will be organized as list of 6 legs, each containing voxel offsets

    # ABDOMEN (rear) - LEAN ellipse
    for dx in range(-12, -1):
        for dy in range(0, 5):
            for dz in range(-3, 4):
                length_pos = (dx + 6.5) / 5.0
                height_factor = 1.0 - ((dy - 2.5) / 2.5) ** 2
                if height_factor > 0:
                    width_at_height = 3.0 * math.sqrt(height_factor)
                    length_taper = 1.0 - (length_pos * length_pos)
                    if length_taper > 0:
                        max_width = width_at_height * math.sqrt(length_taper)
                        if abs(dz) <= max_width:
                            body_voxels.append((dx, dy, dz))

    # Pyramid top layer for abdomen - inset by 1 voxel
    for dx in range(-11, -2):
        for dz in range(-2, 3):
            length_pos = (dx + 6.5) / 5.0
            height_factor = 1.0 - ((5 - 2.5) / 2.5) ** 2
            if height_factor > 0:
                width_at_height = 3.0 * math.sqrt(height_factor)
                length_taper = 1.0 - (length_pos * length_pos)
                if length_taper > 0:
                    max_width = width_at_height * math.sqrt(length_taper)
                    # Inset by 1 voxel from edges
                    if abs(dz) <= max_width - 1 and abs(dz) > 0:
                        body_voxels.append((dx, 5, dz))

    # THORAX (middle) - LEAN with small pronotum bump
    for dx in range(-1, 2):
        for dy in range(0, 2):
            for dz in range(-2, 3):
                width_at_height = 2.0 if dy < 1 else 1.5
                length_factor = 1.0 - abs(dx) / 1.5
                max_width = width_at_height * length_factor
                if abs(dz) <= max_width:
                    body_voxels.append((dx, dy, dz))

    # Pyramid top layer for thorax - inset by 1 voxel
    for dx in range(-1, 2):
        for dz in range(-1, 2):
            if abs(dz) > 0:  # Not on the centerline, inset from edges
                body_voxels.append((dx, 2, dz))

    # Additional pyramid layer on top of thorax - 7x3 ridge (tapered at edges)
    for dx in range(-3, 4):  # 7 voxels long
        # Taper the width at the edges
        if abs(dx) == 3:  # Outermost edges - only centerline
            body_voxels.append((dx, 3, 0))
        elif abs(dx) == 2:  # Second layer - single voxel wide
            for dz in range(-0, 1):
                body_voxels.append((dx, 3, dz))
        else:  # Center section (dx = -1, 0, 1) - full 3 voxels wide
            for dz in range(-1, 2):
                body_voxels.append((dx, 3, dz))

    # HEAD (front) - LEAN
    for dx in range(2, 4):
        for dy in range(0, 1):
            for dz in range(-1, 2):
                width_at_height = 1.5
                if abs(dz) <= width_at_height:
                    body_voxels.append((dx, dy, dz))

    # EXAGGERATED Y-SHAPED HORN - Main shaft with overlapping layers
    for i in range(12):
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
    for i in range(5):
        dx = 13 + i
        dy = 4 + i
        center_dz = -i
        for dz_offset in range(0, 2):  # Changed from range(-1, 2) to range(0, 2) for 2-voxel width
            dz = center_dz + dz_offset
            # Add voxel at current height
            body_voxels.append((dx, dy, dz))
            # Add overlapping voxel below
            body_voxels.append((dx, dy - 1, dz))

    # Right prong with overlapping layers
    for i in range(5):
        dx = 13 + i
        dy = 4 + i
        center_dz = i
        for dz_offset in range(-1, 1):  # Changed from range(-1, 2) to range(-1, 1) for 2-voxel width
            dz = center_dz + dz_offset
            # Add voxel at current height
            body_voxels.append((dx, dy, dz))
            # Add overlapping voxel below
            body_voxels.append((dx, dy - 1, dz))

    # IMPROVED LEGS (6 total) - Separated for animation
    # Leg order: [front_left, front_right, middle_left, middle_right, rear_left, rear_right]

    # Front left leg (leg 0) - MOVED FORWARD 1 VOXEL
    front_left = []
    side = -1
    # COXA
    for i in range(2):
        for extra_y in range(2):
            front_left.append((2, 1 + extra_y, side * (3 + i)))
    # FEMUR
    for i in range(4):
        for extra_y in range(2):
            front_left.append((2, 0 + extra_y, side * (5 + i)))
    # TIBIA
    for point in [(2, 0, side * 9), (2, 0, side * 10), (3, 0, side * 10), (4, 0, side * 10)]:
        front_left.append(point)
    leg_voxels.append(front_left)

    # Front right leg (leg 1) - MOVED FORWARD 1 VOXEL
    front_right = []
    side = 1
    for i in range(2):
        for extra_y in range(2):
            front_right.append((2, 1 + extra_y, side * (3 + i)))
    for i in range(4):
        for extra_y in range(2):
            front_right.append((2, 0 + extra_y, side * (5 + i)))
    for point in [(2, 0, side * 9), (2, 0, side * 10), (3, 0, side * 10), (4, 0, side * 10)]:
        front_right.append(point)
    leg_voxels.append(front_right)

    # Middle left leg (leg 2)
    middle_left = []
    side = -1
    for i in range(2):
        for extra_y in range(2):
            middle_left.append((-2, 1 + extra_y, side * (3 + i)))
    for i in range(4):
        for extra_y in range(2):
            middle_left.append((-2, 0 + extra_y, side * (5 + i)))
    for point in [(-2, 0, side * 9), (-2, 0, side * 10), (-3, 0, side * 10), (-4, 0, side * 10)]:
        middle_left.append(point)
    leg_voxels.append(middle_left)

    # Middle right leg (leg 3)
    middle_right = []
    side = 1
    for i in range(2):
        for extra_y in range(2):
            middle_right.append((-2, 1 + extra_y, side * (3 + i)))
    for i in range(4):
        for extra_y in range(2):
            middle_right.append((-2, 0 + extra_y, side * (5 + i)))
    for point in [(-2, 0, side * 9), (-2, 0, side * 10), (-3, 0, side * 10), (-4, 0, side * 10)]:
        middle_right.append(point)
    leg_voxels.append(middle_right)

    # Rear left leg (leg 4)
    rear_left = []
    side = -1
    for i in range(2):
        for extra_y in range(2):
            rear_left.append((-5, 1 + extra_y, side * (3 + i)))
    for i in range(4):
        for extra_y in range(2):
            rear_left.append((-5 - i, 0 + extra_y, side * (5 + i)))
    for point in [(-9, 0, side * 9), (-9, 0, side * 10), (-10, 0, side * 10), (-11, 0, side * 10)]:
        rear_left.append(point)
    leg_voxels.append(rear_left)

    # Rear right leg (leg 5)
    rear_right = []
    side = 1
    for i in range(2):
        for extra_y in range(2):
            rear_right.append((-5, 1 + extra_y, side * (3 + i)))
    for i in range(4):
        for extra_y in range(2):
            rear_right.append((-5 - i, 0 + extra_y, side * (5 + i)))
    for point in [(-9, 0, side * 9), (-9, 0, side * 10), (-10, 0, side * 10), (-11, 0, side * 10)]:
        rear_right.append(point)
    leg_voxels.append(rear_right)

    return body_voxels, leg_voxels

# Generate beetle geometry ONCE at startup
BEETLE_BODY, BEETLE_LEGS = generate_beetle_geometry()
print(f"Beetle geometry cached: {len(BEETLE_BODY)} body voxels + {sum(len(leg) for leg in BEETLE_LEGS)} leg voxels")

# Create Taichi fields for body geometry cache
body_cache_size = len(BEETLE_BODY)
body_cache_x = ti.field(ti.i32, shape=body_cache_size)
body_cache_y = ti.field(ti.i32, shape=body_cache_size)
body_cache_z = ti.field(ti.i32, shape=body_cache_size)

# Create Taichi fields for leg geometry cache (6 legs)
# Store all leg voxels flattened with offsets to know where each leg starts
leg_voxel_counts = [len(leg) for leg in BEETLE_LEGS]
total_leg_voxels = sum(leg_voxel_counts)
leg_cache_x = ti.field(ti.i32, shape=total_leg_voxels)
leg_cache_y = ti.field(ti.i32, shape=total_leg_voxels)
leg_cache_z = ti.field(ti.i32, shape=total_leg_voxels)

# Leg start indices for each of the 6 legs
leg_start_idx = ti.field(ti.i32, shape=6)
leg_end_idx = ti.field(ti.i32, shape=6)

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

@ti.kernel
def place_animated_beetle(world_x: ti.f32, world_z: ti.f32, rotation: ti.f32, body_color: ti.i32, leg_color: ti.i32, walk_phase: ti.f32):
    """Beetle placement with animated legs using tripod gait - separate body and leg colors"""
    center_x = int(world_x + simulation.n_grid / 2.0)
    center_z = int(world_z + simulation.n_grid / 2.0)

    cos_r = ti.cos(rotation)
    sin_r = ti.sin(rotation)

    # 1. Place body (static)
    for i in range(body_cache_size):
        local_x = float(body_cache_x[i])
        local_y = body_cache_y[i]
        local_z = float(body_cache_z[i])

        # 2D rotation
        rotated_x = local_x * cos_r - local_z * sin_r
        rotated_z = local_x * sin_r + local_z * cos_r

        grid_x = center_x + int(ti.round(rotated_x))
        grid_y = 2 + local_y
        grid_z = center_z + int(ti.round(rotated_z))

        if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
            simulation.voxel_type[grid_x, grid_y, grid_z] = body_color

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

        # Place all voxels for this leg
        start_idx = leg_start_idx[leg_id]
        end_idx = leg_end_idx[leg_id]

        for i in range(start_idx, end_idx):
            local_x = float(leg_cache_x[i]) + sweep  # Apply sweep
            local_y = leg_cache_y[i] + int(lift)  # Apply vertical lift
            local_z = float(leg_cache_z[i])

            # 2D rotation
            rotated_x = local_x * cos_r - local_z * sin_r
            rotated_z = local_x * sin_r + local_z * cos_r

            grid_x = center_x + int(ti.round(rotated_x))
            grid_y = 2 + local_y
            grid_z = center_z + int(ti.round(rotated_z))

            if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid and grid_y >= 0:
                simulation.voxel_type[grid_x, grid_y, grid_z] = leg_color

@ti.kernel
def clear_beetles_bounded(x1: ti.f32, z1: ti.f32, x2: ti.f32, z2: ti.f32):
    """Clear beetles using bounding box - MUCH faster than full grid scan"""
    # Convert to grid coordinates with margin
    margin = 20
    center_x1 = int(x1 + simulation.n_grid / 2.0)
    center_z1 = int(z1 + simulation.n_grid / 2.0)
    center_x2 = int(x2 + simulation.n_grid / 2.0)
    center_z2 = int(z2 + simulation.n_grid / 2.0)

    # Bounding box
    min_x = ti.max(0, ti.min(center_x1, center_x2) - margin)
    max_x = ti.min(simulation.n_grid, ti.max(center_x1, center_x2) + margin)
    min_z = ti.max(0, ti.min(center_z1, center_z2) - margin)
    max_z = ti.min(simulation.n_grid, ti.max(center_z1, center_z2) + margin)

    # Only scan bounding box
    for i in range(min_x, max_x):
        for j in range(0, 15):  # Height range for beetles
            for k in range(min_z, max_z):
                vtype = simulation.voxel_type[i, j, k]
                if vtype == simulation.BEETLE_BLUE or vtype == simulation.BEETLE_RED or \
                   vtype == simulation.BEETLE_BLUE_LEGS or vtype == simulation.BEETLE_RED_LEGS:
                    simulation.voxel_type[i, j, k] = simulation.EMPTY

def beetle_collision(b1, b2, params):
    """Handle collision with voxel-perfect detection and pushing"""
    # Fast GPU-based collision check
    has_collision = check_collision_kernel(b1.x, b1.z, b2.x, b2.z)

    if has_collision:
        # Calculate occupied voxels only when collision detected (for torque)
        b1.occupied_voxels = calculate_occupied_voxels(b1.x, b1.z, b1.rotation)
        b2.occupied_voxels = calculate_occupied_voxels(b2.x, b2.z, b2.rotation)
        overlap_voxels = b1.occupied_voxels & b2.occupied_voxels
        # Collision detected! Use beetle centers for push direction
        dx = b1.x - b2.x
        dz = b1.z - b2.z
        dist = math.sqrt(dx**2 + dz**2)

        if dist > 0.001:
            # Collision normal (from b2 to b1)
            normal_x = dx / dist
            normal_z = dz / dist

            # Calculate collision point (centroid of overlapping voxels)
            if len(overlap_voxels) > 0:
                collision_x = 0.0
                collision_z = 0.0
                for voxel in overlap_voxels:
                    collision_x += voxel[0] - simulation.n_grid / 2.0
                    collision_z += voxel[1] - simulation.n_grid / 2.0
                collision_x /= len(overlap_voxels)
                collision_z /= len(overlap_voxels)
            else:
                collision_x = (b1.x + b2.x) / 2.0
                collision_z = (b1.z + b2.z) / 2.0

            # STRONG separation to prevent stuck collisions
            separation_force = 2.0  # Much stronger push
            b1.x += normal_x * separation_force
            b1.z += normal_z * separation_force
            b2.x -= normal_x * separation_force
            b2.z -= normal_z * separation_force

            # Calculate relative velocity
            rel_vx = b1.vx - b2.vx
            rel_vz = b1.vz - b2.vz
            vel_along_normal = rel_vx * normal_x + rel_vz * normal_z

            # Only apply impulse if moving toward each other
            if vel_along_normal < 0:
                # Impulse magnitude (very low restitution for minimal bouncing)
                impulse = -(1 + params["RESTITUTION"]) * vel_along_normal * params["IMPULSE_MULTIPLIER"]  # Lower momentum transfer = better plowing

                # Apply linear impulse
                impulse_x = impulse * normal_x
                impulse_z = impulse * normal_z
                b1.vx += impulse_x
                b1.vz += impulse_z
                b2.vx -= impulse_x
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
window = ti.ui.Window("Beetle Physics", (1920, 1080), vsync=True)
canvas = window.get_canvas()
scene = window.get_scene()

camera = renderer.Camera()
camera.pos_x = 0.0
camera.pos_y = 60.0
camera.pos_z = 0.0
camera.pitch = -70.0
camera.yaw = 0.0

print("\n=== BEETLE PHYSICS ===")
print("BLUE BEETLE (TFGH) - Tank Controls:")
print("  T - Move Forward")
print("  G - Move Backward")
print("  F - Rotate Left")
print("  H - Rotate Right")
print("")
print("RED BEETLE (IJKL) - Tank Controls:")
print("  I - Move Forward")
print("  K - Move Backward")
print("  J - Rotate Left")
print("  L - Rotate Right")
print("")
print("Push beetles together!")
print("Camera: WASD/Mouse/Q/E")
print("="*40 + "\n")

# Physics parameters dictionary (mutable for sliders)
physics_params = {
    "TORQUE_MULTIPLIER": TORQUE_MULTIPLIER,
    "IMPULSE_MULTIPLIER": IMPULSE_MULTIPLIER,
    "RESTITUTION": RESTITUTION,
    "MOMENT_OF_INERTIA_FACTOR": MOMENT_OF_INERTIA_FACTOR
}

last_time = time.time()

while window.running:
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time
    dt = min(dt, 0.05)

    # Camera
    renderer.handle_camera_controls(camera, window, dt)
    renderer.handle_mouse_look(camera, window)

    # === BLUE BEETLE CONTROLS (TFGH) - TANK STYLE ===
    # Rotation controls (F/H)
    if window.is_pressed('f'):
        # Rotate left (counterclockwise)
        beetle_blue.rotation -= ROTATION_SPEED * dt
    if window.is_pressed('h'):
        # Rotate right (clockwise)
        beetle_blue.rotation += ROTATION_SPEED * dt

    # Movement controls (T/G) - move in facing direction
    if window.is_pressed('t'):
        # Move forward in facing direction
        move_x = math.cos(beetle_blue.rotation)
        move_z = math.sin(beetle_blue.rotation)
        beetle_blue.apply_force(move_x * MOVE_FORCE, move_z * MOVE_FORCE, dt)
    if window.is_pressed('g'):
        # Move backward in facing direction
        move_x = -math.cos(beetle_blue.rotation)
        move_z = -math.sin(beetle_blue.rotation)
        beetle_blue.apply_force(move_x * MOVE_FORCE, move_z * MOVE_FORCE, dt)

    # Always add physics-driven rotation from collisions
    beetle_blue.rotation += beetle_blue.angular_velocity * dt
    beetle_blue.rotation = normalize_angle(beetle_blue.rotation)

    # === RED BEETLE CONTROLS (IJKL) - TANK STYLE ===
    # Rotation controls (J/L)
    if window.is_pressed('j'):
        # Rotate left (counterclockwise)
        beetle_red.rotation -= ROTATION_SPEED * dt
    if window.is_pressed('l'):
        # Rotate right (clockwise)
        beetle_red.rotation += ROTATION_SPEED * dt

    # Movement controls (I/K) - move in facing direction
    if window.is_pressed('i'):
        # Move forward in facing direction
        move_x = math.cos(beetle_red.rotation)
        move_z = math.sin(beetle_red.rotation)
        beetle_red.apply_force(move_x * MOVE_FORCE, move_z * MOVE_FORCE, dt)
    if window.is_pressed('k'):
        # Move backward in facing direction
        move_x = -math.cos(beetle_red.rotation)
        move_z = -math.sin(beetle_red.rotation)
        beetle_red.apply_force(move_x * MOVE_FORCE, move_z * MOVE_FORCE, dt)

    # Always add physics-driven rotation from collisions
    beetle_red.rotation += beetle_red.angular_velocity * dt
    beetle_red.rotation = normalize_angle(beetle_red.rotation)

    # Physics update
    beetle_blue.update_physics(dt)
    beetle_red.update_physics(dt)

    # Update walk animation based on velocity
    blue_speed = math.sqrt(beetle_blue.vx**2 + beetle_blue.vz**2)
    if blue_speed > 0.5:  # Only animate if moving
        beetle_blue.walk_phase += blue_speed * WALK_CYCLE_SPEED * dt
        beetle_blue.walk_phase = beetle_blue.walk_phase % (2 * math.pi)  # Keep in [0, 2π]
        beetle_blue.is_moving = True
    else:
        beetle_blue.is_moving = False

    red_speed = math.sqrt(beetle_red.vx**2 + beetle_red.vz**2)
    if red_speed > 0.5:
        beetle_red.walk_phase += red_speed * WALK_CYCLE_SPEED * dt
        beetle_red.walk_phase = beetle_red.walk_phase % (2 * math.pi)
        beetle_red.is_moving = True
    else:
        beetle_red.is_moving = False

    # Beetle collision (voxel-perfect)
    beetle_collision(beetle_blue, beetle_red, physics_params)

    # Render - ANIMATED with leg walking cycles and different leg colors!
    clear_beetles_bounded(beetle_blue.x, beetle_blue.z, beetle_red.x, beetle_red.z)
    place_animated_beetle(beetle_blue.x, beetle_blue.z, beetle_blue.rotation, simulation.BEETLE_BLUE, simulation.BEETLE_BLUE_LEGS, beetle_blue.walk_phase)
    place_animated_beetle(beetle_red.x, beetle_red.z, beetle_red.rotation, simulation.BEETLE_RED, simulation.BEETLE_RED_LEGS, beetle_red.walk_phase)

    canvas.set_background_color((0.13, 0.35, 0.13))  # Forest green
    renderer.render(camera, canvas, scene, simulation.voxel_type, simulation.n_grid)
    canvas.scene(scene)

    # HUD
    fps = 1.0 / dt if dt > 0 else 0
    window.GUI.begin("Beetle Physics", 0.01, 0.01, 0.35, 0.52)
    window.GUI.text(f"FPS: {fps:.0f}")
    window.GUI.text("")
    window.GUI.text("BLUE BEETLE (TFGH)")
    window.GUI.text(f"  Pos: ({beetle_blue.x:.1f}, {beetle_blue.z:.1f})")
    window.GUI.text(f"  Speed: {math.sqrt(beetle_blue.vx**2 + beetle_blue.vz**2):.1f}")
    window.GUI.text(f"  Facing: {math.degrees(beetle_blue.rotation):.0f}°")
    window.GUI.text("")
    window.GUI.text("RED BEETLE (IJKL)")
    window.GUI.text(f"  Pos: ({beetle_red.x:.1f}, {beetle_red.z:.1f})")
    window.GUI.text(f"  Speed: {math.sqrt(beetle_red.vx**2 + beetle_red.vz**2):.1f}")
    window.GUI.text(f"  Facing: {math.degrees(beetle_red.rotation):.0f}°")
    window.GUI.text("")
    dx = beetle_blue.x - beetle_red.x
    dz = beetle_blue.z - beetle_red.z
    distance = math.sqrt(dx**2 + dz**2)
    window.GUI.text(f"Distance: {distance:.1f}")

    # Physics parameter sliders
    window.GUI.text("")
    window.GUI.text("=== PHYSICS TUNING ===")
    physics_params["TORQUE_MULTIPLIER"] = window.GUI.slider_float("Torque", physics_params["TORQUE_MULTIPLIER"], 0.0, 1.5)
    physics_params["IMPULSE_MULTIPLIER"] = window.GUI.slider_float("Impulse", physics_params["IMPULSE_MULTIPLIER"], 0.0, 1.0)
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
