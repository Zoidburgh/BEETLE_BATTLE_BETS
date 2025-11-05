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
TORQUE_MULTIPLIER = 1.0  # Gentler rotational forces (pushy not bouncy)
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
        self.moment_of_inertia = BEETLE_RADIUS * 0.5  # Resistance to rotation (lower = easier to spin)
        self.color = color
        self.radius = BEETLE_RADIUS
        self.occupied_voxels = set()  # Track voxel positions for accurate collision

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
def check_collision_kernel(x1: ti.f32, z1: ti.f32, x2: ti.f32, z2: ti.f32,
                           result: ti.template()) -> ti.i32:
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

            # Find Y-ranges for both beetles in this column
            for gy in range(0, 15):
                voxel = simulation.voxel_type[gx, gy, gz]
                if voxel == simulation.BEETLE_BLUE:
                    if gy < beetle1_y_min:
                        beetle1_y_min = gy
                    if gy > beetle1_y_max:
                        beetle1_y_max = gy
                elif voxel == simulation.BEETLE_RED:
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
    """Generate beetle geometry once as list of (dx, dy, dz) offsets"""
    voxels = []

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
                            voxels.append((dx, dy, dz))

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
                        voxels.append((dx, 5, dz))

    # THORAX (middle) - LEAN with small pronotum bump
    for dx in range(-1, 2):
        for dy in range(0, 2):
            for dz in range(-2, 3):
                width_at_height = 2.0 if dy < 1 else 1.5
                length_factor = 1.0 - abs(dx) / 1.5
                max_width = width_at_height * length_factor
                if abs(dz) <= max_width:
                    voxels.append((dx, dy, dz))

    # Pyramid top layer for thorax - inset by 1 voxel
    for dx in range(-1, 2):
        for dz in range(-1, 2):
            if abs(dz) > 0:  # Not on the centerline, inset from edges
                voxels.append((dx, 2, dz))

    # HEAD (front) - LEAN
    for dx in range(2, 4):
        for dy in range(0, 1):
            for dz in range(-1, 2):
                width_at_height = 1.5
                if abs(dz) <= width_at_height:
                    voxels.append((dx, dy, dz))

    # EXAGGERATED Y-SHAPED HORN - Main shaft with overlapping layers
    for i in range(12):
        dx = 3 + i
        height_curve = int(i * 0.3)
        dy = 1 + height_curve
        thickness = 1  # Changed from 2 to 1 for 3-voxel width
        for dz in range(-thickness, thickness + 1):
            # Add voxel at current height
            voxels.append((dx, dy, dz))
            # Add overlapping voxel below (unless at bottom)
            if dy > 1:
                voxels.append((dx, dy - 1, dz))

    # EXAGGERATED Y-fork - Left prong with overlapping layers
    for i in range(5):
        dx = 13 + i
        dy = 4 + i
        center_dz = -i
        for dz_offset in range(0, 2):  # Changed from range(-1, 2) to range(0, 2) for 2-voxel width
            dz = center_dz + dz_offset
            # Add voxel at current height
            voxels.append((dx, dy, dz))
            # Add overlapping voxel below
            voxels.append((dx, dy - 1, dz))

    # Right prong with overlapping layers
    for i in range(5):
        dx = 13 + i
        dy = 4 + i
        center_dz = i
        for dz_offset in range(-1, 1):  # Changed from range(-1, 2) to range(-1, 1) for 2-voxel width
            dz = center_dz + dz_offset
            # Add voxel at current height
            voxels.append((dx, dy, dz))
            # Add overlapping voxel below
            voxels.append((dx, dy - 1, dz))

    # IMPROVED LEGS (6 total) - Front legs with smooth curves
    for side in [-1, 1]:
        # COXA - Wider starting position
        for i in range(2):
            dx = 1
            dy = 1
            dz = side * (3 + i)  # Start at dz=3 instead of 2
            for extra_y in range(2):
                voxels.append((dx, dy + extra_y, dz))

        # FEMUR - Extended and smooth
        for i in range(4):  # Extended from 3 to 4
            dx = 1
            dy = 0
            dz = side * (5 + i)  # Shifted to accommodate wider start
            for extra_y in range(2):
                voxels.append((dx, dy + extra_y, dz))

        # TIBIA - Smooth curve with no gaps, 25% longer
        # Create smooth curve from (1, 0, 9) to (3, 0, 10)
        tibia_points = [
            (1, 0, side * 9),
            (1, 0, side * 10),   # Fill gap
            (2, 0, side * 10),   # Fill gap
            (3, 0, side * 10),
        ]
        for point in tibia_points:
            voxels.append(point)

    # Middle legs - Improved with smooth curves
    for side in [-1, 1]:
        # COXA - Wider starting position (moved back 1 voxel)
        for i in range(2):
            dx = -2
            dy = 1
            dz = side * (3 + i)
            for extra_y in range(2):
                voxels.append((dx, dy + extra_y, dz))

        # FEMUR - Extended and smooth
        for i in range(4):
            dx = -2
            dy = 0
            dz = side * (5 + i)
            for extra_y in range(2):
                voxels.append((dx, dy + extra_y, dz))

        # TIBIA - Smooth curve extending backward
        tibia_points = [
            (-2, 0, side * 9),
            (-2, 0, side * 10),  # Fill gap
            (-3, 0, side * 10),  # Fill gap
            (-4, 0, side * 10),
        ]
        for point in tibia_points:
            voxels.append(point)

    # Rear legs - Improved with smooth curves
    for side in [-1, 1]:
        # COXA - Wider starting position (moved back 1 voxel)
        for i in range(2):
            dx = -5
            dy = 1
            dz = side * (3 + i)
            for extra_y in range(2):
                voxels.append((dx, dy + extra_y, dz))

        # FEMUR - Extended both backward and outward
        for i in range(4):
            dx = -5 - i
            dy = 0
            dz = side * (5 + i)
            for extra_y in range(2):
                voxels.append((dx, dy + extra_y, dz))

        # TIBIA - Smooth curve extending backward
        tibia_points = [
            (-9, 0, side * 9),
            (-9, 0, side * 10),   # Fill gap
            (-10, 0, side * 10),  # Fill gap
            (-11, 0, side * 10),
        ]
        for point in tibia_points:
            voxels.append(point)

    return voxels

# Generate beetle geometry ONCE at startup
BEETLE_GEOMETRY = generate_beetle_geometry()
print(f"Beetle geometry cached: {len(BEETLE_GEOMETRY)} voxels")

# Create Taichi fields for geometry cache
beetle_cache_size = len(BEETLE_GEOMETRY)
beetle_cache_x = ti.field(ti.i32, shape=beetle_cache_size)
beetle_cache_y = ti.field(ti.i32, shape=beetle_cache_size)
beetle_cache_z = ti.field(ti.i32, shape=beetle_cache_size)

# Populate Taichi fields with cached geometry
@ti.kernel
def init_beetle_cache():
    """Initialize beetle geometry cache on GPU"""
    pass

# Copy geometry to GPU
for i, (dx, dy, dz) in enumerate(BEETLE_GEOMETRY):
    beetle_cache_x[i] = dx
    beetle_cache_y[i] = dy
    beetle_cache_z[i] = dz

init_beetle_cache()

@ti.kernel
def place_cached_beetle(world_x: ti.f32, world_z: ti.f32, rotation: ti.f32, color_type: ti.i32):
    """Fast beetle placement using pre-generated geometry cache"""
    center_x = int(world_x + simulation.n_grid / 2.0)
    center_z = int(world_z + simulation.n_grid / 2.0)

    cos_r = ti.cos(rotation)
    sin_r = ti.sin(rotation)

    # Transform cached voxels with rotation
    for i in range(beetle_cache_size):
        local_x = float(beetle_cache_x[i])
        local_y = beetle_cache_y[i]
        local_z = float(beetle_cache_z[i])

        # 2D rotation
        rotated_x = local_x * cos_r - local_z * sin_r
        rotated_z = local_x * sin_r + local_z * cos_r

        grid_x = center_x + int(ti.round(rotated_x))
        grid_y = 2 + local_y
        grid_z = center_z + int(ti.round(rotated_z))

        if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
            simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

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
                if simulation.voxel_type[i, j, k] == simulation.BEETLE_BLUE or \
                   simulation.voxel_type[i, j, k] == simulation.BEETLE_RED:
                    simulation.voxel_type[i, j, k] = simulation.EMPTY

def beetle_collision(b1, b2):
    """Handle collision with voxel-perfect detection and pushing"""
    # Fast GPU-based collision check
    collision_result = ti.field(ti.i32, shape=())
    has_collision = check_collision_kernel(b1.x, b1.z, b2.x, b2.z, collision_result)

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
                restitution = 0.05
                impulse = -(1 + restitution) * vel_along_normal * 0.5

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
                angular_impulse1 = (torque1 / b1.moment_of_inertia) * TORQUE_MULTIPLIER
                b1.angular_velocity += angular_impulse1

                # For beetle 2: collision point relative to its center
                r2_x = collision_x - b2.x
                r2_z = collision_z - b2.z
                torque2 = r2_x * (-impulse_z) - r2_z * (-impulse_x)
                angular_impulse2 = (torque2 / b2.moment_of_inertia) * TORQUE_MULTIPLIER
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

    # Beetle collision (voxel-perfect)
    beetle_collision(beetle_blue, beetle_red)

    # Render - OPTIMIZED with geometry caching and bounded clearing
    clear_beetles_bounded(beetle_blue.x, beetle_blue.z, beetle_red.x, beetle_red.z)
    place_cached_beetle(beetle_blue.x, beetle_blue.z, beetle_blue.rotation, beetle_blue.color)
    place_cached_beetle(beetle_red.x, beetle_red.z, beetle_red.rotation, beetle_red.color)

    canvas.set_background_color((0.05, 0.05, 0.08))
    renderer.render(camera, canvas, scene, simulation.voxel_type, simulation.n_grid)
    canvas.scene(scene)

    # HUD
    fps = 1.0 / dt if dt > 0 else 0
    window.GUI.begin("Beetle Physics", 0.01, 0.01, 0.35, 0.30)
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
    window.GUI.end()

    window.show()

print("Done!")
