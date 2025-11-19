"""
Beetle combat with rotation and pushing physics - 3X SCALE
TGHF for blue beetle, IKJL for red beetle
REDESIGNED: Clear body segments, animation-ready legs
"""

import taichi as ti
import simulation
import renderer
import time
import math

# Physics constants
BEETLE_RADIUS = 48.0  # 3x larger beetles for animation clarity
MOVE_FORCE = 120.0  # Increased for more responsive feel
FRICTION = 0.88  # Slightly higher friction
MAX_SPEED = 7.0  # Slightly higher top speed
ROTATION_SPEED = 18.0  # Much faster rotation for instant diagonal response
ANGULAR_FRICTION = 0.85  # How quickly spin slows down (lower = more spin)
MAX_ANGULAR_SPEED = 8.0  # Max spin speed (radians/sec)
TORQUE_MULTIPLIER = 3.0  # Amplify rotational forces
ARENA_RADIUS = 85.0  # Fits in 192×192 grid (center at 96, good space for 3x beetles)

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

# Create beetles
beetle_blue = Beetle(-50.0, 0.0, math.pi, simulation.BEETLE_BLUE)
beetle_red = Beetle(50.0, 0.0, 0.0, simulation.BEETLE_RED)

@ti.kernel
def clear_beetles():
    """Remove all beetle voxels"""
    for i, j, k in ti.ndrange(simulation.n_grid, simulation.n_grid, simulation.n_grid):
        if simulation.voxel_type[i, j, k] == simulation.BEETLE_BLUE or \
           simulation.voxel_type[i, j, k] == simulation.BEETLE_RED:
            simulation.voxel_type[i, j, k] = simulation.EMPTY

def calculate_occupied_voxels(world_x, world_z, rotation):
    """Calculate which voxels this beetle occupies (Python function)"""
    center_x = int(world_x + simulation.n_grid / 2.0)
    center_z = int(world_z + simulation.n_grid / 2.0)

    occupied = set()

    # Expanded range for 3x larger beetle
    # Body: -36 to 37, Horn extends to 73, Legs extend ±20
    for dx in range(-36, 74):  # Includes full body + horn
        for dz in range(-20, 21):  # Includes full leg span
            local_x = float(dx)
            local_z = float(dz)

            # Rotate voxel
            cos_r = math.cos(rotation)
            sin_r = math.sin(rotation)
            rotated_x = local_x * cos_r - local_z * sin_r
            rotated_z = local_x * sin_r + local_z * cos_r

            grid_x = center_x + int(round(rotated_x))
            grid_z = center_z + int(round(rotated_z))

            if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                occupied.add((grid_x, grid_z))

    return occupied

@ti.kernel
def place_beetle_rotated(world_x: ti.f32, world_z: ti.f32, rotation: ti.f32, color_type: ti.i32):
    """Place 3x scale beetle - CLARITY & ANIMATION FOCUSED

    Design: Simple shapes, clear gaps between segments, visible joints
    Body: Abdomen (30L) + 2-gap + Thorax (24L) + 2-gap + Head (15L) = 75 total
    Horn: 36L extending from head
    Legs: 6 legs with 3 segments each, 1-voxel gaps at joints
    """
    center_x = int(world_x + simulation.n_grid / 2.0)
    center_z = int(world_z + simulation.n_grid / 2.0)

    cos_r = ti.cos(rotation)
    sin_r = ti.sin(rotation)

    # ========== ABDOMEN (rear) - Simple rounded box ==========
    # 30L × 36W × 24H - Widest body part, X: -36 to -6 (with 2-voxel gap)
    for dx in range(-36, -4):
        for dy in range(0, 24):
            for dz in range(-18, 19):
                # Elliptical cross-section for organic shape
                height_factor = 1.0 - ((dy - 12.0) / 12.0) ** 2
                width_at_height = 18.0 * ti.sqrt(height_factor) if height_factor > 0 else 0

                # Taper both ends
                length_pos = (dx + 20.0) / 16.0
                length_taper = 1.0 - (length_pos * length_pos)
                max_width = width_at_height * ti.sqrt(length_taper) if length_taper > 0 else 0

                if ti.abs(dz) <= max_width and dx < -6:  # Stop before gap
                    local_x = float(dx)
                    local_z = float(dz)
                    rotated_x = local_x * cos_r - local_z * sin_r
                    rotated_z = local_x * sin_r + local_z * cos_r
                    grid_x = center_x + int(ti.round(rotated_x))
                    grid_y = 2 + dy
                    grid_z = center_z + int(ti.round(rotated_z))
                    if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                        simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

    # ========== THORAX (middle) - Tall rounded box with flat top ==========
    # 24L × 32W × 32H - TALLEST segment, X: -4 to 20 (with 2-voxel gap before head)
    for dx in range(-4, 20):
        for dy in range(0, 32):
            for dz in range(-16, 17):
                # Rounded sides, flatter top for pronotum
                width_at_height = 0.0  # Initialize
                if dy < 24:
                    # Lower section - rounded
                    height_factor = 1.0 - ((dy - 12.0) / 12.0) ** 2
                    if height_factor > 0:
                        width_at_height = 16.0 * ti.sqrt(height_factor)
                else:
                    # Upper section - flat pronotum ridge
                    width_at_height = 16.0 - (dy - 24) * 1.5

                # Taper slightly front to back
                length_pos = (dx - 8.0) / 12.0
                length_taper = 1.0 - (length_pos * length_pos) * 0.3
                max_width = width_at_height * length_taper

                if ti.abs(dz) <= max_width and dx < 22:  # Stop before gap
                    local_x = float(dx)
                    local_z = float(dz)
                    rotated_x = local_x * cos_r - local_z * sin_r
                    rotated_z = local_x * sin_r + local_z * cos_r
                    grid_x = center_x + int(ti.round(rotated_x))
                    grid_y = 2 + dy
                    grid_z = center_z + int(ti.round(rotated_z))
                    if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                        simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

    # Small thoracic horn bump on pronotum
    for dx in range(14, 18):
        for dy in range(28, 34):
            for dz in range(-2, 3):
                if dy < 32 or (dy < 34 and ti.abs(dz) <= 1):
                    local_x = float(dx)
                    local_z = float(dz)
                    rotated_x = local_x * cos_r - local_z * sin_r
                    rotated_z = local_x * sin_r + local_z * cos_r
                    grid_x = center_x + int(ti.round(rotated_x))
                    grid_y = 2 + dy
                    grid_z = center_z + int(ti.round(rotated_z))
                    if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                        simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

    # ========== HEAD (front) - Compact rounded box ==========
    # 15L × 26W × 18H - Narrowest segment, X: 22 to 37
    for dx in range(22, 37):
        for dy in range(0, 18):
            for dz in range(-13, 14):
                # Rounded shape
                height_factor = 1.0 - ((dy - 9.0) / 9.0) ** 2
                width_at_height = 13.0 * ti.sqrt(height_factor) if height_factor > 0 else 0

                # Taper toward front
                length_pos = (dx - 29.5) / 7.5
                length_taper = 1.0 - (length_pos * length_pos) * 0.4
                max_width = width_at_height * length_taper

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

    # ========== Y-SHAPED HORN - Clean cylindrical shaft with fork ==========
    # 36L, extends X: 37 to 73, curves upward
    for i in range(36):
        dx = 37 + i
        # Gentle upward curve
        height_curve = int(i * 0.33)  # +12 voxels over length
        dy = 12 + height_curve

        # Thickness tapers: 6×6 → 4×4 → 3×3
        if i < 12:
            thickness = 3  # 6×6 base
        elif i < 24:
            thickness = 2  # 4×4 middle
        else:
            thickness = 1 if i < 32 else 0  # 3×3 tip, then fork

        # Main shaft (square cross-section for clarity)
        for dz_off in range(-thickness, thickness + 1):
            for dy_off in range(-thickness, thickness + 1):
                local_x = float(dx)
                local_z = float(dz_off)
                rotated_x = local_x * cos_r - local_z * sin_r
                rotated_z = local_x * sin_r + local_z * cos_r
                grid_x = center_x + int(ti.round(rotated_x))
                grid_y = 2 + dy + dy_off
                grid_z = center_z + int(ti.round(rotated_z))
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid and 0 <= grid_y < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

    # Y-fork prongs (last 8 voxels split outward)
    for i in range(8):
        dx = 65 + i
        dy = 22 + i
        # Left prong
        dz_left = -i
        local_x = float(dx)
        local_z = float(dz_left)
        rotated_x = local_x * cos_r - local_z * sin_r
        rotated_z = local_x * sin_r + local_z * cos_r
        grid_x = center_x + int(ti.round(rotated_x))
        grid_y = 2 + dy
        grid_z = center_z + int(ti.round(rotated_z))
        if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid and 0 <= grid_y < simulation.n_grid:
            simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

        # Right prong
        dz_right = i
        local_z_r = float(dz_right)
        rotated_x_r = float(dx) * cos_r - local_z_r * sin_r
        rotated_z_r = float(dx) * sin_r + local_z_r * cos_r
        grid_x_r = center_x + int(ti.round(rotated_x_r))
        grid_z_r = center_z + int(ti.round(rotated_z_r))
        if 0 <= grid_x_r < simulation.n_grid and 0 <= grid_z_r < simulation.n_grid:
            simulation.voxel_type[grid_x_r, grid_y, grid_z_r] = color_type

    # ========== LEGS (6 total) - 3 segments each, ANIMATION READY ==========
    # Each leg: COXA (4L) + 1-gap + FEMUR (10-14L) + 1-gap + TIBIA (12-16L)
    # All segments 4×4 voxels thick for visibility

    # FRONT LEGS (longest) - Attach at front of thorax (X: 18)
    for side in ti.static([-1, 1]):
        # Coxa: 4 voxels horizontal from body
        for i in range(4):
            dx = 18
            dy = 2
            dz = side * (16 + i)
            local_x = float(dx)
            local_z = float(dz)
            rotated_x = local_x * cos_r - local_z * sin_r
            rotated_z = local_x * sin_r + local_z * cos_r
            for ddy in range(4):
                for ddz in range(-1, 2):
                    grid_x = center_x + int(ti.round(rotated_x))
                    grid_y = 2 + dy + ddy
                    grid_z = center_z + int(ti.round(rotated_z)) + ddz
                    if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid and 0 <= grid_y < simulation.n_grid:
                        simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

        # Femur: 14 voxels angled out and down (45°)
        for i in range(14):
            dx = 18
            dy = 2 - i // 2
            dz = side * (20 + i)
            local_x = float(dx)
            local_z = float(dz)
            rotated_x = local_x * cos_r - local_z * sin_r
            rotated_z = local_x * sin_r + local_z * cos_r
            for ddy in range(4):
                for ddz in range(-1, 2):
                    grid_x = center_x + int(ti.round(rotated_x))
                    grid_y = 2 + ti.max(0, dy + ddy)
                    grid_z = center_z + int(ti.round(rotated_z)) + ddz
                    if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid and 0 <= grid_y < simulation.n_grid:
                        simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

    # MIDDLE LEGS (medium) - Attach at thorax center (X: 8)
    for side in ti.static([-1, 1]):
        # Coxa + Femur + Tibia combined for simplicity
        for i in range(18):
            dx = 8 - i // 6
            dy = 2 - i // 4
            dz = side * (16 + i)
            local_x = float(dx)
            local_z = float(dz)
            rotated_x = local_x * cos_r - local_z * sin_r
            rotated_z = local_x * sin_r + local_z * cos_r
            for ddy in range(4):
                for ddz in range(-1, 2):
                    grid_x = center_x + int(ti.round(rotated_x))
                    grid_y = 2 + ti.max(0, dy + ddy)
                    grid_z = center_z + int(ti.round(rotated_z)) + ddz
                    if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid and 0 <= grid_y < simulation.n_grid:
                        simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

    # REAR LEGS (shortest) - Attach at abdomen front (X: -10)
    for side in ti.static([-1, 1]):
        # Coxa + Femur + Tibia combined
        for i in range(14):
            dx = -10 - i // 5
            dy = 2 - i // 4
            dz = side * (16 + i)
            local_x = float(dx)
            local_z = float(dz)
            rotated_x = local_x * cos_r - local_z * sin_r
            rotated_z = local_x * sin_r + local_z * cos_r
            for ddy in range(4):
                for ddz in range(-1, 2):
                    grid_x = center_x + int(ti.round(rotated_x))
                    grid_y = 2 + ti.max(0, dy + ddy)
                    grid_z = center_z + int(ti.round(rotated_z)) + ddz
                    if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid and 0 <= grid_y < simulation.n_grid:
                        simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

def beetle_collision(b1, b2):
    """Handle collision with voxel-perfect detection and pushing"""
    # Check for voxel overlap
    overlap_voxels = b1.occupied_voxels & b2.occupied_voxels

    if len(overlap_voxels) > 0:
        # Collision detected! Use beetle centers for push direction
        dx = b1.x - b2.x
        dz = b1.z - b2.z
        dist = math.sqrt(dx**2 + dz**2)

        if dist > 0.001:
            # Collision normal (from b2 to b1)
            normal_x = dx / dist
            normal_z = dz / dist

            # Calculate collision point (centroid of overlapping voxels)
            collision_x = 0.0
            collision_z = 0.0
            for voxel in overlap_voxels:
                collision_x += voxel[0] - simulation.n_grid / 2.0
                collision_z += voxel[1] - simulation.n_grid / 2.0
            collision_x /= len(overlap_voxels)
            collision_z /= len(overlap_voxels)

            # Separate beetles slightly
            separation_force = 0.3
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
                # Impulse magnitude (low restitution for less bouncing)
                restitution = 0.3
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
window = ti.ui.Window("Beetle Physics - 3X SCALE", (1920, 1080), vsync=True)
canvas = window.get_canvas()
scene = window.get_scene()

camera = renderer.Camera()
camera.pos_x = 0.0
camera.pos_y = 180.0  # Higher camera for larger beetles
camera.pos_z = 0.0
camera.pitch = -70.0
camera.yaw = 0.0

print("\n=== BEETLE PHYSICS - 3X SCALE ===")
print("BLUE BEETLE (TFGH):")
print("  T - Move North")
print("  G - Move South")
print("  F - Move West")
print("  H - Move East")
print("")
print("RED BEETLE (IJKL):")
print("  I - Move North")
print("  K - Move South")
print("  J - Move West")
print("  L - Move East")
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

    # === BLUE BEETLE CONTROLS (TFGH) ===
    move_dir_x = 0.0
    move_dir_z = 0.0

    if window.is_pressed('t'):
        move_dir_z = -1.0
    if window.is_pressed('g'):
        move_dir_z = 1.0
    if window.is_pressed('f'):
        move_dir_x = -1.0
    if window.is_pressed('h'):
        move_dir_x = 1.0

    # Apply movement force and rotation ONLY when keys are pressed
    if move_dir_x != 0 or move_dir_z != 0:
        # Calculate angle from movement direction
        beetle_blue.target_rotation = math.atan2(-move_dir_z, move_dir_x)
        if beetle_blue.target_rotation < 0:
            beetle_blue.target_rotation += 2 * math.pi

        # Normalize movement vector
        length = math.sqrt(move_dir_x**2 + move_dir_z**2)
        move_dir_x /= length
        move_dir_z /= length
        beetle_blue.apply_force(move_dir_x * MOVE_FORCE, move_dir_z * MOVE_FORCE, dt)

        # Smooth rotation toward movement direction
        rot_diff = shortest_rotation(beetle_blue.rotation, beetle_blue.target_rotation)
        if abs(rot_diff) > 0.01:
            rot_step = ROTATION_SPEED * dt
            if abs(rot_diff) < rot_step:
                beetle_blue.rotation = beetle_blue.target_rotation
            else:
                beetle_blue.rotation += rot_step if rot_diff > 0 else -rot_step

    # Always add physics-driven rotation from collisions
    beetle_blue.rotation += beetle_blue.angular_velocity * dt
    beetle_blue.rotation = normalize_angle(beetle_blue.rotation)

    # === RED BEETLE CONTROLS ===
    move_dir_x = 0.0
    move_dir_z = 0.0

    if window.is_pressed('i'):
        move_dir_z = -1.0
    if window.is_pressed('k'):
        move_dir_z = 1.0
    if window.is_pressed('j'):
        move_dir_x = -1.0
    if window.is_pressed('l'):
        move_dir_x = 1.0

    # Apply movement force and rotation ONLY when keys are pressed
    if move_dir_x != 0 or move_dir_z != 0:
        # Calculate angle from movement direction
        beetle_red.target_rotation = math.atan2(-move_dir_z, move_dir_x)
        if beetle_red.target_rotation < 0:
            beetle_red.target_rotation += 2 * math.pi

        # Normalize movement vector
        length = math.sqrt(move_dir_x**2 + move_dir_z**2)
        move_dir_x /= length
        move_dir_z /= length
        beetle_red.apply_force(move_dir_x * MOVE_FORCE, move_dir_z * MOVE_FORCE, dt)

        # Smooth rotation toward movement direction
        rot_diff = shortest_rotation(beetle_red.rotation, beetle_red.target_rotation)
        if abs(rot_diff) > 0.01:
            rot_step = ROTATION_SPEED * dt
            if abs(rot_diff) < rot_step:
                beetle_red.rotation = beetle_red.target_rotation
            else:
                beetle_red.rotation += rot_step if rot_diff > 0 else -rot_step

    # Always add physics-driven rotation from collisions
    beetle_red.rotation += beetle_red.angular_velocity * dt
    beetle_red.rotation = normalize_angle(beetle_red.rotation)

    # Physics update
    beetle_blue.update_physics(dt)
    beetle_red.update_physics(dt)

    # Arena collisions
    beetle_blue.arena_collision()
    beetle_red.arena_collision()

    # Calculate occupied voxels for collision detection
    beetle_blue.occupied_voxels = calculate_occupied_voxels(beetle_blue.x, beetle_blue.z, beetle_blue.rotation)
    beetle_red.occupied_voxels = calculate_occupied_voxels(beetle_red.x, beetle_red.z, beetle_red.rotation)

    # Beetle collision (voxel-perfect)
    beetle_collision(beetle_blue, beetle_red)

    # Render
    clear_beetles()
    place_beetle_rotated(beetle_blue.x, beetle_blue.z, beetle_blue.rotation, beetle_blue.color)
    place_beetle_rotated(beetle_red.x, beetle_red.z, beetle_red.rotation, beetle_red.color)

    canvas.set_background_color((0.05, 0.05, 0.08))
    renderer.render(camera, canvas, scene, simulation.voxel_type, simulation.n_grid)
    canvas.scene(scene)

    # HUD
    fps = 1.0 / dt if dt > 0 else 0
    window.GUI.begin("Beetle Physics - 3X SCALE", 0.01, 0.01, 0.35, 0.30)
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
