"""
Beetle controls with rotation - beetles rotate to face movement direction
TGHF for blue beetle, IKJL for red beetle
"""

import taichi as ti
import simulation
import renderer
import time
import math

# Beetle state
class Beetle:
    def __init__(self, x, z, rotation, color):
        self.x = x
        self.z = z
        self.rotation = rotation  # Radians (0 = East, π/2 = North, π = West, 3π/2 = South)
        self.color = color
        self.target_rotation = rotation

# Create beetles
beetle_blue = Beetle(-15.0, 0.0, math.pi, simulation.BEETLE_BLUE)  # Start facing West
beetle_red = Beetle(15.0, 0.0, 0.0, simulation.BEETLE_RED)  # Start facing East

# Movement speed
MOVE_SPEED = 10.0
ROTATION_SPEED = 8.0  # Radians per second

@ti.kernel
def clear_beetles():
    """Remove all beetle voxels"""
    for i, j, k in ti.ndrange(simulation.n_grid, simulation.n_grid, simulation.n_grid):
        if simulation.voxel_type[i, j, k] == simulation.BEETLE_BLUE or \
           simulation.voxel_type[i, j, k] == simulation.BEETLE_RED:
            simulation.voxel_type[i, j, k] = simulation.EMPTY

@ti.kernel
def place_beetle_rotated(world_x: ti.f32, world_z: ti.f32, rotation: ti.f32, color_type: ti.i32):
    """Place beetle at world position with rotation"""
    center_x = int(world_x + simulation.n_grid / 2.0)
    center_z = int(world_z + simulation.n_grid / 2.0)

    # Beetle shape (elongated along x-axis before rotation)
    # This is a simple rectangular beetle
    for dx in range(-3, 4):  # Length (7 voxels)
        for dy in range(0, 3):  # Height (3 voxels)
            for dz in range(-2, 3):  # Width (5 voxels)
                # Rotate the voxel position around origin
                local_x = float(dx)
                local_z = float(dz)

                # 2D rotation matrix
                cos_r = ti.cos(rotation)
                sin_r = ti.sin(rotation)
                rotated_x = local_x * cos_r - local_z * sin_r
                rotated_z = local_x * sin_r + local_z * cos_r

                # Convert to grid coordinates
                grid_x = center_x + int(ti.round(rotated_x))
                grid_y = 2 + dy
                grid_z = center_z + int(ti.round(rotated_z))

                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

def check_collision(x1, z1, x2, z2):
    """Check if two beetles would collide"""
    dx = x1 - x2
    dz = z1 - z2
    distance = math.sqrt(dx*dx + dz*dz)
    return distance < 8.0

def normalize_angle(angle):
    """Normalize angle to [0, 2π)"""
    while angle < 0:
        angle += 2 * math.pi
    while angle >= 2 * math.pi:
        angle -= 2 * math.pi
    return angle

def shortest_rotation(current, target):
    """Find shortest rotation direction from current to target angle"""
    current = normalize_angle(current)
    target = normalize_angle(target)

    diff = target - current

    # Wrap to [-π, π]
    if diff > math.pi:
        diff -= 2 * math.pi
    elif diff < -math.pi:
        diff += 2 * math.pi

    return diff

# Window
window = ti.ui.Window("Beetle Rotation", (1920, 1080), vsync=True)
canvas = window.get_canvas()
scene = window.get_scene()

camera = renderer.Camera()
camera.pos_x = 0.0
camera.pos_y = 60.0
camera.pos_z = 0.0
camera.pitch = -70.0
camera.yaw = 0.0

print("\n=== BEETLE ROTATION ===")
print("BLUE BEETLE (TGHF):")
print("  T - Move North")
print("  G - Move South")
print("  H - Move West")
print("  F - Move East")
print("")
print("RED BEETLE (IKJL):")
print("  I - Move North")
print("  K - Move South")
print("  J - Move West")
print("  L - Move East")
print("")
print("Beetles rotate to face movement direction!")
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

    # === BLUE BEETLE CONTROLS (TGHF) ===
    move_dir_x = 0.0
    move_dir_z = 0.0

    if window.is_pressed('t'):
        move_dir_z = -1.0
        beetle_blue.target_rotation = math.pi / 2  # North
    if window.is_pressed('g'):
        move_dir_z = 1.0
        beetle_blue.target_rotation = 3 * math.pi / 2  # South
    if window.is_pressed('h'):
        move_dir_x = -1.0
        beetle_blue.target_rotation = math.pi  # West
    if window.is_pressed('f'):
        move_dir_x = 1.0
        beetle_blue.target_rotation = 0.0  # East

    # Handle diagonal movement
    if move_dir_x != 0 and move_dir_z != 0:
        if window.is_pressed('t') and window.is_pressed('f'):
            beetle_blue.target_rotation = math.pi / 4  # NE
        elif window.is_pressed('t') and window.is_pressed('h'):
            beetle_blue.target_rotation = 3 * math.pi / 4  # NW
        elif window.is_pressed('g') and window.is_pressed('f'):
            beetle_blue.target_rotation = 7 * math.pi / 4  # SE
        elif window.is_pressed('g') and window.is_pressed('h'):
            beetle_blue.target_rotation = 5 * math.pi / 4  # SW

    # Smooth rotation
    rot_diff = shortest_rotation(beetle_blue.rotation, beetle_blue.target_rotation)
    if abs(rot_diff) > 0.01:
        rot_step = ROTATION_SPEED * dt
        if abs(rot_diff) < rot_step:
            beetle_blue.rotation = beetle_blue.target_rotation
        else:
            beetle_blue.rotation += rot_step if rot_diff > 0 else -rot_step

    beetle_blue.rotation = normalize_angle(beetle_blue.rotation)

    # Move blue beetle
    if move_dir_x != 0 or move_dir_z != 0:
        length = math.sqrt(move_dir_x**2 + move_dir_z**2)
        move_dir_x /= length
        move_dir_z /= length

        new_x = beetle_blue.x + move_dir_x * MOVE_SPEED * dt
        new_z = beetle_blue.z + move_dir_z * MOVE_SPEED * dt

        if abs(new_x) < 35.0 and abs(new_z) < 35.0:
            if not check_collision(new_x, new_z, beetle_red.x, beetle_red.z):
                beetle_blue.x = new_x
                beetle_blue.z = new_z

    # === RED BEETLE CONTROLS (IKJL) ===
    move_dir_x = 0.0
    move_dir_z = 0.0

    if window.is_pressed('i'):
        move_dir_z = -1.0
        beetle_red.target_rotation = math.pi / 2  # North
    if window.is_pressed('k'):
        move_dir_z = 1.0
        beetle_red.target_rotation = 3 * math.pi / 2  # South
    if window.is_pressed('j'):
        move_dir_x = -1.0
        beetle_red.target_rotation = math.pi  # West
    if window.is_pressed('l'):
        move_dir_x = 1.0
        beetle_red.target_rotation = 0.0  # East

    # Handle diagonal movement
    if move_dir_x != 0 and move_dir_z != 0:
        if window.is_pressed('i') and window.is_pressed('l'):
            beetle_red.target_rotation = math.pi / 4  # NE
        elif window.is_pressed('i') and window.is_pressed('j'):
            beetle_red.target_rotation = 3 * math.pi / 4  # NW
        elif window.is_pressed('k') and window.is_pressed('l'):
            beetle_red.target_rotation = 7 * math.pi / 4  # SE
        elif window.is_pressed('k') and window.is_pressed('j'):
            beetle_red.target_rotation = 5 * math.pi / 4  # SW

    # Smooth rotation
    rot_diff = shortest_rotation(beetle_red.rotation, beetle_red.target_rotation)
    if abs(rot_diff) > 0.01:
        rot_step = ROTATION_SPEED * dt
        if abs(rot_diff) < rot_step:
            beetle_red.rotation = beetle_red.target_rotation
        else:
            beetle_red.rotation += rot_step if rot_diff > 0 else -rot_step

    beetle_red.rotation = normalize_angle(beetle_red.rotation)

    # Move red beetle
    if move_dir_x != 0 or move_dir_z != 0:
        length = math.sqrt(move_dir_x**2 + move_dir_z**2)
        move_dir_x /= length
        move_dir_z /= length

        new_x = beetle_red.x + move_dir_x * MOVE_SPEED * dt
        new_z = beetle_red.z + move_dir_z * MOVE_SPEED * dt

        if abs(new_x) < 35.0 and abs(new_z) < 35.0:
            if not check_collision(new_x, new_z, beetle_blue.x, beetle_blue.z):
                beetle_red.x = new_x
                beetle_red.z = new_z

    # Render
    clear_beetles()
    place_beetle_rotated(beetle_blue.x, beetle_blue.z, beetle_blue.rotation, beetle_blue.color)
    place_beetle_rotated(beetle_red.x, beetle_red.z, beetle_red.rotation, beetle_red.color)

    canvas.set_background_color((0.05, 0.05, 0.08))
    renderer.render(camera, canvas, scene, simulation.voxel_type, simulation.n_grid)
    canvas.scene(scene)

    # HUD
    window.GUI.begin("Beetle Rotation", 0.01, 0.01, 0.35, 0.25)
    window.GUI.text("BLUE BEETLE (TGHF)")
    window.GUI.text(f"  Pos: ({beetle_blue.x:.1f}, {beetle_blue.z:.1f})")
    window.GUI.text(f"  Facing: {math.degrees(beetle_blue.rotation):.0f}°")
    window.GUI.text("")
    window.GUI.text("RED BEETLE (IKJL)")
    window.GUI.text(f"  Pos: ({beetle_red.x:.1f}, {beetle_red.z:.1f})")
    window.GUI.text(f"  Facing: {math.degrees(beetle_red.rotation):.0f}°")
    window.GUI.text("")
    dx = beetle_blue.x - beetle_red.x
    dz = beetle_blue.z - beetle_red.z
    distance = math.sqrt(dx*dx + dz*dz)
    window.GUI.text(f"Distance: {distance:.1f}")
    window.GUI.end()

    window.show()

print("Done!")
