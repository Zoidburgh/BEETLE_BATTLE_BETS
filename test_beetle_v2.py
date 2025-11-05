"""
MINIMAL BEETLE TEST v2 - Using working THERMITE renderer
Goal: One beetle, see arena clearly, test physics feel
"""

import taichi as ti
import time

# Initialize Taichi
ti.init(arch=ti.gpu)

# Simple voxel grid for arena (much smaller than full game)
n_grid = 64
voxel_grid = ti.field(dtype=ti.i32, shape=(n_grid, n_grid, n_grid))

EMPTY = 0
FLOOR = 1
BEETLE = 2

# Build circular arena floor
@ti.kernel
def build_arena():
    # Clear everything
    for i, j, k in ti.ndrange(n_grid, n_grid, n_grid):
        voxel_grid[i, j, k] = EMPTY

    # Arena center
    center_x = n_grid // 2
    center_z = n_grid // 2
    arena_radius = 25

    # Floor circle
    for i in range(center_x - arena_radius, center_x + arena_radius):
        for k in range(center_z - arena_radius, center_z + arena_radius):
            dx = float(i - center_x)
            dz = float(k - center_z)
            dist = ti.sqrt(dx * dx + dz * dz)

            if dist <= arena_radius:
                voxel_grid[i, 0, k] = FLOOR  # Ground level
                # Edge markers
                if dist > arena_radius - 2:
                    voxel_grid[i, 1, k] = FLOOR

print("Building arena...")
build_arena()

# Beetle physics state
beetle_x = 0.0  # World coordinates (centered at 0,0)
beetle_y = 2.0  # Elevated
beetle_z = 0.0
beetle_vx = 0.0
beetle_vy = 0.0
beetle_vz = 0.0
beetle_rotation = 0.0  # radians

# Physics constants
ARENA_RADIUS = 20.0
MOVE_FORCE = 100.0
TURN_SPEED = 3.0
FRICTION = 0.85
MAX_SPEED = 20.0

# Render beetle as voxels
@ti.kernel
def place_beetle(grid_x: ti.i32, grid_y: ti.i32, grid_z: ti.i32):
    # Clear old beetle voxels
    for i, j, k in ti.ndrange(n_grid, n_grid, n_grid):
        if voxel_grid[i, j, k] == BEETLE:
            voxel_grid[i, j, k] = EMPTY

    # Place new beetle (simple 3x3x2 box)
    for dx in range(-1, 2):
        for dy in range(0, 2):
            for dz in range(-1, 2):
                x = grid_x + dx
                y = grid_y + dy
                z = grid_z + dz
                if 0 <= x < n_grid and 0 <= y < n_grid and 0 <= z < n_grid:
                    voxel_grid[x, y, z] = BEETLE

# Prepare voxel rendering data
num_voxels = ti.field(ti.i32, shape=())
voxel_positions = ti.Vector.field(3, dtype=ti.f32, shape=100000)
voxel_colors = ti.Vector.field(3, dtype=ti.f32, shape=100000)

@ti.kernel
def build_voxel_list():
    """Convert grid to renderable voxel list"""
    count = 0
    for i, j, k in ti.ndrange(n_grid, n_grid, n_grid):
        voxel_type = voxel_grid[i, j, k]
        if voxel_type != EMPTY:
            # Convert grid coords to world coords
            world_x = float(i - n_grid // 2)
            world_y = float(j)
            world_z = float(k - n_grid // 2)

            voxel_positions[count] = ti.Vector([world_x, world_y, world_z])

            # Color based on type
            if voxel_type == FLOOR:
                voxel_colors[count] = ti.Vector([0.3, 0.3, 0.3])  # Gray floor
            elif voxel_type == BEETLE:
                voxel_colors[count] = ti.Vector([0.2, 0.6, 1.0])  # Blue beetle

            count += 1

    num_voxels[None] = count

# Create window
window = ti.ui.Window("Beetle Test v2", (1920, 1080), vsync=True)
canvas = window.get_canvas()
scene = window.get_scene()
camera = ti.ui.Camera()

# Camera setup
camera_distance = 60.0
camera_angle = 0.0

print("\n=== BEETLE TEST v2 ===")
print("Controls:")
print("  W - Forward")
print("  S - Backward")
print("  A - Turn left")
print("  D - Turn right")
print("  R - Reset")
print("  ESC - Quit")
print("="*30 + "\n")

last_time = time.time()

while window.running:
    # Delta time
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time
    dt = min(dt, 0.05)  # Cap at 50ms

    # === CONTROLS ===
    thrust = 0.0
    turn = 0.0

    if window.is_pressed('w'):
        thrust += 1.0
    if window.is_pressed('s'):
        thrust -= 1.0
    if window.is_pressed('a'):
        turn += 1.0
    if window.is_pressed('d'):
        turn -= 1.0

    if window.is_pressed('r'):
        beetle_x = 0.0
        beetle_z = 0.0
        beetle_vx = 0.0
        beetle_vz = 0.0
        beetle_rotation = 0.0
        print("Reset!")

    # === PHYSICS ===
    import math

    # Rotation
    beetle_rotation += turn * TURN_SPEED * dt

    # Forward direction
    forward_x = math.cos(beetle_rotation)
    forward_z = math.sin(beetle_rotation)

    # Apply thrust
    beetle_vx += forward_x * thrust * MOVE_FORCE * dt
    beetle_vz += forward_z * thrust * MOVE_FORCE * dt

    # Friction
    beetle_vx *= FRICTION
    beetle_vz *= FRICTION

    # Speed cap
    speed = math.sqrt(beetle_vx**2 + beetle_vz**2)
    if speed > MAX_SPEED:
        beetle_vx = (beetle_vx / speed) * MAX_SPEED
        beetle_vz = (beetle_vz / speed) * MAX_SPEED

    # Update position
    beetle_x += beetle_vx * dt
    beetle_z += beetle_vz * dt

    # Arena boundary
    dist = math.sqrt(beetle_x**2 + beetle_z**2)
    if dist > ARENA_RADIUS:
        angle = math.atan2(beetle_z, beetle_x)
        beetle_x = math.cos(angle) * ARENA_RADIUS
        beetle_z = math.sin(angle) * ARENA_RADIUS
        # Bounce back
        beetle_vx *= -0.5
        beetle_vz *= -0.5

    # === UPDATE VOXELS ===
    # Convert beetle world position to grid position
    grid_x = int(beetle_x + n_grid // 2)
    grid_z = int(beetle_z + n_grid // 2)
    grid_y = int(beetle_y)

    place_beetle(grid_x, grid_y, grid_z)
    build_voxel_list()

    # === RENDER ===
    # Camera orbits around arena
    camera_height = 50.0
    camera.position(0.0, camera_height, camera_distance)
    camera.lookat(0.0, 0.0, 0.0)
    camera.up(0.0, 1.0, 0.0)

    scene.set_camera(camera)
    scene.ambient_light((0.4, 0.4, 0.4))
    scene.point_light(pos=(0.0, 100.0, 0.0), color=(1.0, 1.0, 1.0))

    # Render voxels
    if num_voxels[None] > 0:
        scene.particles(
            voxel_positions,
            radius=0.5,
            per_vertex_color=voxel_colors,
            index_count=num_voxels[None]
        )

    canvas.scene(scene)

    # HUD
    window.GUI.begin("Beetle", 0.01, 0.01, 0.25, 0.20)
    window.GUI.text(f"Position: ({beetle_x:.1f}, {beetle_z:.1f})")
    window.GUI.text(f"Speed: {speed:.1f} m/s")
    window.GUI.text(f"Heading: {math.degrees(beetle_rotation):.0f}°")
    window.GUI.text(f"")
    window.GUI.text(f"Voxels: {num_voxels[None]}")
    window.GUI.end()

    window.show()

print("Done!")
