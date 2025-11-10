"""
Simple beetle controls - grid-based movement with smooth rendering
TGHF for blue beetle, IKJL for red beetle
"""

import taichi as ti
import simulation
import renderer
import time

# Beetle positions (continuous float for smooth rendering)
beetle_blue_x = -15.0
beetle_blue_z = 0.0
beetle_red_x = 15.0
beetle_red_z = 0.0

# Movement speed (units per second)
MOVE_SPEED = 10.0

@ti.kernel
def clear_beetles():
    """Remove all beetle voxels"""
    for i, j, k in ti.ndrange(simulation.n_grid, simulation.n_grid, simulation.n_grid):
        if simulation.voxel_type[i, j, k] == simulation.BEETLE_BLUE or \
           simulation.voxel_type[i, j, k] == simulation.BEETLE_RED:
            simulation.voxel_type[i, j, k] = simulation.EMPTY

@ti.kernel
def place_beetle(world_x: ti.f32, world_z: ti.f32, color_type: ti.i32):
    """Place beetle at world position"""
    center_x = int(world_x + simulation.n_grid / 2.0)
    center_z = int(world_z + simulation.n_grid / 2.0)

    for dx in range(-3, 4):
        for dy in range(0, 3):
            for dz in range(-2, 3):
                grid_x = center_x + dx
                grid_y = 2 + dy
                grid_z = center_z + dz
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

def check_collision(x1, z1, x2, z2):
    """Check if two beetles would collide"""
    dx = x1 - x2
    dz = z1 - z2
    distance = (dx*dx + dz*dz) ** 0.5
    return distance < 8.0

# Window
window = ti.ui.Window("Beetle Controls", (1920, 1080), vsync=True)
canvas = window.get_canvas()
scene = window.get_scene()

camera = renderer.Camera()
camera.pos_x = 0.0
camera.pos_y = 60.0
camera.pos_z = 0.0
camera.pitch = -70.0
camera.yaw = 0.0

print("\n=== BEETLE CONTROLS ===")
print("BLUE BEETLE (TGHF):")
print("  T - North")
print("  G - South")
print("  H - West")
print("  F - East")
print("")
print("RED BEETLE (IKJL):")
print("  I - North")
print("  K - South")
print("  J - West")
print("  L - East")
print("")
print("Camera: WASD/Mouse/Q/E")
print("="*40 + "\n")

last_time = time.time()

while window.running:
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time
    dt = min(dt, 0.05)  # Cap dt

    # Camera
    renderer.handle_camera_controls(camera, window, dt)
    renderer.handle_mouse_look(camera, window)

    # === BLUE BEETLE CONTROLS (TGHF) ===
    new_blue_x = beetle_blue_x
    new_blue_z = beetle_blue_z

    if window.is_pressed('t'):
        new_blue_z -= MOVE_SPEED * dt
    if window.is_pressed('g'):
        new_blue_z += MOVE_SPEED * dt
    if window.is_pressed('h'):
        new_blue_x -= MOVE_SPEED * dt
    if window.is_pressed('f'):
        new_blue_x += MOVE_SPEED * dt

    # Check bounds and collision for blue
    if abs(new_blue_x) < 35.0 and abs(new_blue_z) < 35.0:
        if not check_collision(new_blue_x, new_blue_z, beetle_red_x, beetle_red_z):
            beetle_blue_x = new_blue_x
            beetle_blue_z = new_blue_z

    # === RED BEETLE CONTROLS (IKJL) ===
    new_red_x = beetle_red_x
    new_red_z = beetle_red_z

    if window.is_pressed('i'):
        new_red_z -= MOVE_SPEED * dt
    if window.is_pressed('k'):
        new_red_z += MOVE_SPEED * dt
    if window.is_pressed('j'):
        new_red_x -= MOVE_SPEED * dt
    if window.is_pressed('l'):
        new_red_x += MOVE_SPEED * dt

    # Check bounds and collision for red
    if abs(new_red_x) < 35.0 and abs(new_red_z) < 35.0:
        if not check_collision(new_red_x, new_red_z, beetle_blue_x, beetle_blue_z):
            beetle_red_x = new_red_x
            beetle_red_z = new_red_z

    # Render
    clear_beetles()
    place_beetle(beetle_blue_x, beetle_blue_z, simulation.BEETLE_BLUE)
    place_beetle(beetle_red_x, beetle_red_z, simulation.BEETLE_RED)

    canvas.set_background_color((0.05, 0.05, 0.08))
    renderer.render(camera, canvas, scene, simulation.voxel_type, simulation.n_grid)
    canvas.scene(scene)

    # HUD
    window.GUI.begin("Beetle Controls", 0.01, 0.01, 0.30, 0.20)
    window.GUI.text("BLUE BEETLE (TGHF)")
    window.GUI.text(f"  Pos: ({beetle_blue_x:.1f}, {beetle_blue_z:.1f})")
    window.GUI.text("")
    window.GUI.text("RED BEETLE (IKJL)")
    window.GUI.text(f"  Pos: ({beetle_red_x:.1f}, {beetle_red_z:.1f})")
    window.GUI.text("")
    dx = beetle_blue_x - beetle_red_x
    dz = beetle_blue_z - beetle_red_z
    distance = (dx*dx + dz*dz) ** 0.5
    window.GUI.text(f"Distance: {distance:.1f}")
    window.GUI.end()

    window.show()

print("Done!")
