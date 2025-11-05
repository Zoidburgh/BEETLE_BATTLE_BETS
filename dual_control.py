"""
Step 6: Blue beetle (random AI) vs Red beetle (IJKL player control)
Test collision detection - beetles shouldn't overlap
"""

import taichi as ti
import simulation
import renderer
import time
import random

# Beetle positions (grid coordinates)
beetle_blue_x = 64 - 15  # West side
beetle_blue_z = 64
beetle_red_x = 64 + 15   # East side
beetle_red_z = 64

@ti.kernel
def clear_beetles():
    """Remove all beetle voxels from grid"""
    for i, j, k in ti.ndrange(simulation.n_grid, simulation.n_grid, simulation.n_grid):
        if simulation.voxel_type[i, j, k] == simulation.BEETLE_BLUE or \
           simulation.voxel_type[i, j, k] == simulation.BEETLE_RED:
            simulation.voxel_type[i, j, k] = simulation.EMPTY

@ti.kernel
def place_beetle_blue(x: ti.i32, z: ti.i32):
    """Place blue beetle at position"""
    for dx in range(-3, 4):
        for dy in range(0, 3):
            for dz in range(-2, 3):
                grid_x = x + dx
                grid_y = 2 + dy
                grid_z = z + dz
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = simulation.BEETLE_BLUE

@ti.kernel
def place_beetle_red(x: ti.i32, z: ti.i32):
    """Place red beetle at position"""
    for dx in range(-3, 4):
        for dy in range(0, 3):
            for dz in range(-2, 3):
                grid_x = x + dx
                grid_y = 2 + dy
                grid_z = z + dz
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = simulation.BEETLE_RED

def check_collision(x1, z1, x2, z2):
    """Check if two beetles would collide (simple distance check)"""
    dx = x1 - x2
    dz = z1 - z2
    distance = (dx*dx + dz*dz) ** 0.5
    min_distance = 8  # Beetles need to be at least 8 units apart
    return distance < min_distance

# Initial placement
clear_beetles()
place_beetle_blue(beetle_blue_x, beetle_blue_z)
place_beetle_red(beetle_red_x, beetle_red_z)

# Window
window = ti.ui.Window("Dual Control Test", (1920, 1080), vsync=True)
canvas = window.get_canvas()
scene = window.get_scene()

# Camera
camera = renderer.Camera()
camera.pos_x = 0.0
camera.pos_y = 60.0
camera.pos_z = 0.0
camera.pitch = -70.0
camera.yaw = 0.0

print("\n=== DUAL CONTROL TEST ===")
print("Blue beetle: Random AI")
print("Red beetle: IJKL controls")
print("  I - Forward (north)")
print("  K - Backward (south)")
print("  J - Left (west)")
print("  L - Right (east)")
print("\nTry to bump into the blue beetle!")
print("Camera: WASD/Mouse/Q/E")
print("ESC - Quit")
print("="*40 + "\n")

last_ai_move = 0.0
ai_move_interval = 0.5
last_player_move = 0.0
player_move_cooldown = 0.15

while window.running:
    dt = 0.016
    current_time = time.time()

    # Camera controls
    renderer.handle_camera_controls(camera, window, dt)
    renderer.handle_mouse_look(camera, window)

    # === BLUE BEETLE: Random AI ===
    if current_time - last_ai_move > ai_move_interval:
        direction = random.randint(0, 3)
        new_x = beetle_blue_x
        new_z = beetle_blue_z

        if direction == 0:
            new_z -= 1
        elif direction == 1:
            new_z += 1
        elif direction == 2:
            new_x -= 1
        elif direction == 3:
            new_x += 1

        # Check bounds and collision
        if 10 <= new_x < simulation.n_grid - 10 and 10 <= new_z < simulation.n_grid - 10:
            if not check_collision(new_x, new_z, beetle_red_x, beetle_red_z):
                beetle_blue_x = new_x
                beetle_blue_z = new_z
                last_ai_move = current_time
            else:
                # Collision! Don't move
                print("Blue beetle blocked by red beetle!")
                last_ai_move = current_time

    # === RED BEETLE: Player IJKL controls ===
    if current_time - last_player_move > player_move_cooldown:
        moved = False
        new_x = beetle_red_x
        new_z = beetle_red_z

        if window.is_pressed('i'):
            new_z -= 1
            moved = True
        elif window.is_pressed('k'):
            new_z += 1
            moved = True
        elif window.is_pressed('j'):
            new_x -= 1
            moved = True
        elif window.is_pressed('l'):
            new_x += 1
            moved = True

        if moved:
            # Check bounds and collision
            if 10 <= new_x < simulation.n_grid - 10 and 10 <= new_z < simulation.n_grid - 10:
                if not check_collision(new_x, new_z, beetle_blue_x, beetle_blue_z):
                    beetle_red_x = new_x
                    beetle_red_z = new_z
                    last_player_move = current_time
                else:
                    # Collision! Don't move
                    print("Red beetle bumped into blue beetle!")
                    last_player_move = current_time

    # Update visuals
    clear_beetles()
    place_beetle_blue(beetle_blue_x, beetle_blue_z)
    place_beetle_red(beetle_red_x, beetle_red_z)

    # Render
    canvas.set_background_color((0.05, 0.05, 0.08))
    renderer.render(camera, canvas, scene, simulation.voxel_type, simulation.n_grid)
    canvas.scene(scene)

    # HUD
    window.GUI.begin("Dual Control", 0.01, 0.01, 0.30, 0.25)
    window.GUI.text("BLUE BEETLE (Random AI)")
    blue_wx = beetle_blue_x - 64
    blue_wz = beetle_blue_z - 64
    window.GUI.text(f"  Position: ({blue_wx}, {blue_wz})")
    window.GUI.text("")
    window.GUI.text("RED BEETLE (You control)")
    red_wx = beetle_red_x - 64
    red_wz = beetle_red_z - 64
    window.GUI.text(f"  Position: ({red_wx}, {red_wz})")
    window.GUI.text("")

    # Distance
    dx = beetle_blue_x - beetle_red_x
    dz = beetle_blue_z - beetle_red_z
    distance = (dx*dx + dz*dz) ** 0.5
    window.GUI.text(f"Distance: {distance:.1f}")
    window.GUI.text("")
    window.GUI.text("Use IJKL to chase blue beetle!")
    window.GUI.end()

    window.show()

print("Done!")
