"""
Step 5: Make blue beetle move randomly
Watch it wander around the arena - testing if autonomous movement is interesting
"""

import taichi as ti
import simulation
import renderer
import time
import random

# Beetle positions (grid coordinates)
beetle_blue_x = 64 - 15  # West side
beetle_blue_z = 64
beetle_red_x = 64 + 15   # East side (stays still)
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

# Initial placement
clear_beetles()
place_beetle_blue(beetle_blue_x, beetle_blue_z)
place_beetle_red(beetle_red_x, beetle_red_z)

# Window
window = ti.ui.Window("Random Beetle Test", (1920, 1080), vsync=True)
canvas = window.get_canvas()
scene = window.get_scene()

# Camera
camera = renderer.Camera()
camera.pos_x = 0.0
camera.pos_y = 60.0
camera.pos_z = 0.0
camera.pitch = -70.0
camera.yaw = 0.0

print("\n=== RANDOM BEETLE TEST ===")
print("Blue beetle moves randomly every 0.5 seconds")
print("Red beetle stays still")
print("Just watch!")
print("\nCamera: WASD/Mouse/Q/E")
print("ESC - Quit")
print("="*40 + "\n")

last_move_time = 0.0
move_interval = 0.5  # Move every 0.5 seconds

while window.running:
    dt = 0.016
    current_time = time.time()

    # Camera controls
    renderer.handle_camera_controls(camera, window, dt)
    renderer.handle_mouse_look(camera, window)

    # Random movement for blue beetle
    if current_time - last_move_time > move_interval:
        # Pick random direction: 0=north, 1=south, 2=west, 3=east
        direction = random.randint(0, 3)

        new_x = beetle_blue_x
        new_z = beetle_blue_z

        if direction == 0:  # North (-Z)
            new_z -= 1
        elif direction == 1:  # South (+Z)
            new_z += 1
        elif direction == 2:  # West (-X)
            new_x -= 1
        elif direction == 3:  # East (+X)
            new_x += 1

        # Keep in bounds
        if 10 <= new_x < simulation.n_grid - 10 and 10 <= new_z < simulation.n_grid - 10:
            beetle_blue_x = new_x
            beetle_blue_z = new_z

            # Update positions
            clear_beetles()
            place_beetle_blue(beetle_blue_x, beetle_blue_z)
            place_beetle_red(beetle_red_x, beetle_red_z)

            last_move_time = current_time

            # Print occasionally (not every move)
            if random.random() < 0.2:
                world_x = beetle_blue_x - 64
                world_z = beetle_blue_z - 64
                print(f"Blue beetle at ({world_x}, {world_z})")

    # Render
    canvas.set_background_color((0.05, 0.05, 0.08))
    renderer.render(camera, canvas, scene, simulation.voxel_type, simulation.n_grid)
    canvas.scene(scene)

    # HUD
    window.GUI.begin("Random Beetle", 0.01, 0.01, 0.25, 0.20)
    window.GUI.text("BLUE BEETLE (Random AI)")
    world_x = beetle_blue_x - 64
    world_z = beetle_blue_z - 64
    window.GUI.text(f"  Position: ({world_x}, {world_z})")
    window.GUI.text(f"")
    window.GUI.text("RED BEETLE (Still)")
    red_world_x = beetle_red_x - 64
    red_world_z = beetle_red_z - 64
    window.GUI.text(f"  Position: ({red_world_x}, {red_world_z})")
    window.GUI.text(f"")
    window.GUI.text("Watch the blue beetle wander...")
    window.GUI.end()

    window.show()

print("Done!")
