"""
Step 3: Move the beetle with IJKL keys
I = forward, K = backward, J = left, L = right
"""

import taichi as ti
import simulation
import renderer
import time

# Beetle position (grid coordinates)
beetle_x = 64
beetle_z = 64

@ti.kernel
def clear_beetle():
    """Remove all beetle voxels from grid"""
    for i, j, k in ti.ndrange(simulation.n_grid, simulation.n_grid, simulation.n_grid):
        if simulation.voxel_type[i, j, k] == simulation.BEETLE:
            simulation.voxel_type[i, j, k] = simulation.EMPTY

@ti.kernel
def place_beetle(x: ti.i32, z: ti.i32):
    """Place beetle at grid position x, z"""
    # Place 7x3x5 box at position
    for dx in range(-3, 4):  # Length 7
        for dy in range(0, 3):  # Height 3
            for dz in range(-2, 3):  # Width 5
                grid_x = x + dx
                grid_y = 2 + dy  # Floor is at y=0,1
                grid_z = z + dz
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = simulation.BEETLE

# Initial placement
place_beetle(beetle_x, beetle_z)

# Window
window = ti.ui.Window("Move Beetle Test", (1920, 1080), vsync=True)
canvas = window.get_canvas()
scene = window.get_scene()

# Camera
camera = renderer.Camera()
camera.pos_x = 0.0
camera.pos_y = 60.0
camera.pos_z = 0.0
camera.pitch = -70.0
camera.yaw = 0.0

print("\n=== MOVE BEETLE TEST ===")
print("IJKL controls:")
print("  I - Move forward (north, -Z)")
print("  K - Move backward (south, +Z)")
print("  J - Move left (west, -X)")
print("  L - Move right (east, +X)")
print("\nCamera: WASD/Mouse/Q/E")
print("ESC - Quit")
print("="*40 + "\n")

last_move_time = 0.0
move_cooldown = 0.15  # Seconds between moves

while window.running:
    dt = 0.016
    current_time = time.time()

    # Camera controls (WASD)
    renderer.handle_camera_controls(camera, window, dt)
    renderer.handle_mouse_look(camera, window)

    # Beetle movement (IJKL) - with cooldown so it doesn't move too fast
    if current_time - last_move_time > move_cooldown:
        moved = False
        new_x = beetle_x
        new_z = beetle_z

        if window.is_pressed('i'):
            new_z -= 1  # Forward (north)
            moved = True
        elif window.is_pressed('k'):
            new_z += 1  # Backward (south)
            moved = True
        elif window.is_pressed('j'):
            new_x -= 1  # Left (west)
            moved = True
        elif window.is_pressed('l'):
            new_x += 1  # Right (east)
            moved = True

        if moved:
            # Keep beetle in arena bounds (rough check)
            if 10 <= new_x < simulation.n_grid - 10 and 10 <= new_z < simulation.n_grid - 10:
                beetle_x = new_x
                beetle_z = new_z
                clear_beetle()  # Remove old position
                place_beetle(beetle_x, beetle_z)  # Place at new position
                last_move_time = current_time
                print(f"Beetle moved to ({beetle_x}, {beetle_z})")

    # Render
    canvas.set_background_color((0.05, 0.05, 0.08))
    renderer.render(camera, canvas, scene, simulation.voxel_type, simulation.n_grid)
    canvas.scene(scene)

    # HUD
    window.GUI.begin("Beetle Control", 0.01, 0.01, 0.25, 0.20)
    window.GUI.text(f"Beetle Position:")
    window.GUI.text(f"  Grid: ({beetle_x}, {beetle_z})")
    world_x = beetle_x - 64
    world_z = beetle_z - 64
    window.GUI.text(f"  World: ({world_x}, {world_z})")
    window.GUI.text(f"")
    window.GUI.text("IJKL to move beetle")
    window.GUI.end()

    window.show()

print("Done!")
