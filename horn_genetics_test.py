"""
Horn Genetics Test - Adjust horn parameters with sliders
"""

import taichi as ti
import math
import renderer

ti.init(arch=ti.vulkan)

# Voxel grid
n_grid = 192
voxel_type = ti.field(dtype=ti.i32, shape=(n_grid, n_grid, n_grid))

# Voxel types
EMPTY = 0
CONCRETE = 2
BEETLE_BLUE = 5

# Genetic parameters (mutable)
horn_shaft_length = ti.field(dtype=ti.f32, shape=())
horn_prong_length = ti.field(dtype=ti.f32, shape=())

# Initialize
horn_shaft_length[None] = 8.0
horn_prong_length[None] = 9.0

RENDER_Y_OFFSET = 50.0

@ti.kernel
def clear_grid():
    for i, j, k in voxel_type:
        voxel_type[i, j, k] = EMPTY

@ti.kernel
def create_arena():
    """Create floor"""
    arena_radius = 32
    floor_y_offset = int(RENDER_Y_OFFSET)

    for i, k in ti.ndrange(n_grid, n_grid):
        # Center coordinates
        cx = i - n_grid // 2
        cz = k - n_grid // 2
        dist = ti.sqrt(float(cx * cx + cz * cz))

        if dist <= arena_radius:
            voxel_type[i, floor_y_offset, k] = CONCRETE
            voxel_type[i, floor_y_offset + 1, k] = EMPTY

@ti.kernel
def render_beetle_with_horn(world_x: ti.f32, world_z: ti.f32, shaft_len: ti.f32, prong_len: ti.f32):
    """Render beetle with adjustable horn"""
    # Convert world coords to grid
    center_x = int(world_x + n_grid / 2.0)
    center_y = int(RENDER_Y_OFFSET + 10.0)  # Raised above floor
    center_z = int(world_z + n_grid / 2.0)

    # === BODY (simple sphere) ===
    body_radius = 8
    for dx, dy, dz in ti.ndrange((-body_radius, body_radius+1),
                                   (-body_radius, body_radius+1),
                                   (-body_radius, body_radius+1)):
        dist_sq = dx*dx + dy*dy + dz*dz
        if dist_sq <= body_radius * body_radius:
            x = center_x + dx
            y = center_y + dy
            z = center_z + dz
            if 0 <= x < n_grid and 0 <= y < n_grid and 0 <= z < n_grid:
                voxel_type[x, y, z] = BEETLE_BLUE

    # === HORN (Y-shaped) ===
    horn_x_start = center_x + body_radius  # Start at front of body
    horn_y_start = center_y + 2  # Start slightly above center
    horn_z_center = center_z

    # Main shaft (vertical)
    shaft_voxels = int(shaft_len)
    for i in range(shaft_voxels):
        x = horn_x_start + i
        y = horn_y_start + i  # Diagonal upward
        z = horn_z_center

        # Make shaft 2 voxels thick
        for dz in range(-1, 2):
            if 0 <= x < n_grid and 0 <= y < n_grid and 0 <= z+dz < n_grid:
                voxel_type[x, y, z + dz] = BEETLE_BLUE

    # Prongs (fork at end of shaft)
    prong_start_x = horn_x_start + shaft_voxels
    prong_start_y = horn_y_start + shaft_voxels

    prong_voxels = int(prong_len)
    for i in range(prong_voxels):
        # Left prong (extends +z)
        x = prong_start_x + i
        y = prong_start_y + i // 2  # Slight upward angle
        z = horn_z_center + i
        if 0 <= x < n_grid and 0 <= y < n_grid and 0 <= z < n_grid:
            voxel_type[x, y, z] = BEETLE_BLUE

        # Right prong (extends -z)
        z = horn_z_center - i
        if 0 <= x < n_grid and 0 <= y < n_grid and 0 <= z < n_grid:
            voxel_type[x, y, z] = BEETLE_BLUE

# Initialize
clear_grid()
create_arena()

# Rendering setup
window = ti.ui.Window("Horn Genetics Test", (1280, 720), vsync=True)
canvas = window.get_canvas()
scene = ti.ui.Scene()
camera = renderer.Camera()

# Camera position
camera.pos_x = 0.0
camera.pos_y = 40.0
camera.pos_z = -80.0

print("Horn Genetics Test")
print("=" * 40)
print("Use sliders to adjust horn parameters")
print("Camera: WASD/QE/Arrows")
print("=" * 40)

# Main loop
while window.running:
    # Handle camera
    dt = 0.016  # ~60 FPS
    renderer.handle_camera_controls(camera, window, dt)

    # Clear and rebuild beetle with current genetics
    clear_grid()
    create_arena()
    render_beetle_with_horn(0.0, 0.0, horn_shaft_length[None], horn_prong_length[None])

    # Extract voxels for rendering
    renderer.render(camera, canvas, scene, voxel_type, n_grid)

    # GUI
    window.GUI.begin("HORN GENETICS", 0.02, 0.02, 0.3, 0.3)
    window.GUI.text("=== Adjustable Parameters ===")
    window.GUI.text("")

    # Sliders (returns new value)
    new_shaft = window.GUI.slider_float("Shaft Length", horn_shaft_length[None], 3.0, 15.0)
    new_prong = window.GUI.slider_float("Prong Length", horn_prong_length[None], 2.0, 12.0)

    # Update global values
    horn_shaft_length[None] = new_shaft
    horn_prong_length[None] = new_prong

    window.GUI.text("")
    window.GUI.text(f"Shaft: {horn_shaft_length[None]:.1f} voxels")
    window.GUI.text(f"Prong: {horn_prong_length[None]:.1f} voxels")
    window.GUI.text(f"Total Reach: {horn_shaft_length[None] + horn_prong_length[None]:.1f}")

    window.GUI.end()

    canvas.scene(scene)
    window.show()

print("Done!")
