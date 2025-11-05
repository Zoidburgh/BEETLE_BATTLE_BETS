"""
STEP 1: Just render the arena floor
No beetle. No physics. Just: can we see a circular arena?
"""

import taichi as ti

ti.init(arch=ti.gpu)

# Grid for voxels
n_grid = 64
voxel_grid = ti.field(dtype=ti.i32, shape=(n_grid, n_grid, n_grid))

EMPTY = 0
FLOOR = 1

@ti.kernel
def build_arena():
    """Build a simple circular floor"""
    # Clear everything
    for i, j, k in ti.ndrange(n_grid, n_grid, n_grid):
        voxel_grid[i, j, k] = EMPTY

    center_x = 32  # Middle of 64 grid
    center_z = 32
    radius = 28

    # Just a flat circular floor
    for i in range(n_grid):
        for k in range(n_grid):
            dx = float(i - center_x)
            dz = float(k - center_z)
            dist = ti.sqrt(dx * dx + dz * dz)

            # Inside circle = floor
            if dist <= radius:
                voxel_grid[i, 0, k] = FLOOR

print("Building arena...")
build_arena()
print("Arena built!")

# Rendering setup
num_voxels = ti.field(ti.i32, shape=())
voxel_positions = ti.Vector.field(3, dtype=ti.f32, shape=10000)
voxel_colors = ti.Vector.field(3, dtype=ti.f32, shape=10000)

@ti.kernel
def build_render_list():
    """Convert grid to particles for rendering"""
    count = 0
    for i, j, k in ti.ndrange(n_grid, n_grid, n_grid):
        if voxel_grid[i, j, k] == FLOOR:
            # Convert to world coords (center at 0,0)
            x = float(i - 32)
            y = float(j)
            z = float(k - 32)

            voxel_positions[count] = ti.Vector([x, y, z])
            voxel_colors[count] = ti.Vector([0.4, 0.4, 0.4])  # Gray
            count += 1

    num_voxels[None] = count

build_render_list()
print(f"Voxels to render: {num_voxels[None]}")

# Window
window = ti.ui.Window("Arena Test", (1600, 1200), vsync=True)
canvas = window.get_canvas()
scene = window.get_scene()
camera = ti.ui.Camera()

print("\n=== ARENA TEST ===")
print("Just looking at the arena floor")
print("ESC to quit")
print("="*30 + "\n")

while window.running:
    # Camera - looking down at arena from above
    camera.position(0, 60, 0)
    camera.lookat(0, 0, 0)
    camera.up(0, 0, -1)

    scene.set_camera(camera)
    scene.ambient_light((0.8, 0.8, 0.8))
    scene.point_light(pos=(0, 100, 0), color=(1, 1, 1))

    # Render floor
    scene.particles(
        voxel_positions,
        radius=0.5,
        per_vertex_color=voxel_colors,
        index_count=num_voxels[None]
    )

    canvas.scene(scene)
    window.show()

print("Done!")
