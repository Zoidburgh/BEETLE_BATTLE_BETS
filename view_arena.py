"""
Use existing THERMITE renderer to view the beetle arena
"""

import taichi as ti
import simulation
import renderer
import time

# The arena is already built in simulation.py (init_beetle_arena)
# Just view it

window = ti.ui.Window("Beetle Arena", (1920, 1080), vsync=True)
canvas = window.get_canvas()
scene = window.get_scene()

# Camera setup
camera = renderer.Camera()
camera.pos_x = 0.0
camera.pos_y = 60.0
camera.pos_z = 0.0
camera.pitch = -70.0  # Looking down
camera.yaw = 0.0

print("=== VIEWING BEETLE ARENA ===")
print("Arena should be visible below")
print("Camera controls:")
print("  Mouse - Look around")
print("  WASD - Move")
print("  Q/E - Up/Down")
print("  ESC - Quit")
print("="*40)

while window.running:
    dt = 0.016

    # Camera controls
    renderer.handle_camera_controls(camera, window, dt)
    renderer.handle_mouse_look(camera, window)

    # Render
    canvas.set_background_color((0.05, 0.05, 0.08))
    renderer.render(camera, canvas, scene, simulation.voxel_type, simulation.n_grid)
    canvas.scene(scene)

    # Stats
    window.GUI.begin("Arena", 0.01, 0.01, 0.2, 0.15)
    window.GUI.text(f"Voxels: {renderer.num_voxels[None]}")
    window.GUI.text(f"Camera Y: {camera.pos_y:.1f}")
    window.GUI.text(f"Pitch: {camera.pitch:.1f}")
    window.GUI.end()

    window.show()

print("Done!")
