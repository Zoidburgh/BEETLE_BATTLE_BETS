"""
MINIMAL BEETLE TEST - Vibe Coding Phase 0
Goal: One box beetle, WASD controls, feel the physics

This is the FIRST STEP. No AI, no fancy models, just: can I push a box around?
"""

import taichi as ti

# Initialize Taichi
ti.init(arch=ti.gpu)

# Create window
window = ti.ui.Window("Beetle Test", (1280, 800), vsync=True)
canvas = window.get_canvas()
scene = ti.ui.Scene()
camera = ti.ui.Camera()

# Camera setup - top-down view
camera.position(0.0, 50.0, 0.0)  # High up
camera.lookat(0.0, 0.0, 0.0)  # Looking at center
camera.up(0.0, 0.0, -1.0)  # Z-axis points "up" in screen

# Beetle state (single beetle, simple physics)
beetle_pos = ti.Vector([0.0, 1.0, 0.0])  # Start at center, slightly elevated
beetle_vel = ti.Vector([0.0, 0.0, 0.0])  # Velocity
beetle_rotation = 0.0  # Facing +X

# Physics constants
ARENA_RADIUS = 40.0
MOVE_FORCE = 50.0
FRICTION = 0.9
TURN_SPEED = 2.0

print("MINIMAL BEETLE TEST")
print("Controls:")
print("  W - Forward")
print("  S - Backward")
print("  A - Turn left")
print("  D - Turn right")
print("  R - Reset to center")
print("  ESC - Quit")
print("")

# Voxel data for rendering beetle (simple box)
beetle_voxels = ti.Vector.field(3, dtype=ti.f32, shape=1)
beetle_colors = ti.Vector.field(3, dtype=ti.f32, shape=1)
beetle_voxels[0] = beetle_pos
beetle_colors[0] = ti.Vector([0.2, 0.6, 1.0])  # Blue

# Arena floor (simple circle of points for visualization)
NUM_ARENA_POINTS = 100
arena_points = ti.Vector.field(3, dtype=ti.f32, shape=NUM_ARENA_POINTS)
arena_colors = ti.Vector.field(3, dtype=ti.f32, shape=NUM_ARENA_POINTS)

@ti.kernel
def init_arena():
    for i in range(NUM_ARENA_POINTS):
        angle = float(i) / float(NUM_ARENA_POINTS) * 6.28318
        x = ti.cos(angle) * ARENA_RADIUS
        z = ti.sin(angle) * ARENA_RADIUS
        arena_points[i] = ti.Vector([x, 0.0, z])
        arena_colors[i] = ti.Vector([0.8, 0.8, 0.8])

init_arena()

import time
last_time = time.time()

while window.running:
    # Calculate dt
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time
    dt = min(dt, 0.1)  # Cap at 100ms

    # === CONTROLS ===
    thrust = 0.0
    turn = 0.0

    if window.is_pressed('w'):
        thrust = 1.0
    if window.is_pressed('s'):
        thrust = -1.0
    if window.is_pressed('a'):
        turn = 1.0
    if window.is_pressed('d'):
        turn = -1.0
    if window.is_pressed('r'):
        # Reset
        beetle_pos = ti.Vector([0.0, 1.0, 0.0])
        beetle_vel = ti.Vector([0.0, 0.0, 0.0])
        beetle_rotation = 0.0
        print("Reset!")

    # === PHYSICS ===
    # Apply rotation
    beetle_rotation += turn * TURN_SPEED * dt

    # Calculate forward direction
    import math
    forward_x = math.cos(beetle_rotation)
    forward_z = math.sin(beetle_rotation)

    # Apply thrust
    if thrust != 0.0:
        beetle_vel[0] += forward_x * thrust * MOVE_FORCE * dt
        beetle_vel[2] += forward_z * thrust * MOVE_FORCE * dt

    # Apply friction
    beetle_vel *= FRICTION

    # Update position
    beetle_pos[0] += beetle_vel[0] * dt
    beetle_pos[2] += beetle_vel[2] * dt

    # Arena boundary (simple bounce back)
    dist = math.sqrt(beetle_pos[0]**2 + beetle_pos[2]**2)
    if dist > ARENA_RADIUS - 2.0:
        # Push back to edge
        angle = math.atan2(beetle_pos[2], beetle_pos[0])
        beetle_pos[0] = math.cos(angle) * (ARENA_RADIUS - 2.0)
        beetle_pos[2] = math.sin(angle) * (ARENA_RADIUS - 2.0)
        # Stop velocity
        beetle_vel *= 0.5

    # Update beetle voxel position
    beetle_voxels[0] = beetle_pos

    # === RENDER ===
    camera.position(0.0, 50.0, 0.0)
    camera.lookat(0.0, 0.0, 0.0)
    camera.up(0.0, 0.0, -1.0)

    scene.set_camera(camera)
    scene.ambient_light((0.3, 0.3, 0.3))
    scene.point_light(pos=(0.0, 100.0, 0.0), color=(1.0, 1.0, 1.0))

    # Draw arena
    scene.particles(arena_points, radius=0.3, per_vertex_color=arena_colors)

    # Draw beetle (as big sphere for now)
    scene.particles(beetle_voxels, radius=3.0, per_vertex_color=beetle_colors)

    canvas.scene(scene)

    # Simple HUD
    window.GUI.begin("Beetle", 0.01, 0.01, 0.2, 0.15)
    window.GUI.text(f"Pos: ({beetle_pos[0]:.1f}, {beetle_pos[2]:.1f})")
    window.GUI.text(f"Speed: {math.sqrt(beetle_vel[0]**2 + beetle_vel[2]**2):.1f}")
    window.GUI.text(f"Angle: {math.degrees(beetle_rotation):.0f}°")
    window.GUI.end()

    window.show()

print("Done!")
