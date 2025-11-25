"""
BEETLE BATTLE - Main Game Loop
Test two beetles with manual controls before adding AI
"""

import taichi as ti
import simulation
import renderer
import beetle
import time

# Create Taichi window and scene
window = ti.ui.Window("BEETLE BATTLE - Arena Test", (2560, 1600), vsync=True)
canvas = window.get_canvas()
scene = window.get_scene()

# Create camera
camera = renderer.Camera()

# OPTIMIZATION: Pre-allocate rendering fields once instead of every frame
# This avoids GPU memory allocation overhead in the render loop
beetle_render_positions = ti.Vector.field(3, ti.f32, shape=beetle.MAX_BEETLES)
beetle_render_colors = ti.Vector.field(3, ti.f32, shape=beetle.MAX_BEETLES)

# Position camera for top-down arena view
camera.pos_x = 0.0
camera.pos_y = 50.0  # High up for top-down view
camera.pos_z = 0.0
camera.pitch = -60.0  # Looking down
camera.yaw = 0.0

# Spawn two test beetles
print("\n=== SPAWNING TEST BEETLES ===")
beetle1_idx = beetle.spawn_beetle(
    pos=ti.math.vec3(-15.0, 0.5, 0.0),  # Left side
    rotation=0.0,  # Facing right
    size=1.0
)
print(f"Beetle 1 spawned at index {beetle1_idx}")

beetle2_idx = beetle.spawn_beetle(
    pos=ti.math.vec3(15.0, 0.5, 0.0),  # Right side
    rotation=3.14159,  # Facing left (180 degrees)
    size=1.0
)
print(f"Beetle 2 spawned at index {beetle2_idx}")

# Performance tracking
frame_times = []
last_fps_update = time.time()
fps_display = 0
frame_count = 0

print("\n=== BEETLE BATTLE CONTROLS ===")
print("BEETLE 1 (Blue):")
print("  W/S - Move forward/backward")
print("  A/D - Turn left/right")
print("\nBEETLE 2 (Red):")
print("  Arrow Up/Down - Move forward/backward")
print("  Arrow Left/Right - Turn left/right")
print("\nCAMERA:")
print("  Mouse - Look around")
print("  Q/E - Move up/down")
print("  ESC - Exit")
print("\n" + "="*50)

last_frame_time = time.time()
game_time = 0.0

while window.running:
    frame_start = time.time()
    dt = frame_start - last_frame_time
    last_frame_time = frame_start
    game_time += dt

    # ===== BEETLE 1 CONTROLS (WASD) =====
    thrust1 = 0.0
    turn1 = 0.0

    if window.is_pressed('w'):
        thrust1 += 1.0
    if window.is_pressed('s'):
        thrust1 -= 1.0
    if window.is_pressed('a'):
        turn1 += 1.0
    if window.is_pressed('d'):
        turn1 -= 1.0

    if thrust1 != 0.0 or turn1 != 0.0:
        beetle.apply_beetle_thrust(beetle1_idx, thrust1 * dt)
        beetle.apply_beetle_turn(beetle1_idx, turn1 * dt)

    # ===== BEETLE 2 CONTROLS (ARROW KEYS) =====
    thrust2 = 0.0
    turn2 = 0.0

    if window.is_pressed(ti.ui.UP):
        thrust2 += 1.0
    if window.is_pressed(ti.ui.DOWN):
        thrust2 -= 1.0
    if window.is_pressed(ti.ui.LEFT):
        turn2 += 1.0
    if window.is_pressed(ti.ui.RIGHT):
        turn2 -= 1.0

    if thrust2 != 0.0 or turn2 != 0.0:
        beetle.apply_beetle_thrust(beetle2_idx, thrust2 * dt)
        beetle.apply_beetle_turn(beetle2_idx, turn2 * dt)

    # ===== CAMERA CONTROLS (MOUSE + Q/E) =====
    renderer.handle_camera_controls(camera, window, dt)
    renderer.handle_mouse_look(camera, window)

    # ===== PHYSICS UPDATE =====
    beetle.update_beetle_physics(dt)
    beetle.check_beetle_collisions()

    # ===== RENDER =====
    canvas.set_background_color((0.05, 0.05, 0.08))  # Dark arena

    # Render arena floor
    renderer.render(camera, canvas, scene, simulation.voxel_type, simulation.n_grid)

    # OPTIMIZATION: Render beetles using pre-allocated fields (no per-frame allocation)
    # Batch all active beetles into a single draw call
    active_beetle_count = 0
    beetle_radius = 1.0  # Default radius, will use first active beetle's radius

    for idx in range(beetle.MAX_BEETLES):
        if beetle.beetle_active[idx] == 1:
            pos = beetle.beetle_pos[idx]
            beetle_radius = beetle.beetle_radius[idx]

            # Color code beetles
            if idx == beetle1_idx:
                color = (0.3, 0.5, 1.0)  # Blue
            elif idx == beetle2_idx:
                color = (1.0, 0.3, 0.3)  # Red
            else:
                color = (0.5, 0.5, 0.5)  # Gray

            # Store in pre-allocated fields (no GPU allocation!)
            beetle_render_positions[active_beetle_count] = pos
            beetle_render_colors[active_beetle_count] = ti.math.vec3(color[0], color[1], color[2])
            active_beetle_count += 1

    # Single batched draw call for all beetles
    if active_beetle_count > 0:
        scene.particles(
            beetle_render_positions,
            radius=beetle_radius,
            per_vertex_color=beetle_render_colors,
            index_count=active_beetle_count
        )

    # Render scene
    canvas.scene(scene)

    # Update FPS counter
    if time.time() - last_fps_update > 0.5:
        if frame_times:
            avg_frame = sum(frame_times) / len(frame_times)
            fps_display = int(1 / avg_frame) if avg_frame > 0 else 0
        last_fps_update = time.time()

    # HUD
    window.GUI.begin("Beetle Battle", 0.01, 0.01, 0.30, 0.50)
    window.GUI.text(f"FPS: {fps_display}")
    window.GUI.text(f"Game Time: {game_time:.1f}s")
    window.GUI.text("")

    window.GUI.text("=== BEETLE 1 (Blue) ===")
    if beetle.beetle_active[beetle1_idx]:
        pos1 = beetle.beetle_pos[beetle1_idx]
        vel1 = beetle.beetle_vel[beetle1_idx]
        rot1 = beetle.beetle_rotation[beetle1_idx] * 57.2958  # Convert to degrees
        window.GUI.text(f"Pos: ({pos1[0]:.1f}, {pos1[2]:.1f})")
        window.GUI.text(f"Speed: {vel1.norm():.1f} m/s")
        window.GUI.text(f"Heading: {rot1:.0f}°")

    window.GUI.text("")
    window.GUI.text("=== BEETLE 2 (Red) ===")
    if beetle.beetle_active[beetle2_idx]:
        pos2 = beetle.beetle_pos[beetle2_idx]
        vel2 = beetle.beetle_vel[beetle2_idx]
        rot2 = beetle.beetle_rotation[beetle2_idx] * 57.2958
        window.GUI.text(f"Pos: ({pos2[0]:.1f}, {pos2[2]:.1f})")
        window.GUI.text(f"Speed: {vel2.norm():.1f} m/s")
        window.GUI.text(f"Heading: {rot2:.0f}°")

    window.GUI.text("")
    window.GUI.text(f"Active beetles: {beetle.num_beetles[None]}")

    window.GUI.end()

    # Show frame
    window.show()

    # Track frame time
    frame_time = time.time() - frame_start
    frame_times.append(frame_time)
    if len(frame_times) > 60:
        frame_times.pop(0)

    frame_count += 1

# Performance summary
if frame_times:
    avg_frame = sum(frame_times) / len(frame_times)
    print(f"\nAverage frame time: {avg_frame*1000:.2f}ms ({1/avg_frame:.1f} FPS)")

print("Beetle Battle shut down.")
