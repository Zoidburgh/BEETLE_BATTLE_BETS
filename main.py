import taichi as ti
import importlib
import simulation
import renderer
import targeting
import physics
import time

# Create Taichi window and scene (sized for 2880x1800 screen)
window = ti.ui.Window("SIEGE - Voxel Destruction", (2560, 1600), vsync=True)
canvas = window.get_canvas()
scene = window.get_scene()

# Create camera (in main.py so it doesn't reset on hot reload)
camera = renderer.Camera()

running = True
last_reload = time.time()

# Performance tracking
frame_times = []
last_fps_update = time.time()
fps_display = 0
frame_count = 0
physics_time = 0.0

# Physics parameters (adjustable with sliders)
destruction_radius = 8.0  # Base explosion radius
explosion_force = 30.0    # Debris initial velocity
support_threshold = 2     # Minimum voxels needed below for support (out of 9)
launch_speed = 100.0      # Projectile launch velocity (m/s)

print("THERMITE - Voxel Destruction Physics")
print("Controls:")
print("  WASD - Move forward/left/back/right")
print("  Q/E - Move up/down")
print("  ARROW KEYS - Rotate camera")
print("  Mouse - Look around")
print("  SPACE - Throw cannonball")
print("  +/- - Adjust movement speed")
print("  ESC - Exit")

last_frame_time = time.time()

while window.running:
    frame_start = time.time()
    dt = frame_start - last_frame_time
    last_frame_time = frame_start

    # Handle camera controls
    renderer.handle_camera_controls(camera, window, dt)
    renderer.handle_mouse_look(camera, window)

    # Update targeting (raycast from camera)
    targeting.update_targeting(camera, simulation.voxel_type, simulation.n_grid)

    # ===== PHYSICS UPDATE =====
    physics_start = time.time()

    # SPACE key to throw projectile
    if window.is_pressed(' '):  # Space bar
        # Get camera position and forward direction
        cam_pos = ti.math.vec3(camera.pos_x, camera.pos_y, camera.pos_z)
        forward = camera.get_forward_vector()

        # Calculate launch velocity (forward direction * launch speed)
        launch_vel = ti.math.vec3(
            forward[0] * launch_speed,
            forward[1] * launch_speed,
            forward[2] * launch_speed
        )

        # Spawn projectile slightly in front of camera
        spawn_offset = ti.math.vec3(forward[0] * 2.0, forward[1] * 2.0, forward[2] * 2.0)
        spawn_pos = cam_pos + spawn_offset

        # Spawn the projectile
        idx = physics.spawn_projectile(spawn_pos, launch_vel)
        if idx >= 0:
            print(f"Cannonball fired! (Speed: {launch_speed:.1f} m/s)")
        else:
            print("Too many projectiles in flight!")

    # Update projectile physics every frame
    if simulation.num_projectiles[None] > 0:
        physics.update_projectiles(dt)

        # Check for projectile collisions
        collision_idx = physics.check_projectile_collisions()
        if collision_idx >= 0:
            # Get projectile data before deactivating
            impact_pos = simulation.projectile_pos[collision_idx]
            impact_vel = simulation.projectile_vel[collision_idx]
            projectile_speed = impact_vel.norm()

            # Deactivate projectile
            simulation.projectile_active[collision_idx] = 0
            simulation.num_projectiles[None] -= 1

            # Trigger impact explosion
            stats = physics.trigger_impact(
                impact_pos,
                projectile_speed=projectile_speed,
                base_radius=destruction_radius,
                explosion_force=explosion_force,
                support_threshold=support_threshold
            )
            print(f"IMPACT! Destroyed: {stats['destroyed']}, Falling: {stats['falling']}, Total debris: {stats['total_debris']}")

    # Update debris physics every frame
    if simulation.num_debris[None] > 0:
        physics.update_debris_physics(dt)

    # Cleanup old debris every 60 frames (~1 second)
    if frame_count % 60 == 0 and simulation.num_debris[None] > 0:
        physics.cleanup_old_debris()

    physics_time = time.time() - physics_start
    frame_count += 1

    # Hot reload modules every 0.5 seconds
    # DISABLED: Reloading renderer causes GPU memory leak (recreates Taichi fields)
    # if time.time() - last_reload > 0.5:
    #     try:
    #         importlib.reload(renderer)
    #         last_reload = time.time()
    #     except Exception as e:
    #         print(f"Reload error: {e}")

    # Render
    render_start = time.time()
    canvas.set_background_color((0.08, 0.08, 0.12))

    # Build scene (adds voxels to scene)
    renderer.render(camera, canvas, scene, simulation.voxel_type, simulation.n_grid)

    # Add target marker to scene
    targeting.render_target_marker(scene)

    # Now render complete scene to canvas
    canvas.scene(scene)

    render_time = time.time() - render_start

    # Update FPS counter every 0.5 seconds
    if time.time() - last_fps_update > 0.5:
        if frame_times:
            avg_frame = sum(frame_times) / len(frame_times)
            fps_display = int(1 / avg_frame) if avg_frame > 0 else 0
        last_fps_update = time.time()

    # HUD text (basic text overlay)
    window.GUI.begin("Stats", 0.01, 0.01, 0.30, 0.40)
    window.GUI.text(f"FPS: {fps_display}")
    window.GUI.text(f"Physics: {physics_time*1000:.1f}ms")
    window.GUI.text(f"Render: {render_time*1000:.1f}ms")
    window.GUI.text(f"")
    window.GUI.text(f"Voxels: {renderer.num_voxels[None]}")
    window.GUI.text(f"Debris: {simulation.num_debris[None]}")
    window.GUI.text(f"Projectiles: {simulation.num_projectiles[None]}")
    window.GUI.text(f"")
    window.GUI.text(f"Position: ({camera.pos_x:.1f}, {camera.pos_y:.1f}, {camera.pos_z:.1f})")
    window.GUI.text(f"Look: Yaw {camera.yaw:.1f}° Pitch {camera.pitch:.1f}°")
    window.GUI.text(f"Speed: {camera.move_speed:.1f}")
    window.GUI.text(f"")
    window.GUI.text(f"[SPACE to fire cannonball]")
    window.GUI.end()

    # Physics tuning sliders
    window.GUI.begin("Physics Tuning", 0.01, 0.42, 0.30, 0.40)
    window.GUI.text("DESTRUCTION")
    destruction_radius = window.GUI.slider_float("Radius", destruction_radius, 2.0, 20.0)
    window.GUI.text(f"  {destruction_radius:.1f} voxels")
    window.GUI.text("")
    window.GUI.text("DEBRIS")
    explosion_force = window.GUI.slider_float("Force", explosion_force, 10.0, 60.0)
    window.GUI.text(f"  {explosion_force:.1f} m/s")
    window.GUI.text("")
    window.GUI.text("CANNONBALL")
    launch_speed = window.GUI.slider_float("Speed", launch_speed, 20.0, 200.0)
    window.GUI.text(f"  {launch_speed:.1f} m/s")
    window.GUI.text("")
    window.GUI.text("COLLAPSE")
    support_threshold = window.GUI.slider_int("Support", support_threshold, 0, 5)
    window.GUI.text(f"  {support_threshold}/9 voxels below")
    window.GUI.text("")
    window.GUI.text("RESET")
    if window.GUI.button("Rebuild Beetle"):
        print("Rebuilding RHINO BEETLE...")
        simulation.num_debris[None] = 0  # Clear all debris
        simulation.init_mega_fortress()  # Rebuild structure
        print("Beetle rebuilt!")
    window.GUI.end()

    # Show frame
    window.show()

    # Track frame time
    frame_time = time.time() - frame_start
    frame_times.append(frame_time)
    if len(frame_times) > 60:
        frame_times.pop(0)

# Print performance summary
if frame_times:
    avg_frame = sum(frame_times) / len(frame_times)
    print(f"\nAverage frame time: {avg_frame*1000:.2f}ms ({1/avg_frame:.1f} FPS)")

print("Voxel renderer shut down.")
