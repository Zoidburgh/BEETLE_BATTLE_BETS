import taichi as ti
import numpy as np
import simulation

# Maximum number of voxels to render (increased for dense citadel)
MAX_VOXELS = 200000

# Voxel type constants (must match simulation.py)
MOLTEN = 3
DEBRIS = 4

# Taichi fields for rendering
num_voxels = ti.field(dtype=ti.i32, shape=())
voxel_positions = ti.Vector.field(3, dtype=ti.f32, shape=MAX_VOXELS)
voxel_colors = ti.Vector.field(3, dtype=ti.f32, shape=MAX_VOXELS)

# Projectile rendering (cannonballs)
num_projectiles_render = ti.field(dtype=ti.i32, shape=())
projectile_positions = ti.Vector.field(3, dtype=ti.f32, shape=10)  # Max 10 projectiles
projectile_colors = ti.Vector.field(3, dtype=ti.f32, shape=10)

# Gradient background (2 triangles forming full-screen quad)
gradient_positions = ti.Vector.field(2, dtype=ti.f32, shape=6)
gradient_colors = ti.Vector.field(3, dtype=ti.f32, shape=6)

# OPTIMIZATION: Pre-computed metallic shimmer lookup table (256KB, ~8-12% render speedup)
SHIMMER_TABLE_SIZE = 256
shimmer_lut = ti.field(dtype=ti.f32, shape=(SHIMMER_TABLE_SIZE, SHIMMER_TABLE_SIZE))

@ti.kernel
def init_shimmer_lut():
    """Pre-compute metallic shimmer values for all arena positions"""
    for i, j in ti.ndrange(SHIMMER_TABLE_SIZE, SHIMMER_TABLE_SIZE):
        # Map table indices to world coordinates (arena is 128×128, centered at origin)
        world_x = (i / SHIMMER_TABLE_SIZE) * 128.0 - 64.0
        world_z = (j / SHIMMER_TABLE_SIZE) * 128.0 - 64.0
        # Pre-compute shimmer value using same formula as original
        shimmer_lut[i, j] = 0.85 + 0.105 * ti.sin(world_x * 0.35 + world_z * 0.45)

@ti.func
def get_shimmer_from_lut(world_x: ti.f32, world_z: ti.f32) -> ti.f32:
    """Fast shimmer lookup - replaces expensive sin() calculation"""
    # Map world coords to table indices with wrapping
    i = int((world_x + 64.0) / 128.0 * SHIMMER_TABLE_SIZE) % SHIMMER_TABLE_SIZE
    j = int((world_z + 64.0) / 128.0 * SHIMMER_TABLE_SIZE) % SHIMMER_TABLE_SIZE
    return shimmer_lut[i, j]

@ti.func
def get_voxel_color(voxel_type: ti.i32, world_x: ti.f32, world_z: ti.f32) -> ti.math.vec3:
    """Get color for voxel type with metallic sheen (GPU function)"""
    # Default color (steel/concrete)
    color = ti.math.vec3(0.7, 0.7, 0.75)

    # Steel - slightly blue tint
    if voxel_type == 1:  # STEEL
        color = ti.math.vec3(0.6, 0.65, 0.7)

    # Concrete - darker harmonious gray for arena floor
    elif voxel_type == 2:  # CONCRETE
        color = ti.math.vec3(0.41, 0.39, 0.37)

    # Molten voxels are bright orange (flowing metal)
    elif voxel_type == 3:  # MOLTEN
        color = ti.math.vec3(1.0, 0.4, 0.0)

    # Debris voxels are gray/brown (destroyed rubble)
    elif voxel_type == 4:  # DEBRIS
        color = ti.math.vec3(0.4, 0.35, 0.3)

    # Beetle voxels are blue - customizable via color picker
    elif voxel_type == 5:  # BEETLE_BLUE
        color = simulation.blue_body_color[None]

    # Second beetle is red - customizable via color picker
    elif voxel_type == 6:  # BEETLE_RED
        color = simulation.red_body_color[None]

    # Blue beetle legs - customizable via color picker
    elif voxel_type == 7:  # BEETLE_BLUE_LEGS
        color = simulation.blue_leg_color[None]

    # Red beetle legs - customizable via color picker
    elif voxel_type == 8:  # BEETLE_RED_LEGS
        color = simulation.red_leg_color[None]

    # Blue beetle leg tips - customizable via color picker
    elif voxel_type == 9:  # LEG_TIP_BLUE
        color = simulation.blue_leg_tip_color[None]

    # Red beetle leg tips - customizable via color picker
    elif voxel_type == 10:  # LEG_TIP_RED
        color = simulation.red_leg_tip_color[None]

    # Blue beetle racing stripe - customizable via color picker
    elif voxel_type == 11:  # BEETLE_BLUE_STRIPE
        color = simulation.blue_stripe_color[None]

    # Red beetle racing stripe - customizable via color picker
    elif voxel_type == 12:  # BEETLE_RED_STRIPE
        color = simulation.red_stripe_color[None]

    # Blue beetle horn prong tips - customizable via color picker
    elif voxel_type == 13:  # BEETLE_BLUE_HORN_TIP
        color = simulation.blue_horn_tip_color[None]

    # Red beetle horn prong tips - customizable via color picker
    elif voxel_type == 14:  # BEETLE_RED_HORN_TIP
        color = simulation.red_horn_tip_color[None]

    # Scorpion stinger tips - black/dark grey
    elif voxel_type == 15:  # STINGER_TIP_BLACK
        color = ti.math.vec3(0.15, 0.15, 0.15)  # Dark grey/black

    # Soccer ball - bright orange/yellow
    elif voxel_type == 16:  # BALL
        color = ti.math.vec3(1.0, 0.6, 0.0)  # Bright orange

    # Soccer ball stripe - darker for contrast
    elif voxel_type == 17:  # BALL_STRIPE
        color = ti.math.vec3(0.2, 0.1, 0.0)  # Dark brown/black

    # Shadow blob beneath airborne beetles (slightly darker than arena floor)
    elif voxel_type == 20:  # SHADOW
        color = ti.math.vec3(0.35, 0.33, 0.31)  # ~85% of arena floor color

    # OPTIMIZATION: Metallic sheen from lookup table instead of sin() (~8-12% speedup)
    if voxel_type >= 5 and voxel_type <= 15:  # All beetle parts
        shimmer = get_shimmer_from_lut(world_x, world_z)
        color *= shimmer

    return color

@ti.kernel
def extract_voxels(voxel_field: ti.template(), n_grid: ti.i32):
    """Extract non-empty voxels into render buffers (runs on GPU) - optimized with static bounding box"""
    count = 0

    # Static bounding box optimization: only scan active arena region
    # X/Z: 2-126 covers arena radius (30) + beetle reach + fully extended horns (32) = ±62 from center
    # Y: 1-100 covers falling (-32) to max velocity throws (+20) + scorpion tail reach (+23) with Y_OFFSET=33
    # Reduction: 2.1M voxels → 1.23M voxels (still ~40% fewer checks)
    # Use ti.static for compile-time constants (small performance boost)
    EMPTY = ti.static(0)
    DEBRIS = ti.static(4)

    for i, j, k in ti.ndrange((2, 126), (1, 100), (2, 126)):
        vtype = voxel_field[i, j, k]
        # Skip empty voxels and debris (debris handled by physics system)
        if vtype != EMPTY and vtype != DEBRIS:
            # Calculate world position
            world_pos = ti.math.vec3(
                float(i) - n_grid / 2.0,
                float(j),
                float(k) - n_grid / 2.0
            )
            # Get color with metallic sheen
            color = get_voxel_color(vtype, world_pos.x, world_pos.z)

            # Add to voxel buffer
            idx = ti.atomic_add(count, 1)
            if idx < MAX_VOXELS:
                voxel_positions[idx] = world_pos
                voxel_colors[idx] = color

    num_voxels[None] = min(count, MAX_VOXELS)

@ti.kernel
def extract_debris_particles():
    """Extract debris particles from physics simulation and add to voxel buffer (runs on GPU)"""
    # Get number of active debris particles from simulation
    debris_count = simulation.num_debris[None]

    # Get current voxel count to append debris after regular voxels
    voxel_count = num_voxels[None]

    # Copy debris particles to voxel render buffer (merged rendering for efficiency)
    for idx in range(debris_count):
        write_idx = voxel_count + idx
        if write_idx < MAX_VOXELS:  # Bounds check
            # Get position from physics system
            debris_pos = simulation.debris_pos[idx]
            voxel_positions[write_idx] = debris_pos

            # Get color directly from debris (RGB stored per particle for adaptive beetle colors)
            base_color = simulation.debris_material[idx]

            # Calculate alpha fade based on remaining lifetime
            # Use ratio-based fade that works for any particle lifetime
            lifetime = simulation.debris_lifetime[idx]

            # For short-lived particles (dust), fade over entire lifetime
            # For longer particles (explosions), start fade at 0.4s remaining
            if lifetime < 0.4:
                # Normalize to 0-1 range based on shorter of lifetime or 0.4s
                linear_fade = lifetime / 0.4  # 1.0 to 0.0
                # Gentler curve - just linear fade, no darkening
                alpha = ti.max(linear_fade, 0.0)
                # Fade toward arena dust color (not dark)
                fade_target = ti.math.vec3(0.5, 0.48, 0.45)  # Light dust color
                voxel_colors[write_idx] = base_color * alpha + fade_target * (1.0 - alpha)
            else:
                voxel_colors[write_idx] = base_color

    # Update total voxel count to include debris
    num_voxels[None] = min(voxel_count + debris_count, MAX_VOXELS)

@ti.kernel
def extract_projectiles():
    """Extract active projectiles from physics simulation (runs on GPU)"""
    # Count and copy active projectiles to render buffers
    write_idx = 0

    # Loop through all projectiles and copy active ones
    for idx in range(simulation.MAX_PROJECTILES):
        if simulation.projectile_active[idx] == 1:
            if write_idx < 10:  # Max 10 projectiles to render
                projectile_positions[write_idx] = simulation.projectile_pos[idx]
                # Cannonballs are bright yellow
                projectile_colors[write_idx] = ti.math.vec3(1.0, 1.0, 0.0)
                write_idx += 1

    # Set projectile count for rendering
    num_projectiles_render[None] = write_idx

@ti.kernel
def init_gradient_background():
    """Initialize gradient background (forest pit atmosphere)"""
    # Top color - forest canopy
    top_color = ti.math.vec3(0.22, 0.42, 0.32)
    # Bottom color - darker forest floor (creates depth)
    bottom_color = ti.math.vec3(0.10, 0.22, 0.14)

    # First triangle: bottom-left, bottom-right, top-left
    gradient_positions[0] = ti.math.vec2(0.0, 0.0)
    gradient_colors[0] = bottom_color

    gradient_positions[1] = ti.math.vec2(1.0, 0.0)
    gradient_colors[1] = bottom_color

    gradient_positions[2] = ti.math.vec2(0.0, 1.0)
    gradient_colors[2] = top_color

    # Second triangle: bottom-right, top-right, top-left
    gradient_positions[3] = ti.math.vec2(1.0, 0.0)
    gradient_colors[3] = bottom_color

    gradient_positions[4] = ti.math.vec2(1.0, 1.0)
    gradient_colors[4] = top_color

    gradient_positions[5] = ti.math.vec2(0.0, 1.0)
    gradient_colors[5] = top_color

class Camera:
    """Free-flying FPS camera - fly anywhere, look anywhere"""
    def __init__(self):
        import math

        # Camera position in world space (can be anywhere)
        self.pos_x = 0.0
        self.pos_y = 40.0  # Start at eye level above cathedral
        self.pos_z = -100.0  # Start back from cathedral

        # Camera orientation (Euler angles in degrees)
        self.yaw = 0.0  # Face forward toward cathedral
        self.pitch = 0.0  # Vertical rotation (up/down)

        # Movement speed
        self.move_speed = 30.0  # Units per second
        self.look_sensitivity = 0.15  # Mouse sensitivity

        # Mouse tracking
        self.last_mouse_x = None
        self.last_mouse_y = None
        self.mouse_captured = False

        # OPTIMIZATION: Cached dynamic light positions (~2-5% speedup)
        self.cached_yaw = None
        self.cached_key_light_pos = None
        self.cached_fill_light_pos = None
        self.cached_front_light_pos = (0, 40, 60)  # Default front light position

    def get_forward_vector(self):
        """Get forward direction based on yaw and pitch"""
        import math
        yaw_rad = math.radians(self.yaw)
        pitch_rad = math.radians(self.pitch)

        # Forward vector (where camera is looking)
        forward_x = math.cos(pitch_rad) * math.sin(yaw_rad)
        forward_y = math.sin(pitch_rad)
        forward_z = math.cos(pitch_rad) * math.cos(yaw_rad)

        return (forward_x, forward_y, forward_z)

    def get_right_vector(self):
        """Get right direction (perpendicular to forward on XZ plane)"""
        import math
        yaw_rad = math.radians(self.yaw)
        # Right is perpendicular to forward, ignoring pitch
        right_x = math.cos(yaw_rad)
        right_y = 0.0
        right_z = -math.sin(yaw_rad)
        return (right_x, right_y, right_z)

    def get_up_vector(self):
        """Get up direction - always world up (positive Y)"""
        return (0.0, 1.0, 0.0)

    def get_position(self):
        """Return current camera position"""
        return (self.pos_x, self.pos_y, self.pos_z)

    def get_lookat(self):
        """Point camera is looking at (position + forward)"""
        forward = self.get_forward_vector()
        return (
            self.pos_x + forward[0],
            self.pos_y + forward[1],
            self.pos_z + forward[2]
        )

def setup_camera(camera, scene):
    """Configure camera for the scene"""
    pos = camera.get_position()
    lookat = camera.get_lookat()
    up = camera.get_up_vector()

    # Create ti.ui.Camera with the correct parameters
    cam = ti.ui.Camera()
    cam.position(*pos)
    cam.lookat(*lookat)
    cam.up(*up)
    scene.set_camera(cam)

def render(camera, canvas, scene, voxel_field, n_grid, dynamic_lighting=True, spotlight_pos=None, spotlight_strength=0.55, base_light_brightness=1.0, front_light_strength=0.5):
    """
    Render voxels using Taichi GPU renderer

    Args:
        canvas: Taichi canvas
        scene: Taichi 3D scene
        voxel_field: Voxel data field
        n_grid: Grid size
        dynamic_lighting: If True, key light follows camera angle for cinematic effect
        spotlight_pos: (x, y, z) tuple for spotlight position above beetles (optional)
        spotlight_strength: Intensity of spotlight (default 0.55)
        base_light_brightness: Brightness multiplier for all non-spotlight lights (default 1.0)
        front_light_strength: Intensity of front camera light (default 0.5)
    """
    import math
    import time

    # === RENDER TIMING (for performance analysis) ===
    _t0 = time.perf_counter()

    # Draw gradient background before 3D scene (forest atmosphere)
    canvas.triangles(gradient_positions, per_vertex_color=gradient_colors)

    _t1 = time.perf_counter()

    # Extract voxels from grid (GPU operation)
    num_voxels[None] = 0  # Reset counters
    num_projectiles_render[None] = 0
    extract_voxels(voxel_field, n_grid)

    _t2 = time.perf_counter()

    # Extract debris particles from physics simulation
    extract_debris_particles()

    _t3 = time.perf_counter()

    # Extract projectiles from physics simulation
    extract_projectiles()

    _t4 = time.perf_counter()

    # Set up camera
    setup_camera(camera, scene)

    # Enhanced multi-point lighting setup with 360° coverage
    # Apply brightness multiplier to all non-spotlight lights
    b = base_light_brightness

    # Overhead light - main ambient coverage from above
    scene.point_light(pos=(0, 100, 0), color=(0.5 * b, 0.52 * b, 0.55 * b))

    # OPTIMIZATION: Key light with caching (~2-5% speedup - only recalculate on camera rotation)
    if dynamic_lighting:
        # Camera-relative key light: orbits with camera angle for consistent dramatic lighting
        if camera.yaw != camera.cached_yaw:
            # Recalculate only when camera rotates
            key_angle = math.radians(camera.yaw) + math.radians(45)
            key_distance = 100
            key_height = 70
            key_x = math.cos(key_angle) * key_distance
            key_z = math.sin(key_angle) * key_distance
            camera.cached_key_light_pos = (key_x, key_height, key_z)

            # Also recalculate fill light at same time
            fill_angle = math.radians(camera.yaw) + math.radians(-135)
            fill_x = math.cos(fill_angle) * 90
            fill_z = math.sin(fill_angle) * 90
            camera.cached_fill_light_pos = (fill_x, 60, fill_z)

            # Also recalculate front light (in front of camera)
            front_angle = math.radians(camera.yaw)
            front_distance = 60
            front_height = 40
            front_x = math.cos(front_angle) * front_distance
            front_z = math.sin(front_angle) * front_distance
            camera.cached_front_light_pos = (front_x, front_height, front_z)

            camera.cached_yaw = camera.yaw

        scene.point_light(pos=camera.cached_key_light_pos, color=(0.7 * b, 0.65 * b, 0.5 * b))
    else:
        scene.point_light(pos=(80, 70, -60), color=(0.7 * b, 0.65 * b, 0.5 * b))

    # OPTIMIZATION: Fill light with caching (uses cached values calculated with key light)
    if dynamic_lighting:
        scene.point_light(pos=camera.cached_fill_light_pos, color=(0.35 * b, 0.38 * b, 0.4 * b))
    else:
        scene.point_light(pos=(-60, 60, 70), color=(0.35 * b, 0.38 * b, 0.4 * b))

    # Front camera light - illuminates what the camera is looking at
    if dynamic_lighting and front_light_strength > 0:
        scene.point_light(pos=camera.cached_front_light_pos, color=(front_light_strength, front_light_strength, front_light_strength * 1.1))
    elif front_light_strength > 0:
        # Static front light when dynamic lighting is off
        scene.point_light(pos=(0, 40, 60), color=(front_light_strength, front_light_strength, front_light_strength * 1.1))

    # Spotlight above beetles - follows the action (NOT affected by brightness multiplier)
    if spotlight_pos is not None:
        spot_x, spot_y, spot_z = spotlight_pos
        scene.point_light(pos=(spot_x, spot_y, spot_z), color=(spotlight_strength * 1.15, spotlight_strength, spotlight_strength * 0.85))

    # Enhanced ambient light for better overall visibility
    scene.ambient_light((0.2 * b, 0.21 * b, 0.22 * b))

    _t5 = time.perf_counter()

    # Render all voxels (STEEL, CONCRETE, MOLTEN, DEBRIS) in single batched call
    # Debris particles merged into voxel buffer for better performance
    count = num_voxels[None]
    if count > 0:
        scene.particles(
            voxel_positions,
            radius=0.37,
            per_vertex_color=voxel_colors,
            index_count=count
        )

    _t6 = time.perf_counter()

    # Render projectiles (cannonballs) - larger, sphere-like
    projectile_count = num_projectiles_render[None]
    if projectile_count > 0:
        scene.particles(
            projectile_positions,
            radius=0.8,  # Larger than voxels (cannonball size)
            per_vertex_color=projectile_colors,
            index_count=projectile_count
        )

    _t7 = time.perf_counter()

    # === STORE RENDER TIMING FOR ANALYSIS ===
    # Store timing breakdown in module-level dict for access from main loop
    global _render_timing
    _render_timing = {
        'gradient': (_t1 - _t0) * 1000,
        'extract_voxels': (_t2 - _t1) * 1000,
        'extract_debris': (_t3 - _t2) * 1000,
        'extract_projectiles': (_t4 - _t3) * 1000,
        'lighting_setup': (_t5 - _t4) * 1000,
        'scene_particles': (_t6 - _t5) * 1000,
        'projectile_particles': (_t7 - _t6) * 1000,
        'voxel_count': count,
    }

    # NOTE: Don't render scene to canvas here - let caller add more elements first

# Module-level timing storage
_render_timing = {}

def get_render_timing():
    """Get the last frame's render timing breakdown"""
    return _render_timing

def handle_camera_controls(camera, window, dt):
    """Handle free-flying FPS camera controls"""
    import math

    # Movement speed
    speed = camera.move_speed * dt

    # Get camera vectors
    forward = camera.get_forward_vector()
    right = camera.get_right_vector()

    # WASD movement (relative to camera orientation, locked to horizontal plane)
    # Forward/backward only moves on XZ plane (no vertical component)
    forward_flat_x = forward[0]
    forward_flat_z = forward[2]
    flat_length = math.sqrt(forward_flat_x * forward_flat_x + forward_flat_z * forward_flat_z)
    if flat_length > 0.0001:
        forward_flat_x /= flat_length
        forward_flat_z /= flat_length

    if window.is_pressed('w'):
        # Move forward (horizontal only)
        camera.pos_x += forward_flat_x * speed
        camera.pos_z += forward_flat_z * speed

    if window.is_pressed('s'):
        # Move backward (horizontal only)
        camera.pos_x -= forward_flat_x * speed
        camera.pos_z -= forward_flat_z * speed

    if window.is_pressed('a'):
        # Strafe left
        camera.pos_x += right[0] * speed
        camera.pos_z += right[2] * speed

    if window.is_pressed('d'):
        # Strafe right
        camera.pos_x -= right[0] * speed
        camera.pos_z -= right[2] * speed

    # Up/Down movement (world space) - Q and E keys
    if window.is_pressed('q'):
        camera.pos_y += speed  # Move up
    if window.is_pressed('e'):
        camera.pos_y -= speed  # Move down

    # Arrow keys for camera rotation
    turn_speed = 120.0 * dt  # Degrees per second

    if window.is_pressed(ti.ui.LEFT):
        camera.yaw += turn_speed  # Look left
    if window.is_pressed(ti.ui.RIGHT):
        camera.yaw -= turn_speed  # Look right
    if window.is_pressed(ti.ui.UP):
        camera.pitch += turn_speed  # Look up
        camera.pitch = min(89.0, camera.pitch)  # Clamp
    if window.is_pressed(ti.ui.DOWN):
        camera.pitch -= turn_speed  # Look down
        camera.pitch = max(-89.0, camera.pitch)  # Clamp

    # Wrap yaw
    camera.yaw = camera.yaw % 360.0

    # Speed adjustment
    if window.is_pressed('=') or window.is_pressed('+'):
        camera.move_speed = min(100.0, camera.move_speed * 1.05)
    if window.is_pressed('-') or window.is_pressed('_'):
        camera.move_speed = max(5.0, camera.move_speed * 0.95)

def handle_mouse_look(camera, window):
    """Handle mouse look (called separately to track mouse delta)"""
    # Get current mouse position
    mouse_x, mouse_y = window.get_cursor_pos()

    # Initialize on first frame
    if camera.last_mouse_x is None:
        camera.last_mouse_x = mouse_x
        camera.last_mouse_y = mouse_y
        return

    # Calculate mouse delta
    delta_x = mouse_x - camera.last_mouse_x
    delta_y = mouse_y - camera.last_mouse_y

    # Update camera orientation
    camera.yaw += delta_x * camera.look_sensitivity
    camera.pitch -= delta_y * camera.look_sensitivity  # Inverted Y

    # Clamp pitch to prevent flipping
    camera.pitch = max(-89.0, min(89.0, camera.pitch))

    # Wrap yaw
    camera.yaw = camera.yaw % 360.0

    # Store current mouse position for next frame
    camera.last_mouse_x = mouse_x
    camera.last_mouse_y = mouse_y
