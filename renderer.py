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
# Separate fields for debris (so we can render at different size)
num_debris = ti.field(dtype=ti.i32, shape=())
debris_positions = ti.Vector.field(3, dtype=ti.f32, shape=MAX_VOXELS)
debris_colors = ti.Vector.field(4, dtype=ti.f32, shape=MAX_VOXELS)  # RGBA for alpha transparency

# Projectile rendering (cannonballs)
num_projectiles_render = ti.field(dtype=ti.i32, shape=())
projectile_positions = ti.Vector.field(3, dtype=ti.f32, shape=10)  # Max 10 projectiles
projectile_colors = ti.Vector.field(3, dtype=ti.f32, shape=10)

# Gradient background (2 triangles forming full-screen quad)
gradient_positions = ti.Vector.field(2, dtype=ti.f32, shape=6)
gradient_colors = ti.Vector.field(3, dtype=ti.f32, shape=6)

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

    # Beetle voxels are blue - desaturated for richer depth
    elif voxel_type == 5:  # BEETLE_BLUE
        color = ti.math.vec3(0.25, 0.55, 0.95)

    # Second beetle is red - desaturated for richer depth
    elif voxel_type == 6:  # BEETLE_RED
        color = ti.math.vec3(0.95, 0.25, 0.15)

    # Blue beetle legs - lighter cyan/blue
    elif voxel_type == 7:  # BEETLE_BLUE_LEGS
        color = ti.math.vec3(0.4, 0.7, 1.0)

    # Red beetle legs - lighter orange/red
    elif voxel_type == 8:  # BEETLE_RED_LEGS
        color = ti.math.vec3(1.0, 0.5, 0.3)

    # Blue beetle leg tips - dark blue with slight tint
    elif voxel_type == 9:  # LEG_TIP_BLUE
        color = ti.math.vec3(0.0, 0.0, 0.3)  # Very dark blue

    # Red beetle leg tips - dark red with slight tint
    elif voxel_type == 10:  # LEG_TIP_RED
        color = ti.math.vec3(0.3, 0.0, 0.0)  # Very dark red

    # Blue beetle racing stripe - bright cyan/white
    elif voxel_type == 11:  # BEETLE_BLUE_STRIPE
        color = ti.math.vec3(0.6, 0.9, 1.0)  # Bright cyan

    # Red beetle racing stripe - rich gold/bronze
    elif voxel_type == 12:  # BEETLE_RED_STRIPE
        color = ti.math.vec3(0.85, 0.65, 0.2)  # Rich gold/bronze

    # Blue beetle horn prong tips - bright electric blue (bold but harmonious)
    elif voxel_type == 13:  # BEETLE_BLUE_HORN_TIP
        color = ti.math.vec3(0.4, 0.75, 1.0)  # Bright electric blue

    # Red beetle horn prong tips - deep crimson/burgundy (harmonious)
    elif voxel_type == 14:  # BEETLE_RED_HORN_TIP
        color = ti.math.vec3(0.4, 0.1, 0.1)  # Deep crimson

    # Scorpion stinger tips - black/dark grey
    elif voxel_type == 15:  # STINGER_TIP_BLACK
        color = ti.math.vec3(0.15, 0.15, 0.15)  # Dark grey/black

    # Add metallic sheen to beetle voxels only (performance optimized)
    if voxel_type >= 5 and voxel_type <= 15:  # All beetle parts
        # Subtle metallic shimmer using fast hardware-accelerated sin
        shimmer = 0.85 + 0.105 * ti.sin(world_x * 0.35 + world_z * 0.45)
        color *= shimmer

    return color

@ti.kernel
def extract_voxels(voxel_field: ti.template(), n_grid: ti.i32):
    """Extract non-empty voxels into render buffers (runs on GPU) - optimized with static bounding box"""
    count = 0
    debris_count = 0

    # Static bounding box optimization: only scan active arena region
    # X/Z: 2-126 covers arena radius (30) + beetle reach + fully extended horns (32) = ±62 from center
    # Y: 1-100 covers falling (-32) to max velocity throws (+20) + scorpion tail reach (+23) with Y_OFFSET=33
    # Reduction: 2.1M voxels → 1.23M voxels (still ~40% fewer checks)
    for i, j, k in ti.ndrange((2, 126), (1, 100), (2, 126)):
        vtype = voxel_field[i, j, k]
        if vtype != 0:
            # Calculate world position
            world_pos = ti.math.vec3(
                float(i) - n_grid / 2.0,
                float(j),
                float(k) - n_grid / 2.0
            )
            # Get color with metallic sheen
            color = get_voxel_color(vtype, world_pos.x, world_pos.z)

            # Separate debris from normal voxels
            if vtype == 4:  # DEBRIS
                idx = ti.atomic_add(debris_count, 1)
                if idx < MAX_VOXELS:
                    debris_positions[idx] = world_pos
                    debris_colors[idx] = ti.Vector([color.x, color.y, color.z, 1.0])  # Add alpha=1.0
            else:  # Normal voxels (STEEL, CONCRETE, MOLTEN)
                idx = ti.atomic_add(count, 1)
                if idx < MAX_VOXELS:
                    voxel_positions[idx] = world_pos
                    voxel_colors[idx] = color

    num_voxels[None] = min(count, MAX_VOXELS)
    num_debris[None] = min(debris_count, MAX_VOXELS)

@ti.kernel
def extract_debris_particles():
    """Extract debris particles from physics simulation (runs on GPU)"""
    # Get number of active debris particles from simulation
    count = simulation.num_debris[None]

    # Copy debris particles to render buffers (count is already bounded by MAX_DEBRIS)
    for idx in range(count):
        # Get position from physics system
        debris_pos = simulation.debris_pos[idx]
        debris_positions[idx] = debris_pos

        # Get color based on material type with metallic sheen
        material_type = simulation.debris_material[idx]
        base_color = get_voxel_color(material_type, debris_pos.x, debris_pos.z)

        # Calculate alpha transparency fade based on remaining lifetime
        # Fade to transparent instead of darker for smooth disappearance
        lifetime = simulation.debris_lifetime[idx]
        fade_start = 0.25  # Start fading when 0.25 seconds left (optimized for 0.4-0.7s lifetime)

        # Calculate alpha (1.0 = opaque, 0.0 = transparent) and set RGBA
        if lifetime < fade_start:
            alpha_value = lifetime / fade_start  # Linear fade from 1.0 to 0.0
            debris_colors[idx] = ti.Vector([base_color.x, base_color.y, base_color.z, alpha_value])
        else:
            debris_colors[idx] = ti.Vector([base_color.x, base_color.y, base_color.z, 1.0])

    # Set debris count for rendering
    num_debris[None] = count

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
    """Initialize gradient background (forest canopy at top, forest floor at bottom)"""
    # Top color - deeper forest green (canopy)
    top_color = ti.math.vec3(0.25, 0.45, 0.35)
    # Bottom color - much darker forest green (floor)
    bottom_color = ti.math.vec3(0.12, 0.25, 0.16)

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

def render(camera, canvas, scene, voxel_field, n_grid):
    """
    Render voxels using Taichi GPU renderer

    Args:
        canvas: Taichi canvas
        scene: Taichi 3D scene
        voxel_field: Voxel data field
        n_grid: Grid size
    """
    # Draw gradient background before 3D scene (forest atmosphere)
    canvas.triangles(gradient_positions, per_vertex_color=gradient_colors)

    # Extract voxels from grid (GPU operation)
    num_voxels[None] = 0  # Reset counters
    num_debris[None] = 0
    num_projectiles_render[None] = 0
    extract_voxels(voxel_field, n_grid)

    # Extract debris particles from physics simulation
    extract_debris_particles()

    # Extract projectiles from physics simulation
    extract_projectiles()

    # Set up camera
    setup_camera(camera, scene)

    # Forest-themed lighting setup with rim light for depth
    # Overhead light - cooler overhead (moonlight feel)
    scene.point_light(pos=(0, 100, 0), color=(0.9, 0.95, 1.0))
    # Key light - brighter warm sunbeam for contrast
    scene.point_light(pos=(80, 70, -60), color=(1.4, 1.1, 0.7))
    # Rim light - cool backlight for depth separation
    scene.point_light(pos=(-60, 50, 70), color=(0.3, 0.4, 0.6))
    # Lower ambient light for dramatic depth
    scene.ambient_light((0.15, 0.18, 0.16))

    # Render normal voxels (STEEL, CONCRETE, MOLTEN) - balanced radius for smooth yet defined look
    count = num_voxels[None]
    if count > 0:
        scene.particles(
            voxel_positions,
            radius=0.37,
            per_vertex_color=voxel_colors,
            index_count=count
        )

    # Render debris voxels - visible size for explosions (with smooth color fade)
    debris_count = num_debris[None]
    if debris_count > 0:
        scene.particles(
            debris_positions,
            radius=0.35,  # Fixed radius, fade via color intensity
            per_vertex_color=debris_colors,
            index_count=debris_count
        )

    # Render projectiles (cannonballs) - larger, sphere-like
    projectile_count = num_projectiles_render[None]
    if projectile_count > 0:
        scene.particles(
            projectile_positions,
            radius=0.8,  # Larger than voxels (cannonball size)
            per_vertex_color=projectile_colors,
            index_count=projectile_count
        )

    # NOTE: Don't render scene to canvas here - let caller add more elements first

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
