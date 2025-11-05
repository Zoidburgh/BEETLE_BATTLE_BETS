"""
PHASE 1: Smooth Physics-Based Beetle Combat
Tank controls with rotation, momentum, and pushing

Beetle 1: FTHG (F=forward, G=back, H=turn left, T=turn right)
Beetle 2: LKJI (L=forward, J=back, K=turn left, I=turn right)
"""

import taichi as ti
import simulation
import renderer
import time
import math

# Physics constants (tunable for feel)
BEETLE_MASS = 10.0
BEETLE_RADIUS = 4.0
MOVE_FORCE = 200.0  # High force for responsive feel
TURN_TORQUE = 5.0  # Not used in direct movement
FRICTION = 0.75  # Much stronger friction - stops quickly
ANGULAR_FRICTION = 0.90
MAX_SPEED = 8.0  # Lower max speed for control
MAX_ANGULAR_SPEED = 3.0
ARENA_RADIUS = 35.0

# Beetle physics state (continuous, not grid-based)
class Beetle:
    def __init__(self, x, z, rotation, color_type):
        self.x = x  # World position
        self.z = z
        self.vx = 0.0  # Velocity
        self.vz = 0.0
        self.rotation = rotation  # Radians
        self.angular_vel = 0.0
        self.mass = BEETLE_MASS
        self.radius = BEETLE_RADIUS
        self.color_type = color_type  # BEETLE_BLUE or BEETLE_RED

    def apply_force(self, fx, fz):
        """Apply force (adds to velocity)"""
        ax = fx / self.mass
        az = fz / self.mass
        self.vx += ax
        self.vz += az

    def apply_torque(self, torque):
        """Apply turning torque"""
        self.angular_vel += torque

    def update_physics(self, dt):
        """Update position, apply friction"""
        # Apply friction
        self.vx *= FRICTION ** dt
        self.vz *= FRICTION ** dt
        self.angular_vel *= ANGULAR_FRICTION ** dt

        # Clamp speeds
        speed = math.sqrt(self.vx**2 + self.vz**2)
        if speed > MAX_SPEED:
            self.vx = (self.vx / speed) * MAX_SPEED
            self.vz = (self.vz / speed) * MAX_SPEED

        if abs(self.angular_vel) > MAX_ANGULAR_SPEED:
            self.angular_vel = MAX_ANGULAR_SPEED if self.angular_vel > 0 else -MAX_ANGULAR_SPEED

        # Update position
        self.x += self.vx * dt
        self.z += self.vz * dt

        # Update rotation
        self.rotation += self.angular_vel * dt

        # Normalize rotation to [0, 2π]
        while self.rotation > math.pi * 2:
            self.rotation -= math.pi * 2
        while self.rotation < 0:
            self.rotation += math.pi * 2

    def arena_collision(self):
        """Bounce off arena walls"""
        dist = math.sqrt(self.x**2 + self.z**2)
        if dist > ARENA_RADIUS - self.radius:
            # Push back
            angle = math.atan2(self.z, self.x)
            self.x = math.cos(angle) * (ARENA_RADIUS - self.radius)
            self.z = math.sin(angle) * (ARENA_RADIUS - self.radius)

            # Reflect velocity
            normal_x = self.x / dist
            normal_z = self.z / dist
            dot = self.vx * normal_x + self.vz * normal_z
            self.vx -= 2 * dot * normal_x
            self.vz -= 2 * dot * normal_z

            # Dampen
            self.vx *= 0.5
            self.vz *= 0.5

# Create beetles
beetle1 = Beetle(-15.0, 0.0, 0.0, simulation.BEETLE_BLUE)
beetle2 = Beetle(15.0, 0.0, math.pi, simulation.BEETLE_RED)

@ti.kernel
def clear_beetles():
    """Remove all beetle voxels"""
    for i, j, k in ti.ndrange(simulation.n_grid, simulation.n_grid, simulation.n_grid):
        if simulation.voxel_type[i, j, k] == simulation.BEETLE_BLUE or \
           simulation.voxel_type[i, j, k] == simulation.BEETLE_RED:
            simulation.voxel_type[i, j, k] = simulation.EMPTY

@ti.kernel
def place_beetle(world_x: ti.f32, world_z: ti.f32, rotation: ti.f32, color_type: ti.i32):
    """Place rotated beetle at world position"""
    # Convert world to grid
    center_x = int(world_x + simulation.n_grid / 2.0)
    center_z = int(world_z + simulation.n_grid / 2.0)

    # Place rectangular beetle (will rotate later - for now just place it)
    # TODO: Rotate voxels based on rotation angle
    for dx in range(-3, 4):
        for dy in range(0, 3):
            for dz in range(-2, 3):
                grid_x = center_x + dx
                grid_y = 2 + dy
                grid_z = center_z + dz
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

def beetle_collision(b1, b2):
    """Check and resolve collision between two beetles"""
    dx = b1.x - b2.x
    dz = b1.z - b2.z
    dist = math.sqrt(dx**2 + dz**2)
    min_dist = b1.radius + b2.radius

    if dist < min_dist and dist > 0.001:
        # Collision!
        normal_x = dx / dist
        normal_z = dz / dist

        # Separate beetles
        overlap = min_dist - dist
        b1.x += normal_x * overlap * 0.5
        b1.z += normal_z * overlap * 0.5
        b2.x -= normal_x * overlap * 0.5
        b2.z -= normal_z * overlap * 0.5

        # Calculate relative velocity
        rel_vx = b1.vx - b2.vx
        rel_vz = b1.vz - b2.vz
        vel_along_normal = rel_vx * normal_x + rel_vz * normal_z

        # Only resolve if moving toward each other
        if vel_along_normal < 0:
            # Impulse (elastic collision with restitution)
            restitution = 0.6
            impulse = -(1 + restitution) * vel_along_normal / (1/b1.mass + 1/b2.mass)

            # Apply impulse
            b1.vx += impulse * normal_x / b1.mass
            b1.vz += impulse * normal_z / b1.mass
            b2.vx -= impulse * normal_x / b2.mass
            b2.vz -= impulse * normal_z / b2.mass

            print(f"COLLISION! Impulse: {impulse:.1f}")

# Window
window = ti.ui.Window("Physics Beetles", (1920, 1080), vsync=True)
canvas = window.get_canvas()
scene = window.get_scene()

camera = renderer.Camera()
camera.pos_x = 0.0
camera.pos_y = 60.0
camera.pos_z = 0.0
camera.pitch = -70.0
camera.yaw = 0.0

print("\n=== PHYSICS BEETLE TEST ===")
print("BEETLE 1 (Blue):")
print("  T - North")
print("  G - South")
print("  H - West")
print("  F - East")
print("")
print("BEETLE 2 (Red):")
print("  I - North")
print("  K - South")
print("  J - West")
print("  L - East")
print("")
print("Try pushing beetles together!")
print("Camera: WASD/Mouse/Q/E")
print("="*40 + "\n")

last_time = time.time()

while window.running:
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time
    dt = min(dt, 0.05)  # Cap at 50ms

    # Camera
    renderer.handle_camera_controls(camera, window, dt)
    renderer.handle_mouse_look(camera, window)

    # === BEETLE 1 CONTROLS (FTHG) - Direct movement ===
    move1_x = 0.0
    move1_z = 0.0

    if window.is_pressed('t'):
        move1_z -= 1.0  # North
    if window.is_pressed('g'):
        move1_z += 1.0  # South
    if window.is_pressed('h'):
        move1_x -= 1.0  # West
    if window.is_pressed('f'):
        move1_x += 1.0  # East

    # Apply direct movement forces
    if move1_x != 0 or move1_z != 0:
        # Normalize diagonal movement
        length = math.sqrt(move1_x**2 + move1_z**2)
        move1_x /= length
        move1_z /= length
        beetle1.apply_force(move1_x * MOVE_FORCE * dt, move1_z * MOVE_FORCE * dt)

    # === BEETLE 2 CONTROLS (LKJI) - Direct movement ===
    move2_x = 0.0
    move2_z = 0.0

    if window.is_pressed('i'):
        move2_z -= 1.0  # North
    if window.is_pressed('k'):
        move2_z += 1.0  # South
    if window.is_pressed('j'):
        move2_x -= 1.0  # West
    if window.is_pressed('l'):
        move2_x += 1.0  # East

    # Apply direct movement forces
    if move2_x != 0 or move2_z != 0:
        # Normalize diagonal movement
        length = math.sqrt(move2_x**2 + move2_z**2)
        move2_x /= length
        move2_z /= length
        beetle2.apply_force(move2_x * MOVE_FORCE * dt, move2_z * MOVE_FORCE * dt)

    # Physics update
    beetle1.update_physics(dt)
    beetle2.update_physics(dt)

    # Arena boundaries
    beetle1.arena_collision()
    beetle2.arena_collision()

    # Beetle-beetle collision
    beetle_collision(beetle1, beetle2)

    # Render
    clear_beetles()
    place_beetle(beetle1.x, beetle1.z, beetle1.rotation, beetle1.color_type)
    place_beetle(beetle2.x, beetle2.z, beetle2.rotation, beetle2.color_type)

    canvas.set_background_color((0.05, 0.05, 0.08))
    renderer.render(camera, canvas, scene, simulation.voxel_type, simulation.n_grid)
    canvas.scene(scene)

    # HUD
    window.GUI.begin("Physics Beetles", 0.01, 0.01, 0.35, 0.25)
    window.GUI.text("BLUE BEETLE (TGHF)")
    window.GUI.text(f"  Pos: ({beetle1.x:.1f}, {beetle1.z:.1f})")
    window.GUI.text(f"  Speed: {math.sqrt(beetle1.vx**2 + beetle1.vz**2):.1f}")
    window.GUI.text("")
    window.GUI.text("RED BEETLE (IKJL)")
    window.GUI.text(f"  Pos: ({beetle2.x:.1f}, {beetle2.z:.1f})")
    window.GUI.text(f"  Speed: {math.sqrt(beetle2.vx**2 + beetle2.vz**2):.1f}")
    window.GUI.text("")
    dx = beetle1.x - beetle2.x
    dz = beetle1.z - beetle2.z
    distance = math.sqrt(dx**2 + dz**2)
    window.GUI.text(f"Distance: {distance:.1f}")
    window.GUI.end()

    window.show()

print("Done!")
