"""
Beetle combat with rotation and pushing physics
TGHF for blue beetle, IKJL for red beetle
"""

import taichi as ti
import simulation
import renderer
import time
import math
import random
from collections import deque

# ============================================================================
# PERFORMANCE MONITORING SYSTEM
# ============================================================================
class PerformanceMonitor:
    """
    Real-time performance monitoring for frame time breakdown.
    Tracks timing for each phase of the game loop to identify bottlenecks.
    """
    def __init__(self, history_size=120):
        """Initialize with rolling history (default 120 frames = 2 seconds at 60fps)"""
        self.history_size = history_size

        # Timing categories
        self.categories = [
            'frame_total',      # Total frame time
            'camera',           # Camera updates
            'physics',          # Physics loop (all iterations)
            'animation',        # Walk animation updates
            'voxel_clear',      # Clearing dirty voxels
            'beetle_render',    # Placing beetle voxels
            'ball_render',      # Ball rendering
            'scene_render',     # renderer.render() call
            'gui',              # GUI overlay
        ]

        # Rolling history for each category (in milliseconds)
        self.history = {cat: deque(maxlen=history_size) for cat in self.categories}

        # Current frame timing (accumulator for multi-call sections)
        self.current = {cat: 0.0 for cat in self.categories}

        # Start times for active timers
        self._start_times = {}

        # Stats
        self.frame_count = 0
        self.physics_iterations_per_frame = deque(maxlen=history_size)

        # Enable/disable display
        self.show_stats = True
        self.show_detailed = False  # Show per-category breakdown

    def start(self, category):
        """Start timing a category"""
        self._start_times[category] = time.perf_counter()

    def stop(self, category):
        """Stop timing and accumulate to current frame"""
        if category in self._start_times:
            elapsed = (time.perf_counter() - self._start_times[category]) * 1000  # Convert to ms
            self.current[category] += elapsed
            del self._start_times[category]

    def end_frame(self, physics_iterations=0):
        """End frame: store current timings to history and reset"""
        for cat in self.categories:
            self.history[cat].append(self.current[cat])
            self.current[cat] = 0.0
        self.physics_iterations_per_frame.append(physics_iterations)
        self.frame_count += 1

    def get_avg(self, category):
        """Get average time for category (ms)"""
        h = self.history[category]
        return sum(h) / len(h) if h else 0.0

    def get_max(self, category):
        """Get max time for category (ms)"""
        h = self.history[category]
        return max(h) if h else 0.0

    def get_min(self, category):
        """Get min time for category (ms)"""
        h = self.history[category]
        return min(h) if h else 0.0

    def get_last(self, category):
        """Get last frame time for category (ms)"""
        h = self.history[category]
        return h[-1] if h else 0.0

    def get_avg_physics_iterations(self):
        """Get average physics iterations per frame"""
        return sum(self.physics_iterations_per_frame) / len(self.physics_iterations_per_frame) if self.physics_iterations_per_frame else 0

    def get_summary_string(self):
        """Get a compact summary string for GUI display"""
        total = self.get_avg('frame_total')
        physics = self.get_avg('physics')
        render = self.get_avg('beetle_render') + self.get_avg('ball_render') + self.get_avg('scene_render')
        gui = self.get_avg('gui')
        other = total - physics - render - gui

        return (f"Frame: {total:.1f}ms | "
                f"Physics: {physics:.1f}ms | "
                f"Render: {render:.1f}ms | "
                f"GUI: {gui:.1f}ms")

    def get_detailed_breakdown(self):
        """Get detailed breakdown as list of strings"""
        lines = []
        lines.append("--- Performance Breakdown (avg ms) ---")

        # Sort by average time descending
        sorted_cats = sorted(self.categories, key=lambda c: self.get_avg(c), reverse=True)

        for cat in sorted_cats:
            avg = self.get_avg(cat)
            last = self.get_last(cat)
            max_val = self.get_max(cat)
            if avg > 0.01:  # Only show if measurable
                lines.append(f"  {cat}: {avg:.2f}ms (last: {last:.2f}, max: {max_val:.2f})")

        # Physics iterations
        avg_iters = self.get_avg_physics_iterations()
        lines.append(f"  physics_iters/frame: {avg_iters:.1f}")

        return lines

# Global performance monitor instance
perf_monitor = PerformanceMonitor()

# Physics breakdown timing (similar to renderer breakdown)
_physics_timing = {
    'input_controls': 0.0,
    'beetle_physics': 0.0,
    'ball_physics': 0.0,
    'debris_particles': 0.0,
    'death_explosions': 0.0,
    'respawn_timers': 0.0,
    'floor_collision': 0.0,
    'beetle_collision': 0.0,
}

def get_physics_timing():
    """Get last frame's physics timing breakdown"""
    return _physics_timing

# OPTIMIZATION: Pre-computed constants to avoid repeated calculations
TWO_PI = 2.0 * math.pi  # Avoids ~20 multiplications per frame
PI_HALF = math.pi * 0.5
PI_ONE_HALF = math.pi * 1.5

# Shadow blob constants (for airborne beetles)
SHADOW_HEIGHT_THRESHOLD = 3.0  # Only show shadow when Y > 3
SHADOW_MAX_HEIGHT = 25.0  # Shadow max size at this height
SHADOW_BASE_RADIUS = 1  # Base radius in voxels (close to ground)
SHADOW_MAX_RADIUS = 8  # Max radius at max height

# Physics constants
BEETLE_RADIUS = 16.0  # Back to original scale - lean and mean
MOVE_FORCE = 120.0  # Forward movement force
BACKWARD_MOVE_FORCE = 85.0  # Backward movement force (slower retreat)
FRICTION = 0.88  # Slightly higher friction
MAX_SPEED = 7.0  # Forward top speed
BACKWARD_MAX_SPEED = 5.0  # Backward top speed (slower)
ROTATION_SPEED = 2.5  # Rotation speed for controlled turning
ANGULAR_FRICTION = 0.65  # How quickly spin slows down (lower value = more friction, less spin)
MAX_ANGULAR_SPEED = 8.0  # Max spin speed (radians/sec) from collision impacts
TORQUE_MULTIPLIER = 1.95  # Rotational forces on collision
RESTITUTION = 0.025  # Bounce coefficient (0 = no bounce, 1 = full bounce)
IMPULSE_MULTIPLIER = 0.7  # Linear momentum transfer
MOMENT_OF_INERTIA_FACTOR = 1.15  # Resistance to rotation
ARENA_RADIUS = 32.0  # 25% smaller for closer combat
MAX_VERTICAL_VELOCITY = 20.0  # Maximum upward velocity to prevent outlier launches
AIR_RESISTANCE = 0.02  # Air drag coefficient for vertical movement

# Horn control constants
HORN_TILT_SPEED = 2.0  # Radians per second
HORN_DEFAULT_PITCH = math.radians(10)  # Start at +10 degrees (raised) - for rhino
HORN_DEFAULT_PITCH_STAG = math.radians(18)  # Start at +18 degrees (raised) - for stag pincers
HORN_DEFAULT_PITCH_HERCULES = math.radians(15)  # Start at +15 degrees (jaws slightly open) - for hercules
HORN_DEFAULT_PITCH_SCORPION = math.radians(20)  # Start at +20 degrees (raised) - for scorpion claws
HORN_DEFAULT_PITCH_ATLAS = math.radians(-33)  # Start at -33 degrees (angled down toward ground) - for atlas beetle
HORN_MAX_PITCH = math.radians(40)  # +40 degrees vertical (up) - increased by 10° for stag pincers
HORN_MIN_PITCH = math.radians(5)  # +5 degrees vertical (down) - reduced downward by 15° total for stag pincers
# Rhino-specific limits (shifted 5° higher to keep horn from going too low)
HORN_MAX_PITCH_RHINO = math.radians(35)  # +35 degrees vertical (up) - 5° more than default 30°
HORN_MIN_PITCH_RHINO = math.radians(-5)  # -5 degrees vertical (down) - 5° less downward than default -10°
# Hercules-specific limits (±15° range from default, shifted up to prevent ground clipping)
HORN_MAX_PITCH_HERCULES = math.radians(35)  # +35 degrees (jaws fully open)
HORN_MIN_PITCH_HERCULES = math.radians(2)   # +2 degrees (jaws fully closed)
# Atlas-specific limits (optimized for scoop-and-lift combat mechanics)
HORN_MAX_PITCH_ATLAS = math.radians(20)  # +20 degrees (full upward lift)
HORN_MIN_PITCH_ATLAS = math.radians(-40)   # -40 degrees (angled down toward ground)

# Horn yaw control (Phase 2 - pincer spread for stag, yaw for rhino)
HORN_YAW_SPEED = 1.0  # Radians per second (50% slower for less clipping)
# Default yaw limits (for beetles with symmetric horn movement)
HORN_MAX_YAW = math.radians(20)  # +20 degrees horizontal
HORN_MIN_YAW = math.radians(-20)  # -20 degrees horizontal
# Stag-specific yaw limits (shifted +15° so pincers start more open)
HORN_DEFAULT_YAW_STAG = math.radians(15)  # Stag pincers start 15° open on each side (30° total spread)
HORN_MAX_YAW_STAG = math.radians(35)  # +35 degrees horizontal (stag pincers can open more)
HORN_MIN_YAW_STAG = math.radians(-5)  # -5 degrees horizontal (stag pincers can close less)
HORN_YAW_MIN_DISTANCE = 0.5  # Minimum distance between horn tips (voxels) to allow yaw rotation
HORN_PITCH_MIN_DISTANCE = 0.5  # Minimum distance between horn tips (voxels) to allow pitch rotation

# OPTIMIZATION: Horn type ID mapping and pitch/yaw limit lookup tables
# Eliminates string comparisons in the physics loop (120 checks/sec -> integer lookup)
HORN_TYPE_IDS = {"rhino": 0, "stag": 1, "hercules": 2, "scorpion": 3, "atlas": 4}
# Pitch limits: (max_pitch, min_pitch) indexed by horn_type_id
HORN_PITCH_LIMITS = [
    (HORN_MAX_PITCH_RHINO, HORN_MIN_PITCH_RHINO),       # 0: rhino
    (HORN_MAX_PITCH, HORN_MIN_PITCH),                   # 1: stag
    (HORN_MAX_PITCH_HERCULES, HORN_MIN_PITCH_HERCULES), # 2: hercules
    (HORN_MAX_PITCH, HORN_MIN_PITCH),                   # 3: scorpion (uses default)
    (HORN_MAX_PITCH_ATLAS, HORN_MIN_PITCH_ATLAS),       # 4: atlas
]
# Yaw limits: (max_yaw, min_yaw) indexed by horn_type_id
HORN_YAW_LIMITS = [
    (HORN_MAX_YAW, HORN_MIN_YAW),           # 0: rhino
    (HORN_MAX_YAW_STAG, HORN_MIN_YAW_STAG), # 1: stag
    (HORN_MAX_YAW, HORN_MIN_YAW),           # 2: hercules
    (HORN_MAX_YAW, HORN_MIN_YAW),           # 3: scorpion
    (HORN_MAX_YAW, HORN_MIN_YAW),           # 4: atlas
]

# Edge tipping constants
EDGE_TIPPING_STRENGTH = 0.45  # Force multiplier per over-edge voxel
ARENA_CENTER_X = 64.0  # Arena center X coordinate
ARENA_CENTER_Z = 64.0  # Arena center Z coordinate
ARENA_EDGE_RADIUS = 30.0  # Arena edge radius (voxels beyond this trigger tipping)

# Fixed timestep physics constants
PHYSICS_TIMESTEP = 1.0 / 60.0  # 60 Hz physics update rate (16.67ms per step)
MAX_TIMESTEP_ACCUMULATOR = 0.25  # Max accumulator to prevent spiral of death

# Horn rotation damping constants (Phase 2: Horn Clipping Prevention)
HORN_DAMPING_INTO = 0.80      # Damping when rotating into collision (0.80 = 20% speed, high resistance when pushing into collision)
HORN_DAMPING_AWAY = 0.15      # Damping when rotating away from collision (0.15 = 85% speed)
HORN_DAMPING_NEUTRAL = 0.30   # Damping when not actively rotating
HORN_DAMPING_SMOOTHING = 0.3  # Smoothing rate for damping changes (0-1, higher = faster)
HORN_DAMPING_DECAY_RATE = 5.0 # Damping decay rate when no contact (per second)
HORN_ENGAGEMENT_CONTACT_THRESHOLD = 20.0  # Voxel count for full engagement
HORN_ENGAGEMENT_HEIGHT_WEIGHT = 0.4  # Weight of height vs contact count (0-1)

# Collision normal smoothing (reduces jitter during sustained collisions)
COLLISION_NORMAL_SMOOTHING = 0.3  # 30% blend per frame (0-1, higher = faster response, lower = smoother)

# Body rotation damping constants (Phase 3: Directional Rotation Prevention)
BODY_ROTATION_DAMPING_STRENGTH = 0.95  # 95% damping (5% speed) when rotating into spin direction
BODY_ROTATION_DAMPING_DECAY = 2.0      # Decay rate per second (~0.5s duration at full strength)

# Rendering offset - allows beetles to be visible while falling below arena
RENDER_Y_OFFSET = 33.0  # Shift voxel rendering up so Y=0 maps to grid Y=33 (128 grid, center at 64)

# Ball physics constants (tunable via sliders for smooth rolling/bouncing)
# These will be overridden by slider values in the main loop
BALL_SEPARATION_FORCE = 0.9  # How hard ball pushes away from beetles (0.1-2.0)
BALL_MOMENTUM_TRANSFER = 1.5  # How much beetle velocity transfers to ball (0.0-2.0)
BALL_RESTITUTION = 0.2  # Bounciness coefficient (0=no bounce, 1=full bounce)
BALL_MASS_RATIO = 0.6  # Ball weight vs beetle (0.1=very light, 2.0=heavy)
BALL_ROLLING_FRICTION = 0.99  # Horizontal slowdown (0.80=high friction, 0.99=ice)
BALL_GROUND_BOUNCE = 0.8  # Floor bounce coefficient (0=dead stop, 0.8=super bouncy)
BALL_PUSH_MULTIPLIER = 3.9  # How easily beetles can push the ball (1.0=normal, 3.0=very easy)
BALL_SPIN_MULTIPLIER = 4.3  # How easily ball spins when hit (1.0=normal, 4.0=very spinny)
BALL_ANGULAR_FRICTION = 0.995  # How quickly ball spin slows (0.9=fast stop, 0.99=long spin)

# Ball torque/lift physics (realistic soccer ball behavior)
BALL_LIFT_STRENGTH = 3.6  # How much ball lifts when scooping with horn (0.0-10.0)
BALL_PASSIVE_LIFT_STRENGTH = 0.5  # Passive lift when touching bottom of ball (0.0-10.0)
BALL_TIP_STRENGTH = 5.0  # How much ball tips down when hit from above (0.0-5.0)
BALL_TORQUE_STRENGTH = 8.0  # How much ball spins from side hits (0.0-10.0)
BALL_GRAVITY_MULTIPLIER = 1.8  # Extra gravity for ball (multiplier, 1.0-5.0)

# Bowl perimeter physics (ball mode)
BOWL_SLIDE_STRENGTH = 1.2  # Inward push force on slippery surface (0.0-2.0)

# Beetle state with physics
class Beetle:
    def __init__(self, x, z, rotation, color):
        self.x = x
        self.z = z
        self.vx = 0.0  # Velocity
        self.vz = 0.0
        self.y = 1.0  # Vertical position (floor level)
        self.vy = 0.0  # Vertical velocity
        self.on_ground = True  # Ground contact state
        # Yaw rotation (spin around vertical axis)
        self.rotation = rotation
        self.target_rotation = rotation
        self.angular_velocity = 0.0  # Spin speed (radians/sec)
        self.moment_of_inertia = BEETLE_RADIUS * MOMENT_OF_INERTIA_FACTOR  # Resistance to rotation (higher = more stable, less flinging)

        # Pitch rotation (forward/backward tilt)
        self.pitch = 0.0  # Body pitch angle (radians, positive = front up)
        self.pitch_velocity = 0.0  # Pitch angular velocity (radians/sec)
        self.pitch_inertia = BEETLE_RADIUS * MOMENT_OF_INERTIA_FACTOR  # Same inertia as yaw

        # Roll rotation (side-to-side tilt)
        self.roll = 0.0  # Body roll angle (radians, positive = right side up)
        self.roll_velocity = 0.0  # Roll angular velocity (radians/sec)
        self.roll_inertia = BEETLE_RADIUS * MOMENT_OF_INERTIA_FACTOR  # Same inertia as yaw
        self.color = color
        self.radius = BEETLE_RADIUS
        self.occupied_voxels = set()  # Track voxel positions for accurate collision

        # Animation state for procedural leg movement
        self.walk_phase = 0.0  # Current walk cycle phase (0 to 2π)
        self.prev_walk_phase = 0.0  # Previous frame walk phase (for leg dust touchdown detection)
        self.spin_dust_timer = 0.0  # Timer for spinning dust spawn interval
        self.spin_dust_leg_index = 0  # Current leg index for spinning dust pattern (0-5)
        self.is_moving = False  # Whether beetle is actively walking
        self.is_moving_backward = False  # True when moving backward (for dust/animation)
        self.is_rotating_only = False  # True when rotating without forward movement
        self.rotation_direction = 0  # -1 = left, 0 = none, 1 = right
        self.is_completing_animation = False  # True when finishing walk cycle after input stops
        self.target_walk_phase = 0.0  # Target neutral phase (0 or π) for animation completion
        self.completion_speed = 5.0  # Speed multiplier for completing animation (rad/s)

        # Smoothed collision normal for stable contact resolution
        self.contact_normal_x = 0.0  # Filtered collision normal X component
        self.contact_normal_z = 0.0  # Filtered collision normal Z component

        # Horn control state
        self.horn_pitch = HORN_DEFAULT_PITCH  # Vertical angle in radians - starts at +10°
        self.horn_yaw = 0.0    # Horizontal angle (stag=pincer spread, rhino=horn yaw) in radians
        self.horn_pitch_velocity = 0.0  # Rate of change of horn pitch (radians/sec)
        self.horn_yaw_velocity = 0.0  # Rate of change of horn yaw (radians/sec)
        self.horn_pitch_damping = 0.0  # Horn pitch resistance during collision (0.0-1.0)
        self.horn_yaw_damping = 0.0    # Horn yaw resistance during collision (0.0-1.0)

        # OPTIMIZATION: Cached horn tip positions to avoid recalculating every frame
        self.cached_horn_pitch = HORN_DEFAULT_PITCH  # Last pitch used for calculation
        self.cached_horn_yaw = 0.0  # Last yaw used for calculation
        self.cached_min_distance = 999.0  # Cached minimum distance result

        # Horn geometry (set during geometry generation)
        self.horn_shaft_len = 12  # Default horn shaft length in voxels
        self.horn_prong_len = 5   # Default horn prong length in voxels
        self.horn_length = 17.0   # Cached total horn reach (updated when geometry changes)
        self.horn_type = "rhino"  # Horn type: "rhino" (single horn), "stag" (dual pincers), "hercules" (dual jaws), or "scorpion"
        self.horn_type_id = HORN_TYPE_IDS["rhino"]  # OPTIMIZATION: Integer ID for fast lookup (avoids string comparisons)
        self.body_pitch_offset = 0.0  # Static body tilt angle (radians) - calculated from leg geometry for scorpion

        # Scorpion stinger control (VB/NM keys)
        self.stinger_curvature = 0.0  # Stinger curvature offset (-1.0 to +1.0): negative=dart forward, positive=pull back, 0=neutral
        self.tail_rotation_angle = 0.0  # Tail rotation angle in degrees (VB=down, NM=up)

        # Center of gravity tracking (for edge tipping physics)
        self.cog_x = x  # Center of gravity X
        self.cog_y = 1.0 + 3.0  # Center of gravity Y (approximate beetle center)
        self.cog_z = z  # Center of gravity Z

        # Collision cooldown timers
        self.lift_cooldown = 0.0  # Time remaining before next lift can be applied (seconds)

        # Body rotation damping state (Phase 3: Directional Rotation Prevention)
        self.body_rotation_damping = 0.0  # Rotation resistance after collision (0.0-1.0)
        self.collision_spin_direction = 0  # Which way collision made us spin (-1=CCW, 0=none, 1=CW)
        self.in_horn_collision = False  # True when actively in horn-to-horn collision

        # Beetle active state (for fall death)
        self.active = True  # False when beetle has fallen off arena
        self.is_falling = False  # True when beetle has passed point of no return
        self.has_exploded = False  # True when death particle explosion has been triggered
        self.explosion_delay = 0.0  # Delay before particles start spawning
        self.explosion_timer = 0.0  # Timer for gradual particle spawning (0.3 seconds)
        self.explosion_pos_x = 0.0  # Store explosion position
        self.explosion_pos_y = 0.0
        self.explosion_pos_z = 0.0
        self.is_lifted_high = False  # True when lifted significantly above ground (for leg spaz animation)

        # Hook interior tracking for stag beetles (parallel to body geometry cache)
        self.body_hook_interior_flags = []  # 1=hook interior, 0=regular (4th element from stag pincer voxels)

        # Previous state for fixed timestep interpolation
        self.prev_x = x
        self.prev_y = 1.0
        self.prev_z = z
        self.prev_rotation = rotation
        self.prev_pitch = 0.0
        self.prev_roll = 0.0
        self.prev_horn_pitch = HORN_DEFAULT_PITCH
        self.prev_horn_yaw = 0.0

    def save_previous_state(self):
        """Save current state as previous for interpolation"""
        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_z = self.z
        self.prev_rotation = self.rotation
        self.prev_pitch = self.pitch
        self.prev_roll = self.roll
        self.prev_horn_pitch = self.horn_pitch
        self.prev_horn_yaw = self.horn_yaw

    def apply_force(self, fx, fz, dt):
        """Add force to velocity"""
        self.vx += fx * dt
        self.vz += fz * dt

    def update_physics(self, dt):
        """Apply friction and update position"""
        # === COLLISION COOLDOWN TIMER ===
        # Decrement lift cooldown timer (prevents multi-frame lift application)
        if self.lift_cooldown > 0.0:
            self.lift_cooldown = max(0.0, self.lift_cooldown - dt)

        # Decay horn pitch/yaw damping when not in contact (Phase 2: Horn Clipping Prevention)
        if self.horn_pitch_damping > 0.0:
            self.horn_pitch_damping = max(0.0, self.horn_pitch_damping - HORN_DAMPING_DECAY_RATE * dt)
        if self.horn_yaw_damping > 0.0:
            self.horn_yaw_damping = max(0.0, self.horn_yaw_damping - HORN_DAMPING_DECAY_RATE * dt)

        # Clear horn collision flag (will be reset each frame if still colliding)
        self.in_horn_collision = False

        # Decay body rotation damping after collision (Phase 3: Directional Rotation Prevention)
        if self.body_rotation_damping > 0.0:
            self.body_rotation_damping = max(0.0, self.body_rotation_damping - BODY_ROTATION_DAMPING_DECAY * dt)
            # Clear spin direction when damping reaches zero
            if self.body_rotation_damping == 0.0:
                self.collision_spin_direction = 0

        # === GRAVITY SYSTEM (Phase 1) ===
        # Air resistance (quadratic drag - lightweight calculation)
        # Skip air resistance for ball to prevent "hanging" at arc peak
        if abs(self.vy) > 1.0 and self.horn_type != "ball":
            air_drag = self.vy * abs(self.vy) * AIR_RESISTANCE
            self.vy -= air_drag * dt

        # Apply gravity (always on) - use global adjustable gravity
        self.vy -= physics_params["GRAVITY"] * dt

        # Apply vertical velocity
        self.y += self.vy * dt

        # Cap vertical velocity to prevent outlier launches (hard safety limit)
        if self.vy > MAX_VERTICAL_VELOCITY:
            self.vy = MAX_VERTICAL_VELOCITY

        # Ground contact will be determined by floor collision detection in main loop
        # (No hardcoded floor level here - floor is detected via voxel checking)

        # === HORIZONTAL PHYSICS (existing) ===
        # Apply linear friction (skip for ball when airborne to allow proper arc)
        if self.horn_type == "ball":
            # Ball uses its own rolling friction only when on ground (applied in main loop)
            pass
        else:
            self.vx *= FRICTION
            self.vz *= FRICTION

        # Apply angular friction (yaw)
        # Ball uses lighter angular friction for better spin retention
        if self.horn_type == "ball":
            ball_angular_friction = physics_params.get("BALL_ANGULAR_FRICTION", 0.98)
            self.angular_velocity *= ball_angular_friction
        else:
            self.angular_velocity *= ANGULAR_FRICTION

        # Apply angular friction (pitch/roll) - different damping based on ground contact
        if self.on_ground:
            # Heavy damping when on ground for stability
            self.pitch_velocity *= ANGULAR_FRICTION * 0.9  # Extra damping
            self.roll_velocity *= ANGULAR_FRICTION * 0.9
        else:
            # Light damping when airborne to allow dramatic tumbling (tunable via slider)
            airborne_damping = physics_params.get("AIRBORNE_DAMPING", 0.98)
            self.pitch_velocity *= airborne_damping
            self.roll_velocity *= airborne_damping

        # Ground restoring torque - automatically level out when on ground
        # Only apply strong restoring when beetle is settled, not just bouncing
        if self.on_ground and abs(self.vy) < 2.0:  # Must be on ground AND not bouncing up/down
            # Apply torque to bring pitch/roll back to zero (level)
            RESTORING_STRENGTH = physics_params.get("RESTORING_STRENGTH", 35.0)
            self.pitch_velocity -= self.pitch * RESTORING_STRENGTH * dt
            self.roll_velocity -= self.roll * RESTORING_STRENGTH * dt
        elif self.on_ground and abs(self.vy) >= 2.0:
            # Bouncing/landing - apply moderate restoring to allow tumbling but still settle
            WEAK_RESTORING = physics_params.get("WEAK_RESTORING", 25.0)
            self.pitch_velocity -= self.pitch * WEAK_RESTORING * dt
            self.roll_velocity -= self.roll * WEAK_RESTORING * dt

        # Clamp linear speed (different max for forward vs backward)
        speed = math.sqrt(self.vx**2 + self.vz**2)
        if speed > 0.01:  # Avoid division by zero
            # Calculate direction beetle is moving relative to facing direction
            forward_x = math.cos(self.rotation)
            forward_z = math.sin(self.rotation)
            dot_product = self.vx * forward_x + self.vz * forward_z

            # Apply different speed caps based on direction
            if dot_product >= 0:  # Moving forward
                if speed > MAX_SPEED:
                    self.vx = (self.vx / speed) * MAX_SPEED
                    self.vz = (self.vz / speed) * MAX_SPEED
            else:  # Moving backward
                if speed > BACKWARD_MAX_SPEED:
                    self.vx = (self.vx / speed) * BACKWARD_MAX_SPEED
                    self.vz = (self.vz / speed) * BACKWARD_MAX_SPEED

        # Clamp angular speeds (yaw, pitch, roll)
        if abs(self.angular_velocity) > MAX_ANGULAR_SPEED:
            self.angular_velocity = MAX_ANGULAR_SPEED if self.angular_velocity > 0 else -MAX_ANGULAR_SPEED

        # Different tilt speed caps based on ground contact
        if self.on_ground:
            MAX_TILT_SPEED = 8.0  # Conservative limit when on ground
        else:
            # Allow dramatic tumbling when airborne (tunable via slider)
            MAX_TILT_SPEED = physics_params.get("AIRBORNE_TILT_SPEED", 20.0)

        if abs(self.pitch_velocity) > MAX_TILT_SPEED:
            self.pitch_velocity = MAX_TILT_SPEED if self.pitch_velocity > 0 else -MAX_TILT_SPEED
        if abs(self.roll_velocity) > MAX_TILT_SPEED:
            self.roll_velocity = MAX_TILT_SPEED if self.roll_velocity > 0 else -MAX_TILT_SPEED

        # Update horizontal position
        self.x += self.vx * dt
        self.z += self.vz * dt

        # Update rotation angles (yaw, pitch, roll)
        self.pitch += self.pitch_velocity * dt
        self.roll += self.roll_velocity * dt

        # Clamp pitch/roll only when on ground - allow full rotations when airborne
        if self.on_ground:
            # Tunable max tilt angle when grounded (slider controls in degrees, converted to radians)
            ground_tilt_degrees = physics_params.get("GROUND_TILT_ANGLE", 60.0)
            MAX_TILT_ANGLE = math.radians(ground_tilt_degrees)
            self.pitch = max(-MAX_TILT_ANGLE, min(MAX_TILT_ANGLE, self.pitch))
            self.roll = max(-MAX_TILT_ANGLE, min(MAX_TILT_ANGLE, self.roll))
        # When airborne, allow full 360° tumbling (no clamping)

    def arena_collision(self):
        """Bounce off arena walls"""
        dist = math.sqrt(self.x**2 + self.z**2)
        if dist > ARENA_RADIUS - self.radius:
            # Push back
            angle = math.atan2(self.z, self.x)
            self.x = math.cos(angle) * (ARENA_RADIUS - self.radius)
            self.z = math.sin(angle) * (ARENA_RADIUS - self.radius)

            # Bounce velocity
            normal_x = self.x / dist
            normal_z = self.z / dist
            dot = self.vx * normal_x + self.vz * normal_z
            self.vx -= 2 * dot * normal_x * 0.5
            self.vz -= 2 * dot * normal_z * 0.5

# Game state
match_winner = None  # "BLUE" or "RED" when a beetle dies
victory_pulse_timer = 0.0  # Timer for winner glow effect
VICTORY_PULSE_DURATION = 5.0  # Pulse for 5 seconds after victory
victory_confetti_timer = 0.0  # Timer for spawning confetti waves
VICTORY_CONFETTI_DELAY = 0.6  # Wait 600ms before starting confetti
VICTORY_CONFETTI_INTERVAL = 0.15  # Spawn confetti every 0.15 seconds during victory
VICTORY_CONFETTI_PARTICLES = 30  # Particles per spawn wave

def reset_match():
    """Reset beetles to starting positions for new match"""
    global beetle_blue, beetle_red, match_winner, victory_pulse_timer, victory_confetti_timer, previous_stinger_curvature, previous_tail_rotation, blue_horn_type, red_horn_type

    # Sync GPU to ensure any pending operations complete before reset
    ti.sync()

    beetle_blue = Beetle(-20.0, 0.0, 0.0, simulation.BEETLE_BLUE)
    beetle_red = Beetle(20.0, 0.0, math.pi, simulation.BEETLE_RED)
    match_winner = None
    victory_pulse_timer = 0.0
    victory_confetti_timer = 0.0

    # Restore beetle colors from saved window values (in case restart during victory pulse)
    b = window.blue_body_color
    simulation.blue_body_color[None] = ti.Vector([b[0], b[1], b[2]])
    b = window.blue_leg_color
    simulation.blue_leg_color[None] = ti.Vector([b[0], b[1], b[2]])
    b = window.blue_leg_tip_color
    simulation.blue_leg_tip_color[None] = ti.Vector([b[0], b[1], b[2]])
    b = window.blue_stripe_color
    simulation.blue_stripe_color[None] = ti.Vector([b[0], b[1], b[2]])
    b = window.blue_horn_tip_color
    simulation.blue_horn_tip_color[None] = ti.Vector([b[0], b[1], b[2]])

    r = window.red_body_color
    simulation.red_body_color[None] = ti.Vector([r[0], r[1], r[2]])
    r = window.red_leg_color
    simulation.red_leg_color[None] = ti.Vector([r[0], r[1], r[2]])
    r = window.red_leg_tip_color
    simulation.red_leg_tip_color[None] = ti.Vector([r[0], r[1], r[2]])
    r = window.red_stripe_color
    simulation.red_stripe_color[None] = ti.Vector([r[0], r[1], r[2]])
    r = window.red_horn_tip_color
    simulation.red_horn_tip_color[None] = ti.Vector([r[0], r[1], r[2]])

    # Reset scorpion tracking variables to prevent geometry cache desync
    previous_stinger_curvature = 0.0
    previous_tail_rotation = 0.0

    # Preserve each beetle's horn_type on reset so control mapping stays correct
    beetle_blue.horn_type = blue_horn_type
    beetle_blue.horn_type_id = HORN_TYPE_IDS.get(blue_horn_type, 0)
    beetle_red.horn_type = red_horn_type
    beetle_red.horn_type_id = HORN_TYPE_IDS.get(red_horn_type, 0)

    # Camera always tracks from edge side (beetles in foreground)

    print("\n" + "="*50)
    print("NEW MATCH STARTED!")
    print("="*50 + "\n")

# Create beetles - closer together for smaller arena (horn dimensions set later)
beetle_blue = Beetle(-20.0, 0.0, 0.0, simulation.BEETLE_BLUE)  # Facing right (toward red)
beetle_red = Beetle(20.0, 0.0, math.pi, simulation.BEETLE_RED)  # Facing left (toward blue)
beetle_ball = Beetle(0.0, 0.0, 0.0, simulation.BALL)  # Soccer ball (center of arena, no rotation matters)
beetle_ball.horn_type = "ball"  # Special type for sphere rendering
beetle_ball.active = False  # Ball starts disabled
beetle_ball.radius = 4.0  # Default ball radius
ball_cache_initialized = False  # Will be initialized when ball is first activated

# Ball mode scoring
blue_score = 0
red_score = 0
ball_scored_this_fall = False  # Prevent multiple scores while ball falling

# Ball explosion state
ball_has_exploded = False
ball_explosion_timer = 0.0
ball_explosion_pos_x = 0.0
ball_explosion_pos_y = 0.0
ball_explosion_pos_z = 0.0
ball_dust_cooldown = 0.0  # Cooldown timer for bounce dust

# Goal celebration state (scored-on beetle explodes, then winner confetti/flash)
goal_scored_by = None  # "BLUE" or "RED" - who scored
goal_celebration_timer = 0.0  # Timer for celebration sequence

# Score digit display state
blue_score_bounce_timer = 0.0  # Timer for blue digit bounce animation
red_score_bounce_timer = 0.0  # Timer for red digit bounce animation
blue_score_delay_timer = 0.0  # Delay before animation/explosion starts
red_score_delay_timer = 0.0   # Delay before animation/explosion starts
blue_score_pending = False    # Whether blue has a pending score to add after delay
red_score_pending = False     # Whether red has a pending score to add after delay
SCORE_ANIMATION_DELAY = 1.0   # 1 second delay before score animation and explosion
SCORE_BOUNCE_DURATION = 0.8  # Duration of pop & squash animation (slightly longer for spring effect)
# Score burst particle spawning over time
blue_burst_timer = 0.0        # Timer for spawning blue burst particles over time
red_burst_timer = 0.0         # Timer for spawning red burst particles over time
SCORE_BURST_DURATION = 0.2    # Spawn particles over 0.2 seconds
SCORE_BURST_PARTICLES = 300   # Total particles to spawn
blue_burst_spawned = 0        # Particles spawned so far for blue
red_burst_spawned = 0         # Particles spawned so far for red
BASE_DIGIT_SCALE = 2.0  # Base size multiplier for score digits (1.0 = 5x7 voxels)

# Beetle respawn state
blue_respawn_timer = 0.0  # Timer for blue beetle respawn
red_respawn_timer = 0.0   # Timer for red beetle respawn
BEETLE_RESPAWN_DELAY = 4.0  # Same timing as ball celebration

# Beetle assembly animation state (voxel rain effect)
blue_assembling = False
red_assembling = False
blue_assembly_timer = 0.0
red_assembly_timer = 0.0
ASSEMBLY_DURATION = 3.0  # Time for voxels to assemble (slower, more dramatic)
ASSEMBLY_START_TIME = 1.0  # When assembly starts during respawn delay (earlier to compensate)

# Ball assembly animation state (voxel rain effect)
ball_assembling = False
ball_assembly_timer = 0.0
BALL_ASSEMBLY_DURATION = 3.0  # Time for ball voxels to assemble (slower, more dramatic)

# Track if assembly happened last frame (need one final clear after assembly ends)
assembly_needs_final_clear = False

# Digit patterns for 0-9 (5 wide x 7 tall, stored as row bitmasks)
# Each digit is 7 rows, each row is 5 bits (bit 0 = leftmost)
# Pattern: .###. = 0b01110, .#.#. = 0b01010, etc.
DIGIT_PATTERNS = [
    # 0
    [0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110],
    # 1
    [0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110],
    # 2
    [0b01110, 0b10001, 0b00001, 0b00110, 0b01000, 0b10000, 0b11111],
    # 3
    [0b01110, 0b10001, 0b00001, 0b00110, 0b00001, 0b10001, 0b01110],
    # 4
    [0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010, 0b00010],
    # 5
    [0b11111, 0b10000, 0b11110, 0b00001, 0b00001, 0b10001, 0b01110],
    # 6
    [0b01110, 0b10000, 0b10000, 0b11110, 0b10001, 0b10001, 0b01110],
    # 7
    [0b11111, 0b00001, 0b00010, 0b00100, 0b00100, 0b00100, 0b00100],
    # 8
    [0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001, 0b01110],
    # 9
    [0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b00001, 0b01110],
]

# Store digit patterns in Taichi field for kernel access (10 digits, 7 rows)
digit_pattern_field = ti.field(dtype=ti.i32, shape=(10, 7))

# Initialize digit patterns into Taichi field
for d in range(10):
    for r in range(7):
        digit_pattern_field[d, r] = DIGIT_PATTERNS[d][r]

@ti.kernel
def render_score_digit(digit: ti.i32, base_x: ti.f32, base_y: ti.f32, base_z: ti.f32,
                       voxel_type: ti.i32, scale_x: ti.f32, scale_y: ti.f32,
                       cam_x: ti.f32, cam_z: ti.f32):
    """Render a single digit (0-9) as 3D extruded voxels with billboarding (always faces camera)
    base_x, base_y, base_z: center of digit in grid coords
    voxel_type: SCORE_DIGIT_BLUE or SCORE_DIGIT_RED
    scale_x: horizontal scale (>1.0 = wider, <1.0 = narrower)
    scale_y: vertical scale (>1.0 = taller, <1.0 = squashed)
    cam_x, cam_z: camera position in world coords (for billboarding)
    """
    # Convert digit grid position to world coords (grid center is 64)
    digit_world_x = base_x - 64.0
    digit_world_z = base_z - 64.0

    # Calculate direction from digit to camera
    dx = cam_x - digit_world_x
    dz = cam_z - digit_world_z

    # Normalize to get facing direction
    dist = ti.sqrt(dx * dx + dz * dz)
    if dist > 0.001:
        dx = dx / dist
        dz = dz / dist
    else:
        dx = 0.0
        dz = 1.0

    # The digit's "right" vector is perpendicular to the facing direction
    right_x = dz  # Perpendicular to (dx, dz) - flipped for correct reading direction
    right_z = -dx

    # Forward vector (depth direction, pointing toward camera)
    forward_x = dx
    forward_z = dz

    # Extrusion depth (3 voxels deep)
    depth = 3

    for row in range(7):
        row_pattern = digit_pattern_field[digit, row]
        for col in range(5):
            if (row_pattern >> (4 - col)) & 1:  # Check if bit is set (reversed for correct orientation)
                # Calculate scaled position from center (horizontal offset)
                local_h = (col - 2) * scale_x  # Horizontal offset in digit's local space
                # Flip row so 0 is at bottom, scale from bottom (y offset adjusted)
                local_y = (6 - row) * scale_y

                # Rotate horizontal offset to face camera
                base_offset_x = local_h * right_x
                base_offset_z = local_h * right_z
                offset_y = local_y

                # Extrude along depth (forward/backward from face)
                for d in range(depth):
                    depth_offset = (d - depth // 2) * 1.0  # Center the depth around the base position
                    offset_x = base_offset_x + depth_offset * forward_x
                    offset_z = base_offset_z + depth_offset * forward_z

                    # Place voxel at rotated position
                    vx = ti.cast(base_x + offset_x, ti.i32)
                    vy = ti.cast(base_y + offset_y, ti.i32)
                    vz = ti.cast(base_z + offset_z, ti.i32)

                    # Bounds check
                    if 0 <= vx < 128 and 0 <= vy < 128 and 0 <= vz < 128:
                        simulation.voxel_type[vx, vy, vz] = voxel_type

@ti.kernel
def clear_score_digits():
    """Clear all score digit voxels"""
    for i, j, k in ti.ndrange(128, 128, 128):
        if simulation.voxel_type[i, j, k] == simulation.SCORE_DIGIT_BLUE or \
           simulation.voxel_type[i, j, k] == simulation.SCORE_DIGIT_RED:
            simulation.voxel_type[i, j, k] = simulation.EMPTY

# Assembly animation voxel types (temporary, for clearing)
ASSEMBLY_VOXEL_BLUE = 25
ASSEMBLY_VOXEL_RED = 26
ASSEMBLY_VOXEL_BALL = 27
ASSEMBLY_VOXEL_BALL_STRIPE = 28
ASSEMBLY_VOXEL_BLUE_STRIPE = 29
ASSEMBLY_VOXEL_RED_STRIPE = 30
ASSEMBLY_VOXEL_BLUE_HORN_TIP = 31
ASSEMBLY_VOXEL_RED_HORN_TIP = 32

# Pre-computed scatter offsets for assembly animation (computed once at startup)
MAX_ASSEMBLY_VOXELS = 2500  # Same as MAX_BODY_VOXELS
assembly_scatter_x = ti.field(ti.f32, shape=MAX_ASSEMBLY_VOXELS)
assembly_scatter_y = ti.field(ti.f32, shape=MAX_ASSEMBLY_VOXELS)
assembly_scatter_z = ti.field(ti.f32, shape=MAX_ASSEMBLY_VOXELS)

# Ball-specific scatter offsets (more compact since ball is smaller)
ball_scatter_x = ti.field(ti.f32, shape=MAX_ASSEMBLY_VOXELS)
ball_scatter_y = ti.field(ti.f32, shape=MAX_ASSEMBLY_VOXELS)
ball_scatter_z = ti.field(ti.f32, shape=MAX_ASSEMBLY_VOXELS)

# Pre-compute scatter offsets once at startup
import random as _random
for _i in range(MAX_ASSEMBLY_VOXELS):
    _random.seed(_i * 31337)
    # Beetle scatter - wider spread
    assembly_scatter_x[_i] = _random.uniform(-20, 20)
    assembly_scatter_y[_i] = _random.uniform(25, 45)
    assembly_scatter_z[_i] = _random.uniform(-20, 20)
    # Ball scatter - more compact (smaller range)
    _random.seed(_i * 31337 + 1)  # Different seed for variation
    ball_scatter_x[_i] = _random.uniform(-8, 8)
    ball_scatter_y[_i] = _random.uniform(10, 20)
    ball_scatter_z[_i] = _random.uniform(-8, 8)

@ti.kernel
def clear_assembly_voxels():
    """Clear all assembly animation voxels"""
    for i, j, k in ti.ndrange(128, 128, 128):
        vtype = simulation.voxel_type[i, j, k]
        if vtype == ASSEMBLY_VOXEL_BLUE or vtype == ASSEMBLY_VOXEL_RED or \
           vtype == ASSEMBLY_VOXEL_BALL or vtype == ASSEMBLY_VOXEL_BALL_STRIPE or \
           vtype == ASSEMBLY_VOXEL_BLUE_STRIPE or vtype == ASSEMBLY_VOXEL_RED_STRIPE or \
           vtype == ASSEMBLY_VOXEL_BLUE_HORN_TIP or vtype == ASSEMBLY_VOXEL_RED_HORN_TIP:
            simulation.voxel_type[i, j, k] = simulation.EMPTY

# Ball voxel cache for assembly animation (pre-computed sphere voxels)
MAX_BALL_VOXELS = 1000  # Ball is much smaller than beetles
ball_cache_x = ti.field(ti.i32, shape=MAX_BALL_VOXELS)
ball_cache_y = ti.field(ti.i32, shape=MAX_BALL_VOXELS)
ball_cache_z = ti.field(ti.i32, shape=MAX_BALL_VOXELS)
ball_cache_is_stripe = ti.field(ti.i32, shape=MAX_BALL_VOXELS)  # 1 if stripe, 0 if not (for fast render)
ball_cache_size = ti.field(ti.i32, shape=())

# Track last rendered ball position for fast clearing (grid coordinates)
ball_last_grid_x = ti.field(ti.i32, shape=())
ball_last_grid_y = ti.field(ti.i32, shape=())
ball_last_grid_z = ti.field(ti.i32, shape=())
ball_last_rendered = ti.field(ti.i32, shape=())  # 1 if ball was rendered, 0 if not

# Floor height cache for fast collision lookups (128x128 grid)
floor_height_cache = ti.field(ti.f32, shape=(128, 128))

@ti.kernel
def build_floor_height_cache_kernel():
    """Pre-compute floor heights for entire arena (including bowl ice)"""
    for i, k in ti.ndrange(128, 128):
        highest_y = -1000.0
        floor_base_y = int(RENDER_Y_OFFSET)  # 33
        # Check Y range that includes main floor and bowl slope
        for j in range(floor_base_y - 2, floor_base_y + 5):
            if 0 <= j < 128:
                vtype = simulation.voxel_type[i, j, k]
                if vtype == simulation.CONCRETE or vtype == simulation.SLIPPERY:
                    world_y = float(j) - RENDER_Y_OFFSET
                    if world_y > highest_y:
                        highest_y = world_y
        floor_height_cache[i, k] = highest_y

def build_floor_height_cache():
    """Build/rebuild the floor height cache (call after arena init or ball mode toggle)"""
    build_floor_height_cache_kernel()
    print("Floor height cache built")

def init_ball_cache(radius: float):
    """Pre-compute ball voxel positions relative to center (with stripe info)"""
    idx = 0
    r_int = int(radius) + 1
    stripe_width = 2.0  # Same as render_ball
    for dx in range(-r_int, r_int + 1):
        for dy in range(-r_int, r_int + 1):
            for dz in range(-r_int, r_int + 1):
                dist_sq = dx*dx + dy*dy + dz*dz
                if dist_sq <= radius * radius:
                    if idx < MAX_BALL_VOXELS:
                        ball_cache_x[idx] = dx
                        ball_cache_y[idx] = dy
                        ball_cache_z[idx] = dz
                        # Pre-compute stripe based on x position (default orientation)
                        ball_cache_is_stripe[idx] = 1 if abs(dx) <= stripe_width else 0
                        idx += 1
    ball_cache_size[None] = idx
    ball_last_rendered[None] = 0  # No ball rendered yet
    print(f"Ball cache initialized with {idx} voxels (radius={radius})")

@ti.kernel
def render_assembly_kernel_blue(center_x: ti.i32, center_y: ti.i32, center_z: ti.i32,
                                 t: ti.f32, num_voxels: ti.i32):
    """GPU-accelerated assembly rendering for blue beetle with proper colors"""
    for i in range(num_voxels):
        # Get cached voxel offset
        lx = blue_body_cache_x[i]
        ly = blue_body_cache_y[i]
        lz = blue_body_cache_z[i]

        # Target position
        target_x = center_x + lx
        target_y = center_y + ly
        target_z = center_z + lz

        # Start position (scattered above)
        start_x = target_x + assembly_scatter_x[i]
        start_y = target_y + assembly_scatter_y[i]
        start_z = target_z + assembly_scatter_z[i]

        # Interpolate
        current_x = ti.cast(start_x + (target_x - start_x) * t, ti.i32)
        current_y = ti.cast(start_y + (target_y - start_y) * t, ti.i32)
        current_z = ti.cast(start_z + (target_z - start_z) * t, ti.i32)

        # Bounds check and place only in empty space
        if 0 <= current_x < 128 and 0 <= current_y < 128 and 0 <= current_z < 128:
            existing = simulation.voxel_type[current_x, current_y, current_z]
            if existing == 0 or existing >= ASSEMBLY_VOXEL_BLUE:
                # Determine voxel type based on cached flags
                voxel_type = ASSEMBLY_VOXEL_BLUE  # Default body color
                if blue_body_horn_tip_flags[i] == 1:
                    voxel_type = ASSEMBLY_VOXEL_BLUE_HORN_TIP
                elif blue_body_stripe_flags[i] == 1:
                    voxel_type = ASSEMBLY_VOXEL_BLUE_STRIPE
                simulation.voxel_type[current_x, current_y, current_z] = voxel_type

@ti.kernel
def render_assembly_kernel_red(center_x: ti.i32, center_y: ti.i32, center_z: ti.i32,
                                t: ti.f32, num_voxels: ti.i32):
    """GPU-accelerated assembly rendering for red beetle with proper colors"""
    for i in range(num_voxels):
        # Get cached voxel offset - flip X to face opposite direction (180° rotation)
        lx = -red_body_cache_x[i]  # Flip X for 180° rotation
        ly = red_body_cache_y[i]
        lz = -red_body_cache_z[i]  # Flip Z for 180° rotation

        # Target position
        target_x = center_x + lx
        target_y = center_y + ly
        target_z = center_z + lz

        # Start position (scattered above)
        start_x = target_x + assembly_scatter_x[i]
        start_y = target_y + assembly_scatter_y[i]
        start_z = target_z + assembly_scatter_z[i]

        # Interpolate
        current_x = ti.cast(start_x + (target_x - start_x) * t, ti.i32)
        current_y = ti.cast(start_y + (target_y - start_y) * t, ti.i32)
        current_z = ti.cast(start_z + (target_z - start_z) * t, ti.i32)

        # Bounds check and place only in empty space
        if 0 <= current_x < 128 and 0 <= current_y < 128 and 0 <= current_z < 128:
            existing = simulation.voxel_type[current_x, current_y, current_z]
            if existing == 0 or existing >= ASSEMBLY_VOXEL_BLUE:
                # Determine voxel type based on cached flags
                voxel_type = ASSEMBLY_VOXEL_RED  # Default body color
                if red_body_horn_tip_flags[i] == 1:
                    voxel_type = ASSEMBLY_VOXEL_RED_HORN_TIP
                elif red_body_stripe_flags[i] == 1:
                    voxel_type = ASSEMBLY_VOXEL_RED_STRIPE
                simulation.voxel_type[current_x, current_y, current_z] = voxel_type

def render_beetle_assembly_fast(is_blue, spawn_x, spawn_y, spawn_z, progress):
    """Fast assembly rendering using GPU kernel"""
    # Cubic ease-in
    t = progress * progress * progress

    # Convert to grid coordinates
    center_x = int(spawn_x) + 64
    center_y = int(spawn_y)
    center_z = int(spawn_z) + 64

    if is_blue:
        num_voxels = blue_body_cache_size[None]
        render_assembly_kernel_blue(center_x, center_y, center_z, t, num_voxels)
    else:
        num_voxels = red_body_cache_size[None]
        render_assembly_kernel_red(center_x, center_y, center_z, t, num_voxels)

@ti.kernel
def render_assembly_kernel_ball(center_x: ti.i32, center_y: ti.i32, center_z: ti.i32,
                                 t: ti.f32, num_voxels: ti.i32):
    """GPU-accelerated assembly rendering for ball - smooth with per-voxel timing"""
    for i in range(num_voxels):
        # Get cached voxel offset
        lx = ball_cache_x[i]
        ly = ball_cache_y[i]
        lz = ball_cache_z[i]

        # Per-voxel timing offset for staggered assembly (inner voxels arrive first)
        # Distance from center determines delay - outer voxels start later
        dist_from_center = ti.sqrt(float(lx*lx + ly*ly + lz*lz))
        max_dist = 5.0  # Ball radius
        # Stagger: inner voxels (dist=0) get t, outer voxels (dist=max) get t delayed by 0.3
        stagger = (dist_from_center / max_dist) * 0.3
        local_t = ti.max(0.0, ti.min(1.0, (t - stagger) / (1.0 - stagger)))

        # Smooth ease-in-out for less choppy motion
        # Using smoothstep: 3t^2 - 2t^3
        smooth_t = local_t * local_t * (3.0 - 2.0 * local_t)

        # Target position
        target_x = center_x + lx
        target_y = center_y + ly
        target_z = center_z + lz

        # Start position (scattered above) - use ball-specific compact scatter
        start_x = target_x + ball_scatter_x[i]
        start_y = target_y + ball_scatter_y[i]
        start_z = target_z + ball_scatter_z[i]

        # Interpolate with smooth timing
        current_x = ti.cast(start_x + (target_x - start_x) * smooth_t, ti.i32)
        current_y = ti.cast(start_y + (target_y - start_y) * smooth_t, ti.i32)
        current_z = ti.cast(start_z + (target_z - start_z) * smooth_t, ti.i32)

        # Bounds check and place only in empty space
        if 0 <= current_x < 128 and 0 <= current_y < 128 and 0 <= current_z < 128:
            existing = simulation.voxel_type[current_x, current_y, current_z]
            if existing == 0 or existing == ASSEMBLY_VOXEL_BALL or existing == ASSEMBLY_VOXEL_BALL_STRIPE:
                # Use stripe or main color based on cached stripe info
                if ball_cache_is_stripe[i] == 1:
                    simulation.voxel_type[current_x, current_y, current_z] = ASSEMBLY_VOXEL_BALL_STRIPE
                else:
                    simulation.voxel_type[current_x, current_y, current_z] = ASSEMBLY_VOXEL_BALL

def render_ball_assembly_fast(spawn_x, spawn_y, spawn_z, progress):
    """Fast ball assembly rendering using GPU kernel"""
    # Use linear progress - smoothing is done per-voxel in the kernel
    t = progress

    # Convert to grid coordinates
    center_x = int(spawn_x) + 64
    center_y = int(spawn_y)
    center_z = int(spawn_z) + 64

    num_voxels = ball_cache_size[None]
    render_assembly_kernel_ball(center_x, center_y, center_z, t, num_voxels)

@ti.kernel
def clear_beetles():
    """Remove all beetle voxels"""
    for i, j, k in ti.ndrange(simulation.n_grid, simulation.n_grid, simulation.n_grid):
        if simulation.voxel_type[i, j, k] == simulation.BEETLE_BLUE or \
           simulation.voxel_type[i, j, k] == simulation.BEETLE_RED:
            simulation.voxel_type[i, j, k] = simulation.EMPTY

@ti.kernel
def check_collision_kernel(x1: ti.f32, z1: ti.f32, y1: ti.f32, x2: ti.f32, z2: ti.f32, y2: ti.f32, color1: ti.i32, color2: ti.i32) -> ti.i32:
    """Fast GPU-based collision check - returns 1 if collision, 0 otherwise"""
    collision = 0

    # Convert to grid coordinates
    center1_x = int(x1 + simulation.n_grid / 2.0)
    center1_z = int(z1 + simulation.n_grid / 2.0)
    center2_x = int(x2 + simulation.n_grid / 2.0)
    center2_z = int(z2 + simulation.n_grid / 2.0)

    # Early rejection: If beetles too far apart, skip voxel scanning
    # Optimized: Beetle max radius ~18 voxels (body + horn), so 2x radius + margin = 38
    dx = center1_x - center2_x
    dz = center1_z - center2_z
    dist_sq = dx * dx + dz * dz
    too_far = 0
    if dist_sq > 5776:  # (76 voxels)^2 = 2 * 38 voxel max reach for collision detection
        too_far = 1

    # OPTIMIZATION: Tighter collision bounds based on actual separation distance
    # Instead of scanning full 38-voxel radius around each beetle, only scan the overlap region
    dist_between = int(ti.sqrt(float(dist_sq)))

    # Calculate overlap region center (midpoint between beetles)
    overlap_center_x = (center1_x + center2_x) // 2
    overlap_center_z = (center1_z + center2_z) // 2

    # Overlap radius calculation:
    # - When beetles far apart: small overlap region (dist/2 + body_radius)
    # - When beetles close/touching: larger overlap region to catch all contact
    # - Max body radius (without horn): ~15 voxels
    # - With horns: need to add horn reach when close
    BODY_RADIUS = 15
    HORN_REACH = 21

    # Initialize overlap_radius (required by Taichi before conditional)
    overlap_radius = 0

    # Dynamic radius: when far apart use body radius, when close include horn reach
    # Transition smoothly: if dist > 30, use body only; if dist < 30, add partial horn reach
    if dist_between > 30:
        overlap_radius = (dist_between // 2) + BODY_RADIUS
    else:
        # When close, need to account for horns extending into overlap region
        # Linear interpolation: at dist=0, full horn reach; at dist=30, no horn reach
        horn_factor = (30 - dist_between) / 30.0
        extra_horn = int(HORN_REACH * horn_factor)
        overlap_radius = (dist_between // 2) + BODY_RADIUS + extra_horn

    # Ensure minimum scan radius (never smaller than 12 voxels to catch edge cases)
    overlap_radius = ti.max(12, overlap_radius)

    # Calculate bounds around overlap center (much tighter than 38-voxel radius per beetle)
    x_min = ti.max(0, overlap_center_x - overlap_radius)
    x_max = ti.min(simulation.n_grid, overlap_center_x + overlap_radius)
    z_min = ti.max(0, overlap_center_z - overlap_radius)
    z_max = ti.min(simulation.n_grid, overlap_center_z + overlap_radius)

    # Scan for colliding voxels - TRUE 3D collision detection (only if not too far)
    # Track Y-ranges for each beetle in each XZ column
    for gx in range(x_min, x_max):
        for gz in range(z_min, z_max):
            # Skip remaining checks if collision already found OR beetles too far apart
            if collision == 0 and too_far == 0:
                # Track Y ranges for both beetles in this XZ column
                beetle1_y_min = 999
                beetle1_y_max = -1
                beetle2_y_min = 999
                beetle2_y_max = -1

                # Find Y-ranges for both beetles/ball in this column (includes legs and tips!)
                # OPTIMIZED: Adaptive Y-range based on beetle height (grounded vs airborne)
                # Find grid Y for each beetle
                beetle1_grid_y = int(RENDER_Y_OFFSET + y1)
                beetle2_grid_y = int(RENDER_Y_OFFSET + y2)

                # Adaptive reach: grounded beetles need less vertical scan upward
                beetle1_max_up = 22 if y1 < 2.0 else 28  # Grounded: reduced scan, Airborne: full scan
                beetle1_max_down = 12 if y1 >= 2.0 else 10  # Airborne: increase down, Grounded: reduce
                beetle2_max_up = 22 if y2 < 2.0 else 28
                beetle2_max_down = 12 if y2 >= 2.0 else 10

                # Take union of both beetles' adaptive ranges
                y_start = ti.max(0, ti.min(beetle1_grid_y - beetle1_max_down,
                                            beetle2_grid_y - beetle2_max_down))
                y_end = ti.min(simulation.n_grid, ti.max(beetle1_grid_y + beetle1_max_up,
                                                          beetle2_grid_y + beetle2_max_up))

                # Track if hook interior voxels are present in this column
                has_hook_interior = 0

                for gy in range(y_start, y_end):
                    voxel = simulation.voxel_type[gx, gy, gz]

                    # Check for hook interior voxels
                    if voxel == simulation.STAG_HOOK_INTERIOR_BLUE or voxel == simulation.STAG_HOOK_INTERIOR_RED:
                        has_hook_interior = 1

                    # Check if voxel belongs to entity 1 (based on color1)
                    belongs_to_1 = 0
                    if color1 == simulation.BEETLE_BLUE:  # Blue beetle (5, 7, 9, 11, 13, 18)
                        if voxel == 5 or voxel == 7 or voxel == 9 or voxel == 11 or voxel == 13 or voxel == 18:
                            belongs_to_1 = 1
                    elif color1 == simulation.BEETLE_RED:  # Red beetle (6, 8, 10, 12, 14, 19)
                        if voxel == 6 or voxel == 8 or voxel == 10 or voxel == 12 or voxel == 14 or voxel == 19:
                            belongs_to_1 = 1
                    elif color1 == simulation.BALL:  # Ball (16 and 17 for stripe)
                        if voxel == 16 or voxel == 17:
                            belongs_to_1 = 1

                    # Check if voxel belongs to entity 2 (based on color2)
                    belongs_to_2 = 0
                    if color2 == simulation.BEETLE_BLUE:  # Blue beetle (5, 7, 9, 11, 13, 18)
                        if voxel == 5 or voxel == 7 or voxel == 9 or voxel == 11 or voxel == 13 or voxel == 18:
                            belongs_to_2 = 1
                    elif color2 == simulation.BEETLE_RED:  # Red beetle (6, 8, 10, 12, 14, 19)
                        if voxel == 6 or voxel == 8 or voxel == 10 or voxel == 12 or voxel == 14 or voxel == 19:
                            belongs_to_2 = 1
                    elif color2 == simulation.BALL:  # Ball (16 and 17 for stripe)
                        if voxel == 16 or voxel == 17:
                            belongs_to_2 = 1

                    # Update Y-ranges based on ownership
                    if belongs_to_1 == 1:
                        if gy < beetle1_y_min:
                            beetle1_y_min = gy
                        if gy > beetle1_y_max:
                            beetle1_y_max = gy

                    if belongs_to_2 == 1:
                        if gy < beetle2_y_min:
                            beetle2_y_min = gy
                        if gy > beetle2_y_max:
                            beetle2_y_max = gy

                # Check if Y-ranges overlap or are adjacent (variable tolerance based on voxel types)
                if beetle1_y_max >= 0 and beetle2_y_max >= 0:  # Both beetles present
                    # Use stricter tolerance (±5 voxels) for hook interior collisions to catch them earlier
                    # Use normal tolerance (±2 voxels) for beetle-beetle, tight (±0) for ball collisions
                    is_ball_involved = 0
                    if color1 == simulation.BALL or color2 == simulation.BALL:
                        is_ball_involved = 1

                    # Initialize tolerance (required by Taichi)
                    tolerance = 3  # Default: beetle-beetle (±3 voxels for earlier detection)
                    if has_hook_interior == 1:
                        tolerance = 5
                    elif is_ball_involved == 1:
                        tolerance = 0  # Ball needs tight collision - no early detection

                    if beetle1_y_min <= beetle2_y_max + tolerance and beetle2_y_min <= beetle1_y_max + tolerance:
                        collision = 1

    return collision

@ti.kernel
def place_beetle_rotated(world_x: ti.f32, world_y: ti.f32, world_z: ti.f32, rotation: ti.f32, color_type: ti.i32, front_body_height: ti.i32, back_body_height: ti.i32):
    """LEAN 1/3 SCALE BEETLE - Exaggerated legs & horn for animation"""
    center_x = int(world_x + simulation.n_grid / 2.0)
    center_z = int(world_z + simulation.n_grid / 2.0)
    base_y = int(world_y)  # Use dynamic Y position

    cos_r = ti.cos(rotation)
    sin_r = ti.sin(rotation)

    # ========== ABDOMEN (rear) - LEAN ellipse ==========
    # Height adjusted by back_body_height parameter (range 2-6)
    for dx in range(-12, -1):
        for dy in range(0, back_body_height):
            for dz in range(-3, 4):
                # Lean elliptical shape
                length_pos = (dx + 6.5) / 5.0
                center_height = float(back_body_height - 1) / 2.0
                height_factor = 1.0 - ((dy - center_height) / center_height) ** 2
                # Use minimum width for top/bottom layers (when height_factor = 0)
                width_at_height = 3.0 * ti.sqrt(ti.max(height_factor, 0.01)) if height_factor >= 0 else 0
                length_taper = 1.0 - (length_pos * length_pos)
                max_width = width_at_height * ti.sqrt(length_taper) if length_taper > 0 else 0

                if ti.abs(dz) <= max_width:
                    local_x = float(dx)
                    local_z = float(dz)
                    rotated_x = local_x * cos_r - local_z * sin_r
                    rotated_z = local_x * sin_r + local_z * cos_r
                    grid_x = center_x + int(ti.round(rotated_x))
                    grid_y = base_y + dy
                    grid_z = center_z + int(ti.round(rotated_z))
                    if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                        simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

    # ========== THORAX (middle) - LEAN with small pronotum bump ==========
    # Height adjusted by front_body_height parameter (range 2-6)
    for dx in range(-1, 2):
        for dy in range(0, front_body_height):
            for dz in range(-2, 3):
                # Lean rounded shape
                width_at_height = 2.0 if dy < 1 else 1.5
                length_factor = 1.0 - ti.abs(dx) / 1.5
                max_width = width_at_height * length_factor

                if ti.abs(dz) <= max_width:
                    local_x = float(dx)
                    local_z = float(dz)
                    rotated_x = local_x * cos_r - local_z * sin_r
                    rotated_z = local_x * sin_r + local_z * cos_r
                    grid_x = center_x + int(ti.round(rotated_x))
                    grid_y = base_y + dy
                    grid_z = center_z + int(ti.round(rotated_z))
                    if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                        simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

    # Small thoracic bump (removed - body too tall)

    # ========== HEAD (front) - LEAN ==========
    # 2 voxels long × 3 wide × 1 tall (reduced from 2)
    for dx in range(2, 4):
        for dy in range(0, 1):
            for dz in range(-1, 2):
                width_at_height = 1.5
                if ti.abs(dz) <= width_at_height:
                    local_x = float(dx)
                    local_z = float(dz)
                    rotated_x = local_x * cos_r - local_z * sin_r
                    rotated_z = local_x * sin_r + local_z * cos_r
                    grid_x = center_x + int(ti.round(rotated_x))
                    grid_y = base_y + dy
                    grid_z = center_z + int(ti.round(rotated_z))
                    if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                        simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

    # ========== EXAGGERATED Y-SHAPED HORN - EARLY SPLIT ==========
    # Shorter main shaft - 8 voxels, curves upward, splits earlier
    for i in range(8):
        dx = 3 + i
        height_curve = int(i * 0.3)
        dy = 1 + height_curve

        # Thick solid horn
        thickness = 2

        for dz in range(-thickness, thickness + 1):
            local_x = float(dx)
            local_z = float(dz)
            rotated_x = local_x * cos_r - local_z * sin_r
            rotated_z = local_x * sin_r + local_z * cos_r
            grid_x = center_x + int(ti.round(rotated_x))
            grid_y = 2 + dy
            grid_z = center_z + int(ti.round(rotated_z))
            if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

    # LONG LEFT PRONG - 9 voxels, starts earlier, wider at tip
    for i in range(9):
        dx = 9 + i  # Start at dx=9 (overlaps shaft for solid connection)
        dy = 3 + i  # Continue upward curve
        center_dz = -i  # Angles left

        # Widen at the tip (last 3 voxels)
        if i >= 6:
            # Extra wide tip for catching
            for dz_offset in range(-2, 3):
                dz = center_dz + dz_offset
                local_x = float(dx)
                local_z = float(dz)
                rotated_x = local_x * cos_r - local_z * sin_r
                rotated_z = local_x * sin_r + local_z * cos_r
                grid_x = center_x + int(ti.round(rotated_x))
                grid_y = 2 + dy
                grid_z = center_z + int(ti.round(rotated_z))
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = color_type
        else:
            # Normal thickness for base/middle
            for dz_offset in range(-1, 2):
                dz = center_dz + dz_offset
                local_x = float(dx)
                local_z = float(dz)
                rotated_x = local_x * cos_r - local_z * sin_r
                rotated_z = local_x * sin_r + local_z * cos_r
                grid_x = center_x + int(ti.round(rotated_x))
                grid_y = 2 + dy
                grid_z = center_z + int(ti.round(rotated_z))
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

    # LONG RIGHT PRONG - 9 voxels, starts earlier, wider at tip
    for i in range(9):
        dx = 9 + i  # Start at dx=9 (overlaps shaft for solid connection)
        dy = 3 + i  # Continue upward curve
        center_dz = i  # Angles right

        # Widen at the tip (last 3 voxels)
        if i >= 6:
            # Extra wide tip for catching
            for dz_offset in range(-2, 3):
                dz = center_dz + dz_offset
                local_x = float(dx)
                local_z = float(dz)
                rotated_x = local_x * cos_r - local_z * sin_r
                rotated_z = local_x * sin_r + local_z * cos_r
                grid_x = center_x + int(ti.round(rotated_x))
                grid_y = 2 + dy
                grid_z = center_z + int(ti.round(rotated_z))
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = color_type
        else:
            # Normal thickness for base/middle
            for dz_offset in range(-1, 2):
                dz = center_dz + dz_offset
                local_x = float(dx)
                local_z = float(dz)
                rotated_x = local_x * cos_r - local_z * sin_r
                rotated_z = local_x * sin_r + local_z * cos_r
                grid_x = center_x + int(ti.round(rotated_x))
                grid_y = 2 + dy
                grid_z = center_z + int(ti.round(rotated_z))
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

    # ========== EXAGGERATED LEGS (6 total) - THICK SEGMENTS ==========
    # Front legs (longest) - attach to front thorax
    for side in ti.static([-1, 1]):
        # COXA (thick body attachment) - 2 voxels
        for i in range(2):
            dx = 1
            dy = 1
            dz = side * (2 + i)
            for extra_y in range(2):  # Make it 2 voxels tall
                local_x = float(dx)
                local_z = float(dz)
                rotated_x = local_x * cos_r - local_z * sin_r
                rotated_z = local_x * sin_r + local_z * cos_r
                grid_x = center_x + int(ti.round(rotated_x))
                grid_y = 2 + dy + extra_y
                grid_z = center_z + int(ti.round(rotated_z))
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

        # FEMUR (thick upper leg) - 3 voxels extending out
        for i in range(3):
            dx = 1
            dy = 0
            dz = side * (4 + i)
            for extra_y in range(2):  # Make it 2 voxels tall
                local_x = float(dx)
                local_z = float(dz)
                rotated_x = local_x * cos_r - local_z * sin_r
                rotated_z = local_x * sin_r + local_z * cos_r
                grid_x = center_x + int(ti.round(rotated_x))
                grid_y = 2 + dy + extra_y
                grid_z = center_z + int(ti.round(rotated_z))
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

        # TIBIA (lower leg) - 2 voxels angling down
        for i in range(2):
            dx = 1 + i
            dy = -i
            dz = side * 7
            local_x = float(dx)
            local_z = float(dz)
            rotated_x = local_x * cos_r - local_z * sin_r
            rotated_z = local_x * sin_r + local_z * cos_r
            grid_x = center_x + int(ti.round(rotated_x))
            grid_y = 2 + dy
            grid_z = center_z + int(ti.round(rotated_z))
            if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

    # Middle legs - attach to middle thorax
    for side in ti.static([-1, 1]):
        # COXA - 2 voxels
        for i in range(2):
            dx = -1
            dy = 1
            dz = side * (2 + i)
            for extra_y in range(2):
                local_x = float(dx)
                local_z = float(dz)
                rotated_x = local_x * cos_r - local_z * sin_r
                rotated_z = local_x * sin_r + local_z * cos_r
                grid_x = center_x + int(ti.round(rotated_x))
                grid_y = 2 + dy + extra_y
                grid_z = center_z + int(ti.round(rotated_z))
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

        # FEMUR - 3 voxels
        for i in range(3):
            dx = -1
            dy = 0
            dz = side * (4 + i)
            for extra_y in range(2):
                local_x = float(dx)
                local_z = float(dz)
                rotated_x = local_x * cos_r - local_z * sin_r
                rotated_z = local_x * sin_r + local_z * cos_r
                grid_x = center_x + int(ti.round(rotated_x))
                grid_y = 2 + dy + extra_y
                grid_z = center_z + int(ti.round(rotated_z))
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

        # TIBIA - 2 voxels angling down
        for i in range(2):
            dx = -1 - i
            dy = -i
            dz = side * 7
            local_x = float(dx)
            local_z = float(dz)
            rotated_x = local_x * cos_r - local_z * sin_r
            rotated_z = local_x * sin_r + local_z * cos_r
            grid_x = center_x + int(ti.round(rotated_x))
            grid_y = 2 + dy
            grid_z = center_z + int(ti.round(rotated_z))
            if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

    # Rear legs - attach to front abdomen
    for side in ti.static([-1, 1]):
        # COXA - 2 voxels
        for i in range(2):
            dx = -4
            dy = 1
            dz = side * (2 + i)
            for extra_y in range(2):
                local_x = float(dx)
                local_z = float(dz)
                rotated_x = local_x * cos_r - local_z * sin_r
                rotated_z = local_x * sin_r + local_z * cos_r
                grid_x = center_x + int(ti.round(rotated_x))
                grid_y = 2 + dy + extra_y
                grid_z = center_z + int(ti.round(rotated_z))
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

        # FEMUR - 3 voxels angling back
        for i in range(3):
            dx = -4 - i
            dy = 0
            dz = side * (4 + i)
            for extra_y in range(2):
                local_x = float(dx)
                local_z = float(dz)
                rotated_x = local_x * cos_r - local_z * sin_r
                rotated_z = local_x * sin_r + local_z * cos_r
                grid_x = center_x + int(ti.round(rotated_x))
                grid_y = 2 + dy + extra_y
                grid_z = center_z + int(ti.round(rotated_z))
                if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

        # TIBIA - 2 voxels angling down
        for i in range(2):
            dx = -7 - i
            dy = -i
            dz = side * 7
            local_x = float(dx)
            local_z = float(dz)
            rotated_x = local_x * cos_r - local_z * sin_r
            rotated_z = local_x * sin_r + local_z * cos_r
            grid_x = center_x + int(ti.round(rotated_x))
            grid_y = 2 + dy
            grid_z = center_z + int(ti.round(rotated_z))
            if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid:
                simulation.voxel_type[grid_x, grid_y, grid_z] = color_type

# ========== PERFORMANCE OPTIMIZATION: GEOMETRY CACHING ==========
def generate_beetle_geometry(horn_shaft_len=12, horn_prong_len=5, front_body_height=4, back_body_height=6, body_length=12, body_width=7, leg_length=8, horn_type="rhino", stinger_curvature=0.0, tail_rotation_angle=0.0):
    """Generate beetle geometry separated into body and legs for animation

    Args:
        horn_shaft_len: Length of main horn shaft (default 12 voxels)
        horn_prong_len: Length of each Y-fork prong (default 5 voxels)
        horn_type: Type of horn to generate - "rhino" or "stag" (default "rhino")
        front_body_height: Height of thorax (front body) in voxel layers (default 4, range 2-6)
        back_body_height: Height of abdomen (back body) base layers (default 5, range 2-6)
        body_length: Length of abdomen in voxels (default 12, range 8-16)
        body_width: Width of body in voxels (default 7, range 5-9)
        leg_length: Total length of legs in voxels (default 10, range 6-14)
    """
    # Constrain all parameters to integers to prevent floating voxels from float arithmetic
    horn_shaft_len = round(horn_shaft_len)
    horn_prong_len = round(horn_prong_len)
    front_body_height = int(front_body_height)
    back_body_height = int(back_body_height)
    body_length = int(body_length)
    body_width = int(body_width)
    leg_length = int(leg_length)

    body_voxels = []
    hook_interior_flags = []  # Track which body voxels are hook interiors (for stag beetles)
    leg_voxels = []  # Will be organized as list of 6 legs, each containing voxel offsets

    # ABDOMEN (rear) - LEAN ellipse
    # Height directly controlled by back_body_height parameter
    # Width directly controlled by body_width parameter
    # Pre-calculate height-dependent values for performance
    center_height = (back_body_height - 1) / 2.0
    base_width_radius = body_width / 1.75  # Scale base radius proportionally (7→4.0, 5→2.86, 9→5.14)
    width_at_height_cache = []
    for dy in range(0, back_body_height):
        height_factor = 1.0 - ((dy - center_height) / center_height) ** 2
        if height_factor >= 0:
            width_at_height_cache.append(base_width_radius * math.sqrt(max(height_factor, 0.01)))
        else:
            width_at_height_cache.append(0.0)

    # Dynamic width range based on body_width parameter
    width_half = body_width // 2
    dz_min = -width_half
    dz_max = width_half + 1

    for dx in range(-body_length, -1):
        # Scale taper formula to body_length so abdomen grows properly
        taper_center = -body_length / 2.0  # Center of abdomen
        taper_scale = body_length / 2.5     # Scale factor for smooth taper
        length_pos = (dx - taper_center) / taper_scale
        length_taper = 1.0 - (length_pos * length_pos)
        if length_taper > 0:
            length_taper_sqrt = math.sqrt(length_taper)  # Calculate sqrt once per dx

            # SCORPION: Add Y offset that increases toward the rear (tail up)
            if horn_type == "scorpion":
                # Calculate how far back we are (0.0 at front, 1.0 at rear)
                rear_progress = (dx - (-1)) / (-body_length - (-1))  # 0.0 at dx=-1, 1.0 at dx=-body_length
                # Rear should be 4 voxels higher than front
                y_offset = int(rear_progress * 4)
            else:
                y_offset = 0

            for dy in range(0, back_body_height):
                width_at_height = width_at_height_cache[dy]
                if width_at_height > 0:
                    max_width = width_at_height * length_taper_sqrt
                    for dz in range(dz_min, dz_max):
                        if abs(dz) <= max_width:
                            body_voxels.append((dx, dy + y_offset, dz))

    # THORAX (middle) - LEAN with small pronotum bump, scales with body_width
    # Height directly controlled by front_body_height parameter
    # Width proportional to abdomen (slightly narrower for taper)
    thorax_width = max(3, int(body_width * 5 / 7))  # 71% of abdomen width, min 3
    thorax_half = thorax_width // 2
    thorax_dz_min = -thorax_half
    thorax_dz_max = thorax_half + 1
    thorax_base_width = thorax_width / 2.0  # Scale from constrained thorax_width to prevent floating voxels

    for dx in range(-1, 2):
        for dy in range(0, front_body_height):
            for dz in range(thorax_dz_min, thorax_dz_max):
                width_at_height = thorax_base_width if dy < 1 else thorax_base_width * 0.83
                length_factor = 1.0 - abs(dx) / 1.5
                max_width = width_at_height * length_factor
                if abs(dz) <= max_width:
                    body_voxels.append((dx, dy, dz))

    # HEAD (front) - LEAN, scales with body_width (narrower taper)
    head_width = max(2, int(body_width * 3 / 7))  # 43% of abdomen width, min 2
    head_half = head_width // 2
    head_dz_min = -head_half
    head_dz_max = head_half + 1
    head_base_width = head_width / 1.25  # Scale from constrained head_width to prevent floating voxels

    for dx in range(2, 4):
        for dy in range(0, 1):
            for dz in range(head_dz_min, head_dz_max):
                if abs(dz) <= head_base_width:
                    body_voxels.append((dx, dy, dz))

    # HORN GENERATION - Choose between rhino, stag, hercules, or scorpion (claws)
    if horn_type == "scorpion":
        # SCORPION - Dense, curved claws with thick arms (pedipalps)
        # Arms extend forward and upward from front of body
        arm_length = 6

        # LEFT ARM + CLAW
        # Dense arm that extends forward, upward, and outward
        for i in range(arm_length):
            dx = 2 + i
            dy_base = 2 + (i // 2)  # Rises from 2 to 4
            dz_base = -(i + 2)  # Extends leftward

            # Thicker 3x3 cross-section near body (first 3 segments), then taper to 2x2
            if i < 3:
                # 3x3 cross-section for thick base segments attached to body
                for dy_off in range(-1, 2):
                    for dz_off in range(-1, 2):
                        body_voxels.append((dx, dy_base + dy_off + 1, dz_base - dz_off))
            else:
                # 2x2 cross-section for remaining arm segments
                for dy_off in range(0, 2):
                    for dz_off in range(0, 2):
                        body_voxels.append((dx, dy_base + dy_off, dz_base - dz_off))

        # Left claw - dense curved pincers at end of arm
        claw_base_x = 2 + arm_length
        claw_base_z = -(arm_length + 2)

        # Upper pincer - curves down and inward naturally
        upper_pincer = [
            # Base (wide)
            (0, 5, 0), (0, 5, -1), (0, 6, 0), (0, 6, -1),
            (0, 4, 0), (0, 4, -1),
            # Mid (curving)
            (1, 5, 0), (1, 5, 1), (1, 4, 0), (1, 4, 1),
            (2, 5, 1), (2, 4, 1), (2, 4, 2),
            # Curve
            (3, 4, 2), (3, 4, 3), (3, 3, 2), (3, 3, 3),
            # Tip (narrower, pointing inward)
            (4, 4, 3), (4, 3, 3), (4, 3, 4),
            (5, 3, 4), (5, 3, 5),
        ]
        for offset in upper_pincer:
            body_voxels.append((claw_base_x + offset[0], offset[1], claw_base_z + offset[2]))

        # Lower pincer - curves up and inward naturally
        lower_pincer = [
            # Base (wide) - raised to Y=2 minimum to avoid ground clipping
            (0, 2, 0), (0, 2, -1), (0, 2, 0), (0, 2, -1),
            (0, 3, 0), (0, 3, -1),
            # Mid (curving)
            (1, 2, 0), (1, 2, 1), (1, 3, 0), (1, 3, 1),
            (2, 2, 1), (2, 3, 1), (2, 3, 2),
            # Curve
            (3, 3, 2), (3, 3, 3), (3, 4, 2), (3, 4, 3),
            # Tip (narrower, pointing inward)
            (4, 3, 3), (4, 4, 3), (4, 4, 4),
            (5, 4, 4), (5, 4, 5),
        ]
        for offset in lower_pincer:
            body_voxels.append((claw_base_x + offset[0], offset[1], claw_base_z + offset[2]))

        # RIGHT ARM + CLAW (mirror of left)
        # Dense arm that extends forward, upward, and outward
        for i in range(arm_length):
            dx = 2 + i
            dy_base = 2 + (i // 2)
            dz_base = (i + 2)  # Extends rightward

            # Thicker 3x3 cross-section near body (first 3 segments), then taper to 2x2
            if i < 3:
                # 3x3 cross-section for thick base segments attached to body
                for dy_off in range(-1, 2):
                    for dz_off in range(-1, 2):
                        body_voxels.append((dx, dy_base + dy_off + 1, dz_base + dz_off))
            else:
                # 2x2 cross-section for remaining arm segments
                for dy_off in range(0, 2):
                    for dz_off in range(0, 2):
                        body_voxels.append((dx, dy_base + dy_off, dz_base + dz_off))

        # Right claw - mirror the left claw
        claw_base_z_right = (arm_length + 2)

        # Upper pincer (mirrored)
        for offset in upper_pincer:
            body_voxels.append((claw_base_x + offset[0], offset[1], claw_base_z_right - offset[2]))

        # Lower pincer (mirrored)
        for offset in lower_pincer:
            body_voxels.append((claw_base_x + offset[0], offset[1], claw_base_z_right - offset[2]))

        # SCORPION STINGER/TAIL - Curved tail extending from rear
        # NOTE: tail_rotation_angle is now applied at runtime, not baked into geometry
        stinger_voxels = generate_scorpion_stinger(
            shaft_len=horn_shaft_len,
            prong_len=horn_prong_len,
            body_length=body_length,
            back_body_height=back_body_height,
            stinger_curvature=stinger_curvature,
            tail_rotation_angle=0.0  # Runtime rotation applied in rendering kernel
        )
        body_voxels.extend(stinger_voxels)
    elif horn_type == "stag":
        # STAG BEETLE PINCERS - Horizontal mandibles
        pincer_voxels = generate_stag_pincers(horn_shaft_len - 2, horn_prong_len)
        # Extract voxels and hook interior flags (4-tuples: x, y, z, is_hook)
        for voxel_data in pincer_voxels:
            if len(voxel_data) == 4:  # New format with hook interior flag
                x, y, z, is_hook = voxel_data
                body_voxels.append((x, y, z))
                hook_interior_flags.append(is_hook)
            else:  # Old format (backward compatibility)
                body_voxels.append(voxel_data)
                hook_interior_flags.append(0)
    elif horn_type == "hercules":
        # HERCULES BEETLE HORNS - Vertical dual-jaw pincer system
        hercules_voxels = generate_hercules_horns(horn_shaft_len, horn_prong_len, front_body_height, back_body_height)
        body_voxels.extend(hercules_voxels)
    elif horn_type == "atlas":
        # ATLAS BEETLE - Three-horn system (two stationary + one movable)
        # PRONOTUM HORNS (stationary, top): Two upward-curving horns from shoulders
        # Controlled by prong_len slider
        pronotum_horns = generate_atlas_pronotum_horns(horn_prong_len)
        body_voxels.extend(pronotum_horns)

        # CEPHALIC HORN (movable, bottom): Forward-projecting horn with upward curve
        # Controlled by shaft_len slider and R/Y keys (pitch control)
        # This horn will be rotated in place_animated_beetle based on horn_pitch
        cephalic_horn = generate_atlas_cephalic_horn(horn_shaft_len)
        body_voxels.extend(cephalic_horn)
    else:
        # RHINOCEROS BEETLE HORN - Y-shaped vertical horn
        # Main shaft with overlapping layers
        shaft_segments = round(horn_shaft_len)

        # Add extra meaty base layer at attachment point (x=2, x=3, x=4, x=5)
        # Makes the neck thicker where it connects to body, tapers toward shaft
        for dx_base in range(2, 6):  # x=2, x=3, x=4, x=5 (four layers deep)
            # Front layers start higher, back layer starts at y=0
            if dx_base == 2:
                y_start = 0
            elif dx_base == 3:
                y_start = 1
            elif dx_base == 4:
                y_start = 1
            else:  # x=5 - extra front layer for thickness
                y_start = 1
            for dy_base in range(y_start, 3):  # 3 voxels tall at base
                # Taper: wider at bottom (y=0,1), narrower at top (y=2)
                # x=4 and x=5 layers are narrower (just the shaft width)
                if dx_base >= 4:
                    z_range = range(-1, 2)  # 3 voxels wide (matches shaft)
                elif dy_base < 2:
                    z_range = range(-2, 3)  # 5 voxels wide at bottom
                else:
                    z_range = range(-1, 2)  # 3 voxels wide at top (tapered)
                for dz_base in z_range:
                    body_voxels.append((dx_base, dy_base, dz_base))

        for i in range(shaft_segments):
            dx = 3 + i
            height_curve = int(i * 0.3)
            dy = 1 + height_curve
            thickness = 1  # Changed from 2 to 1 for 3-voxel width
            for dz in range(-thickness, thickness + 1):
                # Add voxel at current height
                body_voxels.append((dx, dy, dz))
                # Add overlapping voxel below (unless at bottom)
                if dy > 1:
                    body_voxels.append((dx, dy - 1, dz))
            # Add second layer behind the shaft at the base (first 3 segments)
            # This makes the base 2 voxels deep in X for better collision
            if i < 3:
                for dz in range(-thickness, thickness + 1):
                    body_voxels.append((dx - 1, dy, dz))
                    if dy > 1:
                        body_voxels.append((dx - 1, dy - 1, dz))

        # Y-fork - Left prong with overlapping layers
        prong_segments = round(horn_prong_len)
        prong_start_x = 3 + shaft_segments  # Start where shaft ends
        prong_start_y = 1 + int(shaft_segments * 0.3)  # Continue from shaft height curve

        for i in range(prong_segments):
            dx = prong_start_x + i
            dy = prong_start_y + i
            center_dz = -i
            # Use range(-1, 1) to create 2-voxel width prong on left side
            for dz_offset in range(-1, 1):
                dz = center_dz + dz_offset
                # Add voxel at current height
                body_voxels.append((dx, dy, dz))
                # Add overlapping voxel below
                body_voxels.append((dx, dy - 1, dz))

        # Right prong with overlapping layers
        for i in range(prong_segments):
            dx = prong_start_x + i
            dy = prong_start_y + i
            center_dz = i
            # Use range(0, 2) to create 2-voxel width prong on right side
            for dz_offset in range(0, 2):
                dz = center_dz + dz_offset
                # Add voxel at current height
                body_voxels.append((dx, dy, dz))
                # Add overlapping voxel below
                body_voxels.append((dx, dy - 1, dz))

    # IMPROVED LEGS (6 for beetles, 8 for scorpion) - Separated for animation
    # Leg order: [front_left, front_right, middle_left, middle_right, rear_left, rear_right, rear2_left*, rear2_right*]
    # *Only for scorpion type

    # Helper function to calculate leg segment lengths with optional length multiplier
    def calc_leg_segments(base_leg_length, length_multiplier=1.0):
        """Calculate coxa, femur, tibia lengths for a leg with optional multiplier"""
        total_length = int(base_leg_length * length_multiplier)
        coxa = max(1, total_length // 5)  # ~20%
        femur = max(1, (total_length * 2) // 5)  # ~40%
        tibia = max(1, total_length - coxa - femur)  # Remainder ensures exact total
        return coxa, femur, tibia

    # For scorpion: graduated leg lengths (front shortest, rear longest)
    # For beetles: all legs same length (multiplier = 1.0)
    if horn_type == "scorpion":
        leg_multipliers = [1.0, 1.0, 1.1, 1.1, 1.2, 1.2, 1.3, 1.3]  # Graduated lengths
    else:
        leg_multipliers = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]  # All same length for beetles

    # Calculate segment lengths for front legs (legs 0, 1) - use multiplier[0]
    coxa_len, femur_len, tibia_len = calc_leg_segments(leg_length, leg_multipliers[0])
    coxa_start = 3
    femur_start = coxa_start + coxa_len
    tibia_start = femur_start + femur_len

    # Front left leg (leg 0) - MOVED BACK 1 VOXEL
    front_left = []
    front_left_tips = []  # Separate tips for black coloring
    side = -1
    # COXA
    for i in range(coxa_len):
        for extra_y in range(2):
            front_left.append((1, 1 + extra_y, side * (coxa_start + i)))
    # FEMUR
    for i in range(femur_len):
        for extra_y in range(2):
            front_left.append((1, 0 + extra_y, side * (femur_start + i)))
    # TIBIA (tips - will be rendered black)
    for i in range(tibia_len):
        tip_x = 1 + min(i // 2, 2)  # Extend forward gradually
        front_left_tips.append((tip_x, 0, side * (tibia_start + i)))
    leg_voxels.append(front_left)

    # Front right leg (leg 1) - MOVED BACK 1 VOXEL
    front_right = []
    front_right_tips = []
    side = 1
    for i in range(coxa_len):
        for extra_y in range(2):
            front_right.append((1, 1 + extra_y, side * (coxa_start + i)))
    for i in range(femur_len):
        for extra_y in range(2):
            front_right.append((1, 0 + extra_y, side * (femur_start + i)))
    for i in range(tibia_len):
        tip_x = 1 + min(i // 2, 2)
        front_right_tips.append((tip_x, 0, side * (tibia_start + i)))
    leg_voxels.append(front_right)

    # Calculate segment lengths for middle legs (legs 2, 3) - use multiplier[2]
    coxa_len, femur_len, tibia_len = calc_leg_segments(leg_length, leg_multipliers[2])
    coxa_start = 3
    femur_start = coxa_start + coxa_len
    tibia_start = femur_start + femur_len

    # Middle left leg (leg 2) - MOVED BACK 1 VOXEL
    middle_left = []
    middle_left_tips = []
    side = -1
    for i in range(coxa_len):
        for extra_y in range(2):
            middle_left.append((-3, 1 + extra_y, side * (coxa_start + i)))
    for i in range(femur_len):
        for extra_y in range(2):
            middle_left.append((-3, 0 + extra_y, side * (femur_start + i)))
    for i in range(tibia_len):
        tip_x = -3 - min(i // 2, 2)  # Extend backward gradually
        middle_left_tips.append((tip_x, 0, side * (tibia_start + i)))
    leg_voxels.append(middle_left)

    # Middle right leg (leg 3) - MOVED BACK 1 VOXEL
    middle_right = []
    middle_right_tips = []
    side = 1
    for i in range(coxa_len):
        for extra_y in range(2):
            middle_right.append((-3, 1 + extra_y, side * (coxa_start + i)))
    for i in range(femur_len):
        for extra_y in range(2):
            middle_right.append((-3, 0 + extra_y, side * (femur_start + i)))
    for i in range(tibia_len):
        tip_x = -3 - min(i // 2, 2)
        middle_right_tips.append((tip_x, 0, side * (tibia_start + i)))
    leg_voxels.append(middle_right)

    # Calculate segment lengths for rear legs (legs 4, 5) - use multiplier[4]
    coxa_len, femur_len, tibia_len = calc_leg_segments(leg_length, leg_multipliers[4])
    coxa_start = 3
    femur_start = coxa_start + coxa_len
    tibia_start = femur_start + femur_len

    # Rear left leg (leg 4) - Position proportional to body length - MOVED BACK 1 VOXEL
    rear_leg_attach_x = -(body_length // 2) - 1  # Proportional to abdomen length
    rear_left = []
    rear_left_tips = []
    side = -1
    for i in range(coxa_len):
        for extra_y in range(2):
            rear_left.append((rear_leg_attach_x, 1 + extra_y, side * (coxa_start + i)))
    for i in range(femur_len):
        for extra_y in range(2):
            rear_left.append((rear_leg_attach_x - i, 0 + extra_y, side * (femur_start + i)))
    for i in range(tibia_len):
        tip_x = rear_leg_attach_x - femur_len - min(i // 2, 2)  # Continue backward
        rear_left_tips.append((tip_x, 0, side * (tibia_start + i)))
    leg_voxels.append(rear_left)

    # Rear right leg (leg 5) - Position proportional to body length
    rear_right = []
    rear_right_tips = []
    side = 1
    for i in range(coxa_len):
        for extra_y in range(2):
            rear_right.append((rear_leg_attach_x, 1 + extra_y, side * (coxa_start + i)))
    for i in range(femur_len):
        for extra_y in range(2):
            rear_right.append((rear_leg_attach_x - i, 0 + extra_y, side * (femur_start + i)))
    for i in range(tibia_len):
        tip_x = rear_leg_attach_x - femur_len - min(i // 2, 2)
        rear_right_tips.append((tip_x, 0, side * (tibia_start + i)))
    leg_voxels.append(rear_right)

    # SCORPION ONLY: Add two extra rear legs (legs 6, 7) - furthest back, longest, raised attachment
    if horn_type == "scorpion":
        # Calculate segment lengths for extra rear legs (legs 6, 7) - use multiplier[6]
        coxa_len, femur_len, tibia_len = calc_leg_segments(leg_length, leg_multipliers[6])
        coxa_start = 3
        femur_start = coxa_start + coxa_len
        tibia_start = femur_start + femur_len

        # Rear-2 left leg (leg 6) - Position furthest back, attach higher (Y=2)
        rear2_leg_attach_x = -(body_length * 3 // 4) - 2  # 75% back along abdomen, plus 2 voxels
        rear2_left = []
        rear2_left_tips = []
        side = -1
        # Add extra connector voxels to bridge from body to leg attachment
        for bridge_z in range(1, coxa_start + 1):  # Z from 1 to leg start (skip z=0 next to body)
            for extra_y in range(3):  # Y=2,3,4 - connects to tilted body
                rear2_left.append((rear2_leg_attach_x, 2 + extra_y, side * bridge_z))
        for i in range(coxa_len):
            for extra_y in range(2):
                rear2_left.append((rear2_leg_attach_x, 2 + extra_y, side * (coxa_start + i)))  # Y=2 (raised)
        for i in range(femur_len):
            for extra_y in range(2):
                rear2_left.append((rear2_leg_attach_x - i, 1 + extra_y, side * (femur_start + i)))  # Y=1
        for i in range(tibia_len):
            tip_x = rear2_leg_attach_x - femur_len - min(i // 2, 2)
            rear2_left_tips.append((tip_x, 0, side * (tibia_start + i)))
        leg_voxels.append(rear2_left)

        # Rear-2 right leg (leg 7) - Position furthest back, attach higher (Y=2)
        rear2_right = []
        rear2_right_tips = []
        side = 1
        # Add extra connector voxels to bridge from body to leg attachment
        for bridge_z in range(1, coxa_start + 1):  # Z from 1 to leg start (skip z=0 next to body)
            for extra_y in range(3):  # Y=2,3,4 - connects to tilted body
                rear2_right.append((rear2_leg_attach_x, 2 + extra_y, side * bridge_z))
        for i in range(coxa_len):
            for extra_y in range(2):
                rear2_right.append((rear2_leg_attach_x, 2 + extra_y, side * (coxa_start + i)))  # Y=2 (raised)
        for i in range(femur_len):
            for extra_y in range(2):
                rear2_right.append((rear2_leg_attach_x - i, 1 + extra_y, side * (femur_start + i)))  # Y=1
        for i in range(tibia_len):
            tip_x = rear2_leg_attach_x - femur_len - min(i // 2, 2)
            rear2_right_tips.append((tip_x, 0, side * (tibia_start + i)))
        leg_voxels.append(rear2_right)

        # Collect all leg tips into single list (8 legs for scorpion)
        leg_tips = [front_left_tips, front_right_tips, middle_left_tips,
                    middle_right_tips, rear_left_tips, rear_right_tips,
                    rear2_left_tips, rear2_right_tips]
    else:
        # Collect all leg tips into single list (6 legs for beetles)
        leg_tips = [front_left_tips, front_right_tips, middle_left_tips,
                    middle_right_tips, rear_left_tips, rear_right_tips]

    # Ensure hook_interior_flags matches body_voxels length (fill with 0s if needed)
    while len(hook_interior_flags) < len(body_voxels):
        hook_interior_flags.append(0)

    # OPTIMIZATION: Pre-compute voxel metadata flags to eliminate per-frame conditional logic
    # Compute stripe, horn_tip, and very_tip flags for each voxel based on horn type
    stripe_flags = []
    horn_tip_flags = []
    very_tip_flags = []

    # Map horn_type string to horn_type_id for consistency with placement logic
    horn_type_id_map = {"rhino": 0, "stag": 1, "hercules": 2, "scorpion": 3, "atlas": 4}
    horn_type_id = horn_type_id_map.get(horn_type, 0)

    for i, (dx, dy, dz) in enumerate(body_voxels):
        # Stripe detection: top centerline of body (not horn, not scorpion)
        is_stripe = 0
        if horn_type_id != 3 and dx < 3 and dy >= 3 and abs(dz) <= 1.0:
            is_stripe = 1
        stripe_flags.append(is_stripe)

        # Horn tip detection: varies by beetle type
        is_horn_tip = 0
        if horn_type_id == 1:  # Stag
            if dx >= 9:
                is_horn_tip = 1
        elif horn_type_id == 2:  # Hercules
            if dx >= 10:
                is_horn_tip = 1
        elif horn_type_id == 3:  # Scorpion
            if abs(dz) > 2:  # Claws
                if dx >= 1:
                    is_horn_tip = 1
            else:  # Tail/stinger
                if dx >= 1:
                    is_horn_tip = 1
        elif horn_type_id == 4:  # Atlas
            if dx >= 11:
                is_horn_tip = 1
        else:  # Rhino (horn_type_id == 0)
            if dx >= 13:
                is_horn_tip = 1
        horn_tip_flags.append(is_horn_tip)

        # Very tip detection: only for scorpion dual-layer tip coloring
        is_very_tip = 0
        if horn_type_id == 3:  # Scorpion only
            if abs(dz) > 2:  # Claws
                if dx >= 12:
                    is_very_tip = 1
            else:  # Stinger
                if dx >= 3:
                    is_very_tip = 1
        very_tip_flags.append(is_very_tip)

    return body_voxels, leg_voxels, leg_tips, hook_interior_flags, stripe_flags, horn_tip_flags, very_tip_flags

# Generate separate geometry for blue and red beetles (start with same default params)
BLUE_BODY, BLUE_LEGS, BLUE_LEG_TIPS, BLUE_HOOK_FLAGS, BLUE_STRIPE_FLAGS, BLUE_HORN_TIP_FLAGS, BLUE_VERY_TIP_FLAGS = generate_beetle_geometry()
RED_BODY, RED_LEGS, RED_LEG_TIPS, RED_HOOK_FLAGS, RED_STRIPE_FLAGS, RED_HORN_TIP_FLAGS, RED_VERY_TIP_FLAGS = generate_beetle_geometry()
print(f"Blue beetle geometry cached: {len(BLUE_BODY)} body voxels + {sum(len(leg) for leg in BLUE_LEGS)} leg voxels + {sum(len(tips) for tips in BLUE_LEG_TIPS)} tip voxels")
print(f"Red beetle geometry cached: {len(RED_BODY)} body voxels + {sum(len(leg) for leg in RED_LEGS)} leg voxels + {sum(len(tips) for tips in RED_LEG_TIPS)} tip voxels")

# Create OVERSIZED Taichi fields for body geometry cache (to allow dynamic resizing)
# Hercules beetle with max settings can reach ~606 voxels
# Scorpion with Y-overlap for gap-free tail can reach ~2000 voxels at max length (3x multiplier in Y)
MAX_BODY_VOXELS = 2500

# Separate body cache fields for blue beetle
blue_body_cache_size = ti.field(ti.i32, shape=())
blue_body_cache_size[None] = len(BLUE_BODY)
blue_body_cache_x = ti.field(ti.i32, shape=MAX_BODY_VOXELS)
blue_body_cache_y = ti.field(ti.i32, shape=MAX_BODY_VOXELS)
blue_body_cache_z = ti.field(ti.i32, shape=MAX_BODY_VOXELS)
blue_body_hook_flags = ti.field(ti.i32, shape=MAX_BODY_VOXELS)  # Hook interior flags for stag beetles
# OPTIMIZATION: Pre-computed voxel metadata to eliminate per-frame conditional logic
blue_body_stripe_flags = ti.field(ti.i32, shape=MAX_BODY_VOXELS)  # 1 if voxel is racing stripe
blue_body_horn_tip_flags = ti.field(ti.i32, shape=MAX_BODY_VOXELS)  # 1 if voxel is horn tip
blue_body_very_tip_flags = ti.field(ti.i32, shape=MAX_BODY_VOXELS)  # 1 if voxel is very tip (scorpion)

# Separate body cache fields for red beetle
red_body_cache_size = ti.field(ti.i32, shape=())
red_body_cache_size[None] = len(RED_BODY)
red_body_cache_x = ti.field(ti.i32, shape=MAX_BODY_VOXELS)
red_body_cache_y = ti.field(ti.i32, shape=MAX_BODY_VOXELS)
red_body_cache_z = ti.field(ti.i32, shape=MAX_BODY_VOXELS)
red_body_hook_flags = ti.field(ti.i32, shape=MAX_BODY_VOXELS)  # Hook interior flags for stag beetles
# OPTIMIZATION: Pre-computed voxel metadata to eliminate per-frame conditional logic
red_body_stripe_flags = ti.field(ti.i32, shape=MAX_BODY_VOXELS)  # 1 if voxel is racing stripe
red_body_horn_tip_flags = ti.field(ti.i32, shape=MAX_BODY_VOXELS)  # 1 if voxel is horn tip
red_body_very_tip_flags = ti.field(ti.i32, shape=MAX_BODY_VOXELS)  # 1 if voxel is very tip (scorpion)

# Create OVERSIZED Taichi fields for leg geometry cache (6 legs)
# Store all leg voxels flattened with offsets to know where each leg starts
# Max leg length=14: coxa(3) + femur(6) + tibia(6) = 15 voxels per segment
# Each segment has 2 height layers, so 15*2 = 30 voxels max per leg
# 8 legs × 30 voxels = 240 voxels max (scorpion has 8 legs)
MAX_LEG_VOXELS = 240

# Separate leg cache fields for blue beetle
blue_leg_cache_x = ti.field(ti.i32, shape=MAX_LEG_VOXELS)
blue_leg_cache_y = ti.field(ti.i32, shape=MAX_LEG_VOXELS)
blue_leg_cache_z = ti.field(ti.i32, shape=MAX_LEG_VOXELS)
blue_leg_start_idx = ti.field(ti.i32, shape=8)
blue_leg_end_idx = ti.field(ti.i32, shape=8)

# Separate leg cache fields for red beetle
red_leg_cache_x = ti.field(ti.i32, shape=MAX_LEG_VOXELS)
red_leg_cache_y = ti.field(ti.i32, shape=MAX_LEG_VOXELS)
red_leg_cache_z = ti.field(ti.i32, shape=MAX_LEG_VOXELS)
red_leg_start_idx = ti.field(ti.i32, shape=8)
red_leg_end_idx = ti.field(ti.i32, shape=8)

# Create OVERSIZED Taichi fields for leg tip geometry cache
# Max leg length=14: tibia tips per leg × 8 legs
# Increased to 65 to provide headroom for scorpion's 8 legs with thicker tip rendering
MAX_LEG_TIP_VOXELS = 65

# Separate leg tip cache fields for blue beetle
blue_leg_tip_cache_x = ti.field(ti.i32, shape=MAX_LEG_TIP_VOXELS)
blue_leg_tip_cache_y = ti.field(ti.i32, shape=MAX_LEG_TIP_VOXELS)
blue_leg_tip_cache_z = ti.field(ti.i32, shape=MAX_LEG_TIP_VOXELS)
blue_leg_tip_start_idx = ti.field(ti.i32, shape=8)
blue_leg_tip_end_idx = ti.field(ti.i32, shape=8)

# Separate leg tip cache fields for red beetle
red_leg_tip_cache_x = ti.field(ti.i32, shape=MAX_LEG_TIP_VOXELS)
red_leg_tip_cache_y = ti.field(ti.i32, shape=MAX_LEG_TIP_VOXELS)
red_leg_tip_cache_z = ti.field(ti.i32, shape=MAX_LEG_TIP_VOXELS)
red_leg_tip_start_idx = ti.field(ti.i32, shape=8)
red_leg_tip_end_idx = ti.field(ti.i32, shape=8)

# Temporary compatibility aliases: old variable names point to blue beetle cache
# (This allows old rendering functions to work while we transition to dual-beetle UI)
body_cache_size = blue_body_cache_size
body_cache_x = blue_body_cache_x
body_cache_y = blue_body_cache_y
body_cache_z = blue_body_cache_z
leg_cache_x = blue_leg_cache_x
leg_cache_y = blue_leg_cache_y
leg_cache_z = blue_leg_cache_z
leg_start_idx = blue_leg_start_idx
leg_end_idx = blue_leg_end_idx
leg_tip_cache_x = blue_leg_tip_cache_x
leg_tip_cache_y = blue_leg_tip_cache_y
leg_tip_cache_z = blue_leg_tip_cache_z
leg_tip_start_idx = blue_leg_tip_start_idx
leg_tip_end_idx = blue_leg_tip_end_idx

# Collision detection fields - store occupied voxels for each beetle (GPU-resident)
# Each beetle can occupy up to ~1600 voxels in a 40x40 area
MAX_OCCUPIED_VOXELS = 1600
beetle1_occupied_x = ti.field(ti.i32, shape=MAX_OCCUPIED_VOXELS)
beetle1_occupied_z = ti.field(ti.i32, shape=MAX_OCCUPIED_VOXELS)
beetle1_occupied_count = ti.field(ti.i32, shape=())

beetle2_occupied_x = ti.field(ti.i32, shape=MAX_OCCUPIED_VOXELS)
beetle2_occupied_z = ti.field(ti.i32, shape=MAX_OCCUPIED_VOXELS)
beetle2_occupied_count = ti.field(ti.i32, shape=())

# Collision point calculation fields
collision_point_x = ti.field(ti.f32, shape=())
collision_point_y = ti.field(ti.f32, shape=())
collision_point_z = ti.field(ti.f32, shape=())
collision_contact_count = ti.field(ti.i32, shape=())
collision_has_horn_tips = ti.field(ti.i32, shape=())  # 1 if horn tip voxels involved, 0 otherwise
collision_has_hook_interiors = ti.field(ti.i32, shape=())  # 1 if stag hook interior voxels involved, 0 otherwise

# OPTIMIZATION: Spatial hash for O(N+M) collision point calculation instead of O(N×M)
# Hash grid covers the arena (128x128 should be sufficient for 256-grid world)
collision_spatial_hash = ti.field(dtype=ti.i32, shape=(128, 128))

# Edge tipping calculation fields
edge_tipping_vy = ti.field(ti.f32, shape=())  # Downward velocity to apply
edge_tipping_pitch_vel = ti.field(ti.f32, shape=())  # Pitch angular velocity
edge_tipping_roll_vel = ti.field(ti.f32, shape=())  # Roll angular velocity

# Dirty voxel tracking for efficient clearing
# Instead of scanning 250K voxels, track only the ~600 voxels we actually place
# Max voxels per beetle: body(600) + legs(180) + tips(40) = 820
# Two beetles: 1640, use 2000 for safety
MAX_DIRTY_VOXELS = 5000  # Increased to handle 2 beetles at max size (back_body_height=8, with horns/tail/legs)
dirty_voxel_x = ti.field(ti.i32, shape=MAX_DIRTY_VOXELS)
dirty_voxel_y = ti.field(ti.i32, shape=MAX_DIRTY_VOXELS)
dirty_voxel_z = ti.field(ti.i32, shape=MAX_DIRTY_VOXELS)
dirty_voxel_count = ti.field(ti.i32, shape=())

# Animation parameters
WALK_CYCLE_SPEED = 1.0  # Radians per second at normal walk speed

# Copy blue beetle body geometry to GPU
for i, (dx, dy, dz) in enumerate(BLUE_BODY):
    blue_body_cache_x[i] = dx
    blue_body_cache_y[i] = dy
    blue_body_cache_z[i] = dz
    # Copy hook interior flags (0 if not available)
    if i < len(BLUE_HOOK_FLAGS):
        blue_body_hook_flags[i] = BLUE_HOOK_FLAGS[i]
    else:
        blue_body_hook_flags[i] = 0
    # Copy pre-computed metadata flags
    if i < len(BLUE_STRIPE_FLAGS):
        blue_body_stripe_flags[i] = BLUE_STRIPE_FLAGS[i]
        blue_body_horn_tip_flags[i] = BLUE_HORN_TIP_FLAGS[i]
        blue_body_very_tip_flags[i] = BLUE_VERY_TIP_FLAGS[i]
    else:
        blue_body_stripe_flags[i] = 0
        blue_body_horn_tip_flags[i] = 0
        blue_body_very_tip_flags[i] = 0

# Copy blue beetle leg geometry to GPU
offset = 0
for leg_id, leg_voxels in enumerate(BLUE_LEGS):
    blue_leg_start_idx[leg_id] = offset
    for i, (dx, dy, dz) in enumerate(leg_voxels):
        blue_leg_cache_x[offset + i] = dx
        blue_leg_cache_y[offset + i] = dy
        blue_leg_cache_z[offset + i] = dz
    offset += len(leg_voxels)
    blue_leg_end_idx[leg_id] = offset

# Copy blue beetle leg tip geometry to GPU
offset = 0
for leg_id, leg_tip_voxels in enumerate(BLUE_LEG_TIPS):
    blue_leg_tip_start_idx[leg_id] = offset
    for i, (dx, dy, dz) in enumerate(leg_tip_voxels):
        blue_leg_tip_cache_x[offset + i] = dx
        blue_leg_tip_cache_y[offset + i] = dy
        blue_leg_tip_cache_z[offset + i] = dz
    offset += len(leg_tip_voxels)
    blue_leg_tip_end_idx[leg_id] = offset

# Copy red beetle body geometry to GPU
for i, (dx, dy, dz) in enumerate(RED_BODY):
    red_body_cache_x[i] = dx
    red_body_cache_y[i] = dy
    red_body_cache_z[i] = dz
    # Copy hook interior flags (0 if not available)
    if i < len(RED_HOOK_FLAGS):
        red_body_hook_flags[i] = RED_HOOK_FLAGS[i]
    else:
        red_body_hook_flags[i] = 0
    # Copy pre-computed metadata flags
    if i < len(RED_STRIPE_FLAGS):
        red_body_stripe_flags[i] = RED_STRIPE_FLAGS[i]
        red_body_horn_tip_flags[i] = RED_HORN_TIP_FLAGS[i]
        red_body_very_tip_flags[i] = RED_VERY_TIP_FLAGS[i]
    else:
        red_body_stripe_flags[i] = 0
        red_body_horn_tip_flags[i] = 0
        red_body_very_tip_flags[i] = 0

# Copy red beetle leg geometry to GPU
offset = 0
for leg_id, leg_voxels in enumerate(RED_LEGS):
    red_leg_start_idx[leg_id] = offset
    for i, (dx, dy, dz) in enumerate(leg_voxels):
        red_leg_cache_x[offset + i] = dx
        red_leg_cache_y[offset + i] = dy
        red_leg_cache_z[offset + i] = dz
    offset += len(leg_voxels)
    red_leg_end_idx[leg_id] = offset

# Copy red beetle leg tip geometry to GPU
offset = 0
for leg_id, leg_tip_voxels in enumerate(RED_LEG_TIPS):
    red_leg_tip_start_idx[leg_id] = offset
    for i, (dx, dy, dz) in enumerate(leg_tip_voxels):
        red_leg_tip_cache_x[offset + i] = dx
        red_leg_tip_cache_y[offset + i] = dy
        red_leg_tip_cache_z[offset + i] = dz
    offset += len(leg_tip_voxels)
    red_leg_tip_end_idx[leg_id] = offset

# Function to rebuild blue beetle geometry with new parameters
def rebuild_blue_beetle(shaft_len, prong_len, front_body_height=4, back_body_height=6, body_length=12, body_width=7, leg_length=8, horn_type="rhino", stinger_curvature=0.0, tail_rotation_angle=0.0):
    """Rebuild blue beetle geometry cache with new horn, body, and leg parameters"""
    global BLUE_BODY, BLUE_LEGS, BLUE_LEG_TIPS, BLUE_HOOK_FLAGS, BLUE_STRIPE_FLAGS, BLUE_HORN_TIP_FLAGS, BLUE_VERY_TIP_FLAGS

    # Generate new geometry
    BLUE_BODY, BLUE_LEGS, BLUE_LEG_TIPS, BLUE_HOOK_FLAGS, BLUE_STRIPE_FLAGS, BLUE_HORN_TIP_FLAGS, BLUE_VERY_TIP_FLAGS = generate_beetle_geometry(shaft_len, prong_len, front_body_height, back_body_height, body_length, body_width, leg_length, horn_type, stinger_curvature, tail_rotation_angle)

    # Update beetle's hook interior flags
    beetle_blue.body_hook_interior_flags = list(BLUE_HOOK_FLAGS)

    # Update body cache
    if len(BLUE_BODY) > MAX_BODY_VOXELS:
        print(f"WARNING: Blue beetle body voxels ({len(BLUE_BODY)}) exceeds max ({MAX_BODY_VOXELS})!")
        return

    # Clear old cache entries to prevent voxel pollution when switching beetle types
    old_size = blue_body_cache_size[None]
    new_size = len(BLUE_BODY)

    # Write new geometry
    for i, (dx, dy, dz) in enumerate(BLUE_BODY):
        blue_body_cache_x[i] = dx
        blue_body_cache_y[i] = dy
        blue_body_cache_z[i] = dz
        # Copy pre-computed metadata flags
        if i < len(BLUE_STRIPE_FLAGS):
            blue_body_stripe_flags[i] = BLUE_STRIPE_FLAGS[i]
            blue_body_horn_tip_flags[i] = BLUE_HORN_TIP_FLAGS[i]
            blue_body_very_tip_flags[i] = BLUE_VERY_TIP_FLAGS[i]
            blue_body_hook_flags[i] = BLUE_HOOK_FLAGS[i]
        else:
            blue_body_stripe_flags[i] = 0
            blue_body_horn_tip_flags[i] = 0
            blue_body_very_tip_flags[i] = 0
            blue_body_hook_flags[i] = 0

    # Zero out any leftover voxels from previous geometry
    for i in range(new_size, old_size):
        blue_body_cache_x[i] = 0
        blue_body_cache_y[i] = 0
        blue_body_cache_z[i] = 0
        blue_body_stripe_flags[i] = 0
        blue_body_horn_tip_flags[i] = 0
        blue_body_very_tip_flags[i] = 0
        blue_body_hook_flags[i] = 0

    # Update size LAST to prevent race conditions
    blue_body_cache_size[None] = new_size

    # Update leg cache
    offset = 0
    for leg_id, leg_voxels in enumerate(BLUE_LEGS):
        blue_leg_start_idx[leg_id] = offset
        for i, (dx, dy, dz) in enumerate(leg_voxels):
            blue_leg_cache_x[offset + i] = dx
            blue_leg_cache_y[offset + i] = dy
            blue_leg_cache_z[offset + i] = dz
        offset += len(leg_voxels)
        blue_leg_end_idx[leg_id] = offset

    # Clear extra legs (6 & 7) when switching from scorpion to 6-legged beetles
    num_legs = len(BLUE_LEGS)
    for leg_id in range(num_legs, 8):
        blue_leg_start_idx[leg_id] = 0
        blue_leg_end_idx[leg_id] = 0

    # Zero out any leftover leg voxels from previous geometry (offset = total new leg voxels)
    for i in range(offset, MAX_LEG_VOXELS):
        blue_leg_cache_x[i] = 0
        blue_leg_cache_y[i] = 0
        blue_leg_cache_z[i] = 0

    # Update leg tip cache
    offset = 0
    for leg_id, leg_tip_voxels in enumerate(BLUE_LEG_TIPS):
        blue_leg_tip_start_idx[leg_id] = offset
        for i, (dx, dy, dz) in enumerate(leg_tip_voxels):
            blue_leg_tip_cache_x[offset + i] = dx
            blue_leg_tip_cache_y[offset + i] = dy
            blue_leg_tip_cache_z[offset + i] = dz
        offset += len(leg_tip_voxels)
        blue_leg_tip_end_idx[leg_id] = offset

    # Clear extra leg tips (6 & 7) when switching from scorpion to 6-legged beetles
    for leg_id in range(num_legs, 8):
        blue_leg_tip_start_idx[leg_id] = 0
        blue_leg_tip_end_idx[leg_id] = 0

    # Zero out any leftover leg tip voxels from previous geometry (offset = total new leg tip voxels)
    for i in range(offset, MAX_LEG_TIP_VOXELS):
        blue_leg_tip_cache_x[i] = 0
        blue_leg_tip_cache_y[i] = 0
        blue_leg_tip_cache_z[i] = 0

    # Update blue beetle horn dimensions for accurate collision detection
    horn_length = calculate_horn_length(shaft_len, prong_len, horn_type)

    beetle_blue.horn_shaft_len = shaft_len
    beetle_blue.horn_prong_len = prong_len
    beetle_blue.horn_length = horn_length
    beetle_blue.horn_type = horn_type
    beetle_blue.horn_type_id = HORN_TYPE_IDS.get(horn_type, 0)

    # Scorpion doesn't use body_pitch_offset - its tilt is built into the geometry
    beetle_blue.body_pitch_offset = 0.0

    print(f"Rebuilt blue beetle: {len(BLUE_BODY)} body voxels (shaft={shaft_len:.0f}, prong={prong_len:.0f}, front={front_body_height:.0f}, back={back_body_height:.0f}, legs={leg_length:.0f})")

# Function to rebuild red beetle geometry with new parameters
def rebuild_red_beetle(shaft_len, prong_len, front_body_height=4, back_body_height=6, body_length=12, body_width=7, leg_length=8, horn_type="rhino", stinger_curvature=0.0, tail_rotation_angle=0.0):
    """Rebuild red beetle geometry cache with new horn, body, and leg parameters"""
    global RED_BODY, RED_LEGS, RED_LEG_TIPS, RED_HOOK_FLAGS, RED_STRIPE_FLAGS, RED_HORN_TIP_FLAGS, RED_VERY_TIP_FLAGS

    # Generate new geometry
    RED_BODY, RED_LEGS, RED_LEG_TIPS, RED_HOOK_FLAGS, RED_STRIPE_FLAGS, RED_HORN_TIP_FLAGS, RED_VERY_TIP_FLAGS = generate_beetle_geometry(shaft_len, prong_len, front_body_height, back_body_height, body_length, body_width, leg_length, horn_type, stinger_curvature, tail_rotation_angle)

    # Update beetle's hook interior flags
    beetle_red.body_hook_interior_flags = list(RED_HOOK_FLAGS)

    # Update body cache
    if len(RED_BODY) > MAX_BODY_VOXELS:
        print(f"WARNING: Red beetle body voxels ({len(RED_BODY)}) exceeds max ({MAX_BODY_VOXELS})!")
        return

    # Clear old cache entries to prevent voxel pollution when switching beetle types
    old_size = red_body_cache_size[None]
    new_size = len(RED_BODY)

    # Write new geometry
    for i, (dx, dy, dz) in enumerate(RED_BODY):
        red_body_cache_x[i] = dx
        red_body_cache_y[i] = dy
        red_body_cache_z[i] = dz
        # Copy pre-computed metadata flags
        if i < len(RED_STRIPE_FLAGS):
            red_body_stripe_flags[i] = RED_STRIPE_FLAGS[i]
            red_body_horn_tip_flags[i] = RED_HORN_TIP_FLAGS[i]
            red_body_very_tip_flags[i] = RED_VERY_TIP_FLAGS[i]
            red_body_hook_flags[i] = RED_HOOK_FLAGS[i]
        else:
            red_body_stripe_flags[i] = 0
            red_body_horn_tip_flags[i] = 0
            red_body_very_tip_flags[i] = 0
            red_body_hook_flags[i] = 0

    # Zero out any leftover voxels from previous geometry
    for i in range(new_size, old_size):
        red_body_cache_x[i] = 0
        red_body_cache_y[i] = 0
        red_body_cache_z[i] = 0
        red_body_stripe_flags[i] = 0
        red_body_horn_tip_flags[i] = 0
        red_body_very_tip_flags[i] = 0
        red_body_hook_flags[i] = 0

    # Update size LAST to prevent race conditions
    red_body_cache_size[None] = new_size

    # Update leg cache
    offset = 0
    for leg_id, leg_voxels in enumerate(RED_LEGS):
        red_leg_start_idx[leg_id] = offset
        for i, (dx, dy, dz) in enumerate(leg_voxels):
            red_leg_cache_x[offset + i] = dx
            red_leg_cache_y[offset + i] = dy
            red_leg_cache_z[offset + i] = dz
        offset += len(leg_voxels)
        red_leg_end_idx[leg_id] = offset

    # Clear extra legs (6 & 7) when switching from scorpion to 6-legged beetles
    num_legs = len(RED_LEGS)
    for leg_id in range(num_legs, 8):
        red_leg_start_idx[leg_id] = 0
        red_leg_end_idx[leg_id] = 0

    # Zero out any leftover leg voxels from previous geometry (offset = total new leg voxels)
    for i in range(offset, MAX_LEG_VOXELS):
        red_leg_cache_x[i] = 0
        red_leg_cache_y[i] = 0
        red_leg_cache_z[i] = 0

    # Update leg tip cache
    offset = 0
    for leg_id, leg_tip_voxels in enumerate(RED_LEG_TIPS):
        red_leg_tip_start_idx[leg_id] = offset
        for i, (dx, dy, dz) in enumerate(leg_tip_voxels):
            red_leg_tip_cache_x[offset + i] = dx
            red_leg_tip_cache_y[offset + i] = dy
            red_leg_tip_cache_z[offset + i] = dz
        offset += len(leg_tip_voxels)
        red_leg_tip_end_idx[leg_id] = offset

    # Clear extra leg tips (6 & 7) when switching from scorpion to 6-legged beetles
    for leg_id in range(num_legs, 8):
        red_leg_tip_start_idx[leg_id] = 0
        red_leg_tip_end_idx[leg_id] = 0

    # Zero out any leftover leg tip voxels from previous geometry (offset = total new leg tip voxels)
    for i in range(offset, MAX_LEG_TIP_VOXELS):
        red_leg_tip_cache_x[i] = 0
        red_leg_tip_cache_y[i] = 0
        red_leg_tip_cache_z[i] = 0

    # Update red beetle horn dimensions for accurate collision detection
    horn_length = calculate_horn_length(shaft_len, prong_len, horn_type)

    beetle_red.horn_shaft_len = shaft_len
    beetle_red.horn_prong_len = prong_len
    beetle_red.horn_length = horn_length
    beetle_red.horn_type = horn_type
    beetle_red.horn_type_id = HORN_TYPE_IDS.get(horn_type, 0)

    # Scorpion doesn't use body_pitch_offset - its tilt is built into the geometry
    beetle_red.body_pitch_offset = 0.0

    print(f"Rebuilt red beetle: {len(RED_BODY)} body voxels (shaft={shaft_len:.0f}, prong={prong_len:.0f}, front={front_body_height:.0f}, back={back_body_height:.0f}, legs={leg_length:.0f})")

def generate_stag_pincers(shaft_length, curve_length):
    """Generate stag beetle pincers as L-shaped clubs (horizontal then 90° up)

    Args:
        shaft_length: Length of horizontal segment before the 90° turn
        curve_length: Length of vertical segment going upward after the turn

    Returns list of (x, y, z) voxel coordinates for both pincers.
    Pivot point: (x=3, y=1, z=0) same as rhinoceros horn
    Club-shaped: thick at base (3x3), tapers to thin at tip (1x2)
    L-shape: horizontal for first half, then 90° turn upward
    """
    pincer_voxels = []

    # Convert to int for voxel generation
    horizontal_length = round(shaft_length)
    vertical_length = round(curve_length)

    # 20 degrees inward = reduce lateral spread by ~36% (tan(20°) ≈ 0.36)
    inward_factor = 0.36

    # LEFT PINCER
    # Part 1: Horizontal section (thick at base, medium at elbow)
    for i in range(horizontal_length):
        progress = i / float(horizontal_length) if horizontal_length > 0 else 0.0

        dx = 3 + i  # Extends forward
        dy = 1  # Flat horizontal
        dz = -int(i * inward_factor) - 1  # Extends left at 20° angle

        # Taper from thick (3x3) to medium (2x2) along horizontal section
        if progress < 0.5:
            # Thick base section (3x3 voxels)
            for dy_offset in range(-1, 2):
                for dz_offset in range(-1, 2):
                    pincer_voxels.append((dx, dy + dy_offset, dz + dz_offset, 0))
        else:
            # Medium section approaching elbow (2x2 voxels)
            for dy_offset in range(0, 2):
                for dz_offset in range(-1, 1):
                    pincer_voxels.append((dx, dy + dy_offset, dz + dz_offset, 0))

    # Part 2: Vertical section (90° turn upward, tapers to thin tip)
    elbow_x = 3 + horizontal_length - 1  # X position at elbow
    elbow_z = -int((horizontal_length - 1) * inward_factor) - 1  # Z position at elbow

    for j in range(vertical_length):
        progress = j / float(vertical_length) if vertical_length > 0 else 0.0

        dx = elbow_x  # Stay at elbow X position
        dy = 1 + j + 1  # Go upward from elbow (start above horizontal section)
        dz = elbow_z  # Stay at elbow Z position

        # Taper from medium (2x2) to thin (1x2) along vertical section
        if progress < 0.5:
            # Medium section (2x2 voxels)
            for dx_offset in range(-1, 1):
                for dz_offset in range(-1, 1):
                    pincer_voxels.append((dx + dx_offset, dy, dz + dz_offset, 0))
        else:
            # Thin tip section (1x2 voxels)
            for dx_offset in range(-1, 1):
                pincer_voxels.append((dx + dx_offset, dy, dz, 0))

    # RIGHT PINCER
    # Part 1: Horizontal section (thick at base, medium at elbow)
    for i in range(horizontal_length):
        progress = i / float(horizontal_length) if horizontal_length > 0 else 0.0

        dx = 3 + i  # Extends forward
        dy = 1  # Flat horizontal
        dz = int(i * inward_factor) + 1  # Extends right at 20° angle

        # Taper from thick (3x3) to medium (2x2) along horizontal section
        if progress < 0.5:
            # Thick base section (3x3 voxels)
            for dy_offset in range(-1, 2):
                for dz_offset in range(-1, 2):
                    pincer_voxels.append((dx, dy + dy_offset, dz + dz_offset, 0))
        else:
            # Medium section approaching elbow (2x2 voxels)
            for dy_offset in range(0, 2):
                for dz_offset in range(0, 2):
                    pincer_voxels.append((dx, dy + dy_offset, dz + dz_offset, 0))

    # Part 2: Vertical section (90° turn upward, tapers to thin tip)
    elbow_x = 3 + horizontal_length - 1  # X position at elbow
    elbow_z = int((horizontal_length - 1) * inward_factor) + 1  # Z position at elbow

    for j in range(vertical_length):
        progress = j / float(vertical_length) if vertical_length > 0 else 0.0

        dx = elbow_x  # Stay at elbow X position
        dy = 1 + j + 1  # Go upward from elbow
        dz = elbow_z  # Stay at elbow Z position

        # Taper from medium (2x2) to thin (1x2) along vertical section
        if progress < 0.5:
            # Medium section (2x2 voxels)
            for dx_offset in range(-1, 1):
                for dz_offset in range(0, 2):
                    pincer_voxels.append((dx + dx_offset, dy, dz + dz_offset, 0))
        else:
            # Thin tip section (1x2 voxels)
            for dx_offset in range(-1, 1):
                pincer_voxels.append((dx + dx_offset, dy, dz, 0))

    return pincer_voxels

def generate_hercules_horns(top_horn_len, bottom_horn_len, front_body_height, back_body_height):
    """Generate Hercules beetle dual-jaw horns (vertical pincer system)

    Args:
        top_horn_len: Length of thoracic horn (upper jaw) - long curved hook
        bottom_horn_len: Length of cephalic horn (lower jaw) - short straight spike
        front_body_height: Height of thorax (needed for horn attachment positioning)
        back_body_height: Height of abdomen (needed for proper scaling)

    Returns list of (x, y, z) voxel coordinates for both horns.
    Pivot point: (x=3, y=1, z=0) same as other horn types
    """
    horn_voxels = []

    # Convert to int for voxel generation
    top_horn_len = round(top_horn_len)
    bottom_horn_len = round(bottom_horn_len)

    # TOP HORN (Thoracic - upper jaw from thorax)
    # Long curved arc that rises then hooks downward like excavator claw
    # Origin: Y=7 (top of body), extends forward with upward curve then downward hook
    # 1:1 linear slider response, scaled to maintain original max length of 17
    # Slider 5→10 voxels, Slider 12→17 voxels (every increment changes size)
    # Angled 20° upward (tan(20°) ≈ 0.364)
    # Then rotated up 10° from base position
    actual_top_len = 5 + top_horn_len

    import math
    top_rotation_angle = math.radians(10)  # Rotate up 10 degrees
    cos_top_rot = math.cos(top_rotation_angle)
    sin_top_rot = math.sin(top_rotation_angle)
    top_pivot_x = 3.0
    top_pivot_y = 7.0

    for i in range(actual_top_len):
        progress = i / float(actual_top_len) if actual_top_len > 0 else 0.0

        dx = 3 + i  # Extends forward from pivot

        # Base upward angle: 20 degrees
        base_angle_rise = int(i * 0.364)

        # Height curve: rises in first 40%, levels 40-70%, hooks down 70-100%
        if progress < 0.4:
            # Rising section
            dy = 7 + base_angle_rise + int(i * 0.4)  # Base angle + additional rise
        elif progress < 0.7:
            # Level section at peak
            dy = 7 + base_angle_rise + int(actual_top_len * 0.4 * 0.4)
        else:
            # Downward hook at tip
            hook_progress = (progress - 0.7) / 0.3
            peak_height = 7 + int(actual_top_len * 0.4 * 0.4)
            dy = base_angle_rise + peak_height - int(hook_progress * actual_top_len * 0.15)

        # Apply +10° rotation around pivot (3, 7)
        # Rotate center of 2x2 block
        rel_x = dx - top_pivot_x
        rel_y = dy - top_pivot_y

        rotated_x = rel_x * cos_top_rot - rel_y * sin_top_rot
        rotated_y = rel_x * sin_top_rot + rel_y * cos_top_rot

        center_x = rotated_x + top_pivot_x
        center_y = rotated_y + top_pivot_y

        # Place 2x2 block around rotated center, using floor to avoid gaps
        base_x = int(center_x)
        base_y = int(center_y)

        for dx_off in [0, 1]:
            for dy_off in [0, 1]:
                for dz in [-1, 0]:
                    horn_voxels.append((base_x + dx_off, base_y + dy_off, dz))

    # Fill gap at top horn attachment point (where horn meets body)
    # Create solid neck/bridge from thorax to horn base
    # Thorax top is at Y=(front_body_height-1), horn base starts around Y=7
    if front_body_height < 8:
        # Body is shorter than horn base - need a substantial bridge
        attach_y_start = front_body_height  # Start at top of thorax
        attach_y_end = 8  # End at horn base
        for attach_y in range(attach_y_start, attach_y_end):
            # Taper the neck: wider at base, narrower at top
            progress = (attach_y - attach_y_start) / float(attach_y_end - attach_y_start) if attach_y_end > attach_y_start else 0
            # Only build the lower portion (first 1/2) to avoid the tall stack at top
            if progress < 0.5:
                x_range = 3 if progress > 0.5 else 4 if progress > 0.25 else 5  # Taper width
                z_width = 1  # 3 voxels wide (-1, 0, 1)
                for attach_x in range(1, x_range):
                    for dz in range(-z_width, z_width + 1):
                        horn_voxels.append((attach_x, attach_y, dz))

    # Fill the empty layer at the base of top horn (Y=6 and Y=7 at x=3,4)
    for fill_y in [6, 7]:
        for fill_x in [3, 4]:
            for dz in [-1, 0]:
                horn_voxels.append((fill_x, fill_y, dz))

    # BOTTOM HORN (Cephalic - lower jaw from head)
    # Clean, smooth upward curve from below head to meet top horn
    # Origin: Y=1 (head level), extends forward with upward arc
    # 1:1 linear slider response, scaled to maintain original max length of 13
    # Slider 0→7 voxels, Slider 6→13 voxels (every increment changes size)
    # Angled 20° upward base + additional upward curve
    # Then rotated down 30° from base position
    actual_bottom_len = 7 + bottom_horn_len

    import math
    rotation_angle = math.radians(-30)  # Rotate down 30 degrees
    cos_rot = math.cos(rotation_angle)
    sin_rot = math.sin(rotation_angle)
    pivot_x = 3.0
    pivot_y = 1.0

    for i in range(actual_bottom_len):
        # Generate initial position
        dx = 3 + i  # Extends forward from pivot

        # Simple, clean upward curve formula
        # Base 20° angle + quadratic upward curve (accelerating rise)
        base_angle_rise = int(i * 0.364)  # 20 degree base angle
        curve_rise = int((i * i) / float(actual_bottom_len) * 0.5)  # Accelerating upward curve

        dy = 1 + base_angle_rise + curve_rise

        # Apply -30° rotation around pivot (3, 1)
        # Rotate center of 2x2 block
        rel_x = dx - pivot_x
        rel_y = dy - pivot_y

        rotated_x = rel_x * cos_rot - rel_y * sin_rot
        rotated_y = rel_x * sin_rot + rel_y * cos_rot

        center_x = rotated_x + pivot_x
        center_y = rotated_y + pivot_y

        # Place 2x2 block around rotated center, using floor to avoid gaps
        base_x = int(center_x)
        base_y = int(center_y)

        for dx_off in [0, 1]:
            for dy_off in [0, 1]:
                for dz in [-1, 0]:
                    horn_voxels.append((base_x + dx_off, base_y + dy_off, dz))

    return horn_voxels

def generate_scorpion_stinger(shaft_len=12, prong_len=5, body_length=12, back_body_height=5, stinger_curvature=0.0, tail_rotation_angle=0.0):
    """
    Generate scorpion tail/stinger - Simple version for debugging

    Args:
        shaft_len: Total tail arc length (6-18) - controls reach
        prong_len: Curvature amount (2-10) - controls curl tightness
        body_length: Body length for parametric attachment
        back_body_height: Rear body height for parametric attachment
        stinger_curvature: Dynamic curvature offset (-1.0 to +1.0) controlled by VB/NM keys
        tail_rotation_angle: Dynamic tail rotation angle in degrees (-20 to +20) controlled by VB/NM keys

    Returns:
        List of (dx, dy, dz) voxel positions for stinger
    """
    import math

    stinger_voxels = []

    # ===== SCORPION TAIL SHAFT (CLEAN REBUILD) =====
    # Attach to top of butt, smooth curve, 3 voxels thick
    # shaft_len slider controls length

    # ATTACHMENT POINT - Top of abdomen (butt)
    start_x = -body_length + 1
    start_y = back_body_height
    start_z = 0

    # SHAFT LENGTH - Scale slider value to reasonable tail size
    # Slider range 8-15 maps to effective length 8-12 (slower scaling)
    # Each tick of slider adds 0.57 effective length
    effective_len = 8 + (shaft_len - 8) * 0.57

    # More samples = smoother curve
    num_samples = int(effective_len * 8)  # 8 samples per voxel length = smooth curve

    # CURVE SHAPE - Simple 4-point smooth arc (fewer points = smoother curve)
    # Scale EVERYTHING proportionally with effective_len for smooth curves at all sizes
    height = int(effective_len * 0.8)  # How high the tail arcs (reduced from 1.0)
    forward = int(effective_len * 1.5)  # How far forward it reaches (reduced from 2.0)

    # Curvature adjustment (VB/NM keys)
    curl = int(stinger_curvature * effective_len * 0.4)

    # Simple 4-point arc - NO SHARP CORNERS, gentle curve
    control_points = [
        (start_x, start_y),  # Base attachment
        (start_x + int(effective_len * 0.4) - curl, start_y + int(effective_len * 0.8)),  # Rising
        (start_x + int(effective_len * 1.0) - curl, start_y + height),  # Peak
        (start_x + forward - curl, start_y + height - int(effective_len * 0.15)),  # End slightly lower
    ]

    num_control = len(control_points)
    prev_x, prev_y = None, None

    # Generate smooth curve using Catmull-Rom spline
    for i in range(num_samples):
        # Progress along curve (0.0 to 1.0)
        t = i / float(max(num_samples - 1, 1))

        # Find which segment we're interpolating
        segment_t = t * (num_control - 1)
        segment_idx = int(segment_t)
        if segment_idx >= num_control - 1:
            segment_idx = num_control - 2

        # Local t within segment
        local_t = segment_t - segment_idx

        # Get 4 control points for Catmull-Rom spline
        if segment_idx == 0:
            p0 = control_points[0]
        else:
            p0 = control_points[segment_idx - 1]

        p1 = control_points[segment_idx]
        p2 = control_points[segment_idx + 1]

        if segment_idx + 2 < num_control:
            p3 = control_points[segment_idx + 2]
        else:
            p3 = control_points[-1]

        # Catmull-Rom interpolation formula
        t2 = local_t * local_t
        t3 = t2 * local_t

        x = int(0.5 * ((2 * p1[0]) +
                       (-p0[0] + p2[0]) * local_t +
                       (2*p0[0] - 5*p1[0] + 4*p2[0] - p3[0]) * t2 +
                       (-p0[0] + 3*p1[0] - 3*p2[0] + p3[0]) * t3))

        y = int(0.5 * ((2 * p1[1]) +
                       (-p0[1] + p2[1]) * local_t +
                       (2*p0[1] - 5*p1[1] + 4*p2[1] - p3[1]) * t2 +
                       (-p0[1] + 3*p1[1] - 3*p2[1] + p3[1]) * t3))

        z = start_z

        # Fill gaps between consecutive voxels for solid shaft
        if prev_x is not None:
            dx = x - prev_x
            dy = y - prev_y
            steps = max(abs(dx), abs(dy)) + 1

            for step in range(steps):
                inter_t = step / float(steps)
                inter_x = int(prev_x + dx * inter_t)
                inter_y = int(prev_y + dy * inter_t)

                # 3 VOXELS THICK (in Z direction) + Y-overlap for continuity at all angles
                for dz in range(-1, 2):  # -1, 0, 1 = 3 layers in Z
                    for dy_overlap in range(-1, 2):  # -1, 0, 1 = 3 layers in Y for overlap
                        stinger_voxels.append((inter_x, inter_y + dy_overlap, z + dz))
        else:
            # First iteration - place starting voxels
            for dz in range(-1, 2):
                for dy_overlap in range(-1, 2):
                    stinger_voxels.append((x, y + dy_overlap, z + dz))

        prev_x, prev_y = x, y

    # ===== SCORPION STINGER TIP (PRONG) =====
    # Pointy tip that curves down and forward like real scorpion stingers
    # prong_len slider controls length (2-10)

    if prong_len > 0 and prev_x is not None:
        # Tip starts from the end of the shaft (prev_x, prev_y)
        tip_start_x = prev_x
        tip_start_y = prev_y
        tip_start_z = start_z

        # Tip length controlled by prong_len
        tip_segments = int(prong_len) * 5  # 5x segments for smooth, gap-free stinger at all lengths

        # Track previous position for interpolation
        last_tip_x = tip_start_x
        last_tip_y = tip_start_y

        # Create pointy tip - tapers from 3 voxels thick to 1 voxel (point)
        # AND curves downward and forward
        for i in range(tip_segments):
            progress = i / float(max(tip_segments - 1, 1))  # 0.0 at base, 1.0 at tip

            # Curve down and forward (like scorpion stinger)
            # Forward: progress-based so angle stays consistent regardless of segment density
            tip_x = tip_start_x + int(progress * prong_len * 1.2)

            # Downward: drops down smoothly (parabolic curve)
            drop = int(progress * progress * prong_len * 0.8)  # Use prong_len, not tip_segments
            tip_y = tip_start_y - drop

            # === INTERPOLATION: Fill gaps ONLY when needed ===
            # Calculate the distance we need to travel
            dx_step = tip_x - last_tip_x
            dy_step = tip_y - last_tip_y
            steps = max(abs(dx_step), abs(dy_step))

            # Only interpolate if there's an actual gap (steps > 1)
            # If positions are adjacent (steps <= 1), just use the current position
            if steps > 1:
                # Fill intermediate positions to close the gap
                for step in range(1, steps):  # Skip 0 (already done) and steps (will be done next iter)
                    t = step / float(steps)
                    inter_x = last_tip_x + int(dx_step * t)
                    inter_y = last_tip_y + int(dy_step * t)

                    # Taper: starts at 5 voxels thick, narrows to 1 voxel at the tip
                    if progress < 0.5:
                        # Base section: 5 voxels thick (fuller stinger) + Y-overlap
                        for dz in range(-2, 3):  # -2, -1, 0, 1, 2
                            for dy_overlap in range(-1, 2):  # Y-overlap for continuity
                                stinger_voxels.append((inter_x, inter_y + dy_overlap, tip_start_z + dz))
                    elif progress < 0.8:
                        # Middle section: 3 voxels thick + Y-overlap
                        for dz in range(-1, 2):  # -1, 0, 1
                            for dy_overlap in range(-1, 2):  # Y-overlap for continuity
                                stinger_voxels.append((inter_x, inter_y + dy_overlap, tip_start_z + dz))
                    else:
                        # Tip section: 1 voxel thick (point) + minimal Y-overlap
                        for dy_overlap in range(-1, 2):
                            stinger_voxels.append((inter_x, inter_y + dy_overlap, tip_start_z))

            # Always add the current position voxels
            if progress < 0.5:
                # Base section: 5 voxels thick (fuller stinger) + Y-overlap
                for dz in range(-2, 3):  # -2, -1, 0, 1, 2
                    for dy_overlap in range(-1, 2):  # Y-overlap for continuity
                        stinger_voxels.append((tip_x, tip_y + dy_overlap, tip_start_z + dz))
            elif progress < 0.8:
                # Middle section: 3 voxels thick + Y-overlap
                for dz in range(-1, 2):  # -1, 0, 1
                    for dy_overlap in range(-1, 2):  # Y-overlap for continuity
                        stinger_voxels.append((tip_x, tip_y + dy_overlap, tip_start_z + dz))
            else:
                # Tip section: 1 voxel thick (point) + minimal Y-overlap
                for dy_overlap in range(-1, 2):
                    stinger_voxels.append((tip_x, tip_y + dy_overlap, tip_start_z))

            # Update last position for next iteration
            last_tip_x = tip_x
            last_tip_y = tip_y

    # Rotate entire tail upward by 15 degrees (base) + dynamic tail_rotation_angle around attachment point
    # Rotation in X-Y plane (around Z-axis)
    # tail_rotation_angle: -20 (down) to +20 (up)
    angle_deg = 15.0 + tail_rotation_angle
    angle_rad = math.radians(angle_deg)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)

    rotated_voxels = []
    for (vx, vy, vz) in stinger_voxels:
        # Translate to origin (attachment point)
        rel_x = vx - start_x
        rel_y = vy - start_y

        # Rotate in X-Y plane (around Z-axis) - tilts tail upward
        new_x = rel_x * cos_a - rel_y * sin_a
        new_y = rel_x * sin_a + rel_y * cos_a

        # Translate back
        final_x = int(round(new_x + start_x))
        final_y = int(round(new_y + start_y))

        rotated_voxels.append((final_x, final_y, vz))

    return rotated_voxels

def generate_atlas_pronotum_horns(prong_len=5):
    """
    Generate Atlas beetle dual stationary pronotum horns with head piece

    Args:
        prong_len: Horn length parameter (5-8) - controls how long the horns are

    Returns:
        List of (dx, dy, dz) voxel positions for both horns (left + right) plus head piece

    The Atlas beetle has TWO STATIONARY HORNS that curve upward and forward from
    the beetle's shoulders (pronotum). These horns are part of the body geometry
    and do NOT rotate with horn pitch/yaw controls.
    """
    horn_voxels = []

    # HEAD PIECE - Raised section between horns and body
    # Creates a 2-voxel thick "head" that the horns attach to
    # This fills the gap between the horns and makes attachment look natural
    for dx in [2, 3]:  # 2 voxels forward from front of body
        for dy in [3, 4, 5]:  # Sticks up above body (height 3-5)
            for dz in [-3, -2, -1, 0, 1, 2, 3]:  # Spans wider to connect with horn bases at ±4
                horn_voxels.append((dx, dy, dz))

    # Calculate horn length based on prong_len slider
    # Scale by 2x to make horns longer, but still add gradually
    # prong_len=5 → 10 segments, prong_len=10 → 20 segments
    horn_length = int(prong_len * 2)  # 2x scaling for longer horns

    # LEFT PRONOTUM HORN
    for i in range(horn_length):
        # Forward and upward arc
        dx = 3 + i  # Forward from front of body
        # Parabolic upward curve: y = base + linear_component + quadratic_component
        # Raised by +1 voxel (changed from 4 to 5)
        # Increased linear component from 0.5 to 0.67 for ~10° steeper angle
        dy = 5 + int(i * 0.67 + (i * i) * 0.02)

        # Curved lateral spread: starts at -4, curves outward, then back inward at tips
        # Similar to stag curve but less intense
        progress = i / float(max(horn_length - 1, 1))  # 0.0 to 1.0
        # Outward spread peaks at middle (progress=0.5), returns toward center at tips
        # Increased multiplier from 4.0 to 12.0 for more visible curve
        lateral_curve = progress * (1.0 - progress) * 12.0  # Peaks at 3.0 when progress=0.5
        dz = -4 - int(lateral_curve)

        # Keep 2x2 thickness throughout - NO single voxel tips
        # This prevents floating single voxels and keeps horns solid
        for dy_off in [0, 1]:
            for dz_off in [0, -1]:
                horn_voxels.append((dx, dy + dy_off, dz + dz_off))

    # RIGHT PRONOTUM HORN (mirror of left)
    for i in range(horn_length):
        dx = 3 + i
        # Raised by +1 voxel (changed from 4 to 5)
        # Increased linear component from 0.5 to 0.67 for ~10° steeper angle
        dy = 5 + int(i * 0.67 + (i * i) * 0.02)

        # Curved lateral spread (mirrored to positive Z)
        progress = i / float(max(horn_length - 1, 1))
        # Increased multiplier from 4.0 to 12.0 for more visible curve
        lateral_curve = progress * (1.0 - progress) * 12.0
        dz = 4 + int(lateral_curve)

        # Keep 2x2 thickness throughout - NO single voxel tips
        for dy_off in [0, 1]:
            for dz_off in [0, 1]:
                horn_voxels.append((dx, dy + dy_off, dz + dz_off))

    return horn_voxels

def generate_atlas_cephalic_horn(shaft_len=5):
    """
    Generate Atlas beetle movable cephalic (head) horn - the bottom horn

    Args:
        shaft_len: Horn length parameter (5-10) - controls how long the horn is

    Returns:
        List of (dx, dy, dz) voxel positions for cephalic horn

    This is the MOVABLE horn that tilts up/down with R/Y keys (pitch control).
    It projects forward and curves gently upward - used for scooping opponents.
    Thicker and broader than the pronotum horns (3x3 base vs 2x2).
    """
    horn_voxels = []

    # Scale horn length - cephalic horn grows moderately (6-10 voxels)
    # Using 0.8x multiplier + 2 base gives: slider 5→6, slider 10→10
    horn_length = int(shaft_len * 0.8) + 2

    # Base attachment point (will be rotated by horn_pitch in rendering)
    base_x = 3  # Matches horn pivot point
    base_y = 2  # Slightly raised above body floor
    base_z = 0  # Centered on midline (no lateral curve)

    for i in range(horn_length):
        # Forward extension
        dx = base_x + i

        # Gentle parabolic upward curve (less steep than pronotum horns)
        # Starts nearly horizontal, gradually curves upward
        dy = base_y + int(i * 0.4 + (i * i) * 0.04)

        # No lateral curve - stays centered (unlike pronotum horns)
        dz = base_z

        # Thickness tapering: 3x3 base → 3x2 mid → 2x2 tip
        # Thicker than pronotum horns for visual distinction
        if i < 4:
            # Base section: 3x3 cross-section (thick and robust)
            for dy_off in [0, 1, 2]:
                for dz_off in [-1, 0, 1]:
                    horn_voxels.append((dx, dy + dy_off, dz + dz_off))
        elif i < 8:
            # Mid section: 3x2 cross-section (blade-like)
            for dy_off in [0, 1, 2]:
                for dz_off in [-1, 0]:
                    horn_voxels.append((dx, dy + dy_off, dz + dz_off))
        else:
            # Tip section: 3x2 cross-section (thicker tip)
            for dy_off in [0, 1, 2]:
                for dz_off in [-1, 0]:
                    horn_voxels.append((dx, dy + dy_off, dz + dz_off))

    return horn_voxels

@ti.kernel
def check_floor_collision(world_x: ti.f32, world_z: ti.f32) -> ti.f32:
    """Check floor height using pre-computed cache - FAST version
    Returns highest floor Y coordinate in WORLD space for the beetle's footprint"""
    center_x = int(world_x + 64.0)  # Convert world to grid coords
    center_z = int(world_z + 64.0)

    # Check area around beetle (beetles are ~20 voxels wide)
    check_radius = 3
    highest_floor_y = -1000.0

    for i in range(center_x - check_radius, center_x + check_radius + 1):
        for k in range(center_z - check_radius, center_z + check_radius + 1):
            # Clamp to valid grid bounds
            gi = ti.max(0, ti.min(127, i))
            gk = ti.max(0, ti.min(127, k))
            cached_y = floor_height_cache[gi, gk]
            if cached_y > highest_floor_y:
                highest_floor_y = cached_y

    return highest_floor_y


def apply_bowl_slide(entity, params):
    """Push entity toward arena center if on slippery bowl perimeter"""
    dist_from_center = math.sqrt(entity.x**2 + entity.z**2)
    if dist_from_center > ARENA_RADIUS:
        # Check if in goal pit area (no ice there, so no slide)
        # Goal pits: x <= -32 or x >= 32, and z within ±12 of center (z=0 in physics space)
        goal_pit_half_width = 12
        in_goal_pit = abs(entity.z) < goal_pit_half_width and (entity.x <= -32 or entity.x >= 32)
        if in_goal_pit:
            return  # No slide in goal pit areas

        # On the bowl - apply direct position slide (can't be resisted)
        slide_strength = params["BOWL_SLIDE_STRENGTH"]
        dist_into_bowl = dist_from_center - ARENA_RADIUS
        # Scale force by how far into the bowl (stronger at edges)
        force_multiplier = 1.0 + dist_into_bowl * 0.2
        # Direction toward center (normalized)
        if dist_from_center > 0.01:  # Avoid division by zero
            dir_x = -entity.x / dist_from_center
            dir_z = -entity.z / dist_from_center
            # Direct position slide (smooth, constant pull toward center)
            slide_amount = slide_strength * force_multiplier * 0.05
            entity.x += dir_x * slide_amount
            entity.z += dir_z * slide_amount
            # Also update prev position to avoid jitter
            entity.prev_x += dir_x * slide_amount
            entity.prev_z += dir_z * slide_amount


def apply_bowl_tilt(beetle):
    """Tilt beetle to match bowl slope when on the perimeter"""
    dist_from_center = math.sqrt(beetle.x**2 + beetle.z**2)
    if dist_from_center > ARENA_RADIUS and dist_from_center > 0.01:
        # Calculate the slope angle based on bowl geometry
        # Bowl rises 0.4 voxels per voxel outward = ~22 degree slope
        bowl_slope_angle = math.atan(0.4)  # ~0.38 radians (~22 degrees)

        # Direction FROM center (outward) - this is the "uphill" direction
        outward_x = beetle.x / dist_from_center
        outward_z = beetle.z / dist_from_center

        # Calculate beetle's facing direction
        facing_x = math.cos(beetle.rotation)
        facing_z = math.sin(beetle.rotation)

        # Right direction (perpendicular to facing)
        right_x = -math.sin(beetle.rotation)
        right_z = math.cos(beetle.rotation)

        # How much is the beetle facing uphill vs sideways?
        # Dot product of facing with outward = how much pitch needed
        # Dot product of right with outward = how much roll needed
        pitch_factor = facing_x * outward_x + facing_z * outward_z
        roll_factor = right_x * outward_x + right_z * outward_z

        # Apply tilt (facing uphill = pitch back, facing downhill = pitch forward)
        # Negative pitch_factor means facing toward center (downhill)
        target_pitch = -pitch_factor * bowl_slope_angle
        target_roll = -roll_factor * bowl_slope_angle

        # Smoothly blend toward target tilt
        blend_speed = 0.15
        beetle.pitch += (target_pitch - beetle.pitch) * blend_speed
        beetle.roll += (target_roll - beetle.roll) * blend_speed
    else:
        # On flat arena - gradually return to level
        blend_speed = 0.1
        beetle.pitch *= (1.0 - blend_speed)
        beetle.roll *= (1.0 - blend_speed)


@ti.kernel
def calculate_beetle_lowest_point(world_y: ti.f32, rotation: ti.f32, pitch: ti.f32, roll: ti.f32, horn_pitch: ti.f32) -> ti.f32:
    """Calculate the lowest Y coordinate the beetle's geometry would occupy after rotation"""
    base_y = int(world_y)

    # Rotation matrices
    cos_yaw = ti.cos(rotation)
    sin_yaw = ti.sin(rotation)
    cos_pitch_body = ti.cos(pitch)
    sin_pitch_body = ti.sin(pitch)
    cos_roll = ti.cos(roll)
    sin_roll = ti.sin(roll)

    # NOTE: horn_pitch parameter kept for API compatibility but no longer used
    # Ground collision is now based on body/legs only, not horn position

    lowest_y = 9999.0  # Start with very high value

    # Check all body voxels
    # NOTE: Horn pitch rotation removed - ground collision should be based on body/legs only,
    # not horn position. This fixes Atlas beetle floating when horn pitched down.
    for i in range(body_cache_size[None]):
        local_x = float(body_cache_x[i])
        local_y = body_cache_y[i]
        local_z = float(body_cache_z[i])

        # Apply 3D rotation (yaw -> pitch -> roll)
        ly = float(local_y)

        # Yaw
        temp_x = local_x * cos_yaw - local_z * sin_yaw
        temp_z = local_x * sin_yaw + local_z * cos_yaw
        temp_y = ly

        # Pitch
        temp2_x = temp_x * cos_pitch_body - temp_y * sin_pitch_body
        temp2_y = temp_x * sin_pitch_body + temp_y * cos_pitch_body
        temp2_z = temp_z

        # Roll
        final_y = temp2_y * cos_roll - temp2_z * sin_roll

        grid_y = float(base_y) + final_y
        if grid_y < lowest_y:
            lowest_y = grid_y

    # Check all leg voxels (up to 8 for scorpion, 6 for others)
    for leg_id in range(8):
        start_idx = leg_start_idx[leg_id]
        end_idx = leg_end_idx[leg_id]

        for i in range(start_idx, end_idx):
            local_x = float(leg_cache_x[i])
            local_y = float(leg_cache_y[i])
            local_z = float(leg_cache_z[i])

            # Apply 3D rotation (same as body)
            ly = local_y

            # Yaw
            temp_x = local_x * cos_yaw - local_z * sin_yaw
            temp_z = local_x * sin_yaw + local_z * cos_yaw
            temp_y = ly

            # Pitch
            temp2_x = temp_x * cos_pitch_body - temp_y * sin_pitch_body
            temp2_y = temp_x * sin_pitch_body + temp_y * cos_pitch_body
            temp2_z = temp_z

            # Roll
            final_y = temp2_y * cos_roll - temp2_z * sin_roll

            grid_y = float(base_y) + final_y
            if grid_y < lowest_y:
                lowest_y = grid_y

    # Check all leg tip voxels (these are typically the lowest points) (up to 8 for scorpion)
    for leg_id in range(8):
        tip_start_idx = leg_tip_start_idx[leg_id]
        tip_end_idx = leg_tip_end_idx[leg_id]

        for i in range(tip_start_idx, tip_end_idx):
            local_x = float(leg_tip_cache_x[i])
            local_y = float(leg_tip_cache_y[i])
            local_z = float(leg_tip_cache_z[i])

            # Apply 3D rotation (same as legs)
            ly = local_y

            # Yaw
            temp_x = local_x * cos_yaw - local_z * sin_yaw
            temp_z = local_x * sin_yaw + local_z * cos_yaw
            temp_y = ly

            # Pitch
            temp2_x = temp_x * cos_pitch_body - temp_y * sin_pitch_body
            temp2_y = temp_x * sin_pitch_body + temp_y * cos_pitch_body
            temp2_z = temp_z

            # Roll
            final_y = temp2_y * cos_roll - temp2_z * sin_roll

            grid_y = float(base_y) + final_y
            if grid_y < lowest_y:
                lowest_y = grid_y

    return lowest_y

@ti.kernel
def place_animated_beetle_blue(world_x: ti.f32, world_y: ti.f32, world_z: ti.f32, rotation: ti.f32, pitch: ti.f32, roll: ti.f32, horn_pitch: ti.f32, horn_yaw: ti.f32, tail_pitch: ti.f32, horn_type_id: ti.i32, body_pitch_offset: ti.f32, body_color: ti.i32, leg_color: ti.i32, leg_tip_color: ti.i32, walk_phase: ti.f32, is_lifted_high: ti.i32, default_horn_pitch: ti.f32, body_length: ti.i32, back_body_height: ti.i32, is_rotating_only: ti.i32, rotation_direction: ti.i32):
    """Beetle placement with 3D rotation (yaw/pitch/roll) and animated legs

    Args:
        horn_yaw: Horizontal horn rotation (stag=pincer spread, rhino/hercules=horn yaw)
        tail_pitch: Scorpion tail rotation angle (degrees, -15 to +15)
        horn_type_id: 0=rhino, 1=stag, 2=hercules, 3=scorpion
        body_pitch_offset: Static body tilt angle for scorpion (radians)
    """
    center_x = int(world_x + simulation.n_grid / 2.0)
    center_z = int(world_z + simulation.n_grid / 2.0)
    base_y = int(world_y + RENDER_Y_OFFSET)  # Apply Y offset for rendering below floor

    # Rotation matrices for yaw, pitch, roll
    cos_yaw = ti.cos(rotation)
    sin_yaw = ti.sin(rotation)
    # Combine physics pitch with static body tilt offset (for scorpion stance)
    total_pitch = pitch + body_pitch_offset
    cos_pitch_body = ti.cos(total_pitch)
    sin_pitch_body = ti.sin(total_pitch)
    cos_roll = ti.cos(roll)
    sin_roll = ti.sin(roll)

    # Pitch rotation for horn (rotation around Z-axis in local space)
    cos_pitch = ti.cos(horn_pitch)
    sin_pitch = ti.sin(horn_pitch)

    # Horn/claw pivot point in local coordinates (where horn/claw attaches to head)
    # Scorpion claws pivot at X=2, beetle horns pivot at X=3
    horn_pivot_x = 2.0 if horn_type_id == 3 else 3.0
    horn_pivot_y = 2

    # OPTIMIZATION: Pre-calculate all trigonometry values ONCE before voxel loop
    # These are constant for all voxels in this beetle, no need to recalculate 600+ times
    cos_horn_pitch = ti.cos(horn_pitch)
    sin_horn_pitch = ti.sin(horn_pitch)
    cos_horn_yaw = ti.cos(horn_yaw)
    sin_horn_yaw = ti.sin(horn_yaw)

    # Scorpion-specific trig (only calculated if needed, but outside loop)
    pitch_deviation = horn_pitch - default_horn_pitch
    cos_default = ti.cos(default_horn_pitch)
    sin_default = ti.sin(default_horn_pitch)
    cos_deviation = ti.cos(pitch_deviation)
    sin_deviation = ti.sin(pitch_deviation)

    # 1. Place body with horn pitch applied
    for i in range(body_cache_size[None]):
        local_x = float(body_cache_x[i])
        local_y = body_cache_y[i]
        local_z = float(body_cache_z[i])

        # Apply horn pitch and yaw rotation ONLY to horn voxels (dx >= 2 for scorpion claws, dx >= 3 for others)
        # Scorpion claws start at dx=2, beetle horns start at dx=3
        should_rotate = False
        if horn_type_id == 3:  # Scorpion - rotate claws (dx >= 2 AND |dz| > 2 to exclude centered tail)
            should_rotate = body_cache_x[i] >= 2 and abs(local_z) > 2.0
        elif horn_type_id == 4:  # Atlas - only rotate cephalic horn (centered Z position)
            # Cephalic horn: dx >= 3 AND |dz| <= 1 (centered on midline Z=0)
            # Pronotum horns: dx >= 3 AND |dz| >= 2 (spread outward Z=±3+) - DON'T rotate
            should_rotate = body_cache_x[i] >= 3 and abs(local_z) <= 1.5
        elif body_cache_x[i] >= 3:  # Other beetles - rotate horns (dx >= 3)
            should_rotate = True

        if should_rotate:
            # This is a horn voxel - apply pitch and yaw rotations around pivot point
            # Translate to pivot
            rel_x = local_x - horn_pivot_x
            rel_y = float(local_y - horn_pivot_y)
            rel_z = local_z  # Z is already centered at 0

            # STEP 1: Pitch rotation (around Z-axis in beetle local space)
            # Positive pitch = horn rotates up
            # For Hercules: ONLY bottom horn (y < 3) rotates, top horn (y >= 3) stays fixed
            # NOTE: cos_horn_pitch and sin_horn_pitch already calculated outside loop

            # Initialize pitched results
            pitched_x = rel_x
            pitched_y = rel_y
            pitched_z = rel_z

            # Apply pitch rotation based on beetle type and horn position
            if horn_type_id == 3:
                # SCORPION: Both claws start at default angle, then alternate when R/Y pressed
                # NOTE: pitch_deviation, cos_default, sin_default, cos_deviation, sin_deviation
                # already calculated outside loop for performance

                # First apply default pitch to both claws equally
                temp_x = rel_x * cos_default - rel_y * sin_default
                temp_y = rel_x * sin_default + rel_y * cos_default

                # Then apply alternating deviation
                if pitch_deviation != 0.0:

                    if rel_z < -1.0:  # Left claw - invert deviation
                        pitched_x = temp_x * cos_deviation + temp_y * sin_deviation
                        pitched_y = -temp_x * sin_deviation + temp_y * cos_deviation
                    elif rel_z > 1.0:  # Right claw - normal deviation
                        pitched_x = temp_x * cos_deviation - temp_y * sin_deviation
                        pitched_y = temp_x * sin_deviation + temp_y * cos_deviation
                    else:  # Center voxels use default only
                        pitched_x = temp_x
                        pitched_y = temp_y
                else:  # No deviation, just use default
                    pitched_x = temp_x
                    pitched_y = temp_y
                # else: center voxels don't rotate
            elif horn_type_id == 2:
                # HERCULES: Only rotate bottom horn, top horn stays fixed
                # After -30 degree rotation, bottom horn is mostly Y < 3, with tip reaching ~Y=5
                # Top horn: After +10 degree rotation, starts at Y~8, goes up to Y=20+
                # Simple conservative rule: if Y < 5, it's bottom horn for sure
                # If Y >= 5 and Y < 8, only rotate if far forward (bottom horn tip area)
                if local_y < 5:
                    # Definitely bottom horn (rotated down 30 degrees)
                    pitched_x = rel_x * cos_horn_pitch - rel_y * sin_horn_pitch
                    pitched_y = rel_x * sin_horn_pitch + rel_y * cos_horn_pitch
                elif local_y < 8 and rel_x >= 10.0:
                    # Mid-height but far forward - bottom horn tip only
                    pitched_x = rel_x * cos_horn_pitch - rel_y * sin_horn_pitch
                    pitched_y = rel_x * sin_horn_pitch + rel_y * cos_horn_pitch
                # else: Y >= 8 or near base - top horn stays fixed
            elif horn_type_id == 4:
                # ATLAS: Only rotate cephalic horn (centered), pronotum horns (sides) stay fixed
                # should_rotate already filtered for |Z| <= 1.5, so just apply rotation
                pitched_x = rel_x * cos_horn_pitch - rel_y * sin_horn_pitch
                pitched_y = rel_x * sin_horn_pitch + rel_y * cos_horn_pitch
            else:
                # STAG/RHINO: Rotate entire horn
                pitched_x = rel_x * cos_horn_pitch - rel_y * sin_horn_pitch
                pitched_y = rel_x * sin_horn_pitch + rel_y * cos_horn_pitch

            # Z unchanged by pitch regardless of beetle type
            pitched_z = rel_z

            # STEP 2: Yaw rotation (around Y-axis in beetle local space)
            # Behavior differs based on beetle type
            # NOTE: cos_horn_yaw and sin_horn_yaw already calculated outside loop

            # Initialize yaw results (required for Taichi)
            yawed_x = pitched_x
            yawed_y = pitched_y
            yawed_z = pitched_z

            if horn_type_id == 1:
                # STAG BEETLE: Apply opposite rotations to left vs right pincers
                # Left pincer (z < 0): positive horn_yaw opens outward (more negative Z)
                # Right pincer (z > 0): positive horn_yaw opens outward (more positive Z)
                if pitched_z < -0.1:
                    # Left pincer - apply rotation as-is
                    yawed_x = pitched_x * cos_horn_yaw + pitched_z * sin_horn_yaw
                    yawed_z = -pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
                elif pitched_z > 0.1:
                    # Right pincer - apply inverse rotation
                    yawed_x = pitched_x * cos_horn_yaw - pitched_z * sin_horn_yaw
                    yawed_z = pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
                # else: Center voxels - keep initialized values (no yaw rotation)
            elif horn_type_id == 2:
                # HERCULES BEETLE: Roll/twist both horns around X-axis (forward axis)
                # Like twisting a ball held between top and bottom hands
                # Both horns roll in same direction (B key = both twist right, V key = both twist left)
                yawed_x = pitched_x  # X unchanged (axis of rotation)
                yawed_y = pitched_y * cos_horn_yaw - pitched_z * sin_horn_yaw
                yawed_z = pitched_y * sin_horn_yaw + pitched_z * cos_horn_yaw
            elif horn_type_id == 4:
                # ATLAS BEETLE: Apply yaw rotation to cephalic horn (scanning left/right)
                # Same as rhino - uniform rotation around Y-axis
                # V/B keys scan the cephalic horn left/right (±15°)
                yawed_x = pitched_x * cos_horn_yaw + pitched_z * sin_horn_yaw
                yawed_z = -pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
            else:
                # RHINO BEETLE (horn_type_id == 0): Apply uniform rotation to entire horn
                yawed_x = pitched_x * cos_horn_yaw + pitched_z * sin_horn_yaw
                yawed_z = -pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw

            # Translate back from pivot
            local_x = yawed_x + horn_pivot_x
            local_y = int(ti.round(yawed_y + float(horn_pivot_y)))
            local_z = yawed_z  # Already centered at 0

        # SCORPION TAIL ROTATION: Apply runtime tail rotation to tail voxels (not claws or body)
        # Tail starts at x = -body_length + 1 and extends forward, curving UPWARD
        if horn_type_id == 3:  # Scorpion only
            # Detect tail voxels: X range at rear, Y starting 3 voxels above attachment, Z near centerline
            # Tail is at centerline (z = -1, 0, 1) while claws are at sides (larger |z|)
            # This combination isolates tail from body/claws regardless of back_body_height
            is_tail_voxel = (body_cache_x[i] >= -body_length) and (body_cache_x[i] <= -body_length + 30) and (body_cache_y[i] >= back_body_height + 3) and (abs(body_cache_z[i]) <= 2)

            if is_tail_voxel:
                # Tail attachment point in body-local coordinates (from generate_scorpion_stinger)
                # Attachment is at top of abdomen: x = -body_length + 1, y = back_body_height, z = 0
                # Body cache uses positive X forward, so attachment is at negative X
                tail_pivot_x = -body_length + 1.0
                tail_pivot_y = float(back_body_height)

                # Translate to tail pivot
                tail_rel_x = local_x - tail_pivot_x
                tail_rel_y = float(local_y) - tail_pivot_y

                # Apply tail rotation in X-Y plane (around Z-axis)
                # tail_pitch is already in radians and includes base 15 degrees + dynamic adjustment
                cos_tail = ti.cos(tail_pitch)
                sin_tail = ti.sin(tail_pitch)

                rotated_tail_x = tail_rel_x * cos_tail - tail_rel_y * sin_tail
                rotated_tail_y = tail_rel_x * sin_tail + tail_rel_y * cos_tail

                # Translate back from tail pivot
                local_x = rotated_tail_x + tail_pivot_x
                local_y = int(ti.round(rotated_tail_y + tail_pivot_y))
                # local_z unchanged (rotation around Z-axis)

        # 3D rotation: Apply yaw → pitch → roll (standard rotation order)
        # Convert local_y to float for rotation
        ly = float(local_y)

        # Step 1: Yaw rotation (around Y-axis)
        temp_x = local_x * cos_yaw - local_z * sin_yaw
        temp_z = local_x * sin_yaw + local_z * cos_yaw
        temp_y = ly

        # Step 2: Pitch rotation (around Z-axis) - nose up/down
        temp2_x = temp_x * cos_pitch_body - temp_y * sin_pitch_body
        temp2_y = temp_x * sin_pitch_body + temp_y * cos_pitch_body
        temp2_z = temp_z

        # Step 3: Roll rotation (around X-axis) - tilt left/right
        final_x = temp2_x
        final_y = temp2_y * cos_roll - temp2_z * sin_roll
        final_z = temp2_y * sin_roll + temp2_z * cos_roll

        grid_x = center_x + int(ti.round(final_x))
        grid_y = base_y + int(ti.round(final_y))
        grid_z = center_z + int(ti.round(final_z))

        if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid and 0 <= grid_y < simulation.n_grid:
            # Don't overwrite floor (CONCRETE), shadow, slippery bowl, or goal voxels
            existing_voxel = simulation.voxel_type[grid_x, grid_y, grid_z]
            if existing_voxel != simulation.CONCRETE and existing_voxel != simulation.SHADOW and existing_voxel != simulation.SLIPPERY and existing_voxel != simulation.GOAL:
                # OPTIMIZATION: Use pre-computed voxel metadata instead of calculating every frame
                # Eliminates ~90 lines of conditional logic per voxel (600-800 voxels per beetle)
                is_hook_interior = blue_body_hook_flags[i]
                is_stripe = blue_body_stripe_flags[i]
                is_horn_tip = blue_body_horn_tip_flags[i]
                is_very_tip = blue_body_very_tip_flags[i]

                # Use appropriate color: hook interior > horn tip > stripe > body color
                voxel_color = body_color

                if is_hook_interior == 1:
                    # Hook interior voxels use special type
                    voxel_color = simulation.STAG_HOOK_INTERIOR_BLUE
                elif is_horn_tip == 1:
                    # Apply color based on tip type
                    if is_very_tip == 1:  # VERY tips: dark leg tip color
                        if body_color == simulation.BEETLE_BLUE:
                            voxel_color = simulation.LEG_TIP_BLUE
                        elif body_color == simulation.BEETLE_RED:
                            voxel_color = simulation.LEG_TIP_RED
                    else:  # Regular tips: bright horn tip color (all beetles including scorpion)
                        if body_color == simulation.BEETLE_BLUE:
                            voxel_color = simulation.BEETLE_BLUE_HORN_TIP
                        elif body_color == simulation.BEETLE_RED:
                            voxel_color = simulation.BEETLE_RED_HORN_TIP
                elif is_stripe == 1:
                    if body_color == simulation.BEETLE_BLUE:
                        voxel_color = simulation.BEETLE_BLUE_STRIPE
                    elif body_color == simulation.BEETLE_RED:
                        voxel_color = simulation.BEETLE_RED_STRIPE

                simulation.voxel_type[grid_x, grid_y, grid_z] = voxel_color
                # Track this voxel for efficient clearing later
                idx = ti.atomic_add(dirty_voxel_count[None], 1)
                if idx < MAX_DIRTY_VOXELS:
                    dirty_voxel_x[idx] = grid_x
                    dirty_voxel_y[idx] = grid_y
                    dirty_voxel_z[idx] = grid_z

    # 2. Place legs (animated with tripod gait) - DIFFERENT COLOR
    # Tripod gait: legs 0, 3, 4 move together (Group A), legs 1, 2, 5 move together (Group B)
    # leg_id: 0=front_left, 1=front_right, 2=middle_left, 3=middle_right, 4=rear_left, 5=rear_right
    # Scorpion adds: 6=rear2_left, 7=rear2_right

    for leg_id in range(8):  # Up to 8 legs for scorpion, 6 for beetles
        # Determine leg phase offset for gait pattern
        # SCORPION (horn_type_id == 3): Quadrupod gait (like tripod but for 8 legs)
        #   Group A (0, 3, 4, 7): phase_offset = 0
        #   Group B (1, 2, 5, 6): phase_offset = π
        # BEETLE: Tripod gait
        #   Group A (0, 3, 4): phase_offset = 0
        #   Group B (1, 2, 5): phase_offset = π
        phase_offset = 0.0
        if horn_type_id == 3:  # Scorpion: quadrupod gait (similar to tripod but for 8 legs)
            # Group A (0, 3, 4, 7): phase_offset = 0
            # Group B (1, 2, 5, 6): phase_offset = π
            if leg_id == 1 or leg_id == 2 or leg_id == 5 or leg_id == 6:
                phase_offset = 3.14159265359  # π
        else:  # Beetle: tripod gait
            if leg_id == 1 or leg_id == 2 or leg_id == 5:
                phase_offset = 3.14159265359  # π

        # Apply asymmetric leg timing for rotation-only movement
        leg_phase_offset = phase_offset
        if is_rotating_only == 1:
            # Asymmetric timing: inside legs (toward turn direction) lag behind
            if rotation_direction == -1:  # Turning left
                # Left legs (leg_id 0,1,2) lag, right legs (3,4,5) normal
                if leg_id == 0 or leg_id == 1 or leg_id == 2:
                    leg_phase_offset += 0.6  # ~35 degree phase lag
            elif rotation_direction == 1:  # Turning right
                # Right legs lag, left legs normal
                if leg_id == 3 or leg_id == 4 or leg_id == 5:
                    leg_phase_offset += 0.6  # ~35 degree phase lag

        leg_phase = walk_phase + leg_phase_offset

        # Calculate animation transforms (vertical lift and minimal forward/back sweep)
        # sin(leg_phase) ranges from -1 to 1
        # We want: lift when sin > 0 (leg in air), on ground when sin <= 0
        base_lift = 2.5  # Normal lift height
        base_sweep = 0.5  # Normal sweep distance

        # Reduce amplitude when rotating only (60% of normal)
        if is_rotating_only == 1:
            base_lift *= 0.6
            base_sweep *= 0.6

        # OPTIMIZATION: Calculate trig once per leg and cache
        leg_sin = ti.sin(leg_phase)
        leg_cos = ti.cos(leg_phase)

        lift = ti.max(0.0, leg_sin) * base_lift  # Lift (reduced during rotation)
        sweep = -leg_cos * base_sweep  # Sweep (reduced during rotation)

        # SPAZ WIGGLE: When beetle is lifted high, add chaotic leg movement
        if is_lifted_high == 1:
            # High-frequency wiggle with per-leg variation for chaos
            # Slower wiggle when rotating only (to prevent excessive speed appearance)
            wiggle_freq = 1.5 if is_rotating_only == 1 else 5.0
            wiggle_phase = leg_phase * wiggle_freq + float(leg_id)  # Each leg different

            # OPTIMIZATION: Pre-calculate wiggle trig
            wiggle_sin = ti.sin(wiggle_phase)
            wiggle_cos = ti.cos(wiggle_phase * 1.3)

            # Add erratic movement to lift and sweep
            wiggle_lift = wiggle_sin * 2.0  # ±2 voxels extra lift
            wiggle_sweep = wiggle_cos * 0.8  # ±0.8 voxels extra sweep

            lift += wiggle_lift
            sweep += wiggle_sweep

        # Place all voxels for this leg
        start_idx = leg_start_idx[leg_id]
        end_idx = leg_end_idx[leg_id]

        for i in range(start_idx, end_idx):
            local_x = float(leg_cache_x[i]) + sweep  # Apply sweep
            local_y = leg_cache_y[i] + int(lift)  # Apply vertical lift
            local_z = float(leg_cache_z[i])

            # 3D rotation (same as body)
            ly = float(local_y)

            # Yaw
            temp_x = local_x * cos_yaw - local_z * sin_yaw
            temp_z = local_x * sin_yaw + local_z * cos_yaw
            temp_y = ly

            # Pitch
            temp2_x = temp_x * cos_pitch_body - temp_y * sin_pitch_body
            temp2_y = temp_x * sin_pitch_body + temp_y * cos_pitch_body
            temp2_z = temp_z

            # Roll
            final_x = temp2_x
            final_y = temp2_y * cos_roll - temp2_z * sin_roll
            final_z = temp2_y * sin_roll + temp2_z * cos_roll

            grid_x = center_x + int(ti.round(final_x))
            grid_y = base_y + int(ti.round(final_y))
            grid_z = center_z + int(ti.round(final_z))

            if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid and 0 <= grid_y < simulation.n_grid:
                # Don't overwrite floor (CONCRETE), shadow, slippery, or goal voxels
                existing_leg = simulation.voxel_type[grid_x, grid_y, grid_z]
                if existing_leg != simulation.CONCRETE and existing_leg != simulation.SHADOW and existing_leg != simulation.SLIPPERY and existing_leg != simulation.GOAL:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = leg_color
                    # Track this voxel for efficient clearing later
                    idx = ti.atomic_add(dirty_voxel_count[None], 1)
                    if idx < MAX_DIRTY_VOXELS:
                        dirty_voxel_x[idx] = grid_x
                        dirty_voxel_y[idx] = grid_y
                        dirty_voxel_z[idx] = grid_z

        # Place leg tips (same animation as legs, but BLACK color for visibility)
        tip_start_idx = leg_tip_start_idx[leg_id]
        tip_end_idx = leg_tip_end_idx[leg_id]

        for i in range(tip_start_idx, tip_end_idx):
            local_x = float(leg_tip_cache_x[i]) + sweep  # Apply same animation
            local_y = leg_tip_cache_y[i] + int(lift)
            local_z = float(leg_tip_cache_z[i])

            # 3D rotation (same as legs)
            ly = float(local_y)

            # Yaw
            temp_x = local_x * cos_yaw - local_z * sin_yaw
            temp_z = local_x * sin_yaw + local_z * cos_yaw
            temp_y = ly

            # Pitch
            temp2_x = temp_x * cos_pitch_body - temp_y * sin_pitch_body
            temp2_y = temp_x * sin_pitch_body + temp_y * cos_pitch_body
            temp2_z = temp_z

            # Roll
            final_x = temp2_x
            final_y = temp2_y * cos_roll - temp2_z * sin_roll
            final_z = temp2_y * sin_roll + temp2_z * cos_roll

            grid_x = center_x + int(ti.round(final_x))
            grid_y = base_y + int(ti.round(final_y))
            grid_z = center_z + int(ti.round(final_z))

            if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid and 0 <= grid_y < simulation.n_grid:
                # Don't overwrite floor (CONCRETE), shadow, slippery, or goal voxels
                existing_tip = simulation.voxel_type[grid_x, grid_y, grid_z]
                if existing_tip != simulation.CONCRETE and existing_tip != simulation.SHADOW and existing_tip != simulation.SLIPPERY and existing_tip != simulation.GOAL:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = leg_tip_color
                    # Track this voxel for efficient clearing later
                    idx = ti.atomic_add(dirty_voxel_count[None], 1)
                    if idx < MAX_DIRTY_VOXELS:
                        dirty_voxel_x[idx] = grid_x
                        dirty_voxel_y[idx] = grid_y
                        dirty_voxel_z[idx] = grid_z

@ti.kernel
def place_animated_beetle_red(world_x: ti.f32, world_y: ti.f32, world_z: ti.f32, rotation: ti.f32, pitch: ti.f32, roll: ti.f32, horn_pitch: ti.f32, horn_yaw: ti.f32, tail_pitch: ti.f32, horn_type_id: ti.i32, body_pitch_offset: ti.f32, body_color: ti.i32, leg_color: ti.i32, leg_tip_color: ti.i32, walk_phase: ti.f32, is_lifted_high: ti.i32, default_horn_pitch: ti.f32, body_length: ti.i32, back_body_height: ti.i32, is_rotating_only: ti.i32, rotation_direction: ti.i32):
    """Beetle placement with 3D rotation (yaw/pitch/roll) and animated legs

    Args:
        horn_yaw: Horizontal horn rotation (stag=pincer spread, rhino/hercules=horn yaw)
        tail_pitch: Scorpion tail rotation angle (degrees, -15 to +15)
        horn_type_id: 0=rhino, 1=stag, 2=hercules, 3=scorpion
        body_pitch_offset: Static body tilt angle for scorpion (radians)
    """
    center_x = int(world_x + simulation.n_grid / 2.0)
    center_z = int(world_z + simulation.n_grid / 2.0)
    base_y = int(world_y + RENDER_Y_OFFSET)  # Apply Y offset for rendering below floor

    # Rotation matrices for yaw, pitch, roll
    cos_yaw = ti.cos(rotation)
    sin_yaw = ti.sin(rotation)
    # Combine physics pitch with static body tilt offset (for scorpion stance)
    total_pitch = pitch + body_pitch_offset
    cos_pitch_body = ti.cos(total_pitch)
    sin_pitch_body = ti.sin(total_pitch)
    cos_roll = ti.cos(roll)
    sin_roll = ti.sin(roll)

    # Pitch rotation for horn (rotation around Z-axis in local space)
    cos_pitch = ti.cos(horn_pitch)
    sin_pitch = ti.sin(horn_pitch)

    # Horn/claw pivot point in local coordinates (where horn/claw attaches to head)
    # Scorpion claws pivot at X=2, beetle horns pivot at X=3
    horn_pivot_x = 2.0 if horn_type_id == 3 else 3.0
    horn_pivot_y = 2

    # OPTIMIZATION: Pre-calculate all trigonometry values ONCE before voxel loop
    # These are constant for all voxels in this beetle, no need to recalculate 600+ times
    cos_horn_pitch = ti.cos(horn_pitch)
    sin_horn_pitch = ti.sin(horn_pitch)
    cos_horn_yaw = ti.cos(horn_yaw)
    sin_horn_yaw = ti.sin(horn_yaw)

    # Scorpion-specific trig (only calculated if needed, but outside loop)
    pitch_deviation = horn_pitch - default_horn_pitch
    cos_default = ti.cos(default_horn_pitch)
    sin_default = ti.sin(default_horn_pitch)
    cos_deviation = ti.cos(pitch_deviation)
    sin_deviation = ti.sin(pitch_deviation)

    # 1. Place body with horn pitch applied
    for i in range(red_body_cache_size[None]):
        local_x = float(red_body_cache_x[i])
        local_y = red_body_cache_y[i]
        local_z = float(red_body_cache_z[i])

        # Apply horn pitch and yaw rotation ONLY to horn voxels (dx >= 2 for scorpion claws, dx >= 3 for others)
        # Scorpion claws start at dx=2, beetle horns start at dx=3
        should_rotate = False
        if horn_type_id == 3:  # Scorpion - rotate claws (dx >= 2 AND |dz| > 2 to exclude centered tail)
            should_rotate = red_body_cache_x[i] >= 2 and abs(local_z) > 2.0
        elif horn_type_id == 4:  # Atlas - only rotate cephalic horn (centered Z position)
            # Cephalic horn: dx >= 3 AND |dz| <= 1 (centered on midline Z=0)
            # Pronotum horns: dx >= 3 AND |dz| >= 2 (spread outward Z=±3+) - DON'T rotate
            should_rotate = red_body_cache_x[i] >= 3 and abs(local_z) <= 1.5
        elif red_body_cache_x[i] >= 3:  # Other beetles - rotate horns (dx >= 3)
            should_rotate = True

        if should_rotate:
            # This is a horn voxel - apply pitch and yaw rotations around pivot point
            # Translate to pivot
            rel_x = local_x - horn_pivot_x
            rel_y = float(local_y - horn_pivot_y)
            rel_z = local_z  # Z is already centered at 0

            # STEP 1: Pitch rotation (around Z-axis in beetle local space)
            # Positive pitch = horn rotates up
            # For Hercules: ONLY bottom horn (y < 3) rotates, top horn (y >= 3) stays fixed
            # NOTE: cos_horn_pitch and sin_horn_pitch already calculated outside loop

            # Initialize pitched results
            pitched_x = rel_x
            pitched_y = rel_y
            pitched_z = rel_z

            # Apply pitch rotation based on beetle type and horn position
            if horn_type_id == 3:
                # SCORPION: Both claws start at default angle, then alternate when R/Y pressed
                # NOTE: pitch_deviation, cos_default, sin_default, cos_deviation, sin_deviation
                # already calculated outside loop for performance

                # First apply default pitch to both claws equally
                temp_x = rel_x * cos_default - rel_y * sin_default
                temp_y = rel_x * sin_default + rel_y * cos_default

                # Then apply alternating deviation
                if pitch_deviation != 0.0:

                    if rel_z < -1.0:  # Left claw - invert deviation
                        pitched_x = temp_x * cos_deviation + temp_y * sin_deviation
                        pitched_y = -temp_x * sin_deviation + temp_y * cos_deviation
                    elif rel_z > 1.0:  # Right claw - normal deviation
                        pitched_x = temp_x * cos_deviation - temp_y * sin_deviation
                        pitched_y = temp_x * sin_deviation + temp_y * cos_deviation
                    else:  # Center voxels use default only
                        pitched_x = temp_x
                        pitched_y = temp_y
                else:  # No deviation, just use default
                    pitched_x = temp_x
                    pitched_y = temp_y
                # else: center voxels don't rotate
            elif horn_type_id == 2:
                # HERCULES: Only rotate bottom horn, top horn stays fixed
                # After -30 degree rotation, bottom horn is mostly Y < 3, with tip reaching ~Y=5
                # Top horn: After +10 degree rotation, starts at Y~8, goes up to Y=20+
                # Simple conservative rule: if Y < 5, it's bottom horn for sure
                # If Y >= 5 and Y < 8, only rotate if far forward (bottom horn tip area)
                if local_y < 5:
                    # Definitely bottom horn (rotated down 30 degrees)
                    pitched_x = rel_x * cos_horn_pitch - rel_y * sin_horn_pitch
                    pitched_y = rel_x * sin_horn_pitch + rel_y * cos_horn_pitch
                elif local_y < 8 and rel_x >= 10.0:
                    # Mid-height but far forward - bottom horn tip only
                    pitched_x = rel_x * cos_horn_pitch - rel_y * sin_horn_pitch
                    pitched_y = rel_x * sin_horn_pitch + rel_y * cos_horn_pitch
                # else: Y >= 8 or near base - top horn stays fixed
            elif horn_type_id == 4:
                # ATLAS: Only rotate cephalic horn (centered), pronotum horns (sides) stay fixed
                # should_rotate already filtered for |Z| <= 1.5, so just apply rotation
                pitched_x = rel_x * cos_horn_pitch - rel_y * sin_horn_pitch
                pitched_y = rel_x * sin_horn_pitch + rel_y * cos_horn_pitch
            else:
                # STAG/RHINO: Rotate entire horn
                pitched_x = rel_x * cos_horn_pitch - rel_y * sin_horn_pitch
                pitched_y = rel_x * sin_horn_pitch + rel_y * cos_horn_pitch

            # Z unchanged by pitch regardless of beetle type
            pitched_z = rel_z

            # STEP 2: Yaw rotation (around Y-axis in beetle local space)
            # Behavior differs based on beetle type
            # NOTE: cos_horn_yaw and sin_horn_yaw already calculated outside loop

            # Initialize yaw results (required for Taichi)
            yawed_x = pitched_x
            yawed_y = pitched_y
            yawed_z = pitched_z

            if horn_type_id == 1:
                # STAG BEETLE: Apply opposite rotations to left vs right pincers
                # Left pincer (z < 0): positive horn_yaw opens outward (more negative Z)
                # Right pincer (z > 0): positive horn_yaw opens outward (more positive Z)
                if pitched_z < -0.1:
                    # Left pincer - apply rotation as-is
                    yawed_x = pitched_x * cos_horn_yaw + pitched_z * sin_horn_yaw
                    yawed_z = -pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
                elif pitched_z > 0.1:
                    # Right pincer - apply inverse rotation
                    yawed_x = pitched_x * cos_horn_yaw - pitched_z * sin_horn_yaw
                    yawed_z = pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
                # else: Center voxels - keep initialized values (no yaw rotation)
            elif horn_type_id == 2:
                # HERCULES BEETLE: Roll/twist both horns around X-axis (forward axis)
                # Like twisting a ball held between top and bottom hands
                # Both horns roll in same direction (B key = both twist right, V key = both twist left)
                yawed_x = pitched_x  # X unchanged (axis of rotation)
                yawed_y = pitched_y * cos_horn_yaw - pitched_z * sin_horn_yaw
                yawed_z = pitched_y * sin_horn_yaw + pitched_z * cos_horn_yaw
            elif horn_type_id == 4:
                # ATLAS BEETLE: Apply yaw rotation to cephalic horn (scanning left/right)
                # Same as rhino - uniform rotation around Y-axis
                # V/B keys scan the cephalic horn left/right (±15°)
                yawed_x = pitched_x * cos_horn_yaw + pitched_z * sin_horn_yaw
                yawed_z = -pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
            else:
                # RHINO BEETLE (horn_type_id == 0): Apply uniform rotation to entire horn
                yawed_x = pitched_x * cos_horn_yaw + pitched_z * sin_horn_yaw
                yawed_z = -pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw

            # Translate back from pivot
            local_x = yawed_x + horn_pivot_x
            local_y = int(ti.round(yawed_y + float(horn_pivot_y)))
            local_z = yawed_z  # Already centered at 0

        # SCORPION TAIL ROTATION: Apply runtime tail rotation to tail voxels (not claws or body)
        # Tail starts at x = -body_length + 1 and extends forward, curving UPWARD
        if horn_type_id == 3:  # Scorpion only
            # Detect tail voxels: X range at rear, Y starting 3 voxels above attachment, Z near centerline
            # Tail is at centerline (z = -1, 0, 1) while claws are at sides (larger |z|)
            # This combination isolates tail from body/claws regardless of back_body_height
            is_tail_voxel = (red_body_cache_x[i] >= -body_length) and (red_body_cache_x[i] <= -body_length + 30) and (red_body_cache_y[i] >= back_body_height + 3) and (abs(red_body_cache_z[i]) <= 2)

            if is_tail_voxel:
                # Tail attachment point in body-local coordinates (from generate_scorpion_stinger)
                # Attachment is at top of abdomen: x = -body_length + 1, y = back_body_height, z = 0
                # Body cache uses positive X forward, so attachment is at negative X
                tail_pivot_x = -body_length + 1.0
                tail_pivot_y = float(back_body_height)

                # Translate to tail pivot
                tail_rel_x = local_x - tail_pivot_x
                tail_rel_y = float(local_y) - tail_pivot_y

                # Apply tail rotation in X-Y plane (around Z-axis)
                # tail_pitch is already in radians and includes base 15 degrees + dynamic adjustment
                cos_tail = ti.cos(tail_pitch)
                sin_tail = ti.sin(tail_pitch)

                rotated_tail_x = tail_rel_x * cos_tail - tail_rel_y * sin_tail
                rotated_tail_y = tail_rel_x * sin_tail + tail_rel_y * cos_tail

                # Translate back from tail pivot
                local_x = rotated_tail_x + tail_pivot_x
                local_y = int(ti.round(rotated_tail_y + tail_pivot_y))
                # local_z unchanged (rotation around Z-axis)

        # 3D rotation: Apply yaw → pitch → roll (standard rotation order)
        # Convert local_y to float for rotation
        ly = float(local_y)

        # Step 1: Yaw rotation (around Y-axis)
        temp_x = local_x * cos_yaw - local_z * sin_yaw
        temp_z = local_x * sin_yaw + local_z * cos_yaw
        temp_y = ly

        # Step 2: Pitch rotation (around Z-axis) - nose up/down
        temp2_x = temp_x * cos_pitch_body - temp_y * sin_pitch_body
        temp2_y = temp_x * sin_pitch_body + temp_y * cos_pitch_body
        temp2_z = temp_z

        # Step 3: Roll rotation (around X-axis) - tilt left/right
        final_x = temp2_x
        final_y = temp2_y * cos_roll - temp2_z * sin_roll
        final_z = temp2_y * sin_roll + temp2_z * cos_roll

        grid_x = center_x + int(ti.round(final_x))
        grid_y = base_y + int(ti.round(final_y))
        grid_z = center_z + int(ti.round(final_z))

        if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid and 0 <= grid_y < simulation.n_grid:
            # Don't overwrite floor (CONCRETE), shadow, slippery bowl, or goal voxels
            existing_voxel = simulation.voxel_type[grid_x, grid_y, grid_z]
            if existing_voxel != simulation.CONCRETE and existing_voxel != simulation.SHADOW and existing_voxel != simulation.SLIPPERY and existing_voxel != simulation.GOAL:
                # OPTIMIZATION: Use pre-computed voxel metadata instead of calculating every frame
                # Eliminates ~90 lines of conditional logic per voxel (600-800 voxels per beetle)
                is_hook_interior = red_body_hook_flags[i]
                is_stripe = red_body_stripe_flags[i]
                is_horn_tip = red_body_horn_tip_flags[i]
                is_very_tip = red_body_very_tip_flags[i]

                # Use appropriate color: hook interior > horn tip > stripe > body color
                voxel_color = body_color

                if is_hook_interior == 1:
                    # Hook interior voxels use special type
                    voxel_color = simulation.STAG_HOOK_INTERIOR_RED
                elif is_horn_tip == 1:
                    # Apply color based on tip type
                    if is_very_tip == 1:  # VERY tips: dark leg tip color
                        if body_color == simulation.BEETLE_BLUE:
                            voxel_color = simulation.LEG_TIP_BLUE
                        elif body_color == simulation.BEETLE_RED:
                            voxel_color = simulation.LEG_TIP_RED
                    else:  # Regular tips: bright horn tip color (all beetles including scorpion)
                        if body_color == simulation.BEETLE_BLUE:
                            voxel_color = simulation.BEETLE_BLUE_HORN_TIP
                        elif body_color == simulation.BEETLE_RED:
                            voxel_color = simulation.BEETLE_RED_HORN_TIP
                elif is_stripe == 1:
                    if body_color == simulation.BEETLE_BLUE:
                        voxel_color = simulation.BEETLE_BLUE_STRIPE
                    elif body_color == simulation.BEETLE_RED:
                        voxel_color = simulation.BEETLE_RED_STRIPE

                simulation.voxel_type[grid_x, grid_y, grid_z] = voxel_color
                # Track this voxel for efficient clearing later
                idx = ti.atomic_add(dirty_voxel_count[None], 1)
                if idx < MAX_DIRTY_VOXELS:
                    dirty_voxel_x[idx] = grid_x
                    dirty_voxel_y[idx] = grid_y
                    dirty_voxel_z[idx] = grid_z

    # 2. Place legs (animated with tripod gait) - DIFFERENT COLOR
    # Tripod gait: legs 0, 3, 4 move together (Group A), legs 1, 2, 5 move together (Group B)
    # leg_id: 0=front_left, 1=front_right, 2=middle_left, 3=middle_right, 4=rear_left, 5=rear_right
    # Scorpion adds: 6=rear2_left, 7=rear2_right

    for leg_id in range(8):  # Up to 8 legs for scorpion, 6 for beetles
        # Determine leg phase offset for gait pattern
        # SCORPION (horn_type_id == 3): Quadrupod gait (like tripod but for 8 legs)
        #   Group A (0, 3, 4, 7): phase_offset = 0
        #   Group B (1, 2, 5, 6): phase_offset = π
        # BEETLE: Tripod gait
        #   Group A (0, 3, 4): phase_offset = 0
        #   Group B (1, 2, 5): phase_offset = π
        phase_offset = 0.0
        if horn_type_id == 3:  # Scorpion: quadrupod gait (similar to tripod but for 8 legs)
            # Group A (0, 3, 4, 7): phase_offset = 0
            # Group B (1, 2, 5, 6): phase_offset = π
            if leg_id == 1 or leg_id == 2 or leg_id == 5 or leg_id == 6:
                phase_offset = 3.14159265359  # π
        else:  # Beetle: tripod gait
            if leg_id == 1 or leg_id == 2 or leg_id == 5:
                phase_offset = 3.14159265359  # π

        # Apply asymmetric leg timing for rotation-only movement
        leg_phase_offset = phase_offset
        if is_rotating_only == 1:
            # Asymmetric timing: inside legs (toward turn direction) lag behind
            if rotation_direction == -1:  # Turning left
                # Left legs (leg_id 0,1,2) lag, right legs (3,4,5) normal
                if leg_id == 0 or leg_id == 1 or leg_id == 2:
                    leg_phase_offset += 0.6  # ~35 degree phase lag
            elif rotation_direction == 1:  # Turning right
                # Right legs lag, left legs normal
                if leg_id == 3 or leg_id == 4 or leg_id == 5:
                    leg_phase_offset += 0.6  # ~35 degree phase lag

        leg_phase = walk_phase + leg_phase_offset

        # Calculate animation transforms (vertical lift and minimal forward/back sweep)
        # sin(leg_phase) ranges from -1 to 1
        # We want: lift when sin > 0 (leg in air), on ground when sin <= 0
        base_lift = 2.5  # Normal lift height
        base_sweep = 0.5  # Normal sweep distance

        # Reduce amplitude when rotating only (60% of normal)
        if is_rotating_only == 1:
            base_lift *= 0.6
            base_sweep *= 0.6

        # OPTIMIZATION: Calculate trig once per leg and cache
        leg_sin = ti.sin(leg_phase)
        leg_cos = ti.cos(leg_phase)

        lift = ti.max(0.0, leg_sin) * base_lift  # Lift (reduced during rotation)
        sweep = -leg_cos * base_sweep  # Sweep (reduced during rotation)

        # SPAZ WIGGLE: When beetle is lifted high, add chaotic leg movement
        if is_lifted_high == 1:
            # High-frequency wiggle with per-leg variation for chaos
            # Slower wiggle when rotating only (to prevent excessive speed appearance)
            wiggle_freq = 1.5 if is_rotating_only == 1 else 5.0
            wiggle_phase = leg_phase * wiggle_freq + float(leg_id)  # Each leg different

            # OPTIMIZATION: Pre-calculate wiggle trig
            wiggle_sin = ti.sin(wiggle_phase)
            wiggle_cos = ti.cos(wiggle_phase * 1.3)

            # Add erratic movement to lift and sweep
            wiggle_lift = wiggle_sin * 2.0  # ±2 voxels extra lift
            wiggle_sweep = wiggle_cos * 0.8  # ±0.8 voxels extra sweep

            lift += wiggle_lift
            sweep += wiggle_sweep

        # Place all voxels for this leg
        start_idx = red_leg_start_idx[leg_id]
        end_idx = red_leg_end_idx[leg_id]

        for i in range(start_idx, end_idx):
            local_x = float(red_leg_cache_x[i]) + sweep  # Apply sweep
            local_y = red_leg_cache_y[i] + int(lift)  # Apply vertical lift
            local_z = float(red_leg_cache_z[i])

            # 3D rotation (same as body)
            ly = float(local_y)

            # Yaw
            temp_x = local_x * cos_yaw - local_z * sin_yaw
            temp_z = local_x * sin_yaw + local_z * cos_yaw
            temp_y = ly

            # Pitch
            temp2_x = temp_x * cos_pitch_body - temp_y * sin_pitch_body
            temp2_y = temp_x * sin_pitch_body + temp_y * cos_pitch_body
            temp2_z = temp_z

            # Roll
            final_x = temp2_x
            final_y = temp2_y * cos_roll - temp2_z * sin_roll
            final_z = temp2_y * sin_roll + temp2_z * cos_roll

            grid_x = center_x + int(ti.round(final_x))
            grid_y = base_y + int(ti.round(final_y))
            grid_z = center_z + int(ti.round(final_z))

            if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid and 0 <= grid_y < simulation.n_grid:
                # Don't overwrite floor (CONCRETE), shadow, slippery, or goal voxels
                existing_leg_r = simulation.voxel_type[grid_x, grid_y, grid_z]
                if existing_leg_r != simulation.CONCRETE and existing_leg_r != simulation.SHADOW and existing_leg_r != simulation.SLIPPERY and existing_leg_r != simulation.GOAL:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = leg_color
                    # Track this voxel for efficient clearing later
                    idx = ti.atomic_add(dirty_voxel_count[None], 1)
                    if idx < MAX_DIRTY_VOXELS:
                        dirty_voxel_x[idx] = grid_x
                        dirty_voxel_y[idx] = grid_y
                        dirty_voxel_z[idx] = grid_z

        # Place leg tips (same animation as legs, but BLACK color for visibility)
        tip_start_idx = red_leg_tip_start_idx[leg_id]
        tip_end_idx = red_leg_tip_end_idx[leg_id]

        for i in range(tip_start_idx, tip_end_idx):
            local_x = float(red_leg_tip_cache_x[i]) + sweep  # Apply same animation
            local_y = red_leg_tip_cache_y[i] + int(lift)
            local_z = float(red_leg_tip_cache_z[i])

            # 3D rotation (same as legs)
            ly = float(local_y)

            # Yaw
            temp_x = local_x * cos_yaw - local_z * sin_yaw
            temp_z = local_x * sin_yaw + local_z * cos_yaw
            temp_y = ly

            # Pitch
            temp2_x = temp_x * cos_pitch_body - temp_y * sin_pitch_body
            temp2_y = temp_x * sin_pitch_body + temp_y * cos_pitch_body
            temp2_z = temp_z

            # Roll
            final_x = temp2_x
            final_y = temp2_y * cos_roll - temp2_z * sin_roll
            final_z = temp2_y * sin_roll + temp2_z * cos_roll

            grid_x = center_x + int(ti.round(final_x))
            grid_y = base_y + int(ti.round(final_y))
            grid_z = center_z + int(ti.round(final_z))

            if 0 <= grid_x < simulation.n_grid and 0 <= grid_z < simulation.n_grid and 0 <= grid_y < simulation.n_grid:
                # Don't overwrite floor (CONCRETE), shadow, slippery, or goal voxels
                existing_tip_r = simulation.voxel_type[grid_x, grid_y, grid_z]
                if existing_tip_r != simulation.CONCRETE and existing_tip_r != simulation.SHADOW and existing_tip_r != simulation.SLIPPERY and existing_tip_r != simulation.GOAL:
                    simulation.voxel_type[grid_x, grid_y, grid_z] = leg_tip_color
                    # Track this voxel for efficient clearing later
                    idx = ti.atomic_add(dirty_voxel_count[None], 1)
                    if idx < MAX_DIRTY_VOXELS:
                        dirty_voxel_x[idx] = grid_x
                        dirty_voxel_y[idx] = grid_y
                        dirty_voxel_z[idx] = grid_z

@ti.kernel
def reset_dirty_voxels():
    """Reset dirty voxel counter before placing beetles"""
    dirty_voxel_count[None] = 0

@ti.kernel
def clear_dirty_voxels():
    """Clear only the voxels we tracked during placement (400x faster than scanning 250K voxels) - Parallel GPU execution"""
    for i in ti.ndrange(dirty_voxel_count[None]):  # Parallel iteration on GPU
        vtype = simulation.voxel_type[dirty_voxel_x[i], dirty_voxel_y[i], dirty_voxel_z[i]]
        # Only clear beetle voxels (not floor/concrete/shadow/slippery/goal) - shadows cleared separately
        if vtype >= simulation.BEETLE_BLUE and vtype != simulation.SHADOW and vtype != simulation.SLIPPERY and vtype != simulation.GOAL:
            simulation.voxel_type[dirty_voxel_x[i], dirty_voxel_y[i], dirty_voxel_z[i]] = simulation.EMPTY

@ti.kernel
def clear_shadow_layer(floor_y: ti.i32, ball_mode_active: ti.i32):
    """Restore shadow voxels back to concrete floor (or slippery if in bowl area and ball mode)"""
    center = 64
    arena_radius = 32.0
    for i, k in ti.ndrange(128, 128):
        if simulation.voxel_type[i, floor_y, k] == simulation.SHADOW:
            # Check if in bowl area (outside arena radius) and ball mode is on
            dx = ti.cast(i - center, ti.f32)
            dz = ti.cast(k - center, ti.f32)
            dist = ti.sqrt(dx * dx + dz * dz)
            if ball_mode_active == 1 and dist > arena_radius:
                simulation.voxel_type[i, floor_y, k] = simulation.SLIPPERY
            else:
                simulation.voxel_type[i, floor_y, k] = simulation.CONCRETE

@ti.kernel
def place_shadow_kernel(x: ti.f32, z: ti.f32, radius: ti.f32, floor_y: ti.i32):
    """Place shadow blob on floor by changing floor voxels to shadow type"""
    grid_x = ti.cast(x + 64, ti.i32)
    grid_z = ti.cast(z + 64, ti.i32)
    radius_int = ti.cast(radius + 1, ti.i32)
    radius_sq = radius * radius
    for dx, dz in ti.ndrange((-radius_int, radius_int + 1), (-radius_int, radius_int + 1)):
        dist_sq = ti.cast(dx * dx + dz * dz, ti.f32)
        if dist_sq <= radius_sq:
            gx = grid_x + dx
            gz = grid_z + dz
            if 0 <= gx < 128 and 0 <= gz < 128:
                # Replace concrete or slippery floor voxels (don't overwrite beetles/other stuff)
                vtype = simulation.voxel_type[gx, floor_y, gz]
                if vtype == simulation.CONCRETE or vtype == simulation.SLIPPERY:
                    simulation.voxel_type[gx, floor_y, gz] = simulation.SHADOW

@ti.kernel
def clear_beetles_bounded(x1: ti.f32, y1: ti.f32, z1: ti.f32, x2: ti.f32, y2: ti.f32, z2: ti.f32):
    """Clear beetles using SEPARATE bounding boxes for each beetle"""
    # Max horn reach: 22 voxels (shaft 14 + prong 7 = 21, plus diagonal extension)
    # Need margin to cover full horn reach in all directions
    margin = 26  # Covers max reach (same as collision detection margin)

    # === BEETLE 1 BOUNDING BOX ===
    center_x1 = int(x1 + simulation.n_grid / 2.0)
    center_z1 = int(z1 + simulation.n_grid / 2.0)

    min_x1 = ti.max(0, center_x1 - margin)
    max_x1 = ti.min(simulation.n_grid, center_x1 + margin)
    min_z1 = ti.max(0, center_z1 - margin)
    max_z1 = ti.min(simulation.n_grid, center_z1 + margin)

    y_margin = 23  # Covers beetle height (~15 voxels) + safety
    min_y1 = ti.max(0, int(y1 + RENDER_Y_OFFSET) - y_margin)
    max_y1 = ti.min(simulation.n_grid, int(y1 + RENDER_Y_OFFSET) + y_margin)

    # Clear beetle 1's area
    for i in range(min_x1, max_x1):
        for j in range(min_y1, max_y1):
            for k in range(min_z1, max_z1):
                vtype = simulation.voxel_type[i, j, k]
                if vtype == simulation.BEETLE_BLUE or vtype == simulation.BEETLE_RED or \
                   vtype == simulation.BEETLE_BLUE_LEGS or vtype == simulation.BEETLE_RED_LEGS or \
                   vtype == simulation.LEG_TIP_BLUE or vtype == simulation.LEG_TIP_RED or \
                   vtype == simulation.BEETLE_BLUE_STRIPE or vtype == simulation.BEETLE_RED_STRIPE or \
                   vtype == simulation.BEETLE_BLUE_HORN_TIP or vtype == simulation.BEETLE_RED_HORN_TIP or \
                   vtype == simulation.STINGER_TIP_BLACK or \
                   vtype == simulation.STAG_HOOK_INTERIOR_BLUE or vtype == simulation.STAG_HOOK_INTERIOR_RED:
                    simulation.voxel_type[i, j, k] = simulation.EMPTY

    # === BEETLE 2 BOUNDING BOX ===
    center_x2 = int(x2 + simulation.n_grid / 2.0)
    center_z2 = int(z2 + simulation.n_grid / 2.0)

    min_x2 = ti.max(0, center_x2 - margin)
    max_x2 = ti.min(simulation.n_grid, center_x2 + margin)
    min_z2 = ti.max(0, center_z2 - margin)
    max_z2 = ti.min(simulation.n_grid, center_z2 + margin)

    min_y2 = ti.max(0, int(y2 + RENDER_Y_OFFSET) - y_margin)
    max_y2 = ti.min(simulation.n_grid, int(y2 + RENDER_Y_OFFSET) + y_margin)

    # Clear beetle 2's area
    for i in range(min_x2, max_x2):
        for j in range(min_y2, max_y2):
            for k in range(min_z2, max_z2):
                vtype = simulation.voxel_type[i, j, k]
                if vtype == simulation.BEETLE_BLUE or vtype == simulation.BEETLE_RED or \
                   vtype == simulation.BEETLE_BLUE_LEGS or vtype == simulation.BEETLE_RED_LEGS or \
                   vtype == simulation.LEG_TIP_BLUE or vtype == simulation.LEG_TIP_RED or \
                   vtype == simulation.BEETLE_BLUE_STRIPE or vtype == simulation.BEETLE_RED_STRIPE or \
                   vtype == simulation.BEETLE_BLUE_HORN_TIP or vtype == simulation.BEETLE_RED_HORN_TIP or \
                   vtype == simulation.STINGER_TIP_BLACK or \
                   vtype == simulation.STAG_HOOK_INTERIOR_BLUE or vtype == simulation.STAG_HOOK_INTERIOR_RED:
                    simulation.voxel_type[i, j, k] = simulation.EMPTY

@ti.kernel
def calculate_occupied_voxels_kernel(world_x: ti.f32, world_z: ti.f32, beetle_color: ti.i32,
                                     occupied_x: ti.template(), occupied_z: ti.template(),
                                     occupied_count: ti.template()):
    """GPU-accelerated calculation of occupied voxels for collision detection"""
    center_x = int(world_x + simulation.n_grid / 2.0)
    center_z = int(world_z + simulation.n_grid / 2.0)

    # Reset count
    occupied_count[None] = 0

    # Scan area around beetle (40x40 area, height range based on render offset)
    y_base = int(RENDER_Y_OFFSET)
    x_min = ti.max(0, center_x - 20)
    x_max = ti.min(simulation.n_grid, center_x + 20)
    z_min = ti.max(0, center_z - 20)
    z_max = ti.min(simulation.n_grid, center_z + 20)
    y_min = ti.max(0, y_base - 5)
    y_max = ti.min(simulation.n_grid, y_base + 30)

    # Scan voxel grid on GPU
    for i in range(x_min, x_max):
        for k in range(z_min, z_max):
            found_in_column = 0
            for j in range(y_min, y_max):
                vtype = simulation.voxel_type[i, j, k]
                # Check if voxel belongs to target beetle/ball
                if beetle_color == simulation.BEETLE_BLUE:
                    if vtype == simulation.BEETLE_BLUE or vtype == simulation.BEETLE_BLUE_LEGS or vtype == simulation.LEG_TIP_BLUE or vtype == simulation.BEETLE_BLUE_STRIPE or vtype == simulation.BEETLE_BLUE_HORN_TIP or vtype == simulation.STINGER_TIP_BLACK:
                        found_in_column = 1
                elif beetle_color == simulation.BEETLE_RED:
                    if vtype == simulation.BEETLE_RED or vtype == simulation.BEETLE_RED_LEGS or vtype == simulation.LEG_TIP_RED or vtype == simulation.BEETLE_RED_STRIPE or vtype == simulation.BEETLE_RED_HORN_TIP or vtype == simulation.STINGER_TIP_BLACK:
                        found_in_column = 1
                elif beetle_color == simulation.BALL:  # Ball
                    if vtype == simulation.BALL:
                        found_in_column = 1

            # Add to occupied list if found (atomic increment to avoid race conditions)
            if found_in_column == 1:
                idx = ti.atomic_add(occupied_count[None], 1)
                if idx < MAX_OCCUPIED_VOXELS:
                    occupied_x[idx] = i
                    occupied_z[idx] = k

@ti.kernel
def calculate_collision_point_kernel(overlap_count: ti.i32):
    """OPTIMIZED: GPU-accelerated O(N+M) collision point calculation using spatial hash"""
    # Reset collision data
    collision_point_x[None] = 0.0
    collision_point_y[None] = 0.0
    collision_point_z[None] = 0.0
    collision_contact_count[None] = 0
    collision_has_horn_tips[None] = 0  # Reset horn tip detection
    collision_has_hook_interiors[None] = 0  # Reset hook interior detection

    # Scan through overlapping voxel columns
    y_scan_start = ti.max(0, int(RENDER_Y_OFFSET) - 5)
    y_scan_end = ti.min(simulation.n_grid, int(RENDER_Y_OFFSET) + 35)

    # PHASE 1: Mark beetle1 occupied XZ positions in spatial hash (O(N) operation)
    # OPTIMIZATION: Direct indexing since n_grid=128 matches hash size exactly
    for i in range(beetle1_occupied_count[None]):
        vx1 = beetle1_occupied_x[i]
        vz1 = beetle1_occupied_z[i]
        collision_spatial_hash[vx1, vz1] = 1  # Mark as occupied

    # PHASE 2: Check beetle2 positions against hash (O(M) operation)
    # OPTIMIZATION: Since n_grid=128 and hash is 128x128, vx % 128 == vx (no hash collisions)
    # This eliminates the O(N) verification loop, making this truly O(M)
    for j in range(beetle2_occupied_count[None]):
        vx2 = beetle2_occupied_x[j]
        vz2 = beetle2_occupied_z[j]
        # Direct lookup (no modulo needed since grid is exactly 128x128)
        # Check if this XZ position overlaps with beetle1
        if collision_spatial_hash[vx2, vz2] == 1:
            # Confirmed overlap - scan vertically to find all contact heights
            for vy_grid in range(y_scan_start, y_scan_end):
                vtype = simulation.voxel_type[vx2, vy_grid, vz2]
                if vtype != 0:  # Non-empty voxel
                    # Check if this is a horn tip voxel
                    if vtype == simulation.BEETLE_BLUE_HORN_TIP or vtype == simulation.BEETLE_RED_HORN_TIP or \
                       vtype == simulation.STINGER_TIP_BLACK:
                        collision_has_horn_tips[None] = 1

                    # Check if this is a hook interior voxel
                    if vtype == simulation.STAG_HOOK_INTERIOR_BLUE or vtype == simulation.STAG_HOOK_INTERIOR_RED:
                        collision_has_hook_interiors[None] = 1

                    # Accumulate collision position
                    ti.atomic_add(collision_point_x[None], float(vx2) - simulation.n_grid / 2.0)
                    ti.atomic_add(collision_point_z[None], float(vz2) - simulation.n_grid / 2.0)
                    ti.atomic_add(collision_point_y[None], float(vy_grid) - RENDER_Y_OFFSET)
                    ti.atomic_add(collision_contact_count[None], 1)

    # PHASE 3: Clear spatial hash for next frame (O(N) operation)
    # OPTIMIZATION: Direct indexing since n_grid=128 matches hash size exactly
    for i in range(beetle1_occupied_count[None]):
        vx1 = beetle1_occupied_x[i]
        vz1 = beetle1_occupied_z[i]
        collision_spatial_hash[vx1, vz1] = 0  # Reset


def calculate_horn_length(shaft_len, prong_len, horn_type):
    """Calculate actual horn reach based on geometry and beetle type

    Args:
        shaft_len: Horn shaft length parameter
        prong_len: Horn prong length parameter
        horn_type: "rhino", "stag", "hercules", "scorpion", or "atlas"
    """
    if horn_type == "stag":
        # Stag pincers: shaft is reduced by 2 for pivot, then prong extends
        return float((shaft_len - 2) + prong_len)
    elif horn_type == "hercules":
        # Hercules dual-jaw: top horn is shaft_len, bottom horn is prong_len
        # Return the longer top horn length as the reach
        return float(shaft_len)
    elif horn_type == "atlas":
        # Atlas pronotum horns: length based on prong_len only (stationary horns)
        # 2x scaling for longer horns - each slider tick adds 2 voxels
        return float(prong_len * 2)
    else:  # rhino or scorpion
        # Rhino horn: full shaft + prong
        # Scorpion: shaft controls tail length, prong controls curvature
        return float(shaft_len + prong_len)

# ========== BALL SYSTEM (Beetle Soccer) ==========

def generate_sphere_voxels(radius):
    """Generate voxel coordinates for a sphere (Python function, runs on CPU)"""
    voxels = []
    r_int = int(radius) + 1
    for x in range(-r_int, r_int + 1):
        for y in range(-r_int, r_int + 1):
            for z in range(-r_int, r_int + 1):
                # Check if point is inside sphere
                dist = math.sqrt(x*x + y*y + z*z)
                if dist <= radius:
                    voxels.append((x, y, z))
    return voxels

@ti.kernel
def update_ball_physics(dt: ti.f32, gravity: ti.f32):
    """Update ball physics with gravity, bounce, and friction (GPU kernel)"""
    if simulation.ball_active[None] == 1:
        # Apply gravity
        simulation.ball_vel[None][1] -= gravity * dt

        # Update position
        simulation.ball_pos[None] += simulation.ball_vel[None] * dt

        # Floor collision (bounce)
        ball_y = simulation.ball_pos[None][1]
        ball_radius = simulation.ball_radius[None]
        if ball_y <= ball_radius:
            simulation.ball_pos[None][1] = ball_radius
            simulation.ball_vel[None][1] = -simulation.ball_vel[None][1] * BALL_BOUNCE_COEFFICIENT

        # Apply friction (slow down horizontal velocity)
        simulation.ball_vel[None][0] *= BALL_FRICTION
        simulation.ball_vel[None][2] *= BALL_FRICTION

@ti.kernel
def check_ball_beetle_collision_kernel(beetle_x: ti.f32, beetle_y: ti.f32, beetle_z: ti.f32) -> ti.i32:
    """Voxel-perfect ball-beetle collision detection (GPU kernel) - returns 1 if collision"""
    collision = 0

    if simulation.ball_active[None] == 1:
        ball_x = simulation.ball_pos[None][0]
        ball_y = simulation.ball_pos[None][1]
        ball_z = simulation.ball_pos[None][2]
        radius = simulation.ball_radius[None]

        # Convert to grid coordinates
        ball_grid_x = int(ball_x + simulation.n_grid / 2.0)
        ball_grid_y = int(ball_y + RENDER_Y_OFFSET)
        ball_grid_z = int(ball_z + simulation.n_grid / 2.0)

        beetle_grid_x = int(beetle_x + simulation.n_grid / 2.0)
        beetle_grid_y = int(beetle_y + RENDER_Y_OFFSET)
        beetle_grid_z = int(beetle_z + simulation.n_grid / 2.0)

        # Early rejection: if ball and beetle are too far apart
        dx = ball_grid_x - beetle_grid_x
        dy = ball_grid_y - beetle_grid_y
        dz = ball_grid_z - beetle_grid_z
        dist_sq = dx * dx + dy * dy + dz * dz

        # Ball radius + beetle max radius (~20 voxels with horns)
        max_dist = int(radius) + 25
        if dist_sq < max_dist * max_dist:
            # Check ball sphere volume for overlaps with beetle voxels
            r_int = int(radius) + 1
            for sphere_dx in range(-r_int, r_int + 1):
                for sphere_dy in range(-r_int, r_int + 1):
                    for sphere_dz in range(-r_int, r_int + 1):
                        if collision == 0:  # Early exit if collision found
                            sphere_dist_sq = sphere_dx*sphere_dx + sphere_dy*sphere_dy + sphere_dz*sphere_dz
                            if sphere_dist_sq <= radius * radius:
                                vx = ball_grid_x + sphere_dx
                                vy = ball_grid_y + sphere_dy
                                vz = ball_grid_z + sphere_dz

                                if 0 <= vx < simulation.n_grid and 0 <= vy < simulation.n_grid and 0 <= vz < simulation.n_grid:
                                    voxel = simulation.voxel_type[vx, vy, vz]
                                    # Check if this voxel contains a beetle (blue: 5-13, red: 6-14)
                                    # Exclude both BALL and BALL_STRIPE voxels
                                    if (5 <= voxel <= 14) and voxel != simulation.BALL and voxel != simulation.BALL_STRIPE:
                                        collision = 1

    return collision

@ti.kernel
def clear_ball():
    """Clear all ball voxels from the grid (GPU kernel)"""
    # Clear all BALL and BALL_STRIPE voxels
    for i, j, k in simulation.voxel_type:
        if simulation.voxel_type[i, j, k] == simulation.BALL or simulation.voxel_type[i, j, k] == simulation.BALL_STRIPE:
            simulation.voxel_type[i, j, k] = 0

@ti.kernel
def render_ball(ball_x: ti.f32, ball_y: ti.f32, ball_z: ti.f32, radius: ti.f32, rotation: ti.f32, pitch: ti.f32, roll: ti.f32):
    """Render ball as sphere voxels with 3D rotating stripe pattern (GPU kernel)"""
    # Convert to grid coordinates
    grid_x = int(ball_x + simulation.n_grid / 2.0)
    grid_y = int(ball_y + RENDER_Y_OFFSET)
    grid_z = int(ball_z + simulation.n_grid / 2.0)

    # Pre-calculate trig for full 3D rotation
    cos_yaw = ti.cos(rotation)
    sin_yaw = ti.sin(rotation)
    cos_pitch = ti.cos(pitch)
    sin_pitch = ti.sin(pitch)
    cos_roll = ti.cos(roll)
    sin_roll = ti.sin(roll)

    # Render sphere voxels with rotating stripe pattern
    r_int = int(radius) + 1
    for dx in range(-r_int, r_int + 1):
        for dy in range(-r_int, r_int + 1):
            for dz in range(-r_int, r_int + 1):
                # Check if point is inside sphere
                dist_sq = dx*dx + dy*dy + dz*dz
                if dist_sq <= radius * radius:
                    vx = grid_x + dx
                    vy = grid_y + dy
                    vz = grid_z + dz

                    # Bounds check and only set if voxel is empty (don't overwrite floor/other objects)
                    if 0 <= vx < simulation.n_grid and 0 <= vy < simulation.n_grid and 0 <= vz < simulation.n_grid:
                        if simulation.voxel_type[vx, vy, vz] == 0:  # Only set if empty
                            # Full 3D rotation for stripe pattern (yaw, pitch, roll)
                            # Step 1: Yaw rotation (around Y axis)
                            temp_x = float(dx) * cos_yaw - float(dz) * sin_yaw
                            temp_z = float(dx) * sin_yaw + float(dz) * cos_yaw
                            temp_y = float(dy)

                            # Step 2: Pitch rotation (around Z axis) - topspin/backspin
                            rot_x = temp_x * cos_pitch - temp_y * sin_pitch
                            rot_y = temp_x * sin_pitch + temp_y * cos_pitch
                            rot_z = temp_z

                            # Step 3: Roll rotation (around X axis) - sidespin
                            final_x = rot_x
                            final_y = rot_y * cos_roll - rot_z * sin_roll
                            final_z = rot_y * sin_roll + rot_z * cos_roll

                            # Create single thick stripe down the middle (2 voxels wide on each side)
                            # Stripe appears when rotated X is near 0 (center)
                            stripe_width = 2.0

                            # Use BALL_STRIPE for center stripe, BALL for rest
                            if abs(final_x) <= stripe_width:
                                simulation.voxel_type[vx, vy, vz] = simulation.BALL_STRIPE
                            else:
                                simulation.voxel_type[vx, vy, vz] = simulation.BALL

@ti.kernel
def clear_ball_fast(last_gx: ti.i32, last_gy: ti.i32, last_gz: ti.i32, num_voxels: ti.i32):
    """Clear ball voxels using cached positions (FAST - only clears ~257 voxels instead of 2M)"""
    for i in range(num_voxels):
        vx = last_gx + ball_cache_x[i]
        vy = last_gy + ball_cache_y[i]
        vz = last_gz + ball_cache_z[i]
        if 0 <= vx < 128 and 0 <= vy < 128 and 0 <= vz < 128:
            vtype = simulation.voxel_type[vx, vy, vz]
            if vtype == simulation.BALL or vtype == simulation.BALL_STRIPE:
                simulation.voxel_type[vx, vy, vz] = 0

@ti.kernel
def render_ball_fast(grid_x: ti.i32, grid_y: ti.i32, grid_z: ti.i32,
                     rotation: ti.f32, pitch: ti.f32, roll: ti.f32, num_voxels: ti.i32):
    """Render ball using cached positions with 3D rotating stripe (FAST)"""
    # Pre-calculate trig for full 3D rotation
    cos_yaw = ti.cos(rotation)
    sin_yaw = ti.sin(rotation)
    cos_pitch = ti.cos(pitch)
    sin_pitch = ti.sin(pitch)
    cos_roll = ti.cos(roll)
    sin_roll = ti.sin(roll)
    stripe_width = 2.0

    for i in range(num_voxels):
        dx = ball_cache_x[i]
        dy = ball_cache_y[i]
        dz = ball_cache_z[i]

        vx = grid_x + dx
        vy = grid_y + dy
        vz = grid_z + dz

        if 0 <= vx < 128 and 0 <= vy < 128 and 0 <= vz < 128:
            if simulation.voxel_type[vx, vy, vz] == 0:  # Only set if empty
                # Full 3D rotation for stripe pattern
                # Step 1: Yaw rotation (around Y axis)
                temp_x = float(dx) * cos_yaw - float(dz) * sin_yaw
                temp_z = float(dx) * sin_yaw + float(dz) * cos_yaw
                temp_y = float(dy)

                # Step 2: Pitch rotation (around Z axis)
                rot_x = temp_x * cos_pitch - temp_y * sin_pitch
                rot_y = temp_x * sin_pitch + temp_y * cos_pitch
                rot_z = temp_z

                # Step 3: Roll rotation (around X axis)
                final_x = rot_x

                # Use BALL_STRIPE for center stripe, BALL for rest
                if ti.abs(final_x) <= stripe_width:
                    simulation.voxel_type[vx, vy, vz] = simulation.BALL_STRIPE
                else:
                    simulation.voxel_type[vx, vy, vz] = simulation.BALL

def clear_and_render_ball_fast(ball_x, ball_y, ball_z, rotation, pitch, roll):
    """Clear old ball position and render at new position using cache (wrapper function)"""
    num_voxels = ball_cache_size[None]
    if num_voxels == 0:
        return  # Cache not initialized

    # Calculate new grid position
    grid_x = int(ball_x + simulation.n_grid / 2.0)
    grid_y = int(ball_y + RENDER_Y_OFFSET)
    grid_z = int(ball_z + simulation.n_grid / 2.0)

    # Clear old position if ball was previously rendered
    if ball_last_rendered[None] == 1:
        clear_ball_fast(ball_last_grid_x[None], ball_last_grid_y[None], ball_last_grid_z[None], num_voxels)

    # Render at new position
    render_ball_fast(grid_x, grid_y, grid_z, rotation, pitch, roll, num_voxels)

    # Store current position for next clear
    ball_last_grid_x[None] = grid_x
    ball_last_grid_y[None] = grid_y
    ball_last_grid_z[None] = grid_z
    ball_last_rendered[None] = 1

def check_ball_beetle_collision(beetle_x, beetle_y, beetle_z, beetle_vx, beetle_vz):
    """Check if ball collides with beetle using voxel-perfect detection and apply impulse"""
    if simulation.ball_active[None] == 0:
        return 0

    # Use GPU-based voxel-perfect collision detection (like beetles do)
    has_collision = check_ball_beetle_collision_kernel(beetle_x, beetle_y, beetle_z)
    return has_collision

def apply_ball_collision_response(beetle, multi_collision):
    """Apply collision response between ball and beetle (action-reaction)"""
    ball_pos = simulation.ball_pos[None]
    ball_radius = simulation.ball_radius[None]

    # Calculate separation vector (ball center - beetle center)
    dx = ball_pos[0] - beetle.x
    dy = ball_pos[1] - beetle.y
    dz = ball_pos[2] - beetle.z
    dist = math.sqrt(dx*dx + dy*dy + dz*dz)

    if dist > 0.1:  # Avoid division by zero
        # Normalize collision direction
        nx = dx / dist
        ny = dy / dist
        nz = dz / dist

        # Push ball away from beetle (separation force based on penetration)
        expected_separation = ball_radius + 10.0  # Ball radius + beetle body radius
        penetration = max(0.0, expected_separation - dist)

        # If both beetles colliding, apply MUCH stronger separation and reduce velocity transfer
        if multi_collision:
            separation_strength = penetration * 12.0  # Extra strong to prevent clipping
            velocity_transfer = 0.3  # Reduced transfer when squeezed
        else:
            separation_strength = penetration * 5.0
            velocity_transfer = 0.7

        # ACTION: Push ball away from beetle
        simulation.ball_vel[None][0] += nx * separation_strength
        simulation.ball_vel[None][1] += ny * separation_strength * 0.3
        simulation.ball_vel[None][2] += nz * separation_strength

        # Transfer beetle velocity to ball
        simulation.ball_vel[None][0] += beetle.vx * velocity_transfer
        simulation.ball_vel[None][2] += beetle.vz * velocity_transfer

        # REACTION: Push beetle away from ball (Newton's 3rd law)
        # Ball pushes back on beetle with equal and opposite force
        reaction_strength = separation_strength * 0.5  # Ball is lighter, so less pushback
        beetle.vx -= nx * reaction_strength
        beetle.vz -= nz * reaction_strength

# Initialize beetle horn dimensions with default geometry values (must be after calculate_horn_length definition)
default_shaft = 12
default_prong = 5
default_horn_type = "rhino"
initial_horn_length = calculate_horn_length(default_shaft, default_prong, default_horn_type)
beetle_blue.horn_shaft_len = default_shaft
beetle_blue.horn_prong_len = default_prong
beetle_blue.horn_length = initial_horn_length
beetle_red.horn_shaft_len = default_shaft
beetle_red.horn_prong_len = default_prong
beetle_red.horn_length = initial_horn_length

def calculate_horn_tip_position(beetle):
    """Calculate world position of horn tip using exact voxel placement transform chain"""
    # Horn tip in local coordinates (furthest voxel + safety margin)
    # For both rhino and stag: max_x = 3 + shaft_len + prong_len (approx)
    tip_local_x = beetle.horn_shaft_len + beetle.horn_prong_len + 3.0  # Actual furthest voxel + margin
    # Rhino Y-fork prongs extend upward, stag pincers pitch forward into Y - both need Y-buffer
    tip_local_y = 1.0 + (beetle.horn_prong_len + 2.0 if beetle.horn_type == "rhino" else
                         beetle.horn_prong_len + 2.0 if beetle.horn_type == "stag" else
                         3.0 if beetle.horn_type == "scorpion" else
                         beetle.horn_prong_len * 2 if beetle.horn_type == "atlas" else 0.0)
    tip_local_z = 0.0 if beetle.horn_type == "rhino" else beetle.horn_prong_len  # Stag pincers extend sideways

    # Add safety buffer for tip width (2-3 voxels wide) + voxel edge + rotation margin
    if beetle.horn_type == "stag":
        tip_local_z += 3.0  # Stag tips extend further in Z (was 1.5)
    else:
        tip_local_x += 3.0  # Rhino tips extend further in X (was 1.0)

    # Step 1: Translate to horn pivot (3.0, 1, 0)
    rel_x = tip_local_x - 3.0
    rel_y = tip_local_y - 1.0
    rel_z = tip_local_z

    # Step 2: Apply horn pitch rotation (around Z-axis)
    cos_horn_pitch = math.cos(beetle.horn_pitch)
    sin_horn_pitch = math.sin(beetle.horn_pitch)
    pitched_x = rel_x * cos_horn_pitch - rel_y * sin_horn_pitch
    pitched_y = rel_x * sin_horn_pitch + rel_y * cos_horn_pitch
    pitched_z = rel_z

    # Step 3: Apply horn yaw rotation - behavior differs by beetle type
    cos_horn_yaw = math.cos(beetle.horn_yaw)
    sin_horn_yaw = math.sin(beetle.horn_yaw)

    if beetle.horn_type == "stag":
        # Stag: Y-axis rotation (use right pincer position - maximum extent)
        yawed_x = pitched_x * cos_horn_yaw - pitched_z * sin_horn_yaw
        yawed_z = pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
        yawed_y = pitched_y
    elif beetle.horn_type == "hercules":
        # Hercules: X-axis rotation (roll/twist around forward axis)
        yawed_x = pitched_x
        yawed_y = pitched_y * cos_horn_yaw - pitched_z * sin_horn_yaw
        yawed_z = pitched_y * sin_horn_yaw + pitched_z * cos_horn_yaw
    else:
        # Rhino: Y-axis rotation (horizontal sweep)
        yawed_x = pitched_x * cos_horn_yaw + pitched_z * sin_horn_yaw
        yawed_z = -pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
        yawed_y = pitched_y

    # Step 4: Translate back from pivot
    local_x = yawed_x + 3.0
    local_y = float(int(ti.round(yawed_y + 1.0)))  # Round to match rendering
    local_z = yawed_z

    # Step 5: Apply beetle body yaw rotation (around Y-axis)
    cos_rotation = math.cos(beetle.rotation)
    sin_rotation = math.sin(beetle.rotation)
    rotated_x = local_x * cos_rotation - local_z * sin_rotation
    rotated_z = local_x * sin_rotation + local_z * cos_rotation
    rotated_y = local_y

    # Step 6: Translate to world position
    world_x = beetle.x + rotated_x
    world_y = beetle.y + rotated_y
    world_z = beetle.z + rotated_z

    return world_x, world_y, world_z

def calculate_horn_tip_position_with_yaw(beetle, yaw_angle):
    """Calculate horn tip position with a specific yaw angle (for predictive collision checking)"""
    # Horn tip in local coordinates (same as base function)
    tip_local_x = beetle.horn_shaft_len + beetle.horn_prong_len + 3.0
    # Rhino Y-fork prongs extend upward, need buffer for prong height + rotation safety
    tip_local_y = 1.0 + (beetle.horn_prong_len + 2.0 if beetle.horn_type == "rhino" else 0.0)
    tip_local_z = 0.0 if beetle.horn_type == "rhino" else beetle.horn_prong_len

    # Add safety buffer for tip width + voxel edge + rotation margin
    if beetle.horn_type == "stag":
        tip_local_z += 3.0  # (was 1.5)
    else:
        tip_local_x += 3.0  # (was 1.0)

    # Step 1: Translate to horn pivot
    rel_x = tip_local_x - 3.0
    rel_y = tip_local_y - 1.0
    rel_z = tip_local_z

    # Step 2: Apply horn pitch rotation (around Z-axis) - use current pitch
    cos_horn_pitch = math.cos(beetle.horn_pitch)
    sin_horn_pitch = math.sin(beetle.horn_pitch)
    pitched_x = rel_x * cos_horn_pitch - rel_y * sin_horn_pitch
    pitched_y = rel_x * sin_horn_pitch + rel_y * cos_horn_pitch
    pitched_z = rel_z

    # Step 3: Apply horn yaw rotation - behavior differs by beetle type
    cos_horn_yaw = math.cos(yaw_angle)
    sin_horn_yaw = math.sin(yaw_angle)

    if beetle.horn_type == "stag":
        # Stag: Y-axis rotation (left/right pincers spread)
        yawed_x = pitched_x * cos_horn_yaw - pitched_z * sin_horn_yaw
        yawed_z = pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
        yawed_y = pitched_y
    elif beetle.horn_type == "hercules":
        # Hercules: X-axis rotation (roll/twist around forward axis)
        yawed_x = pitched_x
        yawed_y = pitched_y * cos_horn_yaw - pitched_z * sin_horn_yaw
        yawed_z = pitched_y * sin_horn_yaw + pitched_z * cos_horn_yaw
    else:
        # Rhino: Y-axis rotation (horizontal sweep)
        yawed_x = pitched_x * cos_horn_yaw + pitched_z * sin_horn_yaw
        yawed_z = -pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
        yawed_y = pitched_y

    # Step 4: Translate back from pivot
    local_x = yawed_x + 3.0
    local_y = float(int(ti.round(yawed_y + 1.0)))  # Round to match rendering
    local_z = yawed_z

    # Step 5: Apply beetle body yaw rotation
    cos_rotation = math.cos(beetle.rotation)
    sin_rotation = math.sin(beetle.rotation)
    rotated_x = local_x * cos_rotation - local_z * sin_rotation
    rotated_z = local_x * sin_rotation + local_z * cos_rotation
    rotated_y = local_y

    # Step 6: Translate to world position
    world_x = beetle.x + rotated_x
    world_y = beetle.y + rotated_y
    world_z = beetle.z + rotated_z

    return world_x, world_y, world_z

def calculate_horn_tip_position_with_pitch(beetle, pitch_angle):
    """Calculate horn tip position with a specific pitch angle (for predictive collision checking)"""
    # Horn tip in local coordinates (same as base function)
    tip_local_x = beetle.horn_shaft_len + beetle.horn_prong_len + 3.0
    # Rhino Y-fork prongs extend upward, need buffer for prong height + rotation safety
    tip_local_y = 1.0 + (beetle.horn_prong_len + 2.0 if beetle.horn_type == "rhino" else 0.0)
    tip_local_z = 0.0 if beetle.horn_type == "rhino" else beetle.horn_prong_len

    # Add safety buffer for tip width + voxel edge + rotation margin
    if beetle.horn_type == "stag":
        tip_local_z += 3.0  # (was 1.5)
    else:
        tip_local_x += 3.0  # (was 1.0)

    # Step 1: Translate to horn pivot
    rel_x = tip_local_x - 3.0
    rel_y = tip_local_y - 1.0
    rel_z = tip_local_z

    # Step 2: Apply horn pitch rotation (around Z-axis) - use PROPOSED pitch_angle
    cos_horn_pitch = math.cos(pitch_angle)
    sin_horn_pitch = math.sin(pitch_angle)
    pitched_x = rel_x * cos_horn_pitch - rel_y * sin_horn_pitch
    pitched_y = rel_x * sin_horn_pitch + rel_y * cos_horn_pitch
    pitched_z = rel_z

    # Step 3: Apply horn yaw rotation - behavior differs by beetle type (use current yaw)
    cos_horn_yaw = math.cos(beetle.horn_yaw)
    sin_horn_yaw = math.sin(beetle.horn_yaw)

    if beetle.horn_type == "stag":
        # Stag: Y-axis rotation (left/right pincers spread)
        yawed_x = pitched_x * cos_horn_yaw - pitched_z * sin_horn_yaw
        yawed_z = pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
        yawed_y = pitched_y
    elif beetle.horn_type == "hercules":
        # Hercules: X-axis rotation (roll/twist around forward axis)
        yawed_x = pitched_x
        yawed_y = pitched_y * cos_horn_yaw - pitched_z * sin_horn_yaw
        yawed_z = pitched_y * sin_horn_yaw + pitched_z * cos_horn_yaw
    else:
        # Rhino: Y-axis rotation (horizontal sweep)
        yawed_x = pitched_x * cos_horn_yaw + pitched_z * sin_horn_yaw
        yawed_z = -pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
        yawed_y = pitched_y

    # Step 4: Translate back from pivot
    local_x = yawed_x + 3.0
    local_y = float(int(ti.round(yawed_y + 1.0)))  # Round to match rendering
    local_z = yawed_z

    # Step 5: Apply beetle body yaw rotation
    cos_rotation = math.cos(beetle.rotation)
    sin_rotation = math.sin(beetle.rotation)
    rotated_x = local_x * cos_rotation - local_z * sin_rotation
    rotated_z = local_x * sin_rotation + local_z * cos_rotation
    rotated_y = local_y

    # Step 6: Translate to world position
    world_x = beetle.x + rotated_x
    world_y = beetle.y + rotated_y
    world_z = beetle.z + rotated_z

    return world_x, world_y, world_z

def calculate_horn_tip_position_with_both(beetle, pitch_angle, yaw_angle):
    """Calculate horn tip position with both pitch and yaw (optimized for combined movements)"""
    # Horn tip in local coordinates (same as base function)
    tip_local_x = beetle.horn_shaft_len + beetle.horn_prong_len + 3.0
    # Rhino Y-fork prongs extend upward, need buffer for prong height + rotation safety
    tip_local_y = 1.0 + (beetle.horn_prong_len + 2.0 if beetle.horn_type == "rhino" else 0.0)
    tip_local_z = 0.0 if beetle.horn_type == "rhino" else beetle.horn_prong_len

    # Add safety buffer for tip width + voxel edge + rotation margin
    if beetle.horn_type == "stag":
        tip_local_z += 3.0  # (was 1.5)
    else:
        tip_local_x += 3.0  # (was 1.0)

    # Step 1: Translate to horn pivot
    rel_x = tip_local_x - 3.0
    rel_y = tip_local_y - 1.0
    rel_z = tip_local_z

    # Step 2: Apply horn pitch rotation (around Z-axis) - use PROPOSED pitch_angle
    cos_horn_pitch = math.cos(pitch_angle)
    sin_horn_pitch = math.sin(pitch_angle)
    pitched_x = rel_x * cos_horn_pitch - rel_y * sin_horn_pitch
    pitched_y = rel_x * sin_horn_pitch + rel_y * cos_horn_pitch
    pitched_z = rel_z

    # Step 3: Apply horn yaw rotation - behavior differs by beetle type
    cos_horn_yaw = math.cos(yaw_angle)
    sin_horn_yaw = math.sin(yaw_angle)

    if beetle.horn_type == "stag":
        # Stag: Y-axis rotation (left/right pincers spread)
        yawed_x = pitched_x * cos_horn_yaw - pitched_z * sin_horn_yaw
        yawed_z = pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
        yawed_y = pitched_y
    elif beetle.horn_type == "hercules":
        # Hercules: X-axis rotation (roll/twist around forward axis)
        yawed_x = pitched_x
        yawed_y = pitched_y * cos_horn_yaw - pitched_z * sin_horn_yaw
        yawed_z = pitched_y * sin_horn_yaw + pitched_z * cos_horn_yaw
    else:
        # Rhino: Y-axis rotation (horizontal sweep)
        yawed_x = pitched_x * cos_horn_yaw + pitched_z * sin_horn_yaw
        yawed_z = -pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
        yawed_y = pitched_y

    # Step 4: Translate back from pivot
    local_x = yawed_x + 3.0
    local_y = float(int(ti.round(yawed_y + 1.0)))  # Round to match rendering
    local_z = yawed_z

    # Step 5: Apply beetle body yaw rotation
    cos_rotation = math.cos(beetle.rotation)
    sin_rotation = math.sin(beetle.rotation)
    rotated_x = local_x * cos_rotation - local_z * sin_rotation
    rotated_z = local_x * sin_rotation + local_z * cos_rotation
    rotated_y = local_y

    # Step 6: Translate to world position
    world_x = beetle.x + rotated_x
    world_y = beetle.y + rotated_y
    world_z = beetle.z + rotated_z

    return world_x, world_y, world_z

def calculate_stag_pincer_tips(beetle, pitch_angle, yaw_angle):
    """Calculate both left and right pincer tip positions for stag beetles"""
    # Stag pincer tips in local coordinates
    tip_local_x = beetle.horn_shaft_len + beetle.horn_prong_len + 3.0
    tip_local_y = 1.0
    tip_local_z = beetle.horn_prong_len + 3.0  # Pincers extend sideways + safety buffer (was 1.5)

    # === LEFT PINCER (extends in -Z direction) ===
    # Step 1: Translate to horn pivot
    rel_x = tip_local_x - 3.0
    rel_y = tip_local_y - 1.0
    rel_z = -tip_local_z  # Negative Z for left pincer

    # Step 2: Apply horn pitch rotation
    cos_horn_pitch = math.cos(pitch_angle)
    sin_horn_pitch = math.sin(pitch_angle)
    pitched_x = rel_x * cos_horn_pitch - rel_y * sin_horn_pitch
    pitched_y = rel_x * sin_horn_pitch + rel_y * cos_horn_pitch
    pitched_z = rel_z

    # Step 3: Apply horn yaw rotation (positive yaw opens left pincer outward)
    cos_horn_yaw = math.cos(yaw_angle)
    sin_horn_yaw = math.sin(yaw_angle)
    yawed_x_left = pitched_x * cos_horn_yaw + pitched_z * sin_horn_yaw
    yawed_z_left = -pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
    yawed_y_left = pitched_y

    # Step 4: Translate back from pivot
    local_x_left = yawed_x_left + 3.0
    local_y_left = yawed_y_left + 1.0
    local_z_left = yawed_z_left

    # Step 5: Apply beetle body yaw rotation
    cos_rotation = math.cos(beetle.rotation)
    sin_rotation = math.sin(beetle.rotation)
    rotated_x_left = local_x_left * cos_rotation - local_z_left * sin_rotation
    rotated_z_left = local_x_left * sin_rotation + local_z_left * cos_rotation
    rotated_y_left = local_y_left

    # Step 6: Translate to world position
    left_tip_x = beetle.x + rotated_x_left
    left_tip_y = beetle.y + rotated_y_left
    left_tip_z = beetle.z + rotated_z_left

    # === RIGHT PINCER (extends in +Z direction) ===
    # Step 1: Translate to horn pivot
    rel_z = tip_local_z  # Positive Z for right pincer

    # Step 2: Pitch rotation (same as left)
    pitched_z = rel_z

    # Step 3: Apply horn yaw rotation (inverse rotation for right pincer)
    yawed_x_right = pitched_x * cos_horn_yaw - pitched_z * sin_horn_yaw
    yawed_z_right = pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
    yawed_y_right = pitched_y

    # Step 4: Translate back from pivot
    local_x_right = yawed_x_right + 3.0
    local_y_right = yawed_y_right + 1.0
    local_z_right = yawed_z_right

    # Step 5: Apply beetle body yaw rotation
    rotated_x_right = local_x_right * cos_rotation - local_z_right * sin_rotation
    rotated_z_right = local_x_right * sin_rotation + local_z_right * cos_rotation
    rotated_y_right = local_y_right

    # Step 6: Translate to world position
    right_tip_x = beetle.x + rotated_x_right
    right_tip_y = beetle.y + rotated_y_right
    right_tip_z = beetle.z + rotated_z_right

    return (left_tip_x, left_tip_y, left_tip_z), (right_tip_x, right_tip_y, right_tip_z)

def get_stag_pincer_inner_points(beetle, pitch_angle, yaw_angle):
    """Get sample points along the inner edges of stag pincers for collision detection.

    The inner edge is the side facing the gap between pincers - where objects get trapped.
    For L-shaped pincers: samples along horizontal inner face + vertical inner face.

    Returns list of world-space (x, y, z) points for both pincers' inner edges.
    """
    inner_points = []

    horizontal_length = beetle.horn_shaft_len
    vertical_length = beetle.horn_prong_len
    inward_factor = 0.36  # 20 degrees inward angle

    # Precompute rotations
    cos_horn_pitch = math.cos(pitch_angle)
    sin_horn_pitch = math.sin(pitch_angle)
    cos_horn_yaw = math.cos(yaw_angle)
    sin_horn_yaw = math.sin(yaw_angle)
    cos_rotation = math.cos(beetle.rotation)
    sin_rotation = math.sin(beetle.rotation)

    def transform_point(local_x, local_y, local_z, is_left_pincer):
        """Transform a local pincer point to world coordinates"""
        # Step 1: Translate to horn pivot
        rel_x = local_x - 3.0
        rel_y = local_y - 1.0
        rel_z = local_z

        # Step 2: Apply horn pitch rotation
        pitched_x = rel_x * cos_horn_pitch - rel_y * sin_horn_pitch
        pitched_y = rel_x * sin_horn_pitch + rel_y * cos_horn_pitch
        pitched_z = rel_z

        # Step 3: Apply horn yaw rotation (opposite for left vs right)
        if is_left_pincer:
            yawed_x = pitched_x * cos_horn_yaw + pitched_z * sin_horn_yaw
            yawed_z = -pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
        else:
            yawed_x = pitched_x * cos_horn_yaw - pitched_z * sin_horn_yaw
            yawed_z = pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
        yawed_y = pitched_y

        # Step 4: Translate back from pivot
        local_x_out = yawed_x + 3.0
        local_y_out = yawed_y + 1.0
        local_z_out = yawed_z

        # Step 5: Apply beetle body yaw rotation
        rotated_x = local_x_out * cos_rotation - local_z_out * sin_rotation
        rotated_z = local_x_out * sin_rotation + local_z_out * cos_rotation
        rotated_y = local_y_out

        # Step 6: Translate to world position
        world_x = beetle.x + rotated_x
        world_y = beetle.y + rotated_y
        world_z = beetle.z + rotated_z

        return (world_x, world_y, world_z)

    # === LEFT PINCER INNER EDGE (facing +Z direction, toward the gap) ===
    # Sample points along horizontal section inner edge
    for i in range(0, int(horizontal_length), 2):  # Sample every 2 voxels
        progress = i / float(horizontal_length) if horizontal_length > 0 else 0.0
        local_x = 3.0 + i
        local_y = 1.0
        # Inner edge is at +Z side (toward center gap)
        local_z = -int(i * inward_factor) - 1 + 1  # +1 to get inner edge
        inner_points.append(transform_point(local_x, local_y, local_z, True))

    # Sample points along vertical section inner edge (at elbow and up)
    elbow_x = 3.0 + horizontal_length - 1
    elbow_z_base = -int((horizontal_length - 1) * inward_factor) - 1
    for j in range(0, int(vertical_length), 2):  # Sample every 2 voxels
        local_x = elbow_x
        local_y = 1.0 + j + 1  # Vertical section
        local_z = elbow_z_base + 1  # Inner edge (+Z side)
        inner_points.append(transform_point(local_x, local_y, local_z, True))

    # === RIGHT PINCER INNER EDGE (facing -Z direction, toward the gap) ===
    # Sample points along horizontal section inner edge
    for i in range(0, int(horizontal_length), 2):
        progress = i / float(horizontal_length) if horizontal_length > 0 else 0.0
        local_x = 3.0 + i
        local_y = 1.0
        # Inner edge is at -Z side (toward center gap)
        local_z = int(i * inward_factor) + 1 - 1  # -1 to get inner edge
        inner_points.append(transform_point(local_x, local_y, local_z, False))

    # Sample points along vertical section inner edge
    elbow_x = 3.0 + horizontal_length - 1
    elbow_z_base = int((horizontal_length - 1) * inward_factor) + 1
    for j in range(0, int(vertical_length), 2):
        local_x = elbow_x
        local_y = 1.0 + j + 1
        local_z = elbow_z_base - 1  # Inner edge (-Z side)
        inner_points.append(transform_point(local_x, local_y, local_z, False))

    return inner_points

def calculate_rhino_prong_tips(beetle, pitch_angle, yaw_angle):
    """Calculate both left and right Y-fork prong tip positions for rhino beetles"""
    import math

    # Rhino Y-fork prong tips extend diagonally up and outward from shaft end
    # Based on geometry at lines 1083-1106: prongs extend in both +Z and -Z directions
    prong_len = beetle.horn_prong_len
    shaft_len = beetle.horn_shaft_len

    # Prong tip positions in local coordinates (matching geometry generation)
    # Prongs start at: (3 + shaft_segments, prong_start_y, ±prong_len)
    tip_local_x = 3.0 + shaft_len + prong_len + 2.0  # Extend forward + safety buffer
    tip_local_y = 1.0 + int(shaft_len * 0.3) + prong_len + 1.5  # Rise diagonally + buffer
    tip_local_z = prong_len + 2.0  # Extend sideways + safety buffer

    # === LEFT PRONG (extends in -Z direction) ===
    # Step 1: Translate to horn pivot (3.0, 1.0, 0.0)
    rel_x = tip_local_x - 3.0
    rel_y = tip_local_y - 1.0
    rel_z = -tip_local_z  # Negative Z for left prong

    # Step 2: Apply horn pitch rotation (around Z-axis)
    cos_horn_pitch = math.cos(pitch_angle)
    sin_horn_pitch = math.sin(pitch_angle)
    pitched_x = rel_x * cos_horn_pitch - rel_y * sin_horn_pitch
    pitched_y = rel_x * sin_horn_pitch + rel_y * cos_horn_pitch
    pitched_z = rel_z

    # Step 3: Apply horn yaw rotation (Y-axis rotation, horizontal sweep)
    cos_horn_yaw = math.cos(yaw_angle)
    sin_horn_yaw = math.sin(yaw_angle)
    yawed_x_left = pitched_x * cos_horn_yaw + pitched_z * sin_horn_yaw
    yawed_z_left = -pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
    yawed_y_left = pitched_y

    # Step 4: Translate back from pivot
    local_x_left = yawed_x_left + 3.0
    local_y_left = yawed_y_left + 1.0
    local_z_left = yawed_z_left

    # Step 5: Apply beetle body yaw rotation
    cos_rotation = math.cos(beetle.rotation)
    sin_rotation = math.sin(beetle.rotation)
    rotated_x_left = local_x_left * cos_rotation - local_z_left * sin_rotation
    rotated_z_left = local_x_left * sin_rotation + local_z_left * cos_rotation
    rotated_y_left = local_y_left

    # Step 6: Translate to world position
    left_tip_x = beetle.x + rotated_x_left
    left_tip_y = beetle.y + rotated_y_left
    left_tip_z = beetle.z + rotated_z_left

    # === RIGHT PRONG (extends in +Z direction) ===
    # Step 1: Translate to horn pivot
    rel_x = tip_local_x - 3.0
    rel_y = tip_local_y - 1.0
    rel_z = tip_local_z  # Positive Z for right prong

    # Step 2: Apply horn pitch rotation
    pitched_x = rel_x * cos_horn_pitch - rel_y * sin_horn_pitch
    pitched_y = rel_x * sin_horn_pitch + rel_y * cos_horn_pitch
    pitched_z = rel_z

    # Step 3: Apply horn yaw rotation
    yawed_x_right = pitched_x * cos_horn_yaw + pitched_z * sin_horn_yaw
    yawed_z_right = -pitched_x * sin_horn_yaw + pitched_z * cos_horn_yaw
    yawed_y_right = pitched_y

    # Step 4: Translate back from pivot
    local_x_right = yawed_x_right + 3.0
    local_y_right = yawed_y_right + 1.0
    local_z_right = yawed_z_right

    # Step 5: Apply beetle body yaw rotation
    rotated_x_right = local_x_right * cos_rotation - local_z_right * sin_rotation
    rotated_z_right = local_x_right * sin_rotation + local_z_right * cos_rotation
    rotated_y_right = local_y_right

    # Step 6: Translate to world position
    right_tip_x = beetle.x + rotated_x_right
    right_tip_y = beetle.y + rotated_y_right
    right_tip_z = beetle.z + rotated_z_right

    return (left_tip_x, left_tip_y, left_tip_z), (right_tip_x, right_tip_y, right_tip_z)

def calculate_hercules_jaw_tips(beetle, pitch_angle, yaw_angle):
    """Calculate both top and bottom jaw tip positions for Hercules beetles"""
    import math

    # Calculate actual horn lengths using 1:1 linear slider response with scaling (same as geometry generation)
    actual_top_len = 5 + beetle.horn_shaft_len  # Slider 5→10, Slider 12→17
    actual_bottom_len = 7 + beetle.horn_prong_len  # Slider 0→7, Slider 6→13

    # === TOP JAW (extends from Y=7, rotated +10° around pivot (3,7)) ===
    # Calculate EXACT position of the last voxel (using same formula as geometry generation)
    i = actual_top_len - 1  # Last voxel index
    progress = float(i) / float(actual_top_len) if actual_top_len > 0 else 0.0

    dx = 3 + i  # Extends forward from pivot
    base_angle_rise = int(i * 0.364)  # 20 degree base angle

    # Height curve: rises in first 40%, levels 40-70%, hooks down 70-100%
    if progress < 0.4:
        # Rising section
        dy = 7 + base_angle_rise + int(i * 0.4)
    elif progress < 0.7:
        # Level section at peak
        dy = 7 + base_angle_rise + int(actual_top_len * 0.4 * 0.4)
    else:
        # Downward hook at tip
        hook_progress = (progress - 0.7) / 0.3
        peak_height = 7 + int(actual_top_len * 0.4 * 0.4)
        dy = base_angle_rise + peak_height - int(hook_progress * actual_top_len * 0.15)

    # Step 1: Translate to top horn pivot (3, 7)
    top_pivot_x = 3.0
    top_pivot_y = 7.0
    top_rel_x = float(dx) - top_pivot_x
    top_rel_y = float(dy) - top_pivot_y
    top_rel_z = 0.0

    # Step 2: Apply +10° rotation around top pivot (same as geometry generation)
    top_rotation_angle = math.radians(10)
    cos_top_rot = math.cos(top_rotation_angle)
    sin_top_rot = math.sin(top_rotation_angle)
    top_rotated_x = top_rel_x * cos_top_rot - top_rel_y * sin_top_rot
    top_rotated_y = top_rel_x * sin_top_rot + top_rel_y * cos_top_rot
    top_rotated_z = top_rel_z

    # Get center position after rotation
    center_x = top_rotated_x + top_pivot_x
    center_y = top_rotated_y + top_pivot_y

    # Account for 2x2x2 block placement (dx_off: [0,1], dy_off: [0,1], dz: [-1,0])
    # The furthest voxel is at int(center_x) + 1 in X, int(center_y) + 1 in Y
    # Z voxels at -1 and 0 can rotate to extend Y when roll applied (up to ±0.5 voxels at ±20° twist)
    top_tip_local_x = float(int(center_x) + 1) + 1.5  # +1 for block, +1.5 for width
    top_tip_local_y = float(int(center_y) + 1) + 2.0  # +1 for block, +2.0 for width + rotation safety
    top_tip_local_z = 0.0

    # Recalculate rotation from the buffered tip position
    top_rel_x = top_tip_local_x - 3.0
    top_rel_y = top_tip_local_y - 7.0
    top_rel_z = top_tip_local_z

    top_rotated_x = top_rel_x
    top_rotated_y = top_rel_y
    top_rotated_z = top_rel_z

    # Step 3: Apply yaw/roll (X-axis rotation) - top horn does NOT apply pitch
    cos_horn_yaw = math.cos(yaw_angle)
    sin_horn_yaw = math.sin(yaw_angle)
    top_yawed_x = top_rotated_x
    top_yawed_y = top_rotated_y * cos_horn_yaw - top_rotated_z * sin_horn_yaw
    top_yawed_z = top_rotated_y * sin_horn_yaw + top_rotated_z * cos_horn_yaw

    # Step 4: Translate back from top pivot (add 7 to Y, 3 to X)
    top_local_x = top_yawed_x + 3.0
    top_local_y = float(int(ti.round(top_yawed_y + 7.0)))  # Round to match rendering
    top_local_z = top_yawed_z

    # Step 5: Apply beetle body rotation
    cos_rotation = math.cos(beetle.rotation)
    sin_rotation = math.sin(beetle.rotation)
    top_rotated_x_body = top_local_x * cos_rotation - top_local_z * sin_rotation
    top_rotated_z_body = top_local_x * sin_rotation + top_local_z * cos_rotation
    top_rotated_y_body = top_local_y

    # Step 6: Translate to world position
    top_tip_x = beetle.x + top_rotated_x_body
    top_tip_y = beetle.y + top_rotated_y_body
    top_tip_z = beetle.z + top_rotated_z_body

    # === BOTTOM JAW (extends from Y=1, rotated -30° around pivot (3,1)) ===
    # Calculate EXACT position of the last voxel (using same formula as geometry generation)
    i_bottom = actual_bottom_len - 1  # Last voxel index

    dx_bottom = 3 + i_bottom  # Extends forward from pivot
    base_angle_rise_bottom = int(i_bottom * 0.364)  # 20 degree base angle
    curve_rise_bottom = int((i_bottom * i_bottom) / float(actual_bottom_len) * 0.5)  # Accelerating upward curve
    dy_bottom = 1 + base_angle_rise_bottom + curve_rise_bottom

    # Step 1: Translate to bottom horn pivot (3, 1)
    bottom_pivot_x = 3.0
    bottom_pivot_y = 1.0
    bottom_rel_x_pre = float(dx_bottom) - bottom_pivot_x
    bottom_rel_y_pre = float(dy_bottom) - bottom_pivot_y
    bottom_rel_z_pre = 0.0

    # Step 2: Apply -30° rotation around bottom pivot (same as geometry generation)
    bottom_rotation_angle = math.radians(-30)
    cos_bottom_rot = math.cos(bottom_rotation_angle)
    sin_bottom_rot = math.sin(bottom_rotation_angle)
    bottom_pre_rotated_x = bottom_rel_x_pre * cos_bottom_rot - bottom_rel_y_pre * sin_bottom_rot
    bottom_pre_rotated_y = bottom_rel_x_pre * sin_bottom_rot + bottom_rel_y_pre * cos_bottom_rot
    bottom_pre_rotated_z = bottom_rel_z_pre

    # Get center position after -30° rotation
    center_x_bottom = bottom_pre_rotated_x + bottom_pivot_x
    center_y_bottom = bottom_pre_rotated_y + bottom_pivot_y

    # Account for 2x2x2 block placement - furthest voxel is at int(center_x) + 1, int(center_y) + 1
    # Z voxels at -1 and 0 can rotate to extend Y when roll applied (up to ±0.5 voxels at ±20° twist)
    bottom_tip_local_x = float(int(center_x_bottom) + 1) + 1.5  # +1 for block, +1.5 for width
    bottom_tip_local_y = float(int(center_y_bottom) + 1) + 2.0  # +1 for block, +2.0 for width + rotation safety
    bottom_tip_local_z = 0.0

    # Recalculate from buffered tip position for pitch/yaw application
    bottom_rel_x = bottom_tip_local_x - 3.0
    bottom_rel_y = bottom_tip_local_y - 1.0
    bottom_rel_z = bottom_tip_local_z

    # Step 3: Apply pitch rotation (R/Y keys) - ONLY bottom jaw rotates with pitch
    cos_horn_pitch = math.cos(pitch_angle)
    sin_horn_pitch = math.sin(pitch_angle)
    bottom_pitched_x = bottom_rel_x * cos_horn_pitch - bottom_rel_y * sin_horn_pitch
    bottom_pitched_y = bottom_rel_x * sin_horn_pitch + bottom_rel_y * cos_horn_pitch
    bottom_pitched_z = bottom_rel_z

    # Don't re-apply the -30° rotation - it's already baked into the tip position
    bottom_rotated_x = bottom_pitched_x
    bottom_rotated_y = bottom_pitched_y
    bottom_rotated_z = bottom_pitched_z

    # Step 4: Apply yaw/roll (X-axis rotation)
    bottom_yawed_x = bottom_rotated_x
    bottom_yawed_y = bottom_rotated_y * cos_horn_yaw - bottom_rotated_z * sin_horn_yaw
    bottom_yawed_z = bottom_rotated_y * sin_horn_yaw + bottom_rotated_z * cos_horn_yaw

    # Step 5: Translate back from bottom pivot (add 1 to Y, 3 to X)
    bottom_local_x = bottom_yawed_x + 3.0
    bottom_local_y = float(int(ti.round(bottom_yawed_y + 1.0)))  # Round to match rendering
    bottom_local_z = bottom_yawed_z

    # Step 6: Apply beetle body rotation
    bottom_rotated_x_body = bottom_local_x * cos_rotation - bottom_local_z * sin_rotation
    bottom_rotated_z_body = bottom_local_x * sin_rotation + bottom_local_z * cos_rotation
    bottom_rotated_y_body = bottom_local_y

    # Step 7: Translate to world position
    bottom_tip_x = beetle.x + bottom_rotated_x_body
    bottom_tip_y = beetle.y + bottom_rotated_y_body
    bottom_tip_z = beetle.z + bottom_rotated_z_body

    return (top_tip_x, top_tip_y, top_tip_z), (bottom_tip_x, bottom_tip_y, bottom_tip_z)

def calculate_min_horn_distance(beetle1, beetle2, pitch1, yaw1, pitch2, yaw2):
    """Calculate minimum distance between horn tips (handles stag, rhino, and hercules)"""

    # OPTIMIZATION: Early distance rejection before expensive calculations
    # Fast 2D planar distance check (cheaper than 3D calculations with sqrt)
    dx_2d = beetle1.x - beetle2.x
    dz_2d = beetle1.z - beetle2.z
    planar_dist_sq = dx_2d * dx_2d + dz_2d * dz_2d

    # Early rejection: if beetles more than combined horn reach apart, skip calculations
    # Max horn reach per beetle ≈ 17 voxels, so 2× + safety margin = 40 voxels
    MAX_HORN_REACH_SQ = 1600.0  # 40^2
    if planar_dist_sq > MAX_HORN_REACH_SQ:
        return 999.0  # Definitely safe, no collision possible

    # OPTIMIZATION: Cache check - if horn angles haven't changed, return cached distance
    # Use small tolerance (0.01 radians ≈ 0.6 degrees) to catch tiny floating point differences
    ANGLE_TOLERANCE = 0.01
    pitch1_unchanged = abs(pitch1 - beetle1.cached_horn_pitch) < ANGLE_TOLERANCE
    yaw1_unchanged = abs(yaw1 - beetle1.cached_horn_yaw) < ANGLE_TOLERANCE
    pitch2_unchanged = abs(pitch2 - beetle2.cached_horn_pitch) < ANGLE_TOLERANCE
    yaw2_unchanged = abs(yaw2 - beetle2.cached_horn_yaw) < ANGLE_TOLERANCE

    if pitch1_unchanged and yaw1_unchanged and pitch2_unchanged and yaw2_unchanged:
        # All angles unchanged - return cached result
        return beetle1.cached_min_distance

    # CASE 1: Both rhino - check all 4 Y-fork prong tip combinations
    if beetle1.horn_type == "rhino" and beetle2.horn_type == "rhino":
        (b1_left_x, b1_left_y, b1_left_z), (b1_right_x, b1_right_y, b1_right_z) = calculate_rhino_prong_tips(beetle1, pitch1, yaw1)
        (b2_left_x, b2_left_y, b2_left_z), (b2_right_x, b2_right_y, b2_right_z) = calculate_rhino_prong_tips(beetle2, pitch2, yaw2)

        # Calculate all 4 distances: left-left, left-right, right-left, right-right
        dx = b1_left_x - b2_left_x
        dy = b1_left_y - b2_left_y
        dz = b1_left_z - b2_left_z
        dist_ll = math.sqrt(dx*dx + dy*dy + dz*dz)

        dx = b1_left_x - b2_right_x
        dy = b1_left_y - b2_right_y
        dz = b1_left_z - b2_right_z
        dist_lr = math.sqrt(dx*dx + dy*dy + dz*dz)

        dx = b1_right_x - b2_left_x
        dy = b1_right_y - b2_left_y
        dz = b1_right_z - b2_left_z
        dist_rl = math.sqrt(dx*dx + dy*dy + dz*dz)

        dx = b1_right_x - b2_right_x
        dy = b1_right_y - b2_right_y
        dz = b1_right_z - b2_right_z
        dist_rr = math.sqrt(dx*dx + dy*dy + dz*dz)

        result = min(dist_ll, dist_lr, dist_rl, dist_rr)
        # Update cache
        beetle1.cached_horn_pitch = pitch1
        beetle1.cached_horn_yaw = yaw1
        beetle2.cached_horn_pitch = pitch2
        beetle2.cached_horn_yaw = yaw2
        beetle1.cached_min_distance = result
        return result

    # CASE 2: Both stag - check all 4 pincer tip combinations
    if beetle1.horn_type == "stag" and beetle2.horn_type == "stag":
        (b1_left_x, b1_left_y, b1_left_z), (b1_right_x, b1_right_y, b1_right_z) = calculate_stag_pincer_tips(beetle1, pitch1, yaw1)
        (b2_left_x, b2_left_y, b2_left_z), (b2_right_x, b2_right_y, b2_right_z) = calculate_stag_pincer_tips(beetle2, pitch2, yaw2)

        # Calculate all 4 distances inline
        dx = b1_left_x - b2_left_x
        dy = b1_left_y - b2_left_y
        dz = b1_left_z - b2_left_z
        dist_ll = math.sqrt(dx*dx + dy*dy + dz*dz)

        dx = b1_left_x - b2_right_x
        dy = b1_left_y - b2_right_y
        dz = b1_left_z - b2_right_z
        dist_lr = math.sqrt(dx*dx + dy*dy + dz*dz)

        dx = b1_right_x - b2_left_x
        dy = b1_right_y - b2_left_y
        dz = b1_right_z - b2_left_z
        dist_rl = math.sqrt(dx*dx + dy*dy + dz*dz)

        dx = b1_right_x - b2_right_x
        dy = b1_right_y - b2_right_y
        dz = b1_right_z - b2_right_z
        dist_rr = math.sqrt(dx*dx + dy*dy + dz*dz)

        result = min(dist_ll, dist_lr, dist_rl, dist_rr)
        # Update cache
        beetle1.cached_horn_pitch = pitch1
        beetle1.cached_horn_yaw = yaw1
        beetle2.cached_horn_pitch = pitch2
        beetle2.cached_horn_yaw = yaw2
        beetle1.cached_min_distance = result
        return result

    # CASE 3: Both hercules - check all 4 jaw tip combinations
    if beetle1.horn_type == "hercules" and beetle2.horn_type == "hercules":
        (b1_top_x, b1_top_y, b1_top_z), (b1_bot_x, b1_bot_y, b1_bot_z) = calculate_hercules_jaw_tips(beetle1, pitch1, yaw1)
        (b2_top_x, b2_top_y, b2_top_z), (b2_bot_x, b2_bot_y, b2_bot_z) = calculate_hercules_jaw_tips(beetle2, pitch2, yaw2)

        # Calculate all 4 distances: top-top, top-bottom, bottom-top, bottom-bottom
        dx = b1_top_x - b2_top_x
        dy = b1_top_y - b2_top_y
        dz = b1_top_z - b2_top_z
        dist_tt = math.sqrt(dx*dx + dy*dy + dz*dz)

        dx = b1_top_x - b2_bot_x
        dy = b1_top_y - b2_bot_y
        dz = b1_top_z - b2_bot_z
        dist_tb = math.sqrt(dx*dx + dy*dy + dz*dz)

        dx = b1_bot_x - b2_top_x
        dy = b1_bot_y - b2_top_y
        dz = b1_bot_z - b2_top_z
        dist_bt = math.sqrt(dx*dx + dy*dy + dz*dz)

        dx = b1_bot_x - b2_bot_x
        dy = b1_bot_y - b2_bot_y
        dz = b1_bot_z - b2_bot_z
        dist_bb = math.sqrt(dx*dx + dy*dy + dz*dz)

        result = min(dist_tt, dist_tb, dist_bt, dist_bb)
        # Update cache
        beetle1.cached_horn_pitch = pitch1
        beetle1.cached_horn_yaw = yaw1
        beetle2.cached_horn_pitch = pitch2
        beetle2.cached_horn_yaw = yaw2
        beetle1.cached_min_distance = result
        return result

    # CASE 4: Mixed types - check dual-tip beetle against single or other dual-tip
    # Handle: stag vs rhino, hercules vs rhino, stag vs hercules

    # Get tips for beetle1
    if beetle1.horn_type == "stag":
        (b1_tip1_x, b1_tip1_y, b1_tip1_z), (b1_tip2_x, b1_tip2_y, b1_tip2_z) = calculate_stag_pincer_tips(beetle1, pitch1, yaw1)
    elif beetle1.horn_type == "hercules":
        (b1_tip1_x, b1_tip1_y, b1_tip1_z), (b1_tip2_x, b1_tip2_y, b1_tip2_z) = calculate_hercules_jaw_tips(beetle1, pitch1, yaw1)
    else:  # rhino
        b1_tip1_x, b1_tip1_y, b1_tip1_z = calculate_horn_tip_position_with_both(beetle1, pitch1, yaw1)
        b1_tip2_x, b1_tip2_y, b1_tip2_z = b1_tip1_x, b1_tip1_y, b1_tip1_z  # Same tip twice for rhino

    # Get tips for beetle2
    if beetle2.horn_type == "stag":
        (b2_tip1_x, b2_tip1_y, b2_tip1_z), (b2_tip2_x, b2_tip2_y, b2_tip2_z) = calculate_stag_pincer_tips(beetle2, pitch2, yaw2)
    elif beetle2.horn_type == "hercules":
        (b2_tip1_x, b2_tip1_y, b2_tip1_z), (b2_tip2_x, b2_tip2_y, b2_tip2_z) = calculate_hercules_jaw_tips(beetle2, pitch2, yaw2)
    else:  # rhino
        b2_tip1_x, b2_tip1_y, b2_tip1_z = calculate_horn_tip_position_with_both(beetle2, pitch2, yaw2)
        b2_tip2_x, b2_tip2_y, b2_tip2_z = b2_tip1_x, b2_tip1_y, b2_tip1_z  # Same tip twice for rhino

    # Calculate all 4 distances (even if some are duplicates for rhino)
    dx = b1_tip1_x - b2_tip1_x
    dy = b1_tip1_y - b2_tip1_y
    dz = b1_tip1_z - b2_tip1_z
    dist_11 = math.sqrt(dx*dx + dy*dy + dz*dz)

    dx = b1_tip1_x - b2_tip2_x
    dy = b1_tip1_y - b2_tip2_y
    dz = b1_tip1_z - b2_tip2_z
    dist_12 = math.sqrt(dx*dx + dy*dy + dz*dz)

    dx = b1_tip2_x - b2_tip1_x
    dy = b1_tip2_y - b2_tip1_y
    dz = b1_tip2_z - b2_tip1_z
    dist_21 = math.sqrt(dx*dx + dy*dy + dz*dz)

    dx = b1_tip2_x - b2_tip2_x
    dy = b1_tip2_y - b2_tip2_y
    dz = b1_tip2_z - b2_tip2_z
    dist_22 = math.sqrt(dx*dx + dy*dy + dz*dz)

    result = min(dist_11, dist_12, dist_21, dist_22)
    # Update cache
    beetle1.cached_horn_pitch = pitch1
    beetle1.cached_horn_yaw = yaw1
    beetle2.cached_horn_pitch = pitch2
    beetle2.cached_horn_yaw = yaw2
    beetle1.cached_min_distance = result
    return result

def check_stag_inner_edge_collision(stag_beetle, target_beetle, pitch, yaw):
    """Check if opponent or their horn is pressing against pincer inner edges.

    Checks:
    1. Opponent body near either pincer inner edge
    2. Opponent horn tips near pincer bases (critical for preventing clip-through)
    Returns (is_collision, min_distance, push_direction_x, push_direction_z)
    """
    # Calculate stag beetle's forward and right directions
    forward_x = math.cos(stag_beetle.rotation)
    forward_z = math.sin(stag_beetle.rotation)
    right_x = -forward_z
    right_z = forward_x

    collision_detected = False
    min_dist = 999.0

    # === CHECK 1: Opponent body position ===
    dx = target_beetle.x - stag_beetle.x
    dz = target_beetle.z - stag_beetle.z
    dist_to_target = math.sqrt(dx*dx + dz*dz)

    if dist_to_target < 22.0:
        dot_forward = dx * forward_x + dz * forward_z
        dot_right = dx * right_x + dz * right_z

        horn_reach = stag_beetle.horn_shaft_len + stag_beetle.horn_prong_len + 5.0

        if dot_forward > 1.0 and dot_forward < horn_reach:
            # Calculate pincer inner edge positions based on yaw
            pincer_offset = 2.0 + max(0.0, yaw) * 15.0

            dist_to_left_inner = abs(dot_right - (-pincer_offset))
            dist_to_right_inner = abs(dot_right - pincer_offset)

            inner_edge_threshold = 2.5
            near_left_inner = dist_to_left_inner < inner_edge_threshold and dot_right < 0
            near_right_inner = dist_to_right_inner < inner_edge_threshold and dot_right > 0

            # Only trigger squeeze when actually touching pincer inner edges
            # No center gap check - squeeze requires contact with the pincers themselves
            if near_left_inner or near_right_inner:
                collision_detected = True
                min_dist = min(dist_to_left_inner, dist_to_right_inner)

    # === CHECK 2: Opponent horn tips near pincer BASES ===
    # This catches horns rotating through the inner base attachment point
    # Get opponent horn tip positions
    if target_beetle.horn_type == "stag":
        (tip1_x, tip1_y, tip1_z), (tip2_x, tip2_y, tip2_z) = calculate_stag_pincer_tips(
            target_beetle, target_beetle.horn_pitch, target_beetle.horn_yaw)
    elif target_beetle.horn_type == "hercules":
        (tip1_x, tip1_y, tip1_z), (tip2_x, tip2_y, tip2_z) = calculate_hercules_jaw_tips(
            target_beetle, target_beetle.horn_pitch, target_beetle.horn_yaw)
    elif target_beetle.horn_type == "rhino":
        (tip1_x, tip1_y, tip1_z), (tip2_x, tip2_y, tip2_z) = calculate_rhino_prong_tips(
            target_beetle, target_beetle.horn_pitch, target_beetle.horn_yaw)
    else:
        # Single-tip horn types
        tip1_x, tip1_y, tip1_z = calculate_horn_tip_position(
            target_beetle, target_beetle.horn_pitch, target_beetle.horn_yaw)
        tip2_x, tip2_y, tip2_z = tip1_x, tip1_y, tip1_z

    # Check multiple points along the inner edge of each pincer shaft
    # Sample from base (x=3) to halfway down the horizontal section
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    # 20 degrees inward factor (matches geometry generation)
    inward_factor = 0.36

    # Sample points along the inner shaft (base to mid-shaft)
    horizontal_length = stag_beetle.horn_shaft_len - 2  # Matches generate_stag_pincers
    sample_points = 4  # Check 4 points along the shaft

    base_threshold = 5.0  # Detection radius around each sample point

    for tip_x, tip_z in [(tip1_x, tip1_z), (tip2_x, tip2_z)]:
        for sample in range(sample_points):
            # Progress along shaft (0 = base, 1 = mid-shaft)
            progress = sample / float(sample_points - 1) if sample_points > 1 else 0.0
            shaft_dist = progress * (horizontal_length * 0.6)  # Check first 60% of shaft

            # Local position along left pincer inner edge
            local_x = 3.0 + shaft_dist
            local_z_left = -int(shaft_dist * inward_factor) - 1 + 1  # Inner edge (+1 toward center)
            local_z_right = int(shaft_dist * inward_factor) + 1 - 1  # Inner edge (-1 toward center)

            # Apply yaw rotation
            left_x = local_x * cos_yaw - local_z_left * sin_yaw
            left_z = local_x * sin_yaw + local_z_left * cos_yaw
            right_x = local_x * cos_yaw - local_z_right * sin_yaw
            right_z = local_x * sin_yaw + local_z_right * cos_yaw

            # Transform to world space
            left_world_x = stag_beetle.x + left_x * forward_x - left_z * right_x
            left_world_z = stag_beetle.z + left_x * forward_z - left_z * right_z
            right_world_x = stag_beetle.x + right_x * forward_x - right_z * right_x
            right_world_z = stag_beetle.z + right_x * forward_z - right_z * right_z

            # Check distance to left inner edge point
            d_left = math.sqrt((tip_x - left_world_x)**2 + (tip_z - left_world_z)**2)
            if d_left < base_threshold:
                collision_detected = True
                min_dist = min(min_dist, d_left)

            # Check distance to right inner edge point
            d_right = math.sqrt((tip_x - right_world_x)**2 + (tip_z - right_world_z)**2)
            if d_right < base_threshold:
                collision_detected = True
                min_dist = min(min_dist, d_right)

    # Push direction: push target away from stag beetle
    push_dir_x = forward_x if collision_detected else 0.0
    push_dir_z = forward_z if collision_detected else 0.0

    return (collision_detected, min_dist, push_dir_x, push_dir_z)

# Stag squeeze damping constant (70% reduction when squeezing - closes at 30% speed)
STAG_SQUEEZE_DAMPING = 0.70

@ti.kernel
def calculate_edge_tipping_kernel(world_x: ti.f32, world_z: ti.f32, beetle_color: ti.i32, dt: ti.f32, pitch_inertia: ti.f32, roll_inertia: ti.f32):
    """GPU-accelerated edge tipping calculation"""
    # Reset outputs
    edge_tipping_vy[None] = 0.0
    edge_tipping_pitch_vel[None] = 0.0
    edge_tipping_roll_vel[None] = 0.0

    # Scan beetle voxels to calculate COG (reduced scan area for performance)
    center_x = int(world_x + simulation.n_grid / 2.0)
    center_z = int(world_z + simulation.n_grid / 2.0)
    y_base = int(RENDER_Y_OFFSET)

    x_min = ti.max(0, center_x - 15)
    x_max = ti.min(simulation.n_grid, center_x + 15)
    z_min = ti.max(0, center_z - 15)
    z_max = ti.min(simulation.n_grid, center_z + 15)
    y_min = ti.max(0, y_base - 3)
    y_max = ti.min(simulation.n_grid, y_base + 20)

    # First pass: calculate center of gravity
    sum_x = 0.0
    sum_y = 0.0
    sum_z = 0.0
    voxel_count = 0

    for i in range(x_min, x_max):
        for k in range(z_min, z_max):
            for j in range(y_min, y_max):
                vtype = simulation.voxel_type[i, j, k]
                is_beetle = 0

                if beetle_color == simulation.BEETLE_BLUE:
                    if vtype == simulation.BEETLE_BLUE or vtype == simulation.BEETLE_BLUE_LEGS or vtype == simulation.LEG_TIP_BLUE or vtype == simulation.BEETLE_BLUE_STRIPE or vtype == simulation.BEETLE_BLUE_HORN_TIP or vtype == simulation.STINGER_TIP_BLACK:
                        is_beetle = 1
                else:
                    if vtype == simulation.BEETLE_RED or vtype == simulation.BEETLE_RED_LEGS or vtype == simulation.LEG_TIP_RED or vtype == simulation.BEETLE_RED_STRIPE or vtype == simulation.BEETLE_RED_HORN_TIP or vtype == simulation.STINGER_TIP_BLACK:
                        is_beetle = 1

                if is_beetle == 1:
                    world_x_v = float(i) - simulation.n_grid / 2.0
                    world_y_v = float(j) - RENDER_Y_OFFSET
                    world_z_v = float(k) - simulation.n_grid / 2.0

                    sum_x += world_x_v
                    sum_y += world_y_v
                    sum_z += world_z_v
                    voxel_count += 1

    # Only continue if voxels were found
    if voxel_count > 0:
        cog_x = sum_x / float(voxel_count)
        cog_y = sum_y / float(voxel_count)
        cog_z = sum_z / float(voxel_count)

        # Second pass: find max lever distance for over-edge voxels
        over_edge_count = 0
        max_lever_dist = 0.0
        arena_center_x = ARENA_CENTER_X - simulation.n_grid / 2.0
        arena_center_z = ARENA_CENTER_Z - simulation.n_grid / 2.0

        for i in range(x_min, x_max):
            for k in range(z_min, z_max):
                for j in range(y_min, y_max):
                    vtype = simulation.voxel_type[i, j, k]
                    is_beetle = 0

                    if beetle_color == simulation.BEETLE_BLUE:
                        if vtype == simulation.BEETLE_BLUE or vtype == simulation.BEETLE_BLUE_LEGS or vtype == simulation.LEG_TIP_BLUE or vtype == simulation.BEETLE_BLUE_STRIPE or vtype == simulation.BEETLE_BLUE_HORN_TIP or vtype == simulation.STINGER_TIP_BLACK:
                            is_beetle = 1
                    else:
                        if vtype == simulation.BEETLE_RED or vtype == simulation.BEETLE_RED_LEGS or vtype == simulation.LEG_TIP_RED or vtype == simulation.BEETLE_RED_STRIPE or vtype == simulation.BEETLE_RED_HORN_TIP or vtype == simulation.STINGER_TIP_BLACK:
                            is_beetle = 1

                    if is_beetle == 1:
                        world_x_v = float(i) - simulation.n_grid / 2.0
                        world_z_v = float(k) - simulation.n_grid / 2.0

                        dist_from_center = ti.sqrt((world_x_v - arena_center_x)**2 + (world_z_v - arena_center_z)**2)

                        if dist_from_center > ARENA_EDGE_RADIUS:
                            over_edge_count += 1
                            lever_dist = ti.sqrt((world_x_v - cog_x)**2 + (world_z_v - cog_z)**2)
                            if lever_dist > max_lever_dist:
                                max_lever_dist = lever_dist

        # Only continue if voxels are over edge
        if over_edge_count > 0:
            tipping_force = float(over_edge_count) * EDGE_TIPPING_STRENGTH

            # Third pass: apply torque at farthest voxels (within 90% of max distance)
            total_vy = 0.0
            total_pitch = 0.0
            total_roll = 0.0
            num_far_voxels = 0

            for i in range(x_min, x_max):
                for k in range(z_min, z_max):
                    for j in range(y_min, y_max):
                        vtype = simulation.voxel_type[i, j, k]
                        is_beetle = 0

                        if beetle_color == simulation.BEETLE_BLUE:
                            if vtype == simulation.BEETLE_BLUE or vtype == simulation.BEETLE_BLUE_LEGS or vtype == simulation.LEG_TIP_BLUE or vtype == simulation.BEETLE_BLUE_STRIPE or vtype == simulation.BEETLE_BLUE_HORN_TIP or vtype == simulation.STINGER_TIP_BLACK:
                                is_beetle = 1
                        else:
                            if vtype == simulation.BEETLE_RED or vtype == simulation.BEETLE_RED_LEGS or vtype == simulation.LEG_TIP_RED or vtype == simulation.BEETLE_RED_STRIPE or vtype == simulation.BEETLE_RED_HORN_TIP or vtype == simulation.STINGER_TIP_BLACK:
                                is_beetle = 1

                        if is_beetle == 1:
                            world_x_v = float(i) - simulation.n_grid / 2.0
                            world_z_v = float(k) - simulation.n_grid / 2.0

                            dist_from_center = ti.sqrt((world_x_v - arena_center_x)**2 + (world_z_v - arena_center_z)**2)

                            if dist_from_center > ARENA_EDGE_RADIUS:
                                lever_dist = ti.sqrt((world_x_v - cog_x)**2 + (world_z_v - cog_z)**2)

                                if lever_dist > max_lever_dist * 0.9 and lever_dist > 0.0 and num_far_voxels < 3:
                                    lever_x = world_x_v - cog_x
                                    lever_z = world_z_v - cog_z

                                    lever_x_norm = lever_x / lever_dist
                                    lever_z_norm = lever_z / lever_dist

                                    total_vy -= tipping_force * dt
                                    total_pitch += (lever_z_norm * tipping_force * lever_dist / pitch_inertia) * dt
                                    total_roll += (lever_x_norm * tipping_force * lever_dist / roll_inertia) * dt
                                    num_far_voxels += 1

            edge_tipping_vy[None] = total_vy
            edge_tipping_pitch_vel[None] = total_pitch
            edge_tipping_roll_vel[None] = total_roll

@ti.kernel
def update_debris_particles(dt: ti.f32):
    """Update debris particle physics - position, velocity, lifetime (GPU parallel)"""
    gravity = 10.0  # Gravity acceleration (lighter for floaty dust arc)
    air_drag = 0.99  # Air resistance per frame (very gentle for smooth motion)

    # Update all active particles in parallel
    for idx in range(simulation.num_debris[None]):
        # Skip dead particles early to save GPU cycles
        if simulation.debris_lifetime[idx] <= 0.0:
            continue

        # Age the particle
        simulation.debris_lifetime[idx] -= dt

        # Update physics (gravity + air drag + position)
        simulation.debris_vel[idx].y -= gravity * dt

        # Apply air drag for more natural motion (slows particles over time)
        simulation.debris_vel[idx] *= air_drag

        simulation.debris_pos[idx] += simulation.debris_vel[idx] * dt

@ti.kernel
def cleanup_dead_debris():
    """Remove dead particles by compacting array (serial due to compaction)"""
    write_idx = 0

    ti.loop_config(serialize=True)
    for read_idx in range(simulation.num_debris[None]):
        if simulation.debris_lifetime[read_idx] > 0.0:
            # Particle still alive, keep it
            if write_idx != read_idx:
                simulation.debris_pos[write_idx] = simulation.debris_pos[read_idx]
                simulation.debris_vel[write_idx] = simulation.debris_vel[read_idx]
                simulation.debris_material[write_idx] = simulation.debris_material[read_idx]
                simulation.debris_lifetime[write_idx] = simulation.debris_lifetime[read_idx]
            write_idx += 1

    simulation.num_debris[None] = write_idx

@ti.kernel
def spawn_death_explosion_batch(pos_x: ti.f32, pos_y: ti.f32, pos_z: ti.f32,
                                  body_r: ti.f32, body_g: ti.f32, body_b: ti.f32,
                                  leg_r: ti.f32, leg_g: ti.f32, leg_b: ti.f32,
                                  stripe_r: ti.f32, stripe_g: ti.f32, stripe_b: ti.f32,
                                  tip_r: ti.f32, tip_g: ti.f32, tip_b: ti.f32,
                                  batch_offset: ti.i32, batch_size: ti.i32, total_particles: ti.i32):
    """Spawn a batch of death explosion particles with adaptive beetle colors - called multiple times for gradual effect"""
    # Pre-compute golden angle constant
    golden_angle = 3.14159 * (1.0 + ti.sqrt(5.0))

    for batch_idx in range(batch_size):
        i = batch_offset + batch_idx
        if i < total_particles:
            # Simplified spherical distribution
            t = (i + 0.5) / total_particles
            phi = ti.acos(1.0 - 2.0 * t)  # Vertical angle
            theta = golden_angle * i  # Golden angle for horizontal

            # 3.12x faster than original (2x * 1.2 * 1.3) for 56% more distance total
            speed = 140.4 + ti.random() * 171.6  # 140-312 units/sec

            # Convert spherical to Cartesian with upward bias
            sin_phi = ti.sin(phi)
            vx = sin_phi * ti.cos(theta) * speed
            vy = ti.abs(ti.cos(phi)) * speed * 1.5  # Upward bias (1.5x vertical)
            vz = sin_phi * ti.sin(theta) * speed

            # Sample color from all beetle colors for maximum variety
            # 55% body, 20% leg, 15% stripe, 10% tip
            rand = ti.random()
            particle_color = ti.math.vec3(body_r, body_g, body_b)  # Default to body
            if rand >= 0.55 and rand < 0.75:
                particle_color = ti.math.vec3(leg_r, leg_g, leg_b)
            elif rand >= 0.75 and rand < 0.90:
                particle_color = ti.math.vec3(stripe_r, stripe_g, stripe_b)
            elif rand >= 0.90:
                particle_color = ti.math.vec3(tip_r, tip_g, tip_b)

            # Add particle to debris system
            idx = ti.atomic_add(simulation.num_debris[None], 1)
            if idx < simulation.MAX_DEBRIS:
                simulation.debris_pos[idx] = ti.math.vec3(pos_x, pos_y, pos_z)
                simulation.debris_vel[idx] = ti.math.vec3(vx, vy, vz)
                simulation.debris_material[idx] = particle_color
                simulation.debris_lifetime[idx] = 0.5 + ti.random() * 0.5  # 0.5-1.0 sec - varied lifetime for more random fade timing

@ti.kernel
def spawn_ball_explosion_batch(pos_x: ti.f32, pos_y: ti.f32, pos_z: ti.f32,
                                batch_offset: ti.i32, batch_size: ti.i32, total_particles: ti.i32):
    """Spawn a batch of ball explosion particles with dung ball colors"""
    # Ball colors (light brown and dark brown stripe)
    ball_main_color = ti.math.vec3(0.65, 0.45, 0.25)  # Light brown
    ball_stripe_color = ti.math.vec3(0.35, 0.22, 0.1)  # Dark brown stripe

    # Pre-compute golden angle constant
    golden_angle = 3.14159 * (1.0 + ti.sqrt(5.0))

    for batch_idx in range(batch_size):
        i = batch_offset + batch_idx
        if i < total_particles:
            # Simplified spherical distribution
            t = (i + 0.5) / total_particles
            phi = ti.acos(1.0 - 2.0 * t)  # Vertical angle
            theta = golden_angle * i  # Golden angle for horizontal

            # Speed similar to beetle explosion
            speed = 140.4 + ti.random() * 171.6  # 140-312 units/sec

            # Convert spherical to Cartesian with upward bias
            sin_phi = ti.sin(phi)
            vx = sin_phi * ti.cos(theta) * speed
            vy = ti.abs(ti.cos(phi)) * speed * 1.5  # Upward bias (1.5x vertical)
            vz = sin_phi * ti.sin(theta) * speed

            # 70% main ball color, 30% stripe color
            rand = ti.random()
            particle_color = ball_main_color
            if rand >= 0.7:
                particle_color = ball_stripe_color

            # Add particle to debris system
            idx = ti.atomic_add(simulation.num_debris[None], 1)
            if idx < simulation.MAX_DEBRIS:
                simulation.debris_pos[idx] = ti.math.vec3(pos_x, pos_y, pos_z)
                simulation.debris_vel[idx] = ti.math.vec3(vx, vy, vz)
                simulation.debris_material[idx] = particle_color
                simulation.debris_lifetime[idx] = 0.5 + ti.random() * 0.5  # 0.5-1.0 sec

@ti.kernel
def spawn_victory_confetti(center_x: ti.f32, center_z: ti.f32, spawn_height: ti.f32,
                           body_r: ti.f32, body_g: ti.f32, body_b: ti.f32,
                           leg_r: ti.f32, leg_g: ti.f32, leg_b: ti.f32,
                           stripe_r: ti.f32, stripe_g: ti.f32, stripe_b: ti.f32,
                           tip_r: ti.f32, tip_g: ti.f32, tip_b: ti.f32,
                           num_particles: ti.i32):
    """Spawn victory confetti particles that rain down around the arena perimeter"""
    for i in range(num_particles):
        # Random spawn position on perimeter ring OUTSIDE the arena
        # Arena radius is 32, so spawn at 35-45 units out (just outside edge)
        spawn_radius = 35.0 + ti.random() * 10.0  # 35-45 unit radius (outside arena)
        angle = ti.random() * 6.28318  # Random angle around center

        pos_x = center_x + ti.cos(angle) * spawn_radius
        pos_z = center_z + ti.sin(angle) * spawn_radius
        pos_y = spawn_height + ti.random() * 10.0  # Slight height variation

        # Gentle downward velocity with some horizontal drift
        vx = (ti.random() - 0.5) * 20.0  # -10 to +10 horizontal drift
        vy = -30.0 - ti.random() * 20.0  # -30 to -50 downward (gentle rain)
        vz = (ti.random() - 0.5) * 20.0  # -10 to +10 horizontal drift

        # Sample color from winner's beetle colors (same distribution as explosion)
        # 55% body, 20% leg, 15% stripe, 10% tip
        rand = ti.random()
        particle_color = ti.math.vec3(body_r, body_g, body_b)  # Default to body
        if rand >= 0.55 and rand < 0.75:
            particle_color = ti.math.vec3(leg_r, leg_g, leg_b)
        elif rand >= 0.75 and rand < 0.90:
            particle_color = ti.math.vec3(stripe_r, stripe_g, stripe_b)
        elif rand >= 0.90:
            particle_color = ti.math.vec3(tip_r, tip_g, tip_b)

        # Add particle to debris system
        idx = ti.atomic_add(simulation.num_debris[None], 1)
        if idx < simulation.MAX_DEBRIS:
            simulation.debris_pos[idx] = ti.math.vec3(pos_x, pos_y, pos_z)
            simulation.debris_vel[idx] = ti.math.vec3(vx, vy, vz)
            simulation.debris_material[idx] = particle_color
            simulation.debris_lifetime[idx] = 1.5 + ti.random() * 1.0  # 1.5-2.5 sec - longer lifetime for confetti

@ti.kernel
def spawn_leg_dust_staggered(pos_x: ti.f32, pos_y: ti.f32, pos_z: ti.f32,
                              dir_x: ti.f32, dir_z: ti.f32, speed: ti.f32,
                              color_r: ti.f32, color_g: ti.f32, color_b: ti.f32,
                              rand_offset_x: ti.f32, rand_offset_z: ti.f32,
                              stagger_scale: ti.f32, num_particles: ti.i32):
    """Spawn staggered dust particles kicked up from leg tip at ~25° angle"""
    # 25° angle: tan(25°) ≈ 0.466, so vertical = horizontal * 0.466
    upward_ratio = 0.466

    for i in range(num_particles):
        idx = ti.atomic_add(simulation.num_debris[None], 1)
        if idx < simulation.MAX_DEBRIS:
            # Stagger particles along kick direction, scaled by leg length
            # 0.7 multiplier keeps total range same as before (was 1.2 with 5 particles)
            stagger = ti.cast(i, ti.f32) * 0.7 * stagger_scale
            # Add per-leg randomization (also scaled) + per-particle lateral spread
            lateral_spread = (ti.random() - 0.5) * 2.0 * stagger_scale  # Side-to-side variance
            spawn_x = pos_x + dir_x * stagger + rand_offset_x * stagger_scale + (ti.random() - 0.5) * 0.8 * stagger_scale - dir_z * lateral_spread
            spawn_z = pos_z + dir_z * stagger + rand_offset_z * stagger_scale + (ti.random() - 0.5) * 0.8 * stagger_scale + dir_x * lateral_spread

            simulation.debris_pos[idx] = ti.math.vec3(spawn_x, pos_y, spawn_z)

            # Kick outward at 45° angle - consistent speed for smooth motion
            particle_speed = speed * (1.1 - ti.cast(i, ti.f32) * 0.04)  # Slight falloff, more uniform
            rand_speed = particle_speed * (0.9 + ti.random() * 0.2)  # Less speed variance for smoother look
            rand_angle = (ti.random() - 0.5) * 0.5  # ±15° horizontal spread (tighter fan)
            vx = dir_x * rand_speed + rand_angle * dir_z * rand_speed
            vz = dir_z * rand_speed - rand_angle * dir_x * rand_speed
            vy = rand_speed * upward_ratio * (0.9 + ti.random() * 0.2)  # Less vertical variance

            simulation.debris_vel[idx] = ti.math.vec3(vx, vy, vz)
            # Dust color with slight variation
            color_var = 0.95 + ti.random() * 0.1
            simulation.debris_material[idx] = ti.math.vec3(color_r * color_var, color_g * color_var, color_b * color_var)
            simulation.debris_lifetime[idx] = 0.5 + ti.random() * 0.2  # 0.5-0.7s to complete full arc back to ground

@ti.kernel
def spawn_spin_dust_puff(pos_x: ti.f32, pos_y: ti.f32, pos_z: ti.f32,
                          dir_x: ti.f32, dir_z: ti.f32,
                          color_r: ti.f32, color_g: ti.f32, color_b: ti.f32,
                          scale: ti.f32, num_particles: ti.i32):
    """Spawn 2 dust particles in narrow cone spread - for spinning"""
    for i in range(2):  # Always spawn 2 particles per call
        idx = ti.atomic_add(simulation.num_debris[None], 1)
        if idx < simulation.MAX_DEBRIS:
            # Cluster spawn around leg tip with random spread
            spread = 1.5 * scale
            spawn_x = pos_x + (ti.random() - 0.5) * spread
            spawn_z = pos_z + (ti.random() - 0.5) * spread

            simulation.debris_pos[idx] = ti.math.vec3(spawn_x, pos_y, spawn_z)

            # Outward kick - very slow so they fall very close
            outward_speed = 1.5 + ti.random() * 0.8  # Very slow, fall in very soon
            upward_speed = outward_speed * 0.466  # ~25° angle like walking dust
            # Narrow cone spread: particle 0 goes slightly left, particle 1 slightly right
            cone_angle = 0.15  # ~8.5° half-angle cone
            side = ti.cast(i, ti.f32) * 2.0 - 1.0  # -1 for i=0, +1 for i=1
            drift_angle = side * cone_angle + (ti.random() - 0.5) * 0.1  # Narrow cone with tiny random
            vx = dir_x * outward_speed + drift_angle * dir_z * outward_speed
            vz = dir_z * outward_speed - drift_angle * dir_x * outward_speed
            vy = upward_speed

            simulation.debris_vel[idx] = ti.math.vec3(vx, vy, vz)
            # Dust color - blend toward white/light for less visual noise against arena
            blend_to_light = 0.5  # 50% blend toward light
            color_var = 0.95 + ti.random() * 0.1
            # Blend color toward lighter value (0.7) instead of darker
            blended_r = color_r * color_var * (1.0 - blend_to_light) + 0.7 * blend_to_light
            blended_g = color_g * color_var * (1.0 - blend_to_light) + 0.68 * blend_to_light
            blended_b = color_b * color_var * (1.0 - blend_to_light) + 0.65 * blend_to_light
            simulation.debris_material[idx] = ti.math.vec3(blended_r, blended_g, blended_b)
            simulation.debris_lifetime[idx] = 1.0 + ti.random() * 0.1  # 1.0-1.1s consistent trail

@ti.kernel
def spawn_ball_bounce_dust(pos_x: ti.f32, pos_y: ti.f32, pos_z: ti.f32,
                           impact_speed: ti.f32, ball_radius: ti.f32):
    """Spawn radial dust ring when ball bounces - more particles for harder impacts"""
    # Scale particle count with impact - fewer at min, more dramatic at high speeds
    # At threshold (7.0): 2 particles, at high speed (15+): ~20 particles
    scaled_speed = ti.max(0.0, impact_speed - 7.0)  # Subtract threshold
    num_particles = 2 + ti.cast(scaled_speed * 2.0, ti.i32)
    num_particles = ti.min(num_particles, 22)  # Cap at 22

    # Dust color (same brownish-gray as leg dust)
    color_r = 0.45
    color_g = 0.40
    color_b = 0.35

    # Speed scales with impact (harder hit = faster dust)
    base_speed = 1.5 + impact_speed * 0.6
    upward_ratio = 0.466  # ~25° angle like leg dust

    # Fixed loop with conditional (Taichi needs compile-time loop bounds)
    for i in range(32):
        if i < num_particles:
            idx = ti.atomic_add(simulation.num_debris[None], 1)
            if idx < simulation.MAX_DEBRIS:
                # Random angle around circle (radial spread)
                angle = ti.random() * 2.0 * 3.14159

                # Spawn at ball edge (slightly inside to look like impact point)
                spawn_radius = ball_radius * 0.8
                spawn_x = pos_x + ti.cos(angle) * spawn_radius
                spawn_z = pos_z + ti.sin(angle) * spawn_radius

                simulation.debris_pos[idx] = ti.math.vec3(spawn_x, pos_y + 0.5, spawn_z)

                # Velocity points outward from center
                particle_speed = base_speed * (0.8 + ti.random() * 0.4)
                vx = ti.cos(angle) * particle_speed
                vz = ti.sin(angle) * particle_speed
                vy = particle_speed * upward_ratio * (0.8 + ti.random() * 0.4)

                simulation.debris_vel[idx] = ti.math.vec3(vx, vy, vz)

                # Dust color with slight variation
                color_var = 0.9 + ti.random() * 0.2
                simulation.debris_material[idx] = ti.math.vec3(color_r * color_var, color_g * color_var, color_b * color_var)

                # Lifetime scales slightly with impact (bigger bounce = longer hang time)
                simulation.debris_lifetime[idx] = 0.4 + ti.min(impact_speed * 0.06, 0.4) + ti.random() * 0.15

@ti.kernel
def spawn_score_burst(pos_x: ti.f32, pos_y: ti.f32, pos_z: ti.f32,
                      body_r: ti.f32, body_g: ti.f32, body_b: ti.f32,
                      leg_r: ti.f32, leg_g: ti.f32, leg_b: ti.f32,
                      stripe_r: ti.f32, stripe_g: ti.f32, stripe_b: ti.f32,
                      tip_r: ti.f32, tip_g: ti.f32, tip_b: ti.f32,
                      num_particles: ti.i32, start_index: ti.i32, total_particles: ti.i32):
    """Spawn celebratory particle burst when score changes - particles fly outward from digit center"""
    for i in range(num_particles):
        particle_i = start_index + i  # Use global index for distribution
        idx = ti.atomic_add(simulation.num_debris[None], 1)
        if idx < simulation.MAX_DEBRIS:
            # Spherical distribution using golden angle for even spread
            golden_angle = 3.14159 * (3.0 - ti.sqrt(5.0))
            theta = golden_angle * ti.cast(particle_i, ti.f32)
            # Vary elevation with some randomness
            phi = ti.acos(1.0 - 2.0 * (ti.cast(particle_i, ti.f32) + ti.random()) / ti.cast(total_particles, ti.f32))

            # Convert to direction
            sin_phi = ti.sin(phi)
            dir_x = sin_phi * ti.cos(theta)
            dir_y = ti.cos(phi)
            dir_z = sin_phi * ti.sin(theta)

            # Spawn at center with slight random offset
            spawn_x = pos_x + (ti.random() - 0.5) * 2.0
            spawn_y = pos_y + (ti.random() - 0.5) * 2.0
            spawn_z = pos_z + (ti.random() - 0.5) * 2.0

            simulation.debris_pos[idx] = ti.math.vec3(spawn_x, spawn_y, spawn_z)

            # Burst outward with varying speeds
            speed = 40.0 + ti.random() * 60.0  # 40-100 units/sec
            simulation.debris_vel[idx] = ti.math.vec3(dir_x * speed, dir_y * speed + 20.0, dir_z * speed)

            # Sample color from beetle colors (same distribution as victory confetti)
            # 55% body, 20% leg, 15% stripe, 10% tip
            rand = ti.random()
            particle_color = ti.math.vec3(body_r, body_g, body_b)  # Default to body
            if rand >= 0.55 and rand < 0.75:
                particle_color = ti.math.vec3(leg_r, leg_g, leg_b)
            elif rand >= 0.75 and rand < 0.90:
                particle_color = ti.math.vec3(stripe_r, stripe_g, stripe_b)
            elif rand >= 0.90:
                particle_color = ti.math.vec3(tip_r, tip_g, tip_b)

            # Add brightness variation (some particles brighter/whiter)
            brightness = 0.8 + ti.random() * 0.4  # 0.8-1.2x
            white_blend = ti.random() * 0.3  # 0-30% white blend for sparkle
            r = particle_color.x * brightness * (1.0 - white_blend) + white_blend
            g = particle_color.y * brightness * (1.0 - white_blend) + white_blend
            b = particle_color.z * brightness * (1.0 - white_blend) + white_blend
            simulation.debris_material[idx] = ti.math.vec3(r, g, b)

            # Short lifetime for snappy burst
            simulation.debris_lifetime[idx] = 0.4 + ti.random() * 0.4  # 0.4-0.8 sec

# Base leg tip offsets from beetle center (unrotated, in voxel units) for leg_length=6
# Beetle coordinate system: +X = forward (head), -X = backward (rear)
# +Z = left side, -Z = right side
# These get scaled by leg_length / 6.0
LEG_TIP_OFFSETS_BASE = [
    (2, 8),   # 0: front_left (forward, left)
    (2, -8),  # 1: front_right (forward, right)
    (-3, 11),  # 2: middle_left (center, far left)
    (-3, -11), # 3: middle_right (center, far right)
    (-10, 8),  # 4: rear_left (backward, left)
    (-10, -8), # 5: rear_right (backward, right)
    (-14, 8),  # 6: rear2_left (scorpion extra back left)
    (-14, -8), # 7: rear2_right (scorpion extra back right)
]

def get_leg_tip_world_position(beetle, leg_id, leg_length=8):
    """Calculate world position of leg tip based on beetle position, rotation, and leg length"""
    base_x, base_z = LEG_TIP_OFFSETS_BASE[leg_id]
    # Softer scaling - sqrt curve so longer legs don't push dust too far
    # At leg_length=6: scale=1.0, at leg_length=10: scale=1.29 (instead of 1.67)
    scale = math.sqrt(leg_length / 6.0)
    local_x = base_x * scale
    local_z = base_z * scale
    # Rotate by beetle facing
    cos_r = math.cos(beetle.rotation)
    sin_r = math.sin(beetle.rotation)
    world_x = beetle.x + local_x * cos_r - local_z * sin_r
    world_z = beetle.z + local_x * sin_r + local_z * cos_r
    return world_x, world_z

def calculate_horn_damping(beetle, collision_x, collision_y, collision_z, engagement_factor):
    """Calculate separate pitch and yaw damping based on their own rotation directions (Phase 2: Horn Clipping Prevention)"""
    # Get horn tip position
    horn_tip_x, horn_tip_y, horn_tip_z = calculate_horn_tip_position(beetle)

    # Vector from horn tip to collision point
    to_collision_x = collision_x - horn_tip_x
    to_collision_y = collision_y - horn_tip_y
    to_collision_z = collision_z - horn_tip_z

    # === PITCH DAMPING (R/Y controls - vertical movement) ===
    rotating_up = beetle.horn_pitch_velocity > 0.1
    rotating_down = beetle.horn_pitch_velocity < -0.1
    collision_above_tip = to_collision_y > 0.0

    # Calculate if pitch is rotating toward or away from collision
    pitch_toward = (rotating_up and collision_above_tip) or (rotating_down and not collision_above_tip)
    pitch_away = (rotating_up and not collision_above_tip) or (rotating_down and collision_above_tip)

    if pitch_toward:
        target_pitch_damping = HORN_DAMPING_INTO * engagement_factor
    elif pitch_away:
        target_pitch_damping = HORN_DAMPING_AWAY * engagement_factor
    else:
        target_pitch_damping = HORN_DAMPING_NEUTRAL * engagement_factor

    # === YAW DAMPING (V/B controls - horizontal spread) ===
    # Calculate horizontal direction to collision relative to beetle facing
    cos_r = math.cos(beetle.rotation)
    sin_r = math.sin(beetle.rotation)
    # Transform collision offset to beetle-local coordinates
    local_collision_x = to_collision_x * cos_r + to_collision_z * sin_r

    rotating_open = beetle.horn_yaw_velocity > 0.1   # Opening pincers / yawing outward
    rotating_close = beetle.horn_yaw_velocity < -0.1  # Closing pincers / yawing inward
    collision_outside = local_collision_x > 0.0  # Collision is in front/outside of horn arc

    # Calculate if yaw is rotating toward or away from collision
    yaw_toward = (rotating_open and collision_outside) or (rotating_close and not collision_outside)
    yaw_away = (rotating_open and not collision_outside) or (rotating_close and collision_outside)

    if yaw_toward:
        target_yaw_damping = HORN_DAMPING_INTO * engagement_factor
    elif yaw_away:
        target_yaw_damping = HORN_DAMPING_AWAY * engagement_factor
    else:
        target_yaw_damping = HORN_DAMPING_NEUTRAL * engagement_factor

    # Smooth interpolation to avoid jerky feel
    new_pitch_damping = beetle.horn_pitch_damping + (target_pitch_damping - beetle.horn_pitch_damping) * HORN_DAMPING_SMOOTHING
    new_yaw_damping = beetle.horn_yaw_damping + (target_yaw_damping - beetle.horn_yaw_damping) * HORN_DAMPING_SMOOTHING

    return new_pitch_damping, new_yaw_damping


def beetle_collision(b1, b2, params):
    """Handle collision with voxel-perfect detection, pushing, and horn leverage"""
    # Detect if this is a ball collision (ball uses different, gentler physics)
    is_ball_collision = (b1.horn_type == "ball" or b2.horn_type == "ball")

    # Fast GPU-based collision check
    has_collision = check_collision_kernel(b1.x, b1.z, b1.y, b2.x, b2.z, b2.y, b1.color, b2.color)

    if has_collision:
        # GPU-ACCELERATED: Calculate occupied voxels on GPU (no CPU transfer!)
        calculate_occupied_voxels_kernel(b1.x, b1.z, b1.color,
                                        beetle1_occupied_x, beetle1_occupied_z, beetle1_occupied_count)
        calculate_occupied_voxels_kernel(b2.x, b2.z, b2.color,
                                        beetle2_occupied_x, beetle2_occupied_z, beetle2_occupied_count)

        # GPU-ACCELERATED: Calculate collision point on GPU (no CPU transfer!)
        calculate_collision_point_kernel(0)  # overlap_count not used in kernel

        # Read results from GPU (minimal data transfer - just 5 values!)
        collision_x = collision_point_x[None]
        collision_y = collision_point_y[None]
        collision_z = collision_point_z[None]
        contact_count = collision_contact_count[None]
        has_horn_tips = collision_has_horn_tips[None]

        # Collision detected! Calculate 3D collision geometry
        dx = b1.x - b2.x
        dz = b1.z - b2.z
        dist = math.sqrt(dx**2 + dz**2)

        if dist > 0.001:
            # Use GPU-calculated collision point or fallback to midpoint
            if contact_count > 0:
                collision_x /= contact_count
                collision_z /= contact_count
                collision_y /= contact_count
            else:
                collision_x = (b1.x + b2.x) / 2.0
                collision_z = (b1.z + b2.z) / 2.0
                collision_y = (b1.y + b2.y) / 2.0

            # Calculate 3D collision normal (from b2 to b1)
            # Include vertical component based on contact geometry
            dy = b1.y - b2.y
            dist_3d = math.sqrt(dx**2 + dy**2 + dz**2)

            if dist_3d > 0.001:
                instant_normal_x = dx / dist_3d
                instant_normal_y = dy / dist_3d
                instant_normal_z = dz / dist_3d
            else:
                # Fallback to horizontal normal
                instant_normal_x = dx / dist
                instant_normal_y = 0.0
                instant_normal_z = dz / dist

            # Apply exponential moving average to smooth collision normal (reduces jitter)
            # This prevents rapid oscillation when beetles are locked horn-to-horn
            b1.contact_normal_x += (instant_normal_x - b1.contact_normal_x) * COLLISION_NORMAL_SMOOTHING
            b1.contact_normal_z += (instant_normal_z - b1.contact_normal_z) * COLLISION_NORMAL_SMOOTHING
            b2.contact_normal_x += (-instant_normal_x - b2.contact_normal_x) * COLLISION_NORMAL_SMOOTHING
            b2.contact_normal_z += (-instant_normal_z - b2.contact_normal_z) * COLLISION_NORMAL_SMOOTHING

            # Use smoothed horizontal normal for separation forces (keeps vertical instant for horn leverage)
            normal_x = b1.contact_normal_x
            normal_y = instant_normal_y  # Keep instant vertical for responsive horn lift
            normal_z = b1.contact_normal_z

            # HORN LEVERAGE: Strong vertical lift when contact is high (horn collision)
            # Ball collisions now treated like beetle collisions for consistent physics
            center_y = (b1.y + b2.y) / 2.0
            contact_height_above_center = collision_y - center_y
            is_horn_contact = contact_height_above_center > 0.0  # Horn contact (at or above center height - includes low horn collisions)

            if is_horn_contact:
                # Mark beetles as in horn collision (blocks rotation during contact)
                # For ball collisions, only mark the beetle (not the ball)
                if is_ball_collision:
                    # Only block rotation on the beetle, not the ball
                    if b1.horn_type != "ball":
                        b1.in_horn_collision = True
                    if b2.horn_type != "ball":
                        b2.in_horn_collision = True
                else:
                    b1.in_horn_collision = True
                    b2.in_horn_collision = True

                # === PHASE 2: HORN ROTATION DAMPING ===
                # Calculate engagement factor (how much horn contact exists)
                normalized_contact_count = min(contact_count / HORN_ENGAGEMENT_CONTACT_THRESHOLD, 1.0)
                height_factor = min((contact_height_above_center - 2.0) / 8.0, 1.0)
                engagement_factor = (normalized_contact_count * (1.0 - HORN_ENGAGEMENT_HEIGHT_WEIGHT) +
                                    height_factor * HORN_ENGAGEMENT_HEIGHT_WEIGHT)

                # Boost engagement for tip collisions - tips should feel solid even with few voxels
                MIN_TIP_ENGAGEMENT = 0.65  # Minimum engagement when horn tips are involved
                if has_horn_tips == 1:
                    engagement_factor = max(engagement_factor, MIN_TIP_ENGAGEMENT)

                # Calculate separate pitch and yaw damping for beetles (not ball) based on their rotation directions
                if b1.horn_type != "ball":
                    b1.horn_pitch_damping, b1.horn_yaw_damping = calculate_horn_damping(
                        b1, collision_x, collision_y, collision_z, engagement_factor)
                if b2.horn_type != "ball":
                    b2.horn_pitch_damping, b2.horn_yaw_damping = calculate_horn_damping(
                        b2, collision_x, collision_y, collision_z, engagement_factor)

                # Skip lift/leverage combat physics for ball collisions (ball has its own physics below)
                # Only apply beetle-vs-beetle lift physics when no ball is involved
                if not is_ball_collision:
                    # Scale up vertical component based on height - logarithmic scaling for diminishing returns
                    raw_leverage = contact_height_above_center / 3.0
                    horn_leverage = min(math.log(raw_leverage + 1.0) * 2.0, 2.5)

                    # Add MASSIVE upward bias to the collision normal
                    normal_y += horn_leverage * 2.5  # 5x stronger than before!

                    # Re-normalize
                    norm_len = math.sqrt(normal_x**2 + normal_y**2 + normal_z**2)
                    if norm_len > 0.001:
                        normal_x /= norm_len
                        normal_y /= norm_len
                        normal_z /= norm_len

                    # SIMPLIFIED SYMMETRIC LIFTING - PURE VELOCITY (for debugging)
                    # Whoever is pressing their lift key (R/U) lifts the opponent
                    lift_impulse = horn_leverage * 30.0  # Upward kick

                    # Calculate where the collision is relative to each beetle's center
                    collision_above_b1 = collision_y - b1.y
                    collision_above_b2 = collision_y - b2.y

                    # PURE VELOCITY ADVANTAGE - who's actively pressing R or U?
                    # Positive velocity = tilting horn UP = you lift opponent
                    # Blue pressing R: b1.horn_pitch_velocity = +2.0
                    # Red pressing U: b2.horn_pitch_velocity = +2.0
                    lift_advantage = b1.horn_pitch_velocity - b2.horn_pitch_velocity

                    # DEBUG: Print collision info
                    # print(f"HORN COLLISION:")
                    # print(f"  Blue: vel={b1.horn_pitch_velocity:.2f}, y={b1.y:.1f}")
                    # print(f"  Red:  vel={b2.horn_pitch_velocity:.2f}, y={b2.y:.1f}")
                    # print(f"  Lift advantage: {lift_advantage:.2f} (>0 = Blue lifts Red, <0 = Red lifts Blue)")

                    # Very low threshold - almost any velocity difference triggers
                    ADVANTAGE_THRESHOLD = 0.5
                    LIFT_COOLDOWN_DURATION = 0.1  # Seconds between lift applications

                    # Calculate height penalty (reduces lift when beetles are already airborne)
                    NORMAL_HEIGHT = 2.0  # Height where lifts start weakening (lower = earlier penalty)
                    HEIGHT_PENALTY_FACTOR = 0.35  # How quickly lift weakens with height (higher = steeper)
                    avg_height = (b1.y + b2.y) / 2.0
                    height_penalty = 1.0
                    if avg_height > NORMAL_HEIGHT:
                        height_above_normal = avg_height - NORMAL_HEIGHT
                        height_penalty = 1.0 / (1.0 + height_above_normal * HEIGHT_PENALTY_FACTOR)

                    # Check if both beetles are off cooldown before applying lift forces
                    if b1.lift_cooldown <= 0.0 and b2.lift_cooldown <= 0.0:
                        # Cooldown expired - can apply lift force
                        if lift_advantage > ADVANTAGE_THRESHOLD:
                            # Blue has advantage - lifts red
                            # print(f"  -> BLUE lifts RED!")
                            lift_force = lift_impulse * 0.195 * height_penalty
                            b2.vy += lift_force  # Red gets lifted HIGHER
                            b1.vy -= lift_impulse * 0.03  # Blue pushes down (reaction)

                            # TORQUE: Apply rotation from off-center force
                            # Calculate lever arm from red's center to collision point
                            lever_x = collision_x - b2.x  # X offset (causes roll)
                            lever_z = collision_z - b2.z  # Z offset (causes pitch)

                            # Pitch torque: force in +Y at position +Z causes nose-up pitch
                            tumble_mult = params.get("TUMBLE_MULTIPLIER", 3.0)
                            pitch_torque = lever_z * lift_force * tumble_mult
                            b2.pitch_velocity += pitch_torque / b2.pitch_inertia

                            # Roll torque: force in +Y at position +X causes right-side-up roll
                            roll_torque = lever_x * lift_force * tumble_mult
                            b2.roll_velocity += roll_torque / b2.roll_inertia

                            # Set cooldown for both beetles
                            b1.lift_cooldown = LIFT_COOLDOWN_DURATION
                            b2.lift_cooldown = LIFT_COOLDOWN_DURATION

                        elif lift_advantage < -ADVANTAGE_THRESHOLD:
                            # Red has advantage - lifts blue
                            # print(f"  -> RED lifts BLUE!")
                            lift_force = lift_impulse * 0.195 * height_penalty
                            b1.vy += lift_force  # Blue gets lifted HIGHER
                            b2.vy -= lift_impulse * 0.03  # Red pushes down (reaction)

                            # TORQUE: Apply rotation from off-center force
                            lever_x = collision_x - b1.x  # X offset (causes roll)
                            lever_z = collision_z - b1.z  # Z offset (causes pitch)

                            # Pitch torque: force in +Y at position +Z causes nose-up pitch
                            tumble_mult = params.get("TUMBLE_MULTIPLIER", 3.0)
                            pitch_torque = lever_z * lift_force * tumble_mult
                            b1.pitch_velocity += pitch_torque / b1.pitch_inertia

                            # Roll torque
                            roll_torque = lever_x * lift_force * tumble_mult
                            b1.roll_velocity += roll_torque / b1.roll_inertia

                            # Set cooldown for both beetles
                            b1.lift_cooldown = LIFT_COOLDOWN_DURATION
                            b2.lift_cooldown = LIFT_COOLDOWN_DURATION

                        else:
                            # Evenly matched - both get pushed (with torque)
                            # print(f"  -> BOTH beetles pushed!")
                            push_force = lift_impulse * 0.06 * height_penalty
                            b1.vy += push_force
                            b2.vy += push_force

                            # Apply torque to both
                            lever_x1 = collision_x - b1.x
                            lever_z1 = collision_z - b1.z
                            b1.pitch_velocity += (-lever_z1 * push_force) / b1.pitch_inertia
                            b1.roll_velocity += (lever_x1 * push_force) / b1.roll_inertia

                            lever_x2 = collision_x - b2.x
                            lever_z2 = collision_z - b2.z
                            b2.pitch_velocity += (-lever_z2 * push_force) / b2.pitch_inertia
                            b2.roll_velocity += (lever_x2 * push_force) / b2.roll_inertia

                            # Set cooldown for both beetles
                            b1.lift_cooldown = LIFT_COOLDOWN_DURATION
                            b2.lift_cooldown = LIFT_COOLDOWN_DURATION
                    else:
                        # Cooldown active - skip lift force but still print debug info
                        pass  # print(f"  -> COOLDOWN ACTIVE (Blue: {b1.lift_cooldown:.3f}s, Red: {b2.lift_cooldown:.3f}s)")

                    # Add horizontal spin based on where horn hit BOTH beetles
                    # Calculate lever arms from beetle centers to collision point
                    dx_to_b1 = collision_x - b1.x
                    dz_to_b1 = collision_z - b1.z
                    dx_to_b2 = collision_x - b2.x
                    dz_to_b2 = collision_z - b2.z

                    # Calculate tangential velocity at collision point (perpendicular to normal)
                    # This captures rotation effects even when linear velocity is small
                    v1_at_collision_x = b1.vx + dz_to_b1 * b1.angular_velocity
                    v1_at_collision_z = b1.vz - dx_to_b1 * b1.angular_velocity
                    v2_at_collision_x = b2.vx + dz_to_b2 * b2.angular_velocity
                    v2_at_collision_z = b2.vz - dx_to_b2 * b2.angular_velocity

                    # Relative velocity at collision point
                    rel_vx = v1_at_collision_x - v2_at_collision_x
                    rel_vz = v1_at_collision_z - v2_at_collision_z

                    # Calculate tangential component (perpendicular to normal) for spin torque
                    # Project onto tangent direction (perpendicular to normal)
                    tangent_x = -normal_z
                    tangent_z = normal_x
                    rel_vel_tangent = rel_vx * tangent_x + rel_vz * tangent_z

                    # Hybrid approach: base torque from geometry + velocity modulation
                    # This ensures torque always exists but scales with collision dynamics
                    base_torque_b1 = dx_to_b1 * normal_z - dz_to_b1 * normal_x
                    base_torque_b2 = dx_to_b2 * normal_z - dz_to_b2 * normal_x

                    # Modulate by tangential velocity (captures rotation into each other)
                    # Add baseline factor to ensure minimum torque even when stationary
                    velocity_factor = 1.0 + abs(rel_vel_tangent) * 0.5

                    torque_b1 = base_torque_b1 * velocity_factor
                    torque_b2 = base_torque_b2 * velocity_factor

                    # Apply angular impulses with horn leverage
                    angular_impulse_b1 = (torque_b1 / b1.moment_of_inertia) * horn_leverage * 2.0
                    angular_impulse_b2 = (torque_b2 / b2.moment_of_inertia) * horn_leverage * 2.0
                    b1.angular_velocity += angular_impulse_b1
                    b2.angular_velocity -= angular_impulse_b2

            # STRONG separation to prevent stuck collisions (now 3D!)
            separation_force = params["SEPARATION_FORCE"]

            b1.x += normal_x * separation_force
            b1.z += normal_z * separation_force
            b2.x -= normal_x * separation_force
            b2.z -= normal_z * separation_force

            # VERTICAL SEPARATION - only when both beetles are airborne
            # This prevents floor voxel destruction and maintains symmetry
            if not b1.on_ground and not b2.on_ground:
                b1.y += normal_y * separation_force
                b2.y -= normal_y * separation_force
                # Update prev positions to prevent interpolation choppiness from position corrections
                # (Ball especially needs this for smooth vertical motion during collisions)
                if is_ball_collision:
                    if b1.horn_type == "ball":
                        b1.prev_y = b1.y
                    if b2.horn_type == "ball":
                        b2.prev_y = b2.y

            # Safety clamp to prevent going below floor voxel layer
            # (Main floor collision handles proper positioning above floor)
            MIN_Y = 0.5  # Minimum Y to prevent center going below floor at y=0
            if b1.y < MIN_Y:
                b1.y = MIN_Y
                b1.vy = 0.0
            if b2.y < MIN_Y:
                b2.y = MIN_Y
                b2.vy = 0.0

            # Calculate 3D relative velocity
            rel_vx = b1.vx - b2.vx
            rel_vy = b1.vy - b2.vy
            rel_vz = b1.vz - b2.vz
            vel_along_normal = rel_vx * normal_x + rel_vy * normal_y + rel_vz * normal_z

            # For ball collisions: allow redirection even when ball is moving away (airborne hits)
            # For beetle collisions: only apply impulse if moving toward each other
            apply_impulse = vel_along_normal < 0  # Default: only when approaching
            if is_ball_collision and vel_along_normal >= 0:
                # Ball is moving away - still apply a redirection impulse based on beetle velocity
                # This allows "steering" the ball mid-air by hitting it from a new angle
                apply_impulse = True
                # Use beetle's velocity projected onto collision normal as the impulse basis
                # This makes the ball respond to where the beetle is pushing, not relative motion
                if b1.horn_type == "ball":
                    beetle = b2
                    beetle_vel_along_normal = -(beetle.vx * normal_x + beetle.vz * normal_z)
                else:
                    beetle = b1
                    beetle_vel_along_normal = beetle.vx * normal_x + beetle.vz * normal_z
                # Override vel_along_normal with beetle's push direction (negative = toward ball)
                if abs(beetle_vel_along_normal) > 0.5:  # Beetle must be actively moving
                    vel_along_normal = -abs(beetle_vel_along_normal) * 0.8  # Treat as approaching

            if apply_impulse:
                # Impulse magnitude (use ball-specific restitution for ball collisions)
                if is_ball_collision:
                    # Ball collision: use ball restitution and momentum transfer
                    restitution = params["BALL_RESTITUTION"]
                    impulse = -(1 + restitution) * vel_along_normal * params["BALL_MOMENTUM_TRANSFER"]
                else:
                    # Beetle collision: use beetle combat physics
                    impulse = -(1 + params["RESTITUTION"]) * vel_along_normal * params["IMPULSE_MULTIPLIER"]

                # Apply 3D linear impulse
                impulse_x = impulse * normal_x
                impulse_z = impulse * normal_z

                # For horn contact, lifting logic handles vertical forces explicitly
                # Only apply vertical impulse for non-horn collisions
                if is_horn_contact:
                    impulse_y = 0.0  # Disable vertical impulse - lifting logic handles it
                else:
                    impulse_y = impulse * normal_y

                b1.vx += impulse_x
                b1.vy += impulse_y
                b1.vz += impulse_z
                b2.vx -= impulse_x
                b2.vy -= impulse_y
                b2.vz -= impulse_z

                # Calculate and apply torque (angular impulse)
                # Torque = r × F (cross product in 2D: rx*Fz - rz*Fx)

                # For beetle 1: collision point relative to its center
                r1_x = collision_x - b1.x
                r1_z = collision_z - b1.z
                torque1 = r1_x * impulse_z - r1_z * impulse_x
                angular_impulse1 = (torque1 / b1.moment_of_inertia) * params["TORQUE_MULTIPLIER"]
                b1.angular_velocity += angular_impulse1

                # For beetle 2: collision point relative to its center
                r2_x = collision_x - b2.x
                r2_z = collision_z - b2.z
                torque2 = r2_x * (-impulse_z) - r2_z * (-impulse_x)
                angular_impulse2 = (torque2 / b2.moment_of_inertia) * params["TORQUE_MULTIPLIER"]
                b2.angular_velocity += angular_impulse2

                # === BALL PHYSICS: TORQUE/LIFT/TIP (realistic contact-based forces) ===
                # Apply special physics when ball is involved (hit from side = spin, from below = lift, from above = tip)
                if is_ball_collision:
                    # Identify which entity is the ball
                    if b1.horn_type == "ball":
                        ball = b1
                        beetle = b2
                        ball_is_b1 = True
                    else:
                        ball = b2
                        beetle = b1
                        ball_is_b1 = False

                    # Get push and spin multipliers for lighter/heavier ball feel
                    push_mult = params["BALL_PUSH_MULTIPLIER"]
                    spin_mult = params["BALL_SPIN_MULTIPLIER"]

                    # Apply extra push force to ball based on push multiplier
                    # This makes the ball feel lighter and easier to push
                    if ball_is_b1:
                        ball.vx += impulse_x * (push_mult - 1.0)  # Extra push on top of normal impulse
                        ball.vz += impulse_z * (push_mult - 1.0)
                    else:
                        ball.vx -= impulse_x * (push_mult - 1.0)
                        ball.vz -= impulse_z * (push_mult - 1.0)

                    # Calculate contact offset from ball center (Y-axis for lift/tip)
                    contact_offset_y = collision_y - ball.y

                    # HORN SCOOP LIFT: When beetle tilts horn UP while touching bottom of ball, lift it!
                    # This mimics beetle-to-beetle combat where horn pitch velocity determines lift
                    if contact_offset_y < ball.radius * 0.3:  # Contact at or below ball center (bottom 80%)
                        # Check if beetle is actively tilting horn upward (positive horn_pitch_velocity)
                        if beetle.horn_pitch_velocity > 0.5:  # Beetle is scooping up
                            # Scale lift force by horn velocity (more aggressive scoop = more lift)
                            scoop_strength = min(beetle.horn_pitch_velocity / 2.0, 1.5)  # Cap at 1.5x
                            horn_scoop_lift = params["BALL_LIFT_STRENGTH"] * 2.0 * scoop_strength * push_mult
                            ball.vy += horn_scoop_lift

                    # PASSIVE LIFT: Hit from below (contact point is below ball center)
                    # Base upward force when beetle pushes ball from below
                    if contact_offset_y < -ball.radius * 0.35:  # Bottom 15% of ball
                        lift_force = params["BALL_PASSIVE_LIFT_STRENGTH"] * push_mult
                        ball.vy += lift_force

                    # TIP: Hit from above (contact point is above ball center)
                    # When beetle hits ball from above, push it down
                    elif contact_offset_y > ball.radius * 0.2:  # Top 20% of ball
                        tip_force = params["BALL_TIP_STRENGTH"]
                        ball.vy -= tip_force

                    # TORQUE: Hits create spin on all 3 axes based on contact point
                    # Calculate offset from ball center (contact_offset_y already calculated above)
                    contact_offset_x = collision_x - ball.x
                    contact_offset_z = collision_z - ball.z

                    # Apply rotational spin based on impact location
                    # Torque = r × F (lever arm cross force)
                    # Use spin_mult to make ball spin easier
                    torque_strength = params["BALL_TORQUE_STRENGTH"] * spin_mult / ball.moment_of_inertia

                    # YAW spin (around Y axis) - from horizontal offset
                    ball_yaw_torque = contact_offset_x * impulse_z - contact_offset_z * impulse_x
                    ball_yaw_impulse = ball_yaw_torque * torque_strength

                    # PITCH spin (around Z axis) - from vertical offset with forward/back force
                    # Hit from above + pushed forward = topspin (pitch forward)
                    # Hit from below + pushed forward = backspin (pitch backward)
                    ball_pitch_torque = contact_offset_y * impulse_x
                    ball_pitch_impulse = ball_pitch_torque * torque_strength * 0.2

                    # ROLL spin (around X axis) - from vertical offset with side force
                    ball_roll_torque = contact_offset_y * impulse_z
                    ball_roll_impulse = ball_roll_torque * torque_strength * 0.2

                    # Apply all torques (add to existing angular velocity for realistic spin accumulation)
                    if ball_is_b1:
                        ball.angular_velocity += ball_yaw_impulse
                        ball.pitch_velocity += ball_pitch_impulse
                        ball.roll_velocity += ball_roll_impulse
                    else:
                        ball.angular_velocity -= ball_yaw_impulse  # Reverse sign for b2
                        ball.pitch_velocity -= ball_pitch_impulse
                        ball.roll_velocity -= ball_roll_impulse

            # === PHASE 3: BODY ROTATION DAMPING ===
            # Track collision-induced spin direction and set damping
            # This prevents immediate rotation into collision direction, preventing horn clipping

            # For beetle 1: if angular_velocity is significant, apply damping
            if abs(b1.angular_velocity) > 0.3:  # Lower threshold for more responsiveness
                # Store spin direction: 1 = clockwise (positive), -1 = counterclockwise (negative)
                b1.collision_spin_direction = 1 if b1.angular_velocity > 0 else -1
                # Apply full damping strength on any significant collision
                b1.body_rotation_damping = BODY_ROTATION_DAMPING_STRENGTH

            # For beetle 2: same logic
            if abs(b2.angular_velocity) > 0.3:
                b2.collision_spin_direction = 1 if b2.angular_velocity > 0 else -1
                # Apply full damping strength on any significant collision
                b2.body_rotation_damping = BODY_ROTATION_DAMPING_STRENGTH
    else:
        # No collision - reset smoothed collision normals
        b1.contact_normal_x = 0.0
        b1.contact_normal_z = 0.0
        b2.contact_normal_x = 0.0
        b2.contact_normal_z = 0.0

def normalize_angle(angle):
    """Normalize angle to [0, 2π)"""
    while angle < 0:
        angle += TWO_PI
    while angle >= TWO_PI:
        angle -= TWO_PI
    return angle

def shortest_rotation(current, target):
    """Find shortest rotation from current to target"""
    current = normalize_angle(current)
    target = normalize_angle(target)
    diff = target - current
    if diff > math.pi:
        diff -= TWO_PI
    elif diff < -math.pi:
        diff += TWO_PI
    return diff

# Window
window = ti.ui.Window("Beetle Physics", (1920, 1080), vsync=True)
canvas = window.get_canvas()
scene = window.get_scene()

camera = renderer.Camera()
camera.pos_x = 0.0
camera.pos_y = 60.0
camera.pos_z = 0.0
camera.pitch = -70.0
camera.yaw = 180.0  # Start facing correct direction for CAMERA_FLIP_VIEW = True

# Auto-follow camera toggle
auto_follow_enabled = True  # Start with auto-follow camera enabled (press C to toggle)
# Camera always uses opposite side view (beetles in foreground, edge in background)
camera_edge_angle = None  # Previous edge angle for smooth transitions (None = not yet initialized)
spotlight_strength = 0.633  # Spotlight intensity (adjustable via GUI slider)
spotlight_height = 36.0  # Spotlight height above beetles (lower = smaller/focused, higher = bigger/softer)
base_light_brightness = 1.313  # Brightness multiplier for all non-spotlight lights (adjustable via GUI slider)
front_light_strength = 0.35  # Front camera light intensity (adjustable via GUI slider)

# Dynamic lighting system
dynamic_lighting_enabled = False  # Camera-relative lighting for cinematic effect (press L to toggle)

# Initialize gradient background for forest atmosphere
renderer.init_gradient_background()

# OPTIMIZATION: Pre-compute metallic shimmer lookup table (8-12% render speedup)
renderer.init_shimmer_lut()

print("\n=== BEETLE PHYSICS ===")
print("BLUE BEETLE (TFGH + RY) - Tank Controls:")
print("  T - Move Forward")
print("  G - Move Backward")
print("  F - Rotate Left")
print("  H - Rotate Right")
print("  R - Tilt Horn Up (+20°)")
print("  Y - Tilt Horn Down (-20°)")
print("")
print("RED BEETLE (IJKL + UO) - Tank Controls:")
print("  I - Move Forward")
print("  K - Move Backward")
print("  J - Rotate Left")
print("  L - Rotate Right")
print("  U - Tilt Horn Up (+20°)")
print("  O - Tilt Horn Down (-20°)")
print("")
print("Push beetles together!")
print("Camera: WASD/Mouse/Q/E")
print("="*40 + "\n")

# Physics parameters dictionary (mutable for sliders)
physics_params = {
    "TORQUE_MULTIPLIER": TORQUE_MULTIPLIER,
    "IMPULSE_MULTIPLIER": IMPULSE_MULTIPLIER,
    "RESTITUTION": RESTITUTION,
    "MOMENT_OF_INERTIA_FACTOR": MOMENT_OF_INERTIA_FACTOR,
    "GRAVITY": 31.0,  # Adjustable gravity
    "SEPARATION_FORCE": 0.4,  # Gradual position separation on collision

    # Airborne tumbling physics parameters
    "AIRBORNE_DAMPING": 0.95,  # Angular damping when airborne (0.95 = 5% loss per frame, more tumbling)
    "AIRBORNE_TILT_SPEED": 900.0,  # Max pitch/roll speed when airborne
    "GROUND_TILT_ANGLE": 300.0,  # Max tilt angle in degrees when on ground
    "TUMBLE_MULTIPLIER": 5.0,  # Multiplier for pitch/roll torque when launching (creates dramatic flips)
    "RESTORING_STRENGTH": 35.0,  # How fast beetles level out when settled on ground
    "WEAK_RESTORING": 25.0,  # How fast beetles level out while bouncing

    # Ball physics parameters (tunable via sliders for smooth rolling/bouncing)
    "BALL_SEPARATION_FORCE": BALL_SEPARATION_FORCE,  # How hard ball pushes away from beetles
    "BALL_MOMENTUM_TRANSFER": BALL_MOMENTUM_TRANSFER,  # How much beetle velocity transfers to ball
    "BALL_RESTITUTION": BALL_RESTITUTION,  # Bounciness coefficient
    "BALL_MASS_RATIO": BALL_MASS_RATIO,  # Ball weight vs beetle
    "BALL_ROLLING_FRICTION": BALL_ROLLING_FRICTION,  # Horizontal slowdown
    "BALL_GROUND_BOUNCE": BALL_GROUND_BOUNCE,  # Floor bounce coefficient
    "BALL_PUSH_MULTIPLIER": BALL_PUSH_MULTIPLIER,  # How easily beetles can push the ball
    "BALL_SPIN_MULTIPLIER": BALL_SPIN_MULTIPLIER,  # How easily ball spins when hit
    "BALL_ANGULAR_FRICTION": BALL_ANGULAR_FRICTION,  # How quickly ball spin slows

    # Ball torque/lift/tip parameters (realistic contact physics)
    "BALL_LIFT_STRENGTH": BALL_LIFT_STRENGTH,  # How much ball lifts when hit from below (scoop)
    "BALL_PASSIVE_LIFT_STRENGTH": BALL_PASSIVE_LIFT_STRENGTH,  # Passive lift when touching bottom of ball
    "BALL_TIP_STRENGTH": BALL_TIP_STRENGTH,  # How much ball tips down when hit from above
    "BALL_TORQUE_STRENGTH": BALL_TORQUE_STRENGTH,  # How much ball spins from side hits
    "BALL_GRAVITY_MULTIPLIER": BALL_GRAVITY_MULTIPLIER,  # Extra gravity for ball (1.0-5.0)

    # Bowl perimeter physics (ball mode)
    "BOWL_SLIDE_STRENGTH": BOWL_SLIDE_STRENGTH,  # Inward push force on slippery surface

    # Camera parameters (for auto-follow mode)
    "CAMERA_PITCH": -30.85,  # Camera pitch angle (-90=top-down, -45=side view, -30.85=default)
    "CAMERA_BASE_HEIGHT": 74.7,  # Base camera height in voxels (before dynamic adjustment)
    "CAMERA_DISTANCE": 76.35  # Camera horizontal distance from midpoint
}

last_time = time.time()
accumulator = 0.0  # Time accumulator for fixed timestep physics
shadows_were_placed = False  # Track if shadows existed last frame (for cleanup)

# Per-beetle horn types (independent type selection for blue and red beetles)
blue_horn_type = "rhino"  # Blue beetle horn type: "rhino", "stag", "hercules", or "scorpion"
red_horn_type = "rhino"   # Red beetle horn type: "rhino", "stag", "hercules", or "scorpion"
horn_type = blue_horn_type  # Temporary compatibility alias (points to blue beetle)

physics_frame = 0  # Frame counter for periodic cleanup

# Per-beetle animation state for scorpion tail
blue_previous_stinger_curvature = 0.0  # Track blue beetle stinger curvature
blue_previous_tail_rotation = 0.0      # Track blue beetle tail rotation
red_previous_stinger_curvature = 0.0   # Track red beetle stinger curvature
red_previous_tail_rotation = 0.0       # Track red beetle tail rotation

# Temporary compatibility aliases (point to blue beetle)
previous_stinger_curvature = blue_previous_stinger_curvature
previous_tail_rotation = blue_previous_tail_rotation

# Ball state now handled by beetle_ball Beetle object (created at line ~318)

# Warm up kernels to avoid first-time JIT compilation stutter
# Call with dummy data during initialization to pre-compile GPU kernels
print("Warming up kernels...")

# Collision detection kernels (these cause the first-collision stutter)
check_collision_kernel(0.0, 0.0, 0.0, 100.0, 100.0, 0.0, simulation.BEETLE_BLUE, simulation.BEETLE_RED)
calculate_occupied_voxels_kernel(0.0, 0.0, simulation.BEETLE_BLUE,
                                 beetle1_occupied_x, beetle1_occupied_z, beetle1_occupied_count)
calculate_occupied_voxels_kernel(0.0, 0.0, simulation.BEETLE_RED,
                                 beetle2_occupied_x, beetle2_occupied_z, beetle2_occupied_count)
calculate_collision_point_kernel(0)

# Death explosion kernel
spawn_death_explosion_batch(0.0, -100.0, 0.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0, 1, 1)

# Debris/particle update kernel (triggered when particles exist)
update_debris_particles(0.016)

# Edge tipping kernel (triggered when beetles near arena edge)
calculate_edge_tipping_kernel(0.0, 0.0, simulation.BEETLE_BLUE, 0.016, 1.0, 1.0)
calculate_edge_tipping_kernel(0.0, 0.0, simulation.BEETLE_RED, 0.016, 1.0, 1.0)

# Clear beetles kernels (used during reset and rendering)
clear_beetles()
clear_beetles_bounded(0.0, 0.0, 0.0, 10.0, 10.0, 10.0)

# Shadow kernels (triggered when beetles are airborne)
clear_shadow_layer(int(RENDER_Y_OFFSET), 0)
place_shadow_kernel(0.0, 0.0, 3.0, int(RENDER_Y_OFFSET))
clear_shadow_layer(int(RENDER_Y_OFFSET), 0)  # Clear the warm-up shadow so it doesn't show on load

# Victory confetti kernel (triggered on match win)
spawn_victory_confetti(0.0, 0.0, -100.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 1)

# Sync GPU to ensure all warm-up compilations complete
ti.sync()

print("All kernels warmed up (pre-compiled)")

# Build initial floor height cache (for fast collision lookups)
build_floor_height_cache()

while window.running:
    # === START FRAME TIMING ===
    perf_monitor.start('frame_total')

    current_time = time.time()
    frame_dt = current_time - last_time
    last_time = current_time

    # Calculate actual FPS before capping (for accurate display)
    actual_fps = 1.0 / frame_dt if frame_dt > 0 else 0

    frame_dt = min(frame_dt, 0.02)  # Cap at 20ms to prevent accumulator spikes

    # Add frame time to accumulator
    accumulator += frame_dt

    # Cap accumulator to prevent spiral during geometry rebuilds (max 3 physics steps per frame)
    MAX_ACCUMULATOR = PHYSICS_TIMESTEP * 3  # ~50ms worth of simulation
    accumulator = min(accumulator, MAX_ACCUMULATOR)

    # === CAMERA TIMING ===
    perf_monitor.start('camera')

    # Camera (runs every frame at frame rate)
    # Toggle auto-follow camera with C key
    if window.is_pressed('c'):
        if not hasattr(window, 'c_key_was_pressed') or not window.c_key_was_pressed:
            auto_follow_enabled = not auto_follow_enabled
            print(f"Auto-follow camera: {'ENABLED' if auto_follow_enabled else 'DISABLED'}")
        window.c_key_was_pressed = True
    else:
        window.c_key_was_pressed = False

    if auto_follow_enabled:
        # Edge-aware auto-follow camera: track beetles AND nearest arena edge
        # Detect if beetles have fallen off the platform
        FALL_HEIGHT_THRESHOLD = -10.0  # Y position below which beetle is considered fallen

        # Use extended radius when ball mode is active (ice bowl extends 12 voxels beyond arena)
        BOWL_WIDTH = 12.0
        effective_radius = ARENA_RADIUS + BOWL_WIDTH if beetle_ball.active else ARENA_RADIUS

        # Check both vertical fall AND horizontal arena boundary (prevents camera spazzing at edges)
        blue_dist_from_center = math.sqrt(beetle_blue.x**2 + beetle_blue.z**2)
        red_dist_from_center = math.sqrt(beetle_red.x**2 + beetle_red.z**2)

        blue_on_platform = (beetle_blue.y > FALL_HEIGHT_THRESHOLD and
                            blue_dist_from_center < effective_radius)
        red_on_platform = (beetle_red.y > FALL_HEIGHT_THRESHOLD and
                           red_dist_from_center < effective_radius)

        # Calculate midpoint - only include beetles still on platform
        if blue_on_platform and red_on_platform:
            # Both beetles on platform: normal midpoint
            mid_x = (beetle_blue.x + beetle_red.x) / 2.0
            mid_z = (beetle_blue.z + beetle_red.z) / 2.0
        elif blue_on_platform:
            # Only blue on platform: focus on blue
            mid_x = beetle_blue.x
            mid_z = beetle_blue.z
        elif red_on_platform:
            # Only red on platform: focus on red
            mid_x = beetle_red.x
            mid_z = beetle_red.z
        else:
            # Both fallen: maintain last valid midpoint (use 0,0 as safe fallback)
            mid_x = 0.0
            mid_z = 0.0

        # Calculate separation distance for dynamic height
        dx = beetle_red.x - beetle_blue.x
        dz = beetle_red.z - beetle_blue.z
        separation = math.sqrt(dx*dx + dz*dz)

        # Find nearest point on circular arena edge
        # Calculate distance and angle from origin to midpoint
        dist_from_origin = math.sqrt(mid_x*mid_x + mid_z*mid_z)

        # Handle center case (beetles at origin)
        if dist_from_origin < 0.1:
            # Default to north edge when at center
            angle_to_midpoint = math.pi / 2.0  # 90 degrees = north
        else:
            angle_to_midpoint = math.atan2(mid_z, mid_x)

        # Smooth angle transitions
        if camera_edge_angle is None:
            # First frame: initialize with current angle
            camera_edge_angle = angle_to_midpoint
        else:
            # Calculate angle difference (handle wrapping)
            angle_diff = angle_to_midpoint - camera_edge_angle
            # Normalize to -pi to +pi range
            while angle_diff > math.pi:
                angle_diff -= 2.0 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2.0 * math.pi

            # Always track beetles with smooth transition
            camera_edge_angle += angle_diff * 0.3  # 30% blend factor

        # Nearest edge point is along smoothed angle at effective radius
        nearest_edge_x = math.cos(camera_edge_angle) * effective_radius
        nearest_edge_z = math.sin(camera_edge_angle) * effective_radius

        # Calculate direction vector FROM midpoint TO edge
        edge_dx = nearest_edge_x - mid_x
        edge_dz = nearest_edge_z - mid_z
        edge_dist = math.sqrt(edge_dx*edge_dx + edge_dz*edge_dz)

        # Normalize edge direction
        if edge_dist > 0.1:
            edge_dx_norm = edge_dx / edge_dist
            edge_dz_norm = edge_dz / edge_dist
        else:
            # Beetles very close to edge, use angle to origin
            edge_dx_norm = math.cos(angle_to_midpoint)
            edge_dz_norm = math.sin(angle_to_midpoint)

        # Calculate edge proximity (0.0 = center, 1.0 = at edge)
        edge_proximity = dist_from_origin / effective_radius

        # Camera distance: reduce when beetles near edge (so edge is visible)
        base_camera_distance = physics_params["CAMERA_DISTANCE"]
        edge_factor = 1.0 - (edge_proximity * 0.4)  # Reduce by up to 40% at edge
        adjusted_camera_distance = base_camera_distance * edge_factor

        # Position camera on SAME SIDE as edge (beetles in foreground, edge in background)
        target_x = mid_x + (edge_dx_norm * adjusted_camera_distance)
        target_z = mid_z + (edge_dz_norm * adjusted_camera_distance)

        # Fixed camera height (dynamic scaling disabled)
        base_height = physics_params["CAMERA_BASE_HEIGHT"]
        target_y = base_height
        target_y = max(20.0, min(150.0, target_y))  # Clamp height

        # Smooth interpolation (lerp) to avoid jarring camera movement
        lerp_factor = 0.1 * frame_dt * 60.0  # Scale by frame time
        lerp_factor = min(1.0, lerp_factor)  # Clamp to 1.0 max

        camera.pos_x += (target_x - camera.pos_x) * lerp_factor
        camera.pos_y += (target_y - camera.pos_y) * lerp_factor
        camera.pos_z += (target_z - camera.pos_z) * lerp_factor

        # Maintain appropriate pitch angle (from slider)
        target_pitch = physics_params["CAMERA_PITCH"]
        camera.pitch += (target_pitch - camera.pitch) * lerp_factor

        # Rotate camera to face TOWARD beetles (edge in background)
        base_yaw = math.degrees(math.atan2(edge_dx_norm, edge_dz_norm))
        target_yaw = (base_yaw + 180.0) % 360.0

        # Smooth yaw transition (handle angle wrapping)
        yaw_diff = target_yaw - camera.yaw
        if yaw_diff > 180.0:
            yaw_diff -= 360.0
        elif yaw_diff < -180.0:
            yaw_diff += 360.0
        camera.yaw += yaw_diff * lerp_factor
        camera.yaw = camera.yaw % 360.0  # Normalize to 0-360
    else:
        # Use manual camera controls
        renderer.handle_camera_controls(camera, window, frame_dt)
        renderer.handle_mouse_look(camera, window)

    perf_monitor.stop('camera')

    # ===== FIXED TIMESTEP PHYSICS LOOP =====
    # Run physics in fixed increments (may run 0+ times per frame)
    perf_monitor.start('physics')
    physics_iterations_this_frame = 0

    # Reset physics timing for this frame
    for key in _physics_timing:
        _physics_timing[key] = 0.0

    while accumulator >= PHYSICS_TIMESTEP:
        # Save previous state for interpolation
        beetle_blue.save_previous_state()
        beetle_red.save_previous_state()
        if beetle_ball.active:
            beetle_ball.save_previous_state()

        # === INPUT/CONTROLS TIMING START ===
        _t_input_start = time.perf_counter()

        # === BLUE BEETLE CONTROLS (TFGH) - TANK STYLE ===
        if beetle_blue.active and not beetle_blue.is_falling:
            # Rotation controls (F/H) - BLOCKED during horn collision
            # 30% faster rotation when spinning in place (not moving forward/backward)
            if not beetle_blue.in_horn_collision:
                # Check if rotating without moving (skill-based faster turning)
                is_moving = window.is_pressed('t') or window.is_pressed('g')
                rotation_multiplier = 1.0 if is_moving else 1.3  # 30% faster when stationary

                if window.is_pressed('f'):
                    beetle_blue.rotation -= ROTATION_SPEED * rotation_multiplier * PHYSICS_TIMESTEP
                if window.is_pressed('h'):
                    beetle_blue.rotation += ROTATION_SPEED * rotation_multiplier * PHYSICS_TIMESTEP

            # Movement controls (T/G) - move in facing direction
            if window.is_pressed('t'):
                # Move forward in facing direction
                move_x = math.cos(beetle_blue.rotation)
                move_z = math.sin(beetle_blue.rotation)
                beetle_blue.apply_force(move_x * MOVE_FORCE, move_z * MOVE_FORCE, PHYSICS_TIMESTEP)
            if window.is_pressed('g'):
                # Move backward in facing direction
                move_x = -math.cos(beetle_blue.rotation)
                move_z = -math.sin(beetle_blue.rotation)
                beetle_blue.apply_force(move_x * BACKWARD_MOVE_FORCE, move_z * BACKWARD_MOVE_FORCE, PHYSICS_TIMESTEP)

            # Horn controls - OPTIMIZED for combined pitch+yaw movements
            # Calculate proposed pitch and yaw changes
            pitch_pressed = window.is_pressed('r') or window.is_pressed('y')
            yaw_pressed = window.is_pressed('v') or window.is_pressed('b')

            new_pitch = beetle_blue.horn_pitch
            new_yaw = beetle_blue.horn_yaw
            pitch_speed = 0.0
            yaw_speed = 0.0

            # Calculate new pitch if pitch keys pressed
            # OPTIMIZATION: Use lookup table instead of string comparisons
            max_pitch_limit, min_pitch_limit = HORN_PITCH_LIMITS[beetle_blue.horn_type_id]

            if window.is_pressed('r'):
                effective_speed = HORN_TILT_SPEED * (1.0 - beetle_blue.horn_pitch_damping)
                new_pitch = beetle_blue.horn_pitch + effective_speed * PHYSICS_TIMESTEP
                new_pitch = min(max_pitch_limit, new_pitch)
                pitch_speed = effective_speed
            elif window.is_pressed('y'):
                effective_speed = HORN_TILT_SPEED * (1.0 - beetle_blue.horn_pitch_damping)
                new_pitch = beetle_blue.horn_pitch - effective_speed * PHYSICS_TIMESTEP
                new_pitch = max(min_pitch_limit, new_pitch)
                pitch_speed = -effective_speed

            # Calculate new yaw if yaw keys pressed
            # For scorpion type, V/B control tail rotation angle instead of horn yaw
            # OPTIMIZATION: Use horn_type_id == 3 instead of string comparison
            if beetle_blue.horn_type_id == 3:  # scorpion
                # SCORPION: V/B control blue beetle's tail rotation
                TAIL_ROTATION_SPEED = 30.0  # Degrees per second
                if window.is_pressed('v'):
                    # V = Rotate tail up
                    beetle_blue.tail_rotation_angle += TAIL_ROTATION_SPEED * PHYSICS_TIMESTEP
                    beetle_blue.tail_rotation_angle = min(25.0, beetle_blue.tail_rotation_angle)
                elif window.is_pressed('b'):
                    # B = Rotate tail down
                    beetle_blue.tail_rotation_angle -= TAIL_ROTATION_SPEED * PHYSICS_TIMESTEP
                    beetle_blue.tail_rotation_angle = max(-25.0, beetle_blue.tail_rotation_angle)
                # Don't set yaw_pressed for scorpion (skip horn collision checks)
                yaw_pressed = False
            else:
                # OTHER TYPES: V/B control horn yaw (claws)
                # OPTIMIZATION: Use lookup table instead of string comparisons
                max_yaw_limit, min_yaw_limit = HORN_YAW_LIMITS[beetle_blue.horn_type_id]

                if window.is_pressed('v'):
                    # V key DECREASES yaw = CLOSES pincers (toward min_yaw_limit)
                    effective_speed = HORN_YAW_SPEED * (1.0 - beetle_blue.horn_yaw_damping)

                    # YAW LIFT: Apply lift to opponent when yawing during collision (all beetle types)
                    if beetle_red.active:
                        has_real_collision = check_collision_kernel(
                            beetle_blue.x, beetle_blue.z, beetle_blue.y,
                            beetle_red.x, beetle_red.z, beetle_red.y,
                            beetle_blue.color, beetle_red.color
                        )
                        if has_real_collision:
                            # Apply push force to opponent (forward + lift)
                            forward_x = math.cos(beetle_blue.rotation)
                            forward_z = math.sin(beetle_blue.rotation)
                            push_force = 40.0 * PHYSICS_TIMESTEP
                            beetle_red.vx += forward_x * push_force
                            beetle_red.vz += forward_z * push_force
                            beetle_red.vy += 25.0 * PHYSICS_TIMESTEP  # Lift up
                            beetle_red.pitch -= 0.02  # Direct pitch tilt (front/grabbed area up)

                    new_yaw = beetle_blue.horn_yaw - effective_speed * PHYSICS_TIMESTEP
                    new_yaw = max(min_yaw_limit, new_yaw)
                    yaw_speed = -effective_speed
                elif window.is_pressed('b'):
                    # B key INCREASES yaw = OPENS pincers (toward max_yaw_limit)
                    effective_speed = HORN_YAW_SPEED * (1.0 - beetle_blue.horn_yaw_damping)

                    # YAW LIFT: Apply lift to opponent when yawing during collision (all beetle types)
                    if beetle_red.active:
                        has_real_collision = check_collision_kernel(
                            beetle_blue.x, beetle_blue.z, beetle_blue.y,
                            beetle_red.x, beetle_red.z, beetle_red.y,
                            beetle_blue.color, beetle_red.color
                        )
                        if has_real_collision:
                            # Apply push force to opponent (forward + lift)
                            forward_x = math.cos(beetle_blue.rotation)
                            forward_z = math.sin(beetle_blue.rotation)
                            push_force = 40.0 * PHYSICS_TIMESTEP
                            beetle_red.vx += forward_x * push_force
                            beetle_red.vz += forward_z * push_force
                            beetle_red.vy += 25.0 * PHYSICS_TIMESTEP  # Lift up
                            beetle_red.pitch -= 0.02  # Direct pitch tilt (front/grabbed area up)

                    new_yaw = beetle_blue.horn_yaw + effective_speed * PHYSICS_TIMESTEP
                    new_yaw = min(max_yaw_limit, new_yaw)
                    yaw_speed = effective_speed

            # Predictive collision check (optimized for combined movements + dual-pincer tracking)
            if (pitch_pressed or yaw_pressed) and beetle_red.active:
                # Calculate minimum distance between horn tips (handles both stag and rhino)
                distance = calculate_min_horn_distance(
                    beetle_blue, beetle_red,
                    new_pitch, new_yaw,
                    beetle_red.horn_pitch, beetle_red.horn_yaw
                )

                # Determine minimum distance threshold (use smaller of the two for safety)
                min_distance = min(HORN_PITCH_MIN_DISTANCE, HORN_YAW_MIN_DISTANCE) if (pitch_pressed and yaw_pressed) else (HORN_PITCH_MIN_DISTANCE if pitch_pressed else HORN_YAW_MIN_DISTANCE)

                # Apply changes if safe, otherwise block
                if distance >= min_distance:
                    beetle_blue.horn_pitch = new_pitch
                    beetle_blue.horn_yaw = new_yaw
                    beetle_blue.horn_pitch_velocity = pitch_speed
                    beetle_blue.horn_yaw_velocity = yaw_speed
                else:
                    # Blocked by collision
                    beetle_blue.horn_pitch_velocity = 0.0
                    beetle_blue.horn_yaw_velocity = 0.0
            elif pitch_pressed or yaw_pressed:
                # No other beetle - allow rotation freely
                beetle_blue.horn_pitch = new_pitch
                beetle_blue.horn_yaw = new_yaw
                beetle_blue.horn_pitch_velocity = pitch_speed
                beetle_blue.horn_yaw_velocity = yaw_speed
            else:
                # Not pressing horn keys - no velocity
                beetle_blue.horn_pitch_velocity = 0.0
                beetle_blue.horn_yaw_velocity = 0.0

            # Always add physics-driven rotation from collisions
            beetle_blue.rotation += beetle_blue.angular_velocity * PHYSICS_TIMESTEP
            beetle_blue.rotation = normalize_angle(beetle_blue.rotation)

        # === RED BEETLE CONTROLS (IJKL) - TANK STYLE ===
        if beetle_red.active and not beetle_red.is_falling:
            # Rotation controls (J/L) - BLOCKED during horn collision
            # 30% faster rotation when spinning in place (not moving forward/backward)
            if not beetle_red.in_horn_collision:
                # Check if rotating without moving (skill-based faster turning)
                is_moving = window.is_pressed('i') or window.is_pressed('k')
                rotation_multiplier = 1.0 if is_moving else 1.3  # 30% faster when stationary

                if window.is_pressed('j'):
                    beetle_red.rotation -= ROTATION_SPEED * rotation_multiplier * PHYSICS_TIMESTEP
                if window.is_pressed('l'):
                    beetle_red.rotation += ROTATION_SPEED * rotation_multiplier * PHYSICS_TIMESTEP

            # Movement controls (I/K) - move in facing direction
            if window.is_pressed('i'):
                # Move forward in facing direction
                move_x = math.cos(beetle_red.rotation)
                move_z = math.sin(beetle_red.rotation)
                beetle_red.apply_force(move_x * MOVE_FORCE, move_z * MOVE_FORCE, PHYSICS_TIMESTEP)
            if window.is_pressed('k'):
                # Move backward in facing direction
                move_x = -math.cos(beetle_red.rotation)
                move_z = -math.sin(beetle_red.rotation)
                beetle_red.apply_force(move_x * BACKWARD_MOVE_FORCE, move_z * BACKWARD_MOVE_FORCE, PHYSICS_TIMESTEP)

            # Horn controls - OPTIMIZED for combined pitch+yaw movements
            # Calculate proposed pitch and yaw changes
            pitch_pressed = window.is_pressed('u') or window.is_pressed('o')
            yaw_pressed = window.is_pressed('n') or window.is_pressed('m')

            new_pitch = beetle_red.horn_pitch
            new_yaw = beetle_red.horn_yaw
            pitch_speed = 0.0
            yaw_speed = 0.0

            # Calculate new pitch if pitch keys pressed
            # OPTIMIZATION: Use lookup table instead of string comparisons
            max_pitch_limit, min_pitch_limit = HORN_PITCH_LIMITS[beetle_red.horn_type_id]

            if window.is_pressed('u'):
                effective_speed = HORN_TILT_SPEED * (1.0 - beetle_red.horn_pitch_damping)
                new_pitch = beetle_red.horn_pitch + effective_speed * PHYSICS_TIMESTEP
                new_pitch = min(max_pitch_limit, new_pitch)
                pitch_speed = effective_speed
            elif window.is_pressed('o'):
                effective_speed = HORN_TILT_SPEED * (1.0 - beetle_red.horn_pitch_damping)
                new_pitch = beetle_red.horn_pitch - effective_speed * PHYSICS_TIMESTEP
                new_pitch = max(min_pitch_limit, new_pitch)
                pitch_speed = -effective_speed

            # Calculate new yaw if yaw keys pressed
            # For scorpion type, N/M control tail rotation angle instead of horn yaw
            # OPTIMIZATION: Use horn_type_id == 3 instead of string comparison
            if beetle_red.horn_type_id == 3:  # scorpion
                # SCORPION: N/M control red beetle's tail rotation
                TAIL_ROTATION_SPEED = 30.0  # Degrees per second
                if window.is_pressed('n'):
                    # N = Rotate tail up
                    beetle_red.tail_rotation_angle += TAIL_ROTATION_SPEED * PHYSICS_TIMESTEP
                    beetle_red.tail_rotation_angle = min(25.0, beetle_red.tail_rotation_angle)
                elif window.is_pressed('m'):
                    # M = Rotate tail down
                    beetle_red.tail_rotation_angle -= TAIL_ROTATION_SPEED * PHYSICS_TIMESTEP
                    beetle_red.tail_rotation_angle = max(-25.0, beetle_red.tail_rotation_angle)
                # Don't set yaw_pressed for scorpion (skip horn collision checks)
                yaw_pressed = False
            else:
                # OTHER TYPES: N/M control horn yaw (claws)
                # OPTIMIZATION: Use lookup table instead of string comparisons
                max_yaw_limit, min_yaw_limit = HORN_YAW_LIMITS[beetle_red.horn_type_id]

                if window.is_pressed('n'):
                    # N key DECREASES yaw = CLOSES pincers (toward min_yaw_limit)
                    effective_speed = HORN_YAW_SPEED * (1.0 - beetle_red.horn_yaw_damping)

                    # YAW LIFT: Apply lift to opponent when yawing during collision (all beetle types)
                    if beetle_blue.active:
                        has_real_collision = check_collision_kernel(
                            beetle_red.x, beetle_red.z, beetle_red.y,
                            beetle_blue.x, beetle_blue.z, beetle_blue.y,
                            beetle_red.color, beetle_blue.color
                        )
                        if has_real_collision:
                            # Apply push force to opponent (forward + lift)
                            forward_x = math.cos(beetle_red.rotation)
                            forward_z = math.sin(beetle_red.rotation)
                            push_force = 40.0 * PHYSICS_TIMESTEP
                            beetle_blue.vx += forward_x * push_force
                            beetle_blue.vz += forward_z * push_force
                            beetle_blue.vy += 25.0 * PHYSICS_TIMESTEP  # Lift up
                            beetle_blue.pitch -= 0.02  # Direct pitch tilt (front/grabbed area up)

                    new_yaw = beetle_red.horn_yaw - effective_speed * PHYSICS_TIMESTEP
                    new_yaw = max(min_yaw_limit, new_yaw)
                    yaw_speed = -effective_speed
                elif window.is_pressed('m'):
                    # M key INCREASES yaw = OPENS pincers (toward max_yaw_limit)
                    effective_speed = HORN_YAW_SPEED * (1.0 - beetle_red.horn_yaw_damping)

                    # YAW LIFT: Apply lift to opponent when yawing during collision (all beetle types)
                    if beetle_blue.active:
                        has_real_collision = check_collision_kernel(
                            beetle_red.x, beetle_red.z, beetle_red.y,
                            beetle_blue.x, beetle_blue.z, beetle_blue.y,
                            beetle_red.color, beetle_blue.color
                        )
                        if has_real_collision:
                            # Apply push force to opponent (forward + lift)
                            forward_x = math.cos(beetle_red.rotation)
                            forward_z = math.sin(beetle_red.rotation)
                            push_force = 40.0 * PHYSICS_TIMESTEP
                            beetle_blue.vx += forward_x * push_force
                            beetle_blue.vz += forward_z * push_force
                            beetle_blue.vy += 25.0 * PHYSICS_TIMESTEP  # Lift up
                            beetle_blue.pitch -= 0.02  # Direct pitch tilt (front/grabbed area up)

                    new_yaw = beetle_red.horn_yaw + effective_speed * PHYSICS_TIMESTEP
                    new_yaw = min(max_yaw_limit, new_yaw)
                    yaw_speed = effective_speed

            # Predictive collision check (optimized for combined movements + dual-pincer tracking)
            if (pitch_pressed or yaw_pressed) and beetle_blue.active:
                # Calculate minimum distance between horn tips (handles both stag and rhino)
                distance = calculate_min_horn_distance(
                    beetle_red, beetle_blue,
                    new_pitch, new_yaw,
                    beetle_blue.horn_pitch, beetle_blue.horn_yaw
                )

                # Determine minimum distance threshold (use smaller of the two for safety)
                min_distance = min(HORN_PITCH_MIN_DISTANCE, HORN_YAW_MIN_DISTANCE) if (pitch_pressed and yaw_pressed) else (HORN_PITCH_MIN_DISTANCE if pitch_pressed else HORN_YAW_MIN_DISTANCE)

                # Apply changes if safe, otherwise block
                if distance >= min_distance:
                    beetle_red.horn_pitch = new_pitch
                    beetle_red.horn_yaw = new_yaw
                    beetle_red.horn_pitch_velocity = pitch_speed
                    beetle_red.horn_yaw_velocity = yaw_speed
                else:
                    # Blocked by collision
                    beetle_red.horn_pitch_velocity = 0.0
                    beetle_red.horn_yaw_velocity = 0.0
            elif pitch_pressed or yaw_pressed:
                # No other beetle - allow rotation freely
                beetle_red.horn_pitch = new_pitch
                beetle_red.horn_yaw = new_yaw
                beetle_red.horn_pitch_velocity = pitch_speed
                beetle_red.horn_yaw_velocity = yaw_speed
            else:
                # Not pressing horn keys - no velocity
                beetle_red.horn_pitch_velocity = 0.0
                beetle_red.horn_yaw_velocity = 0.0

            # Always add physics-driven rotation from collisions
            beetle_red.rotation += beetle_red.angular_velocity * PHYSICS_TIMESTEP
            beetle_red.rotation = normalize_angle(beetle_red.rotation)

        # === INPUT/CONTROLS TIMING END, BEETLE PHYSICS START ===
        _t_input_end = time.perf_counter()
        _physics_timing['input_controls'] += (_t_input_end - _t_input_start) * 1000

        # Physics update
        beetle_blue.update_physics(PHYSICS_TIMESTEP)
        beetle_red.update_physics(PHYSICS_TIMESTEP)

        # Apply bowl slide physics when ball mode is active (slippery perimeter)
        if beetle_ball.active:
            apply_bowl_slide(beetle_blue, physics_params)
            apply_bowl_slide(beetle_red, physics_params)

        # === BEETLE PHYSICS TIMING END ===
        _t_beetle_phys_end = time.perf_counter()
        _physics_timing['beetle_physics'] += (_t_beetle_phys_end - _t_input_end) * 1000

        # Ball physics update (uses same beetle physics now)
        # Skip physics if ball has exploded (waiting for celebration to end)
        if beetle_ball.active and not g['ball_has_exploded']:
            # If ball has scored, just apply gravity and let it fall (no collisions/bounces)
            if g['ball_scored_this_fall']:
                beetle_ball.vy -= physics_params["GRAVITY"] * PHYSICS_TIMESTEP
                beetle_ball.y += beetle_ball.vy * PHYSICS_TIMESTEP
            else:
                beetle_ball.update_physics(PHYSICS_TIMESTEP)

                # Apply extra gravity to ball for faster, more natural falling (on top of normal gravity)
                extra_gravity = physics_params["GRAVITY"] * (physics_params["BALL_GRAVITY_MULTIPLIER"] - 1.0)
                beetle_ball.vy -= extra_gravity * PHYSICS_TIMESTEP

                # Apply rolling friction to ball only when on ground (allows proper arc when airborne)
                if beetle_ball.on_ground:
                    beetle_ball.vx *= physics_params["BALL_ROLLING_FRICTION"]
                    beetle_ball.vz *= physics_params["BALL_ROLLING_FRICTION"]
                    # Apply angular friction to all spin axes when on ground
                    beetle_ball.angular_velocity *= physics_params["BALL_ANGULAR_FRICTION"]
                    beetle_ball.pitch_velocity *= physics_params["BALL_ANGULAR_FRICTION"]
                    beetle_ball.roll_velocity *= physics_params["BALL_ANGULAR_FRICTION"]

                # Apply bowl slide to ball (slippery perimeter pushes toward center)
                apply_bowl_slide(beetle_ball, physics_params)

                # Update ball rotation from angular velocity (all 3 axes for full 3D spin)
                beetle_ball.rotation += beetle_ball.angular_velocity * PHYSICS_TIMESTEP
                beetle_ball.rotation = normalize_angle(beetle_ball.rotation)
                beetle_ball.pitch += beetle_ball.pitch_velocity * PHYSICS_TIMESTEP
                beetle_ball.roll += beetle_ball.roll_velocity * PHYSICS_TIMESTEP

            # Score detection - check if ball fell through goal pit
            # Ball Y is in physics space where floor is at ~0, so check well below
            # Goals are at x=-32 (blue) and x=32 (red) in physics space
            goal_pit_half_width = 12
            g = globals()
            if beetle_ball.y < -10:  # Ball fell well below floor level
                if not g['ball_scored_this_fall']:  # Only score once per fall
                    if abs(beetle_ball.z) < goal_pit_half_width:  # In goal lane (z is centered at 0 in physics space)
                        if beetle_ball.x < -32:  # Blue goal pit (west) - RED scores
                            g['ball_scored_this_fall'] = True
                            g['goal_scored_by'] = "RED"
                            g['goal_celebration_timer'] = 0.0
                            g['red_score_delay_timer'] = SCORE_ANIMATION_DELAY  # Start delay timer
                            g['red_score_pending'] = True  # Score will be added after delay
                            print(f"RED SCORES!")
                        elif beetle_ball.x > 32:  # Red goal pit (east) - BLUE scores
                            g['ball_scored_this_fall'] = True
                            g['goal_scored_by'] = "BLUE"
                            g['goal_celebration_timer'] = 0.0
                            g['blue_score_delay_timer'] = SCORE_ANIMATION_DELAY  # Start delay timer
                            g['blue_score_pending'] = True  # Score will be added after delay
                            print(f"BLUE SCORES!")
            # NOTE: ball_scored_this_fall is only reset on ball respawn, not when ball goes above ground
            # This prevents double-scoring if ball bounces in the goal pit

            # Ball-beetle collision (uses same collision system as beetle-beetle)
            # OPTIMIZATION: Only re-render ball and run ball collision if ball is close to beetle
            # Ball radius ~4, beetle max reach ~20, so collision only possible within ~25 units
            BALL_COLLISION_THRESHOLD_SQ = 30.0 * 30.0  # 900 = 30 units squared

            # Check distance from ball to each beetle (fast squared distance)
            blue_dx = beetle_ball.x - beetle_blue.x
            blue_dy = beetle_ball.y - beetle_blue.y
            blue_dz = beetle_ball.z - beetle_blue.z
            blue_dist_sq = blue_dx*blue_dx + blue_dy*blue_dy + blue_dz*blue_dz
            blue_close_to_ball = beetle_blue.active and blue_dist_sq < BALL_COLLISION_THRESHOLD_SQ

            red_dx = beetle_ball.x - beetle_red.x
            red_dy = beetle_ball.y - beetle_red.y
            red_dz = beetle_ball.z - beetle_red.z
            red_dist_sq = red_dx*red_dx + red_dy*red_dy + red_dz*red_dz
            red_close_to_ball = beetle_red.active and red_dist_sq < BALL_COLLISION_THRESHOLD_SQ

            # Re-render ball only if any beetle is close (OPTIMIZATION)
            if blue_close_to_ball or red_close_to_ball:
                if not g['ball_has_exploded']:
                    clear_and_render_ball_fast(beetle_ball.x, beetle_ball.y, beetle_ball.z, beetle_ball.rotation, beetle_ball.pitch, beetle_ball.roll)
                # Run ball collision only for close beetles
                if blue_close_to_ball:
                    beetle_collision(beetle_blue, beetle_ball, physics_params)
                if red_close_to_ball:
                    beetle_collision(beetle_red, beetle_ball, physics_params)

        # === BALL PHYSICS TIMING END ===
        _t_ball_end = time.perf_counter()
        _physics_timing['ball_physics'] += (_t_ball_end - _t_beetle_phys_end) * 1000

        # Update debris particles (physics, aging) - skip if no debris
        if simulation.num_debris[None] > 0:
            update_debris_particles(PHYSICS_TIMESTEP)

            # Cleanup dead debris every 2 frames
            if physics_frame % 2 == 0:
                cleanup_dead_debris()

        # === DEBRIS PARTICLES TIMING END ===
        _t_debris_end = time.perf_counter()
        _physics_timing['debris_particles'] += (_t_debris_end - _t_ball_end) * 1000

        # Fall death detection - two-stage system
        POINT_OF_NO_RETURN = -5.0  # Once below this, can't recover (5 voxels below floor)
        FALL_DEATH_Y = -25.0  # Fully removed at this point (25 voxels below floor)

        # Stage 1: Point of no return - disable controls but keep rendering
        if beetle_blue.active and not beetle_blue.is_falling and beetle_blue.y < POINT_OF_NO_RETURN:
            beetle_blue.is_falling = True
            print("BLUE BEETLE IS FALLING!")
        if beetle_red.active and not beetle_red.is_falling and beetle_red.y < POINT_OF_NO_RETURN:
            beetle_red.is_falling = True
            print("RED BEETLE IS FALLING!")

        # Stage 1.5: Death explosion - gradual particle burst over 0.7 seconds
        EXPLOSION_TRIGGER_Y = -16.0  # Trigger explosion after falling 16 voxels
        EXPLOSION_DELAY = 0.1  # Delay before particles start spawning
        EXPLOSION_DURATION = 0.7  # Spawn particles over 0.7 seconds
        TOTAL_PARTICLES = 1012  # 50% more than 675 (675 * 1.5 = 1012)
        PARTICLES_PER_FRAME = int(TOTAL_PARTICLES / (EXPLOSION_DURATION * 60))  # ~24 particles per frame at 60 FPS

        # Blue beetle explosion
        if beetle_blue.is_falling and not beetle_blue.has_exploded and beetle_blue.y < EXPLOSION_TRIGGER_Y:
            # Start explosion - store position
            beetle_blue.explosion_pos_x = beetle_blue.x
            beetle_blue.explosion_pos_y = beetle_blue.y + 30.0
            beetle_blue.explosion_pos_z = beetle_blue.z
            beetle_blue.explosion_delay = EXPLOSION_DELAY  # Delay before particles
            beetle_blue.explosion_timer = EXPLOSION_DURATION
            beetle_blue.has_exploded = True
            # Opponent scores and we start respawn timer (works in both normal and ball mode)
            g['red_score_delay_timer'] = SCORE_ANIMATION_DELAY  # Start delay timer
            g['red_score_pending'] = True  # Score will be added after delay
            g['blue_respawn_timer'] = BEETLE_RESPAWN_DELAY
            print(f"RED SCORES!")
            print("BLUE BEETLE EXPLOSION STARTED!")

        # Continue spawning particles during explosion (after delay)
        if beetle_blue.has_exploded:
            if beetle_blue.explosion_delay > 0.0:
                beetle_blue.explosion_delay -= PHYSICS_TIMESTEP
            elif beetle_blue.explosion_timer > 0.0:
                beetle_blue.explosion_timer -= PHYSICS_TIMESTEP
                # Calculate which batch to spawn
                progress = 1.0 - (beetle_blue.explosion_timer / EXPLOSION_DURATION)
                particles_spawned = int(progress * TOTAL_PARTICLES)
                batch_offset = max(0, particles_spawned - PARTICLES_PER_FRAME)
                batch_size = min(PARTICLES_PER_FRAME, TOTAL_PARTICLES - batch_offset)

                if batch_size > 0:
                    # Get blue beetle colors from window
                    blue_body = window.blue_body_color
                    blue_leg = window.blue_leg_color
                    blue_stripe = window.blue_stripe_color
                    blue_tip = window.blue_leg_tip_color
                    spawn_death_explosion_batch(beetle_blue.explosion_pos_x, beetle_blue.explosion_pos_y,
                                               beetle_blue.explosion_pos_z,
                                               blue_body[0], blue_body[1], blue_body[2],
                                               blue_leg[0], blue_leg[1], blue_leg[2],
                                               blue_stripe[0], blue_stripe[1], blue_stripe[2],
                                               blue_tip[0], blue_tip[1], blue_tip[2],
                                               batch_offset, batch_size, TOTAL_PARTICLES)

        # Red beetle explosion
        if beetle_red.is_falling and not beetle_red.has_exploded and beetle_red.y < EXPLOSION_TRIGGER_Y:
            # Start explosion - store position
            beetle_red.explosion_pos_x = beetle_red.x
            beetle_red.explosion_pos_y = beetle_red.y + 30.0
            beetle_red.explosion_pos_z = beetle_red.z
            beetle_red.explosion_delay = EXPLOSION_DELAY  # Delay before particles
            beetle_red.explosion_timer = EXPLOSION_DURATION
            beetle_red.has_exploded = True
            # Opponent scores and we start respawn timer (works in both normal and ball mode)
            g['blue_score_delay_timer'] = SCORE_ANIMATION_DELAY  # Start delay timer
            g['blue_score_pending'] = True  # Score will be added after delay
            g['red_respawn_timer'] = BEETLE_RESPAWN_DELAY
            print(f"BLUE SCORES!")
            print("RED BEETLE EXPLOSION STARTED!")

        # Continue spawning particles during explosion (after delay)
        if beetle_red.has_exploded:
            if beetle_red.explosion_delay > 0.0:
                beetle_red.explosion_delay -= PHYSICS_TIMESTEP
            elif beetle_red.explosion_timer > 0.0:
                beetle_red.explosion_timer -= PHYSICS_TIMESTEP
                # Calculate which batch to spawn
                progress = 1.0 - (beetle_red.explosion_timer / EXPLOSION_DURATION)
                particles_spawned = int(progress * TOTAL_PARTICLES)
                batch_offset = max(0, particles_spawned - PARTICLES_PER_FRAME)
                batch_size = min(PARTICLES_PER_FRAME, TOTAL_PARTICLES - batch_offset)

                if batch_size > 0:
                    # Get red beetle colors from window
                    red_body = window.red_body_color
                    red_leg = window.red_leg_color
                    red_stripe = window.red_stripe_color
                    red_tip = window.red_leg_tip_color
                    spawn_death_explosion_batch(beetle_red.explosion_pos_x, beetle_red.explosion_pos_y,
                                               beetle_red.explosion_pos_z,
                                               red_body[0], red_body[1], red_body[2],
                                               red_leg[0], red_leg[1], red_leg[2],
                                               red_stripe[0], red_stripe[1], red_stripe[2],
                                               red_tip[0], red_tip[1], red_tip[2],
                                               batch_offset, batch_size, TOTAL_PARTICLES)

        # Ball explosion - trigger when ball falls to same level as beetles
        g = globals()
        if beetle_ball.active and not g['ball_has_exploded'] and beetle_ball.y < EXPLOSION_TRIGGER_Y:
            # Start ball explosion - store position
            g['ball_explosion_pos_x'] = beetle_ball.x
            g['ball_explosion_pos_y'] = beetle_ball.y + 4.0  # Offset by ball radius
            g['ball_explosion_pos_z'] = beetle_ball.z
            g['ball_explosion_timer'] = EXPLOSION_DURATION
            g['ball_has_exploded'] = True
            # Hide the ball (stop rendering/physics) but keep it "active" for respawn
            beetle_ball.visible = False
            # Clear ball voxels immediately when explosion starts (same as beetles)
            if ball_last_rendered[None] == 1:
                num_voxels = ball_cache_size[None]
                if num_voxels > 0:
                    clear_ball_fast(ball_last_grid_x[None], ball_last_grid_y[None], ball_last_grid_z[None], num_voxels)
                ball_last_rendered[None] = 0
            print("BALL EXPLOSION!")

        # Continue spawning ball particles during explosion
        if g['ball_has_exploded'] and g['ball_explosion_timer'] > 0.0:
            g['ball_explosion_timer'] = g['ball_explosion_timer'] - PHYSICS_TIMESTEP
            # Calculate which batch to spawn
            progress = 1.0 - (g['ball_explosion_timer'] / EXPLOSION_DURATION)
            particles_spawned = int(progress * TOTAL_PARTICLES)
            batch_offset = max(0, particles_spawned - PARTICLES_PER_FRAME)
            batch_size = min(PARTICLES_PER_FRAME, TOTAL_PARTICLES - batch_offset)

            if batch_size > 0:
                spawn_ball_explosion_batch(g['ball_explosion_pos_x'], g['ball_explosion_pos_y'],
                                          g['ball_explosion_pos_z'],
                                          batch_offset, batch_size, TOTAL_PARTICLES)

        # === DEATH/EXPLOSIONS TIMING END ===
        _t_death_end = time.perf_counter()
        _physics_timing['death_explosions'] += (_t_death_end - _t_debris_end) * 1000

        # Goal celebration - winner gets confetti/flash after ball explodes
        if g['goal_scored_by'] is not None:
            g['goal_celebration_timer'] = g['goal_celebration_timer'] + PHYSICS_TIMESTEP

            # After ball explosion completes, trigger winner celebration (confetti + flash)
            CELEBRATION_DELAY = 0.7  # After ball explosion
            if g['goal_celebration_timer'] >= CELEBRATION_DELAY and victory_pulse_timer == 0.0:
                # Trigger victory pulse for the scorer
                victory_pulse_timer = 0.001  # Start the pulse (non-zero triggers it)
                # Set temporary winner for confetti colors
                match_winner = g['goal_scored_by']
                print(f"{g['goal_scored_by']} SCORES!")

            # Reset celebration after it's done
            CELEBRATION_DURATION = 4.0  # Total celebration time

            # Start ball assembly animation at 2.5 seconds into celebration
            if g['goal_celebration_timer'] >= ASSEMBLY_START_TIME and not g['ball_assembling']:
                # Initialize ball cache for assembly animation if not already done
                if ball_cache_size[None] == 0:
                    init_ball_cache(beetle_ball.radius)
                g['ball_assembling'] = True
                g['ball_assembly_timer'] = 0.0
                print("Ball assembly started!")

            # Update ball assembly timer during assembly
            if g['ball_assembling']:
                g['ball_assembly_timer'] += PHYSICS_TIMESTEP

            if g['goal_celebration_timer'] >= CELEBRATION_DURATION:
                g['goal_scored_by'] = None
                g['goal_celebration_timer'] = 0.0
                # Reset match_winner so it doesn't permanently affect the game
                match_winner = None
                victory_pulse_timer = 0.0
                # Reset ball for next round - respawn at center
                g['ball_has_exploded'] = False
                g['ball_explosion_timer'] = 0.0
                g['ball_assembling'] = False
                g['ball_assembly_timer'] = 0.0
                g['ball_scored_this_fall'] = False  # Reset score flag for new ball
                beetle_ball.x = 0.0
                beetle_ball.y = 28.0  # Drop from higher than beetles (assembly happens at y=57)
                beetle_ball.z = 0.0
                beetle_ball.vx = 0.0
                beetle_ball.vy = 0.0
                beetle_ball.vz = 0.0
                beetle_ball.angular_velocity = 0.0
                beetle_ball.pitch_velocity = 0.0
                beetle_ball.roll_velocity = 0.0
                beetle_ball.visible = True  # Make ball visible again
                print("Ball respawned!")

        # Stage 2: Full removal - deactivate completely
        if beetle_blue.active and beetle_blue.y < FALL_DEATH_Y:
            beetle_blue.active = False
            print("BLUE BEETLE FELL INTO THE ABYSS!")
            if match_winner is None:
                match_winner = "RED"
                # Start victory celebration (flash + confetti) in normal mode
                if not beetle_ball.active:
                    victory_pulse_timer = 0.001  # Trigger victory pulse
                print("\n" + "="*50)
                print("RED BEETLE WINS!")
                print("="*50 + "\n")
        if beetle_red.active and beetle_red.y < FALL_DEATH_Y:
            beetle_red.active = False
            print("RED BEETLE FELL INTO THE ABYSS!")
            if match_winner is None:
                match_winner = "BLUE"
                # Start victory celebration (flash + confetti) in normal mode
                if not beetle_ball.active:
                    victory_pulse_timer = 0.001  # Trigger victory pulse
                print("\n" + "="*50)
                print("BLUE BEETLE WINS!")
                print("="*50 + "\n")

        # Beetle respawn timers (works in both normal and ball mode)
        # Blue beetle respawn with assembly animation
        if g['blue_respawn_timer'] > 0:
            g['blue_respawn_timer'] -= PHYSICS_TIMESTEP
            # Start assembly animation when 1.5s remains
            remaining = g['blue_respawn_timer']
            if remaining <= ASSEMBLY_DURATION and not g['blue_assembling']:
                g['blue_assembling'] = True
                g['blue_assembly_timer'] = 0.0
                print("Blue beetle assembly started!")
            # Update assembly timer
            if g['blue_assembling']:
                g['blue_assembly_timer'] += PHYSICS_TIMESTEP
            # Complete respawn when timer hits 0
            if g['blue_respawn_timer'] <= 0:
                g['blue_respawn_timer'] = 0
                g['blue_assembling'] = False
                g['blue_assembly_timer'] = 0.0
                # Respawn blue beetle at center (drop from above)
                beetle_blue.x = 0.0   # Center
                beetle_blue.y = 15.0  # Drop from above
                beetle_blue.z = 0.0
                beetle_blue.vx = 0.0
                beetle_blue.vy = 0.0
                beetle_blue.vz = 0.0
                beetle_blue.rotation = 0.0  # Facing east
                beetle_blue.pitch = 0.0
                beetle_blue.roll = 0.0
                beetle_blue.angular_velocity = 0.0
                beetle_blue.pitch_velocity = 0.0
                beetle_blue.roll_velocity = 0.0
                beetle_blue.active = True
                beetle_blue.has_exploded = False
                beetle_blue.is_falling = False
                beetle_blue.on_ground = False  # Will fall to ground
                # Reset match winner so next death can trigger celebration
                match_winner = None
                victory_pulse_timer = 0.0
                print("Blue beetle respawned!")

        # Red beetle respawn with assembly animation
        if g['red_respawn_timer'] > 0:
            g['red_respawn_timer'] -= PHYSICS_TIMESTEP
            # Start assembly animation when 1.5s remains
            remaining = g['red_respawn_timer']
            if remaining <= ASSEMBLY_DURATION and not g['red_assembling']:
                g['red_assembling'] = True
                g['red_assembly_timer'] = 0.0
                print("Red beetle assembly started!")
            # Update assembly timer
            if g['red_assembling']:
                g['red_assembly_timer'] += PHYSICS_TIMESTEP
            # Complete respawn when timer hits 0
            if g['red_respawn_timer'] <= 0:
                g['red_respawn_timer'] = 0
                g['red_assembling'] = False
                g['red_assembly_timer'] = 0.0
                # Respawn red beetle at center (drop from above)
                beetle_red.x = 0.0    # Center
                beetle_red.y = 15.0   # Drop from above
                beetle_red.z = 0.0
                beetle_red.vx = 0.0
                beetle_red.vy = 0.0
                beetle_red.vz = 0.0
                beetle_red.rotation = math.pi  # Facing west
                beetle_red.pitch = 0.0
                beetle_red.roll = 0.0
                beetle_red.angular_velocity = 0.0
                beetle_red.pitch_velocity = 0.0
                beetle_red.roll_velocity = 0.0
                beetle_red.active = True
                beetle_red.has_exploded = False
                beetle_red.is_falling = False
                beetle_red.on_ground = False  # Will fall to ground
                # Reset match winner so next death can trigger celebration
                match_winner = None
                victory_pulse_timer = 0.0
                print("Red beetle respawned!")

        # === RESPAWN TIMERS TIMING END ===
        _t_respawn_end = time.perf_counter()
        _physics_timing['respawn_timers'] += (_t_respawn_end - _t_death_end) * 1000

        # Floor collision - prevent penetration by pushing beetles upward
        # Don't check floor collision if beetle is falling
        if beetle_blue.active and not beetle_blue.is_falling:
            floor_y_blue = check_floor_collision(beetle_blue.x, beetle_blue.z)
            if floor_y_blue > -100.0:  # Floor detected under beetle (world space, floor is at Y=0)
                # Calculate lowest point of beetle geometry after rotation
                lowest_point_blue = calculate_beetle_lowest_point(
                    beetle_blue.y, beetle_blue.rotation, beetle_blue.pitch,
                    beetle_blue.roll, beetle_blue.horn_pitch
                )

                # Check if beetle penetrates floor (lowest point goes into or below floor)
                floor_surface = floor_y_blue + 0.5  # Top of floor voxel surface
                if lowest_point_blue < floor_surface:
                    # Penetration detected! Push beetle upward
                    penetration_depth = floor_surface - lowest_point_blue
                    beetle_blue.y += penetration_depth

                    # Apply bounce/resistance
                    if beetle_blue.vy < 0:  # Moving downward
                        beetle_blue.vy = 0.0  # Stop downward motion

                    beetle_blue.on_ground = True
                elif lowest_point_blue < floor_surface + 0.5:  # Close to ground
                    beetle_blue.on_ground = True

        if beetle_red.active and not beetle_red.is_falling:
            floor_y_red = check_floor_collision(beetle_red.x, beetle_red.z)
            if floor_y_red >= 0:  # Floor detected under beetle
                lowest_point_red = calculate_beetle_lowest_point(
                    beetle_red.y, beetle_red.rotation, beetle_red.pitch,
                    beetle_red.roll, beetle_red.horn_pitch
                )

                floor_surface = floor_y_red + 0.5  # Top of floor voxel surface
                if lowest_point_red < floor_surface:
                    penetration_depth = floor_surface - lowest_point_red
                    beetle_red.y += penetration_depth

                    if beetle_red.vy < 0:
                        beetle_red.vy = 0.0

                    beetle_red.on_ground = True
                elif lowest_point_red < floor_surface + 0.5:
                    beetle_red.on_ground = True

        # Decrement ball dust cooldown
        if g['ball_dust_cooldown'] > 0.0:
            g['ball_dust_cooldown'] -= PHYSICS_TIMESTEP

        # Ball floor collision (same as beetles, but skip in goal pit areas)
        if beetle_ball.active:
            # Check if ball is in goal pit area (no floor there)
            goal_pit_half_width = 12
            in_goal_pit = abs(beetle_ball.z) < goal_pit_half_width and (beetle_ball.x <= -32 or beetle_ball.x >= 32)

            if in_goal_pit:
                # Ball is in goal pit - no floor collision, let it fall
                beetle_ball.on_ground = False
            else:
                floor_y_ball = check_floor_collision(beetle_ball.x, beetle_ball.z)
                if floor_y_ball > -100.0:  # Floor detected under ball
                    # Ball's lowest point is center Y minus radius
                    lowest_point_ball = beetle_ball.y - beetle_ball.radius
                    floor_surface = floor_y_ball + 0.5  # Top of floor voxel in world space

                    if lowest_point_ball < floor_surface:  # Ball penetrating floor
                        # Push ball upward to sit on floor
                        beetle_ball.y = floor_surface + beetle_ball.radius
                        # Also update prev_y to prevent interpolation artifacts (teleport, not smooth)
                        beetle_ball.prev_y = beetle_ball.y

                        # Apply bounce (reverse velocity with bounce coefficient)
                        if beetle_ball.vy < 0:
                            # Capture impact speed before reversing velocity
                            impact_speed = abs(beetle_ball.vy)

                            # Spawn dust ring on bounce (only if impact was significant and off cooldown)
                            if impact_speed > 7.0 and g['ball_dust_cooldown'] <= 0.0:
                                spawn_ball_bounce_dust(beetle_ball.x, RENDER_Y_OFFSET + 0.5, beetle_ball.z,
                                                       impact_speed, beetle_ball.radius)
                                g['ball_dust_cooldown'] = 0.1  # 0.1 second cooldown

                            beetle_ball.vy = -beetle_ball.vy * physics_params["BALL_GROUND_BOUNCE"]
                            # If bounce is very small, stop bouncing and settle
                            if abs(beetle_ball.vy) < 0.5:
                                beetle_ball.vy = 0.0

                        beetle_ball.on_ground = True
                    elif lowest_point_ball < floor_surface + 0.5:  # Close to ground
                        beetle_ball.on_ground = True
                    else:
                        # Ball is airborne - clear on_ground so rolling friction doesn't apply
                        beetle_ball.on_ground = False

        # Apply edge tipping physics (GPU-accelerated)
        if beetle_blue.active and not beetle_blue.is_falling:
            floor_y_blue_check = check_floor_collision(beetle_blue.x, beetle_blue.z)
            if floor_y_blue_check <= -100.0:  # No floor support
                calculate_edge_tipping_kernel(beetle_blue.x, beetle_blue.z, simulation.BEETLE_BLUE,
                                              PHYSICS_TIMESTEP, beetle_blue.pitch_inertia, beetle_blue.roll_inertia)
                beetle_blue.vy += edge_tipping_vy[None]
                beetle_blue.pitch_velocity += edge_tipping_pitch_vel[None]
                beetle_blue.roll_velocity += edge_tipping_roll_vel[None]

        if beetle_red.active and not beetle_red.is_falling:
            floor_y_red_check = check_floor_collision(beetle_red.x, beetle_red.z)
            if floor_y_red_check <= -100.0:  # No floor support
                calculate_edge_tipping_kernel(beetle_red.x, beetle_red.z, simulation.BEETLE_RED,
                                              PHYSICS_TIMESTEP, beetle_red.pitch_inertia, beetle_red.roll_inertia)
                beetle_red.vy += edge_tipping_vy[None]
                beetle_red.pitch_velocity += edge_tipping_pitch_vel[None]
                beetle_red.roll_velocity += edge_tipping_roll_vel[None]

        # === FLOOR COLLISION TIMING END ===
        _t_floor_end = time.perf_counter()
        _physics_timing['floor_collision'] += (_t_floor_end - _t_respawn_end) * 1000

        # Beetle collision (voxel-perfect) - only if both beetles are active
        if beetle_blue.active and beetle_red.active:
            beetle_collision(beetle_blue, beetle_red, physics_params)

        # === BEETLE COLLISION TIMING END ===
        _t_collision_end = time.perf_counter()
        _physics_timing['beetle_collision'] += (_t_collision_end - _t_floor_end) * 1000

        # Subtract fixed timestep from accumulator
        accumulator -= PHYSICS_TIMESTEP
        physics_frame += 1  # Increment frame counter
        physics_iterations_this_frame += 1

    # ===== END FIXED TIMESTEP PHYSICS LOOP =====
    perf_monitor.stop('physics')

    # Calculate interpolation alpha for smooth rendering between physics states
    alpha = accumulator / PHYSICS_TIMESTEP

    # Helper function for angle interpolation (shortest path)
    def lerp_angle(a, b, t):
        """Interpolate between angles using shortest path (radians)"""
        diff = (b - a) % TWO_PI
        if diff > math.pi:
            diff -= TWO_PI
        return a + diff * t

    # Interpolate blue beetle state for rendering
    blue_render_x = beetle_blue.prev_x + (beetle_blue.x - beetle_blue.prev_x) * alpha
    blue_render_y = beetle_blue.prev_y + (beetle_blue.y - beetle_blue.prev_y) * alpha
    blue_render_z = beetle_blue.prev_z + (beetle_blue.z - beetle_blue.prev_z) * alpha
    blue_render_rotation = lerp_angle(beetle_blue.prev_rotation, beetle_blue.rotation, alpha)
    blue_render_pitch = lerp_angle(beetle_blue.prev_pitch, beetle_blue.pitch, alpha)
    blue_render_roll = lerp_angle(beetle_blue.prev_roll, beetle_blue.roll, alpha)
    blue_render_horn_pitch = lerp_angle(beetle_blue.prev_horn_pitch, beetle_blue.horn_pitch, alpha)
    blue_render_horn_yaw = lerp_angle(beetle_blue.prev_horn_yaw, beetle_blue.horn_yaw, alpha)
    blue_render_tail_pitch = math.radians(15.0 + beetle_blue.tail_rotation_angle)  # Convert degrees to radians (base 15 + dynamic)

    # Interpolate red beetle state for rendering
    red_render_x = beetle_red.prev_x + (beetle_red.x - beetle_red.prev_x) * alpha
    red_render_y = beetle_red.prev_y + (beetle_red.y - beetle_red.prev_y) * alpha
    red_render_z = beetle_red.prev_z + (beetle_red.z - beetle_red.prev_z) * alpha
    red_render_rotation = lerp_angle(beetle_red.prev_rotation, beetle_red.rotation, alpha)
    red_render_pitch = lerp_angle(beetle_red.prev_pitch, beetle_red.pitch, alpha)
    red_render_roll = lerp_angle(beetle_red.prev_roll, beetle_red.roll, alpha)
    red_render_tail_pitch = math.radians(15.0 + beetle_red.tail_rotation_angle)  # Convert degrees to radians (base 15 + dynamic)
    red_render_horn_pitch = lerp_angle(beetle_red.prev_horn_pitch, beetle_red.horn_pitch, alpha)
    red_render_horn_yaw = lerp_angle(beetle_red.prev_horn_yaw, beetle_red.horn_yaw, alpha)

    # === ANIMATION TIMING ===
    perf_monitor.start('animation')

    # Update walk animation based on velocity and rotation (uses real-time frame_dt)
    # Detect blue beetle rotation-only input
    blue_rotating = window.is_pressed('f') or window.is_pressed('h')
    blue_moving = window.is_pressed('t') or window.is_pressed('g')
    blue_speed = math.sqrt(beetle_blue.vx**2 + beetle_blue.vz**2)

    # Check if rotating without moving forward/backward
    if blue_rotating and not blue_moving:
        # Rotation-only animation (use constant rotation speed)
        # Cancel animation completion if player resumes input
        beetle_blue.is_completing_animation = False
        # When in air: 30% faster than ground, on ground: normal turning speed (25% speed increase overall)
        if beetle_blue.is_lifted_high:
            rotation_animation_speed = 16.875 * 1.3  # 30% faster than ground turning speed
        else:
            rotation_animation_speed = 16.875  # Normal ground turning speed
        beetle_blue.walk_phase += rotation_animation_speed * WALK_CYCLE_SPEED * frame_dt
        beetle_blue.walk_phase = beetle_blue.walk_phase % TWO_PI
        beetle_blue.is_moving = True
        beetle_blue.is_rotating_only = True
        # Detect rotation direction
        if window.is_pressed('f'):
            beetle_blue.rotation_direction = -1  # Turning left
        else:
            beetle_blue.rotation_direction = 1   # Turning right
    elif blue_speed > 0.5:  # Normal forward/backward movement
        # Cancel animation completion if player resumes input
        beetle_blue.is_completing_animation = False
        # Check if moving forward or backward using dot product with facing direction
        facing_x = math.cos(beetle_blue.rotation)
        facing_z = math.sin(beetle_blue.rotation)
        move_dot = beetle_blue.vx * facing_x + beetle_blue.vz * facing_z
        if move_dot >= 0:  # Moving forward
            beetle_blue.walk_phase += blue_speed * WALK_CYCLE_SPEED * frame_dt
            beetle_blue.is_moving_backward = False
        else:  # Moving backward - reverse animation
            beetle_blue.walk_phase -= blue_speed * WALK_CYCLE_SPEED * frame_dt
            beetle_blue.is_moving_backward = True
        beetle_blue.walk_phase = beetle_blue.walk_phase % TWO_PI
        beetle_blue.is_moving = True
        beetle_blue.is_rotating_only = False
        beetle_blue.rotation_direction = 0
    else:
        # No input - check if we need to complete the animation cycle
        current_phase_normalized = beetle_blue.walk_phase % TWO_PI

        # Find nearest neutral position (0 or π)
        if current_phase_normalized < PI_HALF:
            beetle_blue.target_walk_phase = 0.0
        elif current_phase_normalized < PI_ONE_HALF:
            beetle_blue.target_walk_phase = math.pi
        else:
            beetle_blue.target_walk_phase = TWO_PI

        # Calculate distance to target
        phase_diff = abs(current_phase_normalized - (beetle_blue.target_walk_phase % TWO_PI))

        # If very close to neutral, snap immediately
        if phase_diff < 0.3:  # Within ~17 degrees, just snap
            beetle_blue.walk_phase = beetle_blue.target_walk_phase
            beetle_blue.is_completing_animation = False
        elif phase_diff > 0.1:  # Far enough that we need to animate
            # Advance toward target neutral position
            advance_amount = beetle_blue.completion_speed * WALK_CYCLE_SPEED * frame_dt
            # Don't overshoot - clamp to remaining distance
            if advance_amount > phase_diff:
                beetle_blue.walk_phase = beetle_blue.target_walk_phase
                beetle_blue.is_completing_animation = False
            else:
                beetle_blue.walk_phase += advance_amount
                beetle_blue.is_completing_animation = True
        else:
            # Already at neutral
            beetle_blue.is_completing_animation = False

        beetle_blue.walk_phase = beetle_blue.walk_phase % TWO_PI
        beetle_blue.is_moving = False
        beetle_blue.is_rotating_only = False
        beetle_blue.rotation_direction = 0

    # Detect red beetle rotation-only input
    red_rotating = window.is_pressed('j') or window.is_pressed('l')
    red_moving = window.is_pressed('i') or window.is_pressed('k')
    red_speed = math.sqrt(beetle_red.vx**2 + beetle_red.vz**2)

    # Check if rotating without moving forward/backward
    if red_rotating and not red_moving:
        # Rotation-only animation (use constant rotation speed)
        # Cancel animation completion if player resumes input
        beetle_red.is_completing_animation = False
        # When in air: 30% faster than ground, on ground: normal turning speed (25% speed increase overall)
        if beetle_red.is_lifted_high:
            rotation_animation_speed = 16.875 * 1.3  # 30% faster than ground turning speed
        else:
            rotation_animation_speed = 16.875  # Normal ground turning speed
        beetle_red.walk_phase += rotation_animation_speed * WALK_CYCLE_SPEED * frame_dt
        beetle_red.walk_phase = beetle_red.walk_phase % TWO_PI
        beetle_red.is_moving = True
        beetle_red.is_rotating_only = True
        # Detect rotation direction
        if window.is_pressed('j'):
            beetle_red.rotation_direction = -1  # Turning left
        else:
            beetle_red.rotation_direction = 1   # Turning right
    elif red_speed > 0.5:  # Normal forward/backward movement
        # Cancel animation completion if player resumes input
        beetle_red.is_completing_animation = False
        # Check if moving forward or backward using dot product with facing direction
        facing_x = math.cos(beetle_red.rotation)
        facing_z = math.sin(beetle_red.rotation)
        move_dot = beetle_red.vx * facing_x + beetle_red.vz * facing_z
        if move_dot >= 0:  # Moving forward
            beetle_red.walk_phase += red_speed * WALK_CYCLE_SPEED * frame_dt
            beetle_red.is_moving_backward = False
        else:  # Moving backward - reverse animation
            beetle_red.walk_phase -= red_speed * WALK_CYCLE_SPEED * frame_dt
            beetle_red.is_moving_backward = True
        beetle_red.walk_phase = beetle_red.walk_phase % TWO_PI
        beetle_red.is_moving = True
        beetle_red.is_rotating_only = False
        beetle_red.rotation_direction = 0
    else:
        # No input - check if we need to complete the animation cycle
        current_phase_normalized = beetle_red.walk_phase % TWO_PI

        # Find nearest neutral position (0 or π)
        if current_phase_normalized < PI_HALF:
            beetle_red.target_walk_phase = 0.0
        elif current_phase_normalized < PI_ONE_HALF:
            beetle_red.target_walk_phase = math.pi
        else:
            beetle_red.target_walk_phase = TWO_PI

        # Calculate distance to target
        phase_diff = abs(current_phase_normalized - (beetle_red.target_walk_phase % TWO_PI))

        # If very close to neutral, snap immediately
        if phase_diff < 0.3:  # Within ~17 degrees, just snap
            beetle_red.walk_phase = beetle_red.target_walk_phase
            beetle_red.is_completing_animation = False
        elif phase_diff > 0.1:  # Far enough that we need to animate
            # Advance toward target neutral position
            advance_amount = beetle_red.completion_speed * WALK_CYCLE_SPEED * frame_dt
            # Don't overshoot - clamp to remaining distance
            if advance_amount > phase_diff:
                beetle_red.walk_phase = beetle_red.target_walk_phase
                beetle_red.is_completing_animation = False
            else:
                beetle_red.walk_phase += advance_amount
                beetle_red.is_completing_animation = True
        else:
            # Already at neutral
            beetle_red.is_completing_animation = False

        beetle_red.walk_phase = beetle_red.walk_phase % TWO_PI
        beetle_red.is_moving = False
        beetle_red.is_rotating_only = False
        beetle_red.rotation_direction = 0

    # === LEG DUST KICK-UP EFFECT ===
    # Spawn dust particles when legs touch down during walking/turning
    # Particles kick up at ~30° angle from ground, fast fade
    DUST_COLOR = (0.45, 0.40, 0.35)  # Brownish gray arena dust
    DUST_SPEED_WALK = 5.2  # Speed for walking dust (+30%)
    DUST_SPEED_SPIN = 4.0  # Speed for spinning dust (outward from beetle center)

    # OPTIMIZATION: Pre-calculate sin values for walk phases (avoid repeated math.sin calls)
    blue_walk_sin = math.sin(beetle_blue.walk_phase)
    blue_prev_walk_sin = math.sin(beetle_blue.prev_walk_phase)
    blue_walk_sin_offset = math.sin(beetle_blue.walk_phase + math.pi)  # For Group B legs
    blue_prev_walk_sin_offset = math.sin(beetle_blue.prev_walk_phase + math.pi)

    # Blue beetle leg dust
    blue_leg_len = getattr(window, 'blue_leg_length_value', 8)  # Default 8 if not set yet
    blue_stagger_scale = blue_leg_len / 6.0  # Scale stagger based on leg length (6 is min)
    if beetle_blue.active and beetle_blue.y < 3.0:  # Only when on/near ground (within 3 voxels)
        # Walking: legs kick dust on touchdown (back legs when forward, front legs when backward)
        if beetle_blue.is_moving and not beetle_blue.is_rotating_only:
            # Use front legs (0,1) when backward, back legs (4,5 + 6,7 for scorpion) when forward
            if beetle_blue.is_moving_backward:
                dust_legs = [0, 1]  # Front legs
                kick_dir = 1.0  # Kick forward
            else:
                dust_legs = [4, 5]  # Back legs
                if beetle_blue.horn_type == "scorpion":
                    dust_legs = [4, 5, 6, 7]  # Include extra back legs for scorpion
                kick_dir = -1.0  # Kick backward
            for leg_id in dust_legs:
                # Use pre-calculated sin values based on leg group
                # Scorpion quadrupod: Group A (0, 3, 4, 7), Group B (1, 2, 5, 6)
                if leg_id in [0, 3, 4, 7]:  # Group A (no offset)
                    sin_leg = blue_walk_sin
                    sin_prev_leg = blue_prev_walk_sin
                else:  # Group B (pi offset) - legs 1, 2, 5, 6
                    sin_leg = blue_walk_sin_offset
                    sin_prev_leg = blue_prev_walk_sin_offset
                # Detect touchdown: sin crossed below -0.2 (leg firmly on ground)
                # Triggers slightly later in cycle for better sync with animation
                touchdown = sin_prev_leg > -0.2 and sin_leg <= -0.2
                if touchdown:
                    tip_x, tip_z = get_leg_tip_world_position(beetle_blue, leg_id, blue_leg_len)
                    # Only spawn dust if leg tip is on arena
                    tip_dist = math.sqrt(tip_x**2 + tip_z**2)
                    if tip_dist < ARENA_RADIUS:
                        # Kick direction: backward when forward, forward when backward
                        dir_x = kick_dir * math.cos(beetle_blue.rotation)
                        dir_z = kick_dir * math.sin(beetle_blue.rotation)
                        # Per-leg random offset for variety
                        rand_x = (random.random() - 0.5) * 1.5
                        rand_z = (random.random() - 0.5) * 1.5
                        spawn_leg_dust_staggered(tip_x, RENDER_Y_OFFSET + 0.5, tip_z, dir_x, dir_z, DUST_SPEED_WALK,
                                                DUST_COLOR[0], DUST_COLOR[1], DUST_COLOR[2], rand_x, rand_z, blue_stagger_scale, 8)

        # Spinning: spawn dust from back leg on opposite side
        # Left turn (side=-1): back RIGHT leg (leg 5, and 7 for scorpion)
        # Right turn (side=1): back LEFT leg (leg 4, and 6 for scorpion)
        elif beetle_blue.is_rotating_only:
            beetle_blue.spin_dust_timer += frame_dt
            if beetle_blue.spin_dust_timer >= 0.04:  # Every 0.04 seconds
                beetle_blue.spin_dust_timer = 0.0
                side = beetle_blue.rotation_direction  # -1 = left turn, 1 = right turn
                if side != 0:
                    # BACK legs on OPPOSITE side of turn
                    # Left turn -> leg 5 (rear_right), and 7 for scorpion
                    # Right turn -> leg 4 (rear_left), and 6 for scorpion
                    back_legs = [5, 7] if side == 1 else [4, 6]
                    if beetle_blue.horn_type != "scorpion":
                        back_legs = back_legs[:1]  # Only first leg for non-scorpion
                    for back_leg_id in back_legs:
                        tip_x, tip_z = get_leg_tip_world_position(beetle_blue, back_leg_id, blue_leg_len)
                        tip_dist = math.sqrt(tip_x**2 + tip_z**2)
                        if tip_dist < ARENA_RADIUS:
                            dir_x = tip_x - beetle_blue.x
                            dir_z = tip_z - beetle_blue.z
                            dir_len = math.sqrt(dir_x**2 + dir_z**2)
                            if dir_len > 0:
                                dir_x /= dir_len
                                dir_z /= dir_len
                            spawn_spin_dust_puff(tip_x, RENDER_Y_OFFSET + 0.5, tip_z, dir_x, dir_z,
                                                DUST_COLOR[0], DUST_COLOR[1], DUST_COLOR[2], blue_stagger_scale, 1)

                    # FRONT leg on SAME side of turn
                    # Left turn -> leg 0 (front_left)
                    # Right turn -> leg 1 (front_right)
                    front_leg_id = 0 if side == 1 else 1
                    tip_x, tip_z = get_leg_tip_world_position(beetle_blue, front_leg_id, blue_leg_len)
                    tip_dist = math.sqrt(tip_x**2 + tip_z**2)
                    if tip_dist < ARENA_RADIUS:
                        dir_x = tip_x - beetle_blue.x
                        dir_z = tip_z - beetle_blue.z
                        dir_len = math.sqrt(dir_x**2 + dir_z**2)
                        if dir_len > 0:
                            dir_x /= dir_len
                            dir_z /= dir_len
                        spawn_spin_dust_puff(tip_x, RENDER_Y_OFFSET + 0.5, tip_z, dir_x, dir_z,
                                            DUST_COLOR[0], DUST_COLOR[1], DUST_COLOR[2], blue_stagger_scale, 1)
        else:
            beetle_blue.spin_dust_timer = 0.0  # Reset timer when not spinning

    # Red beetle leg dust - pre-calculate sin values
    red_walk_sin = math.sin(beetle_red.walk_phase)
    red_prev_walk_sin = math.sin(beetle_red.prev_walk_phase)
    red_walk_sin_offset = math.sin(beetle_red.walk_phase + math.pi)  # For Group B legs
    red_prev_walk_sin_offset = math.sin(beetle_red.prev_walk_phase + math.pi)

    red_leg_len = getattr(window, 'red_leg_length_value', 8)  # Default 8 if not set yet
    red_stagger_scale = red_leg_len / 6.0  # Scale stagger based on leg length (6 is min)
    if beetle_red.active and beetle_red.y < 3.0:  # Only when on/near ground (within 3 voxels)
        # Walking: legs kick dust on touchdown (back legs when forward, front legs when backward)
        if beetle_red.is_moving and not beetle_red.is_rotating_only:
            # Use front legs (0,1) when backward, back legs (4,5 + 6,7 for scorpion) when forward
            if beetle_red.is_moving_backward:
                dust_legs = [0, 1]  # Front legs
                kick_dir = 1.0  # Kick forward
            else:
                dust_legs = [4, 5]  # Back legs
                if beetle_red.horn_type == "scorpion":
                    dust_legs = [4, 5, 6, 7]  # Include extra back legs for scorpion
                kick_dir = -1.0  # Kick backward
            for leg_id in dust_legs:
                # Use pre-calculated sin values based on leg group
                # Scorpion quadrupod: Group A (0, 3, 4, 7), Group B (1, 2, 5, 6)
                if leg_id in [0, 3, 4, 7]:  # Group A (no offset)
                    sin_leg = red_walk_sin
                    sin_prev_leg = red_prev_walk_sin
                else:  # Group B (pi offset) - legs 1, 2, 5, 6
                    sin_leg = red_walk_sin_offset
                    sin_prev_leg = red_prev_walk_sin_offset
                # Detect touchdown: sin crossed below -0.2 (leg firmly on ground)
                # Triggers slightly later in cycle for better sync with animation
                touchdown = sin_prev_leg > -0.2 and sin_leg <= -0.2
                if touchdown:
                    tip_x, tip_z = get_leg_tip_world_position(beetle_red, leg_id, red_leg_len)
                    # Only spawn dust if leg tip is on arena
                    tip_dist = math.sqrt(tip_x**2 + tip_z**2)
                    if tip_dist < ARENA_RADIUS:
                        # Kick direction: backward when forward, forward when backward
                        dir_x = kick_dir * math.cos(beetle_red.rotation)
                        dir_z = kick_dir * math.sin(beetle_red.rotation)
                        # Per-leg random offset for variety
                        rand_x = (random.random() - 0.5) * 1.5
                        rand_z = (random.random() - 0.5) * 1.5
                        spawn_leg_dust_staggered(tip_x, RENDER_Y_OFFSET + 0.5, tip_z, dir_x, dir_z, DUST_SPEED_WALK,
                                                DUST_COLOR[0], DUST_COLOR[1], DUST_COLOR[2], rand_x, rand_z, red_stagger_scale, 8)

        # Spinning: spawn dust from back leg on opposite side
        # Left turn (side=-1): back RIGHT leg (leg 5, and 7 for scorpion)
        # Right turn (side=1): back LEFT leg (leg 4, and 6 for scorpion)
        elif beetle_red.is_rotating_only:
            beetle_red.spin_dust_timer += frame_dt
            if beetle_red.spin_dust_timer >= 0.04:  # Every 0.04 seconds
                beetle_red.spin_dust_timer = 0.0
                side = beetle_red.rotation_direction  # -1 = left turn, 1 = right turn
                if side != 0:
                    # BACK legs on OPPOSITE side of turn
                    # Left turn -> leg 5 (rear_right), and 7 for scorpion
                    # Right turn -> leg 4 (rear_left), and 6 for scorpion
                    back_legs = [5, 7] if side == 1 else [4, 6]
                    if beetle_red.horn_type != "scorpion":
                        back_legs = back_legs[:1]  # Only first leg for non-scorpion
                    for back_leg_id in back_legs:
                        tip_x, tip_z = get_leg_tip_world_position(beetle_red, back_leg_id, red_leg_len)
                        tip_dist = math.sqrt(tip_x**2 + tip_z**2)
                        if tip_dist < ARENA_RADIUS:
                            dir_x = tip_x - beetle_red.x
                            dir_z = tip_z - beetle_red.z
                            dir_len = math.sqrt(dir_x**2 + dir_z**2)
                            if dir_len > 0:
                                dir_x /= dir_len
                                dir_z /= dir_len
                            spawn_spin_dust_puff(tip_x, RENDER_Y_OFFSET + 0.5, tip_z, dir_x, dir_z,
                                                DUST_COLOR[0], DUST_COLOR[1], DUST_COLOR[2], red_stagger_scale, 1)

                    # FRONT leg on SAME side of turn
                    # Left turn -> leg 0 (front_left)
                    # Right turn -> leg 1 (front_right)
                    front_leg_id = 0 if side == 1 else 1
                    tip_x, tip_z = get_leg_tip_world_position(beetle_red, front_leg_id, red_leg_len)
                    tip_dist = math.sqrt(tip_x**2 + tip_z**2)
                    if tip_dist < ARENA_RADIUS:
                        dir_x = tip_x - beetle_red.x
                        dir_z = tip_z - beetle_red.z
                        dir_len = math.sqrt(dir_x**2 + dir_z**2)
                        if dir_len > 0:
                            dir_x /= dir_len
                            dir_z /= dir_len
                        spawn_spin_dust_puff(tip_x, RENDER_Y_OFFSET + 0.5, tip_z, dir_x, dir_z,
                                            DUST_COLOR[0], DUST_COLOR[1], DUST_COLOR[2], red_stagger_scale, 1)
        else:
            beetle_red.spin_dust_timer = 0.0  # Reset timer when not spinning

    # Update previous walk phase for next frame's touchdown detection
    beetle_blue.prev_walk_phase = beetle_blue.walk_phase
    beetle_red.prev_walk_phase = beetle_red.walk_phase

    # Detect if beetles are lifted high (for leg spaz animation)
    LIFT_THRESHOLD = 5.0  # 4 voxels above normal ground
    beetle_blue.is_lifted_high = (beetle_blue.y > LIFT_THRESHOLD)
    beetle_red.is_lifted_high = (beetle_red.y > LIFT_THRESHOLD)

    # Victory pulse effect - winner beetle glows after a KO
    if match_winner is not None and victory_pulse_timer < VICTORY_PULSE_DURATION:
        victory_pulse_timer += frame_dt
        # Fade out intensity over duration (1.0 at start, 0.0 at end)
        fade = 1.0 - (victory_pulse_timer / VICTORY_PULSE_DURATION)
        # Pulsing brightness: oscillates between 1.0 and 1.6, fading to 1.0 over time
        pulse = 1.0 + 0.6 * fade * math.sin(victory_pulse_timer * 10.0)  # Fast pulse that fades

        # Victory confetti - rain down colored particles during victory (after initial delay)
        if victory_pulse_timer >= VICTORY_CONFETTI_DELAY:
            victory_confetti_timer += frame_dt
            if victory_confetti_timer >= VICTORY_CONFETTI_INTERVAL:
                victory_confetti_timer = 0.0
                # Spawn confetti above the arena center at a high position
                confetti_height = 50.0  # High above the arena
                if match_winner == "BLUE":
                    b_body = window.blue_body_color
                    b_leg = window.blue_leg_color
                    b_stripe = window.blue_stripe_color
                    b_tip = window.blue_horn_tip_color
                    spawn_victory_confetti(0.0, 0.0, confetti_height,
                                           b_body[0], b_body[1], b_body[2],
                                           b_leg[0], b_leg[1], b_leg[2],
                                           b_stripe[0], b_stripe[1], b_stripe[2],
                                           b_tip[0], b_tip[1], b_tip[2],
                                           VICTORY_CONFETTI_PARTICLES)
                else:  # RED winner
                    r_body = window.red_body_color
                    r_leg = window.red_leg_color
                    r_stripe = window.red_stripe_color
                    r_tip = window.red_horn_tip_color
                    spawn_victory_confetti(0.0, 0.0, confetti_height,
                                           r_body[0], r_body[1], r_body[2],
                                           r_leg[0], r_leg[1], r_leg[2],
                                           r_stripe[0], r_stripe[1], r_stripe[2],
                                           r_tip[0], r_tip[1], r_tip[2],
                                           VICTORY_CONFETTI_PARTICLES)

        if match_winner == "BLUE":
            # Pulse all blue beetle colors
            b = window.blue_body_color
            simulation.blue_body_color[None] = ti.Vector([min(b[0] * pulse, 1.0), min(b[1] * pulse, 1.0), min(b[2] * pulse, 1.0)])
            b = window.blue_leg_color
            simulation.blue_leg_color[None] = ti.Vector([min(b[0] * pulse, 1.0), min(b[1] * pulse, 1.0), min(b[2] * pulse, 1.0)])
            b = window.blue_leg_tip_color
            simulation.blue_leg_tip_color[None] = ti.Vector([min(b[0] * pulse, 1.0), min(b[1] * pulse, 1.0), min(b[2] * pulse, 1.0)])
            b = window.blue_stripe_color
            simulation.blue_stripe_color[None] = ti.Vector([min(b[0] * pulse, 1.0), min(b[1] * pulse, 1.0), min(b[2] * pulse, 1.0)])
            b = window.blue_horn_tip_color
            simulation.blue_horn_tip_color[None] = ti.Vector([min(b[0] * pulse, 1.0), min(b[1] * pulse, 1.0), min(b[2] * pulse, 1.0)])
        else:  # RED winner
            # Pulse all red beetle colors
            r = window.red_body_color
            simulation.red_body_color[None] = ti.Vector([min(r[0] * pulse, 1.0), min(r[1] * pulse, 1.0), min(r[2] * pulse, 1.0)])
            r = window.red_leg_color
            simulation.red_leg_color[None] = ti.Vector([min(r[0] * pulse, 1.0), min(r[1] * pulse, 1.0), min(r[2] * pulse, 1.0)])
            r = window.red_leg_tip_color
            simulation.red_leg_tip_color[None] = ti.Vector([min(r[0] * pulse, 1.0), min(r[1] * pulse, 1.0), min(r[2] * pulse, 1.0)])
            r = window.red_stripe_color
            simulation.red_stripe_color[None] = ti.Vector([min(r[0] * pulse, 1.0), min(r[1] * pulse, 1.0), min(r[2] * pulse, 1.0)])
            r = window.red_horn_tip_color
            simulation.red_horn_tip_color[None] = ti.Vector([min(r[0] * pulse, 1.0), min(r[1] * pulse, 1.0), min(r[2] * pulse, 1.0)])
    elif match_winner is not None and victory_pulse_timer >= VICTORY_PULSE_DURATION:
        # Reset to normal colors after pulse ends
        if match_winner == "BLUE":
            b = window.blue_body_color
            simulation.blue_body_color[None] = ti.Vector([b[0], b[1], b[2]])
            b = window.blue_leg_color
            simulation.blue_leg_color[None] = ti.Vector([b[0], b[1], b[2]])
            b = window.blue_leg_tip_color
            simulation.blue_leg_tip_color[None] = ti.Vector([b[0], b[1], b[2]])
            b = window.blue_stripe_color
            simulation.blue_stripe_color[None] = ti.Vector([b[0], b[1], b[2]])
            b = window.blue_horn_tip_color
            simulation.blue_horn_tip_color[None] = ti.Vector([b[0], b[1], b[2]])
        else:
            r = window.red_body_color
            simulation.red_body_color[None] = ti.Vector([r[0], r[1], r[2]])
            r = window.red_leg_color
            simulation.red_leg_color[None] = ti.Vector([r[0], r[1], r[2]])
            r = window.red_leg_tip_color
            simulation.red_leg_tip_color[None] = ti.Vector([r[0], r[1], r[2]])
            r = window.red_stripe_color
            simulation.red_stripe_color[None] = ti.Vector([r[0], r[1], r[2]])
            r = window.red_horn_tip_color
            simulation.red_horn_tip_color[None] = ti.Vector([r[0], r[1], r[2]])

    perf_monitor.stop('animation')

    # Render - ANIMATED with leg walking cycles and interpolated smooth positions!
    # === VOXEL CLEAR TIMING ===
    perf_monitor.start('voxel_clear')
    # OPTIMIZATION: Use dirty voxel tracking instead of scanning 250K voxels
    clear_dirty_voxels()  # Clear voxels from previous frame (only ~600 voxels)
    reset_dirty_voxels()  # Reset counter for this frame's tracking
    perf_monitor.stop('voxel_clear')

    # Place shadows for airborne beetles (only when at least one beetle is airborne)
    blue_needs_shadow = beetle_blue.active and blue_render_y > SHADOW_HEIGHT_THRESHOLD
    red_needs_shadow = beetle_red.active and red_render_y > SHADOW_HEIGHT_THRESHOLD
    floor_y = int(RENDER_Y_OFFSET)  # Shadow replaces floor voxels (flush with surface)

    # Always clear shadows if they existed last frame (prevents artifacts when beetles land)
    if shadows_were_placed:
        clear_shadow_layer(floor_y, 1 if beetle_ball.active else 0)
        shadows_were_placed = False

    if blue_needs_shadow or red_needs_shadow:
        shadows_were_placed = True  # Mark that we're placing shadows this frame

        if blue_needs_shadow:
            height_factor = min((blue_render_y - SHADOW_HEIGHT_THRESHOLD) / (SHADOW_MAX_HEIGHT - SHADOW_HEIGHT_THRESHOLD), 1.0)
            radius_float = SHADOW_BASE_RADIUS + height_factor * (SHADOW_MAX_RADIUS - SHADOW_BASE_RADIUS)
            # Offset shadow 2 voxels toward the butt (opposite of facing direction)
            shadow_x = blue_render_x - 2 * math.cos(blue_render_rotation)
            shadow_z = blue_render_z - 2 * math.sin(blue_render_rotation)
            place_shadow_kernel(shadow_x, shadow_z, radius_float, floor_y)
        if red_needs_shadow:
            height_factor = min((red_render_y - SHADOW_HEIGHT_THRESHOLD) / (SHADOW_MAX_HEIGHT - SHADOW_HEIGHT_THRESHOLD), 1.0)
            radius_float = SHADOW_BASE_RADIUS + height_factor * (SHADOW_MAX_RADIUS - SHADOW_BASE_RADIUS)
            # Offset shadow 2 voxels toward the butt (opposite of facing direction)
            shadow_x = red_render_x - 2 * math.cos(red_render_rotation)
            shadow_z = red_render_z - 2 * math.sin(red_render_rotation)
            place_shadow_kernel(shadow_x, shadow_z, radius_float, floor_y)

    # Convert horn_type string to horn_type_id for each beetle: 0=rhino, 1=stag, 2=hercules, 3=scorpion, 4=atlas
    blue_horn_type_id = 1 if blue_horn_type == "stag" else (2 if blue_horn_type == "hercules" else (3 if blue_horn_type == "scorpion" else (4 if blue_horn_type == "atlas" else 0)))
    red_horn_type_id = 1 if red_horn_type == "stag" else (2 if red_horn_type == "hercules" else (3 if red_horn_type == "scorpion" else (4 if red_horn_type == "atlas" else 0)))

    # Get default horn pitch for blue beetle type
    if blue_horn_type == "scorpion":
        blue_default_horn_pitch = HORN_DEFAULT_PITCH_SCORPION
    elif blue_horn_type == "stag":
        blue_default_horn_pitch = HORN_DEFAULT_PITCH_STAG
    elif blue_horn_type == "hercules":
        blue_default_horn_pitch = HORN_DEFAULT_PITCH_HERCULES
    elif blue_horn_type == "atlas":
        blue_default_horn_pitch = HORN_DEFAULT_PITCH_ATLAS
    else:
        blue_default_horn_pitch = HORN_DEFAULT_PITCH

    # Get default horn pitch for red beetle type
    if red_horn_type == "scorpion":
        red_default_horn_pitch = HORN_DEFAULT_PITCH_SCORPION
    elif red_horn_type == "stag":
        red_default_horn_pitch = HORN_DEFAULT_PITCH_STAG
    elif red_horn_type == "hercules":
        red_default_horn_pitch = HORN_DEFAULT_PITCH_HERCULES
    elif red_horn_type == "atlas":
        red_default_horn_pitch = HORN_DEFAULT_PITCH_ATLAS
    else:
        red_default_horn_pitch = HORN_DEFAULT_PITCH

    # Initialize persistent slider values (needed for kernel parameters) - separate for each beetle
    if not hasattr(window, 'blue_horn_shaft_value'):
        # Blue beetle sliders
        window.blue_horn_shaft_value = 12
        window.blue_horn_prong_value = 5
        window.blue_back_body_height_value = 6
        window.blue_body_length_value = 12
        window.blue_body_width_value = 7
        window.blue_leg_length_value = 8

        # Red beetle sliders
        window.red_horn_shaft_value = 12
        window.red_horn_prong_value = 5
        window.red_back_body_height_value = 6
        window.red_body_length_value = 12
        window.red_body_width_value = 7
        window.red_leg_length_value = 8

        # Old slider variables (for compatibility with existing UI - they update BOTH beetles)
        window.horn_shaft_value = 12
        window.horn_prong_value = 5
        window.back_body_height_value = 6
        window.body_length_value = 12
        window.body_width_value = 7
        window.leg_length_value = 8

        # Blue beetle colors (RGB values 0.0-1.0)
        window.blue_body_color = (0.25, 0.55, 0.95)
        window.blue_leg_color = (0.4, 0.7, 1.0)
        window.blue_leg_tip_color = (0.0, 0.0, 0.3)
        window.blue_stripe_color = (0.6, 0.9, 1.0)
        window.blue_horn_tip_color = (0.4, 0.75, 1.0)

        # Red beetle colors (RGB values 0.0-1.0)
        window.red_body_color = (0.95, 0.25, 0.15)
        window.red_leg_color = (1.0, 0.5, 0.3)
        window.red_leg_tip_color = (0.3, 0.0, 0.0)
        window.red_stripe_color = (0.85, 0.65, 0.2)
        window.red_horn_tip_color = (0.4, 0.1, 0.1)

    # === BEETLE RENDER TIMING ===
    perf_monitor.start('beetle_render')

    if beetle_blue.active:
        # Render blue beetle using its own cache
        place_animated_beetle_blue(blue_render_x, blue_render_y, blue_render_z, blue_render_rotation, blue_render_pitch, blue_render_roll, blue_render_horn_pitch, blue_render_horn_yaw, blue_render_tail_pitch, blue_horn_type_id, beetle_blue.body_pitch_offset, simulation.BEETLE_BLUE, simulation.BEETLE_BLUE_LEGS, simulation.LEG_TIP_BLUE, beetle_blue.walk_phase, 1 if beetle_blue.is_lifted_high else 0, blue_default_horn_pitch, window.blue_body_length_value, window.blue_back_body_height_value, 1 if beetle_blue.is_rotating_only else 0, beetle_blue.rotation_direction)

    if beetle_red.active:
        # Render red beetle using its own cache
        place_animated_beetle_red(red_render_x, red_render_y, red_render_z, red_render_rotation, red_render_pitch, red_render_roll, red_render_horn_pitch, red_render_horn_yaw, red_render_tail_pitch, red_horn_type_id, beetle_red.body_pitch_offset, simulation.BEETLE_RED, simulation.BEETLE_RED_LEGS, simulation.LEG_TIP_RED, beetle_red.walk_phase, 1 if beetle_red.is_lifted_high else 0, red_default_horn_pitch, window.red_body_length_value, window.red_back_body_height_value, 1 if beetle_red.is_rotating_only else 0, beetle_red.rotation_direction)

    # Render beetle assembly animations (voxel rain effect) - GPU accelerated
    g = globals()
    # Track if any assembly is happening this frame
    any_assembling = g['blue_assembling'] or g['red_assembling'] or g['ball_assembling']
    # Only clear assembly voxels if assembly is happening OR we need one final clear (OPTIMIZATION)
    if any_assembling or g['assembly_needs_final_clear']:
        clear_assembly_voxels()  # Clear previous frame's assembly voxels
        g['assembly_needs_final_clear'] = any_assembling  # Set flag for next frame if still assembling
    if g['blue_assembling']:
        progress = min(g['blue_assembly_timer'] / ASSEMBLY_DURATION, 1.0)
        # Assemble high above arena (y=50), beetle will drop from y=15 after assembly
        render_beetle_assembly_fast(True, 0.0, 50.0, 0.0, progress)

    if g['red_assembling']:
        progress = min(g['red_assembly_timer'] / ASSEMBLY_DURATION, 1.0)
        # Assemble high above arena (y=50), beetle will drop from y=15 after assembly
        render_beetle_assembly_fast(False, 0.0, 50.0, 0.0, progress)

    # Render ball assembly animation (voxel rain effect)
    if g['ball_assembling'] and ball_cache_size[None] > 0:
        progress = min(g['ball_assembly_timer'] / BALL_ASSEMBLY_DURATION, 1.0)
        # Assemble high above arena (y=59), ball will drop from y=30 after assembly
        render_ball_assembly_fast(0.0, 57.0, 0.0, progress)

    perf_monitor.stop('beetle_render')

    # === BALL RENDER TIMING ===
    perf_monitor.start('ball_render')

    # Render ball (beetle soccer ball) - clear old voxels first to prevent trail
    if beetle_ball.active:
        # Interpolate ball position for smooth movement
        ball_render_x = beetle_ball.prev_x + (beetle_ball.x - beetle_ball.prev_x) * alpha
        ball_render_y = beetle_ball.prev_y + (beetle_ball.y - beetle_ball.prev_y) * alpha
        ball_render_z = beetle_ball.prev_z + (beetle_ball.z - beetle_ball.prev_z) * alpha
        ball_render_rotation = lerp_angle(beetle_ball.prev_rotation, beetle_ball.rotation, alpha)
        ball_render_pitch = lerp_angle(beetle_ball.prev_pitch, beetle_ball.pitch, alpha)
        ball_render_roll = lerp_angle(beetle_ball.prev_roll, beetle_ball.roll, alpha)

        # Only render ball if it hasn't exploded - OPTIMIZED (clear+render in one call)
        if not ball_has_exploded:
            clear_and_render_ball_fast(ball_render_x, ball_render_y, ball_render_z, ball_render_rotation, ball_render_pitch, ball_render_roll)

            # Add shadow under ball when airborne (ball center must be high enough that bottom clears ground)
            # Ball bottom = ball_render_y - radius, so ball is airborne when bottom > ~1
            ball_bottom_y = ball_render_y - beetle_ball.radius
            if ball_bottom_y > 2.0:  # Ball is clearly off the ground
                # Ball shadow grows slower than beetles - larger height range
                ball_shadow_max_height = 35.0  # Slower growth than beetles (25)
                height_factor = min((ball_bottom_y - 2.0) / (ball_shadow_max_height - 2.0), 1.0)
                ball_shadow_radius = SHADOW_BASE_RADIUS + height_factor * (SHADOW_MAX_RADIUS - SHADOW_BASE_RADIUS)
                place_shadow_kernel(ball_render_x, ball_render_z, ball_shadow_radius, floor_y)
                shadows_were_placed = True

    perf_monitor.stop('ball_render')

    # Render floating score digits above goal pits (always visible, not just in ball mode)
    clear_score_digits()

    # Update delay timers - when they hit zero, trigger animation, explosion, and score increment
    if blue_score_delay_timer > 0:
        blue_score_delay_timer -= 1.0 / 60.0
        if blue_score_delay_timer <= 0:
            blue_score_delay_timer = 0
            if blue_score_pending:
                blue_score += 1  # Now increment the score
                blue_score_pending = False
                print(f"Blue {blue_score} - {red_score} Red")
            blue_score_bounce_timer = SCORE_BOUNCE_DURATION  # Start bounce animation
            # Start burst timer to spawn particles over time
            blue_burst_timer = SCORE_BURST_DURATION
            blue_burst_spawned = 0

    if red_score_delay_timer > 0:
        red_score_delay_timer -= 1.0 / 60.0
        if red_score_delay_timer <= 0:
            red_score_delay_timer = 0
            if red_score_pending:
                red_score += 1  # Now increment the score
                red_score_pending = False
                print(f"Blue {blue_score} - {red_score} Red")
            red_score_bounce_timer = SCORE_BOUNCE_DURATION  # Start bounce animation
            # Start burst timer to spawn particles over time
            red_burst_timer = SCORE_BURST_DURATION
            red_burst_spawned = 0

    # Update bounce timers
    if blue_score_bounce_timer > 0:
        blue_score_bounce_timer -= 1.0 / 60.0  # Approximate frame time
        if blue_score_bounce_timer < 0:
            blue_score_bounce_timer = 0

    if red_score_bounce_timer > 0:
        red_score_bounce_timer -= 1.0 / 60.0
        if red_score_bounce_timer < 0:
            red_score_bounce_timer = 0

    # Spawn burst particles over time for blue
    if blue_burst_timer > 0:
        blue_burst_timer -= 1.0 / 60.0
        # Calculate how many particles should be spawned by now
        progress = 1.0 - (blue_burst_timer / SCORE_BURST_DURATION)
        target_spawned = int(progress * SCORE_BURST_PARTICLES)
        particles_to_spawn = target_spawned - blue_burst_spawned
        if particles_to_spawn > 0:
            b_body = window.blue_body_color
            b_leg = window.blue_leg_color
            b_stripe = window.blue_stripe_color
            b_tip = window.blue_horn_tip_color
            spawn_score_burst(32.0, 58.0, 0.0,
                              b_body[0], b_body[1], b_body[2],
                              b_leg[0], b_leg[1], b_leg[2],
                              b_stripe[0], b_stripe[1], b_stripe[2],
                              b_tip[0], b_tip[1], b_tip[2],
                              particles_to_spawn, blue_burst_spawned, SCORE_BURST_PARTICLES)
            blue_burst_spawned = target_spawned
        if blue_burst_timer <= 0:
            blue_burst_timer = 0
            # Spawn any remaining particles
            remaining = SCORE_BURST_PARTICLES - blue_burst_spawned
            if remaining > 0:
                b_body = window.blue_body_color
                b_leg = window.blue_leg_color
                b_stripe = window.blue_stripe_color
                b_tip = window.blue_horn_tip_color
                spawn_score_burst(32.0, 58.0, 0.0,
                                  b_body[0], b_body[1], b_body[2],
                                  b_leg[0], b_leg[1], b_leg[2],
                                  b_stripe[0], b_stripe[1], b_stripe[2],
                                  b_tip[0], b_tip[1], b_tip[2],
                                  remaining, blue_burst_spawned, SCORE_BURST_PARTICLES)

    # Spawn burst particles over time for red
    if red_burst_timer > 0:
        red_burst_timer -= 1.0 / 60.0
        # Calculate how many particles should be spawned by now
        progress = 1.0 - (red_burst_timer / SCORE_BURST_DURATION)
        target_spawned = int(progress * SCORE_BURST_PARTICLES)
        particles_to_spawn = target_spawned - red_burst_spawned
        if particles_to_spawn > 0:
            r_body = window.red_body_color
            r_leg = window.red_leg_color
            r_stripe = window.red_stripe_color
            r_tip = window.red_horn_tip_color
            spawn_score_burst(-32.0, 58.0, 0.0,
                              r_body[0], r_body[1], r_body[2],
                              r_leg[0], r_leg[1], r_leg[2],
                              r_stripe[0], r_stripe[1], r_stripe[2],
                              r_tip[0], r_tip[1], r_tip[2],
                              particles_to_spawn, red_burst_spawned, SCORE_BURST_PARTICLES)
            red_burst_spawned = target_spawned
        if red_burst_timer <= 0:
            red_burst_timer = 0
            # Spawn any remaining particles
            remaining = SCORE_BURST_PARTICLES - red_burst_spawned
            if remaining > 0:
                r_body = window.red_body_color
                r_leg = window.red_leg_color
                r_stripe = window.red_stripe_color
                r_tip = window.red_horn_tip_color
                spawn_score_burst(-32.0, 58.0, 0.0,
                                  r_body[0], r_body[1], r_body[2],
                                  r_leg[0], r_leg[1], r_leg[2],
                                  r_stripe[0], r_stripe[1], r_stripe[2],
                                  r_tip[0], r_tip[1], r_tip[2],
                                  remaining, red_burst_spawned, SCORE_BURST_PARTICLES)

    # Pop & Squash animation using damped spring physics
    # Phases: 1) Initial squash (flat & wide), 2) Spring up tall & thin, 3) Oscillate and settle
    def get_pop_squash_scales(timer):
        if timer <= 0:
            return 1.0, 1.0  # scale_x, scale_y

        # Normalize timer: 1.0 at start, 0.0 at end
        t = 1.0 - (timer / SCORE_BOUNCE_DURATION)  # t goes 0 -> 1 as animation progresses

        # Damped spring parameters - DRAMATIC BUT SMOOTH
        frequency = 4.0  # Fewer oscillations for smoother feel
        damping = 2.2    # Moderate damping for smooth settling

        # Initial squash phase (first 12% of animation) - smooth squash
        if t < 0.12:
            # Squash down: very wide and flat
            squash_t = t / 0.12  # 0 -> 1 during squash phase
            scale_y = 1.0 - 0.88 * math.sin(squash_t * math.pi * 0.5)  # Squash to 0.12
            scale_x = 1.0 + 1.6 * math.sin(squash_t * math.pi * 0.5)   # Widen to 2.6
        else:
            # Spring oscillation phase (remaining 88%)
            spring_t = (t - 0.12) / 0.88  # Normalize to 0 -> 1

            # Damped oscillation: exp decay * sin wave
            decay = math.exp(-damping * spring_t)
            oscillation = math.sin(frequency * math.pi * spring_t)

            # Scale Y: starts stretched very tall, oscillates and settles to 1.0
            # First peak is tall (positive), then oscillates
            scale_y = 1.0 + 2.5 * decay * oscillation  # Stretch up to 3.5x tall!

            # Scale X: inverse of Y for volume preservation (squash & stretch principle)
            # When tall, get narrower; when short, get wider
            scale_x = 1.0 - 0.75 * decay * oscillation  # Narrow to 0.25x when tall

        # Clamp to reasonable values (increased max for more dramatic effect)
        scale_x = max(0.08, min(3.5, scale_x))
        scale_y = max(0.08, min(3.5, scale_y))

        return scale_x, scale_y

    blue_scale_x, blue_scale_y = get_pop_squash_scales(blue_score_bounce_timer)
    red_scale_x, red_scale_y = get_pop_squash_scales(red_score_bounce_timer)

    # Calculate flash brightness (bright at start, fades quickly)
    def get_flash_brightness(timer):
        if timer <= 0:
            return 1.0  # Normal brightness
        # Flash is brightest at start, fades over first 30% of animation
        t = timer / SCORE_BOUNCE_DURATION  # 1.0 at start, 0.0 at end
        if t > 0.7:  # First 30% of animation (timer goes from 1.0 to 0.0)
            flash_t = (t - 0.7) / 0.3  # 1.0 at start, 0.0 at 30%
            return 1.0 + 1.5 * flash_t  # Up to 2.5x brightness
        return 1.0

    simulation.blue_score_flash[None] = get_flash_brightness(blue_score_bounce_timer)
    simulation.red_score_flash[None] = get_flash_brightness(red_score_bounce_timer)

    # Digit positions in grid coords:
    # Blue score hovers above RED goal pit (east, x=96)
    # Red score hovers above BLUE goal pit (west, x=32)
    digit_y = 53  # Height above floor (floor is at y=33)

    # Blue score digit above red goal (east)
    blue_digit_x = 96.0  # Center over red goal pit
    render_score_digit(blue_score % 10, blue_digit_x, float(digit_y), 64.0,
                      simulation.SCORE_DIGIT_BLUE,
                      blue_scale_x, blue_scale_y, camera.pos_x, camera.pos_z)

    # Red score digit above blue goal (west)
    red_digit_x = 32.0  # Center over blue goal pit
    render_score_digit(red_score % 10, red_digit_x, float(digit_y), 64.0,
                      simulation.SCORE_DIGIT_RED,
                      red_scale_x, red_scale_y, camera.pos_x, camera.pos_z)

    # === SCENE RENDER TIMING ===
    perf_monitor.start('scene_render')

    # Calculate spotlight position above beetles midpoint
    spotlight_mid_x = (blue_render_x + red_render_x) / 2.0
    spotlight_mid_y = max(blue_render_y, red_render_y) + spotlight_height  # Height above highest beetle (higher = bigger/softer, lower = smaller/focused)
    spotlight_mid_z = (blue_render_z + red_render_z) / 2.0

    canvas.set_background_color((0.18, 0.40, 0.22))  # Lighter forest green
    renderer.render(camera, canvas, scene, simulation.voxel_type, simulation.n_grid,
                    dynamic_lighting=dynamic_lighting_enabled,
                    spotlight_pos=(spotlight_mid_x, spotlight_mid_y, spotlight_mid_z),
                    spotlight_strength=spotlight_strength,
                    base_light_brightness=base_light_brightness,
                    front_light_strength=front_light_strength)
    canvas.scene(scene)

    perf_monitor.stop('scene_render')

    # === GUI TIMING ===
    perf_monitor.start('gui')

    # HUD
    window.GUI.begin("Beetle Physics", 0.01, 0.01, 0.35, 0.95)
    window.GUI.text(f"FPS: {actual_fps:.0f}")

    # === PERFORMANCE MONITORING DISPLAY ===
    if perf_monitor.show_stats:
        # Compact summary line
        total_ms = perf_monitor.get_avg('frame_total')
        physics_ms = perf_monitor.get_avg('physics')
        render_ms = perf_monitor.get_avg('beetle_render') + perf_monitor.get_avg('scene_render')
        window.GUI.text(f"Frame: {total_ms:.1f}ms | Phys: {physics_ms:.1f}ms | Rend: {render_ms:.1f}ms")

        # Detailed breakdown toggle
        if perf_monitor.show_detailed:
            for line in perf_monitor.get_detailed_breakdown():
                window.GUI.text(line)

            # Renderer sub-timing breakdown
            rt = renderer.get_render_timing()
            if rt:
                window.GUI.text("--- Renderer Breakdown (last frame) ---")
                window.GUI.text(f"  extract_voxels: {rt.get('extract_voxels', 0):.2f}ms")
                window.GUI.text(f"  extract_debris: {rt.get('extract_debris', 0):.2f}ms")
                window.GUI.text(f"  lighting_setup: {rt.get('lighting_setup', 0):.2f}ms")
                window.GUI.text(f"  scene_particles: {rt.get('scene_particles', 0):.2f}ms")
                window.GUI.text(f"  voxel_count: {rt.get('voxel_count', 0)}")

            # Physics sub-timing breakdown
            pt = get_physics_timing()
            if pt:
                window.GUI.text("--- Physics Breakdown (last frame) ---")
                window.GUI.text(f"  input_controls: {pt.get('input_controls', 0):.2f}ms")
                window.GUI.text(f"  beetle_physics: {pt.get('beetle_physics', 0):.2f}ms")
                window.GUI.text(f"  ball_physics: {pt.get('ball_physics', 0):.2f}ms")
                window.GUI.text(f"  debris_particles: {pt.get('debris_particles', 0):.2f}ms")
                window.GUI.text(f"  death_explosions: {pt.get('death_explosions', 0):.2f}ms")
                window.GUI.text(f"  respawn_timers: {pt.get('respawn_timers', 0):.2f}ms")
                window.GUI.text(f"  floor_collision: {pt.get('floor_collision', 0):.2f}ms")
                window.GUI.text(f"  beetle_collision: {pt.get('beetle_collision', 0):.2f}ms")

    # Performance display toggle buttons
    if window.GUI.button("Toggle Perf Stats"):
        perf_monitor.show_stats = not perf_monitor.show_stats
    if perf_monitor.show_stats:
        if window.GUI.button("Toggle Detailed"):
            perf_monitor.show_detailed = not perf_monitor.show_detailed
        if window.GUI.button("Save Perf Log"):
            with open("perf_log.txt", "w") as f:
                f.write(f"FPS: {actual_fps:.0f}\n")
                f.write(f"Frame count: {perf_monitor.frame_count}\n\n")
                for line in perf_monitor.get_detailed_breakdown():
                    f.write(line + "\n")
                # Add renderer breakdown
                rt = renderer.get_render_timing()
                if rt:
                    f.write("\n--- Renderer Breakdown (last frame) ---\n")
                    f.write(f"  extract_voxels: {rt.get('extract_voxels', 0):.2f}ms\n")
                    f.write(f"  extract_debris: {rt.get('extract_debris', 0):.2f}ms\n")
                    f.write(f"  lighting_setup: {rt.get('lighting_setup', 0):.2f}ms\n")
                    f.write(f"  scene_particles: {rt.get('scene_particles', 0):.2f}ms\n")
                    f.write(f"  voxel_count: {rt.get('voxel_count', 0)}\n")
                # Add physics breakdown
                pt = get_physics_timing()
                if pt:
                    f.write("\n--- Physics Breakdown (last frame) ---\n")
                    f.write(f"  input_controls: {pt.get('input_controls', 0):.2f}ms\n")
                    f.write(f"  beetle_physics: {pt.get('beetle_physics', 0):.2f}ms\n")
                    f.write(f"  ball_physics: {pt.get('ball_physics', 0):.2f}ms\n")
                    f.write(f"  debris_particles: {pt.get('debris_particles', 0):.2f}ms\n")
                    f.write(f"  death_explosions: {pt.get('death_explosions', 0):.2f}ms\n")
                    f.write(f"  respawn_timers: {pt.get('respawn_timers', 0):.2f}ms\n")
                    f.write(f"  floor_collision: {pt.get('floor_collision', 0):.2f}ms\n")
                    f.write(f"  beetle_collision: {pt.get('beetle_collision', 0):.2f}ms\n")
            print("Performance log saved to perf_log.txt")

    window.GUI.text("")

    # Blue beetle status
    if beetle_blue.active:
        window.GUI.text("BLUE BEETLE (TFGH + RY + VB)")
        window.GUI.text(f"  Pos: ({beetle_blue.x:.1f}, {beetle_blue.y:.1f}, {beetle_blue.z:.1f})")
        window.GUI.text(f"  Speed: {math.sqrt(beetle_blue.vx**2 + beetle_blue.vz**2):.1f}")
        window.GUI.text(f"  Facing: {math.degrees(beetle_blue.rotation):.0f}°")
        window.GUI.text(f"  Horn pitch: {math.degrees(beetle_blue.horn_pitch):.1f}°")
        window.GUI.text(f"  Horn yaw: {math.degrees(beetle_blue.horn_yaw):.1f}°")
    else:
        window.GUI.text("BLUE BEETLE: FALLEN")

    window.GUI.text("")

    # Red beetle status
    if beetle_red.active:
        window.GUI.text("RED BEETLE (IJKL + UO + NM)")
        window.GUI.text(f"  Pos: ({beetle_red.x:.1f}, {beetle_red.y:.1f}, {beetle_red.z:.1f})")
        window.GUI.text(f"  Speed: {math.sqrt(beetle_red.vx**2 + beetle_red.vz**2):.1f}")
        window.GUI.text(f"  Facing: {math.degrees(beetle_red.rotation):.0f}°")
        window.GUI.text(f"  Horn pitch: {math.degrees(beetle_red.horn_pitch):.1f}°")
        window.GUI.text(f"  Horn yaw: {math.degrees(beetle_red.horn_yaw):.1f}°")
    else:
        window.GUI.text("RED BEETLE: FALLEN")
    window.GUI.text("")

    # Distance display - only if both beetles are active
    if beetle_blue.active and beetle_red.active:
        dx = beetle_blue.x - beetle_red.x
        dz = beetle_blue.z - beetle_red.z
        distance = math.sqrt(dx**2 + dz**2)
        window.GUI.text(f"Distance: {distance:.1f}")

    # Active beetles count
    active_count = (1 if beetle_blue.active else 0) + (1 if beetle_red.active else 0)
    window.GUI.text(f"Active beetles: {active_count}/2")

    window.GUI.text("")
    window.GUI.text("=== BLUE BEETLE GENETICS ===")

    # Front body (thorax) is fixed at 4 layers
    front_body_height = 4

    # Blue beetle sliders
    new_blue_shaft = window.GUI.slider_int("Blue Horn Shaft", window.blue_horn_shaft_value, 8, 15)
    new_blue_prong = window.GUI.slider_int("Blue Horn Prong", window.blue_horn_prong_value, 3, 6)
    new_blue_back_body = window.GUI.slider_int("Blue Back Body", window.blue_back_body_height_value, 4, 8)
    new_blue_body_length = window.GUI.slider_int("Blue Body Length", window.blue_body_length_value, 9, 14)
    new_blue_body_width = window.GUI.slider_int("Blue Body Width", window.blue_body_width_value, 5, 9)
    new_blue_leg_length = window.GUI.slider_int("Blue Leg Length", window.blue_leg_length_value, 6, 10)

    # Random blue beetle button
    if window.GUI.button("Randomize Blue Beetle"):
        new_blue_shaft = random.randint(8, 15)
        new_blue_prong = random.randint(3, 6)
        new_blue_back_body = random.randint(4, 8)
        new_blue_body_length = random.randint(9, 14)
        new_blue_body_width = random.randint(5, 9)
        new_blue_leg_length = random.randint(6, 10)
        print(f"Randomized blue beetle: shaft={new_blue_shaft}, prong={new_blue_prong}, back={new_blue_back_body}, length={new_blue_body_length}, width={new_blue_body_width}, legs={new_blue_leg_length}")

    # Rebuild blue beetle geometry if sliders changed OR if scorpion tail curvature changed
    if (new_blue_shaft != window.blue_horn_shaft_value or new_blue_prong != window.blue_horn_prong_value or
        new_blue_back_body != window.blue_back_body_height_value or new_blue_body_length != window.blue_body_length_value or
        new_blue_body_width != window.blue_body_width_value or new_blue_leg_length != window.blue_leg_length_value or
        (blue_horn_type == "scorpion" and abs(beetle_blue.stinger_curvature - blue_previous_stinger_curvature) > 0.01)):

        # For scorpion type, use blue beetle's current animation values
        blue_current_stinger_curvature = 0.0
        blue_current_tail_rotation = 0.0
        if blue_horn_type == "scorpion":
            blue_current_stinger_curvature = beetle_blue.stinger_curvature
            blue_current_tail_rotation = beetle_blue.tail_rotation_angle

        # Rebuild only blue beetle
        rebuild_blue_beetle(new_blue_shaft, new_blue_prong, front_body_height, new_blue_back_body, new_blue_body_length, new_blue_body_width, new_blue_leg_length, blue_horn_type, blue_current_stinger_curvature, blue_current_tail_rotation)
        ti.sync()
        window.blue_horn_shaft_value = new_blue_shaft
        window.blue_horn_prong_value = new_blue_prong
        window.blue_back_body_height_value = new_blue_back_body
        window.blue_body_length_value = new_blue_body_length
        window.blue_body_width_value = new_blue_body_width
        window.blue_leg_length_value = new_blue_leg_length
        blue_previous_stinger_curvature = blue_current_stinger_curvature
        blue_previous_tail_rotation = blue_current_tail_rotation

    # Blue beetle stats
    window.GUI.text(f"Blue Shaft: {window.blue_horn_shaft_value} voxels")
    window.GUI.text(f"Blue Prong: {window.blue_horn_prong_value} voxels")
    blue_total_reach = window.blue_horn_shaft_value + window.blue_horn_prong_value
    window.GUI.text(f"Blue Total Horn: {blue_total_reach} voxels")

    # Blue beetle horn type button
    window.GUI.text("")
    if blue_horn_type == "rhino":
        blue_button_text = "Blue: RHINO (click for STAG)"
    elif blue_horn_type == "stag":
        blue_button_text = "Blue: STAG (click for HERCULES)"
    elif blue_horn_type == "hercules":
        blue_button_text = "Blue: HERCULES (click for SCORPION)"
    elif blue_horn_type == "scorpion":
        blue_button_text = "Blue: SCORPION (click for ATLAS)"
    else:  # atlas
        blue_button_text = "Blue: ATLAS (click for RHINO)"

    if window.GUI.button(blue_button_text):
        # Cycle blue beetle horn type
        if blue_horn_type == "rhino":
            blue_horn_type = "stag"
        elif blue_horn_type == "stag":
            blue_horn_type = "hercules"
        elif blue_horn_type == "hercules":
            blue_horn_type = "scorpion"
        elif blue_horn_type == "scorpion":
            blue_horn_type = "atlas"
        else:
            blue_horn_type = "rhino"

        print(f"Blue beetle switching to {blue_horn_type.upper()}...")

        # Update blue beetle horn pitch and yaw defaults
        if blue_horn_type == "scorpion":
            beetle_blue.horn_pitch = HORN_DEFAULT_PITCH_SCORPION
            beetle_blue.prev_horn_pitch = HORN_DEFAULT_PITCH_SCORPION
            beetle_blue.horn_yaw = 0.0
        elif blue_horn_type == "stag":
            beetle_blue.horn_pitch = HORN_DEFAULT_PITCH_STAG
            beetle_blue.prev_horn_pitch = HORN_DEFAULT_PITCH_STAG
            beetle_blue.horn_yaw = HORN_DEFAULT_YAW_STAG  # Stag pincers start more open
        elif blue_horn_type == "hercules":
            beetle_blue.horn_pitch = HORN_DEFAULT_PITCH_HERCULES
            beetle_blue.prev_horn_pitch = HORN_DEFAULT_PITCH_HERCULES
            beetle_blue.horn_yaw = 0.0
        elif blue_horn_type == "atlas":
            beetle_blue.horn_pitch = HORN_DEFAULT_PITCH_ATLAS
            beetle_blue.prev_horn_pitch = HORN_DEFAULT_PITCH_ATLAS
            beetle_blue.horn_yaw = 0.0
        else:  # rhino
            beetle_blue.horn_pitch = HORN_DEFAULT_PITCH
            beetle_blue.prev_horn_pitch = HORN_DEFAULT_PITCH
            beetle_blue.horn_yaw = 0.0

        # Sync GPU before rebuild to prevent Vulkan device loss during geometry changes
        ti.sync()

        # Rebuild blue beetle with new horn type
        rebuild_blue_beetle(
            window.blue_horn_shaft_value,
            window.blue_horn_prong_value,
            front_body_height,
            window.blue_back_body_height_value,
            window.blue_body_length_value,
            window.blue_body_width_value,
            window.blue_leg_length_value,
            blue_horn_type,
            stinger_curvature=0.0
        )
        ti.sync()

    # Blue beetle color pickers
    window.GUI.text("")
    window.GUI.text("=== BLUE BEETLE COLORS ===")

    new_blue_body_color = window.GUI.color_edit_3("Blue Body", window.blue_body_color)
    if new_blue_body_color != window.blue_body_color:
        window.blue_body_color = new_blue_body_color
        simulation.blue_body_color[None] = ti.Vector([new_blue_body_color[0], new_blue_body_color[1], new_blue_body_color[2]])

    new_blue_leg_color = window.GUI.color_edit_3("Blue Legs", window.blue_leg_color)
    if new_blue_leg_color != window.blue_leg_color:
        window.blue_leg_color = new_blue_leg_color
        simulation.blue_leg_color[None] = ti.Vector([new_blue_leg_color[0], new_blue_leg_color[1], new_blue_leg_color[2]])

    new_blue_leg_tip_color = window.GUI.color_edit_3("Blue Leg Tips", window.blue_leg_tip_color)
    if new_blue_leg_tip_color != window.blue_leg_tip_color:
        window.blue_leg_tip_color = new_blue_leg_tip_color
        simulation.blue_leg_tip_color[None] = ti.Vector([new_blue_leg_tip_color[0], new_blue_leg_tip_color[1], new_blue_leg_tip_color[2]])

    new_blue_stripe_color = window.GUI.color_edit_3("Blue Stripe", window.blue_stripe_color)
    if new_blue_stripe_color != window.blue_stripe_color:
        window.blue_stripe_color = new_blue_stripe_color
        simulation.blue_stripe_color[None] = ti.Vector([new_blue_stripe_color[0], new_blue_stripe_color[1], new_blue_stripe_color[2]])

    new_blue_horn_tip_color = window.GUI.color_edit_3("Blue Horn Tips", window.blue_horn_tip_color)
    if new_blue_horn_tip_color != window.blue_horn_tip_color:
        window.blue_horn_tip_color = new_blue_horn_tip_color
        simulation.blue_horn_tip_color[None] = ti.Vector([new_blue_horn_tip_color[0], new_blue_horn_tip_color[1], new_blue_horn_tip_color[2]])

    window.GUI.text("")
    window.GUI.text("=== RED BEETLE GENETICS ===")

    # Red beetle sliders
    new_red_shaft = window.GUI.slider_int("Red Horn Shaft", window.red_horn_shaft_value, 8, 15)
    new_red_prong = window.GUI.slider_int("Red Horn Prong", window.red_horn_prong_value, 3, 6)
    new_red_back_body = window.GUI.slider_int("Red Back Body", window.red_back_body_height_value, 4, 8)
    new_red_body_length = window.GUI.slider_int("Red Body Length", window.red_body_length_value, 9, 14)
    new_red_body_width = window.GUI.slider_int("Red Body Width", window.red_body_width_value, 5, 9)
    new_red_leg_length = window.GUI.slider_int("Red Leg Length", window.red_leg_length_value, 6, 10)

    # Random red beetle button
    if window.GUI.button("Randomize Red Beetle"):
        new_red_shaft = random.randint(8, 15)
        new_red_prong = random.randint(3, 6)
        new_red_back_body = random.randint(4, 8)
        new_red_body_length = random.randint(9, 14)
        new_red_body_width = random.randint(5, 9)
        new_red_leg_length = random.randint(6, 10)
        print(f"Randomized red beetle: shaft={new_red_shaft}, prong={new_red_prong}, back={new_red_back_body}, length={new_red_body_length}, width={new_red_body_width}, legs={new_red_leg_length}")

    # Rebuild red beetle geometry if sliders changed OR if scorpion tail curvature changed
    if (new_red_shaft != window.red_horn_shaft_value or new_red_prong != window.red_horn_prong_value or
        new_red_back_body != window.red_back_body_height_value or new_red_body_length != window.red_body_length_value or
        new_red_body_width != window.red_body_width_value or new_red_leg_length != window.red_leg_length_value or
        (red_horn_type == "scorpion" and abs(beetle_red.stinger_curvature - red_previous_stinger_curvature) > 0.01)):

        # For scorpion type, use red beetle's current animation values
        red_current_stinger_curvature = 0.0
        red_current_tail_rotation = 0.0
        if red_horn_type == "scorpion":
            red_current_stinger_curvature = beetle_red.stinger_curvature
            red_current_tail_rotation = beetle_red.tail_rotation_angle

        # Rebuild only red beetle
        rebuild_red_beetle(new_red_shaft, new_red_prong, front_body_height, new_red_back_body, new_red_body_length, new_red_body_width, new_red_leg_length, red_horn_type, red_current_stinger_curvature, red_current_tail_rotation)
        ti.sync()
        window.red_horn_shaft_value = new_red_shaft
        window.red_horn_prong_value = new_red_prong
        window.red_back_body_height_value = new_red_back_body
        window.red_body_length_value = new_red_body_length
        window.red_body_width_value = new_red_body_width
        window.red_leg_length_value = new_red_leg_length
        red_previous_stinger_curvature = red_current_stinger_curvature
        red_previous_tail_rotation = red_current_tail_rotation

    # Red beetle stats
    window.GUI.text(f"Red Shaft: {window.red_horn_shaft_value} voxels")
    window.GUI.text(f"Red Prong: {window.red_horn_prong_value} voxels")
    red_total_reach = window.red_horn_shaft_value + window.red_horn_prong_value
    window.GUI.text(f"Red Total Horn: {red_total_reach} voxels")

    # Red beetle horn type button
    window.GUI.text("")
    if red_horn_type == "rhino":
        red_button_text = "Red: RHINO (click for STAG)"
    elif red_horn_type == "stag":
        red_button_text = "Red: STAG (click for HERCULES)"
    elif red_horn_type == "hercules":
        red_button_text = "Red: HERCULES (click for SCORPION)"
    elif red_horn_type == "scorpion":
        red_button_text = "Red: SCORPION (click for ATLAS)"
    else:  # atlas
        red_button_text = "Red: ATLAS (click for RHINO)"

    if window.GUI.button(red_button_text):
        # Cycle red beetle horn type
        if red_horn_type == "rhino":
            red_horn_type = "stag"
        elif red_horn_type == "stag":
            red_horn_type = "hercules"
        elif red_horn_type == "hercules":
            red_horn_type = "scorpion"
        elif red_horn_type == "scorpion":
            red_horn_type = "atlas"
        else:
            red_horn_type = "rhino"

        print(f"Red beetle switching to {red_horn_type.upper()}...")

        # Update red beetle horn pitch and yaw defaults
        if red_horn_type == "scorpion":
            beetle_red.horn_pitch = HORN_DEFAULT_PITCH_SCORPION
            beetle_red.prev_horn_pitch = HORN_DEFAULT_PITCH_SCORPION
            beetle_red.horn_yaw = 0.0
        elif red_horn_type == "stag":
            beetle_red.horn_pitch = HORN_DEFAULT_PITCH_STAG
            beetle_red.prev_horn_pitch = HORN_DEFAULT_PITCH_STAG
            beetle_red.horn_yaw = HORN_DEFAULT_YAW_STAG  # Stag pincers start more open
        elif red_horn_type == "hercules":
            beetle_red.horn_pitch = HORN_DEFAULT_PITCH_HERCULES
            beetle_red.prev_horn_pitch = HORN_DEFAULT_PITCH_HERCULES
            beetle_red.horn_yaw = 0.0
        elif red_horn_type == "atlas":
            beetle_red.horn_pitch = HORN_DEFAULT_PITCH_ATLAS
            beetle_red.prev_horn_pitch = HORN_DEFAULT_PITCH_ATLAS
            beetle_red.horn_yaw = 0.0
        else:  # rhino
            beetle_red.horn_pitch = HORN_DEFAULT_PITCH
            beetle_red.prev_horn_pitch = HORN_DEFAULT_PITCH
            beetle_red.horn_yaw = 0.0

        # Sync GPU before rebuild to prevent Vulkan device loss during geometry changes
        ti.sync()

        # Rebuild red beetle with new horn type
        rebuild_red_beetle(
            window.red_horn_shaft_value,
            window.red_horn_prong_value,
            front_body_height,
            window.red_back_body_height_value,
            window.red_body_length_value,
            window.red_body_width_value,
            window.red_leg_length_value,
            red_horn_type,
            stinger_curvature=0.0
        )
        ti.sync()

    # Red beetle color pickers
    window.GUI.text("")
    window.GUI.text("=== RED BEETLE COLORS ===")

    new_red_body_color = window.GUI.color_edit_3("Red Body", window.red_body_color)
    if new_red_body_color != window.red_body_color:
        window.red_body_color = new_red_body_color
        simulation.red_body_color[None] = ti.Vector([new_red_body_color[0], new_red_body_color[1], new_red_body_color[2]])

    new_red_leg_color = window.GUI.color_edit_3("Red Legs", window.red_leg_color)
    if new_red_leg_color != window.red_leg_color:
        window.red_leg_color = new_red_leg_color
        simulation.red_leg_color[None] = ti.Vector([new_red_leg_color[0], new_red_leg_color[1], new_red_leg_color[2]])

    new_red_leg_tip_color = window.GUI.color_edit_3("Red Leg Tips", window.red_leg_tip_color)
    if new_red_leg_tip_color != window.red_leg_tip_color:
        window.red_leg_tip_color = new_red_leg_tip_color
        simulation.red_leg_tip_color[None] = ti.Vector([new_red_leg_tip_color[0], new_red_leg_tip_color[1], new_red_leg_tip_color[2]])

    new_red_stripe_color = window.GUI.color_edit_3("Red Stripe", window.red_stripe_color)
    if new_red_stripe_color != window.red_stripe_color:
        window.red_stripe_color = new_red_stripe_color
        simulation.red_stripe_color[None] = ti.Vector([new_red_stripe_color[0], new_red_stripe_color[1], new_red_stripe_color[2]])

    new_red_horn_tip_color = window.GUI.color_edit_3("Red Horn Tips", window.red_horn_tip_color)
    if new_red_horn_tip_color != window.red_horn_tip_color:
        window.red_horn_tip_color = new_red_horn_tip_color
        simulation.red_horn_tip_color[None] = ti.Vector([new_red_horn_tip_color[0], new_red_horn_tip_color[1], new_red_horn_tip_color[2]])

    # Ball controls (beetle soccer)
    window.GUI.text("")
    window.GUI.text("=== BEETLE BALL (SOCCER MODE) ===")

    # Ball toggle button
    ball_button_text = "Disable Ball" if beetle_ball.active else "Enable Ball"
    if window.GUI.button(ball_button_text):
        if beetle_ball.active:
            # Disabling ball - clear voxels and bowl perimeter (use fast clear if ball was rendered)
            if ball_last_rendered[None] == 1:
                num_voxels = ball_cache_size[None]
                if num_voxels > 0:
                    clear_ball_fast(ball_last_grid_x[None], ball_last_grid_y[None], ball_last_grid_z[None], num_voxels)
                ball_last_rendered[None] = 0
            else:
                clear_ball()  # Fallback to full clear if ball position unknown
            simulation.clear_bowl_perimeter()
            # Rebuild floor height cache without the ice bowl
            build_floor_height_cache()
            # Reset scores when leaving ball mode
            blue_score = 0
            red_score = 0
        beetle_ball.active = not beetle_ball.active
        if beetle_ball.active:
            # Initialize ball cache for assembly animation if not already done
            if not ball_cache_initialized:
                init_ball_cache(beetle_ball.radius)
                ball_cache_initialized = True
            # Reset ball to center when enabling
            beetle_ball.x = 0.0
            beetle_ball.y = 28.0  # Drop from higher than beetles
            beetle_ball.z = 0.0
            beetle_ball.vx = 0.0
            beetle_ball.vy = 0.0
            beetle_ball.vz = 0.0
            beetle_ball.rotation = 0.0
            beetle_ball.angular_velocity = 0.0
            beetle_ball.pitch = 0.0
            beetle_ball.pitch_velocity = 0.0
            beetle_ball.roll = 0.0
            beetle_ball.roll_velocity = 0.0
            # Reset prev state to avoid interpolation jump
            beetle_ball.prev_x = beetle_ball.x
            beetle_ball.prev_y = beetle_ball.y
            beetle_ball.prev_z = beetle_ball.z
            beetle_ball.prev_rotation = beetle_ball.rotation
            beetle_ball.prev_pitch = beetle_ball.pitch
            beetle_ball.prev_roll = beetle_ball.roll
            # Reset scores and flags
            blue_score = 0
            red_score = 0
            ball_scored_this_fall = False
            ball_has_exploded = False
            ball_explosion_timer = 0.0
            # Render the bowl perimeter for ball mode (with goal pit cutouts)
            simulation.render_bowl_perimeter()
            # Rebuild floor height cache to include the ice bowl
            build_floor_height_cache()

    # Ball radius slider (only show when ball is enabled)
    if beetle_ball.active:
        # Display score
        window.GUI.text(f"SCORE: Blue {blue_score} - {red_score} Red")
        window.GUI.text("")
        new_ball_radius = window.GUI.slider_int("Ball Radius", int(beetle_ball.radius), 3, 10)
        if new_ball_radius != int(beetle_ball.radius):
            beetle_ball.radius = float(new_ball_radius)
            # Reinitialize ball cache when radius changes
            init_ball_cache(beetle_ball.radius)
        window.GUI.text(f"Ball Position: ({beetle_ball.x:.1f}, {beetle_ball.y:.1f}, {beetle_ball.z:.1f})")
        window.GUI.text(f"Ball Velocity: ({beetle_ball.vx:.1f}, {beetle_ball.vy:.1f}, {beetle_ball.vz:.1f})")

        # Ball physics tuning sliders
        window.GUI.text("")
        window.GUI.text("--- Ball Physics Tuning ---")

        # Separation Force (how hard ball pushes away from beetles)
        new_separation = window.GUI.slider_float("Separation Force", physics_params["BALL_SEPARATION_FORCE"], 0.1, 2.0)
        if new_separation != physics_params["BALL_SEPARATION_FORCE"]:
            physics_params["BALL_SEPARATION_FORCE"] = new_separation

        # Momentum Transfer (how much beetle velocity transfers to ball)
        new_momentum = window.GUI.slider_float("Momentum Transfer", physics_params["BALL_MOMENTUM_TRANSFER"], 0.0, 2.0)
        if new_momentum != physics_params["BALL_MOMENTUM_TRANSFER"]:
            physics_params["BALL_MOMENTUM_TRANSFER"] = new_momentum

        # Restitution (bounciness in collisions)
        new_restitution = window.GUI.slider_float("Restitution (Bounce)", physics_params["BALL_RESTITUTION"], 0.0, 1.0)
        if new_restitution != physics_params["BALL_RESTITUTION"]:
            physics_params["BALL_RESTITUTION"] = new_restitution

        # Rolling Friction (horizontal slowdown)
        new_friction = window.GUI.slider_float("Rolling Friction", physics_params["BALL_ROLLING_FRICTION"], 0.80, 0.99)
        if new_friction != physics_params["BALL_ROLLING_FRICTION"]:
            physics_params["BALL_ROLLING_FRICTION"] = new_friction

        # Ground Bounce (floor bounce coefficient)
        new_ground_bounce = window.GUI.slider_float("Ground Bounce", physics_params["BALL_GROUND_BOUNCE"], 0.0, 0.8)
        if new_ground_bounce != physics_params["BALL_GROUND_BOUNCE"]:
            physics_params["BALL_GROUND_BOUNCE"] = new_ground_bounce

        # Ball contact physics (torque/lift/tip based on hit location)
        window.GUI.text("")
        window.GUI.text("--- Ball Contact Physics ---")

        # Lift Strength (upward force when scooping with horn)
        new_lift = window.GUI.slider_float("Scoop Lift", physics_params["BALL_LIFT_STRENGTH"], 0.0, 10.0)
        if new_lift != physics_params["BALL_LIFT_STRENGTH"]:
            physics_params["BALL_LIFT_STRENGTH"] = new_lift

        # Passive Lift Strength (automatic lift when touching bottom of ball)
        new_passive_lift = window.GUI.slider_float("Passive Lift", physics_params["BALL_PASSIVE_LIFT_STRENGTH"], 0.0, 10.0)
        if new_passive_lift != physics_params["BALL_PASSIVE_LIFT_STRENGTH"]:
            physics_params["BALL_PASSIVE_LIFT_STRENGTH"] = new_passive_lift

        # Tip Strength (downward force when hit from above)
        new_tip = window.GUI.slider_float("Tip Strength", physics_params["BALL_TIP_STRENGTH"], 0.0, 5.0)
        if new_tip != physics_params["BALL_TIP_STRENGTH"]:
            physics_params["BALL_TIP_STRENGTH"] = new_tip

        # Torque Strength (spin from side hits)
        new_torque = window.GUI.slider_float("Torque Strength", physics_params["BALL_TORQUE_STRENGTH"], 0.0, 10.0)
        if new_torque != physics_params["BALL_TORQUE_STRENGTH"]:
            physics_params["BALL_TORQUE_STRENGTH"] = new_torque

        # Gravity Multiplier (how fast ball falls)
        new_grav = window.GUI.slider_float("Gravity Multiplier", physics_params["BALL_GRAVITY_MULTIPLIER"], 1.0, 5.0)
        if new_grav != physics_params["BALL_GRAVITY_MULTIPLIER"]:
            physics_params["BALL_GRAVITY_MULTIPLIER"] = new_grav

        window.GUI.text("")
        window.GUI.text("--- Ball Feel (Mass/Spin) ---")

        # Push Multiplier (how easily beetles can push the ball)
        new_push = window.GUI.slider_float("Push Ease", physics_params["BALL_PUSH_MULTIPLIER"], 0.5, 4.0)
        if new_push != physics_params["BALL_PUSH_MULTIPLIER"]:
            physics_params["BALL_PUSH_MULTIPLIER"] = new_push

        # Spin Multiplier (how easily ball spins when hit)
        new_spin = window.GUI.slider_float("Spin Ease", physics_params["BALL_SPIN_MULTIPLIER"], 0.5, 5.0)
        if new_spin != physics_params["BALL_SPIN_MULTIPLIER"]:
            physics_params["BALL_SPIN_MULTIPLIER"] = new_spin

        # Angular Friction (how quickly ball spin slows down)
        new_ang_fric = window.GUI.slider_float("Spin Retain", physics_params["BALL_ANGULAR_FRICTION"], 0.90, 0.995)
        if new_ang_fric != physics_params["BALL_ANGULAR_FRICTION"]:
            physics_params["BALL_ANGULAR_FRICTION"] = new_ang_fric

    # Winner announcement and restart button
    if match_winner is not None:
        window.GUI.text("")
        window.GUI.text("="*30)
        if match_winner == "BLUE":
            window.GUI.text("*** BLUE BEETLE WINS! ***")
        else:
            window.GUI.text("*** RED BEETLE WINS! ***")
        window.GUI.text("="*30)
        window.GUI.text("")
        if window.GUI.button("RESTART MATCH"):
            reset_match()

    # Physics parameter sliders (commented out - can re-enable later if needed)
    # window.GUI.text("")
    window.GUI.text("=== PHYSICS TUNING ===")
    physics_params["GRAVITY"] = window.GUI.slider_float("Gravity", physics_params["GRAVITY"], 0.5, 40.0)
    physics_params["TORQUE_MULTIPLIER"] = window.GUI.slider_float("Torque", physics_params["TORQUE_MULTIPLIER"], 0.0, 4.0)
    physics_params["IMPULSE_MULTIPLIER"] = window.GUI.slider_float("Impulse", physics_params["IMPULSE_MULTIPLIER"], 0.0, 1.0)
    physics_params["SEPARATION_FORCE"] = window.GUI.slider_float("Separation", physics_params["SEPARATION_FORCE"], 0.0, 1.0)
    physics_params["RESTITUTION"] = window.GUI.slider_float("Bounce", physics_params["RESTITUTION"], 0.0, 0.5)
    new_inertia_factor = window.GUI.slider_float("Inertia", physics_params["MOMENT_OF_INERTIA_FACTOR"], 0.1, 5.0)

    window.GUI.text("--- Airborne Tumbling ---")
    physics_params["AIRBORNE_DAMPING"] = window.GUI.slider_float("Air Damping", physics_params["AIRBORNE_DAMPING"], 0.2, 0.99)
    physics_params["AIRBORNE_TILT_SPEED"] = window.GUI.slider_float("Air Tilt Speed", physics_params["AIRBORNE_TILT_SPEED"], 8.0, 1000.0)
    physics_params["GROUND_TILT_ANGLE"] = window.GUI.slider_float("Ground Tilt Max", physics_params["GROUND_TILT_ANGLE"], 30.0, 300.0)
    physics_params["TUMBLE_MULTIPLIER"] = window.GUI.slider_float("Tumble Multiplier", physics_params["TUMBLE_MULTIPLIER"], 1.0, 5.0)
    physics_params["RESTORING_STRENGTH"] = window.GUI.slider_float("Restoring (Settled)", physics_params["RESTORING_STRENGTH"], 5.0, 50.0)
    physics_params["WEAK_RESTORING"] = window.GUI.slider_float("Restoring (Bouncing)", physics_params["WEAK_RESTORING"], 5.0, 50.0)

    window.GUI.text("--- Auto-Follow Camera ---")
    physics_params["CAMERA_PITCH"] = window.GUI.slider_float("Camera Angle", physics_params["CAMERA_PITCH"], -90.0, -30.0)
    physics_params["CAMERA_BASE_HEIGHT"] = window.GUI.slider_float("Camera Height", physics_params["CAMERA_BASE_HEIGHT"], 20.0, 120.0)
    physics_params["CAMERA_DISTANCE"] = window.GUI.slider_float("Camera Distance", physics_params["CAMERA_DISTANCE"], 10.0, 80.0)
    spotlight_strength = window.GUI.slider_float("Spotlight Strength", spotlight_strength, 0.0, 1.5)
    spotlight_height = window.GUI.slider_float("Spotlight Size", spotlight_height, 10.0, 100.0)
    base_light_brightness = window.GUI.slider_float("Base Light Brightness", base_light_brightness, 0.0, 2.0)
    front_light_strength = window.GUI.slider_float("Front Light Strength", front_light_strength, 0.0, 1.5)
    # Camera always positions on edge side (beetles in foreground, edge in background)

    # Dynamic lighting toggle
    lighting_button_text = "Dynamic Lighting: ON" if dynamic_lighting_enabled else "Dynamic Lighting: OFF"
    if window.GUI.button(lighting_button_text):
        dynamic_lighting_enabled = not dynamic_lighting_enabled

    # Update beetle inertia if factor changed
    if abs(new_inertia_factor - physics_params["MOMENT_OF_INERTIA_FACTOR"]) > 0.001:
        physics_params["MOMENT_OF_INERTIA_FACTOR"] = new_inertia_factor
        beetle_blue.moment_of_inertia = BEETLE_RADIUS * physics_params["MOMENT_OF_INERTIA_FACTOR"]
        beetle_red.moment_of_inertia = BEETLE_RADIUS * physics_params["MOMENT_OF_INERTIA_FACTOR"]

    window.GUI.end()

    perf_monitor.stop('gui')

    # === END FRAME TIMING ===
    perf_monitor.stop('frame_total')
    perf_monitor.end_frame(physics_iterations=physics_iterations_this_frame)

    window.show()

# Print final performance summary
print("\n" + "="*60)
print("PERFORMANCE SUMMARY (last 120 frames)")
print("="*60)
for line in perf_monitor.get_detailed_breakdown():
    print(line)
print("="*60)

print("Done!")
