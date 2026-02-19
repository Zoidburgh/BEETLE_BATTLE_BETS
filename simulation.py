import os
import sys

# Force GPU selection to prefer discrete NVIDIA over integrated AMD/Intel
# This helps prevent the wild FPS variance on laptops with switchable graphics
os.environ['VK_ICD_FILENAMES'] = ''  # Let system choose
os.environ['DISABLE_LAYER_AMD_SWITCHABLE_GRAPHICS_1'] = '1'  # Disable AMD switchable
os.environ['SHIM_MCCOMPAT'] = '0x800000001'  # Force discrete GPU on NVIDIA Optimus
os.environ['NV_PRIME_RENDER_OFFLOAD'] = '1'  # Linux NVIDIA offload (doesn't hurt on Windows)
os.environ['__GLX_VENDOR_LIBRARY_NAME'] = 'nvidia'  # Linux NVIDIA preference
os.environ['DRI_PRIME'] = '1'  # Force discrete GPU on Linux (doesn't hurt on Windows)

# Set high process priority to reduce Windows scheduling variance
if sys.platform == 'win32':
    try:
        import ctypes
        # Get current process handle
        kernel32 = ctypes.windll.kernel32
        handle = kernel32.GetCurrentProcess()
        # Set HIGH_PRIORITY_CLASS (0x80) - above normal but below realtime
        kernel32.SetPriorityClass(handle, 0x80)
        print("[Performance] Set process priority to HIGH")
    except Exception as e:
        print(f"[Performance] Could not set process priority: {e}")

import taichi as ti
import subprocess

# Backend selection: Vulkan (GPU) vs CPU
# - Vulkan: Best for discrete GPUs (NVIDIA/AMD) - no CPU→GPU transfer overhead
# - CPU: Best for integrated graphics (Intel) - faster physics compute

def detect_gpu_type():
    """Detect GPU type: 'nvidia', 'amd', or None"""
    try:
        # Try PowerShell first (works on modern Windows)
        result = subprocess.run(
            ['powershell', '-Command', 'Get-CimInstance Win32_VideoController | Select-Object -ExpandProperty Name'],
            capture_output=True, text=True, timeout=5,
            creationflags=subprocess.CREATE_NO_WINDOW if sys.platform == 'win32' else 0
        )
        if result.returncode == 0:
            output = result.stdout.lower()
            # Check for NVIDIA discrete GPU (GeForce, RTX, GTX, Quadro)
            if 'nvidia' in output and any(x in output for x in ['geforce', 'rtx', 'gtx', 'quadro']):
                return 'nvidia'
            # Check for AMD discrete GPU
            if 'amd' in output and ('radeon' in output or 'rx ' in output):
                return 'amd'
    except:
        pass
    return None

def choose_backend():
    """Auto-detect best backend, with manual override flags"""
    # Manual overrides first
    if '--vulkan' in sys.argv or '--gpu' in sys.argv:
        return 'vulkan', 'Vulkan (--gpu flag)'
    if '--cuda' in sys.argv:
        return 'cuda', 'CUDA (--cuda flag)'

    # Default to CPU - GPU backends (Vulkan/CUDA) have compatibility issues
    # on many systems, causing worse performance than CPU
    return 'cpu', 'CPU (default - use --gpu or --cuda to try GPU)'

BACKEND, BACKEND_REASON = choose_backend()

# GPU backends need less frequent cleanup (serialized loops are slow on GPU)
# CPU can cleanup every 2 frames, GPU should cleanup less often
CLEANUP_FREQUENCY_DEBRIS = 2 if BACKEND == 'cpu' else 30  # Every 0.5s on GPU
CLEANUP_FREQUENCY_SPRAY = 2 if BACKEND == 'cpu' else 30
CLEANUP_FREQUENCY_SILK = 5 if BACKEND == 'cpu' else 60
BG_ANIM_FREQUENCY = 1 if BACKEND == 'cpu' else 3  # Background animation: every frame on CPU, every 3rd on GPU

# Check if user wants fresh kernel compilation (bypasses cache that might cause variance)
FRESH_COMPILE = '--fresh' in sys.argv

if BACKEND == 'cpu':
    # Pin thread count to reduce variance from Windows thread scheduling
    ti.init(arch=ti.cpu, debug=False, offline_cache=not FRESH_COMPILE, cpu_max_num_threads=8)
elif BACKEND == 'cuda':
    ti.init(arch=ti.cuda, debug=False, offline_cache=not FRESH_COMPILE)
else:
    ti.init(arch=ti.vulkan, debug=False, offline_cache=not FRESH_COMPILE)

if FRESH_COMPILE:
    print("[Taichi] Fresh compile mode - cache disabled")
print(f"Using {BACKEND_REASON}")

# Print GPU info for debugging (helps diagnose performance issues)
def print_gpu_info():
    try:
        # Use PowerShell (works on modern Windows, wmic is deprecated)
        result = subprocess.run(
            ['powershell', '-Command', 'Get-CimInstance Win32_VideoController | Select-Object -ExpandProperty Name'],
            capture_output=True, text=True, timeout=5,
            creationflags=subprocess.CREATE_NO_WINDOW if sys.platform == 'win32' else 0
        )
        if result.returncode == 0:
            lines = [l.strip() for l in result.stdout.strip().split('\n') if l.strip()]
            if lines:
                print(f"[GPU Info] Detected: {', '.join(lines)}")
                # Warn about potential Optimus issues
                has_nvidia = any('nvidia' in l.lower() for l in lines)
                has_intel = any('intel' in l.lower() for l in lines)
                if has_nvidia and has_intel:
                    print("[GPU Info] WARNING: Laptop has both Intel + NVIDIA (Optimus)")
                    print("[GPU Info] If FPS is low, set Python to 'High performance' in Windows Graphics Settings")
    except Exception as e:
        print(f"[GPU Info] Could not detect GPU: {e}")

print_gpu_info()

# 128x128x128 grid - optimal power-of-2 size for beetle battle (GPU cache friendly)
n_grid = 128
voxel_type = ti.field(dtype=ti.i32, shape=(n_grid, n_grid, n_grid))

# Debris particle system (flying particles from destroyed voxels)
MAX_DEBRIS = 20000  # Pre-allocated pool for performance
MAX_DEBRIS_CHECK = 5000  # Cap physics/render iteration to prevent high water mark FPS tank
num_debris = ti.field(dtype=ti.i32, shape=())  # Current particle count (for iteration)
debris_write_idx = ti.field(dtype=ti.i32, shape=())  # Ring buffer write position (wraps around)
debris_pos = ti.Vector.field(3, dtype=ti.f32, shape=MAX_DEBRIS)
debris_vel = ti.Vector.field(3, dtype=ti.f32, shape=MAX_DEBRIS)
debris_material = ti.Vector.field(3, dtype=ti.f32, shape=MAX_DEBRIS)  # RGB color (0.0-1.0)
debris_lifetime = ti.field(dtype=ti.f32, shape=MAX_DEBRIS)  # Time alive (seconds)
debris_active = ti.field(dtype=ti.i32, shape=MAX_DEBRIS)  # 1=alive, 0=dead (for free list pattern)
debris_active_count = ti.field(dtype=ti.i32, shape=())  # Actual live particle count (for high water mark reset)
debris_radius = ti.field(dtype=ti.f32, shape=MAX_DEBRIS)  # Per-particle radius (0 = use default DEBRIS_RADIUS)

# Spray particle system (bombardier beetle acid spray)
MAX_SPRAY = 500  # Pre-allocated pool for spray particles
MAX_SPRAY_CHECK = 500  # Cap physics/render iteration (matches MAX_SPRAY since pool is small)
num_spray = ti.field(dtype=ti.i32, shape=())  # High water mark (max index used)
spray_pos = ti.Vector.field(3, dtype=ti.f32, shape=MAX_SPRAY)
spray_vel = ti.Vector.field(3, dtype=ti.f32, shape=MAX_SPRAY)
spray_color = ti.Vector.field(3, dtype=ti.f32, shape=MAX_SPRAY)  # RGB color (0.0-1.0)
spray_lifetime = ti.field(dtype=ti.f32, shape=MAX_SPRAY)  # Time alive (seconds)
spray_owner = ti.field(dtype=ti.i32, shape=MAX_SPRAY)  # 0=blue, 1=red (don't hit own beetle)
spray_hit = ti.field(dtype=ti.i32, shape=MAX_SPRAY)  # 1=hit beetle this frame, 0=no hit
spray_hit_pos = ti.Vector.field(3, dtype=ti.f32, shape=MAX_SPRAY)  # Position where hit occurred
spray_active = ti.field(dtype=ti.i32, shape=MAX_SPRAY)  # 1=alive, 0=dead (for free list pattern)
spray_active_count = ti.field(dtype=ti.i32, shape=())  # Actual live particle count (for high water mark reset)

# Spider silk particle system (separate from spray - persists longer)
MAX_SILK = 600  # More particles since they persist longer
MAX_SILK_CHECK = 600  # Cap physics/render iteration (matches MAX_SILK since pool is small)
num_silk = ti.field(dtype=ti.i32, shape=())  # High water mark (max index used)
silk_pos = ti.Vector.field(3, dtype=ti.f32, shape=MAX_SILK)
silk_vel = ti.Vector.field(3, dtype=ti.f32, shape=MAX_SILK)
silk_color = ti.Vector.field(3, dtype=ti.f32, shape=MAX_SILK)  # RGB color (cream/white)
silk_lifetime = ti.field(dtype=ti.f32, shape=MAX_SILK)  # Time remaining (seconds)
silk_owner = ti.field(dtype=ti.i32, shape=MAX_SILK)  # 0=blue, 1=red
silk_stuck = ti.field(dtype=ti.i32, shape=MAX_SILK)  # 0=flying, 1=stuck to floor, 2=stuck to beetle, 3=stuck to ball
silk_active = ti.field(dtype=ti.i32, shape=MAX_SILK)  # 1=alive, 0=dead (for free list pattern)
silk_active_count = ti.field(dtype=ti.i32, shape=())  # Actual live particle count (for high water mark reset)
# Beetle-sticking tracking
silk_stuck_beetle = ti.field(dtype=ti.i32, shape=MAX_SILK)  # -1=none/floor, 0=blue, 1=red, 2=ball
# Silk counters per beetle (for slowdown effects)
silk_on_blue = ti.field(dtype=ti.i32, shape=())  # Count of silk stuck to blue beetle
silk_on_red = ti.field(dtype=ti.i32, shape=())   # Count of silk stuck to red beetle
# Floor silk counters (for speed effects when walking over silk)
silk_under_blue = ti.field(dtype=ti.i32, shape=())  # Floor silk near blue beetle
silk_under_red = ti.field(dtype=ti.i32, shape=())   # Floor silk near red beetle
# Ball silk counters (for friction effects in ball mode)
silk_on_ball = ti.field(dtype=ti.i32, shape=())     # Count of silk stuck to ball
silk_under_ball = ti.field(dtype=ti.i32, shape=())  # Floor silk near ball
# OPTIMIZATION: Batched silk counts array for single GPU->CPU transfer
# Indices: 0=on_blue, 1=under_blue, 2=on_red, 3=under_red, 4=on_ball, 5=under_ball
silk_counts_batched = ti.field(dtype=ti.i32, shape=6)
silk_stuck_voxel_idx = ti.field(dtype=ti.i32, shape=MAX_SILK)  # index into body cache
silk_stuck_offset = ti.Vector.field(3, dtype=ti.f32, shape=MAX_SILK)  # small random offset for variation

# Silk spatial grid for O(1) neighbor queries (replaces O(n²) anti-stacking check)
# Arena is 64m diameter, cell_size = 1.0m, grid covers -32 to +32 in x/z
SILK_GRID_SIZE = 64  # 64x64 grid cells
SILK_CELL_SIZE = 1.0  # 1 meter per cell (2x MIN_FLOOR_SPACING of 0.5m)
SILK_MAX_PER_CELL = 32  # Max silk particles per grid cell (doubled for 2 spider matches)
silk_grid_count = ti.field(dtype=ti.i32, shape=(SILK_GRID_SIZE, SILK_GRID_SIZE))  # Count per cell
silk_grid_particles = ti.field(dtype=ti.i32, shape=(SILK_GRID_SIZE, SILK_GRID_SIZE, SILK_MAX_PER_CELL))  # Particle indices

# Projectile system (cannonballs)
MAX_PROJECTILES = 10  # Maximum active projectiles
num_projectiles = ti.field(dtype=ti.i32, shape=())  # Active projectile count
projectile_pos = ti.Vector.field(3, dtype=ti.f32, shape=MAX_PROJECTILES)
projectile_vel = ti.Vector.field(3, dtype=ti.f32, shape=MAX_PROJECTILES)
projectile_active = ti.field(dtype=ti.i32, shape=MAX_PROJECTILES)  # 1 = active, 0 = inactive
projectile_radius = ti.field(dtype=ti.f32, shape=MAX_PROJECTILES)  # Collision radius

# Ball system (beetle soccer ball) - now handled by beetle_ball Beetle object in beetle_physics.py
# (No longer using Taichi fields; ball is a Beetle with horn_type="ball")

# ============================================================
# BACKGROUND VOXEL SYSTEM - Animated backgrounds (stars, grass, etc.)
# ============================================================
MAX_BACKGROUND_VOXELS = 8000  # Budget for all background effects

# Animation types
BG_ANIM_NONE = 0      # Static, no animation
BG_ANIM_TWINKLE = 1   # Brightness oscillation (stars)
BG_ANIM_SWAY = 2      # Horizontal wave motion (grass, seaweed)
BG_ANIM_DRIFT = 3     # Slow position movement (clouds, snow)
BG_ANIM_FLICKER = 4   # Random on/off (sparks)
BG_ANIM_FIREFLY = 5   # Firefly: gentle wandering path + glow pulse
BG_ANIM_WATER = 6     # Water: flowing stream movement
BG_ANIM_JELLYFISH = 7 # Jellyfish: pulsing bell with trailing tentacles
BG_ANIM_BUTTERFLY = 8 # Butterfly: flapping wings with drift
BG_ANIM_WAVE = 9       # Water: rolling wave blobs
BG_ANIM_TREE = 10      # Tree: branch sway with wind effect
BG_ANIM_SCRUNCH = 11   # Grub: vertical scrunching motion
BG_ANIM_FISH = 12      # Fish: jumping arc in/out of waves
BG_ANIM_LAVA = 13      # Lava: slow viscous wave with molten glow
BG_ANIM_LAVA_SPRAY = 14  # Lava spray: cyclic parabolic eruption arcs
BG_ANIM_LAVA_VOLCANO = 15  # Lava volcano: scrunches to squirt out eruption
BG_ANIM_RAIN = 16            # Rain: fast falling drops cycling top to bottom
BG_ANIM_RAIN_SPLASH = 17     # Rain splash: pop-up burst at ground level
BG_ANIM_CLOUD = 18           # Cloud: slow drift + breathing + brightness pulse
BG_ANIM_PTERODACTYL = 19     # Pterodactyl: circular orbit with wing flap
BG_ANIM_FLOWER = 20          # Wildflower: sway + periodic gust blows them away
BG_ANIM_SERPENT_SPLASH = 21  # Serpent splash: burst of water voxels when head dives
BG_ANIM_CONSTELLATION = 22   # Constellation: synced pulse with traveling sparkle along lines
BG_ANIM_COMET = 23           # Comet: orbiting head with trailing tail
BG_ANIM_SWAMP_BUBBLE = 24   # Swamp bubble: rise from water, pop, respawn
BG_ANIM_SPORE_DRIFT = 25    # Spore: lazy upward drift with horizontal wandering
BG_ANIM_GLOW_PULSE = 26     # Mushroom glow: smooth brightness pulse
BG_ANIM_CATERPILLAR = 27    # Caterpillar: slow orbit with inchworm body wave
BG_ANIM_LAVA_SNAIL = 28     # Lava snail: slow orbit with slug crawl + spiral shell
BG_ANIM_DUST_DEVIL = 29     # Dust devil: phased spiral column with lifecycle
BG_ANIM_SCORPION = 30       # Scorpion: ground orbit with tail curl, claw pinch
BG_ANIM_TOAD = 31           # Toad: hopping orbit with idle breathing + throat puff
BG_ANIM_MUD_SPLASH = 32     # Mud splash: burst on toad landing
BG_ANIM_JUMPING_FISH = 33   # Dolphin: periodic jump arc above waves
BG_ANIM_FISH_SPLASH = 34    # Splash burst on dolphin water entry/exit
BG_ANIM_SQUID_TENTACLE = 35 # Giant squid tentacle chain
BG_ANIM_SQUID_SPLASH = 36   # Splash on tentacle emerge/plunge

# Background voxel fields
bg_positions = ti.Vector.field(3, dtype=ti.f32, shape=MAX_BACKGROUND_VOXELS)      # Base position
bg_colors = ti.Vector.field(3, dtype=ti.f32, shape=MAX_BACKGROUND_VOXELS)         # RGB color
bg_phase = ti.field(dtype=ti.f32, shape=MAX_BACKGROUND_VOXELS)                    # Animation phase offset
bg_size = ti.field(dtype=ti.f32, shape=MAX_BACKGROUND_VOXELS)                     # Voxel radius
bg_anim_type = ti.field(dtype=ti.i32, shape=MAX_BACKGROUND_VOXELS)                # Which animation
bg_anim_speed = ti.field(dtype=ti.f32, shape=MAX_BACKGROUND_VOXELS)               # Animation speed
bg_anim_amplitude = ti.field(dtype=ti.f32, shape=MAX_BACKGROUND_VOXELS)           # Animation amplitude
bg_active = ti.field(dtype=ti.i32, shape=MAX_BACKGROUND_VOXELS)                   # 1 = active
num_bg_voxels = ti.field(dtype=ti.i32, shape=())

# Animation output fields (updated each frame)
bg_brightness = ti.field(dtype=ti.f32, shape=MAX_BACKGROUND_VOXELS)               # Current brightness multiplier
bg_offset_x = ti.field(dtype=ti.f32, shape=MAX_BACKGROUND_VOXELS)                 # Current x offset
bg_offset_y = ti.field(dtype=ti.f32, shape=MAX_BACKGROUND_VOXELS)                 # Current y offset
bg_offset_z = ti.field(dtype=ti.f32, shape=MAX_BACKGROUND_VOXELS)                 # Current z offset

# Background theme state
bg_theme_active = ti.field(dtype=ti.i32, shape=())  # 0=off, 1=stars, 2=grass, 3=fireflies, etc.

# ============================================================
# STACKABLE THEME SYSTEM - Multiple themes can be active at once
# ============================================================
# Theme IDs (match bg_theme_active values)
THEME_STARS = 1
THEME_GRASS = 2
THEME_FIREFLIES = 3
THEME_WATER = 4
THEME_JELLYFISH = 5
THEME_BUTTERFLIES = 6
THEME_WAVES = 7
THEME_PALM_TREES = 10
THEME_STADIUM = 11
THEME_LAVA = 12
THEME_RAIN = 13
THEME_CLOUDS = 14
THEME_PTERODACTYL = 15
THEME_SWAMP = 16
THEME_DESERT = 17
THEME_COMET = 18

# Track each theme's voxel range
theme_start_idx = {}   # theme_id -> start index in bg_* arrays
theme_count = {}       # theme_id -> number of voxels for this theme
active_themes = set()  # which themes are currently active

# Stadium crowd excitement (for score reactions)
stadium_excitement = ti.field(dtype=ti.f32, shape=())  # 0-1, current excitement level
stadium_excitement_target = ti.field(dtype=ti.f32, shape=())  # Target to ramp toward
bg_angle = ti.field(dtype=ti.f32, shape=MAX_BACKGROUND_VOXELS)  # Angle around arena

# ---- Numpy staging buffers for background voxel batch writes ----
# Theme builders write here (fast CPU array ops), then bg_flush() copies to GPU in bulk.
import numpy as np
_bg_pos_np = np.zeros((MAX_BACKGROUND_VOXELS, 3), dtype=np.float32)
_bg_col_np = np.zeros((MAX_BACKGROUND_VOXELS, 3), dtype=np.float32)
_bg_phase_np = np.zeros(MAX_BACKGROUND_VOXELS, dtype=np.float32)
_bg_size_np = np.zeros(MAX_BACKGROUND_VOXELS, dtype=np.float32)
_bg_anim_type_np = np.zeros(MAX_BACKGROUND_VOXELS, dtype=np.int32)
_bg_anim_speed_np = np.zeros(MAX_BACKGROUND_VOXELS, dtype=np.float32)
_bg_anim_amp_np = np.zeros(MAX_BACKGROUND_VOXELS, dtype=np.float32)
_bg_active_np = np.zeros(MAX_BACKGROUND_VOXELS, dtype=np.int32)
_bg_brightness_np = np.ones(MAX_BACKGROUND_VOXELS, dtype=np.float32)
_bg_offset_x_np = np.zeros(MAX_BACKGROUND_VOXELS, dtype=np.float32)
_bg_offset_y_np = np.zeros(MAX_BACKGROUND_VOXELS, dtype=np.float32)
_bg_offset_z_np = np.zeros(MAX_BACKGROUND_VOXELS, dtype=np.float32)
_bg_angle_np = np.zeros(MAX_BACKGROUND_VOXELS, dtype=np.float32)
_bg_count = 0  # Python-side voxel counter (mirrors num_bg_voxels on GPU)

def bg_flush():
    """Bulk-copy numpy background buffers to GPU Taichi fields (fast batch transfer)."""
    bg_positions.from_numpy(_bg_pos_np)
    bg_colors.from_numpy(_bg_col_np)
    bg_phase.from_numpy(_bg_phase_np)
    bg_size.from_numpy(_bg_size_np)
    bg_anim_type.from_numpy(_bg_anim_type_np)
    bg_anim_speed.from_numpy(_bg_anim_speed_np)
    bg_anim_amplitude.from_numpy(_bg_anim_amp_np)
    bg_active.from_numpy(_bg_active_np)
    bg_brightness.from_numpy(_bg_brightness_np)
    bg_offset_x.from_numpy(_bg_offset_x_np)
    bg_offset_y.from_numpy(_bg_offset_y_np)
    bg_offset_z.from_numpy(_bg_offset_z_np)
    bg_angle.from_numpy(_bg_angle_np)
    num_bg_voxels[None] = _bg_count

# Voxel types
EMPTY = 0
STEEL = 1
CONCRETE = 2
MOLTEN = 3  # Molten metal/material (will fall due to gravity)
DEBRIS = 4  # Destroyed material (small falling chunks)
BEETLE_BLUE = 5  # Beetle voxels (blue color)
BEETLE_RED = 6  # Second beetle (red color)
BEETLE_BLUE_LEGS = 7  # Blue beetle legs (lighter blue)
BEETLE_RED_LEGS = 8  # Red beetle legs (lighter red)
LEG_TIP_BLUE = 9  # Blue beetle leg tips (dark blue for tracking)
LEG_TIP_RED = 10  # Red beetle leg tips (dark red for tracking)
BEETLE_BLUE_STRIPE = 11  # Blue beetle racing stripe (bright cyan/white)
BEETLE_RED_STRIPE = 12  # Red beetle racing stripe (bright yellow/orange)
BEETLE_BLUE_HORN_TIP = 13  # Blue beetle horn prong tips (bright white/cyan)
BEETLE_RED_HORN_TIP = 14  # Red beetle horn prong tips (dark metallic)
STINGER_TIP_BLACK = 15  # Scorpion stinger tips (black/dark)
BALL = 16  # Soccer ball (bright orange/yellow)
BALL_STRIPE = 17  # Soccer ball stripe pattern (darker for contrast)
STAG_HOOK_INTERIOR_BLUE = 18  # Blue beetle stag hook interior (curved inward section)
STAG_HOOK_INTERIOR_RED = 19  # Red beetle stag hook interior (curved inward section)
SHADOW = 20  # Shadow blob beneath airborne beetles
SLIPPERY = 21  # Slippery bowl perimeter (ball mode only)
GOAL = 22  # Goal doorway walls (ball mode only)
SCORE_DIGIT_BLUE = 23  # Blue team floating score digit
SCORE_DIGIT_RED = 24  # Red team floating score digit
ASSEMBLY_VOXEL_BALL = 27  # Ball assembly animation voxel
VENOM_TIP_BLUE = 33  # Blue scorpion venom bulb/stinger tip (glows with charges)
VENOM_TIP_RED = 34  # Red scorpion venom bulb/stinger tip (glows with charges)

# Ladybug cheerleader voxel types
LADYBUG_SHELL = 35  # Red ladybug shell (dome)
LADYBUG_SPOTS = 36  # Black spots on shell
LADYBUG_HEAD = 37   # Black head
LADYBUG_LEGS = 38   # Black legs
LADYBUG_WINGS = 39  # Translucent/light wings

# Title screen voxel types
TITLE_BLUE = 40     # "BEETLE" text - blue team color
TITLE_RED = 41      # "BATTLE" text - red team color
TITLE_GOLD = 42     # "BROS" text - gold/amber accent
TITLE_WHITE = 43    # Controls text - soft white/cream
TITLE_CYAN = 44     # "PRESS SPACE" text - cyan highlight
UFO_HULL = 45       # UFO saucer hull - silver metallic
UFO_DOME = 46       # UFO cockpit dome - green glass
UFO_LIGHTS = 47     # UFO running lights - pulsing cyan
UFO_BEAM = 48       # UFO laser beam - bright green
UFO_RIM = 49        # UFO middle belt/rim band

# UFO dome flash (1.0 = normal, >1.0 = bright flash for telegraph/fire)
ufo_dome_flash = ti.field(dtype=ti.f32, shape=())

# Score digit flash brightness (1.0 = normal, >1.0 = bright flash)
blue_score_flash = ti.field(dtype=ti.f32, shape=())
red_score_flash = ti.field(dtype=ti.f32, shape=())

# Title screen flash (pulsing glow effect)
title_flash = ti.field(dtype=ti.f32, shape=())
title_flash[None] = 1.0

# Customizable beetle colors (RGB values in range 0.0-1.0)
# Blue beetle colors
blue_body_color = ti.Vector.field(3, dtype=ti.f32, shape=())
blue_leg_color = ti.Vector.field(3, dtype=ti.f32, shape=())
blue_leg_tip_color = ti.Vector.field(3, dtype=ti.f32, shape=())
blue_stripe_color = ti.Vector.field(3, dtype=ti.f32, shape=())
blue_horn_tip_color = ti.Vector.field(3, dtype=ti.f32, shape=())
blue_venom_tip_color = ti.Vector.field(3, dtype=ti.f32, shape=())

# Red beetle colors
red_body_color = ti.Vector.field(3, dtype=ti.f32, shape=())
red_leg_color = ti.Vector.field(3, dtype=ti.f32, shape=())
red_leg_tip_color = ti.Vector.field(3, dtype=ti.f32, shape=())
red_stripe_color = ti.Vector.field(3, dtype=ti.f32, shape=())
red_horn_tip_color = ti.Vector.field(3, dtype=ti.f32, shape=())
red_venom_tip_color = ti.Vector.field(3, dtype=ti.f32, shape=())

# Initialize default colors
blue_body_color[None] = ti.Vector([0.25, 0.55, 0.95])  # Desaturated blue
blue_leg_color[None] = ti.Vector([0.4, 0.7, 1.0])  # Lighter cyan/blue
blue_leg_tip_color[None] = ti.Vector([0.0, 0.0, 0.3])  # Very dark blue
blue_stripe_color[None] = ti.Vector([0.6, 0.9, 1.0])  # Bright cyan
blue_horn_tip_color[None] = ti.Vector([0.4, 0.75, 1.0])  # Bright electric blue
blue_venom_tip_color[None] = ti.Vector([0.6, 0.2, 0.8])  # Bright purple (full venom)

# Initialize score flash to normal brightness
blue_score_flash[None] = 1.0
red_score_flash[None] = 1.0

red_body_color[None] = ti.Vector([0.95, 0.25, 0.15])  # Desaturated red
red_leg_color[None] = ti.Vector([1.0, 0.5, 0.3])  # Lighter orange/red
red_leg_tip_color[None] = ti.Vector([0.3, 0.0, 0.0])  # Very dark red
red_stripe_color[None] = ti.Vector([0.85, 0.65, 0.2])  # Rich gold/bronze
red_horn_tip_color[None] = ti.Vector([0.4, 0.1, 0.1])  # Deep crimson
red_venom_tip_color[None] = ti.Vector([0.6, 0.2, 0.8])  # Bright purple (full venom)

# Ball colors (for future customization)
ball_color = ti.Vector.field(3, dtype=ti.f32, shape=())
ball_stripe_color = ti.Vector.field(3, dtype=ti.f32, shape=())
ball_color[None] = ti.Vector([0.65, 0.45, 0.25])  # Light brown dung color
ball_stripe_color[None] = ti.Vector([0.35, 0.22, 0.1])  # Darker brown stripe

# Arena board color (customizable, not networked)
board_color = ti.Vector.field(3, dtype=ti.f32, shape=())
board_color[None] = ti.Vector([0.41, 0.39, 0.37])  # Default brownish gray

# Material property functions (for physics calculations)
@ti.func
def get_material_hardness(voxel_type: ti.i32) -> ti.f32:
    """Material resistance to destruction - higher = more resistant"""
    hardness = 1.0  # Default
    if voxel_type == 1:  # STEEL
        hardness = 2.0  # Takes half damage
    elif voxel_type == 2:  # CONCRETE
        hardness = 1.0  # Baseline
    elif voxel_type == 3:  # MOLTEN
        hardness = 0.1  # Almost no resistance
    return hardness

@ti.func
def get_material_weight(voxel_type: ti.i32) -> ti.f32:
    """Mass per voxel in kg (for future structural calculations)"""
    weight = 1.0  # Default
    if voxel_type == 1:  # STEEL
        weight = 7.8  # kg per voxel
    elif voxel_type == 2:  # CONCRETE
        weight = 2.4  # kg per voxel
    elif voxel_type == 3:  # MOLTEN
        weight = 7.0  # Similar to steel
    return weight

@ti.kernel
def init_test_building():
    """Create an AWE-INSPIRING GOTHIC CATHEDRAL - Notre-Dame scale magnificence"""
    # Clear everything first
    for i, j, k in ti.ndrange(n_grid, n_grid, n_grid):
        voxel_type[i, j, k] = EMPTY

    # Cathedral center
    center_x = 64
    center_z = 64

    # === MAIN NAVE (central hall) ===
    nave_length = 50
    nave_width = 16
    nave_height = 35

    # Nave walls (hollow interior)
    for nx in range(center_x - nave_width//2, center_x + nave_width//2):
        for nz in range(center_z - nave_length//2, center_z + nave_length//2):
            for ny in range(0, nave_height):
                # Outer walls only (hollow interior)
                if nx == center_x - nave_width//2 or nx == center_x + nave_width//2 - 1 or nz == center_z - nave_length//2 or nz == center_z + nave_length//2 - 1:
                    voxel_type[nx, ny, nz] = CONCRETE if ny < 5 else STEEL

    # === TWIN SPIRES (towering above all) ===
    # West facade spires
    # Left spire
    spire_x = center_x - 10
    spire_z = center_z - nave_length//2 - 5

    # Base tower (wide)
    for sx in range(spire_x - 4, spire_x + 5):
        for sz in range(spire_z - 4, spire_z + 5):
            for sy in range(0, 50):
                if abs(sx - spire_x) == 4 or abs(sz - spire_z) == 4:  # Hollow
                    voxel_type[sx, sy, sz] = STEEL

    # Tapered spire top
    for height_offset in range(0, 35):
        taper = max(1, 3 - height_offset // 12)  # Shrinks as it goes up
        sy = 50 + height_offset
        for sx in range(spire_x - taper, spire_x + taper + 1):
            for sz in range(spire_z - taper, spire_z + taper + 1):
                if abs(sx - spire_x) == taper or abs(sz - spire_z) == taper:
                    voxel_type[sx, sy, sz] = STEEL

    # Right spire
    spire_x = center_x + 10
    spire_z = center_z - nave_length//2 - 5

    # Base tower (wide)
    for sx in range(spire_x - 4, spire_x + 5):
        for sz in range(spire_z - 4, spire_z + 5):
            for sy in range(0, 50):
                if abs(sx - spire_x) == 4 or abs(sz - spire_z) == 4:  # Hollow
                    voxel_type[sx, sy, sz] = STEEL

    # Tapered spire top
    for height_offset in range(0, 35):
        taper = max(1, 3 - height_offset // 12)  # Shrinks as it goes up
        sy = 50 + height_offset
        for sx in range(spire_x - taper, spire_x + taper + 1):
            for sz in range(spire_z - taper, spire_z + taper + 1):
                if abs(sx - spire_x) == taper or abs(sz - spire_z) == taper:
                    voxel_type[sx, sy, sz] = STEEL

    # === ROSE WINDOW (circular hollow pattern on west facade) ===
    rose_x = center_x
    rose_y = 30
    rose_z = center_z - nave_length//2
    rose_radius = 6

    for rx in range(rose_x - rose_radius, rose_x + rose_radius + 1):
        for ry in range(rose_y - rose_radius, rose_y + rose_radius + 1):
            dx = float(rx - rose_x)
            dy = float(ry - rose_y)
            dist = ti.sqrt(dx * dx + dy * dy)

            # Hollow circle with radiating pattern
            if dist < rose_radius and dist > rose_radius - 2:
                voxel_type[rx, ry, rose_z] = STEEL
            # Radiating spokes every 45 degrees
            if dist < rose_radius:
                angle = ti.atan2(dy, dx)
                if ti.abs(angle % (3.14159 / 4.0)) < 0.1:
                    voxel_type[rx, ry, rose_z] = STEEL

    # === FLYING BUTTRESSES (external supports) ===
    # North side buttresses
    for buttress_z in range(center_z - 20, center_z + 20):
        if (buttress_z - (center_z - 20)) % 12 != 0:
            continue
        for by in range(15, 30):
            arch_x_outer = center_x - nave_width//2 - 4
            arch_x_inner = center_x - nave_width//2

            # Diagonal arch support
            progress = float(by - 15) / 15.0
            bx = int(arch_x_outer + progress * (arch_x_inner - arch_x_outer))
            voxel_type[bx, by, buttress_z] = CONCRETE
            voxel_type[bx + 1, by, buttress_z] = CONCRETE

            # Vertical pier at outer edge
            for bpy in range(0, by):
                voxel_type[arch_x_outer, bpy, buttress_z] = CONCRETE
                voxel_type[arch_x_outer + 1, bpy, buttress_z] = CONCRETE

    # South side buttresses (mirror)
    for buttress_z in range(center_z - 20, center_z + 20):
        if (buttress_z - (center_z - 20)) % 12 != 0:
            continue
        for by in range(15, 30):
            arch_x_outer = center_x + nave_width//2 + 4
            arch_x_inner = center_x + nave_width//2

            progress = float(by - 15) / 15.0
            bx = int(arch_x_outer - progress * (arch_x_outer - arch_x_inner))
            voxel_type[bx, by, buttress_z] = CONCRETE
            voxel_type[bx - 1, by, buttress_z] = CONCRETE

            for bpy in range(0, by):
                voxel_type[arch_x_outer, bpy, buttress_z] = CONCRETE
                voxel_type[arch_x_outer - 1, bpy, buttress_z] = CONCRETE

    # === TRANSEPT (cross arms) ===
    transept_width = 12
    transept_length = 30
    transept_height = 30

    for tx in range(center_x - transept_length//2, center_x + transept_length//2):
        for tz in range(center_z - transept_width//2, center_z + transept_width//2):
            for ty in range(0, transept_height):
                # Walls only (hollow)
                if tx == center_x - transept_length//2 or tx == center_x + transept_length//2 - 1 or tz == center_z - transept_width//2 or tz == center_z + transept_width//2 - 1:
                    voxel_type[tx, ty, tz] = CONCRETE if ty < 5 else STEEL

    # === APSE (rounded east end) ===
    apse_center_z = center_z + nave_length//2 + 8
    apse_radius = 10
    apse_height = 25

    for ax in range(center_x - apse_radius, center_x + apse_radius + 1):
        for az in range(apse_center_z - apse_radius, apse_center_z + apse_radius + 1):
            dx = float(ax - center_x)
            dz = float(az - apse_center_z)
            dist = ti.sqrt(dx * dx + dz * dz)

            if dist < apse_radius and dist > apse_radius - 2:  # Shell only
                for ay in range(0, apse_height):
                    voxel_type[ax, ay, az] = STEEL

    # === BELL TOWER (central crossing) ===
    for bx in range(center_x - 5, center_x + 6):
        for bz in range(center_z - 5, center_z + 6):
            for by in range(30, 60):  # Above nave roof
                if abs(bx - center_x) == 5 or abs(bz - center_z) == 5:  # Hollow
                    voxel_type[bx, by, bz] = STEEL
                # Bell chamber (hollow space at top)
                if by > 50 and by < 58:
                    if abs(bx - center_x) == 4 or abs(bz - center_z) == 4:
                        voxel_type[bx, by, bz] = EMPTY

    # === GRAND ENTRANCE PORTAL ===
    portal_z = center_z - nave_length//2
    portal_width = 8
    portal_height = 15

    for px in range(center_x - portal_width//2, center_x + portal_width//2):
        for py in range(0, portal_height):
            # Pointed arch shape
            dx = abs(px - center_x)
            if py > 8:
                # Upper pointed part
                if dx <= (portal_height - py):
                    voxel_type[px, py, portal_z] = EMPTY
            else:
                # Lower rectangular part
                voxel_type[px, py, portal_z] = EMPTY

    # === PINNACLES (decorative spires on buttresses) ===
    for pz in range(center_z - 20, center_z + 20):
        if (pz - (center_z - 20)) % 12 != 0:
            continue
        # Left side pinnacle
        px = center_x - nave_width//2 - 4
        for py in range(30, 42):
            taper = max(0, 2 - (py - 30) // 4)
            for ppx in range(px - taper, px + taper + 1):
                for ppz in range(pz - taper, pz + taper + 1):
                    if abs(ppx - px) == taper or abs(ppz - pz) == taper:
                        voxel_type[ppx, py, ppz] = STEEL

        # Right side pinnacle
        px = center_x + nave_width//2 + 4
        for py in range(30, 42):
            taper = max(0, 2 - (py - 30) // 4)
            for ppx in range(px - taper, px + taper + 1):
                for ppz in range(pz - taper, pz + taper + 1):
                    if abs(ppx - px) == taper or abs(ppz - pz) == taper:
                        voxel_type[ppx, py, ppz] = STEEL

    print("GOTHIC CATHEDRAL constructed - 85 blocks tall with twin spires, flying buttresses, rose window, and apse")

@ti.kernel
def init_beetle_arena():
    """
    BEETLE BATTLE ARENA - Circular fighting pit
    """
    # Clear floor layers only (Y=30-40 covers floor at 33, bowl perimeter up to ~36)
    for i, j, k in ti.ndrange(n_grid, (30, 41), n_grid):
        voxel_type[i, j, k] = EMPTY

    # Arena center (updated for 128 grid)
    center_x = 64
    center_y = -2  # Lower floor so beetle legs touch properly
    center_z = 64

    # Arena dimensions (25% smaller for closer combat)
    arena_radius = 32
    floor_thickness = 2
    floor_y_offset = 33  # Offset to match RENDER_Y_OFFSET in beetle_physics.py

    # Build circular floor - RAISED to allow beetles to fall below and be visible
    for i in range(center_x - arena_radius - 5, center_x + arena_radius + 5):
        for k in range(center_z - arena_radius - 5, center_z + arena_radius + 5):
            dx = float(i - center_x)
            dz = float(k - center_z)
            dist = ti.sqrt(dx * dx + dz * dz)

            # Floor (flat circle) at raised Y position
            if dist <= arena_radius:
                voxel_type[i, floor_y_offset, k] = CONCRETE  # Floor at offset height
                voxel_type[i, floor_y_offset + 1, k] = EMPTY  # Clear space above floor

    # Beetles are now dynamically rendered by beetle_physics.py
    # No static beetles needed in simulation initialization

    print(f"BEETLE BATTLE ARENA constructed - {arena_radius}m radius circular pit")
    print(f"Two beetles placed: blue (center-west) and red (center-east)")

@ti.kernel
def init_donut_arena():
    """
    DONUT ARENA - Circular fighting pit with hole in the middle
    """
    # Clear floor layers only (Y=30-40 covers floor at 33, bowl perimeter up to ~36)
    for i, j, k in ti.ndrange(n_grid, (30, 41), n_grid):
        voxel_type[i, j, k] = EMPTY

    # Arena center (updated for 128 grid)
    center_x = 64
    center_z = 64

    # Arena dimensions
    arena_radius = 32
    inner_radius = 13  # Hole in the middle
    floor_y_offset = 33  # Offset to match RENDER_Y_OFFSET in beetle_physics.py

    # Build donut floor - ring shape with hole in middle
    for i in range(center_x - arena_radius - 5, center_x + arena_radius + 5):
        for k in range(center_z - arena_radius - 5, center_z + arena_radius + 5):
            dx = float(i - center_x)
            dz = float(k - center_z)
            dist = ti.sqrt(dx * dx + dz * dz)

            # Donut: outside inner radius AND inside outer radius
            if dist <= arena_radius and dist > inner_radius:
                voxel_type[i, floor_y_offset, k] = CONCRETE  # Floor at offset height
                voxel_type[i, floor_y_offset + 1, k] = EMPTY  # Clear space above floor

    print(f"DONUT ARENA constructed - outer radius {arena_radius}m, inner hole {inner_radius}m")

@ti.kernel
def init_x_stage_arena():
    """
    X STAGE ARENA - Circle with 4 corner wedges cut out (plus shape)
    Arms extend in cardinal directions (N/S/E/W)
    Beetles fall off if they walk into the cut-out corner wedges
    """
    # Clear floor layers only (Y=30-40 covers floor at 33, bowl perimeter up to ~36)
    for i, j, k in ti.ndrange(n_grid, (30, 41), n_grid):
        voxel_type[i, j, k] = EMPTY

    center_x = 64
    center_z = 64
    arena_radius = 32
    floor_y_offset = 33
    arm_half_width = 12  # Width of each arm (24 voxels total, covers center well)

    # Build plus-shaped floor
    for i in range(center_x - arena_radius - 5, center_x + arena_radius + 5):
        for k in range(center_z - arena_radius - 5, center_z + arena_radius + 5):
            dx = float(i - center_x)
            dz = float(k - center_z)
            dist = ti.sqrt(dx * dx + dz * dz)

            # Check if within circle AND within one of the 4 arms
            # Arms extend in cardinal directions (N/S/E/W)
            in_ns_arm = abs(dx) <= arm_half_width  # North-South arm (vertical)
            in_ew_arm = abs(dz) <= arm_half_width  # East-West arm (horizontal)

            if dist <= arena_radius and (in_ns_arm or in_ew_arm):
                voxel_type[i, floor_y_offset, k] = CONCRETE
                voxel_type[i, floor_y_offset + 1, k] = EMPTY  # Clear space above floor

    print(f"X STAGE ARENA constructed - plus shape with {arm_half_width * 2} voxel wide arms")

@ti.kernel
def init_figure8_arena():
    """
    FIGURE 8 ARENA - Two circles connected by a narrow bridge
    Longer than normal arena, beetles can fall off edges or into gaps
    """
    # Clear floor layers only (Y=30-40 covers floor at 33, bowl perimeter up to ~36)
    for i, j, k in ti.ndrange(n_grid, (30, 41), n_grid):
        voxel_type[i, j, k] = EMPTY

    # Two circles connected by bridge
    # Left circle: center (42, 64), radius 20
    # Right circle: center (86, 64), radius 20
    # Bridge: rectangle connecting them
    left_center_x = 42
    right_center_x = 86
    center_z = 64
    circle_radius = 20
    bridge_half_width = 6  # Bridge is 12 voxels wide
    floor_y_offset = 33

    for i in range(10, 118):  # Extended range for larger arena
        for k in range(30, 98):
            dx_left = float(i - left_center_x)
            dx_right = float(i - right_center_x)
            dz = float(k - center_z)

            dist_left = ti.sqrt(dx_left * dx_left + dz * dz)
            dist_right = ti.sqrt(dx_right * dx_right + dz * dz)

            # Check if in left circle, right circle, or bridge
            in_left_circle = dist_left <= circle_radius
            in_right_circle = dist_right <= circle_radius
            # Bridge connects the two circles (between their centers, narrow strip)
            in_bridge = (i >= left_center_x and i <= right_center_x and
                        abs(k - center_z) <= bridge_half_width)

            if in_left_circle or in_right_circle or in_bridge:
                voxel_type[i, floor_y_offset, k] = CONCRETE
                voxel_type[i, floor_y_offset + 1, k] = EMPTY

    print(f"FIGURE 8 ARENA constructed - two circles (radius {circle_radius}) with bridge")

@ti.kernel
def init_yinyang_arena():
    """
    YIN-YANG ARENA - Large hollow ring with S-curved bridge through middle
    The bridge curves like a yin-yang symbol, creating interesting movement paths
    """
    # Clear floor layers only (Y=30-40 covers floor at 33, bowl perimeter up to ~36)
    for i, j, k in ti.ndrange(n_grid, (30, 41), n_grid):
        voxel_type[i, j, k] = EMPTY

    center_x = 64
    center_z = 64
    outer_radius = 38  # Larger than normal arena
    inner_radius = 26  # Large hole in middle (bigger)
    bridge_half_width = 5  # Width of the S-curve bridge
    floor_y_offset = 33

    # The S-curve is made of two arcs:
    # Left arc: center at (center_x - curve_radius, center_z), bulges toward +z
    # Right arc: center at (center_x + curve_radius, center_z), bulges toward -z
    # Larger curve_radius = gentler/wider angle curves
    curve_radius = inner_radius * 0.8  # 20.8 - wider, gentler arcs

    for i in range(center_x - outer_radius - 2, center_x + outer_radius + 2):
        for k in range(center_z - outer_radius - 2, center_z + outer_radius + 2):
            dx = float(i - center_x)
            dz = float(k - center_z)
            dist = ti.sqrt(dx * dx + dz * dz)

            # Check if in the hollow ring
            in_ring = dist <= outer_radius and dist >= inner_radius

            # Check if on the S-curved bridge (only inside the hole, not extending into ring)
            in_bridge = 0

            # Only consider bridge if we're inside the inner hole
            if dist <= inner_radius:
                # Upper arc (z >= center, curves toward +x) - extend past center for smooth blend
                if k >= center_z - bridge_half_width:
                    arc_center_x = center_x
                    arc_center_z = center_z + curve_radius
                    arc_dx = float(i - arc_center_x)
                    arc_dz = float(k - arc_center_z)
                    arc_dist = ti.sqrt(arc_dx * arc_dx + arc_dz * arc_dz)
                    # Right half, extended past center by bridge width for overlap
                    if ti.abs(arc_dist - curve_radius) <= bridge_half_width and i >= center_x - bridge_half_width:
                        in_bridge = 1

                # Lower arc (z <= center, curves toward -x) - extend past center for smooth blend
                if k <= center_z + bridge_half_width:
                    arc_center_x = center_x
                    arc_center_z = center_z - curve_radius
                    arc_dx = float(i - arc_center_x)
                    arc_dz = float(k - arc_center_z)
                    arc_dist = ti.sqrt(arc_dx * arc_dx + arc_dz * arc_dz)
                    # Left half, extended past center by bridge width for overlap
                    if ti.abs(arc_dist - curve_radius) <= bridge_half_width and i <= center_x + bridge_half_width:
                        in_bridge = 1

            if in_ring or in_bridge == 1:
                voxel_type[i, floor_y_offset, k] = CONCRETE
                voxel_type[i, floor_y_offset + 1, k] = EMPTY

    print(f"YIN-YANG ARENA constructed - ring (outer {outer_radius}, inner {inner_radius}) with S-curve bridge")

@ti.kernel
def init_hourglass_arena():
    """
    HOURGLASS ARENA - Two triangles meeting at a narrow pinch point
    Forces close combat at the center waist, easy to knock off at the sides
    """
    # Clear floor layers only (Y=30-40 covers floor at 33, bowl perimeter up to ~36)
    for i, j, k in ti.ndrange(n_grid, (30, 41), n_grid):
        voxel_type[i, j, k] = EMPTY

    center_x = 64
    center_z = 64
    arena_length = 32  # Half-length from center to tip
    waist_width = 6  # Half-width at the narrow center
    tip_width = 24  # Half-width at the wide ends
    floor_y_offset = 33

    # Hourglass shape: width increases linearly from center to tips
    # At x=0: width = waist_width
    # At x=±arena_length: width = tip_width
    slope = (tip_width - waist_width) / arena_length

    for i in range(center_x - arena_length - 2, center_x + arena_length + 2):
        for k in range(center_z - tip_width - 2, center_z + tip_width + 2):
            dx = float(i - center_x)
            dz = float(k - center_z)

            # Distance from center along x-axis
            dist_x = ti.abs(dx)

            # Only within arena length
            if dist_x <= arena_length:
                # Calculate allowed width at this x position
                allowed_width = waist_width + slope * dist_x

                # Check if within the hourglass shape
                if ti.abs(dz) <= allowed_width:
                    voxel_type[i, floor_y_offset, k] = CONCRETE
                    voxel_type[i, floor_y_offset + 1, k] = EMPTY

    print(f"HOURGLASS ARENA constructed - waist {waist_width*2}, tips {tip_width*2}")

@ti.kernel
def init_square_bridge_arena():
    """
    SQUARE BRIDGE ARENA - Rectangular ring (perimeter) with long bridge through middle
    Like yin-yang but rectangular shape with straight bridge running the long way
    """
    # Clear floor layers only (Y=30-40 covers floor at 33, bowl perimeter up to ~36)
    for i, j, k in ti.ndrange(n_grid, (30, 41), n_grid):
        voxel_type[i, j, k] = EMPTY

    center_x = 64
    center_z = 64
    # Outer rectangle (long shape)
    outer_half_x = 38  # Long dimension (x-axis) - same as yin-yang
    outer_half_z = 25  # Short dimension (z-axis)
    # Inner rectangle (the hole)
    inner_half_x = 28  # Leave 10 voxel perimeter on sides
    inner_half_z = 15  # Leave 10 voxel perimeter on top/bottom
    # Bridge through the hole (runs LONG way - along x-axis)
    bridge_half_width = 5  # Width of bridge (z direction)
    floor_y_offset = 33

    for i in range(center_x - outer_half_x - 2, center_x + outer_half_x + 2):
        for k in range(center_z - outer_half_z - 2, center_z + outer_half_z + 2):
            dx = float(i - center_x)
            dz = float(k - center_z)

            # Check if within outer rectangle
            in_outer = ti.abs(dx) <= outer_half_x and ti.abs(dz) <= outer_half_z

            # Check if in inner hole
            in_hole = ti.abs(dx) <= inner_half_x and ti.abs(dz) <= inner_half_z

            # Check if on bridge (runs LONG way through hole along x-axis, narrow in z)
            on_bridge = ti.abs(dx) <= inner_half_x and ti.abs(dz) <= bridge_half_width

            # Place floor if: in outer bounds AND (NOT in hole OR on bridge)
            if in_outer and (not in_hole or on_bridge):
                voxel_type[i, floor_y_offset, k] = CONCRETE
                voxel_type[i, floor_y_offset + 1, k] = EMPTY

    print(f"SQUARE BRIDGE ARENA constructed - rectangular ring with long bridge")

# ============================================================
# BACKGROUND VOXEL ANIMATION SYSTEM
# ============================================================

@ti.kernel
def animate_background(time: ti.f32):
    """
    Animate all background voxels based on their animation type.
    Called once per frame. Updates brightness and offset fields.
    """
    for i in range(num_bg_voxels[None]):
        if bg_active[i] == 0:
            continue

        anim = bg_anim_type[i]

        # Skip static voxels entirely - they keep their init values (brightness=1, offset=0)
        if anim == BG_ANIM_NONE:
            continue

        phase = bg_phase[i]
        speed = bg_anim_speed[i]
        amplitude = bg_anim_amplitude[i]
        t = time * speed + phase

        # Default values
        bg_brightness[i] = 1.0
        bg_offset_x[i] = 0.0
        bg_offset_y[i] = 0.0

        if anim == BG_ANIM_TWINKLE:
            # Gentle base shimmer + occasional bright twinkle flash
            base = 0.85 + 0.1 * ti.sin(t)
            # Use two overlapping sin waves with different frequencies to create rare alignment peaks
            wave1 = ti.sin(time * speed * 0.7 + amplitude * 3.14)
            wave2 = ti.sin(time * speed * 1.1 + amplitude * 7.77)
            combined = wave1 * wave2  # Only peaks when both waves align (~1.0)
            # Sharp power curve so only the highest peaks create a visible flash
            flash = 0.0
            if combined > 0.7:
                flash = (combined - 0.7) / 0.3  # 0 to 1 ramp
                flash = flash * flash * 0.5  # squared for sharp spike, up to 0.5 extra brightness
            bg_brightness[i] = base + flash

        elif anim == BG_ANIM_CONSTELLATION:
            # Constellation stars: synced pulse per constellation + traveling sparkle on lines
            # phase = constellation_id offset (shared pulse timing)
            # amplitude = 0 for vertex star, >0 for line dot (position along edge 0-1)
            # speed = sparkle speed variation

            # Whole constellation breathes together
            constellation_pulse = 0.5 + 0.5 * ti.sin(time * 0.6 + phase)

            if amplitude < 0.01:
                # Vertex star — bright pulse
                bg_brightness[i] = 0.55 + 0.45 * constellation_pulse
            else:
                # Line dot — traveling sparkle wave along the edge
                sparkle = ti.sin(time * 2.0 + phase + amplitude * 6.28)
                sparkle = ti.max(0.0, sparkle)
                bg_brightness[i] = 0.15 + 0.45 * constellation_pulse + 0.4 * sparkle

        elif anim == BG_ANIM_COMET:
            # Giant comet: dome front + animated rings flowing backward into tail
            # amplitude >= 100: animated ring voxel (amplitude-100 = theta within ring)
            # amplitude < 100: static voxel (tip/spine)

            orbit_radius = 220.0
            orbit_y = 15.0
            dome_r = 4.5
            head_angle = time * 1.775

            if amplitude >= 99.0:
                # === ANIMATED RING: flows from dome front to tail end ===
                ring_theta = amplitude - 100.0
                cycle_offset = phase
                cycle_period = 8.0
                frac = ((time * 3.94 + cycle_offset) % cycle_period) / cycle_period

                # Trail position: starts AT the head, flows backward
                trail_pos = frac * 0.45

                # Ring radius shrinks slower (quadratic — stays wide longer before converging)
                shrink = (1.0 - frac) * (1.0 - frac)
                ring_r = dome_r * shrink

                lat = ring_r * ti.cos(ring_theta)
                vert = ring_r * ti.sin(ring_theta)

                voxel_angle = head_angle - trail_pos
                cos_va = ti.cos(voxel_angle)
                sin_va = ti.sin(voxel_angle)
                px = cos_va * (orbit_radius + lat)
                pz = sin_va * (orbit_radius + lat)
                py = orbit_y + vert

                bg_offset_x[i] = px - bg_positions[i].x
                bg_offset_z[i] = pz - bg_positions[i].z
                bg_offset_y[i] = py - bg_positions[i].y

                # Smooth fade along tail — cubic for gradual dimming
                fade = (1.0 - frac) * (1.0 - frac)
                shimmer = ti.sin(time * 5.0 + ring_theta * 3.0) * 0.1 * fade
                bright = fade + shimmer
                if bright < 0.03:
                    bg_brightness[i] = 0.0
                    bg_offset_y[i] = -200.0  # hide underground, no black flicker
                else:
                    bg_brightness[i] = bright
            else:
                # === STATIC: front tip or core spine ===
                trail_offset = phase
                voxel_angle = head_angle - trail_offset
                cos_va = ti.cos(voxel_angle)
                sin_va = ti.sin(voxel_angle)

                px = cos_va * (orbit_radius + amplitude)
                pz = sin_va * (orbit_radius + amplitude)
                py = orbit_y + speed  # vertical offset

                bg_offset_x[i] = px - bg_positions[i].x
                bg_offset_z[i] = pz - bg_positions[i].z
                bg_offset_y[i] = py - bg_positions[i].y

                shimmer = ti.sin(time * 4.0 - trail_offset * 20.0)
                shimmer = ti.max(0.0, shimmer) * 0.3
                fade = 1.0 - trail_offset * 2.2
                bg_brightness[i] = ti.max(0.05, fade + shimmer)

        elif anim == BG_ANIM_SWAMP_BUBBLE:
            # Tar eruption: idle, rumble violently, then explosive burst
            # phase = shared pool timing, amplitude = eruption height, speed = unique voxel seed
            erupt_period = 10.0
            t_cycle = (time + phase) % erupt_period
            # 0-6: idle | 6-7.5: violent rumble | 7.5-10: explosive burst

            if t_cycle < 7.5:
                # IDLE: hidden underground
                bg_offset_x[i] = 0.0
                bg_offset_y[i] = -200.0
                bg_offset_z[i] = 0.0
                bg_brightness[i] = 0.0
            else:
                # EXPLOSIVE BURST: fast upward arc, fall back through floor
                erupt_t = t_cycle - 7.5  # 0 to 2.5
                normalized = erupt_t / 2.5  # 0 to 1
                # Parabolic arc: up fast then gravity pulls down past floor (20% higher)
                up = amplitude * 1.2 * (normalized * 2.5 - normalized * normalized * 3.0)
                # Each voxel sprays in unique direction
                spray_angle = speed * 0.37
                spray_r = normalized * 6.0
                bg_offset_x[i] = spray_r * ti.sin(spray_angle)
                bg_offset_z[i] = spray_r * ti.cos(spray_angle)
                bg_offset_y[i] = up
                # Stay visible while falling, only hide when well below floor
                if up < -8.0:
                    bg_offset_y[i] = -200.0
                    bg_brightness[i] = 0.0
                else:
                    bg_brightness[i] = 1.0

        elif anim == BG_ANIM_SPORE_DRIFT:
            # Spore: float up from cap, mushroom shakes, spores fly away, respawn
            # phase = stagger, amplitude = rise distance, speed = unique seed
            spore_period = 15.0
            t_spore = (time + phase) % spore_period
            # 0-8: float upward | 8-12: fly away smoothly | 12-15: respawn

            if t_spore < 8.0:
                # FLOATING: lazy upward drift with Lissajous wandering
                float_norm = t_spore / 8.0  # 0 to 1
                bg_offset_y[i] = float_norm * amplitude * 0.5
                wander_x = 3.0 * ti.sin(time * 0.2 + speed * 4.0)
                wander_z = 2.5 * ti.cos(time * 0.15 + speed * 3.0 + 1.0)
                bg_offset_x[i] = wander_x
                bg_offset_z[i] = wander_z
                fade_in = ti.min(1.0, float_norm * 5.0)
                fade_out = ti.min(1.0, (1.0 - float_norm) * 3.0)
                bg_brightness[i] = 1.2 + 0.6 * fade_in * fade_out
            elif t_spore < 12.0:
                # FLY AWAY: smooth detach and swoosh outward with lift + swirl
                fly_t = t_spore - 8.0  # 0 to 4
                float_y = amplitude * 0.5  # Start from top of float
                # Gust direction per cycle
                gust_num = ti.floor((time + phase) / spore_period)
                gust_dx = ti.sin(gust_num * 5.37 + speed * 0.1)
                gust_dz = ti.cos(gust_num * 5.37 + speed * 0.1)
                # Accelerating push outward
                push = fly_t * fly_t * 8.0
                lift = fly_t * 4.0
                # Swirl
                swirl_r = fly_t * 2.5
                swirl_x = swirl_r * ti.sin(time * 4.0 + speed * 3.0)
                swirl_z = swirl_r * ti.cos(time * 4.0 + speed * 3.0)
                perp_dx = -gust_dz
                perp_dz = gust_dx
                bg_offset_x[i] = push * gust_dx + swirl_x * perp_dx
                bg_offset_z[i] = push * gust_dz + swirl_x * perp_dz
                bg_offset_y[i] = float_y + lift + swirl_z
                bg_brightness[i] = 1.8
            else:
                # RESPAWN: fade back in at cap
                respawn_t = (t_spore - 12.0) / 3.0  # 0 to 1
                smooth = respawn_t * respawn_t * (3.0 - 2.0 * respawn_t)
                bg_offset_x[i] = 0.0
                bg_offset_y[i] = 0.0
                bg_offset_z[i] = 0.0
                bg_brightness[i] = smooth * 1.5

        elif anim == BG_ANIM_GLOW_PULSE:
            # Mushroom cap bioluminescent glow: smooth slow brightness pulse
            pulse = ti.sin(t * 0.8)
            bg_brightness[i] = 1.0 + amplitude * pulse * 0.8
            bg_offset_x[i] = 0.0
            bg_offset_y[i] = 0.0
            bg_offset_z[i] = 0.0

        elif anim == BG_ANIM_CATERPILLAR:
            # Giant caterpillar: slow orbit with inchworm body wave
            # amplitude = segment index (0=head, 1-12=body, 13+=antennae chain)
            # phase = caterpillar ID, speed = orbit speed
            seg = amplitude
            orbit_speed_c = speed
            orbit_radius_c = 58.0
            ground_y = 20.5  # Just above grass
            orbit_angle_c = time * orbit_speed_c + phase * 6.28
            seg_spacing = 2.8

            # Direction (tangent to circle)
            dir_x = -ti.sin(orbit_angle_c)
            dir_z = ti.cos(orbit_angle_c)
            perp_x = -dir_z
            perp_z = dir_x
            cx_c = orbit_radius_c * dir_z   # = orbit_radius_c * cos(orbit_angle_c)
            cz_c = -orbit_radius_c * dir_x  # = orbit_radius_c * sin(orbit_angle_c)

            # Inchworm wave: big exaggerated humps
            inch_wave = ti.sin(time * 2.5 - seg * 0.6 + phase * 3.0)
            bunch = inch_wave * 1.5  # stronger compression
            hump = ti.max(0.0, inch_wave) * 5.0  # big dramatic humps

            # Head position (shared by antennae)
            head_fwd = seg_spacing * 0.5
            head_hump = ti.max(0.0, ti.sin(time * 2.5 + phase * 3.0)) * 4.0
            head_px = cx_c + dir_x * head_fwd
            head_pz = cz_c + dir_z * head_fwd
            head_py = ground_y + head_hump

            if seg < 0.5:
                # HEAD
                bg_offset_x[i] = head_px
                bg_offset_y[i] = head_py
                bg_offset_z[i] = head_pz
            elif seg < 12.5:
                # BODY SEGMENTS
                body_idx = seg - 1.0
                trail = -(body_idx * seg_spacing + bunch)
                px = cx_c + dir_x * trail
                pz = cz_c + dir_z * trail
                py = ground_y + hump
                bg_offset_x[i] = px
                bg_offset_y[i] = py
                bg_offset_z[i] = pz
            else:
                # ANTENNAE CHAIN: segments 13-18=left, 19-24=right
                ant_local = seg - 13.0  # 0-11
                is_right = 0.0
                chain_idx = ant_local
                if ant_local >= 6.0:
                    is_right = 1.0
                    chain_idx = ant_local - 6.0
                ant_side = -1.0 + is_right * 2.0  # -1 or 1

                # Chain built link-by-link from head so segments stay connected
                # Start ahead of head so first link clears the body
                cx = head_px + dir_x * 3.0 + perp_x * ant_side * 1.5
                cy = head_py + 3.0
                cz = head_pz + dir_z * 3.0 + perp_z * ant_side * 1.5
                # Head phase: sin > 0 = head up, sin crossing 0 going neg = landing
                head_phase = ti.sin(time * 2.5 + phase * 3.0)
                for k in range(ti.static(6)):
                    if k <= chain_idx:
                        # Delayed head phase — scrunch propagates from base to tip
                        wave_k = ti.sin(time * 2.5 + phase * 3.0 - k * 0.3)
                        sway_k = ti.sin(time * 2.5 + k * 0.8 + is_right * 2.0)
                        # When wave_k < 0 (head landing), step_up shrinks = scrunch
                        step_fwd = 0.6 + wave_k * 0.2
                        step_up = 0.7 + wave_k * 0.5
                        step_side = ant_side * (0.4 + sway_k * 0.25)
                        cx += dir_x * step_fwd + perp_x * step_side
                        cz += dir_z * step_fwd + perp_z * step_side
                        cy += step_up
                px = cx
                py = cy
                pz = cz
                bg_offset_x[i] = px
                bg_offset_y[i] = py
                bg_offset_z[i] = pz

            bg_brightness[i] = 1.0

        elif anim == BG_ANIM_LAVA_SNAIL:
            # Molten lava snail: slow orbit, slug crawl, spiral shell, eye stalks, drips
            # amplitude = part index, phase = snail ID, speed = orbit speed
            part = amplitude
            orbit_speed_s = speed
            orbit_radius_s = 60.0
            lava_ground = 20.5
            orbit_angle_s = time * orbit_speed_s + phase * 6.28

            dir_x = -ti.sin(orbit_angle_s)
            dir_z = ti.cos(orbit_angle_s)
            perp_x = -dir_z
            perp_z = dir_x
            cx_s = orbit_radius_s * dir_z   # = orbit_radius_s * cos(orbit_angle_s)
            cz_s = -orbit_radius_s * dir_x  # = orbit_radius_s * sin(orbit_angle_s)

            # Slug crawl wave (flatter than caterpillar)
            slug_wave = ti.sin(time * 1.8 - part * 0.5 + phase * 3.0)
            bunch = slug_wave * 0.6
            hump = ti.max(0.0, slug_wave) * 1.5

            if part < 12.5:
                # BODY SEGMENTS (0=head, 1-12=body, widest at middle, tapers at ends)
                body_idx = part
                trail = -(body_idx * 2.5 + bunch)
                px = cx_s + dir_x * trail
                pz = cz_s + dir_z * trail
                py = lava_ground + hump
                bg_offset_x[i] = px
                bg_offset_y[i] = py
                bg_offset_z[i] = pz
                # Glow pulse for body
                bg_brightness[i] = 0.8 + 0.2 * ti.sin(time * 1.5 + part * 0.7)
            elif part < 35.5:
                # SHELL: spiral voxels sitting on body segments 4-7 area
                shell_idx = part - 13.0  # 0 to 22
                # Logarithmic spiral
                spiral_angle = shell_idx * 0.55
                spiral_r = 3.0 + shell_idx * 0.6
                # Shell center is above body segment ~5
                shell_trail = -(5.0 * 2.5 + ti.sin(time * 1.8 - 5.0 * 0.5 + phase * 3.0) * 0.6)
                shell_cx = cx_s + dir_x * shell_trail
                shell_cz = cz_s + dir_z * shell_trail
                shell_hump = ti.max(0.0, ti.sin(time * 1.8 - 5.0 * 0.5 + phase * 3.0)) * 1.5
                shell_cy = lava_ground + shell_hump + 3.0

                # Vertical spiral: coils upward with lateral spread
                local_side = spiral_r * ti.cos(spiral_angle) * 0.5  # Lateral wobble
                local_up = spiral_r * ti.sin(spiral_angle) * 0.5 + spiral_r * 0.5  # Spirals upward

                px = shell_cx + dir_x * local_side
                pz = shell_cz + dir_z * local_side
                py = shell_cy + local_up

                # Shell rock with crawl
                rock = 0.5 * ti.sin(time * 1.8 + phase * 3.0)
                py = py + rock

                bg_offset_x[i] = px
                bg_offset_y[i] = py
                bg_offset_z[i] = pz
                # Molten glow: gentle pulse, highlights slightly brighter
                vein = 0.0
                if (shell_idx % 3.0 > 0.5) and (shell_idx % 3.0 < 1.5):
                    vein = 1.0
                if vein > 0.5:
                    bg_brightness[i] = 0.9 + 0.2 * ti.sin(time * 1.0 + shell_idx * 0.5)
                else:
                    bg_brightness[i] = 0.7 + 0.15 * ti.sin(time * 0.6 + shell_idx * 0.4)
            elif part < 45.5:
                # EYE STALKS: 36-40=left (5 voxels), 41-45=right (5 voxels)
                eye_local = part - 36.0  # 0-9
                is_right_e = 0.0
                chain_e = eye_local
                if eye_local >= 5.0:
                    is_right_e = 1.0
                    chain_e = eye_local - 5.0
                eye_side = -1.0 + is_right_e * 2.0
                chain_t = chain_e / 4.0  # 0 to 1 normalized

                # Head position (body seg 0)
                head_bunch = ti.sin(time * 1.8 + phase * 3.0) * 0.6
                head_hump_s = ti.max(0.0, ti.sin(time * 1.8 + phase * 3.0)) * 1.5
                head_px = cx_s + dir_x * (-head_bunch)
                head_pz = cz_s + dir_z * (-head_bunch)
                head_py = lava_ground + head_hump_s

                fwd_e = 0.5 + chain_e * 0.8
                spread_e = eye_side * (2.0 + chain_t * 0.8)
                rise_e = chain_e * 0.9
                wobble = (0.2 + chain_t * 0.5) * ti.sin(time * 2.5 + chain_e * 0.8 + is_right_e * 3.14)
                px = head_px + dir_x * fwd_e + perp_x * spread_e
                pz = head_pz + dir_z * fwd_e + perp_z * spread_e
                py = head_py + rise_e + wobble + 1.5
                bg_offset_x[i] = px
                bg_offset_y[i] = py
                bg_offset_z[i] = pz
                bg_brightness[i] = 1.0
            else:
                # LAVA DRIPS: fall off behind shell, sink into lava
                drip_idx = part - 46.0  # 0 to ~19
                drip_period = 8.0
                t_drip = (time + phase + drip_idx * 0.7) % drip_period

                # Shell tail position (behind the shell)
                shell_trail_d = -(6.0 * 2.5)
                drip_cx = cx_s + dir_x * shell_trail_d
                drip_cz = cz_s + dir_z * shell_trail_d

                if t_drip < 5.0:
                    # Hidden
                    bg_offset_x[i] = 0.0
                    bg_offset_y[i] = -200.0
                    bg_offset_z[i] = 0.0
                    bg_brightness[i] = 0.0
                else:
                    # Drip: detach and fall
                    fall_t = t_drip - 5.0  # 0 to 3
                    spray_a = drip_idx * 1.37
                    spray_r_d = fall_t * 1.5
                    px = drip_cx + spray_r_d * ti.sin(spray_a)
                    pz = drip_cz + spray_r_d * ti.cos(spray_a)
                    py = lava_ground + 5.0 - fall_t * fall_t * 1.5
                    bg_offset_x[i] = px
                    bg_offset_z[i] = pz
                    if py < lava_ground - 3.0:
                        bg_offset_y[i] = -200.0
                        bg_brightness[i] = 0.0
                    else:
                        bg_offset_y[i] = py
                        bg_brightness[i] = 1.0

        elif anim == BG_ANIM_DUST_DEVIL:
            # Desert dust devil: sand lifts from ground, chaotic funnel, blows far away
            # amplitude = particle index (0-374), phase = cyclone ID
            # Pre-computed at init: bg_positions.y = chaos1, bg_angle = chaos2
            p_id = amplitude
            sand_y_dd = 18.0

            # 22s lifecycle, staggered by phase — check FIRST for early skip
            cycle = (time + phase * 7.0) % 22.0

            if cycle >= 19.0:
                # HIDDEN: waiting to respawn — skip all expensive math
                bg_offset_x[i] = bg_positions[i][0]
                bg_offset_y[i] = -200.0
                bg_offset_z[i] = bg_positions[i][2]
                bg_brightness[i] = 0.0
            else:
                height_t = p_id / 374.0  # 0=ground, 1=top
                base_ang = p_id * 2.39996  # golden angle

                # Cyclone base position stored in bg_positions
                base_x = bg_positions[i][0]
                base_z = bg_positions[i][2]

                # Pre-computed chaos seeds (stored at init, no trig needed)
                chaos1 = bg_positions[i][1]
                chaos2 = bg_angle[i]

                # Drift center — wander wide with Lissajous, clamped to sand annulus
                drift_x = base_x + 22.0 * ti.sin(time * 0.3 + phase * 2.0) + 8.0 * ti.sin(time * 0.55 + phase * 5.0)
                drift_z = base_z + 22.0 * ti.cos(time * 0.25 + phase * 3.3) + 8.0 * ti.cos(time * 0.45 + phase * 4.1)
                drift_dist = ti.sqrt(drift_x * drift_x + drift_z * drift_z) + 0.001
                if drift_dist < 35.0:
                    drift_x = drift_x * 35.0 / drift_dist
                    drift_z = drift_z * 35.0 / drift_dist
                elif drift_dist > 55.0:
                    drift_x = drift_x * 55.0 / drift_dist
                    drift_z = drift_z * 55.0 / drift_dist
                cx_dd = drift_x
                cz_dd = drift_z

                # Funnel: narrow tip at ground, wide chaotic top
                funnel_r = 0.3 + height_t * height_t * 7.0
                funnel_h = height_t * 37.5
                spin_speed = 7.0 + height_t * 4.0  # fast spin, faster at top
                spin_ang = base_ang + time * spin_speed
                cos_spin = ti.cos(spin_ang)
                sin_spin = ti.sin(spin_ang)

                # Per-particle chaos wobble
                wobble_r = 1.2 * ti.sin(time * 3.1 + p_id * 0.37) * (0.3 + height_t * 0.7)
                wobble_h = 1.0 * ti.sin(time * 2.3 + p_id * 0.53)
                wobble_tang = 0.7 * ti.cos(time * 2.7 + p_id * 0.71)

                if cycle < 3.0:
                    # EMERGE: sand lifts off the floor surface into funnel
                    emerge_t = cycle / 3.0
                    smooth_e = emerge_t * emerge_t * (3.0 - 2.0 * emerge_t)

                    # Start: scattered on the sand surface
                    start_r = 2.0 + (p_id % 30) * 0.15
                    start_ang_e = base_ang + chaos1 * 2.0
                    start_x = cx_dd + start_r * ti.cos(start_ang_e)
                    start_z = cz_dd + start_r * ti.sin(start_ang_e)
                    start_y = sand_y_dd

                    # Target: funnel position
                    tgt_r = funnel_r + wobble_r
                    tgt_x = cx_dd + tgt_r * cos_spin + wobble_tang * sin_spin
                    tgt_z = cz_dd + tgt_r * sin_spin - wobble_tang * cos_spin
                    tgt_y = sand_y_dd + funnel_h + wobble_h

                    # Lower particles emerge first
                    p_delay = height_t * 0.7
                    local_t = ti.max(0.0, (smooth_e - p_delay) / (1.0 - p_delay + 0.001))

                    px = start_x * (1.0 - local_t) + tgt_x * local_t
                    pz = start_z * (1.0 - local_t) + tgt_z * local_t
                    py = start_y * (1.0 - local_t) + tgt_y * local_t

                    bg_offset_x[i] = px
                    bg_offset_y[i] = py
                    bg_offset_z[i] = pz
                    bg_brightness[i] = 1.0

                elif cycle < 12.0:
                    # ACTIVE: chaotic spinning funnel
                    act_r = funnel_r + wobble_r
                    px = cx_dd + act_r * cos_spin + wobble_tang * sin_spin
                    pz = cz_dd + act_r * sin_spin - wobble_tang * cos_spin
                    py = sand_y_dd + funnel_h + wobble_h

                    # Extra jitter for chaos
                    jit_x = 0.6 * ti.sin(time * 5.3 + p_id * 1.17)
                    jit_z = 0.6 * ti.cos(time * 4.7 + p_id * 1.31)
                    jit_y = 0.4 * ti.sin(time * 4.1 + p_id * 0.89)

                    bg_offset_x[i] = px + jit_x
                    bg_offset_y[i] = py + jit_y
                    bg_offset_z[i] = pz + jit_z
                    bg_brightness[i] = 1.0

                else:
                    # DISSIPATE: blow waaay far away in all directions
                    fly_t = cycle - 12.0  # 0 to 7 seconds

                    # Each particle gets unique gust direction
                    gust_ang_d = base_ang + chaos1 * 1.5 + chaos2 * 0.8
                    gust_dx = ti.cos(gust_ang_d)
                    gust_dz = ti.sin(gust_ang_d)
                    perp_dx = -gust_dz
                    perp_dz = gust_dx

                    # Snapshot of last funnel position
                    act_r = funnel_r + wobble_r
                    last_x = cx_dd + act_r * cos_spin
                    last_z = cz_dd + act_r * sin_spin
                    last_y = sand_y_dd + funnel_h

                    # Massive accelerating push into the distance
                    push = fly_t * fly_t * 24.0
                    lift = fly_t * 8.0
                    # Swirl around gust direction
                    swirl_r = fly_t * 4.8
                    swirl_x = swirl_r * ti.sin(time * 4.0 + p_id * 3.0)
                    swirl_z = swirl_r * ti.cos(time * 4.0 + p_id * 3.0)

                    px = last_x + gust_dx * push + swirl_x * perp_dx
                    pz = last_z + gust_dz * push + swirl_z * perp_dz
                    py = last_y + lift + swirl_z

                    bg_offset_x[i] = px
                    bg_offset_y[i] = py
                    bg_offset_z[i] = pz
                    bg_brightness[i] = 1.0

        elif anim == BG_ANIM_SCORPION:
            # Desert scorpion: ground orbit with crawl, tail curl, claw snaps, leg stride
            # amplitude = part index, phase = scorpion ID, speed = orbit speed
            part_sc = amplitude
            orbit_speed_sc = speed
            orbit_radius_sc = 55.0
            sand_ground_sc = 18.0
            orbit_angle_sc = time * orbit_speed_sc + phase * 6.28

            dir_x_sc = -ti.sin(orbit_angle_sc)
            dir_z_sc = ti.cos(orbit_angle_sc)
            perp_x_sc = -dir_z_sc
            perp_z_sc = dir_x_sc
            cx_sc = orbit_radius_sc * dir_z_sc   # = orbit_radius_sc * cos(orbit_angle_sc)
            cz_sc = -orbit_radius_sc * dir_x_sc  # = orbit_radius_sc * sin(orbit_angle_sc)

            # Flat crawl wave
            crawl_wave = ti.sin(time * 1.5 - part_sc * 0.4 + phase * 3.0)
            bunch_sc = crawl_wave * 0.4
            hump_sc = ti.max(0.0, crawl_wave) * 0.8

            px = 0.0
            py = -200.0
            pz = 0.0

            if part_sc < 6.5:
                # HEAD + BODY (0=head, 1-6=body)
                body_idx_sc = part_sc
                trail_sc = -(body_idx_sc * 2.8 + bunch_sc)
                px = cx_sc + dir_x_sc * trail_sc
                pz = cz_sc + dir_z_sc * trail_sc
                py = sand_ground_sc + hump_sc
                bg_offset_x[i] = px
                bg_offset_y[i] = py
                bg_offset_z[i] = pz
                bg_brightness[i] = 1.0

            elif part_sc < 13.0:
                # TAIL (7-11) + STINGER (12)
                tail_idx_sc = part_sc - 7.0  # 0 to 5
                is_stinger = 0.0
                if tail_idx_sc > 4.5:
                    is_stinger = 1.0

                # Tail anchor: behind body segment 6
                anchor_crawl = ti.sin(time * 1.5 - 6.0 * 0.4 + phase * 3.0)
                anchor_trail = -(6.0 * 2.8 + anchor_crawl * 0.4)
                anchor_px = cx_sc + dir_x_sc * anchor_trail
                anchor_pz = cz_sc + dir_z_sc * anchor_trail
                anchor_py = sand_ground_sc

                # J-curve: backward then upward then forward lean
                t_idx = ti.min(tail_idx_sc, 4.0)  # clamp for stinger
                back_sc = t_idx * 2.0
                up_sc = t_idx * t_idx * 0.8
                forward_lean = 0.0
                if t_idx > 3.0:
                    forward_lean = (t_idx - 3.0) * 1.5

                # Sway
                sway_sc = 0.3 * ti.sin(time * 0.8 + phase * 2.0) * (1.0 + t_idx * 0.3)

                px = anchor_px - dir_x_sc * back_sc + dir_x_sc * forward_lean
                pz = anchor_pz - dir_z_sc * back_sc + dir_z_sc * forward_lean
                py = anchor_py + up_sc + sway_sc

                # Stinger: offset down and forward from tail tip
                if is_stinger > 0.5:
                    py = py - 1.5
                    px = px + dir_x_sc * 1.0
                    pz = pz + dir_z_sc * 1.0

                bg_offset_x[i] = px
                bg_offset_y[i] = py
                bg_offset_z[i] = pz
                bg_brightness[i] = 1.0

            elif part_sc < 25.0:
                # CLAWS (13-18=left, 19-24=right)
                claw_local = part_sc - 13.0  # 0-11
                is_right_c = 0.0
                claw_idx = claw_local
                if claw_local >= 6.0:
                    is_right_c = 1.0
                    claw_idx = claw_local - 6.0
                claw_side = -1.0 + is_right_c * 2.0

                # Head position
                head_crawl = ti.sin(time * 1.5 + phase * 3.0)
                head_bunch_sc = head_crawl * 0.4
                head_hump_sc = ti.max(0.0, head_crawl) * 0.8
                head_px_sc = cx_sc + dir_x_sc * (-head_bunch_sc)
                head_pz_sc = cz_sc + dir_z_sc * (-head_bunch_sc)
                head_py_sc = sand_ground_sc + head_hump_sc

                # Pincer open/close cycle
                pinch_cycle = ti.sin(time * 0.5 + phase * 2.0 + is_right_c * 1.5)
                pinch = 0.5 + 0.5 * pinch_cycle  # 0=closed, 1=open

                if claw_idx < 4.0:
                    # Arm segments: extend forward and outward
                    fwd_c = 2.0 + claw_idx * 2.5
                    spread_c = claw_side * (3.0 + claw_idx * 1.0)
                    px = head_px_sc + dir_x_sc * fwd_c + perp_x_sc * spread_c
                    pz = head_pz_sc + dir_z_sc * fwd_c + perp_z_sc * spread_c
                    py = head_py_sc + 0.5
                elif claw_idx < 5.0:
                    # Upper jaw
                    fwd_c = 2.0 + 4.0 * 2.5 + 1.5
                    spread_c = claw_side * (3.0 + 4.0 * 1.0)
                    jaw_open = pinch * 1.2
                    px = head_px_sc + dir_x_sc * fwd_c + perp_x_sc * spread_c
                    pz = head_pz_sc + dir_z_sc * fwd_c + perp_z_sc * spread_c
                    py = head_py_sc + 0.5 + 0.8 + jaw_open
                else:
                    # Lower jaw
                    fwd_c = 2.0 + 4.0 * 2.5 + 1.5
                    spread_c = claw_side * (3.0 + 4.0 * 1.0)
                    jaw_open = pinch * 0.8
                    px = head_px_sc + dir_x_sc * fwd_c + perp_x_sc * spread_c
                    pz = head_pz_sc + dir_z_sc * fwd_c + perp_z_sc * spread_c
                    py = head_py_sc + 0.5 - 0.5 - jaw_open

                bg_offset_x[i] = px
                bg_offset_y[i] = py
                bg_offset_z[i] = pz
                bg_brightness[i] = 1.0

            else:
                # LEGS (25-32): 4 pairs, alternating stride
                leg_local = part_sc - 25.0  # 0-7
                pair_sc = ti.floor(leg_local / 2.0)  # 0-3
                is_right_leg = leg_local - pair_sc * 2.0  # 0 or 1
                leg_side = -1.0 + is_right_leg * 2.0

                # Attach to body segs 1-4
                body_seg_sc = pair_sc + 1.0
                seg_crawl = ti.sin(time * 1.5 - body_seg_sc * 0.4 + phase * 3.0)
                seg_trail = -(body_seg_sc * 2.8 + seg_crawl * 0.4)
                seg_hump_sc = ti.max(0.0, seg_crawl) * 0.8

                base_px_sc = cx_sc + dir_x_sc * seg_trail
                base_pz_sc = cz_sc + dir_z_sc * seg_trail
                base_py_sc = sand_ground_sc + seg_hump_sc

                # Alternating stride
                stride_phase = time * 3.0 + pair_sc * 1.57 + phase * 2.0
                stride_fwd = 0.8 * ti.sin(stride_phase)
                stride_up = ti.max(0.0, ti.sin(stride_phase)) * 0.5

                spread_leg = leg_side * 3.5
                px = base_px_sc + perp_x_sc * spread_leg + dir_x_sc * stride_fwd
                pz = base_pz_sc + perp_z_sc * spread_leg + dir_z_sc * stride_fwd
                py = base_py_sc - 0.5 + stride_up

                bg_offset_x[i] = px
                bg_offset_y[i] = py
                bg_offset_z[i] = pz
                bg_brightness[i] = 1.0

        elif anim == BG_ANIM_TOAD:
            # Fat swamp toad: hopping orbit with idle breathing, throat puff, stubby legs
            # amplitude = part index, phase = toad ID, speed = orbit speed
            # Parts: 0=body, 1=head, 2=throat, 3-4=eyes, 5-6=pupils,
            #   7-8=back upper legs, 9-10=back lower legs, 11-12=back feet,
            #   13-14=front upper legs, 15-16=front lower legs, 17-18=front feet
            toad_part = amplitude
            orbit_speed_t = speed
            orbit_radius_t = 55.0
            swamp_ground = 18.5

            # 5s hop cycle: 3s idle, 0.3s crouch, 0.7s airborne, 1s land
            hop_cycle = (time + phase * 3.7) % 5.0

            # Orbit advances in discrete hops (~35 degrees per hop)
            hop_count = ti.floor((time + phase * 3.7) / 5.0)
            hop_angle = hop_count * 0.6 + phase * 3.14  # 0.6 rad = ~35 degrees per hop
            next_angle = hop_angle + 0.6

            # Interpolate angle during airborne phase
            current_angle = hop_angle
            if hop_cycle >= 3.3 and hop_cycle < 4.0:
                jump_t = (hop_cycle - 3.3) / 0.7
                smooth_jt = jump_t * jump_t * (3.0 - 2.0 * jump_t)
                current_angle = hop_angle + (next_angle - hop_angle) * smooth_jt
            elif hop_cycle >= 4.0:
                current_angle = next_angle

            # Direction vectors
            dir_x_t = -ti.sin(current_angle)
            dir_z_t = ti.cos(current_angle)
            perp_x_t = -dir_z_t
            perp_z_t = dir_x_t

            # Center position on orbit (reuse direction vectors)
            cx_t = orbit_radius_t * dir_z_t   # = orbit_radius_t * cos(current_angle)
            cz_t = -orbit_radius_t * dir_x_t  # = orbit_radius_t * sin(current_angle)

            # Jump arc height — explosive fast arc
            jump_y = 0.0
            if hop_cycle >= 3.3 and hop_cycle < 4.0:
                jump_t2 = (hop_cycle - 3.3) / 0.7
                jump_y = ti.sin(jump_t2 * 3.14159) * 25.0  # Big explosive arc

            # Breathing pulse during idle (body scale wobble)
            breathe = 0.0
            if hop_cycle < 3.0:
                breathe = 0.15 * ti.sin(time * 1.8 + phase * 2.0)

            # Crouch compression — quick wind-up
            crouch_y = 0.0
            if hop_cycle >= 3.0 and hop_cycle < 3.3:
                crouch_t = (hop_cycle - 3.0) / 0.3
                crouch_y = -2.0 * ti.sin(crouch_t * 3.14159 * 0.5)

            # Land squish
            land_squish = 0.0
            if hop_cycle >= 4.0:
                land_t = (hop_cycle - 4.0) / 1.0
                land_squish = -1.5 * ti.sin(land_t * 3.14159) * (1.0 - land_t)

            # Throat puff (inflates during idle, 3s cycle)
            throat_puff = 0.0
            if hop_cycle < 3.0:
                throat_puff = 0.5 + 0.5 * ti.sin(time * 2.1 + phase * 4.0)

            # Idle sway — gentle rocking side to side and forward/back
            sway_x = 0.0
            sway_z = 0.0
            sway_y = 0.0
            if hop_cycle < 3.0:
                sway_x = perp_x_t * 0.8 * ti.sin(time * 0.9 + phase * 2.5) + dir_x_t * 0.4 * ti.sin(time * 0.7 + phase * 1.3)
                sway_z = perp_z_t * 0.8 * ti.sin(time * 0.9 + phase * 2.5) + dir_z_t * 0.4 * ti.sin(time * 0.7 + phase * 1.3)
                sway_y = 0.3 * ti.sin(time * 1.2 + phase * 3.0)

            body_y = swamp_ground + jump_y + crouch_y + land_squish + sway_y

            # Leg tuck during crouch
            leg_tuck = 0.0
            if hop_cycle >= 3.0 and hop_cycle < 3.3:
                leg_tuck = (hop_cycle - 3.0) / 0.3

            px_t = 0.0
            py_t = -200.0
            pz_t = 0.0

            if toad_part < 0.5:
                # BODY: big fat round center
                px_t = cx_t + sway_x
                py_t = body_y + breathe
                pz_t = cz_t + sway_z
            elif toad_part < 1.5:
                # HEAD: forward and slightly up
                px_t = cx_t + dir_x_t * 3.5 + sway_x * 1.2
                py_t = body_y + 1.0 + breathe * 0.5
                pz_t = cz_t + dir_z_t * 3.5 + sway_z * 1.2
            elif toad_part < 2.5:
                # THROAT POUCH: under head, inflates
                puff_size = 1.0 + throat_puff * 1.5
                px_t = cx_t + dir_x_t * 4.0 + sway_x * 1.3
                py_t = body_y - 0.5 - puff_size * 0.3
                pz_t = cz_t + dir_z_t * 4.0 + sway_z * 1.3
            elif toad_part < 4.5:
                # EYES: bulging on top of head (3=left, 4=right)
                eye_side = -1.0
                if toad_part > 3.5:
                    eye_side = 1.0
                px_t = cx_t + dir_x_t * 4.2 + perp_x_t * eye_side * 2.0 + sway_x * 1.3
                py_t = body_y + 4.5
                pz_t = cz_t + dir_z_t * 4.2 + perp_z_t * eye_side * 2.0 + sway_z * 1.3
            elif toad_part < 6.5:
                # PUPILS: on front of eyes (5=left, 6=right)
                pupil_side = -1.0
                if toad_part > 5.5:
                    pupil_side = 1.0
                px_t = cx_t + dir_x_t * 5.0 + perp_x_t * pupil_side * 2.0 + sway_x * 1.3
                py_t = body_y + 4.7
                pz_t = cz_t + dir_z_t * 5.0 + perp_z_t * pupil_side * 2.0 + sway_z * 1.3
            elif toad_part < 22.5:
                # BACK LEGS: 7-14=left (8 voxels), 15-22=right (8 voxels)
                # Per leg: 0-2=thigh chain, 3-5=shin chain, 6-7=foot/toes
                bleg_idx = toad_part - 7.0  # 0-15
                bleg_side = -1.0
                bleg_local = bleg_idx
                if bleg_idx >= 8.0:
                    bleg_side = 1.0
                    bleg_local = bleg_idx - 8.0

                spread_b = bleg_side * 4.5

                # Toad back legs: thigh angles UP to knee, shin angles DOWN to ground
                # Idle: knee sticks up high (classic toad crouch), Z-shape
                # thigh_up = how much thigh goes UP from hip (positive = up)
                # thigh_back = how far back from hip
                # shin_fwd = how far forward shin goes from knee
                thigh_up = 5.5     # Knee sticks up high above body
                thigh_back = -2.0  # Slightly back
                shin_down = -8.0   # Long shin reaching to ground
                shin_fwd = 2.0     # Shin angles forward

                if hop_cycle >= 3.0 and hop_cycle < 3.3:
                    # Crouch: knee even higher, coil tight
                    coil_t = (hop_cycle - 3.0) / 0.3
                    thigh_up = 5.5 + coil_t * 2.5
                    thigh_back = -2.0 - coil_t * 1.5
                    shin_down = -8.0 - coil_t * 3.0
                    shin_fwd = 2.0 - coil_t * 1.5
                elif hop_cycle >= 3.3 and hop_cycle < 3.7:
                    # Jump launch: legs EXTEND explosively behind and down
                    ext_t = (hop_cycle - 3.3) / 0.4
                    smooth_ext = ext_t * ext_t * (3.0 - 2.0 * ext_t)
                    thigh_up = 8.0 - smooth_ext * 10.0
                    thigh_back = -3.5 - smooth_ext * 5.0
                    shin_down = -11.0 + smooth_ext * 8.0
                    shin_fwd = 0.5 - smooth_ext * 4.0
                elif hop_cycle >= 3.7 and hop_cycle < 4.0:
                    # Airborne: tuck legs under body
                    tuck_t = (hop_cycle - 3.7) / 0.3
                    thigh_up = -2.0 + tuck_t * 4.0
                    thigh_back = -8.5 + tuck_t * 5.0
                    shin_down = -3.0 - tuck_t * 2.5
                    shin_fwd = -3.5 + tuck_t * 3.5
                elif hop_cycle >= 4.0:
                    # Landing: absorb, return to toad crouch
                    land_t2 = (hop_cycle - 4.0) / 1.0
                    smooth_land = land_t2 * land_t2 * (3.0 - 2.0 * land_t2)
                    thigh_up = 2.0 + smooth_land * 3.5
                    thigh_back = -3.5 + smooth_land * 1.5
                    shin_down = -5.5 - smooth_land * 2.5
                    shin_fwd = 0.0 + smooth_land * 2.0

                # Hip: behind and below body (sways with body)
                hip_x = cx_t + dir_x_t * (-3.5) + perp_x_t * spread_b + sway_x * 0.5
                hip_y = body_y - 1.5
                hip_z = cz_t + dir_z_t * (-3.5) + perp_z_t * spread_b + sway_z * 0.5

                # Knee position: up and back from hip
                knee_x = hip_x + dir_x_t * thigh_back + perp_x_t * bleg_side * 0.5
                knee_y = hip_y + thigh_up
                knee_z = hip_z + dir_z_t * thigh_back + perp_z_t * bleg_side * 0.5

                # Ankle position: down and forward from knee
                ankle_x = knee_x + dir_x_t * shin_fwd
                ankle_y = knee_y + shin_down
                ankle_z = knee_z + dir_z_t * shin_fwd

                seg_sp = 1.8  # Spacing between voxels in chain

                if bleg_local < 3.0:
                    # THIGH chain: hip → knee (3 voxels)
                    chain_t = bleg_local / 2.0  # 0, 0.5, 1.0
                    px_t = hip_x + (knee_x - hip_x) * chain_t
                    py_t = hip_y + (knee_y - hip_y) * chain_t
                    pz_t = hip_z + (knee_z - hip_z) * chain_t
                elif bleg_local < 6.0:
                    # SHIN chain: knee → ankle (3 voxels)
                    chain_s = (bleg_local - 3.0) / 2.0  # 0, 0.5, 1.0
                    px_t = knee_x + (ankle_x - knee_x) * chain_s
                    py_t = knee_y + (ankle_y - knee_y) * chain_s
                    pz_t = knee_z + (ankle_z - knee_z) * chain_s
                else:
                    # FOOT/TOES: splayed from ankle (2 voxels)
                    toe_idx = bleg_local - 6.0  # 0 or 1
                    toe_splay = -0.8 + toe_idx * 1.6
                    px_t = ankle_x + dir_x_t * 1.2 + perp_x_t * bleg_side * toe_splay
                    py_t = ti.max(ankle_y, swamp_ground - 1.0 + jump_y * 0.2)
                    pz_t = ankle_z + dir_z_t * 1.2 + perp_z_t * bleg_side * toe_splay

            elif toad_part < 34.5:
                # FRONT LEGS: 23-28=left (6 voxels), 29-34=right (6 voxels)
                # Per leg: 0-1=upper arm, 2-3=forearm, 4-5=hand/toes
                fleg_idx2 = toad_part - 23.0  # 0-11
                fleg_side2 = -1.0
                fleg_local = fleg_idx2
                if fleg_idx2 >= 6.0:
                    fleg_side2 = 1.0
                    fleg_local = fleg_idx2 - 6.0

                spread_f = fleg_side2 * 3.5

                # Animated elbow angle: how far the forearm drops
                # Idle: 90-degree right angle — upper arm horizontal, forearm vertical
                elbow_drop = 5.0  # How far forearm goes straight down
                arm_fwd = 3.0     # Upper arm reach forward
                arm_out = 1.5     # Upper arm pushes outward

                if hop_cycle >= 3.0 and hop_cycle < 3.3:
                    # Crouch: arms brace wider
                    ct_f = (hop_cycle - 3.0) / 0.3
                    arm_out = 1.5 + ct_f * 1.0
                    elbow_drop = 5.0 + ct_f * 1.0
                elif hop_cycle >= 3.3 and hop_cycle < 4.0:
                    # Jump: arms reach forward and out
                    jt_f = (hop_cycle - 3.3) / 0.7
                    jump_f = ti.sin(jt_f * 3.14159)
                    arm_fwd = 3.0 + jump_f * 3.0
                    arm_out = 2.5 - jump_f * 1.0
                    elbow_drop = 6.0 - jump_f * 3.0  # Arms extend out straighter
                elif hop_cycle >= 4.0:
                    # Landing: brace hard, arms lock
                    lt_f = (hop_cycle - 4.0) / 1.0
                    smooth_lf = lt_f * lt_f * (3.0 - 2.0 * lt_f)
                    arm_fwd = 3.0 + (1.0 - smooth_lf) * 1.5
                    arm_out = 1.5 + (1.0 - smooth_lf) * 0.5
                    elbow_drop = 5.0 + (1.0 - smooth_lf) * 1.5

                # Shoulder: on body, forward and out (sways with body)
                shldr_x = cx_t + dir_x_t * 3.0 + perp_x_t * spread_f + sway_x * 0.5
                shldr_y = body_y - 1.0
                shldr_z = cz_t + dir_z_t * 3.0 + perp_z_t * spread_f + sway_z * 0.5

                # Elbow: forward + outward from shoulder (horizontal upper arm)
                elbow_x = shldr_x + dir_x_t * arm_fwd + perp_x_t * fleg_side2 * arm_out
                elbow_y = shldr_y - 0.5  # Slight droop
                elbow_z = shldr_z + dir_z_t * arm_fwd + perp_z_t * fleg_side2 * arm_out

                # Wrist: straight down from elbow (vertical forearm)
                wrist_x = elbow_x + dir_x_t * 0.3
                wrist_y = elbow_y - elbow_drop
                wrist_z = elbow_z + dir_z_t * 0.3

                if fleg_local < 2.0:
                    # UPPER ARM (2 voxels: shoulder → elbow)
                    fu = fleg_local / 1.0  # 0, 1
                    px_t = shldr_x + (elbow_x - shldr_x) * fu
                    py_t = shldr_y + (elbow_y - shldr_y) * fu
                    pz_t = shldr_z + (elbow_z - shldr_z) * fu
                elif fleg_local < 4.0:
                    # FOREARM (2 voxels: elbow → wrist, straight down)
                    ff = (fleg_local - 2.0) / 1.0  # 0, 1
                    px_t = elbow_x + (wrist_x - elbow_x) * ff
                    py_t = elbow_y + (wrist_y - elbow_y) * ff
                    pz_t = elbow_z + (wrist_z - elbow_z) * ff
                else:
                    # HAND/TOES (2 voxels splayed from wrist)
                    toe_f = fleg_local - 4.0  # 0 or 1
                    toe_sp = -0.6 + toe_f * 1.2
                    px_t = wrist_x + dir_x_t * 0.8 + perp_x_t * fleg_side2 * toe_sp
                    py_t = ti.max(wrist_y, swamp_ground - 1.0 + jump_y * 0.1)
                    pz_t = wrist_z + dir_z_t * 0.8 + perp_z_t * fleg_side2 * toe_sp

            elif toad_part < 40.5:
                # WARTS: bumps on body surface (parts 35-40)
                wart_idx = toad_part - 35.0  # 0-5
                wart_ang = wart_idx * 1.05 + phase * 2.0
                wart_fwd = -1.0 + ti.sin(wart_ang * 2.3) * 2.5
                wart_side = ti.cos(wart_ang * 1.7) * 3.0
                wart_up = 0.5 + ti.abs(ti.sin(wart_ang * 3.1)) * 1.5
                px_t = cx_t + dir_x_t * wart_fwd + perp_x_t * wart_side
                py_t = body_y + wart_up + breathe * 0.5
                pz_t = cz_t + dir_z_t * wart_fwd + perp_z_t * wart_side
            else:
                # TONGUE: parts 41-48, shoots out every 3rd jump mid-air
                tongue_idx = toad_part - 41.0  # 0-7 (tip=7)
                tongue_len = 8.0
                hop_count_t = ti.floor((time + phase * 3.7) / 5.0)
                is_tongue_jump = (hop_count_t % 3.0)  # 0 = tongue jump

                # Head position for tongue anchor
                head_x_t = cx_t + dir_x_t * 3.5 + sway_x * 1.2
                head_y_t = body_y + 1.0
                head_z_t = cz_t + dir_z_t * 3.5 + sway_z * 1.2

                # Tongue active during airborne phase (3.3-4.0) on every 3rd jump
                if is_tongue_jump < 0.5 and hop_cycle >= 3.3 and hop_cycle < 4.0:
                    tongue_t = (hop_cycle - 3.3) / 0.7  # 0→1 over airborne

                    # Shoot out first half, retract second half
                    extend = 0.0
                    if tongue_t < 0.4:
                        # Shoot out fast
                        ext_f = tongue_t / 0.4
                        extend = ext_f * ext_f * (3.0 - 2.0 * ext_f)  # smoothstep
                    elif tongue_t < 0.6:
                        # Hold extended
                        extend = 1.0
                    else:
                        # Retract
                        ret_f = (tongue_t - 0.6) / 0.4
                        extend = 1.0 - ret_f * ret_f * (3.0 - 2.0 * ret_f)

                    # Max tongue reach
                    max_reach = 15.0
                    chain_pos = (tongue_idx + 1.0) / tongue_len  # 0.125 to 1.0
                    reach = extend * max_reach * chain_pos

                    # Subtle wiggle that travels along the tongue
                    wiggle_y = ti.sin(chain_pos * 6.28 + time * 12.0) * 0.4 * extend * chain_pos
                    wiggle_x = ti.cos(chain_pos * 6.28 + time * 10.0) * 0.3 * extend * chain_pos

                    px_t = head_x_t + dir_x_t * reach + perp_x_t * wiggle_x
                    py_t = head_y_t + 0.5 + wiggle_y
                    pz_t = head_z_t + dir_z_t * reach + perp_z_t * wiggle_x
                else:
                    # Hidden when not active
                    px_t = 0.0
                    py_t = -200.0
                    pz_t = 0.0

            bg_offset_x[i] = px_t
            bg_offset_y[i] = py_t
            bg_offset_z[i] = pz_t
            bg_brightness[i] = 1.0

        elif anim == BG_ANIM_MUD_SPLASH:
            # Mud splash burst when toad lands
            # amplitude = splash particle index (0-15), phase = toad ID, speed = unused
            sp_idx = amplitude
            num_sp = 16.0
            orbit_radius_ms = 55.0
            swamp_y_ms = 18.5

            # Absolute time since last landing (not wrapped by cycle)
            toad_time = time + phase * 3.7
            hop_count_ms = ti.floor(toad_time / 5.0)
            last_land_time = hop_count_ms * 5.0 + 3.9  # Landing happens at 3.9 in each cycle
            splash_t_ms = toad_time - last_land_time

            # If negative, landing hasn't happened yet this cycle — use previous cycle's landing
            if splash_t_ms < 0.0:
                hop_count_ms = hop_count_ms - 1.0
                last_land_time = hop_count_ms * 5.0 + 3.9
                splash_t_ms = toad_time - last_land_time

            # Landing position (where toad landed)
            hop_angle_ms = hop_count_ms * 0.6 + phase * 3.14
            land_angle_ms = hop_angle_ms + 0.6
            land_x = orbit_radius_ms * ti.cos(land_angle_ms)
            land_z = orbit_radius_ms * ti.sin(land_angle_ms)

            if splash_t_ms >= 0.0 and splash_t_ms < 4.0:
                # Burst ring pattern — start outside the body (radius 6)
                burst_ang = sp_idx * (6.28318 / num_sp)
                burst_dx_ms = ti.cos(burst_ang)
                burst_dz_ms = ti.sin(burst_ang)

                # Start offset: outside toad body edge
                start_r = 6.0

                # Burst speed with variation (reduced to stay close)
                burst_spd_ms = 10.0 + ti.sin(sp_idx * 2.3) * 4.0

                # Upward velocity — mud flies up
                up_spd_ms = 22.0 + ti.cos(sp_idx * 1.7) * 8.0

                # Time when this particle hits ground: up/gravity
                gravity_ms = 28.0
                t_ground = up_spd_ms / gravity_ms

                # Clamp horizontal movement time so XZ freezes at landing
                t_horiz = ti.min(splash_t_ms, t_ground)

                # Parabolic arc for Y
                sp_py = swamp_y_ms + up_spd_ms * splash_t_ms - gravity_ms * splash_t_ms * splash_t_ms

                # XZ position (freezes when particle hits ground)
                sp_px = land_x + burst_dx_ms * (start_r + burst_spd_ms * t_horiz)
                sp_pz = land_z + burst_dz_ms * (start_r + burst_spd_ms * t_horiz)

                if sp_py >= swamp_y_ms:
                    # Above ground: normal flight
                    bg_offset_x[i] = sp_px
                    bg_offset_y[i] = sp_py
                    bg_offset_z[i] = sp_pz
                    bg_brightness[i] = 1.0
                else:
                    # Below ground: slow sink into mud
                    time_below = splash_t_ms - t_ground
                    if time_below < 0.0:
                        time_below = 0.0
                    sink_y = swamp_y_ms - time_below * 2.0
                    if sink_y > swamp_y_ms - 8.0:
                        bg_offset_x[i] = sp_px
                        bg_offset_y[i] = sink_y
                        bg_offset_z[i] = sp_pz
                        sink_frac = time_below * 2.0 / 8.0
                        bg_brightness[i] = ti.max(0.0, 1.0 - sink_frac)
                    else:
                        bg_offset_y[i] = -200.0
                        bg_brightness[i] = 0.0
            else:
                # Hidden between splashes
                bg_offset_y[i] = -200.0
                bg_brightness[i] = 0.0

        elif anim == BG_ANIM_SWAY:
            # Horizontal wave motion (grass swaying) with gust waves
            # Gust modulation - slow wave that increases/decreases intensity
            gust = 0.5 + 0.5 * ti.sin(t * 0.3 + phase * 0.5)  # Slow gust wave
            gust_amp = amplitude * (0.4 + 0.6 * gust)  # Range from 40% to 100% amplitude
            sway_ox = gust_amp * ti.sin(t)
            sway_oy = gust_amp * 0.3 * ti.sin(t * 1.5 + 0.5)

            # Wind gust lean (synced with BG_ANIM_FLOWER cycle)
            gust_period_s = 33.0
            gust_cycle_s = time % gust_period_s
            gust_num_s = ti.floor(time / gust_period_s)
            gust_dx_s = ti.sin(gust_num_s * 7.13)
            gust_dz_s = ti.cos(gust_num_s * 7.13)

            wind_lean = 0.0
            wind_shake = 0.0
            if gust_cycle_s > 18.0 and gust_cycle_s < 22.0:
                # Smooth lean: ramp up 18-19, hold 19-21, ramp down 21-22
                if gust_cycle_s < 19.0:
                    wind_lean = (gust_cycle_s - 18.0) * amplitude * 0.8
                elif gust_cycle_s < 21.0:
                    wind_lean = amplitude * 0.8
                else:
                    wind_lean = (22.0 - gust_cycle_s) * amplitude * 0.8
                # Rapid shaking during wind
                wind_shake = wind_lean * 0.4 * ti.sin(time * 15.0 + phase * 3.0)

            bg_offset_x[i] = sway_ox + wind_lean * gust_dx_s + wind_shake * gust_dz_s
            bg_offset_y[i] = sway_oy
            bg_offset_z[i] = wind_lean * gust_dz_s - wind_shake * gust_dx_s

        elif anim == BG_ANIM_DRIFT:
            # Slow position drift (clouds, ambient particles)
            bg_offset_x[i] = amplitude * ti.sin(t * 0.3)
            bg_offset_y[i] = amplitude * 0.5 * ti.cos(t * 0.2)

        elif anim == BG_ANIM_FLICKER:
            # Random-ish flicker using sin of different frequencies
            flicker = ti.sin(t * 7.0) * ti.sin(t * 11.0) * ti.sin(t * 13.0)
            bg_brightness[i] = 0.5 + 0.5 * ti.max(0.0, flicker)

        elif anim == BG_ANIM_FIREFLY:
            # Firefly: wandering path + realistic flash (on/off blink)
            # Bigger wandering path
            bg_offset_x[i] = amplitude * 1.5 * ti.sin(t * 0.5) * ti.cos(t * 0.25)
            bg_offset_y[i] = amplitude * ti.sin(t * 0.4) * 0.8
            bg_offset_z[i] = amplitude * 0.8 * ti.sin(t * 0.35 + 1.5)

            # Firefly flash: gradual glow that pulses smoothly
            flash_cycle = ti.sin(t * 0.8 + phase * 2.0)
            # Map -1 to 1 range to 0.35 to 1.0 (never fully dark)
            bg_brightness[i] = 0.35 + 0.65 * (flash_cycle * 0.5 + 0.5)

        elif anim == BG_ANIM_WATER:
            # Water: Whirlpool - spiral into center, respawn at edge
            # phase = initial angle, amplitude = initial radius, speed = speed multiplier
            initial_angle = phase
            initial_radius = amplitude
            speed_mult = speed

            # Whirlpool parameters
            outer_radius = 55.0
            inner_radius = 5.0
            radius_range = outer_radius - inner_radius
            rotation_speed = 0.25  # Slow rotation
            inward_speed = 3.0  # Slow inward drift

            # Current angle - rotates over time
            current_angle = initial_angle + t * rotation_speed * speed_mult

            # Current radius - shrinks over time, wraps back to outer
            time_offset = initial_radius / radius_range  # Stagger based on start pos
            radius_cycle = (t * inward_speed / radius_range + time_offset) % 1.0
            current_radius = outer_radius - radius_cycle * radius_range

            # Convert polar to cartesian
            new_x = ti.cos(current_angle) * current_radius
            new_z = ti.sin(current_angle) * current_radius

            # Offset from original position
            orig_x = ti.cos(initial_angle) * initial_radius
            orig_z = ti.sin(initial_angle) * initial_radius
            bg_offset_x[i] = new_x - orig_x
            bg_offset_z[i] = new_z - orig_z

            # Descend as approaching center (like a drain)
            depth = (1.0 - current_radius / outer_radius) * 8.0
            bg_offset_y[i] = -depth

            # Subtle breathing effect - slow global pulse
            breathe = 0.75 + 0.25 * ti.sin(t * 0.12)
            bg_brightness[i] = breathe

        elif anim == BG_ANIM_JELLYFISH:
            # Jellyfish: pulsing float with tentacle lag
            # phase = unique ID for this jellyfish
            # amplitude = vertical offset within jellyfish (0=bell, 1+=tentacles)
            # speed = base Y position
            jelly_id = phase
            part_type = amplitude  # 0=bell, 1-3=tentacles
            base_y = speed

            # Vertical bob - hypnotic wave effect
            bob_speed = 0.075
            bob = 2.0 * ti.sin(t * bob_speed + jelly_id * 0.5)

            # Pulse effect - hypnotic wave
            pulse_speed = 0.1
            pulse = ti.sin(t * pulse_speed + jelly_id * 0.3)

            # Bell movement
            if part_type < 0.5:
                # Bell - direct movement
                bg_offset_y[i] = bob
                bg_offset_x[i] = pulse * 0.3
                bg_offset_z[i] = 0.0
            else:
                # Tentacles - lag behind bell, sway more
                lag = 0.3 * part_type
                bg_offset_y[i] = bob - lag - part_type * 0.5
                bg_offset_x[i] = pulse * (0.5 + part_type * 0.3)
                bg_offset_z[i] = 0.3 * ti.sin(t * 0.3 + part_type)

            # Gentle glow pulse
            bg_brightness[i] = 0.6 + 0.4 * (0.5 + 0.5 * pulse)

        elif anim == BG_ANIM_BUTTERFLY:
            # Butterfly: fluttery flight with 2-voxel wings that flap together
            # phase = butterfly ID, amplitude = part type (0=body, 1-4=wings), speed = movement seed
            part_type = amplitude
            move_seed = speed

            # Lissajous-like path (figure-8 / meandering)
            path_time = time * 0.5 + move_seed * 6.28
            fly_x = 26.0 * ti.sin(path_time * 1.0)
            fly_z = 26.0 * ti.sin(path_time * 1.3 + 0.785)

            # Flight direction for wing orientation
            vel_x = 26.0 * ti.cos(path_time * 1.0) * 0.5
            vel_z = 26.0 * ti.cos(path_time * 1.3 + 0.785) * 0.65
            vel_len = ti.max(ti.sqrt(vel_x * vel_x + vel_z * vel_z), 0.01)
            perp_x = -vel_z / vel_len
            perp_z = vel_x / vel_len

            # Flap cycle: sin wave drives wings AND vertical lift
            flap_raw = ti.sin(time * 8.0 + move_seed * 3.0)
            # flap_raw: +1 = wings up (recharging), -1 = wings down (power stroke)

            # Vertical lift tied to flap: downstroke pushes up, upstroke sinks
            # Integrate flap: -cos gives smooth rise on downstroke, fall on upstroke
            lift = ti.cos(time * 8.0 + move_seed * 3.0)  # +1 after downstroke, -1 after upstroke
            lift_amount = 1.8  # How much the flap lifts/sinks
            fly_y = 8.0 * ti.sin(path_time * 0.7) + lift * lift_amount

            flap = flap_raw
            body_bob = 0.3 * ti.abs(flap)

            if part_type < 0.5:
                # Body
                bg_offset_x[i] = fly_x
                bg_offset_y[i] = fly_y + body_bob
                bg_offset_z[i] = fly_z
            else:
                # Wings: 1=left inner, 2=right inner, 3=left outer, 4=right outer
                is_left = (part_type > 0.5 and part_type < 1.5) or (part_type > 2.5 and part_type < 3.5)
                is_outer = part_type > 2.5
                wing_side = 1.0 if is_left else -1.0

                # Inner wings stay close, outer wings extend further
                spread_dist = 1.4 if is_outer else 0.6
                # Flap angle: wings go up together, outer flaps more
                flap_height = 0.8 if is_outer else 0.5
                flap_y = flap_height * flap

                # Position along perpendicular axis (spread) + up/down (flap)
                side_offset = wing_side * spread_dist
                bg_offset_x[i] = fly_x + perp_x * side_offset
                bg_offset_y[i] = fly_y + body_bob + flap_y
                bg_offset_z[i] = fly_z + perp_z * side_offset

            bg_brightness[i] = 0.85 + 0.15 * ti.abs(flap)

        elif anim == BG_ANIM_WAVE:
            # Water: rolling wave blobs with traveling motion
            # phase = x position, amplitude = z position
            base_x = phase
            base_z = amplitude

            # FASTER wave speed for energetic ocean
            wave_speed = 1.8
            wave_freq = 0.12

            # Primary traveling wave (minus sign = waves travel forward)
            wave1 = ti.sin(time * wave_speed - base_x * wave_freq)

            # Secondary cross-wave
            wave2 = ti.sin(time * wave_speed * 0.8 - base_z * wave_freq * 0.9)

            # Combined height (more dramatic)
            wave_height = 2.5
            total_wave = (wave1 + wave2 * 0.6) * wave_height / 1.5
            bg_offset_y[i] = total_wave

            # TRAVELING MOTION - waves roll across surface
            travel_amount = 0.8
            bg_offset_x[i] = wave1 * travel_amount * 0.5
            bg_offset_z[i] = wave2 * travel_amount * 0.3

            # SIZE PULSING - bigger at peaks
            size_pulse = 1.0 + total_wave * 0.08
            bg_brightness[i] = ti.max(0.7, ti.min(1.4, size_pulse))

        elif anim == BG_ANIM_FISH:
            # Sea serpent/dragon snaking through waves
            # phase = angle offset for this serpent's position
            # amplitude = segment index (0-19=body, 100+=antenna, 300+=whisker)
            # speed = serpent ID
            seg_idx = amplitude
            serpent_id = speed

            # Serpent swims in a circle around the arena
            swim_speed = 0.31
            base_angle = phase + time * swim_speed
            swim_radius = 50.0
            wave_freq = 0.35
            wave_speed = 2.75
            wave_height = 7.0

            if seg_idx < 100:
                # === MAIN BODY SEGMENTS ===
                segment_spacing = 0.055
                seg_angle = base_angle - seg_idx * segment_spacing
                cos_sa = ti.cos(seg_angle)
                sin_sa = ti.sin(seg_angle)

                px = cos_sa * swim_radius
                pz = sin_sa * swim_radius

                snake_wave = ti.sin(seg_idx * wave_freq - time * wave_speed)
                y_offset = snake_wave * wave_height

                head_bias = ti.max(0.0, 1.5 - seg_idx * 0.15)
                y_offset += head_bias

                # Flapping tail
                tail_flap = 0.0
                if seg_idx > 14:
                    tail_intensity = (seg_idx - 14) * 0.7
                    tail_flap = ti.sin(time * 7.0 + seg_idx * 0.4) * tail_intensity

                flap_x = -sin_sa * tail_flap
                flap_z = cos_sa * tail_flap

                bg_offset_x[i] = px - bg_positions[i].x + flap_x
                bg_offset_z[i] = pz - bg_positions[i].z + flap_z
                bg_offset_y[i] = y_offset

                if y_offset > 0:
                    bg_brightness[i] = 1.0
                else:
                    bg_brightness[i] = 0.5

            elif seg_idx < 300:
                # === ANTENNAS - on left/right sides of head ===
                antenna_side = 0 if seg_idx < 200 else 1
                antenna_idx = int(seg_idx) % 100

                # Get exact head position
                head_angle = base_angle
                cos_ha = ti.cos(head_angle)
                sin_ha = ti.sin(head_angle)
                head_x = cos_ha * swim_radius
                head_z = sin_ha * swim_radius
                head_wave = ti.sin(0.0 * wave_freq - time * wave_speed)
                head_y = head_wave * wave_height + 1.5

                # Radial direction (inward/outward from circle center) = left/right of head
                radial_x = cos_ha
                radial_z = sin_ha

                # Forward direction (tangent to circle = direction of travel)
                forward_x = -sin_ha
                forward_z = cos_ha

                # Side offset - closer together
                side_offset = 2.0 if antenna_side == 0 else -2.0

                # Forward lean - antennas angle ~60 deg forward
                forward_lean = antenna_idx * 1.0

                # Root position shifted forward on head
                root_forward = 2.0

                # Head velocity for compensation
                head_vel = -wave_speed * ti.cos(0.0 * wave_freq - time * wave_speed)

                # When head dives (vel negative), push root forward to stay attached
                root_compensate = -head_vel * 0.7

                # Cascaded delay for natural bend
                seg_delay = antenna_idx * 0.1
                delayed_time = time - seg_delay

                # Each segment's delayed velocity for forward/back bend
                delayed_vel = -wave_speed * ti.cos(0.0 * wave_freq - delayed_time * wave_speed)

                # Fling back more
                flap_amount = delayed_vel * 1.0 + delayed_vel * antenna_idx * 0.06

                # Subtle V-spread - tips splay outward slightly
                spread = antenna_idx * 0.25 * (1.0 if antenna_side == 0 else -1.0)

                # Root crawls forward when head dives
                ax = head_x + radial_x * (side_offset + spread) + forward_x * (root_forward + root_compensate + forward_lean + flap_amount)
                az = head_z + radial_z * (side_offset + spread) + forward_z * (root_forward + root_compensate + forward_lean + flap_amount)
                ay = head_y + 2.0 + antenna_idx * 0.85

                bg_offset_x[i] = ax - bg_positions[i].x
                bg_offset_z[i] = az - bg_positions[i].z
                bg_offset_y[i] = ay

                if ay > 0:
                    bg_brightness[i] = 1.0
                else:
                    bg_brightness[i] = 0.5


        elif anim == BG_ANIM_SERPENT_SPLASH:
            # Water splash burst when serpent head dives into water
            # phase = serpent angular offset, amplitude = splash voxel index, speed = serpent_id
            splash_idx = amplitude
            num_splash = 24.0

            # Serpent head timing (must match BG_ANIM_FISH exactly)
            wave_speed_s = 2.75
            swim_speed_s = 0.31
            swim_radius_s = 50.0
            pi2 = 2.0 * 3.14159265

            # Head y = sin(-time * 2.75) * 7.0 + 1.5
            # Trigger slightly BEFORE head hits water (head_y ≈ 0.7)
            # sin(θ) = (0.7-1.5)/7.0 = -0.1143, θ = 2π - arcsin(0.1143) ≈ 6.169
            dive_theta = pi2 - 0.1145

            # Current phase in the sine cycle
            raw_theta = -time * wave_speed_s
            theta = raw_theta - ti.floor(raw_theta / pi2) * pi2  # mod 2pi

            # Time since last dive: how far past dive_theta (theta decreasing)
            phase_since = dive_theta - theta
            phase_since = phase_since - ti.floor(phase_since / pi2) * pi2  # mod 2pi
            splash_t = phase_since / wave_speed_s  # seconds since dive

            # Long enough for particles to arc up, land, and sink
            splash_dur = 4.0

            if splash_t < splash_dur:
                # Head position at moment of dive
                dive_time = time - splash_t
                head_angle = phase + dive_time * swim_speed_s
                cos_ha = ti.cos(head_angle)
                sin_ha = ti.sin(head_angle)
                head_x = cos_ha * swim_radius_s
                head_z = sin_ha * swim_radius_s

                # Forward direction (tangent to swim circle = direction of travel)
                forward_x = -sin_ha
                forward_z = cos_ha

                # Offset splash forward in swim direction
                splash_cx = head_x + forward_x * 6.0
                splash_cz = head_z + forward_z * 6.0

                # Burst direction for this voxel (ring pattern)
                burst_angle = splash_idx * (pi2 / num_splash)

                # Reuse cached trig for radial/tangent frame
                radial_x = cos_ha
                radial_z = sin_ha
                tangent_x = -sin_ha
                tangent_z = cos_ha

                # Burst in local 2D plane
                local_x = ti.cos(burst_angle)
                local_z = ti.sin(burst_angle)

                # Transform to world coordinates
                burst_dx = local_x * radial_x + local_z * tangent_x
                burst_dz = local_x * radial_z + local_z * tangent_z

                # Burst speed with variation
                burst_spd = 14.5 + ti.sin(splash_idx * 2.3) * 5.0

                # Upward velocity varies
                up_speed = 26.0 + ti.cos(splash_idx * 1.7) * 10.0

                # Time when this particle hits water: up/gravity
                gravity = 32.0
                t_ground_sp = up_speed / gravity

                # Clamp horizontal movement so XZ freezes at landing
                t_horiz_sp = ti.min(splash_t, t_ground_sp)

                # Parabolic arc for Y (offset from base position which is already at water level)
                py = up_speed * splash_t - gravity * splash_t * splash_t

                # XZ position (freezes when particle hits water)
                px = splash_cx + burst_dx * burst_spd * t_horiz_sp
                pz = splash_cz + burst_dz * burst_spd * t_horiz_sp

                if py >= 0.0:
                    # Above water: normal flight
                    bg_offset_x[i] = px - bg_positions[i].x
                    bg_offset_y[i] = py
                    bg_offset_z[i] = pz - bg_positions[i].z
                    bg_brightness[i] = 1.0
                else:
                    # Below water: slow sink and fade
                    time_below_sp = splash_t - t_ground_sp
                    if time_below_sp < 0.0:
                        time_below_sp = 0.0
                    sink_y_sp = -time_below_sp * 2.0
                    if sink_y_sp > -16.0:
                        bg_offset_x[i] = px - bg_positions[i].x
                        bg_offset_y[i] = sink_y_sp
                        bg_offset_z[i] = pz - bg_positions[i].z
                        sink_frac_sp = time_below_sp * 2.0 / 16.0
                        bg_brightness[i] = ti.max(0.0, 1.0 - sink_frac_sp)
                    else:
                        bg_offset_y[i] = -200.0
                        bg_brightness[i] = 0.0
            else:
                # No splash active — hide underground so no dark voxels
                bg_brightness[i] = 0.0
                bg_offset_y[i] = -200.0

        elif anim == BG_ANIM_JUMPING_FISH:
            # Dolphin: periodic jump arc above ocean waves
            # Body segments explicitly spaced along the parabolic arc path
            # Each part sits at a fixed distance behind the head — body curves
            # naturally along the parabola. Parts below water hide individually.
            # amplitude = part index (0-15), phase = dolphin ID, speed = jump offset
            part_d = amplitude
            dolphin_id_d = phase
            jump_offset_d = speed
            water_y_d = 14.0       # Lowest wave trough
            period_d = 30.0        # Full cycle (rare jumps)
            underwater_d = 26.5    # Hidden phase (0-26.5s)
            arc_dur_d = 2.5        # Head's parabola duration
            peak_h_d = 28.0        # Peak height above water
            orbit_r_d = 45.0
            travel_d = 100.0       # Wide forward leap (body stretches along this)
            seg_sp_d = 4.0         # World units between body segment centers

            cycle_d = (time + jump_offset_d) % period_d

            # Pre-init outputs (Taichi: must exist before conditionals)
            out_x_d = 0.0
            out_y_d = -200.0
            out_z_d = 0.0
            out_b_d = 0.0

            if cycle_d >= underwater_d:
                # Head's normalized arc position (can exceed 1.0 in jump window)
                pn_head_d = (cycle_d - underwater_d) / arc_dur_d

                if part_d > 15.5:
                    # === BLOWHOLE SPRAY (parts 16-31) ===
                    # Explosive upward mist burst at the peak of the jump
                    spray_idx_d = part_d - 16.0
                    spray_start_d = 0.38
                    spray_end_d = 0.75
                    spray_norm_d = (pn_head_d - spray_start_d) / (spray_end_d - spray_start_d)

                    if spray_norm_d <= 0.0 or spray_norm_d >= 1.0:
                        bg_size[i] = 1.2  # Reset size for next cycle
                    else:
                        # Head position on the arc
                        head_pn_d = pn_head_d - 1.0 * seg_sp_d / travel_d
                        head_arc_d = water_y_d + peak_h_d * 4.0 * head_pn_d * (1.0 - head_pn_d)

                        # Orbit (same calc as body)
                        jc_d = ti.floor((time + jump_offset_d) / period_d)
                        da_d = jc_d * 0.8 + dolphin_id_d * 3.14
                        cos_da = ti.cos(da_d)
                        sin_da = ti.sin(da_d)
                        fx_d = -sin_da
                        fz_d = cos_da
                        cx_d = orbit_r_d * cos_da
                        cz_d = orbit_r_d * sin_da

                        head_fwd_d = (head_pn_d - 0.5) * travel_d
                        hx_d = cx_d + fx_d * head_fwd_d
                        hz_d = cz_d + fz_d * head_fwd_d

                        # Spray cone — 16 particles, blown out fast
                        sp_ang_d = spray_idx_d * (6.28318 / 16.0)
                        sp_rise = 1.0 + ti.sin(spray_idx_d * 3.7) * 0.4
                        sp_horiz_d = spray_norm_d * 8.0 * sp_rise
                        sp_up_d = (4.0 + spray_norm_d * 22.0) * sp_rise

                        out_x_d = hx_d + ti.cos(sp_ang_d) * sp_horiz_d
                        out_y_d = head_arc_d + sp_up_d
                        out_z_d = hz_d + ti.sin(sp_ang_d) * sp_horiz_d
                        out_b_d = 1.0
                        # Shrink particles smoothly
                        fade_d = 1.0 - spray_norm_d
                        bg_size[i] = 1.2 * fade_d * fade_d

                else:
                    # === BODY (parts 0-15) ===
                    # Map part to body segment index (distance behind head)
                    body_seg_d = 0.0
                    if part_d <= 8.0:
                        body_seg_d = part_d        # Main body chain
                    elif part_d <= 10.0:
                        body_seg_d = 9.0           # Flukes behind tail stock
                    elif part_d <= 11.0:
                        body_seg_d = 4.0           # Dorsal at mid-body
                    elif part_d <= 13.0:
                        body_seg_d = 2.0           # Flippers at front body
                    else:
                        body_seg_d = 1.0           # Eyes on head

                    # This segment's position on the parabola
                    seg_pn_d = pn_head_d - body_seg_d * seg_sp_d / travel_d

                    # Arc height — naturally negative outside [0,1] → below water
                    arc_y_d = water_y_d + peak_h_d * 4.0 * seg_pn_d * (1.0 - seg_pn_d)

                    # Hide threshold just above wave troughs so tail sinks fully
                    if arc_y_d > water_y_d + 1.5:
                        # Orbit — advances angle each jump
                        jc_d = ti.floor((time + jump_offset_d) / period_d)
                        da_d = jc_d * 0.8 + dolphin_id_d * 3.14
                        cos_da = ti.cos(da_d)
                        sin_da = ti.sin(da_d)

                        fx_d = -sin_da      # Forward (tangent to orbit)
                        fz_d = cos_da
                        lx_d = -fz_d        # Lateral (perpendicular)
                        lz_d = fx_d

                        cx_d = orbit_r_d * cos_da
                        cz_d = orbit_r_d * sin_da

                        # Forward position from arc parameter
                        seg_fwd_d = (seg_pn_d - 0.5) * travel_d

                        out_x_d = cx_d + fx_d * seg_fwd_d
                        out_y_d = arc_y_d
                        out_z_d = cz_d + fz_d * seg_fwd_d

                        # Attached-part offsets (scaled 2x for bigger dolphin)
                        if part_d > 8.5 and part_d < 10.5:
                            # TAIL FLUKES — horizontal spread
                            fluke_s = (part_d - 9.5) * 2.0
                            out_x_d += lx_d * fluke_s * 5.0
                            out_z_d += lz_d * fluke_s * 5.0
                        elif part_d > 10.5 and part_d < 11.5:
                            # DORSAL FIN — on top of body
                            out_y_d += 5.0
                        elif part_d > 11.5 and part_d < 13.5:
                            # PECTORAL FLIPPERS — spread from body
                            flip_s = (part_d - 12.5) * 2.0
                            out_x_d += lx_d * flip_s * 6.0
                            out_z_d += lz_d * flip_s * 6.0
                            out_y_d -= 1.6
                        elif part_d > 13.5:
                            # EYES — on head sides
                            eye_s = (part_d - 14.5) * 2.0
                            out_x_d += lx_d * eye_s * 2.4 + fx_d * 1.6
                            out_z_d += lz_d * eye_s * 2.4 + fz_d * 1.6
                            out_y_d += 1.0

                        out_b_d = 1.0

            bg_offset_x[i] = out_x_d
            bg_offset_y[i] = out_y_d
            bg_offset_z[i] = out_z_d
            bg_brightness[i] = out_b_d

        elif anim == BG_ANIM_FISH_SPLASH:
            # Splash burst when dolphin exits/enters water
            # amplitude = splash index (0-15), phase = dolphin ID, speed = jump offset
            sp_idx_ds = amplitude
            num_sp_ds = 16.0
            jump_offset_ds = speed
            period_ds = 30.0
            underwater_ds = 26.5
            arc_dur_ds = 2.5
            orbit_r_ds = 45.0
            travel_ds = 100.0
            water_surf_ds = 17.0   # Wave surface Y for splash height

            cycle_ds = (time + jump_offset_ds) % period_ds

            # Splash at two moments: just before head exits, as head re-enters
            exit_cycle_ds = underwater_ds - 0.2   # Splash precedes the dolphin
            enter_cycle_ds = underwater_ds + arc_dur_ds - 0.15  # Splash as head dives
            splash_t_ds = -1.0
            splash_fwd_ds = 0.0

            dt_exit_ds = cycle_ds - exit_cycle_ds
            dt_enter_ds = cycle_ds - enter_cycle_ds

            if dt_exit_ds >= 0.0 and dt_exit_ds < 1.8:
                splash_t_ds = dt_exit_ds
                splash_fwd_ds = -travel_ds * 0.48  # Head exits at arc start
            elif dt_enter_ds >= 0.0 and dt_enter_ds < 1.8:
                splash_t_ds = dt_enter_ds
                splash_fwd_ds = travel_ds * 0.48   # Head enters at arc end

            if splash_t_ds >= 0.0:
                jc_ds = ti.floor((time + jump_offset_ds) / period_ds)
                da_ds = jc_ds * 0.8 + phase * 3.14
                cos_ds = ti.cos(da_ds)
                sin_ds = ti.sin(da_ds)
                fx_ds = -sin_ds
                fz_ds = cos_ds

                scx_ds = orbit_r_ds * cos_ds + fx_ds * splash_fwd_ds
                scz_ds = orbit_r_ds * sin_ds + fz_ds * splash_fwd_ds

                bang_ds = sp_idx_ds * (6.28318 / num_sp_ds)
                bdx_ds = ti.cos(bang_ds)
                bdz_ds = ti.sin(bang_ds)

                bspd_ds = 12.0 + ti.sin(sp_idx_ds * 2.3) * 5.0
                uspd_ds = 24.0 + ti.cos(sp_idx_ds * 1.7) * 8.0
                grav_ds = 28.0
                t_gnd_ds = uspd_ds / grav_ds
                t_hz_ds = ti.min(splash_t_ds, t_gnd_ds)

                spy_ds = water_surf_ds + uspd_ds * splash_t_ds - grav_ds * splash_t_ds * splash_t_ds
                spx_ds = scx_ds + bdx_ds * bspd_ds * t_hz_ds
                spz_ds = scz_ds + bdz_ds * bspd_ds * t_hz_ds

                if spy_ds >= water_surf_ds - 2.0:
                    bg_offset_x[i] = spx_ds
                    bg_offset_y[i] = spy_ds
                    bg_offset_z[i] = spz_ds
                    fade_ds = ti.max(0.0, 1.0 - splash_t_ds * 0.6)
                    bg_brightness[i] = fade_ds
                else:
                    bg_offset_y[i] = -200.0
                    bg_brightness[i] = 0.0
            else:
                bg_offset_y[i] = -200.0
                bg_brightness[i] = 0.0

        elif anim == BG_ANIM_SQUID_TENTACLE:
            # Giant squid tentacle: chain of 24 segments per tentacle, 3 tentacles
            # amplitude = segment index (0-23), phase = tentacle ID (0-2), speed = cycle offset
            seg_sq = amplitude           # 0 = base, 23 = tip
            tent_id_sq = phase           # 0, 1, or 2
            cycle_off_sq = speed
            water_y_sq = 15.5            # Water surface
            period_sq = 20.0             # Full cycle (TEST — normally 45)
            underwater_sq = 14.0         # Hidden phase duration (TEST — normally 39)
            event_dur_sq = 6.0           # Total event window
            rise_dur_sq = 1.8            # Rise phase
            writhe_dur_sq = 2.4          # Writhe phase
            plunge_dur_sq = 1.8          # Plunge phase
            orbit_r_sq = 68.0            # Orbital radius
            link_len_sq = 1.8            # Distance between segment centers (keeps segments touching)
            num_seg_sq = 12.0
            peak_reach_sq = 32.0         # How high tentacles reach above water

            cycle_sq = (time + cycle_off_sq) % period_sq

            # Pre-init (Taichi requirement)
            out_x_sq = 0.0
            out_y_sq = -200.0
            out_z_sq = 0.0
            out_b_sq = 0.0

            if cycle_sq >= underwater_sq:
                event_t_sq = cycle_sq - underwater_sq
                event_norm_sq = event_t_sq / event_dur_sq

                # Timing stagger per tentacle (~0.15s between each)
                stagger_sq = tent_id_sq * 0.15
                local_t_sq = event_t_sq - stagger_sq
                local_norm_sq = local_t_sq / event_dur_sq

                if local_t_sq >= 0.0 and local_t_sq <= event_dur_sq:
                    # === PHASE-DRIVEN BASE ANGLE (smoothstep crossfaded) ===
                    xfade_sq = 0.3  # Crossfade half-width in seconds
                    t1_sq = rise_dur_sq
                    t2_sq = rise_dur_sq + writhe_dur_sq

                    # Smoothstep blend at rise/writhe boundary
                    raw1_sq = ti.max(0.0, ti.min(1.0, (local_t_sq - (t1_sq - xfade_sq)) / (2.0 * xfade_sq)))
                    blend1_sq = raw1_sq * raw1_sq * (3.0 - 2.0 * raw1_sq)
                    # Smoothstep blend at writhe/plunge boundary
                    raw2_sq = ti.max(0.0, ti.min(1.0, (local_t_sq - (t2_sq - xfade_sq)) / (2.0 * xfade_sq)))
                    blend2_sq = raw2_sq * raw2_sq * (3.0 - 2.0 * raw2_sq)

                    w_rise_sq = 1.0 - blend1_sq
                    w_writhe_sq = blend1_sq * (1.0 - blend2_sq)
                    w_plunge_sq = blend2_sq

                    # Clamped phase times (each phase evaluable at any moment)
                    rise_t_sq = ti.max(0.0, ti.min(local_t_sq / rise_dur_sq, 1.0))
                    writhe_t_sq = ti.max(0.0, ti.min((local_t_sq - rise_dur_sq) / writhe_dur_sq, 1.0))
                    plunge_t_sq = ti.max(0.0, ti.min((local_t_sq - rise_dur_sq - writhe_dur_sq) / plunge_dur_sq, 1.0))

                    # --- Rise angle ---
                    rise_angle_sq = -0.5 - 1.0 * rise_t_sq * rise_t_sq

                    # --- Writhe angle ---
                    env_sq = ti.sin(writhe_t_sq * 3.14159)
                    sway1_sq = 0.45 * ti.sin(writhe_t_sq * 6.28318)
                    sway2_sq = 0.2 * env_sq * ti.sin(writhe_t_sq * 15.708 + tent_id_sq * 1.5)
                    writhe_angle_sq = -1.5 + sway1_sq + sway2_sq

                    # --- Plunge: rope-through-hole retract (shape stays, origin sinks) ---
                    # Thrash as it sinks — two layered sways, both zero at boundaries
                    plunge_angle_sq = -1.5 + 0.4 * ti.sin(plunge_t_sq * 9.42478) + 0.15 * ti.sin(plunge_t_sq * 15.708)
                    plunge_sink_sq = 55.0 * (2.0 * plunge_t_sq - plunge_t_sq * plunge_t_sq)  # ease-out: fast yank, slows at end

                    # Blend base angle
                    base_angle_sq = w_rise_sq * rise_angle_sq + w_writhe_sq * writhe_angle_sq + w_plunge_sq * plunge_angle_sq

                    # Blend curl strength
                    writhe_curl_sq = 0.07 + 0.06 * ti.sin(writhe_t_sq * 6.28318) + 0.03 * ti.sin(writhe_t_sq * 12.566)
                    plunge_curl_sq = 0.07  # same shape throughout retract
                    curl_strength_sq = w_rise_sq * 0.07 + w_writhe_sq * writhe_curl_sq + w_plunge_sq * plunge_curl_sq

                    # Blend lag
                    rise_lag_sq = 0.02 * (1.0 - rise_t_sq * rise_t_sq)
                    rate1_sq = 0.45 * 6.28318 / writhe_dur_sq * ti.cos(writhe_t_sq * 6.28318)
                    writhe_lag_sq = -rate1_sq * (1.0 / 23.0) * 0.12 * env_sq
                    plunge_lag_sq = -0.02 * plunge_t_sq * plunge_t_sq
                    lag_per_seg_sq = w_rise_sq * rise_lag_sq + w_writhe_sq * writhe_lag_sq + w_plunge_sq * plunge_lag_sq

                    # Anchor position — orbits arena like dolphins
                    jc_sq = ti.floor((time + cycle_off_sq) / period_sq)
                    da_sq = jc_sq * 1.2 + 2.0  # Different base angle from dolphins
                    cos_da_sq = ti.cos(da_sq)
                    sin_da_sq = ti.sin(da_sq)
                    anchor_x_sq = orbit_r_sq * cos_da_sq
                    anchor_z_sq = orbit_r_sq * sin_da_sq

                    # Spread angle — tentacles fan out at 0°, 120°, 240°
                    spread_ang_sq = tent_id_sq * (6.28318 / 3.0) + da_sq
                    spread_dx_sq = ti.cos(spread_ang_sq)
                    spread_dz_sq = ti.sin(spread_ang_sq)

                    # Surface emergence point — spread 8 units apart from anchor
                    base_x_sq = anchor_x_sq + spread_dx_sq * 8.0
                    base_z_sq = anchor_z_sq + spread_dz_sq * 8.0

                    # Walk chain: accumulate position segment by segment
                    # During plunge, origin sinks below water — base disappears first
                    chain_x_sq = base_x_sq
                    chain_y_sq = water_y_sq - w_plunge_sq * plunge_sink_sq
                    cum_angle_sq = base_angle_sq + spread_dx_sq * 0.3  # Outward lean

                    for s in range(24):
                        if ti.cast(s, ti.f32) <= seg_sq + 0.5:
                            sf = ti.cast(s, ti.f32)
                            frac_sq = sf / 23.0

                            # Golden ratio curl — tips spiral tightly, base stays straight
                            phi_weight_sq = ti.pow(frac_sq, 1.618)
                            curl_sq = curl_strength_sq * phi_weight_sq * 3.0

                            # Three-harmonic ripple — chaotic flailing
                            ripple_sq = 0.12 * frac_sq * ti.sin(
                                time * 3.0 + sf * 0.4 + tent_id_sq * 2.0
                            ) + 0.06 * frac_sq * ti.sin(
                                time * 5.5 - sf * 0.7 + tent_id_sq * 1.3
                            ) + 0.04 * frac_sq * ti.sin(
                                time * 8.0 + sf * 1.1 - tent_id_sq * 0.9
                            )

                            cum_angle_sq += curl_sq + lag_per_seg_sq + ripple_sq
                            chain_x_sq += link_len_sq * ti.cos(cum_angle_sq)
                            chain_y_sq += link_len_sq * (-ti.sin(cum_angle_sq))

                    # Z offset: lean outward in spread direction
                    chain_z_sq = base_z_sq + spread_dz_sq * (chain_x_sq - base_x_sq) * 0.3

                    # Smooth fade at water surface
                    dist_above_sq = chain_y_sq - water_y_sq
                    if dist_above_sq > 3.0:
                        out_x_sq = chain_x_sq
                        out_y_sq = chain_y_sq
                        out_z_sq = chain_z_sq
                        out_b_sq = 1.0
                    elif dist_above_sq > -1.0:
                        out_x_sq = chain_x_sq
                        out_y_sq = chain_y_sq
                        out_z_sq = chain_z_sq
                        out_b_sq = (dist_above_sq + 1.0) / 4.0
                    else:
                        out_y_sq = -200.0
                        out_b_sq = 0.0

            bg_offset_x[i] = out_x_sq
            bg_offset_y[i] = out_y_sq
            bg_offset_z[i] = out_z_sq
            bg_brightness[i] = out_b_sq

        elif anim == BG_ANIM_SQUID_SPLASH:
            # Splash burst when tentacles emerge/plunge
            # amplitude = splash index (0-15), speed = cycle offset
            sp_idx_sq = amplitude
            num_sp_sq = 28.0
            cycle_off_sq2 = speed
            period_sq2 = 20.0            # TEST — normally 45
            underwater_sq2 = 14.0        # TEST — normally 39
            event_dur_sq2 = 6.0
            orbit_r_sq2 = 68.0
            water_surf_sq2 = 17.0

            cycle_sq2 = (time + cycle_off_sq2) % period_sq2

            # Splash at two moments: emergence and retraction (same physics both times)
            exit_cycle_sq2 = underwater_sq2 - 0.2    # 1.8 — emergence
            retract_cycle_sq2 = underwater_sq2 + 4.5   # retraction (relative to event start)
            splash_t_sq2 = -1.0

            dt_exit_sq2 = cycle_sq2 - exit_cycle_sq2
            dt_retract_sq2 = cycle_sq2 - retract_cycle_sq2

            if dt_exit_sq2 >= 0.0 and dt_exit_sq2 < 1.8:
                splash_t_sq2 = dt_exit_sq2
            elif dt_retract_sq2 >= 0.0 and dt_retract_sq2 < 1.8:
                splash_t_sq2 = dt_retract_sq2

            if splash_t_sq2 >= 0.0:
                # Anchor position (same as tentacle anchor)
                jc_sq2 = ti.floor((time + cycle_off_sq2) / period_sq2)
                da_sq2 = jc_sq2 * 1.2 + 2.0
                cos_sq2 = ti.cos(da_sq2)
                sin_sq2 = ti.sin(da_sq2)

                scx_sq2 = orbit_r_sq2 * cos_sq2
                scz_sq2 = orbit_r_sq2 * sin_sq2

                # Radial burst — same physics as dolphin splash
                bang_sq2 = sp_idx_sq * (6.28318 / num_sp_sq)
                bdx_sq2 = ti.cos(bang_sq2)
                bdz_sq2 = ti.sin(bang_sq2)

                bspd_sq2 = 20.0 + ti.sin(sp_idx_sq * 2.3) * 8.0
                uspd_sq2 = 36.0 + ti.cos(sp_idx_sq * 1.7) * 12.0
                grav_sq2 = 36.0
                t_gnd_sq2 = uspd_sq2 / grav_sq2
                t_hz_sq2 = ti.min(splash_t_sq2, t_gnd_sq2)

                spy_sq2 = water_surf_sq2 + uspd_sq2 * splash_t_sq2 - grav_sq2 * splash_t_sq2 * splash_t_sq2
                spx_sq2 = scx_sq2 + bdx_sq2 * bspd_sq2 * t_hz_sq2
                spz_sq2 = scz_sq2 + bdz_sq2 * bspd_sq2 * t_hz_sq2

                if spy_sq2 >= water_surf_sq2 - 2.0:
                    bg_offset_x[i] = spx_sq2
                    bg_offset_y[i] = spy_sq2
                    bg_offset_z[i] = spz_sq2
                    fade_sq2 = ti.max(0.0, 1.0 - splash_t_sq2 * 0.4)
                    bg_brightness[i] = fade_sq2
                else:
                    bg_offset_y[i] = -200.0
                    bg_brightness[i] = 0.0
            else:
                bg_offset_y[i] = -200.0
                bg_brightness[i] = 0.0

        elif anim == BG_ANIM_TREE:
            # Tree branch sway - wind effect
            # amplitude controls how much this part sways (trunk=0.3, branches=1.5+)
            # speed varies slightly per tree for desync

            # Slow wind gust wave
            gust = 0.5 + 0.5 * ti.sin(t * 0.2 + phase * 0.8)
            sway_amp = amplitude * (0.3 + 0.7 * gust)

            # Horizontal sway (mostly X, some Z)
            bg_offset_x[i] = sway_amp * ti.sin(t * 0.7 + phase * 0.5)
            bg_offset_z[i] = sway_amp * 0.4 * ti.sin(t * 0.5 + phase * 0.3 + 1.0)

            # Slight vertical bob
            bg_offset_y[i] = sway_amp * 0.15 * ti.sin(t * 0.9 + phase * 0.4)

        elif anim == BG_ANIM_SCRUNCH:
            # Grub scrunching motion - segments compress and expand
            # amplitude = segment index (0=bottom, 3=head)
            # phase = shared phase for this grub
            seg_idx = amplitude

            # Get excitement level - use squared for smoother transition
            raw_excitement = stadium_excitement[None]
            excitement = raw_excitement * raw_excitement  # Eases out gradually

            # Base scrunch speed
            base_speed = 2.2 + excitement * 1.2

            # Normal scrunch
            scrunch = ti.sin(time * base_speed + phase - seg_idx * 0.8)

            # Vertical compression - more intense idle
            squish_mult = 0.4 + excitement * 0.4
            bg_offset_y[i] = scrunch * squish_mult * (seg_idx - 1.5)

            # Horizontal wobble - more visible
            wobble_mult = 0.25 + excitement * 0.25
            bg_offset_x[i] = ti.sin(time * 1.2 + phase) * wobble_mult
            bg_offset_z[i] = ti.cos(time * 1.2 + phase) * wobble_mult

        elif anim == BG_ANIM_LAVA:
            # Lava: slow viscous wave with molten glow cycling
            # phase = x position, amplitude = z position (same encoding as WAVE)
            base_x = phase
            base_z = amplitude

            lava_speed = 0.6
            lava_freq = 0.08
            wave1 = ti.sin(time * lava_speed - base_x * lava_freq)
            wave2 = ti.sin(time * lava_speed * 0.7 - base_z * lava_freq * 0.8)
            total = (wave1 + wave2 * 0.2) * 1.0

            bg_offset_y[i] = total
            bg_offset_x[i] = wave1 * 0.08
            bg_offset_z[i] = wave2 * 0.04

            # Molten glow: peaks glow bright, valleys go dark/crusty
            glow = 0.5 + 0.5 * total
            bg_brightness[i] = 0.3 + glow * 1.2

        elif anim == BG_ANIM_LAVA_SPRAY:
            # Lava spray: burst then quiet — smooth arcs
            # phase = time offset (burst sync), amplitude = launch height, speed = cycle period
            cycle_period = speed
            t_cycle = (time + phase) % cycle_period

            # Burst lasts 35% of cycle, rest is quiet
            burst_duration = cycle_period * 0.35
            arc_height = 0.0

            if t_cycle < burst_duration:
                # ACTIVE: particle is flying
                normalized = t_cycle / burst_duration  # 0 to 1 within burst

                # Smooth sinusoidal arc — no hard peak, just a smooth hump
                arc_height = amplitude * ti.sin(normalized * 3.14159)

                # Cone spread: fans out on the way up, holds spread while falling
                cone_spread = 1.5 + amplitude * 0.15
                drift_cone = 0.0
                if normalized < 0.4:
                    # Rising: spread grows smoothly
                    rise_ratio = normalized / 0.4
                    drift_cone = rise_ratio * rise_ratio
                else:
                    # Past peak: hold spread
                    drift_cone = 1.0
                bg_offset_y[i] = arc_height
                bg_offset_x[i] = drift_cone * cone_spread * ti.sin(phase * 6.28)
                bg_offset_z[i] = drift_cone * cone_spread * ti.cos(phase * 6.28)

                # Brightness follows same sin curve — 0 at edges, bright at peak, no pop
                bg_brightness[i] = 1.4 * ti.sin(normalized * 3.14159)
            else:
                # QUIET: reset to origin and invisible
                bg_offset_y[i] = 0.0
                bg_offset_x[i] = 0.0
                bg_offset_z[i] = 0.0
                bg_brightness[i] = 0.0

        elif anim == BG_ANIM_LAVA_VOLCANO:
            # Volcano cone: smooth pressure build-up → eruption → settle
            # phase = eruption_base_offset (synced with spray), speed = cycle_period
            # amplitude = layer index (0=base, 3=tip)
            cycle_period = speed
            t_cycle = (time + phase) % cycle_period
            layer = amplitude
            normalized = t_cycle / cycle_period

            # Smooth pressure cycle:
            # 0.0-0.17: eruption release — reverse of charge but 3x faster
            # 0.17-0.5: settle back to rest
            # 0.5-1.0: slow pressure build-up (scrunch down)
            pressure = 0.0
            if normalized > 0.5:
                # Building pressure: smooth quadratic ease-in (slow start, accelerates)
                build = (normalized - 0.5) * 2.0  # 0 to 1
                pressure = build * build
            elif normalized < 0.17:
                # Eruption release: reverse of charge curve, 3x speed
                release = normalized / 0.17  # 0 to 1
                inv = 1.0 - release
                pressure = inv * inv  # Same quadratic curve, reversed
            else:
                # Settling: smooth ease back to rest
                settle = (normalized - 0.17) / 0.33  # 0 to 1
                pressure = 0.0

            # Higher layers move more (tip scrunches most)
            layer_mult = 0.5 + layer * 0.6
            scrunch = -pressure * layer_mult * 1.8
            bulge = ti.max(0.0, pressure) * 0.4
            glow_boost = ti.max(0.0, pressure) * 0.7

            bg_offset_y[i] = scrunch
            bg_offset_x[i] = bulge * ti.sin(phase * 3.0)
            bg_offset_z[i] = bulge * ti.cos(phase * 3.0)
            bg_brightness[i] = 1.0 + glow_boost

        elif anim == BG_ANIM_RAIN:
            # Rain: fast straight falling drops that cycle seamlessly
            # phase = random time offset, amplitude = fall distance, speed = fall rate
            fall_dist = amplitude
            t = (time * speed + phase) % fall_dist

            bg_offset_y[i] = -t                    # Straight down, fast
            bg_offset_x[i] = 0.0
            bg_offset_z[i] = 0.0
            bg_brightness[i] = 0.7 + 0.3 * (1.0 - t / fall_dist)

        elif anim == BG_ANIM_RAIN_SPLASH:
            # Rain splash: quick pop-up arc at ground level
            # phase = timing offset + direction seed, amplitude = outward radius, speed = cycle period
            cycle = speed
            t_splash = (time + phase) % cycle
            normalized = t_splash / cycle

            splash_height = 2.5 * ti.sin(normalized * 3.14159)
            spread = amplitude * normalized

            bg_offset_y[i] = splash_height
            bg_offset_x[i] = spread * ti.sin(phase * 6.28)
            bg_offset_z[i] = spread * ti.cos(phase * 6.28)
            bg_brightness[i] = 1.2 * ti.sin(normalized * 3.14159)

        elif anim == BG_ANIM_CLOUD:
            # Cloud: floating drift + individual breathing + strong size morphing
            # phase = cluster_id (shared drift), amplitude = individual seed, speed = pulse rate

            # Bigger floating drift (all voxels in same cloud move together)
            drift_x = 10.0 * ti.sin(time * 0.15 + phase * 2.0)
            drift_z = 8.0 * ti.cos(time * 0.12 + phase * 1.5)
            drift_y = 2.0 * ti.sin(time * 0.08 + phase * 3.0)

            # Individual breathing (wider wandering within cloud)
            breathe_x = 0.8 * ti.sin(time * speed * 0.3 + amplitude * 5.0)
            breathe_y = 0.4 * ti.sin(time * speed * 0.25 + amplitude * 3.0)
            breathe_z = 0.8 * ti.cos(time * speed * 0.35 + amplitude * 4.0)

            # Fast chaotic jitter (small, rapid, unique per voxel)
            jitter_x = 0.35 * ti.sin(time * 2.3 + amplitude * 11.0)
            jitter_y = 0.2 * ti.cos(time * 1.9 + amplitude * 7.7)
            jitter_z = 0.35 * ti.sin(time * 2.7 + amplitude * 9.3)

            bg_offset_x[i] = drift_x + breathe_x + jitter_x
            bg_offset_y[i] = drift_y + breathe_y + jitter_y
            bg_offset_z[i] = drift_z + breathe_z + jitter_z

            # Brightness pulsing (voxels swell and shrink)
            pulse = ti.sin(time * speed * 0.5 + amplitude * 6.28)
            bg_brightness[i] = 0.7 + 0.25 * pulse  # Range 0.45 to 0.95

        elif anim == BG_ANIM_PTERODACTYL:
            # Pterodactyl: circular orbit with bending wing flap
            # Parts: 0=body, 1=neck, 2=head, 3-7=beak cone, 8-14=crest cone
            #   15-19=left wing, 20-24=right wing, 25-29=tail
            part = amplitude
            orbit_speed = speed
            orbit_radius = 70.0

            # Circular orbit
            orbit_angle = time * orbit_speed + phase * 6.28
            px = orbit_radius * ti.cos(orbit_angle)
            pz = orbit_radius * ti.sin(orbit_angle)
            # Slow drift + subtle lift synced to wing flaps (rises on downstroke)
            flap_lift = ti.max(0.0, -ti.sin(time * 2.5 + phase * 1.5))  # lift on downstroke
            py = 2.0 * ti.sin(time * 0.5 + phase * 2.0) + 2.0 * flap_lift

            # Flight direction (tangent to circle)
            dir_x = -ti.sin(orbit_angle)
            dir_z = ti.cos(orbit_angle)

            # Perpendicular axis (for wings)
            perp_x = -dir_z
            perp_z = dir_x

            # Base flap cycle
            flap_base = time * 2.5 + phase * 1.5

            # Head position (shared by beak and crest)
            head_fwd = 9.0
            head_y = 2.0

            off_x = 0.0
            off_y = 0.0
            off_z = 0.0

            if part < 0.5:
                # Body
                off_x = px
                off_y = py
                off_z = pz
            elif part < 1.5:
                # Neck
                off_x = px + dir_x * 5.0
                off_y = py + 1.0
                off_z = pz + dir_z * 5.0
            elif part < 2.5:
                # Head
                off_x = px + dir_x * head_fwd
                off_y = py + head_y
                off_z = pz + dir_z * head_fwd
            elif part < 7.5:
                # Beak cone (3-7): 5 voxels extending forward from head
                beak_idx = part - 3.0  # 0 to 4
                beak_fwd = head_fwd + 2.0 + beak_idx * 1.5  # Extends forward
                beak_drop = -beak_idx * 0.2  # Slight downward angle
                off_x = px + dir_x * beak_fwd
                off_y = py + head_y + beak_drop
                off_z = pz + dir_z * beak_fwd
            elif part < 14.5:
                # Crest cone (8-14): 7 voxels curving up+back, flaps with flight
                crest_idx = part - 8.0  # 0 to 6
                crest_back = head_fwd - 1.0 - crest_idx * 1.2  # Tighter spacing
                crest_up = head_y + 2.5 + crest_idx * 0.9
                # Crest sways with wing flap — more at the tip
                crest_flap = ti.sin(flap_base) * crest_idx * 0.3
                off_x = px + dir_x * crest_back
                off_y = py + crest_up + crest_flap
                off_z = pz + dir_z * crest_back
            elif part < 19.5:
                # Left wing (15-19): bending flap
                wing_idx = part - 15.0
                spread = 5.0 + wing_idx * 4.0
                flap_delay = wing_idx * 0.25
                flap = ti.sin(flap_base - flap_delay)
                flap_h = (2.0 + wing_idx * 1.5) * flap
                droop_fwd = -wing_idx * 0.4 * ti.max(0.0, -flap)
                off_x = px + perp_x * spread + dir_x * droop_fwd
                off_y = py + flap_h
                off_z = pz + perp_z * spread + dir_z * droop_fwd
            elif part < 24.5:
                # Right wing (20-24): mirror of left
                wing_idx = part - 20.0
                spread = -(5.0 + wing_idx * 4.0)
                flap_delay = wing_idx * 0.25
                flap = ti.sin(flap_base - flap_delay)
                flap_h = (2.0 + wing_idx * 1.5) * flap
                droop_fwd = -wing_idx * 0.4 * ti.max(0.0, -flap)
                off_x = px + perp_x * spread + dir_x * droop_fwd
                off_y = py + flap_h
                off_z = pz + perp_z * spread + dir_z * droop_fwd
            else:
                # Tail (25-29): 5 voxels, tighter spacing, wavy
                tail_idx = part - 25.0
                trail = -(3.0 + tail_idx * 2.5)  # 3, 5.5, 8, 10.5, 13
                # Wave travels down the tail with delay per segment
                tail_sway = (0.4 + tail_idx * 0.4) * ti.sin(time * 2.5 - tail_idx * 0.5)
                tail_bob = (0.3 + tail_idx * 0.2) * ti.sin(time * 2.0 - tail_idx * 0.4)
                off_x = px + dir_x * trail + perp_x * tail_sway
                off_y = py - 0.3 - tail_idx * 0.5 + tail_bob
                off_z = pz + dir_z * trail + perp_z * tail_sway

            bg_offset_x[i] = off_x
            bg_offset_y[i] = off_y
            bg_offset_z[i] = off_z
            bg_brightness[i] = 1.0

        elif anim == BG_ANIM_FLOWER:
            # Wildflower: normal sway + periodic gust that blows them away
            # phase = sway phase, amplitude = sway amount, speed = sway speed
            # Gust cycle: 33 seconds total
            #   0-18: Normal sway
            #   18-19: Wind builds (flowers lean)
            #   19-22: Full gust (flowers fly off, fade out)
            #   22-33: Flowers gently fade back in staggered over ~8s

            gust_period = 33.0
            gust_cycle = time % gust_period

            # Pseudo-random gust direction per cycle
            gust_num = ti.floor(time / gust_period)
            gust_dx = ti.sin(gust_num * 7.13)
            gust_dz = ti.cos(gust_num * 7.13)

            # Wave sweep: flowers on incoming side blow first
            base_x = bg_positions[i][0]
            base_z = bg_positions[i][2]
            along_gust = base_x * gust_dx + base_z * gust_dz
            # Normalize to 0-1 across field (radius ~85)
            wave_delay = (along_gust + 85.0) / 170.0
            wave_delay = ti.max(0.0, ti.min(1.0, wave_delay))

            # Match BG_ANIM_SWAY exactly: t = time * speed + phase
            t_sway = time * speed + phase
            gust_mod = 0.5 + 0.5 * ti.sin(t_sway * 0.3 + phase * 0.5)
            sway_amp = amplitude * (0.4 + 0.6 * gust_mod)
            sway_x = sway_amp * ti.sin(t_sway)
            sway_y = sway_amp * 0.3 * ti.sin(t_sway * 1.5 + 0.5)

            # Compute wind lean + shake matching BG_ANIM_SWAY exactly
            wind_lean_f = 0.0
            wind_shake_f = 0.0
            if gust_cycle > 18.0 and gust_cycle < 22.0:
                if gust_cycle < 19.0:
                    wind_lean_f = (gust_cycle - 18.0) * amplitude * 0.8
                elif gust_cycle < 21.0:
                    wind_lean_f = amplitude * 0.8
                else:
                    wind_lean_f = (22.0 - gust_cycle) * amplitude * 0.8
                wind_shake_f = wind_lean_f * 0.4 * ti.sin(time * 15.0 + phase * 3.0)

            # Base position: sway + wind lean + shake (matches grass blade exactly)
            base_ox = sway_x + wind_lean_f * gust_dx + wind_shake_f * gust_dz
            base_oy = sway_y
            base_oz = wind_lean_f * gust_dz - wind_shake_f * gust_dx

            if gust_cycle < 19.0:
                # Normal sway + wind lean — flower sits on blade tip
                bg_offset_x[i] = base_ox
                bg_offset_y[i] = base_oy
                bg_offset_z[i] = base_oz
                bg_brightness[i] = 1.6 + 0.3 * ti.sin(time * 1.2 + phase)
            elif gust_cycle < 24.0:
                # Full gust — detach from blade and fly off with swirl
                gust_t = gust_cycle - 19.0  # 0 to 5
                # Delayed start based on position in wave
                local_t = gust_t - wave_delay * 1.5
                local_t = ti.max(0.0, local_t)
                # Accelerating push in gust direction (starts from blade position)
                push = local_t * local_t * 15.0
                lift = local_t * 6.0
                # Swirl: spiral perpendicular to gust direction
                swirl_speed = 4.0 + phase * 0.5
                swirl_radius = local_t * 3.0
                swirl_x = swirl_radius * ti.sin(time * swirl_speed + phase * 3.0)
                swirl_z = swirl_radius * ti.cos(time * swirl_speed + phase * 3.0)
                perp_dx = -gust_dz
                perp_dz = gust_dx
                bg_offset_x[i] = base_ox + push * gust_dx + swirl_x * perp_dx
                bg_offset_z[i] = base_oz + push * gust_dz + swirl_x * perp_dz
                bg_offset_y[i] = base_oy + lift + swirl_z
                bg_brightness[i] = 1.6
            else:
                # Staggered respawn — pseudo-random per flower using sin hash
                hash_val = ti.sin(phase * 127.1 + speed * 311.7) * 43758.5453
                stagger = hash_val - ti.floor(hash_val)  # 0 to 1, well distributed
                respawn_start = 24.0 + stagger * 8.0  # Spread over 8 seconds
                respawn_dur = 1.5
                if gust_cycle < respawn_start:
                    # Hidden until this flower's respawn time
                    bg_offset_y[i] = -200.0
                    bg_brightness[i] = 0.0
                else:
                    respawn_t = ti.min(1.0, (gust_cycle - respawn_start) / respawn_dur)
                    # Smooth ease-in: cubic
                    smooth = respawn_t * respawn_t * (3.0 - 2.0 * respawn_t)
                    bg_offset_x[i] = base_ox
                    bg_offset_y[i] = base_oy
                    bg_offset_z[i] = base_oz
                    bg_brightness[i] = smooth * (1.6 + 0.3 * ti.sin(time * 1.2 + phase))

def clear_background():
    """Clear all background voxels by zeroing numpy buffers and flushing to GPU."""
    global _bg_count
    _bg_pos_np[:] = 0; _bg_col_np[:] = 0; _bg_phase_np[:] = 0
    _bg_size_np[:] = 0; _bg_anim_type_np[:] = 0; _bg_anim_speed_np[:] = 0
    _bg_anim_amp_np[:] = 0; _bg_active_np[:] = 0; _bg_angle_np[:] = 0
    _bg_brightness_np[:] = 1.0
    _bg_offset_x_np[:] = 0; _bg_offset_y_np[:] = 0; _bg_offset_z_np[:] = 0
    _bg_count = 0
    bg_theme_active[None] = 0
    bg_flush()

def generate_stars(count: int = 2400, seed: int = 42):
    """
    Generate stars on the outer edges, far from arena.
    Dense starfield surrounding the play area.
    """
    global _bg_count
    import random
    import math
    random.seed(seed)

    clear_background()

    idx = 0
    min_dist = 55  # Stars must be at least this far from center

    for _ in range(count * 3):  # Generate extra, reject those too close
        if idx >= MAX_BACKGROUND_VOXELS or idx >= count:
            break

        # Distribute across large outer area
        x = random.uniform(-120, 120)
        y = random.uniform(-20, 120)
        z = random.uniform(-120, 120)

        # Only keep stars far from arena center
        dist_xz = math.sqrt(x*x + z*z)
        if dist_xz < min_dist:
            continue  # Too close to arena

        _bg_pos_np[idx] = [x, y, z]

        # Color: white to pale blue, slight variation
        blue_tint = random.uniform(0.0, 0.2)
        brightness = random.uniform(0.6, 1.0)
        _bg_col_np[idx] = [
            brightness * (1.0 - blue_tint * 0.5),
            brightness * (1.0 - blue_tint * 0.3),
            brightness
        ]

        # Size: varied, some bigger bright stars
        if random.random() < 0.1:
            _bg_size_np[idx] = random.uniform(0.3, 0.45)  # Bigger bright stars
        else:
            _bg_size_np[idx] = random.uniform(0.12, 0.25)

        # Animation: twinkle with varied speeds
        _bg_anim_type_np[idx] = BG_ANIM_TWINKLE
        _bg_anim_speed_np[idx] = random.uniform(1.5, 4.0)
        _bg_anim_amp_np[idx] = 0.0
        _bg_phase_np[idx] = random.uniform(0, 6.28)

        _bg_brightness_np[idx] = 1.0
        _bg_offset_x_np[idx] = 0.0
        _bg_offset_y_np[idx] = 0.0
        _bg_active_np[idx] = 1
        idx += 1

    _bg_count = idx
    bg_flush()
    bg_theme_active[None] = 1  # Stars theme
    print(f"Generated {idx} stars for background")

def generate_grass(count: int = 2500, seed: int = 42):
    """
    Generate grass carpet just below the arena (where beetles fall into).
    Each blade is 3 voxels tall for actual length.
    """
    global _bg_count
    import random
    import math
    random.seed(seed)

    clear_background()

    idx = 0

    # Flat carpet below arena floor (floor is at y=33)
    grass_y_base = 17.5  # Lower, 3 voxels down

    # Uniform grid of grass blades
    grid_size = int(math.sqrt(count))  # e.g. 35x35 for 1250
    spacing = 110.0 / grid_size  # Cover -55 to 55

    blade_idx = 0
    for gx in range(grid_size):
        for gz in range(grid_size):
            if idx >= MAX_BACKGROUND_VOXELS - 2:
                break

            # Grid with random offset for natural look
            x = -55 + gx * spacing + random.uniform(-0.9, 0.9)
            z = -55 + gz * spacing + random.uniform(-0.9, 0.9)

            # Base height with slight variation
            y_base = grass_y_base + random.uniform(0, 1.5)

            # Shared properties for this blade
            green_var = random.uniform(0.35, 0.7)
            blade_phase = x * 0.08
            blade_speed = 1.0

            # Stack 3 voxels vertically for each grass blade
            for h in range(3):
                if idx >= MAX_BACKGROUND_VOXELS:
                    break

                y = y_base + h * 1.0  # Vertical spacing (bigger)

                _bg_pos_np[idx] = [x, y, z]

                # Color: darker at base, brighter at top
                brightness = 0.7 + h * 0.15
                _bg_col_np[idx] = [
                    random.uniform(0.1, 0.25) * brightness,
                    green_var * brightness,
                    random.uniform(0.05, 0.15) * brightness
                ]

                # Size: slightly smaller at top (tapered blade) - 30% bigger again
                _bg_size_np[idx] = 0.76 - h * 0.135

                # Animation: top sways more than bottom - more intense
                _bg_anim_type_np[idx] = BG_ANIM_SWAY
                _bg_anim_speed_np[idx] = blade_speed
                _bg_anim_amp_np[idx] = 0.6 + h * 0.8  # Bottom: 0.6, Top: 2.2
                _bg_phase_np[idx] = blade_phase

                _bg_brightness_np[idx] = 1.0
                _bg_offset_x_np[idx] = 0.0
                _bg_offset_y_np[idx] = 0.0
                _bg_active_np[idx] = 1
                idx += 1

    _bg_count = idx
    bg_flush()
    bg_theme_active[None] = 2  # Grass theme
    print(f"Generated {idx} grass voxels for background")

def generate_fireflies(count: int = 2800, seed: int = 42):
    """
    Generate floating fireflies on outer edges, far from arena.
    Fireflies wander in gentle paths and glow.
    """
    global _bg_count
    import random
    import math
    random.seed(seed)

    clear_background()

    idx = 0
    min_dist = 55  # Fireflies must be at least this far from center (further from arena)

    for _ in range(count * 4):  # Generate extra, reject those too close
        if idx >= MAX_BACKGROUND_VOXELS or idx >= count:
            break

        # Scattered in outer area - use ring distribution for better coverage
        angle = random.uniform(0, 2 * math.pi)
        dist = random.uniform(min_dist, 110)  # Ring from min_dist outward
        x = math.cos(angle) * dist
        z = math.sin(angle) * dist
        y = random.uniform(15, 90)

        _bg_pos_np[idx] = [x, y, z]

        # Color: warm yellow/green glow (more natural firefly color)
        _bg_col_np[idx] = [
            random.uniform(0.8, 1.0),
            random.uniform(0.9, 1.0),
            random.uniform(0.1, 0.3)
        ]

        # Size: tiny glowing dots
        _bg_size_np[idx] = random.uniform(0.05, 0.12)

        # Animation: firefly wandering path + glow
        _bg_anim_type_np[idx] = BG_ANIM_FIREFLY
        _bg_anim_speed_np[idx] = random.uniform(0.4, 1.0)
        _bg_anim_amp_np[idx] = random.uniform(8.0, 20.0)  # Wander radius - long distances
        _bg_phase_np[idx] = random.uniform(0, 6.28)

        _bg_brightness_np[idx] = 1.0
        _bg_offset_x_np[idx] = 0.0
        _bg_offset_y_np[idx] = 0.0
        _bg_active_np[idx] = 1
        idx += 1

    _bg_count = idx
    bg_flush()
    bg_theme_active[None] = 3  # Fireflies theme
    print(f"Generated {idx} fireflies for background")

def generate_water(count: int = 3000, seed: int = 42):
    """
    Generate whirlpool water particles in uniform concentric rings.
    """
    global _bg_count
    import random
    import math
    random.seed(seed)

    clear_background()

    idx = 0
    water_y_base = 22

    outer_radius = 55.0
    inner_radius = 5.0  # Dead center zone

    # Fermat spiral (golden angle) - no gaps mathematically
    golden_angle = math.pi * (3.0 - math.sqrt(5.0))  # ~137.5 degrees
    num_particles = 5000

    for i in range(num_particles):
        if idx >= MAX_BACKGROUND_VOXELS:
            break

        # Golden angle spiral - each point rotated by golden angle
        angle = i * golden_angle
        # Radius grows with sqrt for even density
        radius = inner_radius + (outer_radius - inner_radius) * math.sqrt(i / num_particles)

        # Convert to cartesian for base position
        x = math.cos(angle) * radius
        z = math.sin(angle) * radius
        y = water_y_base

        _bg_pos_np[idx] = [x, y, z]

        # Tree branch browns/tans
        _bg_col_np[idx] = [0.45, 0.30, 0.15]

        # Size - 30% smaller
        _bg_size_np[idx] = random.uniform(0.22, 0.32)

        # Animation: store initial angle and radius
        _bg_anim_type_np[idx] = BG_ANIM_WATER
        _bg_phase_np[idx] = angle  # Initial angle
        _bg_anim_amp_np[idx] = radius  # Initial radius
        _bg_anim_speed_np[idx] = 1.0  # All same speed

        _bg_brightness_np[idx] = 1.0
        _bg_offset_x_np[idx] = 0.0
        _bg_offset_y_np[idx] = 0.0
        _bg_offset_z_np[idx] = 0.0
        _bg_active_np[idx] = 1
        idx += 1

    _bg_count = idx
    bg_flush()
    bg_theme_active[None] = 4  # Water theme
    print(f"Generated {idx} water particles")

def generate_jellyfish(count: int = 30, seed: int = 42):
    """
    Generate jellyfish floating around the SIDES of the arena.
    Each jellyfish is 5-6 voxels: 1 bell + 4-5 tentacles.
    """
    global _bg_count
    import random
    import math
    random.seed(seed)

    clear_background()

    idx = 0
    arena_radius = 65  # Stay outside this (further from arena)
    outer_radius = 110  # Don't go beyond this
    min_y = 22
    max_y = 42

    # Evenly space jellyfish in rings
    num_rings = 3  # Rings at different heights for hypnotic wave
    per_ring = count // num_rings
    jelly = 0

    for ring in range(num_rings):
        ring_y = min_y + (max_y - min_y) * ring / (num_rings - 1) if num_rings > 1 else (min_y + max_y) / 2
        ring_radius = arena_radius + (outer_radius - arena_radius) * (ring + 1) / (num_rings + 1)

        for j in range(per_ring):
            if idx >= MAX_BACKGROUND_VOXELS - 6:
                break

            # Evenly spaced angle
            angle = (j / per_ring) * 2 * math.pi

            base_x = math.cos(angle) * ring_radius
            base_z = math.sin(angle) * ring_radius
            base_y = ring_y

            # Jellyfish colors - cycle through
            color_choice = jelly % 3
            if color_choice == 0:
                color = ti.Vector([0.9, 0.4, 0.7])  # Pink
            elif color_choice == 1:
                color = ti.Vector([0.5, 0.3, 0.9])  # Purple
            else:
                color = ti.Vector([0.3, 0.8, 0.9])  # Cyan

            jelly_id = float(jelly)
            jelly += 1

            # Bell (main body) - 1 voxel
            _bg_pos_np[idx] = [base_x, base_y, base_z]
            _bg_col_np[idx] = color
            _bg_size_np[idx] = random.uniform(0.6, 0.9)
            _bg_anim_type_np[idx] = BG_ANIM_JELLYFISH
            _bg_phase_np[idx] = jelly_id
            _bg_anim_amp_np[idx] = 0.0  # Bell
            _bg_anim_speed_np[idx] = base_y
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

            # Tentacles - 4-5 voxels hanging below
            num_tentacles = random.randint(4, 5)
            for t in range(num_tentacles):
                if idx >= MAX_BACKGROUND_VOXELS:
                    break

                # Offset tentacles slightly from center
                t_angle = (t / num_tentacles) * 2 * math.pi
                t_offset = 0.4
                t_x = base_x + math.cos(t_angle) * t_offset
                t_z = base_z + math.sin(t_angle) * t_offset
                t_y = base_y - 0.8 - t * 0.3  # Hang below bell

                _bg_pos_np[idx] = [t_x, t_y, t_z]
                _bg_col_np[idx] = color * 0.7  # Slightly dimmer
                _bg_size_np[idx] = random.uniform(0.25, 0.4)
                _bg_anim_type_np[idx] = BG_ANIM_JELLYFISH
                _bg_phase_np[idx] = jelly_id
                _bg_anim_amp_np[idx] = float(t + 1)  # Tentacle index
                _bg_anim_speed_np[idx] = base_y
                _bg_brightness_np[idx] = 1.0
                _bg_offset_x_np[idx] = 0.0
                _bg_offset_y_np[idx] = 0.0
                _bg_offset_z_np[idx] = 0.0
                _bg_active_np[idx] = 1
                idx += 1

    _bg_count = idx
    bg_flush()
    bg_theme_active[None] = 5  # Jellyfish theme
    print(f"Generated {idx} voxels for {count} jellyfish")

def generate_butterflies(count: int = 60, seed: int = 42):
    """
    Generate butterflies floating around the sides of the arena.
    Each butterfly is 5 voxels: 1 body + 4 wings.
    """
    global _bg_count
    import random
    import math
    random.seed(seed)

    clear_background()

    idx = 0
    arena_radius = 75  # Stay outside this (further from arena)
    outer_radius = 120  # Don't go beyond this
    min_y = 35
    max_y = 75

    # Butterfly colors - vibrant
    colors = [
        ti.Vector([1.0, 0.5, 0.1]),   # Orange
        ti.Vector([0.9, 0.2, 0.6]),   # Magenta
        ti.Vector([0.3, 0.6, 1.0]),   # Blue
        ti.Vector([1.0, 0.9, 0.2]),   # Yellow
        ti.Vector([0.6, 0.2, 0.9]),   # Purple
    ]

    # Evenly space butterflies around arena
    for b in range(count):
        if idx >= MAX_BACKGROUND_VOXELS - 9:
            break

        # Evenly spaced angle
        angle = (b / count) * 2 * math.pi
        radius = arena_radius + (b % 3) * 15  # 3 rings
        base_x = math.cos(angle) * radius
        base_z = math.sin(angle) * radius
        base_y = min_y + (b % 5) * 8  # Vary heights

        color = colors[b % len(colors)]
        butterfly_id = float(b)
        move_seed = float(b) * 0.37  # Each butterfly unique, evenly spread

        # Body - 1 voxel (small, dark)
        _bg_pos_np[idx] = [base_x, base_y, base_z]
        _bg_col_np[idx] = color * 0.4  # Darker body
        _bg_size_np[idx] = 0.45
        _bg_anim_type_np[idx] = BG_ANIM_BUTTERFLY
        _bg_phase_np[idx] = butterfly_id
        _bg_anim_amp_np[idx] = 0.0  # Body
        _bg_anim_speed_np[idx] = move_seed
        _bg_brightness_np[idx] = 1.0
        _bg_offset_x_np[idx] = 0.0
        _bg_offset_y_np[idx] = 0.0
        _bg_offset_z_np[idx] = 0.0
        _bg_active_np[idx] = 1
        idx += 1

        # Wings - 2 voxels each (outer big, inner small) = 8 voxels total
        # (wx, wy, wz, part_id, size)
        wing_offsets = [
            # Upper left wing - outer (big) and inner (small)
            (-0.6, 0.2, 0.0, 1.0, 0.8),    # Upper left outer
            (-0.3, 0.15, 0.0, 1.0, 0.5),   # Upper left inner
            # Upper right wing
            (0.6, 0.2, 0.0, 2.0, 0.8),     # Upper right outer
            (0.3, 0.15, 0.0, 2.0, 0.5),    # Upper right inner
            # Lower left wing
            (-0.5, -0.1, 0.0, 3.0, 0.6),   # Lower left outer
            (-0.25, -0.05, 0.0, 3.0, 0.4), # Lower left inner
            # Lower right wing
            (0.5, -0.1, 0.0, 4.0, 0.6),    # Lower right outer
            (0.25, -0.05, 0.0, 4.0, 0.4),  # Lower right inner
        ]

        for wx, wy, wz, part_id, size in wing_offsets:
            if idx >= MAX_BACKGROUND_VOXELS:
                break

            _bg_pos_np[idx] = [base_x + wx, base_y + wy, base_z + wz]
            _bg_col_np[idx] = color
            _bg_size_np[idx] = size
            _bg_anim_type_np[idx] = BG_ANIM_BUTTERFLY
            _bg_phase_np[idx] = butterfly_id
            _bg_anim_amp_np[idx] = part_id  # Wing type
            _bg_anim_speed_np[idx] = move_seed
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

    _bg_count = idx
    bg_flush()
    bg_theme_active[None] = 6  # Butterfly theme
    print(f"Generated {idx} voxels for {count} butterflies")

def generate_waves(count: int = 1600, seed: int = 42):
    """
    Generate water wave blobs in a grid pattern below arena.
    """
    global _bg_count
    import random
    import math
    random.seed(seed)

    clear_background()

    idx = 0
    water_y_base = 17  # 5 voxels lower than grass

    # Uniform grid of water blobs
    grid_size = int(math.sqrt(count))  # e.g. 40x40
    spacing = 110.0 / grid_size

    for gx in range(grid_size):
        for gz in range(grid_size):
            if idx >= MAX_BACKGROUND_VOXELS:
                break

            # Grid with minimal random offset
            x = -55 + gx * spacing + random.uniform(-0.15, 0.15)
            z = -55 + gz * spacing + random.uniform(-0.15, 0.15)
            y = water_y_base + random.uniform(0, 0.3)

            _bg_pos_np[idx] = [x, y, z]

            # Water colors - blue/cyan, varies slightly
            depth_var = random.uniform(0.8, 1.0)
            _bg_col_np[idx] = [
                0.1 * depth_var,
                0.4 * depth_var,
                0.8 * depth_var
            ]

            # Size - 20% bigger
            _bg_size_np[idx] = random.uniform(0.95, 1.12)

            # Animation: store x and z for wave sync
            _bg_anim_type_np[idx] = BG_ANIM_WAVE
            _bg_phase_np[idx] = x  # X position for primary wave
            _bg_anim_amp_np[idx] = z  # Z position for cross-wave
            _bg_anim_speed_np[idx] = 1.0

            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

    _bg_count = idx
    bg_flush()
    bg_theme_active[None] = 7  # Waves theme
    print(f"Generated {idx} water wave blobs")

def generate_tree_branches(count: int = 12, seed: int = 42):
    """
    Generate BIG palm trees around the arena sides.
    Tall solid trunks with bold fronds at the top.
    """
    global _bg_count
    import random
    import math
    random.seed(seed)

    clear_background()

    idx = 0
    tree_radius = 90  # Distance from center - further from arena
    base_y = -15      # Trees start very low (adding to bottom)

    tree_id = 0

    for t in range(count):
        if idx >= MAX_BACKGROUND_VOXELS - 200:
            break

        # Evenly spaced around the circle
        angle = (t / count) * 2 * math.pi
        tree_x = math.cos(angle) * tree_radius
        tree_z = math.sin(angle) * tree_radius

        tree_phase = float(tree_id)
        tree_id += 1

        # Palm trunk color - tan/brown
        trunk_color = ti.Vector([0.45, 0.32, 0.18])
        # Frond color - tropical green
        frond_color = ti.Vector([0.25, 0.50, 0.20])

        # === TALL SOLID TRUNK (dense voxels, no gaps) ===
        trunk_height = random.randint(38, 42)  # More segments - same top, lower bottom
        trunk_spacing = 1.5  # Tight spacing for solid trunk

        for h in range(trunk_height):
            y = base_y + h * trunk_spacing
            # Trunk tapers slightly - starts thick
            trunk_size = 1.5 - h * 0.015
            if trunk_size < 0.8:
                trunk_size = 0.8

            # Trunk barely sways - very subtle
            sway_amp = 0.02 + h * 0.003

            _bg_pos_np[idx] = [tree_x, y, tree_z]
            _bg_col_np[idx] = trunk_color
            _bg_size_np[idx] = trunk_size
            _bg_anim_type_np[idx] = BG_ANIM_TREE
            _bg_phase_np[idx] = tree_phase
            _bg_anim_amp_np[idx] = sway_amp
            _bg_anim_speed_np[idx] = random.uniform(0.9, 1.1)
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

        # === BOLD PALM FRONDS AT TOP ===
        tree_top_y = base_y + trunk_height * trunk_spacing

        # 10 big fronds radiating outward and drooping down
        num_fronds = 10
        for f in range(num_fronds):
            frond_angle = angle + (f / num_fronds) * 2 * math.pi

            # Each frond is multiple voxels in a line going outward and down
            frond_length = random.randint(8, 10)
            for seg in range(frond_length):
                # Frond curves outward and droops
                reach = 1.5 + seg * 1.8
                droop = seg * seg * 0.12  # Quadratic droop

                f_x = tree_x + math.cos(frond_angle) * reach
                f_z = tree_z + math.sin(frond_angle) * reach
                f_y = tree_top_y + 0.5 - droop

                # Fronds taper toward tips
                frond_size = 0.9 - seg * 0.05
                if frond_size < 0.35:
                    frond_size = 0.35

                # More sway on fronds
                sway_amp = 0.6 + seg * 0.2

                _bg_pos_np[idx] = [f_x, f_y, f_z]
                _bg_col_np[idx] = frond_color
                _bg_size_np[idx] = frond_size
                _bg_anim_type_np[idx] = BG_ANIM_TREE
                _bg_phase_np[idx] = tree_phase + f * 0.15  # Slight phase offset per frond
                _bg_anim_amp_np[idx] = sway_amp
                _bg_anim_speed_np[idx] = random.uniform(0.8, 1.2)
                _bg_brightness_np[idx] = 1.0
                _bg_offset_x_np[idx] = 0.0
                _bg_offset_y_np[idx] = 0.0
                _bg_offset_z_np[idx] = 0.0
                _bg_active_np[idx] = 1
                idx += 1

    _bg_count = idx
    bg_flush()
    bg_theme_active[None] = 10  # Tree theme
    print(f"Generated {idx} palm tree voxels ({tree_id} trees)")

def generate_stadium(seed: int = 42):
    """
    Generate stadium seating with beetle larvae spectators.
    Each larva is 4 voxels tall - grub-like creatures standing in tiered rows.
    """
    global _bg_count
    import random
    import math
    random.seed(seed)

    clear_background()

    idx = 0
    base_y = 28  # Stadium starts just below arena level
    inner_radius = 70  # First row of seats - further from arena
    row_spacing = 5  # Distance between rows
    row_height = 3.5  # Height increase per row
    num_rows = 5  # Tiers of seating

    # Generate tiered seating rows
    for row in range(num_rows):
        row_radius = inner_radius + row * row_spacing
        row_y = base_y + row * row_height

        # Number of larvae in this row - sparse for big grubs
        circumference = 2 * math.pi * row_radius
        num_larvae = int(circumference / 40.0)  # Even fewer grubs

        for s in range(num_larvae):
            if idx >= MAX_BACKGROUND_VOXELS - 10:
                break

            angle = (s / num_larvae) * 2 * math.pi
            # Slight random offset
            angle += random.uniform(-0.03, 0.03)

            base_x = math.cos(angle) * row_radius
            base_z = math.sin(angle) * row_radius

            # Larva properties - shared for all segments
            larva_phase = random.uniform(0, 6.28)
            larva_speed = random.uniform(1.0, 1.5)

            # Color variation per larva - creamy white body
            body_tint = random.uniform(0.85, 1.0)

            # Each larva is 4 voxels stacked vertically - BIG fat grubs
            for seg in range(4):
                if idx >= MAX_BACKGROUND_VOXELS:
                    break

                y = row_y + seg * 1.8  # Big vertical spacing

                # Body segments: really fat grubby shape
                if seg == 0:  # Bottom/tail
                    seg_size = 0.9
                    # Cream/white tail
                    color = ti.Vector([0.90 * body_tint, 0.85 * body_tint, 0.70 * body_tint])
                elif seg == 1:  # Lower body - fattest
                    seg_size = 1.3
                    color = ti.Vector([0.92 * body_tint, 0.88 * body_tint, 0.72 * body_tint])
                elif seg == 2:  # Upper body - still fat
                    seg_size = 1.15
                    color = ti.Vector([0.88 * body_tint, 0.82 * body_tint, 0.68 * body_tint])
                else:  # Head - darker brown, bigger
                    seg_size = 1.0
                    color = ti.Vector([0.45, 0.30, 0.18])

                _bg_pos_np[idx] = [base_x, y, base_z]
                _bg_col_np[idx] = color
                _bg_size_np[idx] = seg_size
                _bg_anim_type_np[idx] = BG_ANIM_SCRUNCH
                _bg_phase_np[idx] = larva_phase
                _bg_anim_amp_np[idx] = float(seg)  # Segment index for scrunch wave
                _bg_anim_speed_np[idx] = larva_speed
                _bg_angle_np[idx] = angle  # Store angle for stadium wave
                _bg_brightness_np[idx] = 1.0
                _bg_offset_x_np[idx] = 0.0
                _bg_offset_y_np[idx] = 0.0
                _bg_offset_z_np[idx] = 0.0
                _bg_active_np[idx] = 1
                idx += 1

    _bg_count = idx
    bg_flush()
    stadium_excitement[None] = 0.0  # Reset excitement
    stadium_excitement_target[None] = 0.0
    bg_theme_active[None] = 11  # Stadium theme
    print(f"Generated {idx} larva voxels in stadium")

def trigger_stadium_excitement():
    """Call this when a score happens to make the crowd go wild."""
    stadium_excitement[None] = 1.0
    print("STADIUM CROWD CHEERING!")

def decay_stadium_excitement(dt: float):
    """Call each frame to slowly decay excitement."""
    current = stadium_excitement[None]
    if current > 0.01:
        # Simple slow decay over ~6 seconds
        stadium_excitement[None] = current * (1.0 - dt * 0.25)
    else:
        stadium_excitement[None] = 0.0

# ============================================================
# STACKABLE THEME TOGGLE FUNCTIONS
# ============================================================

def compact_background_voxels(removed_start: int, removed_count: int):
    """Shift voxels down to fill gap left by removed theme."""
    global theme_start_idx, theme_count, _bg_count
    total = _bg_count
    shift_start = removed_start + removed_count

    # Shift all voxels after the gap down
    for i in range(shift_start, total):
        new_idx = i - removed_count
        # Copy all fields
        _bg_pos_np[new_idx] = _bg_pos_np[i]
        _bg_col_np[new_idx] = _bg_col_np[i]
        _bg_size_np[new_idx] = _bg_size_np[i]
        _bg_phase_np[new_idx] = _bg_phase_np[i]
        _bg_anim_type_np[new_idx] = _bg_anim_type_np[i]
        _bg_anim_speed_np[new_idx] = _bg_anim_speed_np[i]
        _bg_anim_amp_np[new_idx] = _bg_anim_amp_np[i]
        _bg_active_np[new_idx] = _bg_active_np[i]
        _bg_brightness_np[new_idx] = _bg_brightness_np[i]
        _bg_offset_x_np[new_idx] = _bg_offset_x_np[i]
        _bg_offset_y_np[new_idx] = _bg_offset_y_np[i]
        _bg_offset_z_np[new_idx] = _bg_offset_z_np[i]
        _bg_angle_np[new_idx] = _bg_angle_np[i]

    # Clear the old slots at the end
    for i in range(total - removed_count, total):
        _bg_active_np[i] = 0
        _bg_size_np[i] = 0.0

    # Update indices for themes that shifted
    for theme_id in list(active_themes):
        if theme_id in theme_start_idx and theme_start_idx[theme_id] > removed_start:
            theme_start_idx[theme_id] -= removed_count

    _bg_count = total - removed_count
    bg_flush()

def remove_theme(theme_id: int):
    """Remove a theme and compact the voxel arrays."""
    global active_themes, theme_start_idx, theme_count

    if theme_id not in active_themes:
        return  # Not active

    start = theme_start_idx[theme_id]
    count = theme_count[theme_id]

    # Compact the array
    compact_background_voxels(start, count)

    # Remove from tracking
    active_themes.remove(theme_id)
    del theme_start_idx[theme_id]
    del theme_count[theme_id]

    # Update bg_theme_active for renderer
    if len(active_themes) == 0:
        bg_theme_active[None] = 0

    print(f"Removed theme {theme_id}, {count} voxels freed")

def toggle_theme(theme_id: int):
    """Toggle a theme on or off."""
    if theme_id in active_themes:
        remove_theme(theme_id)
    else:
        # Add the theme
        add_functions = {
            THEME_STARS: lambda: add_stars(1200),
            THEME_GRASS: lambda: add_grass(625),
            THEME_FIREFLIES: lambda: add_fireflies(930),
            THEME_WATER: lambda: add_water(2250),
            THEME_JELLYFISH: lambda: add_jellyfish(240),
            THEME_BUTTERFLIES: lambda: add_butterflies(80),
            THEME_WAVES: lambda: add_waves(3600),
            THEME_PALM_TREES: lambda: add_tree_branches(12),
            THEME_STADIUM: lambda: add_stadium(),
            THEME_LAVA: lambda: add_lava(),
            THEME_RAIN: lambda: add_rain(),
            THEME_CLOUDS: lambda: add_clouds(),
            THEME_PTERODACTYL: lambda: add_pterodactyl(),
            THEME_SWAMP: lambda: add_swamp(),
            THEME_DESERT: lambda: add_desert(),
            THEME_COMET: lambda: add_comet(),
        }
        if theme_id in add_functions:
            add_functions[theme_id]()
            # Enable background rendering
            bg_theme_active[None] = 1

def is_theme_active(theme_id: int) -> bool:
    """Check if a theme is currently active."""
    return theme_id in active_themes

# ============================================================
# STACKABLE THEME ADD FUNCTIONS (append to existing voxels)
# ============================================================

def add_stars(count: int = 1200, seed: int = 42):
    """Add constellation star patterns + scattered background stars."""
    global active_themes, theme_start_idx, theme_count, _bg_count
    import random
    import math
    random.seed(seed)

    if THEME_STARS in active_themes:
        return

    start_idx = _bg_count
    idx = start_idx

    # === CONSTELLATION DEFINITIONS ===
    # Each: vertices as (x, y) local coords, edges as (v1, v2) pairs
    constellations = [
        {  # Stag Beetle - curved mandibles, body, legs
            'verts': [
                # Left mandible (single curve)
                (-2, 20),                              # 0: left horn tip
                (-5, 17),                              # 1: left horn curve
                (-4, 14),                              # 2: left horn base
                # Right mandible
                (2, 20),                               # 3: right horn tip
                (5, 17),                               # 4: right horn curve
                (4, 14),                               # 5: right horn base
                # Head
                (0, 13),                               # 6: head
                # Body
                (-4, 10), (4, 10),                     # 7-8: body top
                (-5, 5), (5, 5),                       # 9-10: body widest
                (-3, 0), (3, 0),                       # 11-12: body bottom
                # Legs
                (-8, 9), (8, 9),                       # 13-14: front legs
                (-9, 4), (9, 4),                       # 15-16: back legs
            ],
            'edges': [
                (0, 1), (1, 2), (2, 6),                # left horn curve
                (3, 4), (4, 5), (5, 6),                # right horn curve
                (6, 7), (6, 8), (7, 8),                # head to body top
                (7, 9), (8, 10),                       # body sides
                (9, 11), (10, 12), (11, 12),           # body bottom
                (7, 13), (8, 14),                      # front legs
                (9, 15), (10, 16),                     # back legs
            ],
        },
    ]

    # === PLACEMENT: single beetle constellation ===
    sky_radius = 155.0
    placements = [
        (0, -1),        # beetle
    ]
    scales = [2.6]
    tilts = [45]  # beetle tilted left

    for ci, constellation in enumerate(constellations):
        az_deg, el_deg = placements[ci]
        scale = scales[ci]
        tilt = math.radians(tilts[ci])
        az = math.radians(az_deg)
        el = math.radians(el_deg)

        # Center on sky sphere
        center_x = sky_radius * math.cos(az) * math.cos(el)
        center_y = sky_radius * math.sin(el)
        center_z = sky_radius * math.sin(az) * math.cos(el)

        # Local coordinate axes on the sphere surface (facing inward)
        right_x = -math.sin(az)
        right_y = 0.0
        right_z = math.cos(az)
        up_x = -math.cos(az) * math.sin(el)
        up_y = math.cos(el)
        up_z = -math.sin(az) * math.sin(el)

        # Constellation ID for synced pulsing
        const_phase = ci * 1.8

        verts = constellation['verts']
        edges = constellation['edges']

        # Transform vertices to world space
        world_verts = []
        cos_tilt = math.cos(tilt)
        sin_tilt = math.sin(tilt)
        for lx, ly in verts:
            lx *= scale
            ly *= scale
            # Rotate in local 2D plane so constellation is tilted
            rx = lx * cos_tilt - ly * sin_tilt
            ry = lx * sin_tilt + ly * cos_tilt
            lx, ly = rx, ry
            wx = center_x + lx * right_x + ly * up_x
            wy = center_y + lx * right_y + ly * up_y
            wz = center_z + lx * right_z + ly * up_z
            world_verts.append((wx, wy, wz))

        # Vertex star colors: warm white, blue-white, yellow-white
        star_colors = [
            (1.0, 0.95, 0.85),   # warm white
            (0.85, 0.9, 1.0),    # blue-white
            (1.0, 0.92, 0.7),    # yellow
        ]

        # Place vertex stars (bright, bigger)
        for vi, (wx, wy, wz) in enumerate(world_verts):
            if idx >= MAX_BACKGROUND_VOXELS - 10:
                break
            _bg_pos_np[idx] = [wx, wy, wz]
            sc = star_colors[vi % len(star_colors)]
            _bg_col_np[idx] = [sc[0], sc[1], sc[2]]
            _bg_size_np[idx] = random.uniform(0.5, 0.7)
            _bg_anim_type_np[idx] = BG_ANIM_CONSTELLATION
            _bg_phase_np[idx] = const_phase
            _bg_anim_amp_np[idx] = 0.0  # 0 = vertex star
            _bg_anim_speed_np[idx] = 1.0
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

        # Place line dots along each edge
        for v1i, v2i in edges:
            w1 = world_verts[v1i]
            w2 = world_verts[v2i]
            edge_len = math.sqrt(sum((a - b) ** 2 for a, b in zip(w1, w2)))
            num_dots = max(3, int(edge_len / 1.8))

            for d in range(1, num_dots + 1):
                if idx >= MAX_BACKGROUND_VOXELS - 10:
                    break
                frac = d / (num_dots + 1)
                dx = w1[0] + frac * (w2[0] - w1[0])
                dy = w1[1] + frac * (w2[1] - w1[1])
                dz = w1[2] + frac * (w2[2] - w1[2])

                _bg_pos_np[idx] = [dx, dy, dz]
                _bg_col_np[idx] = [0.7, 0.75, 0.9]  # Dim blue-white
                _bg_size_np[idx] = random.uniform(0.15, 0.25)
                _bg_anim_type_np[idx] = BG_ANIM_CONSTELLATION
                _bg_phase_np[idx] = const_phase
                _bg_anim_amp_np[idx] = frac  # Position along edge (for traveling sparkle)
                _bg_anim_speed_np[idx] = 1.0
                _bg_brightness_np[idx] = 1.0
                _bg_offset_x_np[idx] = 0.0
                _bg_offset_y_np[idx] = 0.0
                _bg_offset_z_np[idx] = 0.0
                _bg_active_np[idx] = 1
                idx += 1

    # === SCATTERED BACKGROUND STARS (dimmer, fill the sky) ===
    min_dist = 85
    bg_star_count = count
    attempts = 0
    placed = 0
    while placed < bg_star_count and attempts < bg_star_count * 3:
        attempts += 1
        if idx >= MAX_BACKGROUND_VOXELS - 5:
            break

        x = random.uniform(-150, 150)
        y = random.uniform(-10, 130)
        z = random.uniform(-150, 150)

        dist_xz = math.sqrt(x * x + z * z)
        if dist_xz < min_dist:
            continue

        _bg_pos_np[idx] = [x, y, z]
        blue_tint = random.uniform(0.0, 0.25)
        brightness = random.uniform(0.4, 0.8)
        _bg_col_np[idx] = [
            brightness * (1.0 - blue_tint * 0.5),
            brightness * (1.0 - blue_tint * 0.3),
            brightness
        ]

        if random.random() < 0.08:
            _bg_size_np[idx] = random.uniform(0.15, 0.25)
        else:
            _bg_size_np[idx] = random.uniform(0.05, 0.12)

        _bg_anim_type_np[idx] = BG_ANIM_TWINKLE
        _bg_anim_speed_np[idx] = random.uniform(1.5, 4.0)
        _bg_anim_amp_np[idx] = 0.0
        _bg_phase_np[idx] = random.uniform(0, 6.28)
        _bg_brightness_np[idx] = 1.0
        _bg_offset_x_np[idx] = 0.0
        _bg_offset_y_np[idx] = 0.0
        _bg_offset_z_np[idx] = 0.0
        _bg_active_np[idx] = 1
        idx += 1
        placed += 1

    theme_start_idx[THEME_STARS] = start_idx
    theme_count[THEME_STARS] = idx - start_idx
    active_themes.add(THEME_STARS)
    _bg_count = idx
    bg_flush()
    print(f"Added {idx - start_idx} star voxels (total: {idx})")

def add_comet():
    """Add a giant comet orbiting the arena."""
    global active_themes, theme_start_idx, theme_count, _bg_count
    import math

    if THEME_COMET in active_themes:
        return

    start_idx = _bg_count
    idx = start_idx

    dome_R = 4.5
    orbit_R = 112.0

    def add_comet_voxel(idx, trail, lateral, vertical, size, r, g, b):
        _bg_pos_np[idx] = [0.0, 40.0, 0.0]
        _bg_col_np[idx] = [r, g, b]
        _bg_size_np[idx] = size
        _bg_anim_type_np[idx] = BG_ANIM_COMET
        _bg_phase_np[idx] = trail
        _bg_anim_amp_np[idx] = lateral
        _bg_anim_speed_np[idx] = vertical
        _bg_brightness_np[idx] = 1.0
        _bg_offset_x_np[idx] = 0.0
        _bg_offset_y_np[idx] = 0.0
        _bg_offset_z_np[idx] = 0.0
        _bg_active_np[idx] = 1

    # === FRONT TIP: static bright core ===
    tip_trail = -(dome_R) / orbit_R
    add_comet_voxel(idx, tip_trail, 0.0, 0.0, 4.1, 1.0, 0.4, 0.1)
    idx += 1

    # === ANIMATED RINGS: flow from dome front to tail end ===
    num_rings = 10
    voxels_per_ring = 8
    for ring_id in range(num_rings):
        cycle_offset = ring_id * (8.0 / num_rings)
        for k in range(voxels_per_ring):
            if idx >= MAX_BACKGROUND_VOXELS - 5: break
            theta = k * 2 * math.pi / voxels_per_ring
            _bg_pos_np[idx] = [0.0, 40.0, 0.0]
            _bg_col_np[idx] = [1.0, 0.3, 0.05]
            _bg_size_np[idx] = 0.9
            _bg_anim_type_np[idx] = BG_ANIM_COMET
            _bg_phase_np[idx] = cycle_offset
            _bg_anim_amp_np[idx] = 100.0 + theta
            _bg_anim_speed_np[idx] = 0.0
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

    # === CORE SPINE: static center tail line ===
    num_core = 15
    for t in range(num_core):
        if idx >= MAX_BACKGROUND_VOXELS - 5: break
        frac = t / num_core
        trail = 0.005 + frac * 0.5
        size = 2.2 * (1.0 - frac * 0.85)
        r = 1.0 - frac * 0.3
        g = 0.35 - frac * 0.25
        b = 0.08 - frac * 0.05
        add_comet_voxel(idx, trail, 0.0, 0.0, size, r, g, b)
        idx += 1

    theme_start_idx[THEME_COMET] = start_idx
    theme_count[THEME_COMET] = idx - start_idx
    active_themes.add(THEME_COMET)
    _bg_count = idx
    bg_flush()
    print(f"Added {idx - start_idx} comet voxels (total: {idx})")

def add_grass(count: int = 625, seed: int = 42):
    """Add lush grass with ground cover, clumps, varied heights, and wildflowers."""
    global active_themes, theme_start_idx, theme_count, _bg_count
    import random
    import math
    random.seed(seed)

    if THEME_GRASS in active_themes:
        return

    start_idx = _bg_count
    idx = start_idx
    grass_y_base = 17.5
    circle_radius = 85.0

    # Flower colors for wildflower accents
    flower_colors = [
        ti.Vector([0.9, 0.8, 0.2]),   # Yellow
        ti.Vector([0.9, 0.9, 0.85]),   # White
        ti.Vector([0.6, 0.3, 0.8]),    # Purple
        ti.Vector([0.8, 0.2, 0.2]),    # Red
    ]

    # === PASS 1: GROUND COVER - dense carpet of fat low voxels ===
    ground_grid = 40  # Fewer, bigger voxels for same coverage
    ground_spacing = (circle_radius * 2) / ground_grid

    for gx in range(ground_grid):
        for gz in range(ground_grid):
            if idx >= MAX_BACKGROUND_VOXELS - 20:
                break

            x = -circle_radius + gx * ground_spacing + random.uniform(-1.0, 1.0)
            z = -circle_radius + gz * ground_spacing + random.uniform(-1.0, 1.0)

            dist = math.sqrt(x * x + z * z)
            if dist > circle_radius:
                continue

            _bg_pos_np[idx] = [x, grass_y_base - 0.5 + random.uniform(-0.3, 0.3), z]
            # Dark earthy green
            g = random.uniform(0.2, 0.35)
            _bg_col_np[idx] = [g * 0.4, g, g * 0.25]
            _bg_size_np[idx] = random.uniform(2.2, 2.7)
            _bg_anim_type_np[idx] = BG_ANIM_NONE  # Static - no animation needed for ground cover
            _bg_anim_speed_np[idx] = 0.0
            _bg_anim_amp_np[idx] = 0.0
            _bg_phase_np[idx] = 0.0
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

    # === PASS 2: GRASS BLADE CLUSTERS with varied heights and flowers ===
    num_clusters = 150
    min_cluster_dist = 8.0  # Minimum distance between cluster centers
    cluster_centers = []

    for c in range(num_clusters):
        if idx >= MAX_BACKGROUND_VOXELS - 20:
            break

        # Random cluster center within circle, with min distance check
        cx, cz = 0.0, 0.0
        placed = False
        for _ in range(30):  # Rejection sampling
            cx = random.uniform(-circle_radius, circle_radius)
            cz = random.uniform(-circle_radius, circle_radius)
            if math.sqrt(cx * cx + cz * cz) > circle_radius:
                continue
            # Check min distance from existing clusters
            too_close = False
            for ox, oz in cluster_centers:
                if math.sqrt((cx - ox)**2 + (cz - oz)**2) < min_cluster_dist:
                    too_close = True
                    break
            if not too_close:
                placed = True
                break

        if not placed:
            continue  # Skip this cluster if can't find valid spot

        cluster_centers.append((cx, cz))

        # 3-6 blades per cluster
        blades_in_cluster = random.randint(3, 6)

        for b in range(blades_in_cluster):
            if idx >= MAX_BACKGROUND_VOXELS - 10:
                break

            # Blades close together within cluster
            x = cx + random.uniform(-2.5, 2.5)
            z = cz + random.uniform(-2.5, 2.5)

            dist = math.sqrt(x * x + z * z)
            if dist > circle_radius:
                continue

            y_base = grass_y_base + 1.0 + random.uniform(0, 1.2)
            green_var = random.uniform(0.35, 0.7)
            blade_phase = x * 0.08 + z * 0.03
            blade_speed = random.uniform(0.8, 1.2)

            # Varied heights: weighted random
            roll = random.random()
            if roll < 0.2:
                blade_height = 2
            elif roll < 0.6:
                blade_height = 3
            elif roll < 0.9:
                blade_height = 4
            else:
                blade_height = 5

            for h in range(blade_height):
                if idx >= MAX_BACKGROUND_VOXELS:
                    break

                y = y_base + h * 1.0

                _bg_pos_np[idx] = [x, y, z]

                brightness = 0.7 + h * 0.1
                _bg_col_np[idx] = [
                    random.uniform(0.1, 0.25) * brightness,
                    green_var * brightness,
                    random.uniform(0.05, 0.15) * brightness
                ]

                # Fatter base, tapered
                blade_size = 1.48 - h * 0.24
                if blade_size < 0.48:
                    blade_size = 0.48
                _bg_size_np[idx] = blade_size

                _bg_anim_type_np[idx] = BG_ANIM_SWAY
                _bg_anim_speed_np[idx] = blade_speed
                _bg_anim_amp_np[idx] = 0.25 + h * 0.45
                _bg_phase_np[idx] = blade_phase
                _bg_brightness_np[idx] = 1.0
                _bg_offset_x_np[idx] = 0.0
                _bg_offset_y_np[idx] = 0.0
                _bg_offset_z_np[idx] = 0.0
                _bg_active_np[idx] = 1
                idx += 1

            # Wildflower accent on ~22.5% of blades
            if random.random() < 0.225 and idx < MAX_BACKGROUND_VOXELS:
                flower_y = y_base + blade_height * 1.0
                _bg_pos_np[idx] = [x, flower_y, z]
                _bg_col_np[idx] = flower_colors[random.randint(0, 3)]
                _bg_size_np[idx] = 0.45
                _bg_anim_type_np[idx] = BG_ANIM_FLOWER
                _bg_anim_speed_np[idx] = blade_speed
                _bg_anim_amp_np[idx] = 0.4 + blade_height * 0.7
                _bg_phase_np[idx] = blade_phase
                _bg_brightness_np[idx] = 1.0
                _bg_offset_x_np[idx] = 0.0
                _bg_offset_y_np[idx] = 0.0
                _bg_offset_z_np[idx] = 0.0
                _bg_active_np[idx] = 1
                idx += 1

    # === GIANT CATERPILLAR ===
    caterpillar_id = 0.0
    # Segments: 0=head, 1-12=body, 13-16=left antenna chain, 17-20=right antenna chain
    parts = [
        # (seg_id, size, r, g, b)
        (0, 3.5, 0.85, 0.35, 0.10),       # Head (orange-red)
        (1, 3.2, 0.75, 0.85, 0.12),       # Body 1 (bright yellow-green)
        (2, 3.4, 0.90, 0.40, 0.12),       # Body 2 (orange stripe)
        (3, 3.2, 0.75, 0.85, 0.12),       # Body 3
        (4, 3.4, 0.90, 0.40, 0.12),       # Body 4 (orange stripe)
        (5, 3.2, 0.75, 0.85, 0.12),       # Body 5
        (6, 3.4, 0.90, 0.40, 0.12),       # Body 6 (orange stripe)
        (7, 3.2, 0.75, 0.85, 0.12),       # Body 7
        (8, 3.3, 0.90, 0.40, 0.12),       # Body 8 (orange stripe)
        (9, 3.0, 0.75, 0.85, 0.12),       # Body 9
        (10, 2.8, 0.90, 0.40, 0.12),      # Body 10 (orange stripe)
        (11, 2.4, 0.75, 0.85, 0.12),      # Body 11
        (12, 1.8, 0.85, 0.35, 0.10),      # Body 12 (tail tip, orange-red)
        # Left antenna chain (6 voxels, tapering)
        (13, 1.2, 0.10, 0.10, 0.10),
        (14, 1.0, 0.10, 0.10, 0.10),
        (15, 0.85, 0.10, 0.10, 0.10),
        (16, 0.7, 0.10, 0.10, 0.10),
        (17, 0.5, 0.10, 0.10, 0.10),
        (18, 0.6, 0.85, 0.15, 0.10),      # Tip: red bulb
        # Right antenna chain (6 voxels, tapering)
        (19, 1.2, 0.10, 0.10, 0.10),
        (20, 1.0, 0.10, 0.10, 0.10),
        (21, 0.85, 0.10, 0.10, 0.10),
        (22, 0.7, 0.10, 0.10, 0.10),
        (23, 0.5, 0.10, 0.10, 0.10),
        (24, 0.6, 0.85, 0.15, 0.10),      # Tip: red bulb
    ]

    for seg_id, size, r, g, b in parts:
        if idx >= MAX_BACKGROUND_VOXELS - 5:
            break
        _bg_pos_np[idx] = [0, 0, 0]
        _bg_col_np[idx] = [r, g, b]
        _bg_size_np[idx] = size
        _bg_anim_type_np[idx] = BG_ANIM_CATERPILLAR
        _bg_phase_np[idx] = caterpillar_id
        _bg_anim_amp_np[idx] = float(seg_id)
        _bg_anim_speed_np[idx] = 0.08  # Slow orbit
        _bg_brightness_np[idx] = 1.0
        _bg_offset_x_np[idx] = 0.0
        _bg_offset_y_np[idx] = 0.0
        _bg_offset_z_np[idx] = 0.0
        _bg_active_np[idx] = 1
        idx += 1

    theme_start_idx[THEME_GRASS] = start_idx
    theme_count[THEME_GRASS] = idx - start_idx
    active_themes.add(THEME_GRASS)
    _bg_count = idx
    bg_flush()
    print(f"Added {idx - start_idx} grass voxels (total: {idx})")

def add_lava(seed: int = 42):
    """Add lava ground with dark crust, molten flow, eruption sprays, and floating embers."""
    global active_themes, theme_start_idx, theme_count, _bg_count
    import random
    import math
    random.seed(seed)

    if THEME_LAVA in active_themes:
        return

    start_idx = _bg_count
    idx = start_idx
    lava_y_base = 17
    circle_radius = 85.0

    # === PASS 1: DARK CRUST - static cooled rock base ===
    crust_grid = 35
    crust_spacing = (circle_radius * 2) / crust_grid

    for gx in range(crust_grid):
        for gz in range(crust_grid):
            if idx >= MAX_BACKGROUND_VOXELS - 200:
                break

            x = -circle_radius + gx * crust_spacing + random.uniform(-1.5, 1.5)
            z = -circle_radius + gz * crust_spacing + random.uniform(-1.5, 1.5)

            dist = math.sqrt(x * x + z * z)
            if dist > circle_radius:
                continue

            _bg_pos_np[idx] = [x, lava_y_base - 0.5 + random.uniform(-0.3, 0.3), z]
            # Dark charcoal/brown rock
            g = random.uniform(0.08, 0.2)
            _bg_col_np[idx] = [g * 1.2, g * 0.5, g * 0.2]
            _bg_size_np[idx] = random.uniform(2.5, 3.0)
            _bg_anim_type_np[idx] = BG_ANIM_NONE
            _bg_anim_speed_np[idx] = 0.0
            _bg_anim_amp_np[idx] = 0.0
            _bg_phase_np[idx] = 0.0
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

    # === PASS 2: MOLTEN LAVA FLOW - glowing animated surface ===
    lava_grid = 50
    lava_spacing = (circle_radius * 2) / lava_grid

    for gx in range(lava_grid):
        for gz in range(lava_grid):
            if idx >= MAX_BACKGROUND_VOXELS - 120:
                break

            x = -circle_radius + gx * lava_spacing + random.uniform(-0.3, 0.3)
            z = -circle_radius + gz * lava_spacing + random.uniform(-0.3, 0.3)

            dist = math.sqrt(x * x + z * z)
            if dist > circle_radius:
                continue

            _bg_pos_np[idx] = [x, lava_y_base + random.uniform(-0.2, 0.2), z]
            # Bright red molten base color (brightness modulation creates glow)
            r = random.uniform(0.85, 1.0)
            g = random.uniform(0.08, 0.18)
            b = random.uniform(0.01, 0.05)
            _bg_col_np[idx] = [r, g, b]
            _bg_size_np[idx] = random.uniform(1.0, 1.3)
            _bg_anim_type_np[idx] = BG_ANIM_LAVA
            _bg_anim_speed_np[idx] = 1.0
            _bg_anim_amp_np[idx] = z  # Store z position for cross-wave
            _bg_phase_np[idx] = x  # Store x position for primary wave
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

    # === PASS 3: ERUPTION SPRAYS - gentle bubbles + occasional BOOM eruptions ===
    arena_avoid_radius = 35.0  # Stay outside the basic circular arena (~32 + margin)
    num_eruption_points = 15
    for ep in range(num_eruption_points):
        if idx >= MAX_BACKGROUND_VOXELS - 80:
            break

        # Random eruption point within lava field, OUTSIDE the arena
        placed = False
        ep_x, ep_z = 0.0, 0.0
        for _ in range(30):
            ep_angle = random.uniform(0, 2 * math.pi)
            ep_radius = random.uniform(15, 75)
            ep_x = math.cos(ep_angle) * ep_radius
            ep_z = math.sin(ep_angle) * ep_radius
            if math.sqrt(ep_x * ep_x + ep_z * ep_z) > arena_avoid_radius:
                placed = True
                break
        if not placed:
            continue

        # ~20% chance of BOOM eruption, rest are gentle bubbles
        is_boom = random.random() < 0.2

        if is_boom:
            # BOOM: huge dramatic plume, long build-up then explosive burst
            particles_per_eruption = random.randint(60, 200)
            cycle_period = random.uniform(10.0, 16.0)  # Long cycle — mostly quiet
            launch_height_base = 38.0
            launch_height_var = 25.0
            spread = 3.0
            voxel_size_min = 0.15
            voxel_size_max = 0.35
        else:
            # Gentle: lazy slow bubbles, smooth arcs
            particles_per_eruption = random.randint(3, 5)
            cycle_period = random.uniform(5.0, 8.0)  # Slower cycle for smoother motion
            launch_height_base = 4.0
            launch_height_var = 3.0
            spread = 1.5
            voxel_size_min = 0.2
            voxel_size_max = 0.4

        # All particles share a base offset so the eruption point bursts together
        eruption_base_offset = random.uniform(0, cycle_period)

        # Build a mini volcano cone — animated to scrunch before eruption
        volcano_layers = 4
        for v in range(volcano_layers):
            if idx >= MAX_BACKGROUND_VOXELS - 200:
                break
            v_size = 4.2 - v * 0.75  # 4.2, 3.45, 2.7, 1.95
            v_y = lava_y_base + 1.5 + v * 2.0
            rock_r = 0.15 + v * 0.12
            rock_g = 0.06 + v * 0.03
            rock_b = 0.03 + v * 0.01
            _bg_pos_np[idx] = [ep_x, v_y, ep_z]
            _bg_col_np[idx] = [rock_r, rock_g, rock_b]
            _bg_size_np[idx] = v_size
            _bg_anim_type_np[idx] = BG_ANIM_LAVA_VOLCANO
            _bg_anim_speed_np[idx] = cycle_period
            _bg_anim_amp_np[idx] = float(v)  # Layer index
            _bg_phase_np[idx] = eruption_base_offset  # Synced with spray timing
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

        for p in range(particles_per_eruption):
            if idx >= MAX_BACKGROUND_VOXELS - 60:
                break

            # Stagger within a SHORT burst window (not spread across whole cycle)
            burst_window = cycle_period * 0.09  # Stagger within 9% of cycle
            time_offset = eruption_base_offset + random.uniform(0, burst_window)
            launch_height = launch_height_base + random.uniform(0, launch_height_var)

            _bg_pos_np[idx] = [ep_x + random.uniform(-0.5, 0.5), lava_y_base + 8.0, ep_z + random.uniform(-0.5, 0.5)]
            # Fiery bright lava colors
            if is_boom:
                _bg_col_np[idx] = [1.0, random.uniform(0.4, 0.7), random.uniform(0.05, 0.2)]
            else:
                _bg_col_np[idx] = [1.0, random.uniform(0.25, 0.5), random.uniform(0.02, 0.1)]
            _bg_size_np[idx] = random.uniform(voxel_size_min, voxel_size_max)
            _bg_anim_type_np[idx] = BG_ANIM_LAVA_SPRAY
            _bg_anim_speed_np[idx] = cycle_period
            _bg_anim_amp_np[idx] = launch_height
            _bg_phase_np[idx] = time_offset
            _bg_brightness_np[idx] = 0.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

    # === MOLTEN LAVA SNAIL ===
    snail_id = 0.0
    snail_parts = [
        # BODY: 0=head, 1-12=body (orange-red, widest in middle, longer slug)
        (0, 3.3, 1.0, 0.45, 0.08),        # Head (bright orange)
        (1, 3.5, 0.95, 0.40, 0.06),
        (2, 3.85, 0.90, 0.35, 0.05),
        (3, 4.2, 0.85, 0.30, 0.05),        # Widest
        (4, 4.2, 0.85, 0.30, 0.05),
        (5, 4.2, 0.82, 0.28, 0.05),
        (6, 3.85, 0.80, 0.28, 0.04),
        (7, 3.5, 0.75, 0.25, 0.04),
        (8, 3.1, 0.70, 0.22, 0.03),
        (9, 2.75, 0.65, 0.20, 0.03),
        (10, 2.2, 0.60, 0.18, 0.03),
        (11, 1.65, 0.55, 0.16, 0.03),
        (12, 1.1, 0.50, 0.14, 0.02),       # Tail tip
    ]
    # SHELL: spiral voxels (23 voxels, warm gradient — deep ember to burnt sienna)
    for si in range(23):
        t_norm = si / 22.0  # 0 to 1
        size = 5.0 - t_norm * 3.0  # 5.0 at center, 2.0 at tip
        if si % 3 == 1:
            # Warm amber highlight
            r = 0.85 - t_norm * 0.15
            g = 0.38 - t_norm * 0.08
            b = 0.08 + t_norm * 0.02
        else:
            # Deep burnt sienna / ember
            r = 0.55 - t_norm * 0.12
            g = 0.18 - t_norm * 0.04
            b = 0.06 + t_norm * 0.02
        snail_parts.append((13 + si, size, r, g, b))

    # EYE STALKS: 36-40=left (5 voxels), 41-45=right (5 voxels)
    for eye in range(10):
        chain = eye % 5
        is_tip = chain == 4
        snail_parts.append((36 + eye, 0.6 if not is_tip else 0.9,
                           0.15 if not is_tip else 1.0,
                           0.10 if not is_tip else 0.6,
                           0.05 if not is_tip else 0.05))

    # LAVA DRIPS: 46-65
    for di in range(20):
        snail_parts.append((46 + di, random.uniform(0.3, 0.6), 1.0, 0.4, 0.05))

    for part_id, size, r, g, b in snail_parts:
        if idx >= MAX_BACKGROUND_VOXELS - 5:
            break
        _bg_pos_np[idx] = [0, 0, 0]
        _bg_col_np[idx] = [r, g, b]
        _bg_size_np[idx] = size
        _bg_anim_type_np[idx] = BG_ANIM_LAVA_SNAIL
        _bg_phase_np[idx] = snail_id
        _bg_anim_amp_np[idx] = float(part_id)
        _bg_anim_speed_np[idx] = 0.06  # Slow orbit
        _bg_brightness_np[idx] = 1.0
        _bg_offset_x_np[idx] = 0.0
        _bg_offset_y_np[idx] = 0.0
        _bg_offset_z_np[idx] = 0.0
        _bg_active_np[idx] = 1
        idx += 1

    theme_start_idx[THEME_LAVA] = start_idx
    theme_count[THEME_LAVA] = idx - start_idx
    active_themes.add(THEME_LAVA)
    _bg_count = idx
    bg_flush()
    print(f"Added {idx - start_idx} lava voxels (total: {idx})")

def add_rain(seed: int = 42):
    """Add rain with falling drops and ground splashes."""
    global active_themes, theme_start_idx, theme_count, _bg_count
    import random
    import math
    random.seed(seed)

    if THEME_RAIN in active_themes:
        return

    start_idx = _bg_count
    idx = start_idx
    rain_y_base = 17
    circle_radius = 85.0

    # === PASS 1: FALLING RAINDROPS (grid-based for guaranteed even coverage) ===
    rain_grid = 39  # 39x39 grid → ~1200 drops inside circle
    rain_spacing = (circle_radius * 2) / rain_grid

    for gx in range(rain_grid):
        for gz in range(rain_grid):
            if idx >= MAX_BACKGROUND_VOXELS - 200:
                break

            x = -circle_radius + gx * rain_spacing + random.uniform(-2.0, 2.0)
            z = -circle_radius + gz * rain_spacing + random.uniform(-2.0, 2.0)

            dist = math.sqrt(x * x + z * z)
            if dist > circle_radius:
                continue

            y = random.uniform(50, 90)

            _bg_pos_np[idx] = [x, y, z]
            # Pale blue-white
            r = random.uniform(0.55, 0.70)
            g = random.uniform(0.65, 0.80)
            b = random.uniform(0.90, 1.00)
            _bg_col_np[idx] = [r, g, b]
            _bg_size_np[idx] = random.uniform(0.03, 0.07)
            _bg_anim_type_np[idx] = BG_ANIM_RAIN
            _bg_phase_np[idx] = random.uniform(0, 100)
            _bg_anim_amp_np[idx] = random.uniform(50, 80)  # Fall distance
            _bg_anim_speed_np[idx] = random.uniform(55, 75)  # Fall speed (fast!)
            _bg_active_np[idx] = 1
            _bg_brightness_np[idx] = 1.0
            idx += 1

    # === PASS 2: GROUND SPLASHES (grid-based for even ground coverage) ===
    splash_grid = 11  # 11x11 → ~95 splashes inside circle
    splash_spacing = (circle_radius * 2) / splash_grid

    for gx in range(splash_grid):
        for gz in range(splash_grid):
            if idx >= MAX_BACKGROUND_VOXELS - 50:
                break

            x = -circle_radius + gx * splash_spacing + random.uniform(-3.0, 3.0)
            z = -circle_radius + gz * splash_spacing + random.uniform(-3.0, 3.0)

            dist = math.sqrt(x * x + z * z)
            if dist > circle_radius:
                continue

            _bg_pos_np[idx] = [x, rain_y_base, z]
        # White-blue splash
        r = random.uniform(0.70, 0.85)
        g = random.uniform(0.80, 0.90)
        b = random.uniform(0.95, 1.00)
        _bg_col_np[idx] = [r, g, b]
        _bg_size_np[idx] = random.uniform(0.15, 0.20)
        _bg_anim_type_np[idx] = BG_ANIM_RAIN_SPLASH
        _bg_phase_np[idx] = random.uniform(0, 10)  # Stagger timing + direction seed
        _bg_anim_amp_np[idx] = random.uniform(1.0, 2.0)  # Outward spread radius
        _bg_anim_speed_np[idx] = random.uniform(0.4, 0.8)  # Cycle period
        _bg_active_np[idx] = 1
        _bg_brightness_np[idx] = 0.0
        idx += 1

    theme_start_idx[THEME_RAIN] = start_idx
    theme_count[THEME_RAIN] = idx - start_idx
    active_themes.add(THEME_RAIN)
    _bg_count = idx
    bg_flush()
    print(f"Added {idx - start_idx} rain voxels (total: {idx})")

def add_clouds(seed: int = 42):
    """Add puffy cloud groups that drift and morph above the arena."""
    global active_themes, theme_start_idx, theme_count, _bg_count
    import random
    import math
    random.seed(seed)

    if THEME_CLOUDS in active_themes:
        return

    start_idx = _bg_count
    idx = start_idx

    num_clusters = 6
    for c in range(num_clusters):
        if idx >= MAX_BACKGROUND_VOXELS - 50:
            break

        # Place clusters in a ring around the arena
        angle = (c / num_clusters) * 2 * math.pi + random.uniform(-0.3, 0.3)
        radius = random.uniform(75, 120)
        cx = math.cos(angle) * radius
        cz = math.sin(angle) * radius
        cy = random.uniform(42, 50)
        cluster_id = float(c)

        # Cumulus: dense flat base slab + puffy dome bumps on top
        cloud_width = random.uniform(13.1, 19.7)
        cloud_depth = random.uniform(9.8, 15.3)
        cloud_height = random.uniform(7.6, 12.0)

        # 3-4 dome bumps for puffy cauliflower top, clustered near center
        num_bumps = random.randint(3, 4)
        bump_centers = []
        for _ in range(num_bumps):
            bx = random.uniform(-cloud_width * 0.35, cloud_width * 0.35)
            bz = random.uniform(-cloud_depth * 0.35, cloud_depth * 0.35)
            br = random.uniform(3.5, 5.6)  # Bump radius
            bh = random.uniform(0.7, 1.0)  # Height multiplier
            bump_centers.append((bx, bz, br, bh))

        # PASS A: Flat base layer — dense slab at cy, bigger in center
        num_base = random.randint(18, 25)
        for p in range(num_base):
            if idx >= MAX_BACKGROUND_VOXELS - 10:
                break
            # Center-biased distribution (squared random pulls toward center)
            rx = random.gauss(0, 0.35)
            rz = random.gauss(0, 0.35)
            rx = max(-1.0, min(1.0, rx))
            rz = max(-1.0, min(1.0, rz))
            ox = rx * cloud_width
            oz = rz * cloud_depth

            # Distance from center (0=center, 1=edge)
            dist = math.sqrt(rx * rx + rz * rz)
            if dist > 1.0:
                continue

            # Bigger voxels in center, smaller at edges
            center_factor = 1.0 - dist
            voxel_size = 1.05 + center_factor * 1.75  # ~1.05 at edge, ~2.8 at center

            _bg_pos_np[idx] = [cx + ox, cy, cz + oz]  # All at cy — perfectly flat
            gray = random.uniform(0.88, 0.93)
            _bg_col_np[idx] = [gray, gray, gray + random.uniform(0.0, 0.03)]
            _bg_size_np[idx] = voxel_size
            _bg_anim_type_np[idx] = BG_ANIM_CLOUD
            _bg_phase_np[idx] = cluster_id
            _bg_anim_amp_np[idx] = random.uniform(0, 10)
            _bg_anim_speed_np[idx] = random.uniform(0.6, 1.2)
            _bg_active_np[idx] = 1
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            idx += 1

        # PASS B: Puffy dome top — voxels inside bump spheres, bigger near center
        num_top = random.randint(25, 35)
        for p in range(num_top):
            if idx >= MAX_BACKGROUND_VOXELS - 10:
                break

            # Pick a random bump to place voxel in
            bx, bz, br, bh = random.choice(bump_centers)
            # Center-biased placement inside bump (cubed random packs toward center)
            r_raw = random.uniform(0, 1.0) ** 0.6 * br
            angle_h = random.uniform(0, 2 * math.pi)
            angle_v = random.uniform(0, math.pi * 0.45)  # Upper hemisphere, slightly tighter
            ox = bx + r_raw * math.cos(angle_h) * math.sin(angle_v)
            oz = bz + r_raw * math.sin(angle_h) * math.sin(angle_v)
            oy = r_raw * math.cos(angle_v) * bh * (cloud_height / br)

            # Reject if outside cloud footprint
            norm_dist = math.sqrt((ox / cloud_width) ** 2 + (oz / cloud_depth) ** 2)
            if norm_dist > 1.2:
                continue

            # Distance from cloud center for size scaling
            center_factor = 1.0 - min(1.0, norm_dist)
            voxel_size = 0.84 + center_factor * 1.96  # ~0.84 at edge, ~2.8 at center

            gray = random.uniform(0.92, 1.0)
            _bg_pos_np[idx] = [cx + ox, cy + oy, cz + oz]
            _bg_col_np[idx] = [gray, gray, gray + random.uniform(0.0, 0.03)]
            _bg_size_np[idx] = voxel_size
            _bg_anim_type_np[idx] = BG_ANIM_CLOUD
            _bg_phase_np[idx] = cluster_id  # Shared drift per cluster
            _bg_anim_amp_np[idx] = random.uniform(0, 10)  # Individual seed
            _bg_anim_speed_np[idx] = random.uniform(0.6, 1.2)  # Individual pulse rate
            _bg_active_np[idx] = 1
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            idx += 1

    theme_start_idx[THEME_CLOUDS] = start_idx
    theme_count[THEME_CLOUDS] = idx - start_idx
    active_themes.add(THEME_CLOUDS)
    _bg_count = idx
    bg_flush()
    print(f"Added {idx - start_idx} cloud voxels (total: {idx})")

def add_pterodactyl(seed: int = 42):
    """Add pterodactyl(s) that orbit around the arena at cloud level."""
    global active_themes, theme_start_idx, theme_count, _bg_count
    import random
    import math
    random.seed(seed)

    if THEME_PTERODACTYL in active_themes:
        return

    start_idx = _bg_count
    idx = start_idx

    # Pterodactyl parts:
    # 0: body, 1: neck, 2: head
    # 3-7: beak cone (5 voxels)
    # 8-14: crest cone (7 voxels, curves up+back)
    # 15-19: left wing (5 segments)
    # 20-24: right wing (5 segments)
    # 25-29: tail (5 voxels, tight spacing)
    # Total: 30 voxels per pterodactyl

    num_pteros = 1
    orbit_radius = 70.0
    cloud_y = 48.5

    for p in range(num_pteros):
        ptero_id = float(p)

        parts = [
            # (part_type, size, r, g, b)
            (0, 4.0, 0.45, 0.35, 0.25),     # Body
            (1, 2.0, 0.42, 0.33, 0.23),     # Neck
            (2, 2.8, 0.48, 0.37, 0.27),     # Head
            # Beak cone: 5 voxels shrinking to a point
            (3, 1.8, 0.50, 0.38, 0.28),     # Beak 1 (base)
            (4, 1.4, 0.52, 0.39, 0.28),     # Beak 2
            (5, 1.0, 0.54, 0.40, 0.28),     # Beak 3
            (6, 0.7, 0.56, 0.41, 0.28),     # Beak 4
            (7, 0.4, 0.58, 0.42, 0.28),     # Beak 5 (tip)
            # Crest cone: 7 voxels curving up and back (tighter spacing)
            (8, 1.6, 0.55, 0.25, 0.15),     # Crest 1 (base)
            (9, 1.4, 0.56, 0.245, 0.145),   # Crest 2
            (10, 1.2, 0.57, 0.24, 0.14),    # Crest 3
            (11, 1.0, 0.58, 0.235, 0.135),  # Crest 4
            (12, 0.8, 0.59, 0.23, 0.13),    # Crest 5
            (13, 0.6, 0.61, 0.22, 0.12),    # Crest 6
            (14, 0.4, 0.63, 0.21, 0.11),    # Crest 7 (tip)
            # Left wing
            (15, 2.8, 0.50, 0.40, 0.28),    # LW 1 (inner)
            (16, 2.4, 0.48, 0.38, 0.26),    # LW 2
            (17, 2.0, 0.46, 0.36, 0.24),    # LW 3
            (18, 1.6, 0.44, 0.34, 0.22),    # LW 4
            (19, 1.0, 0.42, 0.32, 0.20),    # LW 5 (tip)
            # Right wing
            (20, 2.8, 0.50, 0.40, 0.28),    # RW 1 (inner)
            (21, 2.4, 0.48, 0.38, 0.26),    # RW 2
            (22, 2.0, 0.46, 0.36, 0.24),    # RW 3
            (23, 1.6, 0.44, 0.34, 0.22),    # RW 4
            (24, 1.0, 0.42, 0.32, 0.20),    # RW 5 (tip)
            # Tail: 5 voxels, tighter spacing
            (25, 2.8, 0.42, 0.33, 0.23),    # Tail 1
            (26, 2.4, 0.41, 0.32, 0.22),    # Tail 2
            (27, 2.0, 0.40, 0.31, 0.21),    # Tail 3
            (28, 1.5, 0.39, 0.30, 0.20),    # Tail 4
            (29, 1.0, 0.38, 0.29, 0.19),    # Tail 5 (tip)
        ]

        for part_type, size, r, g, b in parts:
            if idx >= MAX_BACKGROUND_VOXELS - 10:
                break

            _bg_pos_np[idx] = [0, cloud_y, 0]
            _bg_col_np[idx] = [r, g, b]
            _bg_size_np[idx] = size
            _bg_anim_type_np[idx] = BG_ANIM_PTERODACTYL
            _bg_phase_np[idx] = ptero_id  # Which pterodactyl (shared orbit)
            _bg_anim_amp_np[idx] = float(part_type)  # Which body part
            _bg_anim_speed_np[idx] = 0.3  # Orbit speed
            _bg_active_np[idx] = 1
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            idx += 1

    theme_start_idx[THEME_PTERODACTYL] = start_idx
    theme_count[THEME_PTERODACTYL] = idx - start_idx
    active_themes.add(THEME_PTERODACTYL)
    _bg_count = idx
    bg_flush()
    print(f"Added {idx - start_idx} pterodactyl voxels (total: {idx})")

def add_fireflies(count: int = 2800, seed: int = 42):
    """Add fireflies to the background (appends to existing voxels)."""
    global active_themes, theme_start_idx, theme_count, _bg_count
    import random
    import math
    random.seed(seed)

    if THEME_FIREFLIES in active_themes:
        return

    start_idx = _bg_count
    idx = start_idx
    min_dist = 55

    for _ in range(count * 4):
        if idx >= MAX_BACKGROUND_VOXELS or idx >= start_idx + count:
            break

        angle = random.uniform(0, 2 * math.pi)
        dist = random.uniform(min_dist, 110)
        x = math.cos(angle) * dist
        z = math.sin(angle) * dist
        y = random.uniform(15, 90)

        _bg_pos_np[idx] = [x, y, z]
        _bg_col_np[idx] = [
            random.uniform(0.8, 1.0),
            random.uniform(0.9, 1.0),
            random.uniform(0.1, 0.3)
        ]
        _bg_size_np[idx] = random.uniform(0.05, 0.12)
        _bg_anim_type_np[idx] = BG_ANIM_FIREFLY
        _bg_anim_speed_np[idx] = random.uniform(0.4, 1.0)
        _bg_anim_amp_np[idx] = random.uniform(8.0, 20.0)
        _bg_phase_np[idx] = random.uniform(0, 6.28)
        _bg_brightness_np[idx] = 1.0
        _bg_offset_x_np[idx] = 0.0
        _bg_offset_y_np[idx] = 0.0
        _bg_offset_z_np[idx] = 0.0
        _bg_active_np[idx] = 1
        idx += 1

    theme_start_idx[THEME_FIREFLIES] = start_idx
    theme_count[THEME_FIREFLIES] = idx - start_idx
    active_themes.add(THEME_FIREFLIES)
    _bg_count = idx
    bg_flush()
    print(f"Added {idx - start_idx} fireflies (total: {idx})")

def add_water(count: int = 3000, seed: int = 42):
    """Add water to the background (appends to existing voxels)."""
    global active_themes, theme_start_idx, theme_count, _bg_count
    import random
    import math
    random.seed(seed)

    if THEME_WATER in active_themes:
        return

    start_idx = _bg_count
    idx = start_idx
    water_y_base = 22

    num_rings = 15
    particles_per_ring = count // num_rings

    for ring in range(num_rings):
        if idx >= MAX_BACKGROUND_VOXELS:
            break

        ring_radius = 35 + ring * 5
        ring_phase = ring * 0.5

        for p in range(particles_per_ring):
            if idx >= MAX_BACKGROUND_VOXELS:
                break

            angle = (p / particles_per_ring) * 2 * math.pi
            x = math.cos(angle) * ring_radius
            z = math.sin(angle) * ring_radius
            y = water_y_base + random.uniform(-0.5, 0.5)

            _bg_pos_np[idx] = [x, y, z]
            blue_var = random.uniform(0.4, 0.8)
            _bg_col_np[idx] = [0.1, 0.3 + blue_var * 0.3, 0.5 + blue_var * 0.5]
            _bg_size_np[idx] = random.uniform(0.4, 0.7)
            _bg_anim_type_np[idx] = BG_ANIM_WATER
            _bg_anim_speed_np[idx] = random.uniform(0.8, 1.2)
            _bg_anim_amp_np[idx] = ring_phase
            _bg_phase_np[idx] = angle
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

    theme_start_idx[THEME_WATER] = start_idx
    theme_count[THEME_WATER] = idx - start_idx
    active_themes.add(THEME_WATER)
    _bg_count = idx
    bg_flush()
    print(f"Added {idx - start_idx} water particles (total: {idx})")

def add_jellyfish(count: int = 30, seed: int = 42):
    """Add jellyfish to the background (appends to existing voxels)."""
    global active_themes, theme_start_idx, theme_count, _bg_count
    import random
    import math
    random.seed(seed)

    if THEME_JELLYFISH in active_themes:
        return

    start_idx = _bg_count
    idx = start_idx
    arena_radius = 65
    outer_radius = 110
    min_y = 22
    max_y = 42
    num_rings = 3
    per_ring = count // num_rings
    jelly = 0

    for ring in range(num_rings):
        ring_y = min_y + (max_y - min_y) * ring / (num_rings - 1) if num_rings > 1 else (min_y + max_y) / 2
        ring_radius = arena_radius + (outer_radius - arena_radius) * (ring + 1) / (num_rings + 1)

        for j in range(per_ring):
            if idx >= MAX_BACKGROUND_VOXELS - 6:
                break

            angle = (j / per_ring) * 2 * math.pi
            base_x = math.cos(angle) * ring_radius
            base_z = math.sin(angle) * ring_radius
            base_y = ring_y

            color_choice = jelly % 3
            if color_choice == 0:
                color = ti.Vector([0.9, 0.4, 0.7])
            elif color_choice == 1:
                color = ti.Vector([0.5, 0.3, 0.9])
            else:
                color = ti.Vector([0.3, 0.8, 0.9])

            jelly_id = float(jelly)
            jelly += 1

            # Bell
            _bg_pos_np[idx] = [base_x, base_y, base_z]
            _bg_col_np[idx] = color
            _bg_size_np[idx] = random.uniform(0.6, 0.9)
            _bg_anim_type_np[idx] = BG_ANIM_JELLYFISH
            _bg_phase_np[idx] = jelly_id
            _bg_anim_amp_np[idx] = 0.0
            _bg_anim_speed_np[idx] = base_y
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

            # Tentacles
            num_tentacles = random.randint(4, 5)
            for t in range(num_tentacles):
                if idx >= MAX_BACKGROUND_VOXELS:
                    break

                t_angle = (t / num_tentacles) * 2 * math.pi
                t_offset = 0.4
                t_x = base_x + math.cos(t_angle) * t_offset
                t_z = base_z + math.sin(t_angle) * t_offset
                t_y = base_y - 0.8 - t * 0.3

                _bg_pos_np[idx] = [t_x, t_y, t_z]
                _bg_col_np[idx] = color * 0.7
                _bg_size_np[idx] = random.uniform(0.25, 0.4)
                _bg_anim_type_np[idx] = BG_ANIM_JELLYFISH
                _bg_phase_np[idx] = jelly_id
                _bg_anim_amp_np[idx] = float(t + 1)
                _bg_anim_speed_np[idx] = base_y
                _bg_brightness_np[idx] = 1.0
                _bg_offset_x_np[idx] = 0.0
                _bg_offset_y_np[idx] = 0.0
                _bg_offset_z_np[idx] = 0.0
                _bg_active_np[idx] = 1
                idx += 1

    theme_start_idx[THEME_JELLYFISH] = start_idx
    theme_count[THEME_JELLYFISH] = idx - start_idx
    active_themes.add(THEME_JELLYFISH)
    _bg_count = idx
    bg_flush()
    print(f"Added {idx - start_idx} jellyfish voxels (total: {idx})")

def add_butterflies(count: int = 60, seed: int = 42):
    """Add butterflies to the background (appends to existing voxels)."""
    global active_themes, theme_start_idx, theme_count, _bg_count
    import random
    import math
    random.seed(seed)

    if THEME_BUTTERFLIES in active_themes:
        return

    start_idx = _bg_count
    idx = start_idx
    arena_radius = 75
    outer_radius = 120
    min_y = 35
    max_y = 75

    colors = [
        ti.Vector([1.0, 0.5, 0.1]),
        ti.Vector([0.9, 0.2, 0.6]),
        ti.Vector([0.3, 0.6, 1.0]),
        ti.Vector([1.0, 0.9, 0.2]),
        ti.Vector([0.6, 0.2, 0.9]),
    ]

    for b in range(count):
        if idx >= MAX_BACKGROUND_VOXELS - 9:
            break

        angle = (b / count) * 2 * math.pi
        radius = arena_radius + (b % 3) * 15
        base_x = math.cos(angle) * radius
        base_z = math.sin(angle) * radius
        base_y = min_y + (b % 5) * 8

        color = colors[b % len(colors)]
        butterfly_id = float(b)
        move_seed = float(b) * 0.37

        # Body
        _bg_pos_np[idx] = [base_x, base_y, base_z]
        _bg_col_np[idx] = color * 0.4
        _bg_size_np[idx] = 0.45
        _bg_anim_type_np[idx] = BG_ANIM_BUTTERFLY
        _bg_phase_np[idx] = butterfly_id
        _bg_anim_amp_np[idx] = 0.0
        _bg_anim_speed_np[idx] = move_seed
        _bg_brightness_np[idx] = 1.0
        _bg_offset_x_np[idx] = 0.0
        _bg_offset_y_np[idx] = 0.0
        _bg_offset_z_np[idx] = 0.0
        _bg_active_np[idx] = 1
        idx += 1

        # Wings: 2 per side (inner + outer), 4 total
        # part_type encoding: 1=left inner, 2=right inner, 3=left outer, 4=right outer
        wing_parts = [
            (1.0, 0.55),   # left inner - bigger
            (2.0, 0.55),   # right inner - bigger
            (3.0, 0.4),    # left outer - smaller tip
            (4.0, 0.4),    # right outer - smaller tip
        ]

        for wing_id, w_size in wing_parts:
            if idx >= MAX_BACKGROUND_VOXELS:
                break

            _bg_pos_np[idx] = [base_x, base_y, base_z]
            _bg_col_np[idx] = color if wing_id <= 2 else color * 0.8
            _bg_size_np[idx] = w_size
            _bg_anim_type_np[idx] = BG_ANIM_BUTTERFLY
            _bg_phase_np[idx] = butterfly_id
            _bg_anim_amp_np[idx] = wing_id
            _bg_anim_speed_np[idx] = move_seed
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

    theme_start_idx[THEME_BUTTERFLIES] = start_idx
    theme_count[THEME_BUTTERFLIES] = idx - start_idx
    active_themes.add(THEME_BUTTERFLIES)
    _bg_count = idx
    bg_flush()
    print(f"Added {idx - start_idx} butterfly voxels (total: {idx})")

def add_waves(count: int = 1600, seed: int = 42):
    """Add waves to the background (appends to existing voxels)."""
    global active_themes, theme_start_idx, theme_count, _bg_count
    import random
    import math
    random.seed(seed)

    if THEME_WAVES in active_themes:
        return

    start_idx = _bg_count
    idx = start_idx
    water_y_base = 17  # 5 voxels lower than grass

    # Bigger grid, but only keep voxels within circle
    circle_radius = 85.0  # Circular boundary
    grid_extent = 90.0  # Grid goes from -90 to 90
    grid_size = int(math.sqrt(count * 1.6))  # Bigger grid to compensate for cut corners
    spacing = (grid_extent * 2) / grid_size

    for gx in range(grid_size):
        for gz in range(grid_size):
            if idx >= MAX_BACKGROUND_VOXELS:
                break

            # Grid with minimal random offset
            x = -grid_extent + gx * spacing + random.uniform(-0.15, 0.15)
            z = -grid_extent + gz * spacing + random.uniform(-0.15, 0.15)

            # Only keep voxels within circle
            dist = math.sqrt(x * x + z * z)
            if dist > circle_radius:
                continue

            y = water_y_base + random.uniform(0, 0.3)

            _bg_pos_np[idx] = [x, y, z]

            # Water colors - blue/cyan, varies slightly
            depth_var = random.uniform(0.8, 1.0)
            _bg_col_np[idx] = [
                0.1 * depth_var,
                0.4 * depth_var,
                0.8 * depth_var
            ]

            # Size - 20% bigger
            _bg_size_np[idx] = random.uniform(0.95, 1.12)

            # Animation: store x and z for wave sync
            _bg_anim_type_np[idx] = BG_ANIM_WAVE
            _bg_phase_np[idx] = x  # X position for primary wave
            _bg_anim_amp_np[idx] = z  # Z position for cross-wave
            _bg_anim_speed_np[idx] = 1.0

            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

    # === ADD SEA SERPENT/DRAGON ===
    num_serpents = 1  # One big sea dragon
    num_segments = 20  # Long snaking body with more segments for smoothness

    for serpent_id in range(num_serpents):
        if idx >= MAX_BACKGROUND_VOXELS - num_segments - 5:
            break

        # Starting angle for this serpent
        serpent_phase = serpent_id * (2 * math.pi / max(1, num_serpents))

        for seg in range(num_segments):
            # Base position (will be overridden by animation)
            _bg_pos_np[idx] = [0.0, water_y_base, 0.0]

            # Color: dark blue/teal body, menacing red head
            if seg == 0:
                # HEAD - menacing red/dark, smaller
                _bg_col_np[idx] = [0.8, 0.1, 0.1]
                _bg_size_np[idx] = 2.6
            elif seg == 1:
                # Neck - still red-ish
                _bg_col_np[idx] = [0.6, 0.1, 0.15]
                _bg_size_np[idx] = 2.5
            elif seg == 2:
                # Transition to body
                _bg_col_np[idx] = [0.3, 0.15, 0.25]
                _bg_size_np[idx] = 2.5
            elif seg < 6:
                # Upper body - darker blue
                _bg_col_np[idx] = [0.12, 0.2, 0.4]
                _bg_size_np[idx] = 2.6
            elif seg < num_segments - 4:
                # Body segments - blue/teal, consistent size
                t = seg / num_segments
                _bg_col_np[idx] = [0.1, 0.25 + t * 0.15, 0.45 + t * 0.15]
                _bg_size_np[idx] = 2.4
            else:
                # Tail - tapers, lighter color
                tail_pos = seg - (num_segments - 4)
                _bg_col_np[idx] = [0.15, 0.4, 0.55]
                _bg_size_np[idx] = 2.2 - tail_pos * 0.4

            # Minimum size for tail tip
            if _bg_size_np[idx] < 0.8:
                _bg_size_np[idx] = 0.8

            _bg_anim_type_np[idx] = BG_ANIM_FISH
            _bg_phase_np[idx] = serpent_phase
            _bg_anim_amp_np[idx] = float(seg)  # Segment index
            _bg_anim_speed_np[idx] = float(serpent_id)
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

        # === ANTENNAS - two long feelers on top of head ===
        for antenna_side in range(2):  # Left and right
            for a in range(6):  # 6 voxels per antenna for no gaps
                _bg_pos_np[idx] = [0.0, water_y_base, 0.0]
                # Yellow/gold antennas
                _bg_col_np[idx] = [0.9, 0.7, 0.2]
                _bg_size_np[idx] = 0.9 - a * 0.08  # Bigger, taper toward tip
                if _bg_size_np[idx] < 0.4:
                    _bg_size_np[idx] = 0.4
                _bg_anim_type_np[idx] = BG_ANIM_FISH
                _bg_phase_np[idx] = serpent_phase
                # 100+ = left antenna, 200+ = right antenna
                _bg_anim_amp_np[idx] = float(100 + antenna_side * 100 + a)
                _bg_anim_speed_np[idx] = float(serpent_id)
                _bg_brightness_np[idx] = 1.0
                _bg_offset_x_np[idx] = 0.0
                _bg_offset_y_np[idx] = 0.0
                _bg_offset_z_np[idx] = 0.0
                _bg_active_np[idx] = 1
                idx += 1

        # === SPLASH VOXELS - burst when head dives into water ===
        num_splash = 24
        for s in range(num_splash):
            _bg_pos_np[idx] = [0.0, water_y_base, 0.0]
            # Water splash colors - blue-white
            white_mix = random.uniform(0.3, 0.7)
            _bg_col_np[idx] = [
                0.4 + 0.6 * white_mix,
                0.6 + 0.4 * white_mix,
                0.85 + 0.15 * white_mix
            ]
            _bg_size_np[idx] = random.uniform(0.5, 1.0)
            _bg_anim_type_np[idx] = BG_ANIM_SERPENT_SPLASH
            _bg_phase_np[idx] = serpent_phase
            _bg_anim_amp_np[idx] = float(s)  # Splash voxel index (burst direction)
            _bg_anim_speed_np[idx] = float(serpent_id)
            _bg_brightness_np[idx] = 0.0  # Start hidden
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1


    # === ADD JUMPING DOLPHINS (2 dolphins) ===
    # Base pos (0,0,0) — animation offsets are absolute world coordinates
    num_dolphins = 2
    num_dolphin_parts = 32   # Body parts per dolphin (0-15 body, 16-31 blowhole spray)
    num_dolphin_splash = 16  # Splash particles per dolphin

    for dolph_id in range(num_dolphins):
        if idx >= MAX_BACKGROUND_VOXELS - (num_dolphin_parts + num_dolphin_splash + 10):
            break

        # Stagger timing so dolphins don't jump simultaneously
        jump_offset = dolph_id * 14.0  # ~half cycle offset

        for part in range(num_dolphin_parts):
            _bg_pos_np[idx] = [0.0, 0.0, 0.0]  # Origin — offsets are absolute
            _bg_anim_type_np[idx] = BG_ANIM_JUMPING_FISH
            _bg_phase_np[idx] = float(dolph_id)
            _bg_anim_amp_np[idx] = float(part)
            _bg_anim_speed_np[idx] = jump_offset
            _bg_brightness_np[idx] = 0.0  # Start hidden
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = -200.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1

            if part == 0:
                # ROSTRUM (beak) — pointed snout
                _bg_col_np[idx] = [0.55, 0.58, 0.64]
                _bg_size_np[idx] = 1.6
            elif part == 1:
                # HEAD (melon) — rounded forehead
                _bg_col_np[idx] = [0.48, 0.51, 0.58]
                _bg_size_np[idx] = 3.0
            elif part <= 7:
                # BODY (2-7): sleek taper, widest at segment 3
                body_sizes = [3.8, 4.4, 4.2, 3.6, 2.8, 2.0]
                bi = part - 2
                _bg_size_np[idx] = body_sizes[bi]
                t = bi / 5.0
                _bg_col_np[idx] = [0.40 - t * 0.11, 0.43 - t * 0.11, 0.52 - t * 0.10]
            elif part == 8:
                # TAIL STOCK — narrow
                _bg_col_np[idx] = [0.27, 0.30, 0.38]
                _bg_size_np[idx] = 1.4
            elif part <= 10:
                # TAIL FLUKES (9-10) — horizontal spread
                _bg_col_np[idx] = [0.25, 0.28, 0.36]
                _bg_size_np[idx] = 3.0
            elif part == 11:
                # DORSAL FIN — tall, iconic
                _bg_col_np[idx] = [0.28, 0.31, 0.39]
                _bg_size_np[idx] = 3.6
            elif part <= 13:
                # PECTORAL FLIPPERS (12-13)
                _bg_col_np[idx] = [0.38, 0.41, 0.48]
                _bg_size_np[idx] = 2.0
            elif part <= 15:
                # EYES (14-15)
                _bg_col_np[idx] = [0.03, 0.03, 0.05]
                _bg_size_np[idx] = 1.0
            else:
                # BLOWHOLE SPRAY (16-31) — misty droplets
                white_amt = random.uniform(0.5, 0.9)
                _bg_col_np[idx] = [0.7 + 0.3 * white_amt, 0.78 + 0.22 * white_amt, 0.88 + 0.12 * white_amt]
                _bg_size_np[idx] = random.uniform(0.8, 1.2)

            idx += 1

        # SPLASH PARTICLES — bigger for dramatic effect
        for s in range(num_dolphin_splash):
            _bg_pos_np[idx] = [0.0, 0.0, 0.0]  # Origin — offsets are absolute
            white_mix = random.uniform(0.3, 0.7)
            _bg_col_np[idx] = [0.5 + 0.5 * white_mix, 0.65 + 0.35 * white_mix, 0.85 + 0.15 * white_mix]
            _bg_size_np[idx] = random.uniform(1.0, 2.0)
            _bg_anim_type_np[idx] = BG_ANIM_FISH_SPLASH
            _bg_phase_np[idx] = float(dolph_id)
            _bg_anim_amp_np[idx] = float(s)
            _bg_anim_speed_np[idx] = jump_offset
            _bg_brightness_np[idx] = 0.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = -200.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

    # === ADD GIANT SQUID TENTACLES (1 group of 3) ===
    num_tentacles = 3
    num_segments = 24      # Segments per tentacle (base=0, tip=23)
    num_squid_splash = 28  # Shared splash particles for the group

    # Segment sizes: taper from thick base to thin tip (24 segments)
    seg_sizes = [
        2.2, 2.15, 2.1, 2.05, 2.0, 1.95, 1.9, 1.8,
        1.7, 1.6, 1.5, 1.4, 1.3, 1.25, 1.2, 1.15,
        1.1, 1.05, 1.0, 0.95, 0.9, 0.85, 0.8, 0.75,
    ]
    # Colors: deep reddish-purple base → pinkish tip
    base_col = [0.55, 0.15, 0.2]
    tip_col = [0.75, 0.35, 0.4]

    squid_cycle_offset = 1.0  # Offset so it doesn't sync with dolphins (TEST — normally 7)

    for tent_id in range(num_tentacles):
        if idx >= MAX_BACKGROUND_VOXELS - (num_segments + num_squid_splash + 10):
            break

        for seg in range(num_segments):
            _bg_pos_np[idx] = [0.0, 0.0, 0.0]
            _bg_anim_type_np[idx] = BG_ANIM_SQUID_TENTACLE
            _bg_anim_amp_np[idx] = float(seg)          # segment index
            _bg_phase_np[idx] = float(tent_id)          # tentacle ID (0,1,2)
            _bg_anim_speed_np[idx] = squid_cycle_offset
            _bg_brightness_np[idx] = 0.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = -200.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            _bg_size_np[idx] = seg_sizes[seg]

            # Gradient color: base → tip
            t = seg / 23.0
            _bg_col_np[idx] = [
                base_col[0] + (tip_col[0] - base_col[0]) * t,
                base_col[1] + (tip_col[1] - base_col[1]) * t,
                base_col[2] + (tip_col[2] - base_col[2]) * t,
            ]
            idx += 1

    # SQUID SPLASH PARTICLES
    for s in range(num_squid_splash):
        _bg_pos_np[idx] = [0.0, 0.0, 0.0]
        white_mix = random.uniform(0.3, 0.7)
        _bg_col_np[idx] = [0.5 + 0.5 * white_mix, 0.65 + 0.35 * white_mix, 0.85 + 0.15 * white_mix]
        _bg_size_np[idx] = random.uniform(1.0, 2.0)
        _bg_anim_type_np[idx] = BG_ANIM_SQUID_SPLASH
        _bg_phase_np[idx] = 0.0               # Not per-tentacle
        _bg_anim_amp_np[idx] = float(s)
        _bg_anim_speed_np[idx] = squid_cycle_offset
        _bg_brightness_np[idx] = 0.0
        _bg_offset_x_np[idx] = 0.0
        _bg_offset_y_np[idx] = -200.0
        _bg_offset_z_np[idx] = 0.0
        _bg_active_np[idx] = 1
        idx += 1

    wave_count = idx - start_idx
    theme_start_idx[THEME_WAVES] = start_idx
    theme_count[THEME_WAVES] = wave_count
    active_themes.add(THEME_WAVES)
    _bg_count = idx
    bg_flush()
    print(f"Added {wave_count} wave voxels + sea serpent + dolphins + squid (total: {idx})")

def add_tree_branches(count: int = 12, seed: int = 42):
    """Add palm trees to the background (appends to existing voxels)."""
    global active_themes, theme_start_idx, theme_count, _bg_count
    import random
    import math
    random.seed(seed)

    if THEME_PALM_TREES in active_themes:
        return

    start_idx = _bg_count
    idx = start_idx
    tree_radius = 90
    base_y = -15
    tree_id = 0

    for t in range(count):
        if idx >= MAX_BACKGROUND_VOXELS - 200:
            break

        angle = (t / count) * 2 * math.pi
        tree_x = math.cos(angle) * tree_radius
        tree_z = math.sin(angle) * tree_radius

        tree_phase = float(tree_id)
        tree_id += 1

        trunk_color = ti.Vector([0.45, 0.32, 0.18])
        frond_color = ti.Vector([0.25, 0.50, 0.20])

        # Trunk
        trunk_height = random.randint(38, 42)
        trunk_spacing = 1.5

        for h in range(trunk_height):
            y = base_y + h * trunk_spacing
            trunk_size = 1.5 - h * 0.015
            if trunk_size < 0.8:
                trunk_size = 0.8
            sway_amp = 0.02 + h * 0.003

            _bg_pos_np[idx] = [tree_x, y, tree_z]
            _bg_col_np[idx] = trunk_color
            _bg_size_np[idx] = trunk_size
            _bg_anim_type_np[idx] = BG_ANIM_TREE
            _bg_phase_np[idx] = tree_phase
            _bg_anim_amp_np[idx] = sway_amp
            _bg_anim_speed_np[idx] = random.uniform(0.9, 1.1)
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

        # Fronds
        tree_top_y = base_y + trunk_height * trunk_spacing
        num_fronds = 10

        for f in range(num_fronds):
            frond_angle = angle + (f / num_fronds) * 2 * math.pi
            frond_length = random.randint(8, 10)

            for seg in range(frond_length):
                reach = 1.5 + seg * 1.8
                droop = seg * seg * 0.12

                f_x = tree_x + math.cos(frond_angle) * reach
                f_z = tree_z + math.sin(frond_angle) * reach
                f_y = tree_top_y + 0.5 - droop

                frond_size = 0.9 - seg * 0.05
                if frond_size < 0.35:
                    frond_size = 0.35
                sway_amp = 0.6 + seg * 0.2

                _bg_pos_np[idx] = [f_x, f_y, f_z]
                _bg_col_np[idx] = frond_color
                _bg_size_np[idx] = frond_size
                _bg_anim_type_np[idx] = BG_ANIM_TREE
                _bg_phase_np[idx] = tree_phase + f * 0.15
                _bg_anim_amp_np[idx] = sway_amp
                _bg_anim_speed_np[idx] = random.uniform(0.8, 1.2)
                _bg_brightness_np[idx] = 1.0
                _bg_offset_x_np[idx] = 0.0
                _bg_offset_y_np[idx] = 0.0
                _bg_offset_z_np[idx] = 0.0
                _bg_active_np[idx] = 1
                idx += 1

    theme_start_idx[THEME_PALM_TREES] = start_idx
    theme_count[THEME_PALM_TREES] = idx - start_idx
    active_themes.add(THEME_PALM_TREES)
    _bg_count = idx
    bg_flush()
    print(f"Added {idx - start_idx} palm tree voxels (total: {idx})")

def add_swamp(seed: int = 42):
    """Add swamp biome: giant mushrooms, fog, bubbling water, spores, hanging moss."""
    global active_themes, theme_start_idx, theme_count, _bg_count
    import random
    import math
    random.seed(seed)

    if THEME_SWAMP in active_themes:
        return

    start_idx = _bg_count
    idx = start_idx
    water_y = 17.0

    # === MURKY WATER SURFACE (covers under board + outer ring) ===
    water_inner = 0.0
    water_outer = 85.0
    water_grid = 40
    water_spacing = (water_outer * 2) / water_grid

    for gx in range(water_grid):
        for gz in range(water_grid):
            if idx >= MAX_BACKGROUND_VOXELS - 200:
                break
            x = -water_outer + gx * water_spacing + random.uniform(-1.0, 1.0)
            z = -water_outer + gz * water_spacing + random.uniform(-1.0, 1.0)
            dist = math.sqrt(x * x + z * z)
            if dist > water_outer:
                continue

            _bg_pos_np[idx] = [x, water_y - 0.5 + random.uniform(-0.3, 0.3), z]
            g = random.uniform(0.12, 0.22)
            _bg_col_np[idx] = [g * 0.7, g, g * 0.4]
            _bg_size_np[idx] = random.uniform(2.0, 2.5)
            _bg_anim_type_np[idx] = BG_ANIM_NONE
            _bg_phase_np[idx] = 0.0
            _bg_anim_amp_np[idx] = 0.0
            _bg_anim_speed_np[idx] = 0.0
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

    water_count = idx - start_idx

    # === GIANT MUSHROOMS ===
    num_big_shrooms = 8
    shroom_radius = 80.0
    mushroom_data = []  # Store cap positions for spore spawning

    for s in range(num_big_shrooms):
        if idx >= MAX_BACKGROUND_VOXELS - 300:
            break

        angle = (s / num_big_shrooms) * 2 * math.pi + random.uniform(-0.15, 0.15)
        sx = math.cos(angle) * shroom_radius + random.uniform(-5, 5)
        sz = math.sin(angle) * shroom_radius + random.uniform(-5, 5)
        shroom_height = random.uniform(15, 22.5)
        cap_radius = random.uniform(6.0, 10.0)

        # Stem: thick, tapered, pale cream/mossy
        stem_segments = int(shroom_height / 1.2)
        for h in range(stem_segments):
            y = water_y + h * 1.2
            stem_size = 2.0 - h * 0.03
            if stem_size < 1.2:
                stem_size = 1.2

            _bg_pos_np[idx] = [sx, y, sz]
            tint = random.uniform(0.9, 1.0)
            _bg_col_np[idx] = [0.65 * tint, 0.60 * tint, 0.45 * tint]
            _bg_size_np[idx] = stem_size
            _bg_anim_type_np[idx] = BG_ANIM_NONE
            _bg_phase_np[idx] = 0.0
            _bg_anim_amp_np[idx] = 0.0
            _bg_anim_speed_np[idx] = 0.0
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

        # Cap: dome of overlapping voxels
        cap_y = water_y + shroom_height
        cap_type = random.choice(['purple', 'teal', 'orange'])
        if cap_type == 'purple':
            cap_base = (0.45, 0.15, 0.55)
        elif cap_type == 'teal':
            cap_base = (0.12, 0.50, 0.45)
        else:
            cap_base = (0.75, 0.35, 0.10)

        for c in range(30):
            theta = random.uniform(0, 2 * math.pi)
            phi = random.uniform(0, math.pi * 0.45)
            r = cap_radius * (0.6 + 0.4 * random.random())

            cx = sx + r * math.cos(theta) * math.sin(phi)
            cz = sz + r * math.sin(theta) * math.sin(phi)
            cy = cap_y + r * math.cos(phi) * 0.4  # Flatten dome

            tint = random.uniform(0.85, 1.0)
            _bg_pos_np[idx] = [cx, cy, cz]
            _bg_col_np[idx] = [cap_base[0]*tint, cap_base[1]*tint, cap_base[2]*tint]
            _bg_size_np[idx] = random.uniform(1.8, 3.0)
            _bg_anim_type_np[idx] = BG_ANIM_NONE
            _bg_phase_np[idx] = 0.0
            _bg_anim_amp_np[idx] = 0.0
            _bg_anim_speed_np[idx] = 0.0
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

        # Hanging moss from cap edges
        for m in range(6):
            chain_angle = (m / 6) * 2 * math.pi + random.uniform(-0.3, 0.3)
            chain_base_x = sx + cap_radius * 0.85 * math.cos(chain_angle)
            chain_base_z = sz + cap_radius * 0.85 * math.sin(chain_angle)
            chain_base_y = cap_y - 0.5
            chain_length = random.randint(3, 5)

            for link in range(chain_length):
                _bg_pos_np[idx] = [
                    chain_base_x,
                    chain_base_y - link * 1.8,
                    chain_base_z
                ]
                mt = random.uniform(0.8, 1.0)
                _bg_col_np[idx] = [0.35*mt, 0.50*mt, 0.30*mt]
                _bg_size_np[idx] = random.uniform(0.35, 0.55)
                _bg_anim_type_np[idx] = BG_ANIM_SWAY
                _bg_phase_np[idx] = random.uniform(0, 6.28)
                _bg_anim_amp_np[idx] = 0.5 + link * 0.3
                _bg_anim_speed_np[idx] = random.uniform(0.5, 0.9)
                _bg_brightness_np[idx] = 1.0
                _bg_offset_x_np[idx] = 0.0
                _bg_offset_y_np[idx] = 0.0
                _bg_offset_z_np[idx] = 0.0
                _bg_active_np[idx] = 1
                idx += 1

        # Store cap info for spores
        mushroom_data.append((sx, sz, cap_y, cap_radius))

    # === SMALL MUSHROOMS ===
    for s in range(12):
        if idx >= MAX_BACKGROUND_VOXELS - 50:
            break
        angle = random.uniform(0, 2 * math.pi)
        radius = random.uniform(10, 82)
        sx = math.cos(angle) * radius
        sz = math.sin(angle) * radius
        shroom_height = random.uniform(6.75, 10.8)
        cap_radius = random.uniform(2.5, 4.0)

        cap_type = random.choice(['purple', 'teal', 'orange'])
        if cap_type == 'purple':
            cap_base = (0.45, 0.15, 0.55)
        elif cap_type == 'teal':
            cap_base = (0.12, 0.50, 0.45)
        else:
            cap_base = (0.75, 0.35, 0.10)

        # Small stem
        stem_segs = int(shroom_height / 1.5)
        for h in range(stem_segs):
            y = water_y + h * 1.5
            tint = random.uniform(0.85, 1.0)
            _bg_pos_np[idx] = [sx, y, sz]
            _bg_col_np[idx] = [0.55*tint, 0.50*tint, 0.38*tint]
            _bg_size_np[idx] = random.uniform(1.0, 1.5)
            _bg_anim_type_np[idx] = BG_ANIM_NONE
            _bg_phase_np[idx] = 0.0
            _bg_anim_amp_np[idx] = 0.0
            _bg_anim_speed_np[idx] = 0.0
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

        # Small cap (5 voxels, all glow)
        cap_y = water_y + shroom_height
        for c in range(5):
            theta = random.uniform(0, 2 * math.pi)
            phi = random.uniform(0, math.pi * 0.4)
            r = cap_radius * random.uniform(0.5, 1.0)
            _bg_pos_np[idx] = [
                sx + r * math.cos(theta) * math.sin(phi),
                cap_y + r * math.cos(phi) * 0.4,
                sz + r * math.sin(theta) * math.sin(phi)
            ]
            tint = random.uniform(0.85, 1.0)
            _bg_col_np[idx] = [cap_base[0]*tint, cap_base[1]*tint, cap_base[2]*tint]
            _bg_size_np[idx] = random.uniform(1.5, 2.5)
            _bg_anim_type_np[idx] = BG_ANIM_GLOW_PULSE
            _bg_phase_np[idx] = random.uniform(0, 6.28)
            _bg_anim_amp_np[idx] = random.uniform(0.3, 0.6)
            _bg_anim_speed_np[idx] = random.uniform(0.4, 0.8)
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

    # === RISING BUBBLES ===
    for b in range(40):
        if idx >= MAX_BACKGROUND_VOXELS - 50:
            break
        b_angle = random.uniform(0, 2 * math.pi)
        b_radius = random.uniform(45, 90)
        bx = math.cos(b_angle) * b_radius
        bz = math.sin(b_angle) * b_radius

        for bp in range(5):
            _bg_pos_np[idx] = [
                bx + random.uniform(-2, 2),
                water_y,
                bz + random.uniform(-2, 2)
            ]
            _bg_col_np[idx] = [0.3, 0.55, 0.3]
            _bg_size_np[idx] = random.uniform(0.2, 0.5)
            _bg_anim_type_np[idx] = BG_ANIM_SWAMP_BUBBLE
            _bg_phase_np[idx] = random.uniform(0, 6.28)
            _bg_anim_amp_np[idx] = random.uniform(5.0, 12.0)
            _bg_anim_speed_np[idx] = random.uniform(1.5, 3.0)
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

    # === TAR PITS (dark patches + occasional burst of tiny dark voxels) ===
    for tc in range(24):
        if idx >= MAX_BACKGROUND_VOXELS - 60:
            break
        tar_angle = (tc / 24) * 2 * math.pi + random.uniform(-0.3, 0.3)
        tar_radius = random.uniform(15, 75)
        tar_cx = math.cos(tar_angle) * tar_radius
        tar_cz = math.sin(tar_angle) * tar_radius
        tar_cy = water_y + 0.3

        # Dark tar pool base (static)
        for tv in range(8):
            ox = random.gauss(0, 4.0)
            oz = random.gauss(0, 4.0)

            _bg_pos_np[idx] = [tar_cx + ox, tar_cy + random.uniform(-0.2, 0.2), tar_cz + oz]
            d = random.uniform(0.05, 0.12)
            _bg_col_np[idx] = [d, d * 0.8, d * 0.5]  # Very dark brown/black
            _bg_size_np[idx] = random.uniform(2.0, 3.0)
            _bg_anim_type_np[idx] = BG_ANIM_NONE
            _bg_phase_np[idx] = 0.0
            _bg_anim_amp_np[idx] = 0.0
            _bg_anim_speed_np[idx] = 0.0
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

        # Tar eruption particles (dark voxels that shake and burst out)
        pool_phase = random.uniform(0, 10.0)  # Shared timing per pool
        for tb in range(25):
            _bg_pos_np[idx] = [
                tar_cx + random.uniform(-2, 2),
                tar_cy,
                tar_cz + random.uniform(-2, 2)
            ]
            d = random.uniform(0.03, 0.10)
            _bg_col_np[idx] = [d, d * 0.7, d * 0.4]
            _bg_size_np[idx] = random.uniform(0.4, 1.0)
            _bg_anim_type_np[idx] = BG_ANIM_SWAMP_BUBBLE
            _bg_phase_np[idx] = pool_phase  # All voxels in same pool erupt together
            _bg_anim_amp_np[idx] = random.uniform(8.0, 20.0)  # Big eruptions
            _bg_anim_speed_np[idx] = random.uniform(0.0, 100.0)  # Unique seed per voxel
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

    # === GLOWING SPORES (from each big mushroom cap) ===
    for (mx, mz, mcap_y, mcap_r) in mushroom_data:
        for sp in range(35):
            if idx >= MAX_BACKGROUND_VOXELS - 5:
                break
            sp_theta = random.uniform(0, 2 * math.pi)
            sp_r = random.uniform(0, mcap_r * 0.8)
            sp_x = mx + sp_r * math.cos(sp_theta)
            sp_z = mz + sp_r * math.sin(sp_theta)
            sp_y = mcap_y + random.uniform(-1.0, 3.0)

            _bg_pos_np[idx] = [sp_x, sp_y, sp_z]
            sp_tint = random.uniform(0.7, 1.0)
            _bg_col_np[idx] = [0.6*sp_tint, 1.0*sp_tint, 0.3*sp_tint]
            _bg_size_np[idx] = random.uniform(0.15, 0.30)
            _bg_anim_type_np[idx] = BG_ANIM_SPORE_DRIFT
            _bg_phase_np[idx] = random.uniform(0, 6.28)
            _bg_anim_amp_np[idx] = random.uniform(15.0, 25.0)
            _bg_anim_speed_np[idx] = random.uniform(0.8, 1.5)
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

    # === FAT TOADS ===
    # 2 toads, 41 parts each (high-res legs)
    # Parts: 0=body, 1=head, 2=throat, 3-4=eyes, 5-6=pupils,
    #   7-14=left back leg (3 thigh + 3 shin + 2 toes)
    #   15-22=right back leg
    #   23-28=left front leg (2 upper + 2 forearm + 2 hand)
    #   29-34=right front leg
    #   35-40=warts
    for toad_id in range(2):
        toad_parts = [
            # (part_id, size, r, g, b)
            (0, 5.0, 0.35, 0.40, 0.15),         # Body: fat olive green
            (1, 3.5, 0.38, 0.43, 0.17),         # Head: slightly lighter
            (2, 2.5, 0.75, 0.70, 0.40),         # Throat pouch: pale yellow
            (3, 1.8, 0.80, 0.75, 0.10),         # Left eye: golden yellow
            (4, 1.8, 0.80, 0.75, 0.10),         # Right eye: golden yellow
            (5, 0.7, 0.05, 0.05, 0.05),         # Left pupil: black
            (6, 0.7, 0.05, 0.05, 0.05),         # Right pupil: black
            # Left back leg: thigh chain
            (7, 2.2, 0.32, 0.38, 0.13),
            (8, 2.0, 0.32, 0.38, 0.13),
            (9, 1.8, 0.31, 0.37, 0.12),
            # Left back leg: shin chain
            (10, 1.8, 0.30, 0.36, 0.12),
            (11, 1.6, 0.30, 0.36, 0.12),
            (12, 1.5, 0.29, 0.35, 0.11),
            # Left back leg: toes
            (13, 1.3, 0.28, 0.34, 0.11),
            (14, 1.3, 0.28, 0.34, 0.11),
            # Right back leg: thigh chain
            (15, 2.2, 0.32, 0.38, 0.13),
            (16, 2.0, 0.32, 0.38, 0.13),
            (17, 1.8, 0.31, 0.37, 0.12),
            # Right back leg: shin chain
            (18, 1.8, 0.30, 0.36, 0.12),
            (19, 1.6, 0.30, 0.36, 0.12),
            (20, 1.5, 0.29, 0.35, 0.11),
            # Right back leg: toes
            (21, 1.3, 0.28, 0.34, 0.11),
            (22, 1.3, 0.28, 0.34, 0.11),
            # Left front leg: upper arm
            (23, 1.8, 0.33, 0.39, 0.14),
            (24, 1.6, 0.33, 0.39, 0.14),
            # Left front leg: forearm
            (25, 1.5, 0.32, 0.38, 0.13),
            (26, 1.3, 0.32, 0.38, 0.13),
            # Left front leg: hand/toes
            (27, 1.2, 0.30, 0.35, 0.12),
            (28, 1.2, 0.30, 0.35, 0.12),
            # Right front leg: upper arm
            (29, 1.8, 0.33, 0.39, 0.14),
            (30, 1.6, 0.33, 0.39, 0.14),
            # Right front leg: forearm
            (31, 1.5, 0.32, 0.38, 0.13),
            (32, 1.3, 0.32, 0.38, 0.13),
            # Right front leg: hand/toes
            (33, 1.2, 0.30, 0.35, 0.12),
            (34, 1.2, 0.30, 0.35, 0.12),
        ]

        # Add wart bumps on body (extra small voxels)
        wart_parts = []
        for w in range(6):
            wart_parts.append((35 + w, random.uniform(0.6, 1.0),
                              0.28 + random.uniform(0, 0.05),
                              0.33 + random.uniform(0, 0.05),
                              0.10 + random.uniform(0, 0.03)))

        # Tongue chain (8 voxels, tapering to tip, pink/red)
        tongue_parts = []
        for t in range(8):
            t_size = 0.9 - t * 0.07  # Taper from 0.9 to ~0.4
            # Pink base → red tip
            t_r = 0.85 + t * 0.02
            t_g = 0.25 - t * 0.02
            t_b = 0.25 - t * 0.02
            tongue_parts.append((41 + t, t_size, t_r, t_g, t_b))

        for part_id, size, r, g, b in toad_parts + wart_parts + tongue_parts:
            if idx >= MAX_BACKGROUND_VOXELS - 5:
                break
            _bg_pos_np[idx] = [0, 0, 0]
            _bg_col_np[idx] = [r, g, b]
            _bg_size_np[idx] = size
            _bg_anim_type_np[idx] = BG_ANIM_TOAD
            _bg_phase_np[idx] = float(toad_id)
            _bg_anim_amp_np[idx] = float(part_id)
            _bg_anim_speed_np[idx] = 0.0
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

        # Mud splash particles (16 per toad)
        for sp in range(16):
            if idx >= MAX_BACKGROUND_VOXELS - 5:
                break
            _bg_pos_np[idx] = [0, 0, 0]
            # Mud colors: dark brown/green swamp muck
            mud_t = random.uniform(0.7, 1.0)
            _bg_col_np[idx] = [0.25 * mud_t, 0.20 * mud_t, 0.08 * mud_t]
            _bg_size_np[idx] = random.uniform(0.7, 1.4)
            _bg_anim_type_np[idx] = BG_ANIM_MUD_SPLASH
            _bg_phase_np[idx] = float(toad_id)
            _bg_anim_amp_np[idx] = float(sp)
            _bg_anim_speed_np[idx] = 0.0
            _bg_brightness_np[idx] = 0.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = -200.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

    theme_start_idx[THEME_SWAMP] = start_idx
    theme_count[THEME_SWAMP] = idx - start_idx
    active_themes.add(THEME_SWAMP)
    _bg_count = idx
    bg_flush()
    print(f"Added {idx - start_idx} swamp voxels (total: {idx})")


def add_desert(seed: int = 42):
    """Add desert biome: sand ground, pyramids, cacti, dust devils, scorpions."""
    global active_themes, theme_start_idx, theme_count, _bg_count
    import random
    import math
    random.seed(seed)

    if THEME_DESERT in active_themes:
        return

    start_idx = _bg_count
    idx = start_idx
    sand_y = 17.5
    circle_radius = 85.0

    # === SAND GROUND FILLER ===
    sand_grid = 40
    sand_spacing = (circle_radius * 2) / sand_grid
    for gx in range(sand_grid):
        for gz in range(sand_grid):
            if idx >= MAX_BACKGROUND_VOXELS - 200:
                break
            x = -circle_radius + gx * sand_spacing + random.uniform(-1.0, 1.0)
            z = -circle_radius + gz * sand_spacing + random.uniform(-1.0, 1.0)
            dist = math.sqrt(x * x + z * z)
            if dist > circle_radius:
                continue
            # Varied sand palette — mix of warm tan, pale gold, dusty amber
            tone = random.random()
            if tone < 0.4:
                # Warm tan (most common)
                r = random.uniform(0.58, 0.68)
                g = random.uniform(0.45, 0.53)
                b = random.uniform(0.26, 0.32)
            elif tone < 0.7:
                # Pale gold
                r = random.uniform(0.64, 0.74)
                g = random.uniform(0.52, 0.60)
                b = random.uniform(0.28, 0.35)
            elif tone < 0.9:
                # Dusty amber (darker patches)
                r = random.uniform(0.48, 0.58)
                g = random.uniform(0.36, 0.43)
                b = random.uniform(0.20, 0.26)
            else:
                # Occasional dark grit
                r = random.uniform(0.38, 0.48)
                g = random.uniform(0.28, 0.35)
                b = random.uniform(0.16, 0.22)
            _bg_pos_np[idx] = [x, sand_y - 0.5 + random.uniform(-0.2, 0.2), z]
            _bg_col_np[idx] = [r, g, b]
            _bg_size_np[idx] = random.uniform(1.8, 2.3)
            _bg_anim_type_np[idx] = BG_ANIM_NONE
            _bg_anim_speed_np[idx] = 0.0
            _bg_anim_amp_np[idx] = 0.0
            _bg_phase_np[idx] = 0.0
            _bg_brightness_np[idx] = 1.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = 0.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1

    # === PYRAMIDS (2, opposite sides) ===
    pyramid_angles = [0.4, 0.4 + math.pi]  # Opposite sides
    pyramid_radius = 75.0
    for pa in pyramid_angles:
        px_base = math.cos(pa) * pyramid_radius
        pz_base = math.sin(pa) * pyramid_radius
        layers = [15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1]
        layer_y = sand_y + 0.5
        voxel_spacing = 1.8
        for li, layer_size in enumerate(layers):
            t_layer = li / (len(layers) - 1.0)  # 0 to 1
            # Smooth sandstone gradient: warm tan base → pale golden top
            base_r = 0.52 + t_layer * 0.22
            base_g = 0.40 + t_layer * 0.20
            base_b = 0.24 + t_layer * 0.14
            vox_size = 0.9 - t_layer * 0.15

            half = (layer_size - 1) * voxel_spacing / 2.0
            for lx in range(layer_size):
                for lz in range(layer_size):
                    if idx >= MAX_BACKGROUND_VOXELS - 200:
                        break
                    x = px_base + lx * voxel_spacing - half
                    z = pz_base + lz * voxel_spacing - half

                    if li >= len(layers) - 5:
                        # Top 3 layers: golden glow
                        glow_t = (li - (len(layers) - 5)) / 4.0  # 0 to 1.0
                        gr = 0.75 + glow_t * 0.20
                        gg = 0.65 + glow_t * 0.20
                        gb = 0.20 + glow_t * 0.05
                        _bg_pos_np[idx] = [x, layer_y, z]
                        _bg_col_np[idx] = [gr, gg, gb]
                        _bg_size_np[idx] = vox_size
                        _bg_anim_type_np[idx] = BG_ANIM_GLOW_PULSE
                        _bg_anim_speed_np[idx] = 0.8
                        _bg_anim_amp_np[idx] = 0.5
                        _bg_phase_np[idx] = random.uniform(0, 6.28)
                        _bg_brightness_np[idx] = 3.0 + glow_t * 1.0
                    else:
                        r_v = base_r + random.uniform(-0.03, 0.03)
                        g_v = base_g + random.uniform(-0.03, 0.03)
                        b_v = base_b + random.uniform(-0.03, 0.03)
                        _bg_pos_np[idx] = [x, layer_y, z]
                        _bg_col_np[idx] = [r_v, g_v, b_v]
                        _bg_size_np[idx] = vox_size
                        _bg_anim_type_np[idx] = BG_ANIM_NONE
                        _bg_anim_speed_np[idx] = 0.0
                        _bg_anim_amp_np[idx] = 0.0
                        _bg_phase_np[idx] = 0.0

                    if li < len(layers) - 5:
                        _bg_brightness_np[idx] = 1.0
                    _bg_offset_x_np[idx] = 0.0
                    _bg_offset_y_np[idx] = 0.0
                    _bg_offset_z_np[idx] = 0.0
                    _bg_active_np[idx] = 1
                    idx += 1
            layer_y += 1.8

    # === CACTI ===
    def place_cactus_voxel(x, y, z, size, r, g, b, anim=BG_ANIM_NONE, spd=0.0, amp=0.0, ph=0.0):
        nonlocal idx
        if idx >= MAX_BACKGROUND_VOXELS - 100:
            return
        _bg_pos_np[idx] = [x, y, z]
        _bg_col_np[idx] = [r, g, b]
        _bg_size_np[idx] = size
        _bg_anim_type_np[idx] = anim
        _bg_anim_speed_np[idx] = spd
        _bg_anim_amp_np[idx] = amp
        _bg_phase_np[idx] = ph
        _bg_brightness_np[idx] = 1.0
        _bg_offset_x_np[idx] = 0.0
        _bg_offset_y_np[idx] = 0.0
        _bg_offset_z_np[idx] = 0.0
        _bg_active_np[idx] = 1
        idx += 1

    # --- TALL SAGUAROS (8) ---
    for ci in range(8):
        angle_c = (ci / 8.0) * math.pi * 2 + random.uniform(-0.25, 0.25)
        rad_c = random.uniform(45, 78)
        cx = math.cos(angle_c) * rad_c
        cz = math.sin(angle_c) * rad_c
        trunk_levels = random.randint(8, 12)
        vsp = 1.6  # vertical spacing
        vsz = 1.8  # big chunky voxels

        def cactus_green(t_h):
            gv = 0.30 + t_h * 0.10 + random.uniform(-0.02, 0.02)
            return gv * 0.50, gv, gv * 0.35

        # Thick trunk: cross pattern (center + 4 around) per level
        for h in range(trunk_levels):
            t_h = h / max(trunk_levels - 1, 1)
            rv, gv, bv = cactus_green(t_h)
            y = sand_y + 1.5 + h * vsp
            # Center
            place_cactus_voxel(cx, y, cz, vsz, rv, gv, bv)
            # Cross pattern for roundness
            off = vsz * 0.45
            place_cactus_voxel(cx + off, y, cz, vsz * 0.7, rv, gv, bv)
            place_cactus_voxel(cx - off, y, cz, vsz * 0.7, rv, gv, bv)
            place_cactus_voxel(cx, y, cz + off, vsz * 0.7, rv, gv, bv)
            place_cactus_voxel(cx, y, cz - off, vsz * 0.7, rv, gv, bv)

        # Arms (2 per saguaro, opposite sides)
        num_arms = 2
        for arm in range(num_arms):
            arm_h = random.randint(3, trunk_levels - 4)
            arm_y_base = sand_y + 1.5 + arm_h * vsp
            # Opposite sides
            arm_dir = (math.pi * 2 / num_arms) * arm + random.uniform(-0.4, 0.4) + angle_c
            arm_dx = math.cos(arm_dir)
            arm_dz = math.sin(arm_dir)

            rv, gv, bv = cactus_green(arm_h / trunk_levels)
            arm_vsz = vsz * 0.75

            # Horizontal elbow: 3 thick voxels reaching outward
            for e in range(3):
                dist_e = 1.8 + e * 1.4
                ex = cx + arm_dx * dist_e
                ez = cz + arm_dz * dist_e
                place_cactus_voxel(ex, arm_y_base, ez, arm_vsz, rv, gv, bv)
                # Thickness voxel above
                if e < 2:
                    place_cactus_voxel(ex, arm_y_base + arm_vsz * 0.4, ez,
                                       arm_vsz * 0.6, rv, gv, bv)

            # Vertical rise from elbow tip: 4-6 voxels
            elbow_x = cx + arm_dx * (1.8 + 2 * 1.4)
            elbow_z = cz + arm_dz * (1.8 + 2 * 1.4)
            arm_rise = random.randint(4, 6)
            for av in range(arm_rise):
                av_t = av / max(arm_rise - 1, 1)
                rv2, gv2, bv2 = cactus_green(0.5 + av_t * 0.3)
                place_cactus_voxel(elbow_x, arm_y_base + (av + 1) * vsp,
                                   elbow_z, arm_vsz * (1.0 - av_t * 0.15), rv2, gv2, bv2)
                # Thickness
                if av < arm_rise - 1:
                    place_cactus_voxel(elbow_x + arm_dx * arm_vsz * 0.3,
                                       arm_y_base + (av + 1) * vsp,
                                       elbow_z + arm_dz * arm_vsz * 0.3,
                                       arm_vsz * 0.5, rv2, gv2, bv2)

            # Flower on arm tip
            if random.random() < 0.6:
                fc = random.choice([(0.95, 0.25, 0.45), (1.0, 0.85, 0.15),
                                    (1.0, 0.40, 0.12), (0.95, 0.55, 0.70)])
                fy = arm_y_base + (arm_rise + 1) * vsp
                place_cactus_voxel(elbow_x, fy, elbow_z, 0.7, fc[0], fc[1], fc[2],
                                   BG_ANIM_GLOW_PULSE, 1.0, 0.6, random.uniform(0, 6.28))

        # Flower cluster on trunk top
        trunk_top_y = sand_y + 1.5 + trunk_levels * vsp + 0.5
        for fl in range(random.randint(1, 3)):
            fc = random.choice([(0.95, 0.25, 0.45), (1.0, 0.85, 0.15),
                                (1.0, 0.40, 0.12), (0.95, 0.55, 0.70)])
            fx = cx + random.uniform(-0.5, 0.5)
            fz = cz + random.uniform(-0.5, 0.5)
            place_cactus_voxel(fx, trunk_top_y + fl * 0.4, fz, 0.6, fc[0], fc[1], fc[2],
                               BG_ANIM_GLOW_PULSE, 1.0, 0.6, random.uniform(0, 6.28))

    # --- BARREL CACTI (8) ---
    for ci in range(8):
        angle_c = (ci / 8.0) * math.pi * 2 + 0.39 + random.uniform(-0.25, 0.25)
        rad_c = random.uniform(30, 78)
        cx = math.cos(angle_c) * rad_c
        cz = math.sin(angle_c) * rad_c

        gv_base = random.uniform(0.28, 0.38)
        barrel_h = random.randint(3, 5)
        bsz = 3.0  # barrel voxel size

        # Stacked rings for round barrel shape
        for h in range(barrel_h):
            t_h = h / max(barrel_h - 1, 1)
            # Wider in middle, narrow at top/bottom
            width = 1.0 - abs(t_h - 0.4) * 0.8
            gv = gv_base + t_h * 0.06 + random.uniform(-0.02, 0.02)
            rv = gv * 0.55
            bv = gv * 0.38
            y = sand_y + 1.5 + h * 2.8
            # Center
            place_cactus_voxel(cx, y, cz, bsz * width, rv, gv, bv)
            # Ring of 4 around
            ring_r = bsz * 0.5 * width
            for a in range(4):
                ang = a * (math.pi / 2) + h * 0.4  # rotate per layer
                bx = cx + math.cos(ang) * ring_r
                bz = cz + math.sin(ang) * ring_r
                place_cactus_voxel(bx, y, bz, bsz * 0.65 * width, rv, gv, bv)

        # Flower on top (most get one)
        if random.random() < 0.75:
            fc = random.choice([(0.95, 0.25, 0.45), (1.0, 0.85, 0.15),
                                (1.0, 0.40, 0.12), (0.95, 0.55, 0.70)])
            fy = sand_y + 1.5 + barrel_h * 2.8 + 0.5
            place_cactus_voxel(cx, fy, cz, 1.2, fc[0], fc[1], fc[2],
                               BG_ANIM_GLOW_PULSE, 1.0, 0.6, random.uniform(0, 6.28))

    # === DUST DEVILS (3 cyclones, 200 tiny particles each) ===
    cyclone_spots = [
        (math.cos(0.8) * 30, math.sin(0.8) * 30),
        (math.cos(2.8) * 35, math.sin(2.8) * 35),
        (math.cos(4.5) * 25, math.sin(4.5) * 25),
    ]
    for ci_dd, (cx_dd, cz_dd) in enumerate(cyclone_spots):
        for p in range(375):
            if idx >= MAX_BACKGROUND_VOXELS - 100:
                break
            # Tiny sand grains — varied sand tones
            tone = random.random()
            if tone < 0.5:
                base_col = random.uniform(0.78, 0.90)
                rc, gc, bc = base_col, base_col * 0.78, base_col * 0.42
            elif tone < 0.8:
                base_col = random.uniform(0.70, 0.82)
                rc, gc, bc = base_col, base_col * 0.72, base_col * 0.38
            else:
                base_col = random.uniform(0.55, 0.68)
                rc, gc, bc = base_col, base_col * 0.68, base_col * 0.35
            # Pre-compute chaos seeds (constant per particle, saves 3 trig/frame)
            chaos1_val = math.sin(p * 7.13 + ci_dd * 3.7)
            chaos2_val = math.cos(p * 11.3 + ci_dd * 5.1)
            _bg_pos_np[idx] = [cx_dd, chaos1_val, cz_dd]  # .y stores chaos1
            _bg_col_np[idx] = [rc, gc, bc]
            _bg_size_np[idx] = random.uniform(0.15, 0.45)
            _bg_anim_type_np[idx] = BG_ANIM_DUST_DEVIL
            _bg_anim_speed_np[idx] = 1.0
            _bg_anim_amp_np[idx] = float(p)
            _bg_phase_np[idx] = float(ci_dd)
            _bg_angle_np[idx] = chaos2_val  # stores chaos2
            _bg_brightness_np[idx] = 0.0
            _bg_offset_x_np[idx] = 0.0
            _bg_offset_y_np[idx] = -200.0
            _bg_offset_z_np[idx] = 0.0
            _bg_active_np[idx] = 1
            idx += 1



    theme_start_idx[THEME_DESERT] = start_idx
    theme_count[THEME_DESERT] = idx - start_idx
    active_themes.add(THEME_DESERT)
    _bg_count = idx
    bg_flush()
    print(f"Added {idx - start_idx} desert voxels (total: {idx})")


def add_stadium(seed: int = 42):
    """Add stadium with beetle larvae to the background (appends to existing voxels)."""
    global active_themes, theme_start_idx, theme_count, _bg_count
    import random
    import math
    random.seed(seed)

    if THEME_STADIUM in active_themes:
        return

    start_idx = _bg_count
    idx = start_idx
    base_y = 28
    inner_radius = 70
    row_spacing = 5
    row_height = 3.5
    num_rows = 5

    for row in range(num_rows):
        row_radius = inner_radius + row * row_spacing
        row_y = base_y + row * row_height

        circumference = 2 * math.pi * row_radius
        num_larvae = int(circumference / 40.0)

        for s in range(num_larvae):
            if idx >= MAX_BACKGROUND_VOXELS - 10:
                break

            angle = (s / num_larvae) * 2 * math.pi
            angle += random.uniform(-0.03, 0.03)

            base_x = math.cos(angle) * row_radius
            base_z = math.sin(angle) * row_radius

            larva_phase = random.uniform(0, 6.28)
            larva_speed = random.uniform(1.0, 1.5)
            body_tint = random.uniform(0.85, 1.0)

            for seg in range(4):
                if idx >= MAX_BACKGROUND_VOXELS:
                    break

                y = row_y + seg * 1.8

                if seg == 0:
                    seg_size = 0.9
                    color = ti.Vector([0.90 * body_tint, 0.85 * body_tint, 0.70 * body_tint])
                elif seg == 1:
                    seg_size = 1.3
                    color = ti.Vector([0.92 * body_tint, 0.88 * body_tint, 0.72 * body_tint])
                elif seg == 2:
                    seg_size = 1.15
                    color = ti.Vector([0.88 * body_tint, 0.82 * body_tint, 0.68 * body_tint])
                else:
                    seg_size = 1.0
                    color = ti.Vector([0.45, 0.30, 0.18])

                _bg_pos_np[idx] = [base_x, y, base_z]
                _bg_col_np[idx] = color
                _bg_size_np[idx] = seg_size
                _bg_anim_type_np[idx] = BG_ANIM_SCRUNCH
                _bg_phase_np[idx] = larva_phase
                _bg_anim_amp_np[idx] = float(seg)
                _bg_anim_speed_np[idx] = larva_speed
                _bg_angle_np[idx] = angle
                _bg_brightness_np[idx] = 1.0
                _bg_offset_x_np[idx] = 0.0
                _bg_offset_y_np[idx] = 0.0
                _bg_offset_z_np[idx] = 0.0
                _bg_active_np[idx] = 1
                idx += 1

    theme_start_idx[THEME_STADIUM] = start_idx
    theme_count[THEME_STADIUM] = idx - start_idx
    active_themes.add(THEME_STADIUM)
    _bg_count = idx
    bg_flush()
    stadium_excitement[None] = 0.0
    stadium_excitement_target[None] = 0.0
    print(f"Added {idx - start_idx} stadium larva voxels (total: {idx})")

def clear_all_themes():
    """Clear all active themes and reset background."""
    global active_themes, theme_start_idx, theme_count, _bg_count
    active_themes.clear()
    theme_start_idx.clear()
    theme_count.clear()
    _bg_count = 0
    bg_theme_active[None] = 0
    for i in range(MAX_BACKGROUND_VOXELS):
        _bg_active_np[i] = 0
        _bg_size_np[i] = 0.0
    bg_flush()
    print("Cleared all background themes")

@ti.kernel
def render_bowl_perimeter():
    """
    Render slippery bowl perimeter around arena (for ball mode)
    Creates shallow upward slope extending 8 voxels from arena edge
    Skips goal pit areas so ball can fall through
    """
    center_x = 64
    center_z = 64
    arena_radius = 32
    bowl_width = 12
    floor_y_offset = 33
    bowl_slope = 0.15  # Height increase per voxel outward (0.15 = rises 1 voxel every ~7 voxels)

    # Goal pit parameters - wide enough for ball (max radius 10 = diameter 20)
    goal_pit_half_width = 12  # Half width of pit opening (24 total, fits ball easily)

    # Iterate through the bowl ring area
    for i in range(center_x - arena_radius - bowl_width - 1, center_x + arena_radius + bowl_width + 2):
        for k in range(center_z - arena_radius - bowl_width - 1, center_z + arena_radius + bowl_width + 2):
            dx = float(i - center_x)
            dz = float(k - center_z)
            dist = ti.sqrt(dx * dx + dz * dz)

            # Only place voxels in the bowl ring (outside arena, within bowl width)
            if dist > arena_radius and dist <= arena_radius + bowl_width:
                # Skip goal pit areas (blue goal at x<32, red goal at x>96, both at z~64)
                in_goal_pit = False
                if abs(k - center_z) < goal_pit_half_width:
                    if i <= 32 or i >= 96:  # Goal pit zones
                        in_goal_pit = True

                if not in_goal_pit:
                    # Calculate height based on distance from arena edge
                    dist_from_edge = dist - arena_radius
                    bowl_height = int(dist_from_edge * bowl_slope)
                    bowl_y = floor_y_offset + bowl_height

                    # Place slippery voxel
                    if 0 <= i < n_grid and 0 <= bowl_y < n_grid and 0 <= k < n_grid:
                        voxel_type[i, bowl_y, k] = SLIPPERY

@ti.kernel
def clear_bowl_perimeter():
    """
    Clear the bowl perimeter voxels (when disabling ball mode)
    """
    center_x = 64
    center_z = 64
    arena_radius = 32
    bowl_width = 12
    floor_y_offset = 33
    bowl_slope = 0.15
    max_bowl_height = int(bowl_width * bowl_slope) + 2

    # Clear the bowl ring area
    for i in range(center_x - arena_radius - bowl_width - 1, center_x + arena_radius + bowl_width + 2):
        for k in range(center_z - arena_radius - bowl_width - 1, center_z + arena_radius + bowl_width + 2):
            dx = float(i - center_x)
            dz = float(k - center_z)
            dist = ti.sqrt(dx * dx + dz * dz)

            # Only clear voxels in the bowl ring area
            if dist > arena_radius and dist <= arena_radius + bowl_width + 1:
                for j in range(floor_y_offset, floor_y_offset + max_bowl_height + 1):
                    if 0 <= i < n_grid and 0 <= j < n_grid and 0 <= k < n_grid:
                        if voxel_type[i, j, k] == SLIPPERY:
                            voxel_type[i, j, k] = EMPTY

@ti.kernel
def init_mega_fortress():
    """
    RHINO BEETLE - Anatomically Accurate Voxel Sculpture
    Based on Dynastinae family characteristics:
    - Dual horn system (head + thorax)
    - Six segmented legs with claws
    - Thick exoskeleton with wing covers
    - Segmented body (head, thorax, abdomen)
    """
    # Clear everything first
    for i, j, k in ti.ndrange(n_grid, n_grid, n_grid):
        voxel_type[i, j, k] = EMPTY

    # Beetle positioned in center, facing +X direction
    center_x = 64
    center_y = 20  # Elevated off ground
    center_z = 64

    # ============================================================
    # SECTION 1: ABDOMEN (Rear body - segmented oval)
    # ============================================================
    # Abdomen: 7 segments, goes BACKWARD (negative X), tapers toward rear
    # Beetle faces +X, so abdomen is behind thorax at center_x
    abdomen_segments = 7

    for segment in range(abdomen_segments):
        # Start at center and go backward (segment 0 is at center, segment 6 is at rear)
        seg_x = center_x - segment * 4
        # Each segment is smaller as we go back (tapers toward rear)
        width_y = 10 - segment  # Height radius
        width_z = 12 - segment  # Side radius

        # Safety check: skip if width becomes invalid
        if width_y > 0 and width_z > 0:
            # Create ellipsoid segment
            for dx in range(-2, 3):
                for dy in range(-width_y, width_y + 1):
                    for dz in range(-width_z, width_z + 1):
                        # Ellipsoid equation
                        dist_sq = (float(dy) / float(width_y)) * (float(dy) / float(width_y)) + \
                                  (float(dz) / float(width_z)) * (float(dz) / float(width_z))

                        if dist_sq <= 1.0:
                            x = seg_x + dx
                            y = center_y + dy
                            z = center_z + dz
                            if 0 <= x < n_grid and 0 <= y < n_grid and 0 <= z < n_grid:
                                voxel_type[x, y, z] = CONCRETE

    # ============================================================
    # SECTION 2: THORAX (Middle body - largest segment with legs and wing covers)
    # ============================================================
    thorax_x = center_x + 2  # Slightly FORWARD of center (abdomen is behind)
    thorax_width_y = 12
    thorax_width_z = 14

    # Main thorax body (large rounded segment)
    for dx in range(-8, 9):
        for dy in range(-thorax_width_y, thorax_width_y + 1):
            for dz in range(-thorax_width_z, thorax_width_z + 1):
                # Ellipsoid equation
                dist_sq = (float(dx) / 8.0) * (float(dx) / 8.0) + \
                          (float(dy) / float(thorax_width_y)) * (float(dy) / float(thorax_width_y)) + \
                          (float(dz) / float(thorax_width_z)) * (float(dz) / float(thorax_width_z))

                if dist_sq <= 1.0:
                    x = thorax_x + dx
                    y = center_y + dy
                    z = center_z + dz
                    if 0 <= x < n_grid and 0 <= y < n_grid and 0 <= z < n_grid:
                        voxel_type[x, y, z] = STEEL

    # WING COVERS (Elytra) - Hard shell covering wings on back
    # Two symmetric covers on top of thorax and abdomen
    for side in range(2):  # Left and right
        z_offset = 8 if side == 0 else -8

        # Wing cover extends from rear abdomen to thorax
        for wing_x in range(-24, 8):  # From rear of abdomen to front of thorax
            for wing_y in range(0, 8):
                # Curved shell shape
                width = 4 - wing_y // 2
                for dz in range(-width, width + 1):
                    x = center_x + wing_x
                    y = center_y + wing_y + 6
                    z = center_z + z_offset + dz
                    if 0 <= x < n_grid and 0 <= y < n_grid and 0 <= z < n_grid:
                        voxel_type[x, y, z] = STEEL

    # ============================================================
    # SECTION 3: HEAD (Front segment with eyes, mandibles, head horn)
    # ============================================================
    head_x = center_x + 10  # Forward of thorax
    head_width_y = 8
    head_width_z = 10

    # Main head capsule
    for dx in range(-6, 7):
        for dy in range(-head_width_y, head_width_y + 1):
            for dz in range(-head_width_z, head_width_z + 1):
                dist_sq = (float(dx) / 6.0) * (float(dx) / 6.0) + \
                          (float(dy) / float(head_width_y)) * (float(dy) / float(head_width_y)) + \
                          (float(dz) / float(head_width_z)) * (float(dz) / float(head_width_z))

                if dist_sq <= 1.0:
                    x = head_x + dx
                    y = center_y + dy
                    z = center_z + dz
                    if 0 <= x < n_grid and 0 <= y < n_grid and 0 <= z < n_grid:
                        voxel_type[x, y, z] = STEEL

    # COMPOUND EYES (two bulbous eyes on sides of head)
    for side in range(2):  # Left and right
        eye_z = 9 if side == 0 else -9

        for dx in range(-2, 3):
            for dy in range(-3, 4):
                for dz in range(-3, 4):
                    dist_sq = (float(dx) / 2.0) * (float(dx) / 2.0) + \
                              (float(dy) / 3.0) * (float(dy) / 3.0) + \
                              (float(dz) / 3.0) * (float(dz) / 3.0)

                    if dist_sq <= 1.0:
                        x = head_x + 2 + dx
                        y = center_y + 2 + dy
                        z = center_z + eye_z + dz
                        if 0 <= x < n_grid and 0 <= y < n_grid and 0 <= z < n_grid:
                            voxel_type[x, y, z] = CONCRETE  # Different color for eyes

    # MANDIBLES (pincers on front of head)
    for side in range(2):  # Left and right
        mandible_z = 5 if side == 0 else -5

        for mandible_seg in range(8):
            x = head_x + 8 + mandible_seg
            y = center_y - 4 - mandible_seg // 2
            z = center_z + mandible_z + mandible_seg // 2 * (1 if side == 0 else -1)

            for dx in range(-1, 2):
                for dy in range(-1, 2):
                    for dz in range(-1, 2):
                        if 0 <= x + dx < n_grid and 0 <= y + dy < n_grid and 0 <= z + dz < n_grid:
                            voxel_type[x + dx, y + dy, z + dz] = STEEL

    # ============================================================
    # SECTION 4: DUAL HORN SYSTEM (Signature feature!)
    # ============================================================

    # HEAD HORN - Large curved horn pointing upward and forward
    # Classic rhino beetle Y-shaped horn
    horn_base_x = head_x
    horn_base_y = center_y + 5

    for horn_seg in range(20):
        # Parabolic curve upward and forward
        progress = float(horn_seg) / 20.0
        curve_x = int(progress * 15.0)
        curve_y = int(progress * progress * 25.0)  # Quadratic curve
        thickness = 3 - horn_seg // 7  # Tapers toward tip

        x = horn_base_x + curve_x
        y = horn_base_y + curve_y
        z = center_z

        for dx in range(-thickness, thickness + 1):
            for dy in range(-thickness, thickness + 1):
                for dz in range(-thickness, thickness + 1):
                    if 0 <= x + dx < n_grid and 0 <= y + dy < n_grid and 0 <= z + dz < n_grid:
                        voxel_type[x + dx, y + dy, z + dz] = STEEL

    # THORAX HORN - Forward-pointing horn from top of thorax
    thorax_horn_x = thorax_x + 3
    thorax_horn_y = center_y + 10

    for horn_seg in range(12):
        progress = float(horn_seg) / 12.0
        curve_x = int(progress * 12.0)
        curve_y = int(progress * 3.0)  # Slight upward angle
        thickness = 2 - horn_seg // 6

        x = thorax_horn_x + curve_x
        y = thorax_horn_y + curve_y
        z = center_z

        for dx in range(-thickness, thickness + 1):
            for dy in range(-thickness, thickness + 1):
                for dz in range(-thickness, thickness + 1):
                    if 0 <= x + dx < n_grid and 0 <= y + dy < n_grid and 0 <= z + dz < n_grid:
                        voxel_type[x + dx, y + dy, z + dz] = STEEL

    # ============================================================
    # SECTION 5: SIX LEGS (Jointed insect legs with claws)
    # ============================================================
    # All beetles have 6 legs: 2 front, 2 middle, 2 rear
    # Must manually unroll (Taichi can't iterate Python lists)

    # Helper function to build one leg
    # Manually build all 6 legs with different offsets

    # LEG 1: Front Left
    leg_x_off = -4
    leg_z_off = 14  # Attach at side of thorax body
    base_x = thorax_x + leg_x_off
    base_y = center_y  # Attach at middle of thorax height
    base_z = center_z + leg_z_off

    # Coxa
    for seg in range(3):
        x = base_x
        y = base_y - seg
        z = base_z + seg
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                for dz in range(-1, 2):
                    if 0 <= x + dx < n_grid and 0 <= y + dy < n_grid and 0 <= z + dz < n_grid:
                        voxel_type[x + dx, y + dy, z + dz] = STEEL

    # Femur + Tibia + Claw for leg 1
    for seg in range(8):
        progress = float(seg) / 8.0
        x = int(float(base_x) + progress * 5.0)
        y = int(float(base_y - 3) - progress * 8.0)
        z = int(float(base_z + 3) + progress * 5.0)
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                for dz in range(-1, 2):
                    if 0 <= x + dx < n_grid and 0 <= y + dy < n_grid and 0 <= z + dz < n_grid:
                        voxel_type[x + dx, y + dy, z + dz] = STEEL

    for seg in range(10):
        progress = float(seg) / 10.0
        x = int(float(base_x + 5) + progress * 3.0)
        y = int(float(base_y - 11) - progress * 10.0)
        z = base_z + 8
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                if 0 <= x + dx < n_grid and 0 <= y + dy < n_grid and 0 <= z < n_grid:
                    voxel_type[x + dx, y + dy, z] = STEEL

    # LEG 2: Front Right (mirror of leg 1)
    leg_x_off = -4
    leg_z_off = -14
    base_x = thorax_x + leg_x_off
    base_z = center_z + leg_z_off

    for seg in range(3):
        x = base_x
        y = base_y - seg
        z = base_z - seg
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                for dz in range(-1, 2):
                    if 0 <= x + dx < n_grid and 0 <= y + dy < n_grid and 0 <= z + dz < n_grid:
                        voxel_type[x + dx, y + dy, z + dz] = STEEL

    for seg in range(8):
        progress = float(seg) / 8.0
        x = int(float(base_x) + progress * 5.0)
        y = int(float(base_y - 3) - progress * 8.0)
        z = int(float(base_z - 3) - progress * 5.0)
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                for dz in range(-1, 2):
                    if 0 <= x + dx < n_grid and 0 <= y + dy < n_grid and 0 <= z + dz < n_grid:
                        voxel_type[x + dx, y + dy, z + dz] = STEEL

    for seg in range(10):
        progress = float(seg) / 10.0
        x = int(float(base_x + 5) + progress * 3.0)
        y = int(float(base_y - 11) - progress * 10.0)
        z = base_z - 8
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                if 0 <= x + dx < n_grid and 0 <= y + dy < n_grid and 0 <= z < n_grid:
                    voxel_type[x + dx, y + dy, z] = STEEL

    # LEGS 3-6: Middle and Rear (simplified for brevity - same pattern)
    # Middle Left
    leg_x_off = 0
    leg_z_off = 14
    base_x = thorax_x + leg_x_off
    base_z = center_z + leg_z_off
    for seg in range(20):
        progress = float(seg) / 20.0
        x = int(float(base_x) + progress * 6.0)
        y = int(float(center_y) - progress * 18.0)
        z = base_z
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                if 0 <= x + dx < n_grid and 0 <= y + dy < n_grid and 0 <= z < n_grid:
                    voxel_type[x + dx, y + dy, z] = STEEL

    # Middle Right
    leg_z_off = -14
    base_z = center_z + leg_z_off
    for seg in range(20):
        progress = float(seg) / 20.0
        x = int(float(base_x) + progress * 6.0)
        y = int(float(center_y) - progress * 18.0)
        z = base_z
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                if 0 <= x + dx < n_grid and 0 <= y + dy < n_grid and 0 <= z < n_grid:
                    voxel_type[x + dx, y + dy, z] = STEEL

    # Rear Left
    leg_x_off = 4
    leg_z_off = 14
    base_x = thorax_x + leg_x_off
    base_z = center_z + leg_z_off
    for seg in range(20):
        progress = float(seg) / 20.0
        x = int(float(base_x) + progress * 6.0)
        y = int(float(center_y) - progress * 18.0)
        z = base_z
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                if 0 <= x + dx < n_grid and 0 <= y + dy < n_grid and 0 <= z < n_grid:
                    voxel_type[x + dx, y + dy, z] = STEEL

    # Rear Right
    leg_z_off = -14
    base_z = center_z + leg_z_off
    for seg in range(20):
        progress = float(seg) / 20.0
        x = int(float(base_x) + progress * 6.0)
        y = int(float(center_y) - progress * 18.0)
        z = base_z
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                if 0 <= x + dx < n_grid and 0 <= y + dy < n_grid and 0 <= z < n_grid:
                    voxel_type[x + dx, y + dy, z] = STEEL

    print("RHINO BEETLE VOXEL SCULPTURE constructed!")
    print("- Anatomically accurate Dynastinae family beetle")
    print("- 7-segment tapered abdomen")
    print("- Thorax with dual wing covers (elytra)")
    print("- Head with compound eyes and mandibles")
    print("- DUAL HORN SYSTEM: Curved head horn + forward thorax horn")
    print("- Six fully articulated legs with claws")
    print("- Ready for destruction physics!")

@ti.kernel
def init_colosseum():
    """Create the Roman Colosseum"""
    # Clear everything first
    for i, j, k in ti.ndrange(n_grid, n_grid, n_grid):
        voxel_type[i, j, k] = EMPTY

    # Colosseum parameters (centered in grid)
    center_x = 64
    center_z = 64

    # Elliptical dimensions (scaled to fit grid)
    outer_radius_x = 40  # Semi-major axis
    outer_radius_z = 32  # Semi-minor axis
    inner_radius_x = 30
    inner_radius_z = 22

    # Height parameters
    num_tiers = 4
    tier_height = 10
    total_height = num_tiers * tier_height

    # Build each tier
    for tier in range(num_tiers):
        base_y = tier * tier_height

        # Each tier is slightly smaller (tapers inward)
        tier_scale = 1.0 - tier * 0.05
        tier_outer_x = int(outer_radius_x * tier_scale)
        tier_outer_z = int(outer_radius_z * tier_scale)
        tier_inner_x = int(inner_radius_x * tier_scale)
        tier_inner_z = int(inner_radius_z * tier_scale)

        # Build the elliptical ring for this tier
        for i in range(center_x - tier_outer_x - 2, center_x + tier_outer_x + 2):
            for k in range(center_z - tier_outer_z - 2, center_z + tier_outer_z + 2):
                # Calculate distance from center using ellipse formula
                dx = float(i - center_x)
                dz = float(k - center_z)

                # Outer ellipse
                dist_outer = (dx * dx) / (tier_outer_x * tier_outer_x) + (dz * dz) / (tier_outer_z * tier_outer_z)
                # Inner ellipse
                dist_inner = (dx * dx) / (tier_inner_x * tier_inner_x) + (dz * dz) / (tier_inner_z * tier_inner_z)

                # Check if point is between inner and outer ellipse
                if dist_outer <= 1.0 and dist_inner >= 1.0:
                    # Calculate angle for arch pattern
                    angle = ti.atan2(dz, dx)
                    arch_index = int((angle + 3.14159) / (6.28318 / 48.0))  # 48 arches around

                    # Build walls with arch openings
                    for y in range(base_y, base_y + tier_height):
                        # Arches: every other section, and only on lower part
                        is_arch_section = (arch_index % 2 == 0)
                        is_lower = (y - base_y) < tier_height // 2

                        # Check if close to outer edge (for columns)
                        is_outer_edge = dist_outer > 0.85

                        if is_outer_edge:
                            # Always build outer wall
                            voxel_type[i, y, k] = CONCRETE
                        elif not (is_arch_section and is_lower):
                            # Fill in wall between columns (except arch openings)
                            voxel_type[i, y, k] = CONCRETE

    # Arena floor (sand/dirt - represented as concrete for now)
    for i in range(center_x - inner_radius_x, center_x + inner_radius_x):
        for k in range(center_z - inner_radius_z, center_z + inner_radius_z):
            dx = float(i - center_x)
            dz = float(k - center_z)
            dist = (dx * dx) / (inner_radius_x * inner_radius_x) + (dz * dz) / (inner_radius_z * inner_radius_z)
            if dist <= 1.0:
                voxel_type[i, 0, k] = CONCRETE

    # Add some structural damage (missing sections) for realism
    damage_x = center_x + 30
    damage_z = center_z + 20
    for i in range(damage_x - 8, damage_x + 8):
        for k in range(damage_z - 8, damage_z + 8):
            for y in range(0, total_height):
                voxel_type[i, y, k] = EMPTY

@ti.kernel
def init_golden_gate():
    """Create the Golden Gate Bridge"""
    # Clear everything first
    for i, j, k in ti.ndrange(n_grid, n_grid, n_grid):
        voxel_type[i, j, k] = EMPTY

    # Bridge parameters
    tower1_x = 30
    tower2_x = 98
    bridge_z = 64
    deck_y = 10
    tower_height = 50

    # Deck dimensions
    deck_width = 8
    deck_thickness = 2

    # Tower has two legs with cross-bracing
    leg_width = 4
    leg_separation = 10

    # Build Tower 1
    # Left leg
    for x in range(tower1_x - leg_separation // 2 - leg_width, tower1_x - leg_separation // 2):
        for z in range(bridge_z - leg_width // 2, bridge_z + leg_width // 2):
            for y in range(0, tower_height):
                if y > tower_height - 10:
                    if x == tower1_x - leg_separation // 2 - leg_width or x == tower1_x - leg_separation // 2 - 1:
                        voxel_type[x, y, z] = STEEL
                else:
                    voxel_type[x, y, z] = STEEL

    # Right leg
    for x in range(tower1_x + leg_separation // 2, tower1_x + leg_separation // 2 + leg_width):
        for z in range(bridge_z - leg_width // 2, bridge_z + leg_width // 2):
            for y in range(0, tower_height):
                if y > tower_height - 10:
                    if x == tower1_x + leg_separation // 2 or x == tower1_x + leg_separation // 2 + leg_width - 1:
                        voxel_type[x, y, z] = STEEL
                else:
                    voxel_type[x, y, z] = STEEL

    # Cross-bracing (manually unroll)
    for x in range(tower1_x - leg_separation // 2, tower1_x + leg_separation // 2 + leg_width):
        for z in range(bridge_z - leg_width // 2, bridge_z + leg_width // 2):
            voxel_type[x, 15, z] = STEEL
            voxel_type[x, 30, z] = STEEL
            voxel_type[x, 45, z] = STEEL

    # Build Tower 2
    # Left leg
    for x in range(tower2_x - leg_separation // 2 - leg_width, tower2_x - leg_separation // 2):
        for z in range(bridge_z - leg_width // 2, bridge_z + leg_width // 2):
            for y in range(0, tower_height):
                if y > tower_height - 10:
                    if x == tower2_x - leg_separation // 2 - leg_width or x == tower2_x - leg_separation // 2 - 1:
                        voxel_type[x, y, z] = STEEL
                else:
                    voxel_type[x, y, z] = STEEL

    # Right leg
    for x in range(tower2_x + leg_separation // 2, tower2_x + leg_separation // 2 + leg_width):
        for z in range(bridge_z - leg_width // 2, bridge_z + leg_width // 2):
            for y in range(0, tower_height):
                if y > tower_height - 10:
                    if x == tower2_x + leg_separation // 2 or x == tower2_x + leg_separation // 2 + leg_width - 1:
                        voxel_type[x, y, z] = STEEL
                else:
                    voxel_type[x, y, z] = STEEL

    # Cross-bracing (manually unroll)
    for x in range(tower2_x - leg_separation // 2, tower2_x + leg_separation // 2 + leg_width):
        for z in range(bridge_z - leg_width // 2, bridge_z + leg_width // 2):
            voxel_type[x, 15, z] = STEEL
            voxel_type[x, 30, z] = STEEL
            voxel_type[x, 45, z] = STEEL

    # Build roadway deck spanning between towers
    for x in range(tower1_x - 10, tower2_x + 10):
        for z in range(bridge_z - deck_width // 2, bridge_z + deck_width // 2):
            for y in range(deck_y, deck_y + deck_thickness):
                voxel_type[x, y, z] = CONCRETE

    # Main suspension cables (catenary curve)
    # Cable attaches at top of each tower and sags to deck mid-span
    cable_attach_y = tower_height - 5
    mid_span_x = (tower1_x + tower2_x) // 2
    cable_sag = 15  # How much cable drops below attachment point

    # Left main cable
    for x in range(tower1_x, tower2_x + 1):
        # Catenary curve approximation using parabola
        dx_from_tower1 = float(x - tower1_x)
        dx_from_tower2 = float(tower2_x - x)
        span = float(tower2_x - tower1_x)

        # Parabolic sag
        t = dx_from_tower1 / span  # 0 to 1 across span
        cable_y = int(cable_attach_y - cable_sag * 4.0 * t * (1.0 - t))

        # Place cable voxels (left side)
        for z in range(bridge_z - deck_width // 2 - 2, bridge_z - deck_width // 2):
            voxel_type[x, cable_y, z] = STEEL
            voxel_type[x, cable_y + 1, z] = STEEL  # Thicker cable

        # Place cable voxels (right side)
        for z in range(bridge_z + deck_width // 2, bridge_z + deck_width // 2 + 2):
            voxel_type[x, cable_y, z] = STEEL
            voxel_type[x, cable_y + 1, z] = STEEL

    # Vertical suspender cables (every 5 voxels) - manually unroll since Taichi doesn't support range step
    for x in range(tower1_x, tower2_x):
        # Only place suspender every 5 voxels
        if (x - tower1_x) % 5 == 0 and x > tower1_x:
            # Calculate cable height at this x position
            dx_from_tower1 = float(x - tower1_x)
            span = float(tower2_x - tower1_x)
            t = dx_from_tower1 / span
            cable_y = int(cable_attach_y - cable_sag * 4.0 * t * (1.0 - t))

            # Left suspender
            for y in range(deck_y + deck_thickness, cable_y):
                for z in range(bridge_z - deck_width // 2 - 2, bridge_z - deck_width // 2):
                    voxel_type[x, y, z] = STEEL

            # Right suspender
            for y in range(deck_y + deck_thickness, cable_y):
                for z in range(bridge_z + deck_width // 2, bridge_z + deck_width // 2 + 2):
                    voxel_type[x, y, z] = STEEL

# OPTIMIZATION: Batch all silk counts into single array for one GPU->CPU transfer
@ti.kernel
def batch_silk_counts():
    """Copy all silk count scalars into single array for efficient GPU->CPU transfer.

    Reduces 6 separate GPU reads to 1, saving ~1-1.5ms on dedicated GPUs.
    Indices: 0=on_blue, 1=under_blue, 2=on_red, 3=under_red, 4=on_ball, 5=under_ball
    """
    silk_counts_batched[0] = silk_on_blue[None]
    silk_counts_batched[1] = silk_under_blue[None]
    silk_counts_batched[2] = silk_on_red[None]
    silk_counts_batched[3] = silk_under_red[None]
    silk_counts_batched[4] = silk_on_ball[None]
    silk_counts_batched[5] = silk_under_ball[None]

# Initialize voxel sphere on module load
print(f"Initializing voxel grid: {n_grid}x{n_grid}x{n_grid}")
init_beetle_arena()  # Changed to beetle arena
print("Beetle Battle Arena ready!")
