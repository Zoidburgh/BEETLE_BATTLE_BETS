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
    # Clear everything first
    for i, j, k in ti.ndrange(n_grid, n_grid, n_grid):
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
    # Clear everything first
    for i, j, k in ti.ndrange(n_grid, n_grid, n_grid):
        voxel_type[i, j, k] = EMPTY

    # Arena center (updated for 128 grid)
    center_x = 64
    center_z = 64

    # Arena dimensions
    arena_radius = 32
    inner_radius = 11  # Hole in the middle (1 voxel wider)
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
    # Clear everything first
    for i, j, k in ti.ndrange(n_grid, n_grid, n_grid):
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
