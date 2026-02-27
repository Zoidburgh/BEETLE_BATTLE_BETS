import taichi as ti
import numpy as np
import simulation

# Maximum number of voxels to render
# Reduced from 200000 to minimize CPU->GPU transfer overhead
# Typical usage: arena ~4000 + 2 beetles ~2400 + particles ~1000 = ~7500
MAX_VOXELS = 20000

# Particle render caps - prevents high water mark from tanking FPS
# These cap how many particle slots we CHECK (not just render) to avoid
# iterating over thousands of inactive slots after spray attacks
MAX_DEBRIS_CHECK = 5000   # Cap debris iteration (actual active usually < 500)
MAX_SPRAY_CHECK = 500     # Cap spray iteration (actual active usually < 100)
MAX_SILK_CHECK = 600      # Cap silk iteration (matches MAX_SILK)

# Voxel type constants (must match simulation.py)
MOLTEN = 3
DEBRIS = 4

# Taichi fields for rendering
num_voxels = ti.field(dtype=ti.i32, shape=())
voxel_positions = ti.Vector.field(3, dtype=ti.f32, shape=MAX_VOXELS)
voxel_colors = ti.Vector.field(3, dtype=ti.f32, shape=MAX_VOXELS)
voxel_radii = ti.field(dtype=ti.f32, shape=MAX_VOXELS)  # Per-vertex radius for mixed voxel/debris sizes

# Floor mesh fields (flat quads + bevel skirts — single merged mesh for one draw call)
MAX_FLOOR_QUADS = 10000
MAX_FLOOR_VERTS = MAX_FLOOR_QUADS * 4

num_floor_quads = ti.field(dtype=ti.i32, shape=())
floor_vertices = ti.Vector.field(3, dtype=ti.f32, shape=MAX_FLOOR_VERTS)
floor_normals = ti.Vector.field(3, dtype=ti.f32, shape=MAX_FLOOR_VERTS)
floor_colors = ti.Vector.field(3, dtype=ti.f32, shape=MAX_FLOOR_VERTS)

# Floor texture contrast (runtime-tunable — changing these won't trigger kernel recompilation)
stone_coarse_strength = ti.field(dtype=ti.f32, shape=())   # ±coarse patch variation
stone_fine_strength = ti.field(dtype=ti.f32, shape=())     # ±fine grain variation
stone_temp_strength = ti.field(dtype=ti.f32, shape=())     # ±warm/cool shift
stone_coarse_strength[None] = 0.035
stone_fine_strength[None] = 0.055
stone_temp_strength[None] = 0.02

# Silk emissive brightness (runtime-tunable)
silk_emissive = ti.field(dtype=ti.f32, shape=())
silk_emissive[None] = 2.8

# Pre-computed numpy index array (static quad pattern) — avoids GPU-CPU sync
_floor_indices_np = np.zeros(MAX_FLOOR_QUADS * 6, dtype=np.int32)
for _q in range(MAX_FLOOR_QUADS):
    _bv, _bi = _q * 4, _q * 6
    _floor_indices_np[_bi:_bi+6] = [_bv, _bv+1, _bv+2, _bv, _bv+2, _bv+3]

SKIRT_DEPTH = 2.0
SKIRT_SHADE = 0.55

# Shadow disc mesh fields (perfect circles instead of grid-based blobs)
MAX_SHADOW_DISCS = 4  # 2 beetles + 1 ball + 1 spare
SHADOW_DISC_SEGMENTS = 32
SHADOW_VERTS_PER_DISC = SHADOW_DISC_SEGMENTS + 1  # center + ring
SHADOW_TRIS_PER_DISC = SHADOW_DISC_SEGMENTS
MAX_SHADOW_VERTS = MAX_SHADOW_DISCS * SHADOW_VERTS_PER_DISC
MAX_SHADOW_INDICES = MAX_SHADOW_DISCS * SHADOW_TRIS_PER_DISC * 3

num_shadow_discs = ti.field(dtype=ti.i32, shape=())
shadow_disc_params = ti.Vector.field(3, dtype=ti.f32, shape=MAX_SHADOW_DISCS)  # [x, z, radius]
shadow_vertices = ti.Vector.field(3, dtype=ti.f32, shape=MAX_SHADOW_VERTS)
shadow_normals = ti.Vector.field(3, dtype=ti.f32, shape=MAX_SHADOW_VERTS)
shadow_colors = ti.Vector.field(3, dtype=ti.f32, shape=MAX_SHADOW_VERTS)

# Pre-computed numpy index array for triangle fans
_shadow_indices_np = np.zeros(MAX_SHADOW_INDICES, dtype=np.int32)
for _d in range(MAX_SHADOW_DISCS):
    _center = _d * SHADOW_VERTS_PER_DISC
    for _s in range(SHADOW_DISC_SEGMENTS):
        _ti_base = (_d * SHADOW_TRIS_PER_DISC + _s) * 3
        _shadow_indices_np[_ti_base] = _center
        _shadow_indices_np[_ti_base + 1] = _center + 1 + _s
        _shadow_indices_np[_ti_base + 2] = _center + 1 + (_s + 1) % SHADOW_DISC_SEGMENTS

# Particle radius constants
VOXEL_RADIUS = 0.407  # Standard voxel size (10% bigger)
DEBRIS_RADIUS = 0.25  # Smaller dust/debris particles

# Arena SDF shape mode for smooth border snapping
# 0=disabled, 1=circle, 2=donut, 3=x_stage, 4=barbell, 5=yinyang, 6=hourglass, 7=square_bridge, 8=circle+bowl, 9=figure8
arena_shape_mode = ti.field(ti.i32, shape=())

# Precomputed SDF cache at half-voxel resolution (257x257 covers grid 0..128 corners)
# Index (ci, ck) maps to world coords: wx = ci * 0.5 - 64.0, wz = ck * 0.5 - 64.0
# Voxel (i, k) corner v0 maps to ci=2*i-1, ck=2*k-1 etc.
sdf_cache = ti.field(dtype=ti.f32, shape=(257, 257))

def set_arena_snap(mode):
    arena_shape_mode[None] = mode
    if mode > 0:
        precompute_sdf(mode)
    invalidate_floor_cache()

# Floor rendering toggle (mesh quads vs old sphere voxels)
mesh_floor_enabled = True

# Floor mesh caching — arena floor is static, no need to rebuild every frame
floor_cache_valid = False
cached_floor_count = 0
ice_active = False  # When True, bypass floor cache so ice circles update each frame

def invalidate_floor_cache():
    global floor_cache_valid, cached_floor_count
    floor_cache_valid = False
    cached_floor_count = 0

def set_ice_active(active):
    global ice_active
    ice_active = active

# Moving hole hazard state
hole_active = False

def set_hole_active(active):
    global hole_active
    hole_active = active

# Projectiles (cannonballs) are now merged into main voxel buffer with larger radius
# This eliminates a separate scene.particles() call, reducing CPU->GPU sync overhead

# Ice circle params for smooth distance-based blending (c1x, c1z, c2x, c2z, radius)
# When radius > 0, ice is active and get_voxel_color uses smooth falloff instead of binary overlay
ice_params = ti.Vector.field(5, dtype=ti.f32, shape=())

# Moving hole params (cx, cz, radius) — floor voxels inside this circle are skipped
hole_params = ti.Vector.field(3, dtype=ti.f32, shape=())

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

    # Concrete - customizable arena floor color (with smooth ice blending)
    elif voxel_type == 2:  # CONCRETE
        color = simulation.board_color[None]
        ip = ice_params[None]
        if ip[4] > 0.0:  # radius > 0 means ice active
            d1 = ti.sqrt((world_x - ip[0])**2 + (world_z - ip[1])**2)
            d2 = ti.sqrt((world_x - ip[2])**2 + (world_z - ip[3])**2)
            dist = ti.min(d1, d2)
            blend = ti.math.clamp((ip[4] - dist) / 2.0, 0.0, 1.0)
            ice_color = ti.math.vec3(0.45, 0.65, 0.88)
            color = color * (1.0 - blend) + ice_color * blend

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

    # Dung ball - use customizable ball color
    elif voxel_type == 16:  # BALL
        color = simulation.ball_color[None]

    # Dung ball stripe - use customizable ball stripe color
    elif voxel_type == 17:  # BALL_STRIPE
        color = simulation.ball_stripe_color[None]

    # Stag beetle hook interior - use body color (inner curve of pincers)
    elif voxel_type == 18:  # STAG_HOOK_INTERIOR_BLUE
        color = simulation.blue_body_color[None]
    elif voxel_type == 19:  # STAG_HOOK_INTERIOR_RED
        color = simulation.red_body_color[None]

    # Shadow blob beneath airborne beetles (contrast-adaptive)
    elif voxel_type == 20:  # SHADOW
        bc = simulation.board_color[None]
        lum = bc[0] * 0.299 + bc[1] * 0.587 + bc[2] * 0.114
        if lum > 0.3:
            color = bc * 0.05
        else:
            color = bc * 0.4 + ti.math.vec3(0.18, 0.18, 0.18)

    # Slippery bowl perimeter (slightly blue-tinted to indicate slippery)
    elif voxel_type == 21:  # SLIPPERY
        color = ti.math.vec3(0.35, 0.40, 0.50)  # Blue-gray to indicate slippery ice-like surface
        ip = ice_params[None]
        if ip[4] > 0.0:  # radius > 0 means ice active
            d1 = ti.sqrt((world_x - ip[0])**2 + (world_z - ip[1])**2)
            d2 = ti.sqrt((world_x - ip[2])**2 + (world_z - ip[3])**2)
            dist = ti.min(d1, d2)
            blend = ti.math.clamp((ip[4] - dist) / 2.0, 0.0, 1.0)
            ice_color = ti.math.vec3(0.45, 0.65, 0.88)
            color = color * (1.0 - blend) + ice_color * blend

    # Goal doorway walls (sandy/tan stone)
    elif voxel_type == 22:  # GOAL
        color = ti.math.vec3(0.6, 0.5, 0.3)  # Sandy/tan stone color
    # Floating score digits (with flash effect support)
    elif voxel_type == 23:  # SCORE_DIGIT_BLUE
        flash = simulation.blue_score_flash[None]
        color = ti.math.vec3(0.3 * flash, 0.6 * flash, 1.0 * flash)  # Bright blue with flash
    elif voxel_type == 24:  # SCORE_DIGIT_RED
        flash = simulation.red_score_flash[None]
        color = ti.math.vec3(1.0 * flash, 0.3 * flash, 0.2 * flash)  # Bright red with flash
    # Assembly animation voxels - use exact beetle body colors (dynamically from settings)
    elif voxel_type == 25:  # ASSEMBLY_VOXEL_BLUE
        color = simulation.blue_body_color[None]
    elif voxel_type == 26:  # ASSEMBLY_VOXEL_RED
        color = simulation.red_body_color[None]
    elif voxel_type == 27:  # ASSEMBLY_VOXEL_BALL
        color = simulation.ball_color[None]
    elif voxel_type == 28:  # ASSEMBLY_VOXEL_BALL_STRIPE
        color = simulation.ball_stripe_color[None]
    elif voxel_type == 29:  # ASSEMBLY_VOXEL_BLUE_STRIPE
        color = simulation.blue_stripe_color[None]
    elif voxel_type == 30:  # ASSEMBLY_VOXEL_RED_STRIPE
        color = simulation.red_stripe_color[None]
    elif voxel_type == 31:  # ASSEMBLY_VOXEL_BLUE_HORN_TIP
        color = simulation.blue_horn_tip_color[None]
    elif voxel_type == 32:  # ASSEMBLY_VOXEL_RED_HORN_TIP
        color = simulation.red_horn_tip_color[None]
    # Scorpion venom tip - glows based on venom charges
    elif voxel_type == 33:  # VENOM_TIP_BLUE
        color = simulation.blue_venom_tip_color[None]
    elif voxel_type == 34:  # VENOM_TIP_RED
        color = simulation.red_venom_tip_color[None]

    # Ladybug cheerleader colors
    elif voxel_type == 35:  # LADYBUG_SHELL - bright red
        color = ti.math.vec3(0.85, 0.12, 0.08)
    elif voxel_type == 36:  # LADYBUG_SPOTS - black
        color = ti.math.vec3(0.08, 0.08, 0.08)
    elif voxel_type == 37:  # LADYBUG_HEAD - black
        color = ti.math.vec3(0.1, 0.1, 0.1)
    elif voxel_type == 38:  # LADYBUG_LEGS - dark brown/black
        color = ti.math.vec3(0.12, 0.1, 0.08)
    elif voxel_type == 39:  # LADYBUG_WINGS - translucent amber/gold
        color = ti.math.vec3(0.95, 0.85, 0.6)

    # Title screen text colors (with flash effect)
    elif voxel_type == 40:  # TITLE_BLUE - "BEETLE" text
        flash = simulation.title_flash[None]
        color = ti.math.vec3(0.3 * flash, 0.6 * flash, 1.0 * flash)
    elif voxel_type == 41:  # TITLE_RED - "BATTLE" text
        flash = simulation.title_flash[None]
        color = ti.math.vec3(1.0 * flash, 0.3 * flash, 0.2 * flash)
    elif voxel_type == 42:  # TITLE_GOLD - "BROS" text
        flash = simulation.title_flash[None]
        color = ti.math.vec3(0.9 * flash, 0.75 * flash, 0.3 * flash)
    elif voxel_type == 43:  # TITLE_WHITE - controls text
        color = ti.math.vec3(0.9, 0.9, 0.85)
    elif voxel_type == 44:  # TITLE_PINK - "SPACE/A/START" text (with flash)
        flash = simulation.title_flash[None]
        color = ti.math.vec3(1.0 * flash, 0.4 * flash, 0.7 * flash)  # Nice pink
    elif voxel_type == 45:  # UFO_HULL - silver metallic
        color = ti.math.vec3(0.75, 0.78, 0.82)
    elif voxel_type == 46:  # UFO_DOME - green glass (flashes during telegraph/fire)
        flash = simulation.ufo_dome_flash[None]
        color = ti.math.vec3(0.2 * flash, 0.85 * flash, 0.3 * flash)
    elif voxel_type == 47:  # UFO_LIGHTS - warm orange engine glow
        color = ti.math.vec3(1.0, 0.6, 0.15)
    elif voxel_type == 48:  # UFO_BEAM - bright green
        color = ti.math.vec3(0.3, 1.0, 0.3)
    elif voxel_type == 49:  # UFO_RIM - bright purple belt
        color = ti.math.vec3(0.7, 0.15, 0.95)
    # OPTIMIZATION: Metallic sheen from lookup table instead of sin() (~8-12% speedup)
    if (voxel_type >= 5 and voxel_type <= 15) or voxel_type == 18 or voxel_type == 19 or voxel_type == 33 or voxel_type == 34 or voxel_type == 35 or voxel_type == 45 or voxel_type == 49:  # All beetle/ladybug shell/UFO hull/rim parts
        shimmer = get_shimmer_from_lut(world_x, world_z)
        color *= shimmer

    return color

@ti.func
def get_floor_color(voxel_type: ti.i32, world_x: ti.f32, world_z: ti.f32) -> ti.math.vec3:
    """Simplified color for floor-only types (CONCRETE=2, SLIPPERY=21).
    Avoids inlining the full 45-branch get_voxel_color into merge kernels."""
    color = simulation.board_color[None]
    if voxel_type == 21:  # SLIPPERY
        color = ti.math.vec3(0.35, 0.40, 0.50)
    ip = ice_params[None]
    if ip[4] > 0.0:
        d1 = ti.sqrt((world_x - ip[0])**2 + (world_z - ip[1])**2)
        d2 = ti.sqrt((world_x - ip[2])**2 + (world_z - ip[3])**2)
        dist = ti.min(d1, d2)
        blend = ti.math.clamp((ip[4] - dist) / 2.0, 0.0, 1.0)
        ice_color = ti.math.vec3(0.45, 0.65, 0.88)
        color = color * (1.0 - blend) + ice_color * blend
    return color

@ti.func
def _region_hash(ri: ti.i32, rk: ti.i32) -> ti.f32:
    """Hash a region coordinate to a -0.5..+0.5 value."""
    h = (ri * 48611) ^ (rk * 95317)
    return (h % 1000) / 1000.0 - 0.5

@ti.func
def stone_texture_color(color: ti.math.vec3, ci: ti.i32, ck: ti.i32) -> ti.math.vec3:
    """Two-scale stone texture with smooth blending for floor quads"""
    # Coarse: smoothly blended region tonal shift (no hard edges)
    REGION = ti.static(4)
    ri = ci // REGION
    rk = ck // REGION
    # Fractional position within region cell (0..1)
    fi = float(ci % REGION) / float(REGION)
    fk = float(ck % REGION) / float(REGION)
    # Bilinear interpolation of 4 neighboring region hashes
    h00 = _region_hash(ri, rk)
    h10 = _region_hash(ri + 1, rk)
    h01 = _region_hash(ri, rk + 1)
    h11 = _region_hash(ri + 1, rk + 1)
    q_shift = (h00 * (1.0 - fi) * (1.0 - fk) + h10 * fi * (1.0 - fk) +
               h01 * (1.0 - fi) * fk + h11 * fi * fk) * stone_coarse_strength[None]
    # Fine: per-corner grain
    h = (ci * 73856093) ^ (ck * 19349663)
    fine = ((h % 1000) / 1000.0 - 0.5) * stone_fine_strength[None]
    # Warm/cool color temperature shift
    h2 = (ci * 29423) ^ (ck * 61781)
    temp = ((h2 % 1000) / 1000.0 - 0.5) * stone_temp_strength[None]
    warm = ti.math.vec3(temp, 0.0, -temp)
    return color * (1.0 + q_shift + fine) + warm

@ti.func
def arena_sdf(x: ti.f32, z: ti.f32, mode: ti.i32) -> ti.f32:
    """Signed distance function for arena shapes. Negative=inside, positive=outside."""
    d = 999.0
    if mode == 1:
        # Circle: radius 32
        d = ti.sqrt(x * x + z * z) - 32.0
    elif mode == 2:
        # Donut: outer r=32, inner r=13
        dist = ti.sqrt(x * x + z * z)
        d = ti.max(dist - 32.0, 13.0 - dist)
    elif mode == 3:
        # X Stage: circle r=32 intersected with cross (arm half-width=12)
        dist = ti.sqrt(x * x + z * z)
        d = ti.max(dist - 32.0, ti.min(ti.abs(x) - 12.0, ti.abs(z) - 12.0))
    elif mode == 4:
        # Barbell: two circles r=20 at x=+/-22, plus bridge |x|<=22 |z|<=6
        dx_l = x + 22.0
        dist_left = ti.sqrt(dx_l * dx_l + z * z)
        dx_r = x - 22.0
        dist_right = ti.sqrt(dx_r * dx_r + z * z)
        bridge = ti.max(ti.abs(x) - 22.0, ti.abs(z) - 6.0)
        d = ti.min(ti.min(dist_left - 20.0, dist_right - 20.0), bridge)
    elif mode == 5:
        # Yin Yang: ring (inner=26, outer=38) + S-curve bridge
        dist = ti.sqrt(x * x + z * z)
        ring = ti.max(dist - 38.0, 26.0 - dist)
        # Upper arc: center (0, 20.8), radius 20.8, half-width 5
        # Restricted to z>=-5, x>=-5, and inside inner circle (dist<=26)
        arc_dz_u = z - 20.8
        arc_dist_u = ti.sqrt(x * x + arc_dz_u * arc_dz_u)
        upper_arc = ti.max(ti.abs(arc_dist_u - 20.8) - 5.0, ti.max(-z - 5.0, ti.max(-x - 5.0, dist - 26.0)))
        # Lower arc: center (0, -20.8), radius 20.8, half-width 5
        # Restricted to z<=5, x<=5, and inside inner circle (dist<=26)
        arc_dz_l = z + 20.8
        arc_dist_l = ti.sqrt(x * x + arc_dz_l * arc_dz_l)
        lower_arc = ti.max(ti.abs(arc_dist_l - 20.8) - 5.0, ti.max(z - 5.0, ti.max(x - 5.0, dist - 26.0)))
        d = ti.min(ring, ti.min(upper_arc, lower_arc))
    elif mode == 6:
        # Hourglass: |x|<=32, |z|<=6+0.5625*|x|
        d = ti.max(ti.abs(x) - 32.0, ti.abs(z) - 6.0 - 0.5625 * ti.abs(x))
    elif mode == 7:
        # Square Bridge: outer 38x25, hole 28x15, bridge |x|<=28 |z|<=5
        outer = ti.max(ti.abs(x) - 38.0, ti.abs(z) - 25.0)
        hole_interior = ti.min(28.0 - ti.abs(x), 15.0 - ti.abs(z))
        ring = ti.max(outer, hole_interior)
        bridge = ti.max(ti.abs(x) - 28.0, ti.abs(z) - 5.0)
        d = ti.min(ring, bridge)
    elif mode == 8:
        # Circle with bowl perimeter (beetle ball): radius 44 (32 arena + 12 bowl)
        d = ti.sqrt(x * x + z * z) - 44.0
    elif mode == 9:
        # True figure 8: two circles r=18 at x=+-18, path half-width 6
        dx_l = x + 18.0
        dist_left = ti.sqrt(dx_l * dx_l + z * z)
        dx_r = x - 18.0
        dist_right = ti.sqrt(dx_r * dx_r + z * z)
        d = ti.min(ti.abs(dist_left - 18.0), ti.abs(dist_right - 18.0)) - 6.0
    elif mode == 10:
        # Squiggle: serpentine with 5 vertical segments + 4 horizontal connectors
        hw = ti.static(5.0)  # path half-width
        min_d = ti.cast(999.0, ti.f32)
        # 5 vertical segments at x = -42, -21, 0, +21, +42; z from -20 to +20
        for seg_i in ti.static(range(5)):
            sx = -42.0 + seg_i * 21.0
            dx_s = x - sx
            cz_s = ti.max(0.0, ti.max(-20.0 - z, z - 20.0))
            seg_d = ti.sqrt(dx_s * dx_s + cz_s * cz_s)
            min_d = ti.min(min_d, seg_d)
        # 4 horizontal connectors
        for conn_i in ti.static(range(4)):
            cx_l = -42.0 + conn_i * 21.0
            cx_r = cx_l + 21.0
            conn_z = 20.0 if conn_i % 2 == 0 else -20.0
            dz_c = z - conn_z
            cx_c = ti.max(0.0, ti.max(cx_l - x, x - cx_r))
            conn_d = ti.sqrt(cx_c * cx_c + dz_c * dz_c)
            min_d = ti.min(min_d, conn_d)
        d = min_d - hw
    return d

@ti.kernel
def precompute_sdf(mode: ti.i32):
    """Fill sdf_cache at half-voxel resolution. Called once when arena changes."""
    for ci, ck in ti.ndrange(257, 257):
        wx = ci * 0.5 - 64.0
        wz = ck * 0.5 - 64.0
        sdf_cache[ci, ck] = arena_sdf(wx, wz, mode)

@ti.func
def sdf_snap_corner_cached(ci: ti.i32, ck: ti.i32) -> ti.math.vec2:
    """Project corner onto SDF=0 boundary using cached SDF + Newton step."""
    wx = ci * 0.5 - 64.0
    wz = ck * 0.5 - 64.0
    sdf_val = sdf_cache[ci, ck]
    # Gradient from cached finite differences (spacing = 0.5 world units)
    gx = (sdf_cache[ci + 1, ck] - sdf_cache[ci - 1, ck])  # / (2*0.5) = /1.0
    gz = (sdf_cache[ci, ck + 1] - sdf_cache[ci, ck - 1])
    g_dot = gx * gx + gz * gz
    rx = wx
    rz = wz
    if g_dot > 1.0e-8:
        step_x = sdf_val * gx / g_dot
        step_z = sdf_val * gz / g_dot
        # Clamp step to prevent overshoot at SDF gradient discontinuities
        step_len = ti.sqrt(step_x * step_x + step_z * step_z)
        if step_len > 1.5:
            step_x *= 1.5 / step_len
            step_z *= 1.5 / step_len
        rx = wx - step_x
        rz = wz - step_z
    return ti.math.vec2(rx, rz)

@ti.kernel
def build_shadow_discs(floor_y: ti.f32, voxel_field: ti.template(), n_grid: ti.i32, use_mesh_floor: ti.i32):
    """Build circular disc meshes for shadows, clipped to arena floor"""
    CONCRETE_T = ti.static(2)
    SLIPPERY_T = ti.static(21)
    PI2 = 3.14159265358979 * 2.0
    floor_j = ti.cast(floor_y, ti.i32)

    for d in range(num_shadow_discs[None]):
        cx = shadow_disc_params[d][0]
        cz = shadow_disc_params[d][1]
        radius = shadow_disc_params[d][2]
        base = d * SHADOW_VERTS_PER_DISC
        # Mesh floor: slight offset above quads. Sphere floor: higher to sit above sphere tops
        top_y = floor_y + VOXEL_RADIUS + 0.06
        if use_mesh_floor == 0:
            top_y = floor_y + VOXEL_RADIUS + 0.15
        # Shadow color: subtract fixed amount from board color (constant contrast)
        bc = simulation.board_color[None]
        drop = ti.math.vec3(0.15, 0.15, 0.15)
        if use_mesh_floor == 0:
            drop = ti.math.vec3(0.29, 0.29, 0.29)
        shadow_color = ti.max(bc - drop, 0.0)
        up = ti.math.vec3(0.0, 1.0, 0.0)
        center_pos = ti.math.vec3(cx, top_y, cz)

        # Check if center is on floor — if not, skip entire disc
        ci = ti.cast(cx + n_grid / 2.0, ti.i32)
        ck = ti.cast(cz + n_grid / 2.0, ti.i32)
        center_on_floor = 0
        if 0 <= ci < n_grid and 0 <= ck < n_grid:
            cvt = voxel_field[ci, floor_j, ck]
            if cvt == CONCRETE_T or cvt == SLIPPERY_T:
                center_on_floor = 1

        # Edge color: gradient falloff (subtler on sphere floor)
        edge_color = shadow_color * 0.7 + bc * 0.3
        if use_mesh_floor:
            edge_color = shadow_color * 0.5 + bc * 0.5

        # Center vertex (darkest)
        shadow_vertices[base] = center_pos
        shadow_normals[base] = up
        shadow_colors[base] = shadow_color

        # Ring vertices — walk inward along radius until on floor (smooth edge clipping)
        for s in range(SHADOW_DISC_SEGMENTS):
            angle = PI2 * s / SHADOW_DISC_SEGMENTS
            dx = ti.cos(angle)
            dz = ti.sin(angle)

            # Start at full radius, step inward until on floor (8 steps = ~1 voxel resolution)
            placed = 0
            if center_on_floor:
                for step in range(9):  # 0=full radius, 8=center
                    frac = 1.0 - step / 8.0
                    vx = cx + radius * frac * dx
                    vz = cz + radius * frac * dz
                    gi = ti.cast(vx + n_grid / 2.0, ti.i32)
                    gk = ti.cast(vz + n_grid / 2.0, ti.i32)
                    if 0 <= gi < n_grid and 0 <= gk < n_grid:
                        vt = voxel_field[gi, floor_j, gk]
                        if vt == CONCRETE_T or vt == SLIPPERY_T:
                            shadow_vertices[base + 1 + s] = ti.math.vec3(vx, top_y, vz)
                            placed = 1
                            break

            if placed == 0:
                shadow_vertices[base + 1 + s] = center_pos
            shadow_normals[base + 1 + s] = up
            shadow_colors[base + 1 + s] = edge_color

def set_shadow_params(index, x, z, radius):
    """Set shadow disc position/radius (called from beetle_physics)"""
    shadow_disc_params[index][0] = x
    shadow_disc_params[index][1] = z
    shadow_disc_params[index][2] = radius

def set_num_shadows(count):
    """Set number of active shadow discs (called from beetle_physics)"""
    num_shadow_discs[None] = count

@ti.func
def _emit_merged_quad(run_start: ti.i32, i_end: ti.i32, j: ti.i32, k: ti.i32,
                       half_grid: ti.f32, top_y: ti.f32, up: ti.math.vec3,
                       vt_start: ti.i32, vt_end: ti.i32):
    """Emit a single wide quad covering cells [run_start, i_end] in row k."""
    HALF = ti.static(0.5)
    wx_start = float(run_start) - half_grid
    wx_end = float(i_end) - half_grid
    wz = float(k) - half_grid
    qi = ti.atomic_add(num_floor_quads[None], 1)
    if qi < MAX_FLOOR_QUADS:
        base = qi * 4
        floor_vertices[base + 0] = ti.math.vec3(wx_start - HALF, top_y, wz - HALF)
        floor_vertices[base + 1] = ti.math.vec3(wx_end + HALF, top_y, wz - HALF)
        floor_vertices[base + 2] = ti.math.vec3(wx_end + HALF, top_y, wz + HALF)
        floor_vertices[base + 3] = ti.math.vec3(wx_start - HALF, top_y, wz + HALF)
        floor_normals[base + 0] = up
        floor_normals[base + 1] = up
        floor_normals[base + 2] = up
        floor_normals[base + 3] = up
        c0 = get_floor_color(vt_start, wx_start - HALF, wz - HALF)
        c1 = get_floor_color(vt_end, wx_end + HALF, wz - HALF)
        c2 = get_floor_color(vt_end, wx_end + HALF, wz + HALF)
        c3 = get_floor_color(vt_start, wx_start - HALF, wz + HALF)
        floor_colors[base + 0] = stone_texture_color(c0, run_start, k)
        floor_colors[base + 1] = stone_texture_color(c1, i_end + 1, k)
        floor_colors[base + 2] = stone_texture_color(c2, i_end + 1, k + 1)
        floor_colors[base + 3] = stone_texture_color(c3, run_start, k + 1)


@ti.kernel
def merge_interior_floor(voxel_field: ti.template(), n_grid: ti.i32, floor_j: ti.i32):
    """Row-wise greedy merge of fully-interior floor cells into wide quads.

    Reduces ~2800 interior 1x1 quads to ~80-100 row-runs.
    """
    CONCRETE = ti.static(2)
    SLIPPERY = ti.static(21)

    up = ti.math.vec3(0.0, 1.0, 0.0)
    half_grid = float(n_grid) / 2.0

    # Scan floor_j through floor_j+2 to cover bowl perimeter raised voxels (j=34)
    for k in range(2, 126):
        for j_off in range(3):
            j = floor_j + j_off
            top_y = float(j) + VOXEL_RADIUS
            run_start = -1
            for i in range(2, 126):
                vtype = voxel_field[i, j, k]
                is_interior = 0
                if vtype == CONCRETE or vtype == SLIPPERY:
                    n_mx = voxel_field[i - 1, j, k]
                    n_px = voxel_field[i + 1, j, k]
                    n_mz = voxel_field[i, j, k - 1]
                    n_pz = voxel_field[i, j, k + 1]
                    n_mxmz = voxel_field[i - 1, j, k - 1]
                    n_pxmz = voxel_field[i + 1, j, k - 1]
                    n_pxpz = voxel_field[i + 1, j, k + 1]
                    n_mxpz = voxel_field[i - 1, j, k + 1]
                    if (n_mx == CONCRETE or n_mx == SLIPPERY) and \
                       (n_px == CONCRETE or n_px == SLIPPERY) and \
                       (n_mz == CONCRETE or n_mz == SLIPPERY) and \
                       (n_pz == CONCRETE or n_pz == SLIPPERY) and \
                       (n_mxmz == CONCRETE or n_mxmz == SLIPPERY) and \
                       (n_pxmz == CONCRETE or n_pxmz == SLIPPERY) and \
                       (n_pxpz == CONCRETE or n_pxpz == SLIPPERY) and \
                       (n_mxpz == CONCRETE or n_mxpz == SLIPPERY):
                        is_interior = 1
                    # Skip interior voxels inside the moving hole
                    if is_interior == 1:
                        hp = hole_params[None]
                        if hp[2] > 0.0:
                            wx = float(i) - half_grid
                            wz = float(k) - half_grid
                            hdx = wx - hp[0]
                            hdz = wz - hp[1]
                            if hdx * hdx + hdz * hdz < hp[2] * hp[2]:
                                is_interior = 0

                if is_interior:
                    if run_start < 0:
                        run_start = i
                    if i - run_start >= 3:  # Max 4 cells per merged quad
                        vt0 = voxel_field[run_start, j, k]
                        vt1 = voxel_field[i, j, k]
                        _emit_merged_quad(run_start, i, j, k, half_grid, top_y, up, vt0, vt1)
                        run_start = -1
                else:
                    if run_start >= 0:
                        i_end = i - 1
                        vt0 = voxel_field[run_start, j, k]
                        vt1 = voxel_field[i_end, j, k]
                        _emit_merged_quad(run_start, i_end, j, k, half_grid, top_y, up, vt0, vt1)
                        run_start = -1

            # End of row: flush any open run
            if run_start >= 0:
                i_end = 125
                vt0 = voxel_field[run_start, j, k]
                vt1 = voxel_field[i_end, j, k]
                _emit_merged_quad(run_start, i_end, j, k, half_grid, top_y, up, vt0, vt1)


@ti.kernel
def extract_voxels(voxel_field: ti.template(), n_grid: ti.i32, use_mesh_floor: ti.i32, skip_floor: ti.i32):
    """Extract arena/beetle voxels into render buffer (split from megakernel for faster compilation)."""
    # ===== PHASE 1: Extract arena/beetle voxels =====
    # Static bounding box optimization: only scan active arena region
    EMPTY = ti.static(0)
    DEBRIS_TYPE = ti.static(4)

    CONCRETE = ti.static(2)
    SLIPPERY = ti.static(21)
    HALF = ti.static(0.5)

    for i, j, k in ti.ndrange((2, 126), (1, 100), (2, 126)):
        vtype = voxel_field[i, j, k]
        if vtype != EMPTY and vtype != DEBRIS_TYPE:
            world_x = float(i) - n_grid / 2.0
            world_y = float(j)
            world_z = float(k) - n_grid / 2.0
            color = get_voxel_color(vtype, world_x, world_z)

            if vtype == CONCRETE or vtype == SLIPPERY:
                # Check if floor voxel is inside the moving hole
                hp = hole_params[None]
                in_hole = 0
                if hp[2] > 0.0:
                    hdx = world_x - hp[0]
                    hdz = world_z - hp[1]
                    if hdx * hdx + hdz * hdz < hp[2] * hp[2]:
                        in_hole = 1
                if in_hole:
                    pass  # Skip - floor voxel is inside the hole
                elif use_mesh_floor and skip_floor:
                    # Floor mesh cached — skip entirely
                    pass
                elif use_mesh_floor:
                    shape_mode = arena_shape_mode[None]
                    if shape_mode > 0:
                        # SDF snap mode — cache 4 cardinal neighbors
                        n_mx = voxel_field[i - 1, j, k]
                        n_px = voxel_field[i + 1, j, k]
                        n_mz = voxel_field[i, j, k - 1]
                        n_pz = voxel_field[i, j, k + 1]
                        is_floor_mx = (n_mx == CONCRETE or n_mx == SLIPPERY)
                        is_floor_px = (n_px == CONCRETE or n_px == SLIPPERY)
                        is_floor_mz = (n_mz == CONCRETE or n_mz == SLIPPERY)
                        is_floor_pz = (n_pz == CONCRETE or n_pz == SLIPPERY)
                        all_cardinal = is_floor_mx and is_floor_px and is_floor_mz and is_floor_pz

                        top_y = world_y + VOXEL_RADIUS
                        up = ti.math.vec3(0.0, 1.0, 0.0)

                        if all_cardinal:
                            # Check diagonals to distinguish fully-interior vs partial
                            n_mxmz = voxel_field[i - 1, j, k - 1]
                            n_pxmz = voxel_field[i + 1, j, k - 1]
                            n_pxpz = voxel_field[i + 1, j, k + 1]
                            n_mxpz = voxel_field[i - 1, j, k + 1]
                            all_diag = (n_mxmz == CONCRETE or n_mxmz == SLIPPERY) and \
                                       (n_pxmz == CONCRETE or n_pxmz == SLIPPERY) and \
                                       (n_pxpz == CONCRETE or n_pxpz == SLIPPERY) and \
                                       (n_mxpz == CONCRETE or n_mxpz == SLIPPERY)
                            if all_diag:
                                pass  # Fully interior — merge kernel handles
                            else:
                                # Partial interior: diag-triggered snap, no bevel
                                qi = ti.atomic_add(num_floor_quads[None], 1)
                                if qi < MAX_FLOOR_QUADS:
                                    base = qi * 4
                                    for v in ti.static(range(4)):
                                        corner_x = world_x + (-HALF if v == 0 or v == 3 else HALF)
                                        corner_z = world_z + (-HALF if v == 0 or v == 1 else HALF)
                                        n_diag = n_mxmz  # default (v==0)
                                        if ti.static(v == 1):
                                            n_diag = n_pxmz
                                        elif ti.static(v == 2):
                                            n_diag = n_pxpz
                                        elif ti.static(v == 3):
                                            n_diag = n_mxpz
                                        if n_diag != CONCRETE and n_diag != SLIPPERY:
                                            corner_ci = 2 * i + (-1 if v == 0 or v == 3 else 1)
                                            corner_ck = 2 * k + (-1 if v == 0 or v == 1 else 1)
                                            snapped = sdf_snap_corner_cached(corner_ci, corner_ck)
                                            corner_x = snapped[0]
                                            corner_z = snapped[1]
                                        floor_vertices[base + v] = ti.math.vec3(corner_x, top_y, corner_z)
                                        floor_normals[base + v] = up
                                        ci = i + (1 if v == 1 or v == 2 else 0)
                                        ck = k + (1 if v == 2 or v == 3 else 0)
                                        floor_colors[base + v] = stone_texture_color(color, ci, ck)
                        else:
                                # Edge voxel: full snap + bevel (uses cached cardinals)
                                qi = ti.atomic_add(num_floor_quads[None], 1)
                                if qi < MAX_FLOOR_QUADS:
                                    base = qi * 4
                                    n_mxmz = voxel_field[i - 1, j, k - 1]
                                    n_pxmz = voxel_field[i + 1, j, k - 1]
                                    n_pxpz = voxel_field[i + 1, j, k + 1]
                                    n_mxpz = voxel_field[i - 1, j, k + 1]

                                    snap_x = ti.Vector([0.0, 0.0, 0.0, 0.0])
                                    snap_z = ti.Vector([0.0, 0.0, 0.0, 0.0])

                                    for v in ti.static(range(4)):
                                        corner_x = world_x + (-HALF if v == 0 or v == 3 else HALF)
                                        corner_z = world_z + (-HALF if v == 0 or v == 1 else HALF)
                                        nc = is_floor_mx  # default (v==0)
                                        nz_v = is_floor_mz
                                        nd = (n_mxmz == CONCRETE or n_mxmz == SLIPPERY)
                                        if ti.static(v == 1):
                                            nc = is_floor_px
                                            nz_v = is_floor_mz
                                            nd = (n_pxmz == CONCRETE or n_pxmz == SLIPPERY)
                                        elif ti.static(v == 2):
                                            nc = is_floor_px
                                            nz_v = is_floor_pz
                                            nd = (n_pxpz == CONCRETE or n_pxpz == SLIPPERY)
                                        elif ti.static(v == 3):
                                            nc = is_floor_mx
                                            nz_v = is_floor_pz
                                            nd = (n_mxpz == CONCRETE or n_mxpz == SLIPPERY)
                                        if not nc or not nz_v or not nd:
                                            corner_ci = 2 * i + (-1 if v == 0 or v == 3 else 1)
                                            corner_ck = 2 * k + (-1 if v == 0 or v == 1 else 1)
                                            snapped = sdf_snap_corner_cached(corner_ci, corner_ck)
                                            corner_x = snapped[0]
                                            corner_z = snapped[1]

                                        snap_x[v] = corner_x
                                        snap_z[v] = corner_z
                                        floor_vertices[base + v] = ti.math.vec3(corner_x, top_y, corner_z)
                                        floor_normals[base + v] = up
                                        ci = i + (1 if v == 1 or v == 2 else 0)
                                        ck = k + (1 if v == 2 or v == 3 else 0)
                                        floor_colors[base + v] = stone_texture_color(color, ci, ck)

                                    # Bevel (skirt) quads — merged into floor mesh
                                    # All bevels use downward normal so lighting is uniform (hides seams)
                                    dark_color = color * SKIRT_SHADE
                                    bot_y = top_y - SKIRT_DEPTH
                                    bevel_norm = ti.math.vec3(0.0, 1.0, 0.0)
                                    if not is_floor_mx:
                                        bi = ti.atomic_add(num_floor_quads[None], 1)
                                        if bi < MAX_FLOOR_QUADS:
                                            bb = bi * 4
                                            floor_vertices[bb + 0] = ti.math.vec3(snap_x[0], bot_y, snap_z[0])
                                            floor_vertices[bb + 1] = ti.math.vec3(snap_x[3], bot_y, snap_z[3])
                                            floor_vertices[bb + 2] = ti.math.vec3(snap_x[3], top_y, snap_z[3])
                                            floor_vertices[bb + 3] = ti.math.vec3(snap_x[0], top_y, snap_z[0])
                                            for bv in ti.static(range(4)):
                                                floor_normals[bb + bv] = bevel_norm
                                                floor_colors[bb + bv] = dark_color
                                    if not is_floor_px:
                                        bi = ti.atomic_add(num_floor_quads[None], 1)
                                        if bi < MAX_FLOOR_QUADS:
                                            bb = bi * 4
                                            floor_vertices[bb + 0] = ti.math.vec3(snap_x[2], bot_y, snap_z[2])
                                            floor_vertices[bb + 1] = ti.math.vec3(snap_x[1], bot_y, snap_z[1])
                                            floor_vertices[bb + 2] = ti.math.vec3(snap_x[1], top_y, snap_z[1])
                                            floor_vertices[bb + 3] = ti.math.vec3(snap_x[2], top_y, snap_z[2])
                                            for bv in ti.static(range(4)):
                                                floor_normals[bb + bv] = bevel_norm
                                                floor_colors[bb + bv] = dark_color
                                    if not is_floor_mz:
                                        bi = ti.atomic_add(num_floor_quads[None], 1)
                                        if bi < MAX_FLOOR_QUADS:
                                            bb = bi * 4
                                            floor_vertices[bb + 0] = ti.math.vec3(snap_x[1], bot_y, snap_z[1])
                                            floor_vertices[bb + 1] = ti.math.vec3(snap_x[0], bot_y, snap_z[0])
                                            floor_vertices[bb + 2] = ti.math.vec3(snap_x[0], top_y, snap_z[0])
                                            floor_vertices[bb + 3] = ti.math.vec3(snap_x[1], top_y, snap_z[1])
                                            for bv in ti.static(range(4)):
                                                floor_normals[bb + bv] = bevel_norm
                                                floor_colors[bb + bv] = dark_color
                                    if not is_floor_pz:
                                        bi = ti.atomic_add(num_floor_quads[None], 1)
                                        if bi < MAX_FLOOR_QUADS:
                                            bb = bi * 4
                                            floor_vertices[bb + 0] = ti.math.vec3(snap_x[3], bot_y, snap_z[3])
                                            floor_vertices[bb + 1] = ti.math.vec3(snap_x[2], bot_y, snap_z[2])
                                            floor_vertices[bb + 2] = ti.math.vec3(snap_x[2], top_y, snap_z[2])
                                            floor_vertices[bb + 3] = ti.math.vec3(snap_x[3], top_y, snap_z[3])
                                            for bv in ti.static(range(4)):
                                                floor_normals[bb + bv] = bevel_norm
                                                floor_colors[bb + bv] = dark_color
                    else:
                        # Non-circle arena: original edge/interior logic
                        is_edge = 0
                        for di, dk in ti.static([(-1, 0), (1, 0), (0, -1), (0, 1)]):
                            ntype = voxel_field[i + di, j, k + dk]
                            if ntype != CONCRETE and ntype != SLIPPERY:
                                is_edge = 1
                        if is_edge:
                            # Edge → sphere fallback
                            idx = ti.atomic_add(num_voxels[None], 1)
                            if idx < MAX_VOXELS:
                                voxel_positions[idx] = ti.math.vec3(world_x, world_y, world_z)
                                voxel_colors[idx] = color
                                voxel_radii[idx] = 0.47
                        else:
                            # Check diagonals — fully interior skipped (merge kernel handles)
                            all_diag_nonsdf = 1
                            for di, dk in ti.static([(-1, -1), (1, -1), (1, 1), (-1, 1)]):
                                ntype2 = voxel_field[i + di, j, k + dk]
                                if ntype2 != CONCRETE and ntype2 != SLIPPERY:
                                    all_diag_nonsdf = 0
                            if all_diag_nonsdf:
                                pass  # Fully interior — merge kernel handles
                            else:
                                # Partial interior → flat mesh quad
                                qi = ti.atomic_add(num_floor_quads[None], 1)
                                if qi < MAX_FLOOR_QUADS:
                                    base = qi * 4
                                    top_y = world_y + VOXEL_RADIUS
                                    floor_vertices[base + 0] = ti.math.vec3(world_x - HALF, top_y, world_z - HALF)
                                    floor_vertices[base + 1] = ti.math.vec3(world_x + HALF, top_y, world_z - HALF)
                                    floor_vertices[base + 2] = ti.math.vec3(world_x + HALF, top_y, world_z + HALF)
                                    floor_vertices[base + 3] = ti.math.vec3(world_x - HALF, top_y, world_z + HALF)
                                    up = ti.math.vec3(0.0, 1.0, 0.0)
                                    floor_normals[base + 0] = up
                                    floor_normals[base + 1] = up
                                    floor_normals[base + 2] = up
                                    floor_normals[base + 3] = up
                                    for v in ti.static(range(4)):
                                        ci = i + (1 if v == 1 or v == 2 else 0)
                                        ck = k + (1 if v == 2 or v == 3 else 0)
                                        floor_colors[base + v] = stone_texture_color(color, ci, ck)
                else:
                    # Old style: all floor as spheres
                    idx = ti.atomic_add(num_voxels[None], 1)
                    if idx < MAX_VOXELS:
                        voxel_positions[idx] = ti.math.vec3(world_x, world_y, world_z)
                        voxel_colors[idx] = color
                        voxel_radii[idx] = VOXEL_RADIUS
            else:
                # Non-floor voxels → particle buffer (spheres)
                idx = ti.atomic_add(num_voxels[None], 1)
                if idx < MAX_VOXELS:
                    voxel_positions[idx] = ti.math.vec3(world_x, world_y, world_z)
                    voxel_colors[idx] = color
                    if vtype == 23 or vtype == 24:  # SCORE_DIGIT_BLUE or SCORE_DIGIT_RED
                        voxel_radii[idx] = VOXEL_RADIUS * 0.72
                    else:
                        voxel_radii[idx] = VOXEL_RADIUS

@ti.kernel
def extract_particles():
    """Extract debris, spray, silk, projectiles, background into render buffer."""
    # ===== PHASE 2: Extract debris particles =====
    debris_count = simulation.num_debris[None]
    debris_check = ti.min(debris_count, MAX_DEBRIS_CHECK)

    for idx in range(debris_check):
        if simulation.debris_active[idx] == 0:
            continue

        write_idx = ti.atomic_add(num_voxels[None], 1)
        if write_idx < MAX_VOXELS:
            voxel_positions[write_idx] = simulation.debris_pos[idx]
            base_color = simulation.debris_material[idx]
            lifetime = simulation.debris_lifetime[idx]

            # Per-particle radius: use custom if set, otherwise default
            base_radius = simulation.debris_radius[idx]
            if base_radius < 0.01:
                base_radius = DEBRIS_RADIUS

            if lifetime < 0.4:
                t = lifetime / 0.4
                alpha = ti.max(t * t, 0.0)
                fade_target = base_color * 0.3 + ti.math.vec3(0.7, 0.7, 0.7)
                voxel_colors[write_idx] = base_color * alpha + fade_target * (1.0 - alpha)
                voxel_radii[write_idx] = base_radius * (0.3 + 0.7 * t)
            else:
                voxel_colors[write_idx] = base_color
                voxel_radii[write_idx] = base_radius

    # ===== PHASE 3: Extract spray particles =====
    spray_count = simulation.num_spray[None]
    spray_check = ti.min(spray_count, MAX_SPRAY_CHECK)

    if spray_check > 0:
        for idx in range(spray_check):
            if simulation.spray_active[idx] == 0:
                continue

            write_idx = ti.atomic_add(num_voxels[None], 1)
            if write_idx < MAX_VOXELS:
                voxel_positions[write_idx] = simulation.spray_pos[idx]
                base_color = simulation.spray_color[idx]
                lifetime = simulation.spray_lifetime[idx]

                is_venom = base_color[0] > base_color[1]
                alpha = 1.0
                if lifetime < 0.3:
                    alpha = lifetime / 0.3

                glow = 1.0
                if is_venom:
                    glow = 1.3 + 0.4 * ti.sin(lifetime * 20.0)

                voxel_colors[write_idx] = base_color * alpha * glow
                voxel_radii[write_idx] = DEBRIS_RADIUS

    # ===== PHASE 4: Extract silk particles =====
    SILK_FADE_TIME = ti.static(2.0)
    SILK_EMISSIVE_VAL = silk_emissive[None]
    SILK_RADIUS = ti.static(DEBRIS_RADIUS * 1.44)

    silk_count = simulation.num_silk[None]
    silk_check = ti.min(silk_count, MAX_SILK_CHECK)

    if silk_check > 0:
        for idx in range(silk_check):
            if simulation.silk_active[idx] == 0:
                continue

            lifetime = simulation.silk_lifetime[idx]
            write_idx = ti.atomic_add(num_voxels[None], 1)
            if write_idx < MAX_VOXELS:
                voxel_positions[write_idx] = simulation.silk_pos[idx]
                base_color = simulation.silk_color[idx]

                pulse = 1.0
                if simulation.silk_stuck[idx] >= 1:
                    pulse = 1.0 + 0.08 * ti.sin(lifetime * 12.0)

                voxel_colors[write_idx] = base_color * pulse * SILK_EMISSIVE_VAL

                if lifetime < SILK_FADE_TIME:
                    t = lifetime / SILK_FADE_TIME
                    voxel_radii[write_idx] = SILK_RADIUS * t
                else:
                    voxel_radii[write_idx] = SILK_RADIUS

    # ===== PHASE 5: Extract projectiles =====
    PROJECTILE_RADIUS = ti.static(0.8)

    for idx in range(simulation.MAX_PROJECTILES):
        if simulation.projectile_active[idx] == 1:
            write_idx = ti.atomic_add(num_voxels[None], 1)
            if write_idx < MAX_VOXELS:
                voxel_positions[write_idx] = simulation.projectile_pos[idx]
                voxel_colors[write_idx] = ti.math.vec3(1.0, 1.0, 0.0)
                voxel_radii[write_idx] = PROJECTILE_RADIUS

    # ===== PHASE 6: Extract background voxels from cache =====
    if simulation.bg_theme_active[None] > 0:
        bg_vis = simulation.num_visible_bg[None]
        for idx in range(bg_vis):
            write_idx = ti.atomic_add(num_voxels[None], 1)
            if write_idx < MAX_VOXELS:
                voxel_positions[write_idx] = simulation.bg_cache_positions[idx]
                voxel_colors[write_idx] = simulation.bg_cache_colors[idx]
                voxel_radii[write_idx] = simulation.bg_cache_radii[idx]

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

        # OPTIMIZATION: Cached key light position (only recalculate on camera rotation)
        self.cached_yaw = None
        self.cached_key_light_pos = None

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

def render(camera, canvas, scene, voxel_field, n_grid, dynamic_lighting=True, spotlight_pos=None, spotlight_strength=0.55, base_light_brightness=1.0, front_light_strength=0.5, floor_y=1.0):
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
        front_light_strength: DEPRECATED - no longer used (kept for API compatibility)
    """
    import math
    import time

    # === RENDER TIMING (for performance analysis) ===
    _t0 = time.perf_counter()

    # OPTIMIZATION: Gradient background removed - each GPU call costs ~15 FPS on integrated GPUs
    # Background color now comes from ambient light + scene clearing
    _t1 = time.perf_counter()

    # Extract voxels (arena/beetles) and particles (debris/spray/silk/projectiles/bg)
    # Split into 2 kernels for faster compilation (~60s each vs ~120s monolithic)
    global floor_cache_valid, cached_floor_count
    num_voxels[None] = 0  # Reset counter
    use_mesh = 1 if mesh_floor_enabled else 0
    skip_floor = 0
    if use_mesh and floor_cache_valid and not ice_active and not hole_active:
        # Floor mesh cached — skip floor extraction, reuse cached floor fields
        skip_floor = 1
    else:
        num_floor_quads[None] = 0  # Reset floor mesh counter (rebuilding)
    extract_voxels(voxel_field, n_grid, use_mesh, skip_floor)
    extract_particles()
    _t_merge0 = time.perf_counter()
    if use_mesh and not skip_floor:
        merge_interior_floor(voxel_field, n_grid, 33)
    _t_merge1 = time.perf_counter()
    if use_mesh and not floor_cache_valid:
        # Cache the freshly-built floor mesh count (fill + edge + bevel + merged quads)
        cached_floor_count = num_floor_quads[None]
        floor_cache_valid = True

    _t2 = time.perf_counter()

    # Set up camera
    setup_camera(camera, scene)

    # OPTIMIZED LIGHTING: Reduced from 5-6 point_light calls to 2-3
    # Each point_light() call has ~15 FPS overhead on integrated GPUs
    # Removed: fill light, front light. Boosted: ambient for even coverage
    b = base_light_brightness

    # Main overhead light - centered above arena for even coverage
    scene.point_light(pos=(0, 120, 0), color=(0.66 * b, 0.65 * b, 0.62 * b))

    # Secondary light - slight offset for depth, but mostly overhead to avoid uneven sides
    if dynamic_lighting:
        if camera.yaw != camera.cached_yaw:
            # Gentle offset from camera angle (reduced from 100 to 30 distance, higher up)
            key_angle = math.radians(camera.yaw) + math.radians(30)
            key_x = math.cos(key_angle) * 30
            key_z = math.sin(key_angle) * 30
            camera.cached_key_light_pos = (key_x, 110, key_z)
            camera.cached_yaw = camera.yaw

        scene.point_light(pos=camera.cached_key_light_pos, color=(0.36 * b, 0.34 * b, 0.30 * b))
    else:
        scene.point_light(pos=(20, 110, -20), color=(0.36 * b, 0.34 * b, 0.30 * b))

    # Spotlight above beetles - follows the action (conditional, only when needed)
    if spotlight_pos is not None:
        spot_x, spot_y, spot_z = spotlight_pos
        scene.point_light(pos=(spot_x, spot_y, spot_z), color=(spotlight_strength * 1.15, spotlight_strength, spotlight_strength * 0.85))

    # Ambient light for even base illumination
    scene.ambient_light((0.26 * b, 0.26 * b, 0.29 * b))

    _t5 = time.perf_counter()

    # Render non-floor particles as spheres
    # (beetles, steel, goals, debris, spray, silk, projectiles, etc.)
    count = num_voxels[None]
    _t_particles0 = time.perf_counter()
    if count > 0:
        scene.particles(
            voxel_positions,
            radius=VOXEL_RADIUS,  # Fallback radius (per_vertex_radius overrides this)
            per_vertex_color=voxel_colors,
            per_vertex_radius=voxel_radii,  # Mixed sizes: 0.37 voxels, 0.25 debris/spray/silk, 0.8 projectiles
            index_count=count
        )
    _t_particles1 = time.perf_counter()

    # Mesh floor quads (only when mesh floor enabled)
    floor_count = 0
    _t_mesh0 = time.perf_counter()
    if mesh_floor_enabled:
        floor_count = cached_floor_count if floor_cache_valid else num_floor_quads[None]
        if floor_count > 0:
            scene.mesh(floor_vertices, indices=_floor_indices_np, normals=floor_normals,
                       per_vertex_color=floor_colors, two_sided=False,
                       vertex_count=floor_count * 4, index_count=floor_count * 6)
    _t_mesh1 = time.perf_counter()

    # Shadow discs (always — works on both mesh and sphere floors)
    disc_count = num_shadow_discs[None]
    _t_shadow0 = time.perf_counter()
    if disc_count > 0:
        build_shadow_discs(float(floor_y), voxel_field, n_grid, use_mesh)
        scene.mesh(shadow_vertices, indices=_shadow_indices_np, normals=shadow_normals,
                   per_vertex_color=shadow_colors, two_sided=False,
                   vertex_count=disc_count * SHADOW_VERTS_PER_DISC,
                   index_count=disc_count * SHADOW_TRIS_PER_DISC * 3)
    _t_shadow1 = time.perf_counter()

    if (_t_merge1 - _t_merge0) > 1.0 or (_t_particles1 - _t_particles0) > 1.0 or (_t_mesh1 - _t_mesh0) > 1.0 or (_t_shadow1 - _t_shadow0) > 1.0:
        print(f"[Timing] render() breakdown: merge={(_t_merge1 - _t_merge0):.2f}s particles={(_t_particles1 - _t_particles0):.2f}s mesh={(_t_mesh1 - _t_mesh0):.2f}s shadow={(_t_shadow1 - _t_shadow0):.2f}s")

    _t6 = time.perf_counter()

    # === STORE RENDER TIMING FOR ANALYSIS ===
    # Store timing breakdown in module-level dict for access from main loop
    global _render_timing
    _render_timing = {
        'extract_all': (_t2 - _t1) * 1000,  # Megakernel: voxels + debris + spray + silk + projectiles
        'lighting_setup': (_t5 - _t2) * 1000,
        'scene_draw': (_t6 - _t5) * 1000,  # particles + mesh
        'voxel_count': count,
        'floor_quads': floor_count,
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
