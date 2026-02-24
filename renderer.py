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

# Floor mesh fields (flat quads instead of spheres — cleaner visuals, fewer raycasts)
MAX_FLOOR_QUADS = 5000
MAX_FLOOR_VERTS = MAX_FLOOR_QUADS * 4

num_floor_quads = ti.field(dtype=ti.i32, shape=())
floor_vertices = ti.Vector.field(3, dtype=ti.f32, shape=MAX_FLOOR_VERTS)
floor_normals = ti.Vector.field(3, dtype=ti.f32, shape=MAX_FLOOR_VERTS)
floor_colors = ti.Vector.field(3, dtype=ti.f32, shape=MAX_FLOOR_VERTS)

# Pre-computed numpy index array (static quad pattern) — avoids GPU-CPU sync
_floor_indices_np = np.zeros(MAX_FLOOR_QUADS * 6, dtype=np.int32)
for _q in range(MAX_FLOOR_QUADS):
    _bv, _bi = _q * 4, _q * 6
    _floor_indices_np[_bi:_bi+6] = [_bv, _bv+1, _bv+2, _bv, _bv+2, _bv+3]

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

# Floor rendering toggle (mesh quads vs old sphere voxels)
mesh_floor_enabled = False

# Projectiles (cannonballs) are now merged into main voxel buffer with larger radius
# This eliminates a separate scene.particles() call, reducing CPU->GPU sync overhead

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

    # Concrete - customizable arena floor color (with ice overlay check)
    elif voxel_type == 2:  # CONCRETE
        gi = int(world_x + 64.0)
        gk = int(world_z + 64.0)
        if 0 <= gi < 128 and 0 <= gk < 128 and simulation.ice_overlay[gi, gk] == 1:
            color = ti.math.vec3(0.65, 0.78, 0.92)  # Icy blue-white
        else:
            color = simulation.board_color[None]

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

    # Shadow blob beneath airborne beetles (darker than arena floor)
    elif voxel_type == 20:  # SHADOW
        color = ti.math.vec3(0.13, 0.12, 0.11)  # ~60% of arena floor color

    # Slippery bowl perimeter (slightly blue-tinted to indicate slippery)
    elif voxel_type == 21:  # SLIPPERY
        gi = int(world_x + 64.0)
        gk = int(world_z + 64.0)
        if 0 <= gi < 128 and 0 <= gk < 128 and simulation.ice_overlay[gi, gk] == 1:
            color = ti.math.vec3(0.65, 0.78, 0.92)  # Icy blue-white (same as concrete ice)
        else:
            color = ti.math.vec3(0.35, 0.40, 0.50)  # Blue-gray to indicate slippery ice-like surface

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

@ti.kernel
def build_shadow_discs(floor_y: ti.f32, voxel_field: ti.template(), n_grid: ti.i32):
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
        top_y = floor_y + VOXEL_RADIUS + 0.06  # Offset above floor to prevent z-fight with floor quads
        shadow_color = ti.math.vec3(0.13, 0.12, 0.11)
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

        # Center vertex
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
            shadow_colors[base + 1 + s] = shadow_color

def set_shadow_params(index, x, z, radius):
    """Set shadow disc position/radius (called from beetle_physics)"""
    shadow_disc_params[index][0] = x
    shadow_disc_params[index][1] = z
    shadow_disc_params[index][2] = radius

def set_num_shadows(count):
    """Set number of active shadow discs (called from beetle_physics)"""
    num_shadow_discs[None] = count

@ti.kernel
def extract_all_particles(voxel_field: ti.template(), n_grid: ti.i32, use_mesh_floor: ti.i32):
    """MEGAKERNEL: Extract all voxels and particles into render buffer in single kernel launch.

    Combines 5 separate kernels into 1 to reduce Python→GPU launch overhead.
    Each kernel launch costs ~2-4ms on integrated GPUs, so this saves ~8-16ms per frame.

    Extracts: arena voxels, debris, spray, silk, projectiles
    """
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
                if use_mesh_floor:
                    # Check if interior (all 4 cardinal neighbors are floor)
                    is_edge = 0
                    for di, dk in ti.static([(-1, 0), (1, 0), (0, -1), (0, 1)]):
                        ntype = voxel_field[i + di, j, k + dk]
                        if ntype != CONCRETE and ntype != SLIPPERY:
                            is_edge = 1

                    if is_edge:
                        # Edge floor → sphere (smooth boundary)
                        idx = ti.atomic_add(num_voxels[None], 1)
                        if idx < MAX_VOXELS:
                            voxel_positions[idx] = ti.math.vec3(world_x, world_y, world_z)
                            voxel_colors[idx] = color
                            voxel_radii[idx] = 0.47
                    else:
                        # Interior floor → flat mesh quad
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
                            floor_colors[base + 0] = color
                            floor_colors[base + 1] = color
                            floor_colors[base + 2] = color
                            floor_colors[base + 3] = color
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
    SILK_EMISSIVE = ti.static(1.7)
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

                alpha = 1.0
                if lifetime < SILK_FADE_TIME:
                    t = lifetime / SILK_FADE_TIME
                    alpha = t * t

                pulse = 1.0
                if simulation.silk_stuck[idx] >= 1:
                    pulse = 1.15 + 0.35 * ti.sin(lifetime * 12.0)

                voxel_colors[write_idx] = base_color * alpha * pulse * SILK_EMISSIVE

                if lifetime < SILK_FADE_TIME:
                    voxel_radii[write_idx] = SILK_RADIUS * (0.5 + 0.5 * (lifetime / SILK_FADE_TIME))
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

    # MEGAKERNEL: Extract all voxels and particles in single kernel launch
    # Combines 5 kernels into 1 to reduce Python→GPU launch overhead (~8-16ms savings)
    num_voxels[None] = 0  # Reset counter
    num_floor_quads[None] = 0  # Reset floor mesh counter
    use_mesh = 1 if mesh_floor_enabled else 0
    extract_all_particles(voxel_field, n_grid, use_mesh)

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
    if count > 0:
        scene.particles(
            voxel_positions,
            radius=VOXEL_RADIUS,  # Fallback radius (per_vertex_radius overrides this)
            per_vertex_color=voxel_colors,
            per_vertex_radius=voxel_radii,  # Mixed sizes: 0.37 voxels, 0.25 debris/spray/silk, 0.8 projectiles
            index_count=count
        )

    # Mesh floor quads (only when mesh floor enabled)
    floor_count = 0
    if mesh_floor_enabled:
        floor_count = num_floor_quads[None]
        if floor_count > 0:
            scene.mesh(floor_vertices, indices=_floor_indices_np, normals=floor_normals,
                       per_vertex_color=floor_colors, two_sided=False,
                       vertex_count=floor_count * 4, index_count=floor_count * 6)

    # Shadow discs (always — works on both mesh and sphere floors)
    disc_count = num_shadow_discs[None]
    if disc_count > 0:
        build_shadow_discs(float(floor_y), voxel_field, n_grid)
        scene.mesh(shadow_vertices, indices=_shadow_indices_np, normals=shadow_normals,
                   per_vertex_color=shadow_colors, two_sided=False,
                   vertex_count=disc_count * SHADOW_VERTS_PER_DISC,
                   index_count=disc_count * SHADOW_TRIS_PER_DISC * 3)

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
