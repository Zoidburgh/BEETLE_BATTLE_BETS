import taichi as ti

# Initialize Taichi with Vulkan (skip CUDA attempt for faster startup)
ti.init(arch=ti.vulkan, debug=False)

# 128x128x128 grid - optimal power-of-2 size for beetle battle (GPU cache friendly)
n_grid = 128
voxel_type = ti.field(dtype=ti.i32, shape=(n_grid, n_grid, n_grid))

# Debris particle system (flying particles from destroyed voxels)
MAX_DEBRIS = 20000  # Pre-allocated pool for performance
num_debris = ti.field(dtype=ti.i32, shape=())  # Active particle count
debris_pos = ti.Vector.field(3, dtype=ti.f32, shape=MAX_DEBRIS)
debris_vel = ti.Vector.field(3, dtype=ti.f32, shape=MAX_DEBRIS)
debris_material = ti.field(dtype=ti.i32, shape=MAX_DEBRIS)  # Original voxel type
debris_lifetime = ti.field(dtype=ti.f32, shape=MAX_DEBRIS)  # Time alive (seconds)

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

# Customizable beetle colors (RGB values in range 0.0-1.0)
# Blue beetle colors
blue_body_color = ti.Vector.field(3, dtype=ti.f32, shape=())
blue_leg_color = ti.Vector.field(3, dtype=ti.f32, shape=())
blue_leg_tip_color = ti.Vector.field(3, dtype=ti.f32, shape=())
blue_stripe_color = ti.Vector.field(3, dtype=ti.f32, shape=())
blue_horn_tip_color = ti.Vector.field(3, dtype=ti.f32, shape=())

# Red beetle colors
red_body_color = ti.Vector.field(3, dtype=ti.f32, shape=())
red_leg_color = ti.Vector.field(3, dtype=ti.f32, shape=())
red_leg_tip_color = ti.Vector.field(3, dtype=ti.f32, shape=())
red_stripe_color = ti.Vector.field(3, dtype=ti.f32, shape=())
red_horn_tip_color = ti.Vector.field(3, dtype=ti.f32, shape=())

# Initialize default colors
blue_body_color[None] = ti.Vector([0.25, 0.55, 0.95])  # Desaturated blue
blue_leg_color[None] = ti.Vector([0.4, 0.7, 1.0])  # Lighter cyan/blue
blue_leg_tip_color[None] = ti.Vector([0.0, 0.0, 0.3])  # Very dark blue
blue_stripe_color[None] = ti.Vector([0.6, 0.9, 1.0])  # Bright cyan
blue_horn_tip_color[None] = ti.Vector([0.4, 0.75, 1.0])  # Bright electric blue

red_body_color[None] = ti.Vector([0.95, 0.25, 0.15])  # Desaturated red
red_leg_color[None] = ti.Vector([1.0, 0.5, 0.3])  # Lighter orange/red
red_leg_tip_color[None] = ti.Vector([0.3, 0.0, 0.0])  # Very dark red
red_stripe_color[None] = ti.Vector([0.85, 0.65, 0.2])  # Rich gold/bronze
red_horn_tip_color[None] = ti.Vector([0.4, 0.1, 0.1])  # Deep crimson

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

# Initialize voxel sphere on module load
print(f"Initializing voxel grid: {n_grid}x{n_grid}x{n_grid}")
init_beetle_arena()  # Changed to beetle arena
print("Beetle Battle Arena ready!")
