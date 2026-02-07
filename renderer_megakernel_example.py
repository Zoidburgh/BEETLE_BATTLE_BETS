# EXAMPLE: Megakernel pattern - merge all 5 extract kernels into ONE
# This reduces kernel launch overhead from 5 calls to 1 call per frame

@ti.kernel
def extract_all_particles(voxel_field: ti.template(), n_grid: ti.i32):
    """MEGAKERNEL: Extract all renderable geometry in ONE kernel launch

    Combines:
    - extract_voxels()
    - extract_debris_particles()
    - extract_spray_particles()
    - extract_silk_particles()
    - extract_projectiles()

    Expected speedup: 4 × kernel_launch_overhead (~4-8ms on integrated GPUs)
    """
    count = 0

    # === PART 1: Extract voxels (original extract_voxels logic) ===
    EMPTY = ti.static(0)
    DEBRIS = ti.static(4)

    for i, j, k in ti.ndrange((2, 126), (1, 100), (2, 126)):
        vtype = voxel_field[i, j, k]
        if vtype != EMPTY and vtype != DEBRIS:
            world_pos = ti.math.vec3(
                float(i) - n_grid / 2.0,
                float(j),
                float(k) - n_grid / 2.0
            )
            color = get_voxel_color(vtype, world_pos.x, world_pos.z)

            idx = ti.atomic_add(count, 1)
            if idx < MAX_VOXELS:
                voxel_positions[idx] = world_pos
                voxel_colors[idx] = color
                if vtype == 23 or vtype == 24:  # Score digits
                    voxel_radii[idx] = VOXEL_RADIUS * 0.72
                else:
                    voxel_radii[idx] = VOXEL_RADIUS

    # === PART 2: Extract debris particles (original extract_debris_particles logic) ===
    debris_count = simulation.num_debris[None]
    check_count = ti.min(debris_count, MAX_DEBRIS_CHECK)

    for idx in range(check_count):
        if simulation.debris_active[idx] == 0:
            continue

        write_idx = ti.atomic_add(count, 1)
        if write_idx < MAX_VOXELS:
            debris_pos = simulation.debris_pos[idx]
            voxel_positions[write_idx] = debris_pos

            base_color = simulation.debris_material[idx]
            lifetime = simulation.debris_lifetime[idx]

            if lifetime < 0.4:
                t = lifetime / 0.4
                alpha = t * t
                alpha = ti.max(alpha, 0.0)
                fade_target = base_color * 0.3 + ti.math.vec3(0.7, 0.7, 0.7)
                voxel_colors[write_idx] = base_color * alpha + fade_target * (1.0 - alpha)
                voxel_radii[write_idx] = DEBRIS_RADIUS * (0.3 + 0.7 * t)
            else:
                voxel_colors[write_idx] = base_color
                voxel_radii[write_idx] = DEBRIS_RADIUS

    # === PART 3: Extract spray particles (original extract_spray_particles logic) ===
    spray_count = simulation.num_spray[None]
    check_count = ti.min(spray_count, MAX_SPRAY_CHECK)

    for idx in range(check_count):
        if simulation.spray_active[idx] == 0:
            continue

        write_idx = ti.atomic_add(count, 1)
        if write_idx < MAX_VOXELS:
            spray_pos = simulation.spray_pos[idx]
            voxel_positions[write_idx] = spray_pos

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

    # === PART 4: Extract silk particles (original extract_silk_particles logic) ===
    silk_count = simulation.num_silk[None]
    SILK_FADE_TIME = 2.0
    check_count = ti.min(silk_count, MAX_SILK_CHECK)

    for idx in range(check_count):
        if simulation.silk_active[idx] == 0:
            continue

        lifetime = simulation.silk_lifetime[idx]

        write_idx = ti.atomic_add(count, 1)
        if write_idx < MAX_VOXELS:
            silk_pos = simulation.silk_pos[idx]
            voxel_positions[write_idx] = silk_pos

            base_color = simulation.silk_color[idx]

            alpha = 1.0
            if lifetime < SILK_FADE_TIME:
                t = lifetime / SILK_FADE_TIME
                alpha = t * t

            pulse = 1.0
            if simulation.silk_stuck[idx] >= 1:
                pulse = 1.15 + 0.35 * ti.sin(lifetime * 12.0)

            SILK_EMISSIVE = 1.7
            voxel_colors[write_idx] = base_color * alpha * pulse * SILK_EMISSIVE

            SILK_RADIUS = DEBRIS_RADIUS * 1.44
            if lifetime < SILK_FADE_TIME:
                voxel_radii[write_idx] = SILK_RADIUS * (0.5 + 0.5 * (lifetime / SILK_FADE_TIME))
            else:
                voxel_radii[write_idx] = SILK_RADIUS

    # === PART 5: Extract projectiles (original extract_projectiles logic) ===
    PROJECTILE_RADIUS = 0.8

    for idx in range(simulation.MAX_PROJECTILES):
        if simulation.projectile_active[idx] == 1:
            write_idx = ti.atomic_add(count, 1)
            if write_idx < MAX_VOXELS:
                voxel_positions[write_idx] = simulation.projectile_pos[idx]
                voxel_colors[write_idx] = ti.math.vec3(1.0, 1.0, 0.0)
                voxel_radii[write_idx] = PROJECTILE_RADIUS

    # Store final count
    num_voxels[None] = ti.min(count, MAX_VOXELS)


# USAGE in render() function:
def render(...):
    # OLD (5 kernel launches):
    # num_voxels[None] = 0
    # extract_voxels(voxel_field, n_grid)
    # extract_debris_particles()
    # extract_spray_particles()
    # extract_silk_particles()
    # extract_projectiles()

    # NEW (1 kernel launch):
    num_voxels[None] = 0
    extract_all_particles(voxel_field, n_grid)

    # Rest of render code unchanged...
