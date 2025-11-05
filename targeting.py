import taichi as ti
import math

# Taichi fields for targeted voxel
targeted_voxel = ti.Vector.field(3, dtype=ti.i32, shape=())  # (x, y, z) grid coordinates
has_target = ti.field(dtype=ti.i32, shape=())  # 0 or 1
target_distance = ti.field(dtype=ti.f32, shape=())  # Distance to targeted voxel

# Visualization
target_marker = ti.Vector.field(3, dtype=ti.f32, shape=1)  # World position for rendering

@ti.kernel
def raycast_voxel(
    origin_x: ti.f32, origin_y: ti.f32, origin_z: ti.f32,
    dir_x: ti.f32, dir_y: ti.f32, dir_z: ti.f32,
    voxel_field: ti.template(),
    n_grid: ti.i32,
    max_distance: ti.f32
):
    """
    Cast ray from camera through voxel grid to find first non-empty voxel

    Uses DDA (Digital Differential Analyzer) algorithm for efficient voxel traversal

    Args:
        origin: Ray start position (camera position)
        dir: Ray direction (normalized forward vector)
        voxel_field: Voxel grid to test
        n_grid: Grid size
        max_distance: Maximum raycast distance
    """
    # Reset target
    has_target[None] = 0
    target_distance[None] = max_distance

    # Normalize direction (ensure it's unit length)
    dir_length = ti.sqrt(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z)

    dx = 0.0
    dy = 0.0
    dz = 0.0

    if dir_length >= 0.0001:
        dx = dir_x / dir_length
        dy = dir_y / dir_length
        dz = dir_z / dir_length

        # DDA raycast - step through voxel grid
        # Start position in voxel grid coordinates
        # Remember: voxels are centered around origin, so we need to transform
        # World position to grid position: grid_pos = world_pos + (n_grid / 2)

        current_x = origin_x
        current_y = origin_y
        current_z = origin_z

        # Step size (how far to move along ray each iteration)
        step_size = 0.5  # Half a voxel

        # Maximum steps (prevents infinite loop)
        max_steps = int(max_distance / step_size)

        for step in range(max_steps):
            # Convert world position to grid coordinates
            grid_x = int(current_x + n_grid / 2.0)
            grid_y = int(current_y)
            grid_z = int(current_z + n_grid / 2.0)

            # Track whether we found a hit this step
            found_hit = 0

            # Check if in bounds
            if grid_x >= 0 and grid_x < n_grid and grid_y >= 0 and grid_y < n_grid and grid_z >= 0 and grid_z < n_grid:
                # Check if voxel is non-empty
                if voxel_field[grid_x, grid_y, grid_z] != 0:
                    # Found target!
                    targeted_voxel[None] = ti.Vector([grid_x, grid_y, grid_z])
                    has_target[None] = 1

                    # Calculate distance
                    dist_x = current_x - origin_x
                    dist_y = current_y - origin_y
                    dist_z = current_z - origin_z
                    target_distance[None] = ti.sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z)

                    # Set marker for rendering (convert back to world coordinates)
                    target_marker[0] = ti.Vector([
                        float(grid_x) - n_grid / 2.0,
                        float(grid_y),
                        float(grid_z) - n_grid / 2.0
                    ])

                    found_hit = 1

            # If we found a hit, exit loop
            if found_hit == 1:
                break

            # Step forward along ray
            current_x += dx * step_size
            current_y += dy * step_size
            current_z += dz * step_size

def update_targeting(camera, voxel_field, n_grid, max_distance=200.0):
    """
    Update targeting system - cast ray from camera to find targeted voxel

    Args:
        camera: Camera object with position and forward vector
        voxel_field: Voxel grid
        n_grid: Grid size
        max_distance: Maximum targeting distance
    """
    # Get camera position and forward direction
    pos = camera.get_position()
    forward = camera.get_forward_vector()

    # Cast ray
    raycast_voxel(
        pos[0], pos[1], pos[2],
        forward[0], forward[1], forward[2],
        voxel_field,
        n_grid,
        max_distance
    )

def get_target_info():
    """
    Get information about currently targeted voxel

    Returns:
        (has_target, grid_coords, distance) tuple
        has_target: True if voxel is targeted
        grid_coords: (x, y, z) grid coordinates
        distance: Distance to target in world units
    """
    if has_target[None] == 1:
        coords = targeted_voxel[None]
        return (True, (coords[0], coords[1], coords[2]), target_distance[None])
    else:
        return (False, None, 0.0)

def render_target_marker(scene):
    """
    Render targeting reticle on targeted voxel

    Args:
        scene: Taichi scene
    """
    if has_target[None] == 1:
        # Render bright crosshair marker on targeted voxel
        scene.particles(
            target_marker,
            radius=1.5,  # Larger than voxel to stand out
            color=(0.0, 1.0, 0.0),  # Bright green
            index_count=1
        )
