import open3d as o3d
import heapq
import numpy as np
import time

mesh = o3d.io.read_triangle_mesh("dock.stl")
mesh.translate([431.8, 0, 0])
mesh.scale(0.001, center=[0, 0, 0])  # Convert mm to meters
dock_voxel_grid = o3d.io.read_voxel_grid("dock_voxel_gapped.ply")
# voxel_indeces = []
# for voxel in dock_voxel_grid.get_voxels():
#     voxel_indeces.append(tuple(voxel.grid_index))

# for index in voxel_indeces:
#     for dx in range(-PADDING_CELLS, PADDING_CELLS):
#         for dy in range(-PADDING_CELLS, PADDING_CELLS):
#             for dz in range(-PADDING_CELLS, PADDING_CELLS):
#                 print(time.monotonic())
#                 dock_voxel_grid.add_voxel(o3d.geometry.Voxel([index[0] + dx, index[1] + dy, index[2] + dz]))
                
voxel_indeces = {}
for voxel in dock_voxel_grid.get_voxels():
    voxel_indeces[tuple(voxel.grid_index)] = None

start_position = tuple(dock_voxel_grid.get_voxel([0, 0, 0]))
goal_position = tuple(dock_voxel_grid.get_voxel([10, 3, 3]))

print('hashed!')

def get_neighbors(pos):
    step_size = int(euclidean(pos, goal_position) / 100) + 1
    print(euclidean(pos, goal_position))
    directions = [
        (step_size, 0, 0), (-step_size, 0, 0),
        (0, step_size, 0), (0, -step_size, 0),
        (0, 0, step_size), (0, 0, -step_size)
    ]
    neighbors = []
    for d in directions:
        neighbor = (pos[0] + d[0], pos[1] + d[1], pos[2] + d[2])
        if neighbor not in voxel_indeces:
            neighbors.append(neighbor)
    return neighbors

def line_of_sight(a, b):
	a = np.array(a)
	b = np.array(b)
	diff = b - a
	n = np.max(np.abs(diff))
	if n == 0:
		return True
	for i in range(1, n+1):
		interp = tuple(np.round(a + diff * (i / n)).astype(int))
		if interp in voxel_indeces:
			return False
	return True

def euclidean(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def theta_star(start, goal):
    open_set = []
    heapq.heappush(open_set, (euclidean(start, goal), start))
    g_score = {start: 0}
    parent = {start: start}
    closed_set = set()
    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            # reconstruct path
            path = [current]
            while current != parent[current]:
                current = parent[current]
                path.append(current)
            return path[::-1]
        closed_set.add(current)
        for neighbor in get_neighbors(current):
            if neighbor in closed_set:
                continue
            # If parent[current] has line of sight to neighbor, try shortcut
            if line_of_sight(parent[current], neighbor):
                tentative_g = g_score[parent[current]] + euclidean(parent[current], neighbor)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g # type: ignore
                    parent[neighbor] = parent[current]
                    heapq.heappush(open_set, (tentative_g + (euclidean(neighbor, goal) * 2), neighbor))
            else:
                tentative_g = g_score[current] + euclidean(current, neighbor)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g # type: ignore
                    parent[neighbor] = current
                    heapq.heappush(open_set, (tentative_g + (euclidean(neighbor, goal) * 2), neighbor))
    return None

def astar(start, goal):
    open_set = []
    heapq.heappush(open_set, (euclidean(start, goal), 0, start, [start]))
    closed_set = set()
    while open_set:
        f, g, current, path = heapq.heappop(open_set)
        if current == goal:
            return path
        if current in closed_set:
            continue
        closed_set.add(current)
        for neighbor in get_neighbors(current):
            if neighbor in closed_set:
                continue
            heapq.heappush(open_set, (g + 1 + euclidean(neighbor, goal), g + 1, neighbor, path + [neighbor]))
    return None


# Example usage:
st = time.monotonic()
path = theta_star(start_position, goal_position)
print("Theta* path:", path)
print(f"{time.monotonic() - st}")

real_coords = []
for coord in path:
    position = dock_voxel_grid.origin + np.array(coord) * dock_voxel_grid.voxel_size
    real_coords.append(coord)
print(real_coords)

# Create geometry for the path
path_points = []
for coord in path:
    point = dock_voxel_grid.origin + np.array(coord) * dock_voxel_grid.voxel_size
    path_points.append(point)
path_points = np.array(path_points)

# Create line set for the path
if len(path_points) > 1:
    lines = [[i, i+1] for i in range(len(path_points)-1)]
    colors = [[1, 0, 0] for _ in lines]  # Red color for path
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(path_points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([dock, line_set]) # type: ignore
else:
    # o3d.visualization.draw_geometries([mesh])
    pass
