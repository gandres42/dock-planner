import open3d as o3d
import heapq
import numpy as np
import time

dock_voxel_grid = o3d.io.read_voxel_grid("dock_voxel.ply")
voxel_indeces = {}
for voxel in dock_voxel_grid.get_voxels():
    voxel_indeces[tuple(voxel.grid_index)] = None
print('hashed!')

start_position = tuple(dock_voxel_grid.get_voxel([0, 0, 0]))
goal_position = tuple(dock_voxel_grid.get_voxel([1, 0, 0]))

def get_neighbors(pos):
    directions = [
        (5, 0, 0), (-5, 0, 0),
        (0, 5, 0), (0, -5, 0),
        (0, 0, 5), (0, 0, -5)
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

def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def theta_star(start, goal):
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), start))
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
                tentative_g = g_score[parent[current]] + heuristic(parent[current], neighbor)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    parent[neighbor] = parent[current]
                    heapq.heappush(open_set, (tentative_g + heuristic(neighbor, goal), neighbor))
            else:
                tentative_g = g_score[current] + heuristic(current, neighbor)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    parent[neighbor] = current
                    heapq.heappush(open_set, (tentative_g + heuristic(neighbor, goal), neighbor))
    return None

def astar(start, goal):
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), 0, start, [start]))
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
            heapq.heappush(open_set, (g + 1 + heuristic(neighbor, goal), g + 1, neighbor, path + [neighbor]))
    return None


# Example usage:
st = time.monotonic()
path = theta_star(start_position, goal_position)
print("Theta* path:", path)
print(f"{time.monotonic() - st}")
