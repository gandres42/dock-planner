#!/usr/bin/env python3

import ompl.base as ob
import ompl.geometric as og
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import open3d as o3d

VOXEL_RESOLUTION = 0.01 # 1cm voxels

class Planner:
    def __init__(self):
        self.dock_voxel_grid = o3d.io.read_voxel_grid("dock_voxel.ply")

    def is_state_valid(self, state):
        x, y, z = int(state[0]), int(state[1]), int(state[2])
        if x >= self.grid.shape[0] or y >= self.grid.shape[1] or z >= self.grid.shape[2]:
            return False
        else:
            return self.grid[x, y, z] == 0

    def plan_and_visualize(self, start_position, max_time=1):
        # work with disposable copy of voxel grid
        voxel_grid = self.dock_voxel_grid.__copy__()

        # add starting position as voxel in grid to set dimensions
        grid_index = voxel_grid.get_voxel(start_position)
        if grid_index is not None:
            voxel_grid.add_voxel(o3d.geometry.Voxel(grid_index=grid_index))

        # update current grid
        voxels = np.array([v.grid_index for v in voxel_grid.get_voxels()])
        min_indices = voxels.min(axis=0)
        max_indices = voxels.max(axis=0)
        self.grid = np.zeros((
            max_indices[0] - min_indices[0] + 1,
            max_indices[1] - min_indices[1] + 1,
            max_indices[2] - min_indices[2] + 1
        ), dtype=int)
        
        for voxel in voxels:
            self.grid[voxel[0], voxel[1], voxel[2]] = 1
        self.grid[grid_index[0], grid_index[1], grid_index[2]] = 0

        # define the state space
        space = ob.RealVectorStateSpace(3)
        bounds = ob.RealVectorBounds(3)
        bounds.setLow(0)
        bounds.setHigh(max(self.grid.shape))
        space.setBounds(bounds)

        # set up the space information
        si = ob.SpaceInformation(space)
        si.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_state_valid))
        si.setup()

        # allocate start and goal states
        start = si.allocState()
        goal = si.allocState()
        path_points = []
        
        # Set start and goal positions
        start[0], start[1], start[2] = list(map(int, voxel_grid.get_voxel(start_position)))
        goal[0], goal[1], goal[2] = list(map(int, voxel_grid.get_voxel([0, 0, 0])))

        # Create problem definition
        pdef = ob.ProblemDefinition(si)
        pdef.setStartAndGoalStates(start, goal)

        # Create and setup planner
        planner = og.RRTstar(si)
        # planner.setRange(0.01)  # Limit the maximum length of a motion (step size)
        planner.setProblemDefinition(pdef)
        planner.setup()
        solved = planner.solve(max_time)

        if solved:
            print(f"Solution found! Status: {solved}")
            path = pdef.getSolutionPath()
            
            # Interpolate for smoother visualization
            # path.interpolate(100)  # Add intermediate points
            
            # Extract path points
            states = path.getStates()
            for i in range(path.getStateCount()):
                s = states[i]
                voxel_point = [float(s[0]), float(s[1]), float(s[2])]
                voxel_center = voxel_grid.get_voxel_center_coordinate(voxel_point)
                path_points.append(list(voxel_point))
            path_array = np.array(path_points)
            print(path_array)
            lines = [[i, i + 1] for i in range(len(path_points) - 1)]
            colors = [[1, 0, 0] for _ in lines]  # Red color for path

            line_set = o3d.geometry.LineSet()
            line_set.points = o3d.utility.Vector3dVector(path_array)
            line_set.lines = o3d.utility.Vector2iVector(lines)
            line_set.colors = o3d.utility.Vector3dVector(colors)

            # Visualize
            o3d.visualization.draw_geometries([self.dock_voxel_grid, line_set])
        si.freeState(start)
        si.freeState(goal)

if __name__ == "__main__":    
    planner = Planner()
    planner.plan_and_visualize([1, 2, 0])
    os._exit(0)