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
        return True
        # x, y, z = int(state[0]), int(state[1]), int(state[2])
        # if x >= self.grid.shape[0] or y >= self.grid.shape[1] or z >= self.grid.shape[2]:
        #     return False
        # else:
        #     return self.grid[x, y, z] == 0

    def plan(self, start_position, max_time=1):
        # define the state space
        space = ob.RealVectorStateSpace(3)
        bounds = ob.RealVectorBounds(3)
        bounds.setLow(-100)
        bounds.setHigh(100)
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
        start[0], start[1], start[2] = start_position
        goal[0], goal[1], goal[2] = [0, 0, 0]

        # Create problem definition
        pdef = ob.ProblemDefinition(si)
        pdef.setStartAndGoalStates(start, goal)

        # Create and setup planner
        planner = og.RRTstar(si)
        planner.setProblemDefinition(pdef)
        planner.setup()
        solved = planner.solve(max_time)

        if solved:
            print(f"Solution found! Status: {solved}")
        si.freeState(start)
        si.freeState(goal)

if __name__ == "__main__":    
    planner = Planner()
    planner.plan([1, 2, 0], max_time=0.2)
    os._exit(0)