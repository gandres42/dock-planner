import ompl.base
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
        self.dock_mesh: o3d.geometry.TriangleMesh = o3d.io.read_triangle_mesh("simple_dock.stl")
        # self.dock_voxel_grid: o3d.geometry.VoxelGrid = o3d.geometry.VoxelGrid.create_from_triangle_mesh(self.dock_mesh, voxel_size=VOXEL_RESOLUTION)
        self.dock_voxel_grid = o3d.io.read_voxel_grid('dock_voxel_gapped.ply')
        self.dock_pcd: o3d.geometry.PointCloud = self.dock_mesh.sample_points_uniformly(number_of_points=10000)
        self.voxel_indices = {}
        for voxel in self.dock_voxel_grid.get_voxels():
            self.voxel_indices[tuple(voxel.grid_index)] = None
        print('hashing complete')

    def nearest_distance_compute(self, pt):
        pt = np.asarray(pt, dtype=float)
        if len(self.dock_pcd.points) == 0:
            raise ValueError("empty point cloud")
        q = o3d.geometry.PointCloud()
        q.points = o3d.utility.Vector3dVector([pt])
        dists = np.asarray(q.compute_point_cloud_distance(self.dock_pcd))
        return np.min(dists)

    def is_state_valid(self, state: ompl.base.RealVectorState):
        real_coord = (state[0], state[1], state[2])
        voxel_coord = tuple(self.dock_voxel_grid.get_voxel(real_coord))
        return voxel_coord not in self.voxel_indices # and self.nearest_distance_compute(real_coord) > .2

    def plan(self, start_position, max_time=1.0):
        # define the state .setspace
        
        space = ob.RealVectorStateSpace(3)
        bounds = ob.RealVectorBounds(3)
        bounds.setLow(-5)
        bounds.setHigh(5)
        space.setBounds(bounds)

        # set up the space information
        si = ob.SpaceInformation(space)
        si.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_state_valid))
        si.setup()

        # allocate start and goal states
        start = si.allocState()
        goal = si.allocState()

        # Set start and goal positions
        start[0], start[1], start[2] = start_position
        goal[0], goal[1], goal[2] = [0.01, 0, 0]

        # Create problem definition
        pdef = ob.ProblemDefinition(si)
        pdef.setStartAndGoalStates(goal, start)

        # Create and setup planner
        problem_planner = og.RRT(si)
        problem_planner.setRange(0.02)
        problem_planner.setProblemDefinition(pdef)
        problem_planner.setup()
        solved = problem_planner.solve(max_time)


        if solved:
            print(f"Solution found! Status: {solved}")
            path = pdef.getSolutionPath()

            # Extract points from the solution path
            points_np = np.array([(state[0], state[1], state[2]) for state in path.getStates()])

            # print(points_np)

            # Create Open3D PointCloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points_np)

            # Create lines connecting consecutive points
            lines = [[i, i+1] for i in range(len(points_np)-1)]

            # Create LineSet
            line_set = o3d.geometry.LineSet()
            line_set.points = o3d.utility.Vector3dVector(points_np)
            line_set.lines = o3d.utility.Vector2iVector(lines)

            # Set line color (red)
            colors = [[1, 0, 0] for _ in lines]
            line_set.colors = o3d.utility.Vector3dVector(colors)

            # Visualize both points and lines
            axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
            o3d.visualization.draw_geometries([pcd, line_set, self.dock_mesh])
        else:
            print("No solution found")


        si.freeState(start)
        si.freeState(goal)

if __name__ == "__main__":    
    planner = Planner()
    planner.plan([4, 2, 0], max_time=.5)
    os._exit(0)