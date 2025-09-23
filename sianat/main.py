import ompl.base
import ompl.base as ob
import ompl.geometric as og
import numpy as np
import os
import open3d as o3d
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
import time

VOXEL_RESOLUTION = 0.01 # 1cm voxels

class Planner(Node):
    def __init__(self):
        start_time = time.monotonic()
        super().__init__('path_planner')

        # read dock mesh
        self.dock_mesh: o3d.geometry.TriangleMesh = o3d.io.read_triangle_mesh("simple_dock.stl") # type: ignore

        # create hash table with occupied voxel grid locations
        self.dock_voxel_grid = o3d.io.read_voxel_grid('dock_voxel_gapped.ply') # type: ignore
        self.voxel_indices = {}
        for voxel in self.dock_voxel_grid.get_voxels():
            self.voxel_indices[tuple(voxel.grid_index)] = None

        # set up ompl planner
        space = ob.RealVectorStateSpace(3)
        bounds = ob.RealVectorBounds(3)
        bounds.setLow(-5) # type: ignore
        bounds.setHigh(5) # type: ignore
        space.setBounds(bounds)

        # set up the space information
        self.si = ob.SpaceInformation(space)
        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_state_valid)) # type: ignore
        self.si.setup() # type: ignore

        # allocate start and goal states
        self.start = self.si.allocState() # type: ignore
        self.goal = self.si.allocState() # type: ignore

        # Create and setup planner
        self.problem_planner = og.RRT(self.si)
        self.problem_planner.setRange(0.01) # type: ignore
        self.problem_planner.setup() # type: ignore

        # path publisher and pose subscriber
        self.path_pub = self.create_publisher(Path, '/sianat/path', 1)
        self.pose_sub = self.create_subscription(Pose, '/sianat/pose', self.pose_update, 1)

        self.get_logger().info(f"setup complete in {time.monotonic() - start_time}")

    def pose_update(self, msg):
        pass

    def is_state_valid(self, state: ompl.base.RealVectorState):
        voxel_coord = tuple(self.dock_voxel_grid.get_voxel((state[0], state[1], state[2]))) # type: ignore
        return voxel_coord not in self.voxel_indices

    def plan(self, start_position, max_time=1.0):
        self.start[0], self.start[1], self.start[2] = start_position
        self.goal[0], self.goal[1], self.goal[2] = [0.01, 0, 0]
        pdef = ob.ProblemDefinition(self.si)
        pdef.setStartAndGoalStates(self.goal, self.start)
        self.problem_planner.clear() # type: ignore
        self.problem_planner.setProblemDefinition(pdef) # type: ignore
        solved = self.problem_planner.solve(max_time) # type: ignore

        if solved:
            self.get_logger().info(f"solution found: {solved}")
            path = pdef.getSolutionPath() # type: ignore

            # Extract points from the solution path
            points_np = np.array([(state[0], state[1], state[2]) for state in path.getStates()])

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
            axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0]) # type: ignore
            o3d.visualization.draw_geometries([pcd, line_set, self.dock_mesh])
        else:
            self.get_logger().info(f"no solution found: {solved}")
 

if __name__ == "__main__":
    rclpy.init()
    planner = Planner()
    planner.plan([4, 2, 0], max_time=2)
    planner.plan([3, 0, 0], max_time=.5)
    rclpy.shutdown()
    os._exit(0)