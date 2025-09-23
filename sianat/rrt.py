import ompl.base
import ompl.base as ob
import ompl.geometric as og
import numpy as np
import os
import open3d as o3d
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time
from scipy.spatial.transform import Rotation as R
import ompl.util as ou
ou.setLogLevel(ou.LogLevel.LOG_NONE)

VOXEL_RESOLUTION = 0.01 # 1cm voxels
REPLAN_TIME = 0.5

class Planner(Node):
    def __init__(self):
        start_time = time.monotonic()
        super().__init__('path_planner')

        # read dock mesh
        self.dock_mesh: o3d.geometry.TriangleMesh = o3d.io.read_triangle_mesh("simple_dock.stl") # type: ignore

        # create hash table with occupied voxel grid locations
        self.dock_voxel_grid = o3d.io.read_voxel_grid('dock_voxel_gapped.ply') # type: ignore
        # self.dock_voxel_grid.rotate(R.from_euler('xyz', (0, 0, 180), degrees=True).as_matrix(), center=[0, 0, 0])
        self.voxel_indices = {}
        for voxel in self.dock_voxel_grid.get_voxels():
            self.voxel_indices[tuple(voxel.grid_index)] = None

        # set up ompl planner
        space = ob.RealVectorStateSpace(3)
        bounds = ob.RealVectorBounds(3)
        bounds.setLow(-5) # type: ignore
        bounds.setHigh(5) # type: ignore
        space.setBounds(bounds)

        self.si = ob.SpaceInformation(space)
        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_state_valid)) # type: ignore
        self.si.setup() # type: ignore

        self.start = self.si.allocState() # type: ignore
        self.goal = self.si.allocState() # type: ignore

        self.problem_planner = og.RRT(self.si)
        self.problem_planner.setRange(0.01) # type: ignore
        self.problem_planner.setup() # type: ignore

        # path publisher and pose subscriber
        self.path_pub = self.create_publisher(Path, '/sianat/raw_path', 1)
        self.pose_sub = self.create_subscription(PoseStamped, '/aruco/pose', self.pose_update, 1)
        self.plan_timer = self.create_timer(REPLAN_TIME, self.plan_cb)
        self.position = (None, None, None)
        self.get_logger().info(f"setup complete in {time.monotonic() - start_time}")

    def pose_update(self, msg: PoseStamped):
        self.position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def plan_cb(self):
        if None in self.position:
            return
        path = self.plan(self.position, max_time=REPLAN_TIME / 2)
        if path.size > 0:
            path_msg: Path = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = "base_link"
            for pt in path:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = float(pt[0])
                pose.pose.position.y = float(pt[1])
                pose.pose.position.z = float(pt[2])
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose) # type: ignore
            self.path_pub.publish(path_msg)

    def is_state_valid(self, state: ompl.base.RealVectorState):
        voxel_coord = tuple(self.dock_voxel_grid.get_voxel((state[0], state[1], state[2]))) # type: ignore
        return voxel_coord not in self.voxel_indices

    def plan(self, start_position, max_time) -> np.ndarray:
        # create problem definition
        self.start[0], self.start[1], self.start[2] = start_position
        self.goal[0], self.goal[1], self.goal[2] = [-0.01, 0, 0]
        pdef = ob.ProblemDefinition(self.si)
        pdef.setStartAndGoalStates(self.goal, self.start)

        # reset planner and update with new problem definition
        self.problem_planner.clear() # type: ignore
        self.problem_planner.setProblemDefinition(pdef) # type: ignore
        solved = self.problem_planner.solve(max_time) # type: ignore

        if solved:
            self.get_logger().info(f"solution found: {solved}")
            path = pdef.getSolutionPath() # type: ignore
            return np.array([(state[0], state[1], state[2]) for state in path.getStates()])
        else:
            self.get_logger().info(f"no solution found: {solved}")
            return np.array([])
 

if __name__ == "__main__":
    rclpy.init()
    planner = Planner()
    rclpy.spin(planner)
    # planner.plan([4, 2, 0], max_time=2)
    # planner.plan([3, 0, 0], max_time=.5)
    rclpy.shutdown()
    os._exit(0)