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
from visualization_msgs.msg import Marker
import ompl.util as ou
ou.setLogLevel(ou.LogLevel.LOG_NONE)

VOXEL_RESOLUTION = 0.01 # 1cm voxels
REPLAN_TIME = 0.5
HORIZON_LENGTH = 0.25

class Planner(Node):
    def __init__(self):
        start_time = time.monotonic()
        super().__init__('path_planner')

        # read dock mesh
        self.dock_mesh: o3d.geometry.TriangleMesh = o3d.io.read_triangle_mesh("simple_dock.stl") # type: ignore
        
        # create hash table with occupied voxel grid locations
        self.dock_voxel_grid = o3d.io.read_voxel_grid('dock_voxel_gapped.ply') # type: ignore
        self.voxel_indices = {}
        self.get_logger().info(f"hashing voxel grid...")
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

        # save path
        self.path = np.array([])

        # path publisher and pose subscriber
        self.path_pub = self.create_publisher(Path, '/sianat/path', 1)
        self.waypoint_pub = self.create_publisher(PoseStamped, '/sianat/waypoint', 1)
        self.pose_sub = self.create_subscription(PoseStamped, '/aruco/pose', self.pose_update, 1)
        self.get_logger().info(f"setup complete in {time.monotonic() - start_time}")

    def pose_update(self, msg: PoseStamped):
        # get positiona and distances to all points in current path
        position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        dists = [np.linalg.norm(position - path_point) for path_point in self.path]
        
        # replan if needed or first time
        if len(self.path) == 0 or np.min(dists) > HORIZON_LENGTH:
            self.path = self.plan(position, max_time=REPLAN_TIME)
            path_msg: Path = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = "base_link"
            for pt in self.path:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = float(pt[0])
                pose.pose.position.y = float(pt[1])
                pose.pose.position.z = float(pt[2])
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose) # type: ignore
            self.path_pub.publish(path_msg)

        # recalculate path distances
        for point in self.path:
            dist = np.linalg.norm(point - position)
            if dist < HORIZON_LENGTH:
                waypoint = PoseStamped()
                waypoint.header = msg.header
                waypoint.pose.position.x = point[0]
                waypoint.pose.position.y = point[1]
                waypoint.pose.position.z = point[2]
                self.waypoint_pub.publish(waypoint)
                break



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
    rclpy.shutdown()
    os._exit(0)