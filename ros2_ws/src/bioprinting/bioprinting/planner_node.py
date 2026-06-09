import copy
import math
import os
from scipy.spatial.transform import Rotation
import rclpy
from tf2_ros import Buffer
from tf2_ros import TransformListener
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from ament_index_python.packages import get_package_share_directory

# The quaternion for the tool pointing down is [0.0, 1.0, 0.0, 0.0] ... apparently

"""
def create_waypoints(center_pose, radius=0.05):
    start = copy.deepcopy(center_pose)
    start.orientation.x = 0.0
    start.orientation.y = 1.0
    start.orientation.z = 0.0
    start.orientation.w = 0.0

    waypoints = [start]

    # Describing a cube for now

    for j in range(4):
        for i in range(64):
            angle = 2.0 * math.pi * i / 64
            pose = Pose()

            pose.position.x = center_pose.position.x + radius * math.cos(angle)
            pose.position.y = center_pose.position.y + radius * math.sin(angle)
            pose.position.z = center_pose.position.z + radius * j

            pose.orientation.x = 0.0
            pose.orientation.y = 1.0
            pose.orientation.z = 0.0
            pose.orientation.w = 0.0 

            waypoints.append(pose)

    waypoints.append(waypoints[0])
    waypoints.append(center_pose)

    return waypoints
"""

import copy
from geometry_msgs.msg import Pose, Quaternion

accepted_codes = ("G0", "G1", "G90", "G91")
rejected_actuations = ("E", "F")

def clean_gcode_line(l):
    split_actuations = l.split(" ")
    cleaned_actuations = list(filter(lambda l: not l.startswith(rejected_actuations), split_actuations))
    joined_actuations = " ".join(cleaned_actuations)
    return joined_actuations


def generate_waypoints(gcode_lines: list[str], current_pose):
    # Holds the presently parsed waypoint
    coords = {"x": 0.0, "y": 0.0, "z": 0.0}
    waypoints = []

    for line in gcode_lines:
        for code in line.split(" "):
            match code[0]:
                case "X":
                    coords["x"] = float(code[1:])
                case "Y":
                    coords["y"] = float(code[1:])
                case "Z":
                    coords["z"] = float(code[1:])

        pose = Pose()
        pose.position.x = coords["x"]/1000
        pose.position.y = coords["y"]/1000
        pose.position.z = coords["z"]/1000

        pose.orientation.x = 0.0
        pose.orientation.y = 1.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0

        waypoints.append(pose)

    start_waypoint = copy.deepcopy(waypoints[0])

    for w in waypoints:
        w.position.x = w.position.x - start_waypoint.position.x
        w.position.y = w.position.y - start_waypoint.position.y
        w.position.z = w.position.z - start_waypoint.position.z

    for w in waypoints:
        w.position.x = w.position.x + current_pose.position.x
        w.position.y = w.position.y + current_pose.position.y
        w.position.z = w.position.z + current_pose.position.z

    return waypoints


def gcode_to_waypoints(path, current_pose):
    with open(path, "r") as file:
        gcode_lines = file.readlines()

    filtered_lines = list(filter(lambda l: l.startswith(accepted_codes), gcode_lines))
    parsed_lines = list(map(lambda l: clean_gcode_line(l), filtered_lines))
    waypoints = generate_waypoints(parsed_lines, current_pose)

    return waypoints


class HexagonPlanner(Node):
    def __init__(self):
        super().__init__("hexagon_drawer")
        self.group_name = "ar_manipulator"
        self.tool_link = "link_6"           # Should be checked
        self.base_frame = "base_link"       # Should be checked

        self.cartesian_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info("Waiting for MoveIt services...")
        self.cartesian_client.wait_for_service()
        self.execute_client.wait_for_server()
        self.get_logger().info("Connected to MoveIt.")

    def get_current_tcp_pose(self):

        while rclpy.ok():
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.tool_link,
                    rclpy.time.Time()
                )

                break
            except Exception as ex:
                self.get_logger().info(f"Waiting for TF {ex}")
                rclpy.spin_once(self, timeout_sec=0.1)

        pose = Pose()

        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = (transform.transform.rotation)

        return pose


    def compute_cartesian_path(self, waypoints):
        # Build request and fill fields
        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_frame
        req.group_name = self.group_name
        req.link_name = self.tool_link
        req.waypoints = waypoints
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = 0.20
        req.max_acceleration_scaling_factor = 0.20

        # Send out the request asynchronously
        self.get_logger().info("Calling cartesian path compute")
        future = self.cartesian_client.call_async(req)
        self.get_logger().info("Waiting for response")
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("Response received")

        result = future.result()

        if result is None:
            raise RuntimeError("Cartesian planning call failed.")
        
        self.get_logger().info(f"Cartesian path fraction: {result.fraction:.3f}")

        print(result.error_code)
        print(result.error_code.val)

        return result.solution
    
    def execute_trajectory(self, trajectory):
        # Build goal and populate
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        
        send_goal_future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            raise RuntimeError("Trajectory execution rejected.")
        self.get_logger().info("Trajectory accepted.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()

        self.get_logger().info("Trajectory execution finished.")

        return result

    def draw_hexagon(self):

        current_pose = self.get_current_tcp_pose()
        self.get_logger().info(
            f"Current pose: "
            f"x={current_pose.position.x:.4f} "
            f"y={current_pose.position.y:.4f} "
            f"z={current_pose.position.z:.4f} "
        )

        
        radius = 0.10

        self.get_logger().info(
            f"Current pose: "
            f"x={current_pose.position.x:.4f} "
            f"y={current_pose.position.y:.4f} "
            f"z={current_pose.position.z:.4f} "
        )

        self.get_logger().info("Generating waypoints...")
        # waypoints = create_waypoints(current_pose, radius)
        
        # package_share_dir = get_package_share_directory('bioprinting')
        # gcode_file_path = os.path.join(package_share_dir, 'gcode', 'box.gcode')

        package_share_dir = get_package_share_directory('bioprinting')

        from pathlib import Path

        print([f.name for f in Path(get_package_share_directory('bioprinting')).iterdir() if f.is_file()])
        
        waypoints = gcode_to_waypoints(os.path.join(package_share_dir, "box.gcode"), current_pose)


        # Just some debugging
        # target = copy.deepcopy(current_pose)
        # target.position.x += 0.10
        # waypoints = [target]
        
        self.get_logger().info(f"Waypoints generated ({len(waypoints)}):")

        for i, p in enumerate(waypoints):
            self.get_logger().info(
                f"WP {i}: "
                f"x={p.position.x:.4f} "
                f"y={p.position.y:.4f} "
                f"z={p.position.z:.4f} "
            )

        
        self.get_logger().info("Computing trajectory...")
        trajectory = self.compute_cartesian_path(waypoints)
        self.get_logger().info("Trajectory computed.")

        self.get_logger().info("Executing trajectory...")
        self.execute_trajectory(trajectory)
        self.get_logger().info("Trajectory complete.")    


def main():
    rclpy.init()
    node = HexagonPlanner()
    node.draw_hexagon()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
