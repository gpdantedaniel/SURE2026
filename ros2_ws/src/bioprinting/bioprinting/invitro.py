import argparse
import copy
import os
import yaml
import time
import rclpy
import serial
from geometry_msgs.msg import Pose
from rosidl_runtime_py import message_to_yaml, set_message_fields
from enum import Enum, auto
from tf2_ros import Buffer
from tf2_ros import TransformListener
from rclpy.node import Node
from rclpy.task import Future
from rclpy.action import ActionClient
from action_msgs.srv import CancelGoal
from rclpy.parameter import Parameter
from geometry_msgs.msg import Pose, TwistStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from moveit.planning import MoveItPy
from ament_index_python.packages import get_package_share_directory

# Useful for filtering and parsing GCODE
accepted_codes = ("G0", "G1", "G90", "G91")
rejected_actuations = ("E", "F")

nano_serial = serial.Serial(
    port="/dev/ttyUSB0",
    baudrate=115200,
    timeout=0.001
)

class ProbingState(Enum):
    IDLE = auto()
    PROBING = auto()

class InVitroPlanner(Node):
    def __init__(self):
        super().__init__("invitro_planner")
        # MoveIt names by Yifey Cheng
        self.group_name = "ar_manipulator"
        self.tool_link = "link_6"           
        self.base_frame = "base_link"
        
        # Instantiate the planning component (Move Group)
        # self.move_group = self.moveit.get_planning_component("panda_arm")

        # Declare parameters and defaults
        self.declare_parameter("gcode_file", "")
        self.declare_parameter("pose_filename", "")

        # For computing and executing trajectories
        self.cartesian_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.delta_twist_pub= self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)

        # Old control loop for calibration
        self.probing_timer = self.create_timer(0.02, self.probing_loop)
        self.contact_timer = self.create_timer(0.02, self.detect_contact)
        self.probing_state = ProbingState.IDLE
        self.probe_future = None

        # Probing (subject to tuning)
        self.contact = 0
        self.no_motion_counter = 0
        self.required_stationary_cycles = 1

        # Listen to tool_link's Pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initializing connections
        self.get_logger().info("⌛ Waiting for MoveIt services...")
        self.cartesian_client.wait_for_service()
        self.execute_client.wait_for_server()
        self.get_logger().info("✅ Connected to MoveIt!")

    # ----- Utility functions

    def save_pose(self, pose, filename):
        with open(f"../poses/{filename}.yaml", "w") as file:
            file.write(message_to_yaml(pose))

    def load_pose(self, filename):
        with open(f"../poses/{filename}.yaml", "r") as file:
            pose_data = yaml.safe_load(file)

        pose = Pose()
        set_message_fields(pose, pose_data)
        return pose
    
    def move_relative_to(self, pose, dx, dy, dz):
        target = copy.deepcopy(pose)
        target.position.x += dx
        target.position.y += dy
        target.position.z += dz
        trajectory = self.compute_cartesian_path([target])
        self.execute_trajectory(trajectory)

    def move_relative_to_current(self):
        # Uses hard-coded values just for testing
        current_pose = self.get_current_tcp_pose()
        self.move_relative(current_pose, 0.0, 0.0, 0.010)
    
    # ----- Troubleshooting and manual operations

    def save_current_pose(self):
        current_pose = self.get_current_tcp_pose()
        self.get_logger().info(f"Current pose: {current_pose}")
        filename = self.get_parameter("pose_filename").value
        self.get_logger().info(f"Saving at: {filename}.yaml")
        self.save_pose(current_pose, filename)
        
    def assume_pose(self):
        pose_filename = self.get_parameter("pose_filename").value
        self.get_logger().info(f"Reading pose at: {pose_filename}.yaml")

        pose = self.load_pose(pose_filename)
        trajectory = self.compute_cartesian_path([pose])
        self.execute_trajectory(trajectory)

    def probing_loop(self):
        match self.probing_state:
            case ProbingState.PROBING:
                if self.contact:
                    self.get_logger().info("💥 Probing contact detected")
                    self.probing_state = ProbingState.IDLE
                    current_pose = self.get_current_tcp_pose()

                    # Should come up with a better solution to absolutely halt all movement
                    self.publish_probe_twist(0.00)

                    self.no_motion_counter = 0
                    self.contact = False
                    self.probe_future.set_result(current_pose)


                self.publish_probe_twist(-0.01)


            case ProbingState.IDLE:
                pass
    
    def probe_point(self):
        self.get_logger().info("Probing point...")
        future = Future()
        self.probe_future = future
        self.probing_state = ProbingState.PROBING
        rclpy.spin_until_future_complete(self, future)
        pose = future.result()

        return pose

    def calibrate(self):
        """
        Calibration proceeds through three points that are saved locally
        to perform plane fitting subsequently and 3D print!
        """

        # Some hard-coded values...
        # can be dynamically populated later

        points_and_offsets = {
            "plane_p1": {"dx": 0, "dy": 0.05, "dz": 0.01},
            "plane_p2": {"dx": -0.05, "dy": 0, "dz": 0.01},
            "plane_p3": {"dx": 0.05, "dy": -0.05, "dz": 0.01}
        }

        for point_name, end_offset in points_and_offsets.items():
            pose = self.probe_point()
            self.save_pose(pose, point_name)
            self.get_logger().info(f"{point_name} recorded")
            
            time.sleep(1)
            self.move_relative_to(pose, end_offset["dx"], end_offset["dy"], end_offset["dz"])
            time.sleep(1)
        
        self.get_logger().info("✅ Calibration successful!")

    
    def publish_probe_twist(self, z_velocity):
        """
        Commands the end-effector to probe slowly until it
        senses a collision... or something of the sort
        """
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.z = z_velocity
        self.delta_twist_pub.publish(msg)

    def detect_contact(self):
        """
        Detects contact with a surface by reading the
        contact state read by the Arduino Nano
        and the poor man's limit switch
        """
    
        # Read whatever the Arduino Nano is saying
        line = nano_serial.readline().decode("utf-8").strip()

        if line == "PRESSED":
            self.no_motion_counter += 1
        else:
            self.no_motion_counter = 0

        self.contact = self.no_motion_counter >= self.required_stationary_cycles

    def gcode_to_waypoints(self, gcode_path: str, current_pose: Pose):
        """
        Reads, parses and translates GCODE commands into waypoints
        suitable for AR4 MK3 execution with MoveIt
        """
        with open(gcode_path, "r") as file:
            gcode_lines = file.readlines()

        # Preprocessing should probably be refined later
        filtered_lines = list(filter(lambda l: l.startswith(accepted_codes), gcode_lines))
        # More deterministic filtering algorithm required - specific to Prusa Slicer
        filtered_lines = filtered_lines[14:-2]

        # Holds the sequentially updated position
        coords = {"x": 0.0, "y": 0.0, "z": 0.0}
        waypoints = []

        # Read each GCODE position
        for line in filtered_lines:
            for code in line.split(" "):
                match code[0]:
                    case "X":
                        coords["x"] = float(code[1:])
                    case "Y":
                        coords["y"] = float(code[1:])
                    case "Z":
                        coords["z"] = float(code[1:])

            pose = Pose()

            # Scale positions from mm to m
            pose.position.x = coords["x"]/1000
            pose.position.y = coords["y"]/1000
            pose.position.z = coords["z"]/1000

            # Orient tool_link downwards
            pose.orientation.x = 0.0
            pose.orientation.y = 1.0
            pose.orientation.z = 0.0
            pose.orientation.w = 0.0

            waypoints.append(pose)

        start_waypoint = copy.deepcopy(waypoints[0])

        for w in waypoints:
            # Normalize subsequent positions with respect to print start
            w.position.x = w.position.x - start_waypoint.position.x
            w.position.y = w.position.y - start_waypoint.position.y
            w.position.z = w.position.z - start_waypoint.position.z

            # Normalize with respect to current pose (refactor later)
            w.position.x = w.position.x + current_pose.position.x
            w.position.y = w.position.y + current_pose.position.y
            w.position.z = w.position.z + current_pose.position.z

        return waypoints
        
    def get_current_tcp_pose(self):
        """
        Query tool_link's transform, represents as a Pose and return
        """
        while rclpy.ok():
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame=self.base_frame,
                    source_frame=self.tool_link,
                    time=rclpy.time.Time()
                )
                break
            except Exception as ex:
                self.get_logger().info(f"Waiting for TF {ex}")
                rclpy.spin_once(self, timeout_sec=0.1)

        pose = Pose()

        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z

        # actually quaternion is indeed required
        pose.orientation.x = transform.transform.rotation.x
        pose.orientation.y = transform.transform.rotation.y
        pose.orientation.z = transform.transform.rotation.z
        pose.orientation.w = transform.transform.rotation.w

        # For debugging purposes
        self.get_logger().info(
            f"Current pose: "
            f"x={pose.position.x:.4f} "
            f"y={pose.position.y:.4f} "
            f"z={pose.position.z:.4f} "
        )

        return pose

    def compute_cartesian_path(self, waypoints: list[Pose], callback=None):
        """
        Build and send request to /compute_cartesian_path 
        service and return computed trajectory
        """
        # Build and populate request
        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_frame
        req.group_name = self.group_name
        req.link_name = self.tool_link
        req.waypoints = waypoints
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = 1.0
        req.max_acceleration_scaling_factor = 1.0

        # Send out the request asynchronously
        self.get_logger().info("Calling cartesian path compute")
        future = self.cartesian_client.call_async(req)

        # Should be refined afterwards
        if callback:
            future.add_done_callback(callback)
        else:
            # Get the service to spin and read result
            self.get_logger().info("Waiting for response")
            rclpy.spin_until_future_complete(self, future)

            
            self.get_logger().info("Response received")
            result = future.result()

            path_fraction = result.fraction
            self.get_logger().info(f"Path fraction: {path_fraction:.3f}")

            if result is None:
                raise RuntimeError("Cartesian planning call failed.")

            return result.solution
    
    def execute_trajectory(self, trajectory: ExecuteTrajectory, callback=None):
        """
        Send out trajectory goal and reports
        acceptance and execution statuses
        """
        # Build goal with trajectory
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        
        self.get_logger().info("Sending out trajectory...")
        send_goal_future = self.execute_client.send_goal_async(goal)

        if callback:
            
            def goal_response_cb(future):
                goal_handle = future.result()
                if not goal_handle.accepted:
                    self.get_logger().error("Trajectory execution rejected")
                    return
                
                self.get_logger().info(f"Accepted = {goal_handle.accepted}")

                result_future = goal_handle.get_result_async()
                self.get_logger().info("Executing trajectory...")

                result_future.add_done_callback(callback)
            
            send_goal_future.add_done_callback(callback=goal_response_cb)

        else:
            rclpy.spin_until_future_complete(self, send_goal_future)
            self.get_logger().info("Waiting for execution acceptance...")

            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                raise RuntimeError("Trajectory execution rejected.")
            else:
                self.get_logger().info("Trajectory accepted.")

            self.get_logger().info("Executing trajectory...")
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            result = result_future.result()
            self.get_logger().info("Trajectory execution finished.")

            return result
    
    def orient(self):
        """
        Orients the end-effector downwards wherever it stands.
        Should account for end-effector length.
        """
        current_pose = self.get_current_tcp_pose()
        downards_pose = Pose()

        # Keep the end-effector at the same XYZ point
        downards_pose.position.x = current_pose.position.x
        downards_pose.position.y = current_pose.position.y
        downards_pose.position.z = current_pose.position.z

        # Point end-effector down
        downards_pose.orientation.x = 0.0
        downards_pose.orientation.y = 1.0
        downards_pose.orientation.z = 0.0
        downards_pose.orientation.w = 0.0

        self.get_logger().info("Computing trajectory...")
        trajectory = self.compute_cartesian_path([downards_pose])
        self.get_logger().info("Trajectory computed.")

        self.get_logger().info("Executing trajectory...")
        self.execute_trajectory(trajectory)
        self.get_logger().info("Trajectory complete.")

    def execute_print(self):
        """
        Computes and executes printing trajectory based on GCODE 
        file gcode_file requested and provided at startup
        """
        current_pose = self.get_current_tcp_pose()

        # Read GCODE lines and parse into waypoints
        gcode_file = self.get_parameter("gcode_file").value
        self.get_logger().info(f"Reading {gcode_file}")

        package_share_dir = get_package_share_directory('bioprinting')
        gcode_path = os.path.join(package_share_dir, gcode_file)

        # Should be renamed gcode_to_waypoints
        waypoints = self.gcode_to_waypoints(gcode_path, current_pose)
        
        self.get_logger().info("Computing trajectory...")
        trajectory = self.compute_cartesian_path(waypoints)
        self.get_logger().info("Trajectory computed.")

        self.get_logger().info("Executing trajectory...")
        self.execute_trajectory(trajectory)
        self.get_logger().info("Trajectory complete.")    

commands = {
    "calibrate": lambda node: node.calibrate(),
    "move_relative_to_current": lambda node: node.move_relative_to_current(),
    "execute_print": lambda node: node.execute_print(),
    "orient": lambda node: node.orient(),
    "save_current_pose": lambda node: node.save_current_pose(),
    "assume_pose": lambda node: node.assume_pose() 
}

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--command", type=str, default='orient', choices=commands.keys())
    parsed_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = InVitroPlanner()

    commands[parsed_args.command](node)
    node.destroy_node()

    nano_serial.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()