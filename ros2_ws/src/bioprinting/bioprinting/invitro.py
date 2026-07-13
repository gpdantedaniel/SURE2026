import argparse
import copy
import os
import yaml
import time
import json
import rclpy
import numpy as np
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
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Pose, TwistStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from moveit.planning import MoveItPy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from ament_index_python.packages import get_package_share_directory

import sys
import select

# Useful for filtering and parsing GCODE
accepted_codes = ("G0", "G1", "G90", "G91")
rejected_actuations = ("E", "F")

"""
nano_serial = serial.Serial(
    port="/dev/ttyUSB0",
    baudrate=115200,
    timeout=0.001
)
"""

qos_joint_trajectory = QoSProfile(depth=10)
qos_joint_trajectory.reliability = ReliabilityPolicy.RELIABLE
qos_joint_trajectory.durability = DurabilityPolicy.VOLATILE

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
        self.declare_parameter("target_joint", "")

        # For computing and executing trajectories
        self.cartesian_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.delta_twist_pub= self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)

        self.joint_state_sub = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        self.joint_trajectory_pub = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", qos_joint_trajectory)

        self.latest_positions = None
        self.joint_name_to_index = None

        # Old control loop for calibration
        # self.probing_timer = self.create_timer(0.02, self.probing_loop)
        # self.contact_timer = self.create_timer(0.02, self.detect_contact)
        # self.probing_state = ProbingState.IDLE
        # self.probe_future = None
        # self.contact = 0
        # self.no_motion_counter = 0
        # self.required_stationary_cycles = 1

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
    
    def stop_probe_motion(self, repeats=10, period=0.02):
        for _ in range(repeats):
            self.publish_probe_twist(0.0)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)
    
    def manual_probe_point(self, point_name, z_velocity=-0.001, period=0.02, timeout_sec=30.0):
        self.get_logger().info(f"Ready to manually probe {point_name}")
        self.get_logger().info("Place tool above the intended contact point.")
        input("Press ENTER to begin descent")

        self.get_logger().info(f"Descending at {abs(z_velocity) * 1000:.1f} mm/s")

        start_time = time.time()

        try:
            while rclpy.ok():
                if sys.stdin in select.select([sys.stdin], [], [], 0.0)[0]:
                    sys.stdin.readline()
                    self.get_logger().info("ENTER detected! Halting trigger received...")
                    break
                if time.time() - start_time > timeout_sec:
                    raise RuntimeError("Manual probing timed out:(")
                
                self.publish_probe_twist(z_velocity)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(period)
        
        finally:
            self.stop_probe_motion()

        time.sleep(0.1)
        pose = self.get_current_tcp_pose()
        self.get_logger().info(f"Recorded manual probe pose for {point_name}")
        return pose


    def calibrate(self):
        """
        Calibration proceeds through three points that are saved locally
        to perform plane fitting subsequently and 3D print!
        """

        # Some hard-coded values...
        # can be dynamically populated later

        points_and_offsets = {
            "plane_p1": {"dx": 0, "dy": 0.05, "dz": 0.005},
            "plane_p2": {"dx": -0.05, "dy": 0, "dz": 0.005},
            "plane_p3": {"dx": 0.05, "dy": -0.05, "dz": 0.005}
        }

        for point_name, end_offset in points_and_offsets.items():
            # pose = self.probe_point()
            pose = self.manual_probe_point(point_name)

            self.save_pose(pose, point_name)
            self.get_logger().info(f"{point_name} recorded")
            
            time.sleep(1)
            self.get_logger().info("Moving above next probing region...")
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

    '''
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
    '''

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
        filtered_lines = filtered_lines[15:-2]

        # Holds the presently parsed waypoint
        coords = {"x": 0.0, "y": 0.0, "z": 0.0}
        waypoints = []

        for line in filtered_lines:
            if "X" in line or "Y" in line or "Z" in line:
                for code in line.split(" "):
                    match code[0]:
                        case "X":
                            coords["x"] = float(code[1:])/1000
                        case "Y":
                            coords["y"] = float(code[1:])/1000
                        case "Z":
                            coords["z"] = float(code[1:])/1000

                waypoints.append(np.array([
                    coords["x"],
                    coords["y"],
                    coords["z"]
                ]))

        waypoints = np.array(waypoints)
        distance = np.sqrt(waypoints[:, 0]**2 + waypoints[:, 1]**2)
        waypoints = waypoints - waypoints[0]

        origin = np.array([[
            current_pose.position.x,
            current_pose.position.y,
            current_pose.position.z
        ]])

        print('translated waypoints: ')
        for w in waypoints:
            print(w)

        waypoints = waypoints + origin

        print('origin: ', origin)

        print("waypoints: ")
        for w in waypoints:
            print(w)

        poses = []

        for w in waypoints:

            pose = Pose()

            pose.position.x = float(w[0])
            pose.position.y = float(w[1])
            pose.position.z = float(w[2])

            pose.orientation.x = 0.0
            pose.orientation.y = 1.0
            pose.orientation.z = 0.0
            pose.orientation.w = 0.0

            poses.append(pose)

        return poses
        
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

        print("gcode file: ", self.get_parameter("gcode_file"))

        if self.get_parameter("gcode_file").value != "":
            req.max_velocity_scaling_factor = 0.01
            req.max_acceleration_scaling_factor = 0.01
        else:
            req.max_velocity_scaling_factor = 1.0
            req.max_acceleration_scaling_factor = 1.0

        req.max_velocity_scaling_factor = 1.0
        req.max_acceleration_scaling_factor = 1.0

        # Send out the request asynchronously
        self.get_logger().info("Calling cartesian path compute")
        future = self.cartesian_client.call_async(req)

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

        self.get_logger().info(f"🔧 Compensating backlash...")
        trajectory = self.compensate_backlash_j5(trajectory)

        self.get_logger().info("Executing trajectory...")
        self.execute_trajectory(trajectory)
        self.get_logger().info("Trajectory complete.")

    def read_pose_coordinates(self, pose_filename):
        with open(f"../poses/{pose_filename}.yaml", "r") as file:
            yaml_data = yaml.safe_load(file)

        coords = yaml_data["position"]
        position = np.array([coords['x'], coords['y'], coords['z']])
        return position

    def transform_waypoints(self, waypoints):
        p1 = self.read_pose_coordinates("plane_p1")
        p2 = self.read_pose_coordinates("plane_p2")
        p3 = self.read_pose_coordinates("plane_p3")

        u = p2 - p1
        v = p3 - p1
        n = np.cross(u, v)

        x_p = u / np.linalg.norm(u)
        z_p = n / np.linalg.norm(n)
        y_p = np.cross(x_p, z_p) / np.linalg.norm(np.cross(x_p, z_p))

        R = np.column_stack((x_p, y_p, z_p))
        t = p1

        transformed_points = np.array([
            np.dot(R, np.array([w.position.x, w.position.y, w.position.z])) + t
            for w in waypoints[14:-2]
        ])

        transformed_waypoints = []

        for a in transformed_points:
            pose = Pose()

            pose.position.x = a[0]
            pose.position.y = a[1]
            pose.position.z = a[2]

            # Orient downwards
            pose.orientation.x = 0.0
            pose.orientation.y = 1.0
            pose.orientation.z = 0.0
            pose.orientation.w = 0.0

            transformed_waypoints.append(pose)

        return transformed_waypoints
    

    def execute_print(self):
        """
        Computes and executes printing trajectory based on GCODE 
        file gcode_file requested and provided at startup
        """
        # Read GCODE lines and parse into waypoints
        gcode_file = self.get_parameter("gcode_file").value
        self.get_logger().info(f"Reading {gcode_file}")

        package_share_dir = get_package_share_directory('bioprinting')
        gcode_path = os.path.join(package_share_dir, gcode_file)

        p1 = self.load_pose("plane_p1")

        waypoints = self.gcode_to_waypoints(gcode_path, p1)

        # EXPERIMENTAL : should be fixed... for some reason sending to infinity... likely how gcode is read
        # waypoints = self.transform_waypoints(waypoints)

        self.get_logger().info("Applying -2 mm offset to all waypoints")
        for w in waypoints:
            w.position.z = w.position.z - 0.002

        
        self.get_logger().info("Computing trajectory...")
        trajectory = self.compute_cartesian_path(waypoints)

        # EXPERIMENTAL : compensating backlash... should be tuned
        # trajectory = self.compensate_backlash_j5(trajectory)

        self.get_logger().info("Trajectory computed.")

        self.get_logger().info("Executing trajectory...")
        self.execute_trajectory(trajectory)
        self.get_logger().info("Trajectory complete.")

    # ----- New methods for (hopefully functionally) diagnosing backlash

    def joint_state_callback(self, msg):
        if self.joint_name_to_index is None:
            self.joint_names = list(msg.name)
            self.joint_name_to_index = {name: i for i, name in enumerate(msg.name)}

            self.get_logger().info(f"🦾 Found joints:")
            for name in self.joint_names:
                self.get_logger().info(f"---- {name}")

        self.latest_positions = msg.position

    def get_q(self):
        if self.latest_positions is None: return None
        return self.latest_positions

    def wait_for_joint_states(self, timeout_sec=5.0):
        start = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.latest_positions is not None and self.joint_name_to_index is not None):
                return
            if time.time() - start > timeout_sec:
                raise RuntimeError("TImed out waiting for /joint_states")

    def publish_joint_configuration(self, positions, move_time_sec=2):
        msg = JointTrajectory()
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start.sec = move_time_sec
        msg.joint_names = list(self.joint_name_to_index.keys())
        msg.points.append(point)
        self.joint_trajectory_pub.publish(msg)

    def probe_backlash(self, base_q, joint_name, joint_idx, sign):
        # Hard-coded values... subject to configuration
        preload_deg, step_deg, max_deg = -10.0 * sign, 1.0, 8.0
        
        # Preload joint for inspection
        preload_q = base_q.copy()
        preload_q[joint_idx] += np.deg2rad(preload_deg)
        self.publish_joint_configuration(preload_q)
        input(f"Preloaded {joint_name} at {preload_deg:.1f} degrees. Press ENTER to continue.")

        for offset_deg in np.arange(step_deg, max_deg + step_deg, step_deg):
            q = preload_q.copy()
            q[joint_idx] += np.deg2rad(offset_deg * sign)

            self.publish_joint_configuration(q)
            self.get_logger().info(f"Testing J5 offset = {offset_deg:.2f} deg")
            answer = input("Did the wrist visibly move? [y = yes, ENTER = continue]")

            if answer == "y":
                self.get_logger().info(f"Positive threshold = {offset_deg:.2f} deg")
                return offset_deg
    
        self.get_logger().info("Backlash detection inconclusive... returning 0")
        return 0

    def diagnose_backlash(self):
        joint_name = self.get_parameter("target_joint").value
        self.get_logger().info(f"🩺 Running backlash diagnosis on {joint_name}")

        # Read states and identify joint
        self.wait_for_joint_states()
        base_q = list(self.get_q())
        joint_idx = self.joint_name_to_index[joint_name]
        
        # Probe in positive and negative directions
        pos_b = self.probe_backlash(base_q, joint_name, joint_idx, +1)
        neg_b = self.probe_backlash(base_q, joint_name, joint_idx, -1)

        self.record_backlash(joint_name, joint_idx, pos_b, neg_b)

        self.get_logger().info(f"✅ Finished diagnosing {joint_name}")

    def record_backlash(self, joint_name, joint_idx, pos_b, neg_b):

        file_path = "../config/backlash.json"

        if os.path.exists(file_path):
            with open(file_path, "r", encoding="utf-8") as file:
                data = json.load(file)
        else:
            data = {}

        data[joint_name] = {
            "joint_idx": joint_idx,
            "pos_b": pos_b,
            "neg_b": neg_b,
            "tot_b": pos_b + neg_b
        }

        with open(file_path, "w", encoding="utf-8") as file:
            json.dump(data, file, indent=4, ensure_ascii=False)

    def compensate_backlash_j5(self, trajectory, backlash_degree=8.0, epsilon_deg=0.01):
        
        # Hard-coded stuff for testing just for now...
        jt = trajectory.joint_trajectory
        j5_idx = jt.joint_names.index("joint_5")

        backlash_rad = np.deg2rad(backlash_degree)
        epsilon_rad = np.deg2rad(epsilon_deg)

        previous_direction = 0

        points = jt.points

        # for p in points:
        #   rad = p.positions[j5_idx]
        #   self.get_logger().info(f"J5 goes to: {rad}")

        for i in range(1, len(points)):
            current = points[i].positions[j5_idx]
            previous = points[i - 1].positions[j5_idx]

            delta = current - previous

            # self.get_logger().info(f"{current} - {previous} = {delta}")

            if abs(delta) < epsilon_rad:
                continue

            direction = np.sign(delta)

            # self.get_logger().info(f"delta: {delta} & direction: {str(direction)}")

            if (previous_direction != 0 and direction != previous_direction):
                corrected = list(points[i].positions)
                corrected[j5_idx] += direction * backlash_rad / 2
                points[i].positions = corrected

                self.get_logger().info("🔄 Reversing!")

            previous_direction = direction
        
        return trajectory

commands = {
    "calibrate": lambda node: node.calibrate(),
    "move_relative_to_current": lambda node: node.move_relative_to_current(),
    "execute_print": lambda node: node.execute_print(),
    "orient": lambda node: node.orient(),
    "save_current_pose": lambda node: node.save_current_pose(),
    "park": lambda node: node.assume_pose(),
    "diagnose_backlash": lambda node: node.diagnose_backlash()
}

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--command", type=str, default='orient', choices=commands.keys())
    parsed_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = InVitroPlanner()

    rclpy.spin_once(node, timeout_sec=0.1)

    commands[parsed_args.command](node)
    
    node.destroy_node()

    # nano_serial.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()