#!/usr/bin/env python3

import math
import time
import rclpy
import csv
import numpy as np
import ast
from datetime import datetime
from threading import Thread
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur3e
from geometry_msgs.msg import PoseStamped, Quaternion, WrenchStamped, Point, Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import QoSProfile
from builtin_interfaces.msg import Duration
from typing import List
from ur_ikfast import ur_kinematics

class DynamicTrajectoryExecutor(Node):
    def __init__(self):
        super().__init__("dynamic_trajectory_executor")

        # Declare and get parameters
        self.declare_and_get_parameters()

        # Create a callback group for concurrent processing
        callback_group = ReentrantCallbackGroup()

        # Initialize MoveIt2 interface for robot control
        self.setup_moveit2(callback_group)

        # Set up publisher for joint trajectory commands
        self.setup_joint_trajectory_publisher()

        # Set up data logging for joint states and force/torque readings
        self.setup_data_logging()

        # Set maximum velocity for the robot in moveit2
        self.moveit2.max_velocity = 0.1

    def declare_and_get_parameters(self):
        """Declare and retrieve all necessary parameters for the node."""
        # Declare parameters with default values
        self.declare_parameter("waypoints", "[[[0.35, 0.14, 0.25], [0.5, 0.5, 0.5, 0.5]]]")
        self.declare_parameter("velocity", 0.02)
        self.declare_parameter("real_robot", False)
        self.declare_parameter("dt", 0.2)
        self.declare_parameter("frame", "world")
        self.declare_parameter("initial_joint_pos", 
                               [-1.9991989999999866,-1.835606000000002,-2.0968710000000046,
                                -2.349238999999997,-0.4251269999999927,-0.0012429999999703512])
        self.declare_parameter("initial_move", True)
        self.declare_parameter("synchronous", True)
        self.declare_parameter("cancel_after_secs", 0.0)
        self.declare_parameter("planner_id", "RRTConfigDefault")
        self.declare_parameter("cartesian", False)
        self.declare_parameter("cartesian_max_step", 0.0025)
        self.declare_parameter("cartesian_fraction_threshold", 0.0)
        self.declare_parameter("cartesian_jump_threshold", 0.0)
        self.declare_parameter("cartesian_avoid_collisions", False)

        # Get parameter values
        waypoints_str = self.get_parameter("waypoints").value
        self.waypoints = ast.literal_eval(waypoints_str)
        self.velocity = self.get_parameter("velocity").value
        self.real_robot = self.get_parameter("real_robot").value
        self.dt = self.get_parameter("dt").value
        self.frame = self.get_parameter("frame").value
        self.initial_joint_pos = self.get_parameter("initial_joint_pos").value
        self.initial_move = self.get_parameter("initial_move").value
        self.ur3e_arm = ur_kinematics.URKinematics('ur3e')

        '''  PID values for untested force control implementation      
        self.Kp = 0.001  # Proportional gain
        self.Ki = 0.0001  # Integral gain
        self.Kd = 0.00001  # Derivative gain
        self.integral = np.zeros(3)
        self.previous_error = np.zeros(3)
        self.desired_force = np.array([0, 0, -5])  
        '''

    def setup_moveit2(self, callback_group):
        """Initialize MoveIt2 interface for robot control."""
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ur3e.joint_names(),
            base_link_name=ur3e.base_link_name(),
            end_effector_name=ur3e.end_effector_name(),
            group_name=ur3e.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        self.moveit2.planner_id = self.get_parameter("planner_id").get_parameter_value().string_value

    def setup_joint_trajectory_publisher(self):
        """Set up publisher for sending joint trajectory commands to the robot."""
        topic = '/scaled_joint_trajectory_controller/joint_trajectory' if self.real_robot else '/joint_trajectory_controller/joint_trajectory'
        self.joint_trajectory_pub = self.create_publisher(JointTrajectory, topic, QoSProfile(depth=10))

    def setup_data_logging(self):
        """Set up CSV logging for joint states and force/torque data."""
        # Create CSV file with timestamp in filename
        current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f"joint_states_{current_time}.csv"
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write CSV header
        header = ["time"] + [f"{joint}_{attr}" for joint in ur3e.joint_names() for attr in ["position", "velocity", "effort"]]
        header += ["force_x", "force_y", "force_z", "torque_x", "torque_y", "torque_z"]
        self.csv_writer.writerow(header)

        # Initialize data storage
        self.joint_data = {name: {"position": None, "velocity": None, "effort": None} for name in ur3e.joint_names()}
        self.force_torque_data = {"force": {"x": None, "y": None, "z": None}, "torque": {"x": None, "y": None, "z": None}}

        # Set up subscribers for joint states and force/torque data
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, QoSProfile(depth=10))
        self.create_subscription(WrenchStamped, 'force_torque_sensor_broadcaster/wrench', self.wrench_callback, QoSProfile(depth=10))

    def joint_state_callback(self, msg):
        """Callback function to process and log joint state data."""
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        row = [current_time]

        # Update joint data
        for i, name in enumerate(msg.name):
            if name in self.joint_data:
                self.joint_data[name]["position"] = msg.position[i] if i < len(msg.position) else None
                self.joint_data[name]["velocity"] = msg.velocity[i] if i < len(msg.velocity) else None
                self.joint_data[name]["effort"] = msg.effort[i] if i < len(msg.effort) else None

        # Prepare row for CSV
        for joint in ur3e.joint_names():
            data = self.joint_data[joint]
            row.extend([data["position"], data["velocity"], data["effort"]])

        # Add force/torque data
        row.extend([
            self.force_torque_data["force"]["x"], self.force_torque_data["force"]["y"], self.force_torque_data["force"]["z"],
            self.force_torque_data["torque"]["x"], self.force_torque_data["torque"]["y"], self.force_torque_data["torque"]["z"]
        ])

        # Write to CSV
        self.csv_writer.writerow(row)

    def wrench_callback(self, msg):
        """Callback function to process force/torque sensor data."""
        self.force_torque_data["force"]["x"] = msg.wrench.force.x
        self.force_torque_data["force"]["y"] = msg.wrench.force.y
        self.force_torque_data["force"]["z"] = msg.wrench.force.z
        self.force_torque_data["torque"]["x"] = msg.wrench.torque.x
        self.force_torque_data["torque"]["y"] = msg.wrench.torque.y
        self.force_torque_data["torque"]["z"] = msg.wrench.torque.z

    def setup_collision_objects(self):
        """Set up collision objects in the planning scene."""
        # Add floor as a collision object
        self.moveit2.add_collision_box(
            id='floor', position=[0.0, 0.0, -0.01], quat_xyzw=[0.0, 0.0, 0.0, 1.0], size=[2.0, 2.0, 0.001]
        )
        time.sleep(1)

        # Get current end-effector pose
        fk_pose = self.moveit2.compute_fk()
        time.sleep(0.1)

        # Extract position and orientation
        pen_position = [
            fk_pose.pose.position.x,
            fk_pose.pose.position.y,
            fk_pose.pose.position.z
        ]
        pen_orientation = [
            fk_pose.pose.orientation.x,
            fk_pose.pose.orientation.y,
            fk_pose.pose.orientation.z,
            fk_pose.pose.orientation.w
        ]

        # Add pen as a collision object
        #self.moveit2.add_collision_box(
        #    id='pen', position=pen_position, quat_xyzw=pen_orientation, size=[0.03, 0.03, 0.18]
        #)
        time.sleep(0.1)

        # Attach the pen to the robot
        #self.moveit2.attach_collision_object(id='pen', weight=0.0)
        time.sleep(0.1)
        return pen_position, pen_orientation

    def interpolate_waypoint(self, start, end, alpha):
        """Interpolate between start and end poses based on alpha (0 to 1)."""
        position = [(1 - alpha) * start[0][j] + alpha * end[0][j] for j in range(3)]
        orientation = [(1 - alpha) * start[1][j] + alpha * end[1][j] for j in range(4)]
        return [position, orientation]

    def compute_and_execute_trajectory(self, waypoints, dt):
        """Compute and execute a trajectory through the given waypoints."""
        self.get_logger().info(f"Computing and executing trajectory with dt={dt}.")
        
        # Add the current pose as the first waypoint
        waypoints = list(waypoints)
        time.sleep(0.1)
        current_pose = self.moveit2.compute_fk()
        
        if current_pose is not None:
            position = [current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z]
            orientation = [current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w]
            waypoints.insert(0, [position, orientation])
        else:
            self.get_logger().error("Failed to compute FK for current joint states.")

        # Get current joint positions for initial guess
        current_joint_states = self.get_joint_states()
        current_joints = [current_joint_states[joint]["position"] for joint in ur3e.joint_names()]

        # Iterate through waypoints
        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]
            distance = np.linalg.norm(np.array(end[0]) - np.array(start[0]))
            num_interpolations = int(distance / (self.velocity * dt))
            
            for t in range(num_interpolations):
                start_time = time.time()
                
                alpha = t / num_interpolations
                interpolated_point = self.interpolate_waypoint(start, end, alpha)

                # Compute inverse kinematics using URKinematics
                pose_quat = interpolated_point[0] + interpolated_point[1]  # Combine position and orientation
                joint_positions = self.ur3e_arm.inverse(pose_quat, False, q_guess=current_joints)
                
                if joint_positions is None:
                    self.get_logger().warn("URKinematics IK failed. Falling back to MoveIt.")
                    joint_positions = self.moveit2.compute_ik(interpolated_point[0], interpolated_point[1])
                
                if joint_positions is None:
                    self.get_logger().error(f"Both URKinematics and MoveIt IK failed for interpolated point")
                    continue
                
                # Create and publish joint trajectory
                joint_trajectory = JointTrajectory()
                joint_trajectory.joint_names = ur3e.joint_names()
                
                point = JointTrajectoryPoint()
                # Handle different types of joint_positions (numpy array, list, or object with position attribute)
                if isinstance(joint_positions, np.ndarray):
                    point.positions = joint_positions.tolist()
                elif isinstance(joint_positions, list):
                    point.positions = joint_positions
                else:
                    point.positions = joint_positions.position
                point.time_from_start = Duration(sec=0, nanosec=int(dt * 1e9))
                
                joint_trajectory.points.append(point)
                
                self.joint_trajectory_pub.publish(joint_trajectory)
                
                # Update current_joints for next iteration
                current_joints = point.positions
                
                # Wait for the next cycle
                computation_time = time.time() - start_time
                sleep_time = max(0, dt - computation_time)
                print(computation_time)
                time.sleep(sleep_time)

        self.get_logger().info("Finished executing trajectory.")

    def move_to_first_waypoint(self, joint_configuration):
        """Move the robot to the initial joint configuration."""
        self.get_logger().info("Moving to first waypoint.")
        time.sleep(1)

        #self.setup_collision_objects()

        # Move to the provided joint configuration
        self.moveit2.move_to_configuration(joint_configuration)
        self.moveit2.wait_until_executed()
        self.get_logger().info("Reached first waypoint.")
        return True

    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions."""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        ]

    def quaternion_to_rotation_matrix(self, q):
        """Convert a quaternion to a rotation matrix."""
        x, y, z, w = q
        return np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
        ])

    def end_effector_frame_tf(self, waypoints: List[List[List[float]]]) -> List[Pose]:
        """
        Transform waypoints from the end effector frame to the world frame.
        
        :param waypoints: List of waypoints, where each waypoint is a list containing
                        [position, orientation] lists.
        :return: List of poses transformed to the world frame.
        """
        # Compute the forward kinematics to get the end effector pose in the world frame
        time.sleep(1)
        end_effector_pose = self.moveit2.compute_fk()

        # Extract translation and rotation from the end effector pose
        ee_translation = np.array([
            end_effector_pose.pose.position.x,
            end_effector_pose.pose.position.y,
            end_effector_pose.pose.position.z
        ])

        ee_orientation = np.array([
            end_effector_pose.pose.orientation.x,
            end_effector_pose.pose.orientation.y,
            end_effector_pose.pose.orientation.z,
            end_effector_pose.pose.orientation.w
        ])

        # Create a homogeneous transformation matrix for the end effector
        ee_rotation_matrix = self.quaternion_to_rotation_matrix(ee_orientation)
        ee_transformation_matrix = np.eye(4)
        ee_transformation_matrix[:3, :3] = ee_rotation_matrix
        ee_transformation_matrix[:3, 3] = ee_translation

        transformed_waypoints = []
        for waypoint in waypoints:
            position, orientation = waypoint
            
            # Convert the waypoint position to a numpy array
            waypoint_position = np.array([position[0], position[1], position[2], 1])

            # Apply the transformation
            world_position = np.dot(ee_transformation_matrix, waypoint_position)[:3]

            # Rotate the orientation
            waypoint_orientation = np.array(orientation)
            world_orientation = self.quaternion_multiply(ee_orientation, waypoint_orientation)

            # Create the transformed Pose
            transformed_pose = [world_position, world_orientation]

            transformed_waypoints.append(transformed_pose)

        return transformed_waypoints

    def get_joint_states(self):
        """
        Get the current joint states.
        
        :return: A dictionary containing position, velocity, and effort for each joint.
        """
        return {
            joint: {
                "position": self.joint_data[joint]["position"],
                "velocity": self.joint_data[joint]["velocity"],
                "effort": self.joint_data[joint]["effort"]
            }
            for joint in ur3e.joint_names()
        }

    def get_force_torque(self):
        """
        Get the current force and torque readings.
        
        :return: A dictionary containing force and torque data.
        """
        return {
            "force": {
                "x": self.force_torque_data["force"]["x"],
                "y": self.force_torque_data["force"]["y"],
                "z": self.force_torque_data["force"]["z"]
            },
            "torque": {
                "x": self.force_torque_data["torque"]["x"],
                "y": self.force_torque_data["torque"]["y"],
                "z": self.force_torque_data["torque"]["z"]
            }
        }
''' EXPERIMENTAL, NOT TESTED
    def force_pid(self, interpolated_point, force_feedback, direction_quat):
        """
        Adjust the interpolated point based on force feedback using a PID controller.
        
        :param interpolated_point: Current interpolated point [position, orientation]
        :param force_feedback: Force feedback from get_force_torque()
        :param direction_quat: Quaternion representing the direction of motion
        :return: Adjusted point
        """
        # Extract current force
        current_force = np.array([
            force_feedback['force']['x'],
            force_feedback['force']['y'],
            force_feedback['force']['z']
        ])

        error = self.desired_force - current_force

        self.integral += error

        derivative = error - self.previous_error

        adjustment = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        rotation_matrix = self.quaternion_to_rotation_matrix(direction_quat)
        world_adjustment = rotation_matrix.dot(adjustment)

        adjusted_point = np.array(interpolated_point[0]) + world_adjustment

        self.previous_error = error

        return adjusted_point.tolist()
'''

def main():
    rclpy.init()

    executor = DynamicTrajectoryExecutor()
    
    executor_thread = Thread(target=rclpy.spin, args=(executor,), daemon=True)
    executor_thread.start()

    if executor.initial_move:
        executor.move_to_first_waypoint(executor.initial_joint_pos)
    else:
        if executor.frame == "world":
            executor.compute_and_execute_trajectory(executor.waypoints, dt=executor.dt)
        else:
            executor.compute_and_execute_trajectory(executor.end_effector_frame_tf(executor.waypoints), dt=executor.dt)

    rclpy.shutdown()
    executor_thread.join()
    executor.csv_file.close()
    exit(0)

if __name__ == "__main__":
    main()