#!/usr/bin/env python3

from threading import Thread
import math
import time
import rclpy
import csv
import numpy as np
from datetime import datetime
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2, MoveIt2State
from pymoveit2.robots import ur3e
from geometry_msgs.msg import PoseStamped, Quaternion, WrenchStamped, Point
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import QoSProfile
from builtin_interfaces.msg import Duration

WAYPOINTS = [
    # "W"
    [[0.3, 0.14, 0.15], [0.5, 0.5, 0.5, 0.5]],

    [[0.35, 0.14, 0.25], [0.5, 0.5, 0.5, 0.5]],  # Start of W
    [[0.35, 0.11, 0.15], [0.5, 0.5, 0.5, 0.5]],  # Mid bottom of W
    [[0.35, 0.08, 0.25], [0.5, 0.5, 0.5, 0.5]],  # Mid top of W
    [[0.35, 0.05, 0.15], [0.5, 0.5, 0.5, 0.5]],  # Mid bottom of W
    [[0.35, 0.02, 0.25], [0.5, 0.5, 0.5, 0.5]],  # End of W

    [[0.33, 0.02, 0.25], [0.5, 0.5, 0.5, 0.5]],
    [[0.33, -0.1, 0.25], [0.5, 0.5, 0.5, 0.5]],

    # Moving to start of "C"
    [[0.35, -0.1, 0.25], [0.5, 0.5, 0.5, 0.5]],    # Start of C
    [[0.35, -0.03, 0.25], [0.5, 0.5, 0.5, 0.5]],   # Top left of C
    [[0.35, -0.03, 0.15], [0.5, 0.5, 0.5, 0.5]],   # Bottom left of C
    [[0.35, -0.1, 0.15], [0.5, 0.5, 0.5, 0.5]],
    [[0.33, -0.1, 0.15], [0.5, 0.5, 0.5, 0.5]],    # Bottom right of C
]

home_position = [0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]
REAL_VELOCITY = 0.02

class DynamicTrajectoryExecutor(Node):
    def __init__(self):
        super().__init__("dynamic_trajectory_executor")

        self.declare_parameter("position", [0.0, 0.22315, 0.69395])
        self.declare_parameter("quat_xyzw", [-0.7071, 0.0, 0.0, 0.7071])
        self.declare_parameter("synchronous", True)
        self.declare_parameter("cancel_after_secs", 0.0)
        self.declare_parameter("planner_id", "RRTConfigDefault")
        self.declare_parameter("cartesian", False)
        self.declare_parameter("cartesian_max_step", 0.0025)
        self.declare_parameter("cartesian_fraction_threshold", 0.0)
        self.declare_parameter("cartesian_jump_threshold", 0.0)
        self.declare_parameter("cartesian_avoid_collisions", False)

        callback_group = ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ur3e.joint_names(),
            base_link_name=ur3e.base_link_name(),
            end_effector_name=ur3e.end_effector_name(),
            group_name=ur3e.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        self.moveit2.planner_id = self.get_parameter("planner_id").get_parameter_value().string_value

        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory, 
            '/scaled_joint_trajectory_controller/joint_trajectory', 
            QoSProfile(depth=10)
        )

        self.setup_data_logging()
        time.sleep(0.5)
        self.setup_collision_objects()
        time.sleep(0.5)


        self.moveit2.max_velocity = 0.1

    def setup_data_logging(self):
        current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f"joint_states_{current_time}.csv"
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        header = ["time"] + [f"{joint}_{attr}" for joint in ur3e.joint_names() for attr in ["position", "velocity", "effort"]]
        header += ["force_x", "force_y", "force_z", "torque_x", "torque_y", "torque_z"]
        self.csv_writer.writerow(header)

        self.joint_data = {name: {"position": None, "velocity": None, "effort": None} for name in ur3e.joint_names()}
        self.force_torque_data = {"force": {"x": None, "y": None, "z": None}, "torque": {"x": None, "y": None, "z": None}}

        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, QoSProfile(depth=10))
        self.create_subscription(WrenchStamped, 'force_torque_sensor_broadcaster/wrench', self.wrench_callback, QoSProfile(depth=10))

    def joint_state_callback(self, msg):
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        row = [current_time]

        for i, name in enumerate(msg.name):
            if name in self.joint_data:
                self.joint_data[name]["position"] = msg.position[i] if i < len(msg.position) else None
                self.joint_data[name]["velocity"] = msg.velocity[i] if i < len(msg.velocity) else None
                self.joint_data[name]["effort"] = msg.effort[i] if i < len(msg.effort) else None

        for joint in ur3e.joint_names():
            data = self.joint_data[joint]
            row.extend([data["position"], data["velocity"], data["effort"]])

        row.extend([
            self.force_torque_data["force"]["x"], self.force_torque_data["force"]["y"], self.force_torque_data["force"]["z"],
            self.force_torque_data["torque"]["x"], self.force_torque_data["torque"]["y"], self.force_torque_data["torque"]["z"]
        ])

        self.csv_writer.writerow(row)

    def wrench_callback(self, msg):
        self.force_torque_data["force"]["x"] = msg.wrench.force.x
        self.force_torque_data["force"]["y"] = msg.wrench.force.y
        self.force_torque_data["force"]["z"] = msg.wrench.force.z
        self.force_torque_data["torque"]["x"] = msg.wrench.torque.x
        self.force_torque_data["torque"]["y"] = msg.wrench.torque.y
        self.force_torque_data["torque"]["z"] = msg.wrench.torque.z

    def setup_collision_objects(self):
        time.sleep(0.5)
        self.moveit2.add_collision_box(
            id='floor', position=[0.0, 0.0, -0.01], quat_xyzw=[0.0, 0.0, 0.0, 1.0], size=[2.0, 2.0, 0.001]
        )
        # time.sleep(0.5)
        # self.moveit2.add_collision_box(
        #     id='whiteboard', position=[0.45, 0.0, 0.3], quat_xyzw=[0.0, 0.0, 0.0, 1.0], size=[0.001, 0.4, 0.4]
        # )
        time.sleep(0.5)
        self.get_logger().info("1done")
        
        

    def interpolate_waypoint(self, start, end, alpha):
        position = [(1 - alpha) * start[0][j] + alpha * end[0][j] for j in range(3)]
        orientation = [(1 - alpha) * start[1][j] + alpha * end[1][j] for j in range(4)]
        return [position, orientation]

    def compute_and_execute_trajectory(self, waypoints, dt):
        self.get_logger().info(f"Computing and executing trajectory with dt={dt}.")
        
        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]
            distance = np.linalg.norm(np.array(end[0]) - np.array(start[0]))
            num_interpolations = int(distance / (REAL_VELOCITY * dt))
            
            for t in range(num_interpolations):
                start_time = time.time()
                
                alpha = t / num_interpolations
                interpolated_point = self.interpolate_waypoint(start, end, alpha)
                
                joint_positions = self.moveit2.compute_ik(interpolated_point[0], interpolated_point[1])
                
                if joint_positions is None:
                    self.get_logger().error(f"IK failed for interpolated point")
                    continue
                
                joint_trajectory = JointTrajectory()
                joint_trajectory.joint_names = ur3e.joint_names()
                
                point = JointTrajectoryPoint()
                point.positions = joint_positions.position
                point.time_from_start = Duration(sec=0, nanosec=int(dt * 1e9))
                
                joint_trajectory.points.append(point)
                
                self.joint_trajectory_pub.publish(joint_trajectory)
                
                # force control logic
                # For example:
                # force_feedback = self.get_force_feedback()
                # adjusted_point = self.adjust_point_based_on_force(interpolated_point, force_feedback)
                
                computation_time = time.time() - start_time
                sleep_time = max(0, dt - computation_time)
                time.sleep(sleep_time)

        self.get_logger().info("Finished executing trajectory.")

    def move_to_first_waypoint(self, waypoints):
        self.get_logger().info("Moving to first waypoint.")

        fk_pose = self.moveit2.compute_fk()
        time.sleep(0.5)
        self.get_logger().info("2done")

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

        self.moveit2.add_collision_box(
            id='pen', position=pen_position, quat_xyzw=pen_orientation, size=[0.03, 0.03, 0.18]
        )
        time.sleep(0.5)
        self.moveit2.attach_collision_object(id='pen', weight=0.0)
        time.sleep(0.5)


        # Provided joint configuration
        joint_config = [
            -1.9991989999999866,  # shoulder_pan_joint
            -1.835606000000002,   # shoulder_lift_joint
            -2.0968710000000046,  # elbow_joint
            -2.349238999999997,   # wrist_1_joint
            -0.4251269999999927,  # wrist_2_joint
            -0.0012429999999703512 # wrist_3_joint
        ]

        time.sleep(1)
        self.moveit2.move_to_configuration(joint_config)
        self.moveit2.wait_until_executed()
        self.get_logger().info("Reached first waypoint.")
        return True

def main():
    rclpy.init()

    executor = DynamicTrajectoryExecutor()
    
    executor_thread = Thread(target=rclpy.spin, args=(executor,), daemon=True)
    executor_thread.start()
    
    if executor.move_to_first_waypoint(WAYPOINTS):
        executor.compute_and_execute_trajectory(WAYPOINTS, dt=0.5)
        #executor.get_logger().error("1Failed to move to first waypoint. Aborting execution.")
    else:
        executor.get_logger().error("Failed to move to first waypoint. Aborting execution.")

    rclpy.shutdown()
    executor_thread.join()
    executor.csv_file.close()
    exit(0)

if __name__ == "__main__":
    main()