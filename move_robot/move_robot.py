#!/usr/bin/env python3
from scipy.spatial.transform import Rotation as R
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
from rclpy.qos import QoSProfile

WAYPOINTS = [
    # "W"
    [[0.35, 0.14, 0.25], [0.5, 0.5, 0.5, 0.5]],  # Start of W
    [[0.35, 0.11, 0.15], [0.5, 0.5, 0.5, 0.5]],  # Mid bottom of W
    [[0.35, 0.08, 0.25], [0.5, 0.5, 0.5, 0.5]],  # Mid top of W
    [[0.35, 0.05, 0.15], [0.5, 0.5, 0.5, 0.5]],  # Mid bottom of W
    [[0.35, 0.02, 0.25], [0.5, 0.5, 0.5, 0.5]],  # End of W

    [[0.3, 0.02, 0.25], [0.5, 0.5, 0.5, 0.5]],
    [[0.3, -0.1, 0.25], [0.5, 0.5, 0.5, 0.5]],

    # Moving to start of "C"
    [[0.35, -0.1, 0.25], [0.5, 0.5, 0.5, 0.5]],    # Start of C
    [[0.35, -0.03, 0.25], [0.5, 0.5, 0.5, 0.5]],   # Top left of C
    [[0.35, -0.03, 0.15], [0.5, 0.5, 0.5, 0.5]],   # Bottom left of C
    [[0.35, -0.1, 0.15], [0.5, 0.5, 0.5, 0.5]],    # Bottom right of C
]

home_position = [0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]
REAL_VELOCITY = 0.1


def main():
    rclpy.init()

    node = Node("moveit_waypoints_example")

    node.declare_parameter("position", [0.0, 0.22315, 0.69395])
    node.declare_parameter("quat_xyzw", [-0.7071, 0.0, 0.0, 0.7071])
    node.declare_parameter("synchronous", True)
    node.declare_parameter("cancel_after_secs", 0.0)
    node.declare_parameter("planner_id", "RRTConfigDefault")
    node.declare_parameter("cartesian", False)
    node.declare_parameter("cartesian_max_step", 0.0025)
    node.declare_parameter("cartesian_fraction_threshold", 0.0)
    node.declare_parameter("cartesian_jump_threshold", 0.0)
    node.declare_parameter("cartesian_avoid_collisions", False)

    callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=ur3e.joint_names(),
        base_link_name=ur3e.base_link_name(),
        end_effector_name=ur3e.end_effector_name(),
        group_name=ur3e.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )
    moveit2.planner_id = node.get_parameter("planner_id").get_parameter_value().string_value

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    node.create_rate(1.0).sleep()

    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    synchronous = node.get_parameter("synchronous").get_parameter_value().bool_value
    cancel_after_secs = node.get_parameter("cancel_after_secs").get_parameter_value().double_value
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value
    cartesian_max_step = node.get_parameter("cartesian_max_step").get_parameter_value().double_value
    cartesian_fraction_threshold = node.get_parameter("cartesian_fraction_threshold").get_parameter_value().double_value
    cartesian_jump_threshold = node.get_parameter("cartesian_jump_threshold").get_parameter_value().double_value
    cartesian_avoid_collisions = node.get_parameter("cartesian_avoid_collisions").get_parameter_value().bool_value

    moveit2.cartesian_avoid_collisions = cartesian_avoid_collisions
    moveit2.cartesian_jump_threshold = cartesian_jump_threshold

    current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"joint_states_{current_time}.csv"
    csv_file = open(csv_filename, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    header = ["time"] + [f"{joint}_{attr}" for joint in ur3e.joint_names() for attr in ["position", "velocity", "effort"]]
    header += ["force_x", "force_y", "force_z", "torque_x", "torque_y", "torque_z"]
    csv_writer.writerow(header)

    joint_data = {name: {"position": None, "velocity": None, "effort": None} for name in ur3e.joint_names()}
    force_torque_data = {"force": {"x": None, "y": None, "z": None}, "torque": {"x": None, "y": None, "z": None}}

    def joint_state_callback(msg):
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        row = [current_time]

        for i, name in enumerate(msg.name):
            if name in joint_data:
                joint_data[name]["position"] = msg.position[i] if i < len(msg.position) else None
                joint_data[name]["velocity"] = msg.velocity[i] if i < len(msg.velocity) else None
                joint_data[name]["effort"] = msg.effort[i] if i < len(msg.effort) else None

        for joint in ur3e.joint_names():
            data = joint_data[joint]
            row.extend([data["position"], data["velocity"], data["effort"]])

        row.extend([
            force_torque_data["force"]["x"], force_torque_data["force"]["y"], force_torque_data["force"]["z"],
            force_torque_data["torque"]["x"], force_torque_data["torque"]["y"], force_torque_data["torque"]["z"]
        ])

        csv_writer.writerow(row)

    def wrench_callback(msg):
        force_torque_data["force"]["x"] = msg.wrench.force.x
        force_torque_data["force"]["y"] = msg.wrench.force.y
        force_torque_data["force"]["z"] = msg.wrench.force.z
        force_torque_data["torque"]["x"] = msg.wrench.torque.x
        force_torque_data["torque"]["y"] = msg.wrench.torque.y
        force_torque_data["torque"]["z"] = msg.wrench.torque.z

    joint_state_sub = node.create_subscription(JointState, 'joint_states', joint_state_callback, QoSProfile(depth=10))
    wrench_sub = node.create_subscription(WrenchStamped, 'force_torque_sensor_broadcaster/wrench', wrench_callback, QoSProfile(depth=10))

    moveit2.add_collision_box(
        id='floor', position=[0.0, 0.0, -0.01], quat_xyzw=[0.0, 0.0, 0.0, 1.0], size=[2.0, 2.0, 0.001]
    )
    time.sleep(1)
    moveit2.add_collision_box(
        id='whiteboard', position=[0.43, 0.0, 0.2], quat_xyzw=[0.0, 0.0, 0.0, 1.0], size=[0.001, 0.4, 0.4]
    )
    time.sleep(1)

    # Get the current pose of the end effector
    fk_pose = moveit2.compute_fk()

    pen_position = [
        fk_pose.pose.position.x,
        fk_pose.pose.position.y,
        fk_pose.pose.position.z
    ]

    # Original pen orientation
    pen_orientation = [
        fk_pose.pose.orientation.x,
        fk_pose.pose.orientation.y,
        fk_pose.pose.orientation.z,
        fk_pose.pose.orientation.w
    ]

    # Quaternion representing 90 degree rotation around z-axis
    rotation_quaternion = R.from_euler('z', 90, degrees=True).as_quat()

    # Apply the rotation to the original orientation
    original_orientation = R.from_quat(pen_orientation)
    rotated_orientation = R.from_quat(rotation_quaternion)
    pen_orientation_rotated = (rotated_orientation * original_orientation).as_quat()

    # moveit2.add_collision_mesh(
    #     id='pen',
    #     filepath="models/marker.stl",
    #     position=pen_position,
    #     quat_xyzw=pen_orientation_rotated,
    #     scale=0.001
    # )
    # time.sleep(0.5)
    # moveit2.attach_collision_object(id='pen', weight=0.0)
    # time.sleep(0.5)

    moveit2.max_velocity = 0.3

    

    def move_to_waypoint(waypoints_input):
        node.get_logger().info("Moving to waypoint.")
        waypoint_position, waypoint_orientation = waypoints_input[0]

        position = Point(x=waypoint_position[0], y=waypoint_position[1], z=waypoint_position[2])
        quat_xyzw = Quaternion(
            x=waypoint_orientation[0],
            y=waypoint_orientation[1],
            z=waypoint_orientation[2],
            w=waypoint_orientation[3]
        )

        # Compute the joint configuration for the given pose
        joint_state = moveit2.compute_ik(position, quat_xyzw)
        if joint_state is not None:
            joint_config = list(joint_state.position)
            moveit2.move_to_configuration(joint_config)
            moveit2.wait_until_executed()
            node.get_logger().info("Reached waypoint.")
        else:
            node.get_logger().warn("Failed to compute IK for the first waypoint.")

    def interpolate_waypoints(waypoints, increment):
        interpolated_points = []
        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]
            distance = np.linalg.norm(np.array(end[0]) - np.array(start[0]))
            num_interpolations = int(distance / increment)
            for t in range(num_interpolations):
                alpha = t / num_interpolations
                position = [(1 - alpha) * start[0][j] + alpha * end[0][j] for j in range(3)]
                orientation = [(1 - alpha) * start[1][j] + alpha * end[1][j] for j in range(4)]
                interpolated_points.append([position, orientation])
        interpolated_points.append(waypoints[-1])
        return interpolated_points


    def move_to_interpolated_waypoints(waypoints_input, dt):
        interpolated_waypoints = interpolate_waypoints(waypoints_input, dt)
        node.get_logger().info(f"Moving to interpolated waypoints with dt={dt}.")
        for idx, waypoint in enumerate(interpolated_waypoints):
            waypoint_position, waypoint_orientation = waypoint

            position = Point(x=waypoint_position[0], y=waypoint_position[1], z=waypoint_position[2])
            quat_xyzw = Quaternion(
                x=waypoint_orientation[0],
                y=waypoint_orientation[1],
                z=waypoint_orientation[2],
                w=waypoint_orientation[3]
            )

            # Compute the joint configuration for the given pose
            joint_state = moveit2.compute_ik(position, quat_xyzw)
            if joint_state is not None:
                joint_config = list(joint_state.position)
                moveit2.move_to_configuration(joint_config)
                moveit2.wait_until_executed()
                node.get_logger().info(f"Reached interpolated waypoint {idx + 1}")
            else:
                node.get_logger().warn(f"Failed to compute IK for interpolated waypoint {idx + 1}")

            time.sleep(0.01)

        node.get_logger().info("Finished moving to interpolated waypoints.")

    move_to_waypoint(WAYPOINTS)
    move_to_interpolated_waypoints(WAYPOINTS, dt=0.005)  # Example value for dt

    rclpy.shutdown()
    executor_thread.join()
    csv_file.close()
    exit(0)

if __name__ == "__main__":
    main()
