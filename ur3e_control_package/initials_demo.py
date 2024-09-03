#!/usr/bin/env python3

import rclpy
from threading import Thread
from ur3e_control_package.move_command import DynamicTrajectoryExecutor

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

joint_config = [
            -1.9991989999999866,  # shoulder_pan_joint
            -1.835606000000002,   # shoulder_lift_joint
            -2.0968710000000046,  # elbow_joint
            -2.349238999999997,   # wrist_1_joint
            -0.4251269999999927,  # wrist_2_joint
            -0.0012429999999703512 # wrist_3_joint
        ]

def main():
    # Initialize rclpy
    rclpy.init()

    # Create an instance of your executor
    executor = DynamicTrajectoryExecutor()

    # Start the ROS 2 spin in a separate thread
    executor_thread = Thread(target=rclpy.spin, args=(executor,), daemon=True)
    executor_thread.start()
    
    # execute the movements
    executor.move_to_first_waypoint(joint_config)
    executor.compute_and_execute_trajectory(WAYPOINTS, dt=0.2)

    # Shutdown and cleanup
    rclpy.shutdown()
    executor_thread.join()
    executor.csv_file.close()

if __name__ == "__main__":
    main()
