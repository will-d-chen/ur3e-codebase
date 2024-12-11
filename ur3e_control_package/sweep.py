#!/usr/bin/env python3

import rclpy
from threading import Thread
from ur3e_control_package.move_command import DynamicTrajectoryExecutor
import time

WAYPOINTS = [
    # "W"
    [[-0.05, 0.35, 0.05], [1, 0, 0, 0]]

]

joint_config = [
    -1.6006999999999998,  # shoulder_pan_joint
    -1.7271,              # shoulder_lift_joint
    -2.2029999999999994,  # elbow_joint
    -0.8079999999999998,  # wrist_1_joint
    1.5951,               # wrist_2_joint
    -0.030999999999999694 # wrist_3_joint
]



def generate_square_sweep_waypoints(width, height, z_height, discretize_step):
    """
    Generate waypoints for a square sweep pattern, offset by the first WAYPOINTS position.
    
    :param width: Width of the square in meters
    :param height: Height of the square in meters
    :param z_height: Height of the sweep plane in meters
    :param discretize_step: Step size for discretization in millimeters
    :return: List of waypoints in the specified format
    """
    waypoints = []
    identity_quat = [1, 0, 0, 0]  # Updated to match the orientation in WAYPOINTS
    step = discretize_step / 1000  # Convert mm to meters
    
    # Get the offset from the first WAYPOINTS position
    offset_x, offset_y, offset_z = WAYPOINTS[0][0]

    # Calculate number of steps in each direction
    num_steps_x = int(width / step) + 1
    num_steps_y = int(height / step) + 1

    for i in range(num_steps_x):
        x = -i * step + offset_x
        if i % 2 == 0:  # Even rows: bottom to top
            y_range = range(num_steps_y)
        else:  # Odd rows: top to bottom
            y_range = range(num_steps_y - 1, -1, -1)
        
        for j in y_range:
            y = -j * step + offset_y
            
            # Add point at z_height (offset by base z position)
            waypoints.append([[x, y, -z_height + offset_z], identity_quat])
            
            # Add point at z=0 (offset by base z position)
            waypoints.append([[x, y, offset_z], identity_quat])
            
            # Add point back at z_height (offset by base z position)
            waypoints.append([[x, y, -z_height + offset_z], identity_quat])

    return waypoints

WAYPOINTS1 = generate_square_sweep_waypoints(width=0.1, height=0.1, z_height=-0.02, discretize_step=50)

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
    print(WAYPOINTS1)
    #time.sleep(1)
    #print(executor.get_joint_states())
    count = 0

    # Initialize the previous waypoint to None for the first iteration
    previous_waypoint = None

    for waypoint in WAYPOINTS1:
        if previous_waypoint is not None:
            # Calculate the difference between current and previous positions
            diff_x = waypoint[0][0] - previous_waypoint[0][0]
            diff_y = waypoint[0][1] - previous_waypoint[0][1]
            diff_z = waypoint[0][2] - previous_waypoint[0][2]
            
            # Create a single-point trajectory with the difference
            # Keep the quaternion from the current waypoint
            difference_point = [[diff_x, diff_y, diff_z], waypoint[1]]
            single_point = [difference_point]
            
            count = count + 1
            if count % 3 == 0:
                time.sleep(1)  # take measurement
                
            executor.compute_and_execute_trajectory(single_point, dt=0.2)
        
        # Update previous waypoint for the next iteration
        previous_waypoint = waypoint

    # Shutdown and cleanup
    rclpy.shutdown()
    executor_thread.join()
    executor.csv_file.close()

if __name__ == "__main__":
    main()
