#!/usr/bin/env python3

import rclpy
from threading import Thread
from ur3e_control_package.move_command import DynamicTrajectoryExecutor
import time
import ble_measurement as ble

class IntegratedController:
    def __init__(self):
        # Initialize MAX30009 parameters
        self.f_exc = 75  # kHz {60, 75, 100}
        self.I_rms = 128  # uA {64, 128, 256}
        self.board = None
        self.calib_params = None
        
        # Initialize ROS components
        rclpy.init()
        self.executor = DynamicTrajectoryExecutor()
        self.executor_thread = Thread(target=rclpy.spin, args=(self.executor,), daemon=True)
        self.executor_thread.start()

    def initialize_max30009(self):
        """Initialize and calibrate the MAX30009 board"""
        self.board = ble.MAX30009("JSearch-Laptop")
        connect_success = self.board.connect(5)
        
        if not connect_success:
            raise Exception("Failed to connect to MAX30009")
            
        write_fails = self.board.initialize(self.f_exc, self.I_rms)
        print(f"Freq = {self.board.meas_freq} kHz")
        print(f"Amp = {self.board.meas_amp} uA_rms")
        
        # Perform internal calibration
        init_success, write_success, calib_success = self.board.internal_calibrate(900, 250, 0.5)
        
        if not all([init_success, write_success, calib_success]):
            raise Exception("Calibration failed")
            
        # Store calibration parameters
        I_off = self.board.I_offset
        I_coeff = self.board.I_coeff
        I_pha_coeff = self.board.I_pha_coeff
        Q_off = self.board.Q_offset
        Q_coeff = self.board.Q_coeff
        Q_pha_coeff = self.board.Q_pha_coeff
        
        self.calib_params = [I_off[0], I_coeff, I_pha_coeff, Q_off[0], Q_coeff, Q_pha_coeff]

    def take_measurement(self):
        """Take a single measurement and process it"""
        if not self.board or not self.calib_params:
            raise Exception("MAX30009 not initialized")
            
        samp_meas, meas_success = self.board.measurement_sample(1000, 0.5)
        if meas_success:
            processed_meas = ble.raw2imp_data(samp_meas, self.I_rms*1e-6, "mag/phase", 
                                            calib_params=self.calib_params)
            return processed_meas
        return None

    def generate_square_sweep_waypoints(self, width, height, z_height, discretize_step, WAYPOINTS):
        """Generate waypoints for a square sweep pattern"""
        waypoints = []
        identity_quat = [1, 0, 0, 0]
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

    def run_measurement_sequence(self):
        """Execute the full measurement sequence"""
        try:
            # Initialize MAX30009
            self.initialize_max30009()
            
            # Define initial waypoints and joint configuration
            WAYPOINTS = [
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
            
            # Generate sweep waypoints
            sweep_waypoints = self.generate_square_sweep_waypoints(
                width=0.1, height=0.1, z_height=-0.02, discretize_step=50, waypoints = WAYPOINTS
            )
            
            # Execute initial movement
            self.executor.move_to_first_waypoint(joint_config)
            self.executor.compute_and_execute_trajectory(WAYPOINTS, dt=0.2)
            
            # Execute sweep pattern with measurements
            count = 0
            measurements = []
            
            for waypoint in sweep_waypoints:
                single_point = [waypoint]
                count += 1
                
                # Take measurement every third point (when robot is stationary)
                if count % 3 == 0:
                    # Take measurement during the pause
                    measurement = self.take_measurement()
                    if measurement:
                        measurements.append({
                            'position': waypoint[0],
                            'measurement': measurement
                        })
                    time.sleep(1)  # Keep the original sleep
                
                self.executor.compute_and_execute_trajectory(single_point, dt=0.2)
            
            return measurements
            
        finally:
            # Cleanup
            if self.board:
                self.board.disconnect()
            rclpy.shutdown()
            self.executor_thread.join()
            self.executor.csv_file.close()

def main():
    controller = IntegratedController()
    measurements = controller.run_measurement_sequence()
    
    # Process or save measurements as needed
    print(f"Completed {len(measurements)} measurements")

if __name__ == "__main__":
    main()