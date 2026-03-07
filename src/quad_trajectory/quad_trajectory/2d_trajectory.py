import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import r2_score
import time
from ament_index_python.packages import get_package_share_directory
import os

class TwoDTrajectory(Node):
    def __init__(self):
        super().__init__('two_diamention')
        
        # 1. Load the MATLAB CSV
        try:
            # Find the path to the installed share directory
            package_share_directory = get_package_share_directory('quad_trajectory')
            csv_path = os.path.join(package_share_directory, 'config', 'dual_axis_trajectories.csv')
    
            self.df = pd.read_csv(csv_path)
            self.get_logger().info(f'CSV Loaded from: {csv_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load CSV: {str(e)}')
            return 

        # 2. Publishers & Subscribers
        # Changing to /setpoint with Float32MultiArray
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/setpoint', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # 3. State Machine Vars
        self.trajectories = ['Z_Line', 'Z_Sine', 'Z_Triangle']
        self.current_traj_idx = 0
        self.step_idx = 0
        self.state = "GO_HOME" 
        
        # Data for R^2 analysis
        self.actual_x, self.actual_z = [], []
        self.target_x, self.target_z = [], []
        self.curr_x, self.curr_z = 0.0, 0.0
        
        # 4. Control Loop (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.wait_start_time = None
        self.z_start = 5.0

    def odom_callback(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_z = msg.pose.pose.position.z
        
        if self.state == "EXECUTING":
            self.actual_x.append(self.curr_x)
            self.actual_z.append(self.curr_z)

    def send_setpoint(self, x, y, z):
        """Helper to format and publish Float32MultiArray"""
        msg = Float32MultiArray()
        msg.data = [float(x), float(y), float(z)]
        self.cmd_pub.publish(msg)

    def control_loop(self):
        if self.state == "GO_HOME":
            # Target [0, 0, 5]
            self.send_setpoint(0.0, 0.0, self.z_start)
            
            # Distance check to see if we are "Home"
            dist_to_home = np.sqrt(self.curr_x**2 + (self.curr_z - self.z_start)**2)
            if dist_to_home < 0.1:
                self.get_logger().info('At Home [0,0,5]. Waiting 10s...')
                self.state = "WAITING"
                self.wait_start_time = time.time()

        elif self.state == "WAITING":
            if time.time() - self.wait_start_time > 10.0:
                self.get_logger().info(f'Starting: {self.trajectories[self.current_traj_idx]}')
                self.state = "EXECUTING"
                self.step_idx = 0
                self.actual_x, self.actual_z = [], [] # Clear for new R^2
                self.target_x, self.target_z = [], []

        elif self.state == "EXECUTING":
            if self.step_idx < len(self.df):
                col_name = self.trajectories[self.current_traj_idx]
                
                tx = self.df['X_Position'][self.step_idx]
                tz = self.df[col_name][self.step_idx]
                
                # Publish [X, Y, Z] -> Y is 0 for 2D trajectory
                self.send_setpoint(tx, 0.0, tz)
                
                self.target_x.append(tx)
                self.target_z.append(tz)
                self.step_idx += 1
            else:
                self.process_results()

    def process_results(self):
        traj_name = self.trajectories[self.current_traj_idx]
        
        # R^2 Calculation (Z-axis accuracy)
        # Note: We align lengths in case odom frequency differs from loop
        min_len = min(len(self.actual_z), len(self.target_z))
        if min_len > 1:
            r2 = r2_score(self.target_z[:min_len], self.actual_z[:min_len])
            self.get_logger().info(f'COMPLETED {traj_name} | R^2 Score: {r2:.4f}')
            
            # Plot comparison
            plt.figure(figsize=(8, 5))
            plt.plot(self.target_x, self.target_z, 'r--', label='Target (MATLAB)')
            plt.plot(self.actual_x, self.actual_z, 'b-', label='Actual (Odom)')
            plt.title(f'Path Analysis: {traj_name} (R^2: {r2:.4f})')
            plt.xlabel('X (m)')
            plt.ylabel('Z (m)')
            plt.legend()
            plt.grid(True)
            plt.show(block=False) # Non-blocking so code continues
            plt.pause(3) 
            plt.close()
        
        # Cycle to next trajectory or Loop back to start
        self.current_traj_idx = (self.current_traj_idx + 1) % len(self.trajectories)
        self.state = "GO_HOME"

def main(args=None):
    rclpy.init(args=args)
    node = TwoDTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()