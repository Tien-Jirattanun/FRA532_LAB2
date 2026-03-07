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
        
        # 1. Load CSV
        try:
            package_share_directory = get_package_share_directory('quad_trajectory')
            csv_path = os.path.join(package_share_directory, 'config', 'dual_axis_trajectories.csv')
            self.df = pd.read_csv(csv_path)
            self.get_logger().info(f'CSV Loaded from: {csv_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load CSV: {str(e)}')
            return 

        # 2. Pubs/Subs
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/setpoint', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # 3. State Machine
        self.trajectories = ['Z_Line', 'Z_Sine', 'Z_Triangle']
        self.current_traj_idx = 0
        self.step_idx = 0
        self.state = "GO_HOME" 
        
        self.actual_x, self.actual_z = [], []
        self.target_x, self.target_z = [], []
        self.curr_x, self.curr_z = 0.0, 0.0
        
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
        msg = Float32MultiArray()
        msg.data = [float(x), float(y), float(z)]
        self.cmd_pub.publish(msg)

    def control_loop(self):
        if self.state == "FINISHED":
            self.send_setpoint(0.0, 0.0, self.z_start)
            return

        if self.state in ["GO_HOME", "POST_SEQUENCE_HOME"]:
            self.send_setpoint(0.0, 0.0, self.z_start)
            dist_to_home = np.sqrt(self.curr_x**2 + (self.curr_z - self.z_start)**2)
            
            if dist_to_home < 0.1:
                if self.state == "POST_SEQUENCE_HOME":
                    self.get_logger().info('Final Return Complete. All plots are now interactive.')
                    self.state = "FINISHED"
                    # This is the magic line that keeps all windows open and responsive
                    plt.show() 
                else:
                    self.get_logger().info('At Home. Waiting 5s...')
                    self.state = "WAITING"
                    self.wait_start_time = time.time()

        elif self.state == "WAITING":
            if time.time() - self.wait_start_time > 5.0:
                self.get_logger().info(f'Starting: {self.trajectories[self.current_traj_idx]}')
                self.state = "EXECUTING"
                self.step_idx = 0
                self.actual_x, self.actual_z = [], [] 
                self.target_x, self.target_z = [], []

        elif self.state == "EXECUTING":
            if self.step_idx < len(self.df):
                col_name = self.trajectories[self.current_traj_idx]
                tx = self.df['X_Position'][self.step_idx]
                tz = self.df[col_name][self.step_idx]
                self.send_setpoint(tx, 0.0, tz)
                self.target_x.append(tx)
                self.target_z.append(tz)
                self.step_idx += 1
            else:
                self.process_results()

    def process_results(self):
        traj_name = self.trajectories[self.current_traj_idx]
        min_len = min(len(self.actual_z), len(self.target_z))
        
        if min_len > 1:
            r2 = r2_score(self.target_z[:min_len], self.actual_z[:min_len])
            self.get_logger().info(f'COMPLETED {traj_name} | R^2: {r2:.4f}')
            
            # Use a unique window number (1, 2, 3)
            plt.figure(self.current_traj_idx + 1) 
            plt.plot(self.target_x, self.target_z, 'r--', label='Target', alpha=0.7)
            plt.plot(self.actual_x, self.actual_z, 'b-', label='Actual', linewidth=1.5)
            plt.title(f'{traj_name} Accuracy (R^2: {r2:.4f})')
            plt.xlabel('X (m)')
            plt.ylabel('Z (m)')
            plt.legend()
            plt.grid(True)
            
            # Show window but don't stop the code yet
            plt.show(block=False)
            plt.pause(0.1) 

        if self.current_traj_idx < len(self.trajectories) - 1:
            self.current_traj_idx += 1
            self.state = "GO_HOME"
        else:
            self.get_logger().info('Sequence complete. Returning home...')
            self.state = "POST_SEQUENCE_HOME"

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