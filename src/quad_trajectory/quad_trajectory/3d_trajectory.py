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

class ThreeDTrajectory(Node):
    def __init__(self):
        super().__init__('three_dimension_trajectory')
        
        # 1. Load the 3D MATLAB CSV
        try:
            package_share_directory = get_package_share_directory('quad_trajectory')
            csv_path = os.path.join(package_share_directory, 'config', 'tri_axis_trajectories_3D.csv')
            self.df = pd.read_csv(csv_path)
            self.get_logger().info(f'3D CSV Loaded from: {csv_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load CSV: {str(e)}')
            return 

        # 2. Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/setpoint', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # 3. State Machine Vars
        # We group the X and Y columns; Z is shared for all
        self.traj_types = ['Line', 'Helix', 'Mobius']
        self.current_traj_idx = 0
        self.step_idx = 0
        self.state = "GO_HOME" 
        
        # Data storage for 3D analysis
        self.actual_x, self.actual_y, self.actual_z = [], [], []
        self.target_x, self.target_y, self.target_z = [], [], []
        self.curr_x, self.curr_y, self.curr_z = 0.0, 0.0, 0.0
        
        # 4. Control Loop (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.wait_start_time = None
        self.z_start = 5.0 # Starting altitude

    def odom_callback(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        self.curr_z = msg.pose.pose.position.z
        
        if self.state == "EXECUTING":
            self.actual_x.append(self.curr_x)
            self.actual_y.append(self.curr_y)
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
            dist_to_home = np.sqrt(self.curr_x**2 + self.curr_y**2 + (self.curr_z - self.z_start)**2)
            
            if dist_to_home < 0.15: # Arrival threshold
                if self.state == "POST_SEQUENCE_HOME":
                    self.get_logger().info('Returning to [0,0,5] complete. Inspect plots.')
                    self.state = "FINISHED"
                    plt.show() # Keep all windows open
                else:
                    self.get_logger().info(f'At Start Point. Waiting 5s to begin {self.traj_types[self.current_traj_idx]}...')
                    self.state = "WAITING"
                    self.wait_start_time = time.time()

        elif self.state == "WAITING":
            if time.time() - self.wait_start_time > 5.0:
                self.state = "EXECUTING"
                self.step_idx = 0
                self.clear_data()

        elif self.state == "EXECUTING":
            if self.step_idx < len(self.df):
                type_name = self.traj_types[self.current_traj_idx]
                
                # Pull X and Y based on type, Z is common
                tx = self.df[f'X_{type_name}'][self.step_idx]
                ty = self.df[f'Y_{type_name}'][self.step_idx]
                tz = self.df['Z'][self.step_idx]
                
                self.send_setpoint(tx, ty, tz)
                self.target_x.append(tx); self.target_y.append(ty); self.target_z.append(tz)
                self.step_idx += 1
            else:
                self.process_results()

    def clear_data(self):
        self.actual_x, self.actual_y, self.actual_z = [], [], []
        self.target_x, self.target_y, self.target_z = [], [], []

    def process_results(self):
        name = self.traj_types[self.current_traj_idx]
        min_l = min(len(self.actual_z), len(self.target_z))
        
        if min_l > 1:
            # Calculate R^2 for each dimension to see where the error is
            r2_x = r2_score(self.target_x[:min_l], self.actual_x[:min_l])
            r2_y = r2_score(self.target_y[:min_l], self.actual_y[:min_l])
            r2_z = r2_score(self.target_z[:min_l], self.actual_z[:min_l])
            total_r2 = (r2_x + r2_y + r2_z) / 3
            
            self.get_logger().info(f'FINISHED {name} | Avg R^2: {total_r2:.4f}')
            
            # Plotting in 3D
            fig = plt.figure(self.current_traj_idx + 1)
            ax = fig.add_subplot(111, projection='3d')
            ax.plot(self.target_x, self.target_y, self.target_z, 'r--', label='Target Path')
            ax.plot(self.actual_x, self.actual_y, self.actual_z, 'b-', label='Actual Odom')
            ax.set_title(f'3D {name} Trajectory\nAvg R^2: {total_r2:.4f}')
            ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
            ax.legend(); ax.grid(True)
            
            plt.show(block=False)
            plt.pause(0.1)

        if self.current_traj_idx < len(self.traj_types) - 1:
            self.current_traj_idx += 1
            self.state = "GO_HOME"
        else:
            self.state = "POST_SEQUENCE_HOME"

def main(args=None):
    rclpy.init(args=args)
    node = ThreeDTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()