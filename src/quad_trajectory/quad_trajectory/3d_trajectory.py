import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import KDTree # Used for Nearest Neighbor calculation
import time
from ament_index_python.packages import get_package_share_directory
import os

class ThreeDTrajectory(Node):
    def __init__(self):
        super().__init__('three_dimension_spatial_evaluator')
        
        # 1. Load CSV
        try:
            package_share_directory = get_package_share_directory('quad_trajectory')
            csv_path = os.path.join(package_share_directory, 'config', 'tri_axis_trajectories_3D.csv')
            self.df = pd.read_csv(csv_path)
            self.get_logger().info(f'3D CSV Loaded: {csv_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load CSV: {str(e)}')
            return 

        # 2. Pubs/Subs
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/setpoint', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # 3. Data Storage
        self.traj_types = ['Line', 'Helix', 'Mobius']
        self.history = {name: {'tx': [], 'ty': [], 'tz': [], 'ax': [], 'ay': [], 'az': []} 
                        for name in self.traj_types}
        
        # 4. State Machine
        self.current_traj_idx = 0
        self.step_idx = 0
        self.state = "GO_HOME" 
        self.curr_x, self.curr_y, self.curr_z = 0.0, 0.0, 0.0
        self.z_start = 5.0 
        self.timer = self.create_timer(0.05, self.control_loop)
        self.wait_start_time = None

    def odom_callback(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        self.curr_z = msg.pose.pose.position.z
        
        if self.state == "EXECUTING":
            name = self.traj_types[self.current_traj_idx]
            self.history[name]['ax'].append(self.curr_x)
            self.history[name]['ay'].append(self.curr_y)
            self.history[name]['az'].append(self.curr_z)

    def send_setpoint(self, x, y, z):
        msg = Float32MultiArray()
        msg.data = [float(x), float(y), float(z)]
        self.cmd_pub.publish(msg)

    def control_loop(self):
        if self.state == "FINISHED": return

        if self.state in ["GO_HOME", "POST_SEQUENCE_HOME"]:
            self.send_setpoint(0.0, 0.0, self.z_start)
            dist = np.sqrt(self.curr_x**2 + self.curr_y**2 + (self.curr_z - self.z_start)**2)
            if dist < 0.25:
                if self.state == "POST_SEQUENCE_HOME":
                    self.final_evaluation_3d()
                    self.state = "FINISHED"
                else:
                    self.state = "WAITING"
                    self.wait_start_time = time.time()

        elif self.state == "WAITING":
            if time.time() - self.wait_start_time > 4.0:
                self.state = "EXECUTING"
                self.step_idx = 0

        elif self.state == "EXECUTING":
            if self.step_idx < len(self.df):
                name = self.traj_types[self.current_traj_idx]
                tx = self.df[f'X_{name}'][self.step_idx]
                ty = self.df[f'Y_{name}'][self.step_idx]
                tz = self.df['Z'][self.step_idx]
                self.send_setpoint(tx, ty, tz)
                self.history[name]['tx'].append(tx)
                self.history[name]['ty'].append(ty)
                self.history[name]['tz'].append(tz)
                self.step_idx += 1
            else:
                self.finish_trajectory()

    def finish_trajectory(self):
        if self.current_traj_idx < len(self.traj_types) - 1:
            self.current_traj_idx += 1
            self.state = "GO_HOME"
        else:
            self.state = "POST_SEQUENCE_HOME"

    def final_evaluation_3d(self):
        """Calculates spatial error using Nearest Neighbor (KDTree)."""
        fig = plt.figure(figsize=(20, 7))
        fig.suptitle('3D Trajectory', fontsize=16)

        for i, name in enumerate(self.traj_types):
            data = self.history[name]
            
            # 1. Prepare Point Clouds
            # Target Trajectory Points
            targets = np.vstack((data['tx'], data['ty'], data['tz'])).T
            # Actual Odom Points
            actuals = np.vstack((data['ax'], data['ay'], data['az'])).T
            
            if len(targets) > 0 and len(actuals) > 0:
                # 2. Build KDTree for the Target Trajectory
                tree = KDTree(targets)
                
                # 3. Query: For each 'actual' point, find the distance to the nearest 'target' point
                # This ignores timing/velocity differences.
                distances, _ = tree.query(actuals)
                
                mean_spatial_error = np.mean(distances)
                max_spatial_error = np.max(distances)
                rmse_spatial = np.sqrt(np.mean(distances**2))

                # 4. Plotting
                ax = fig.add_subplot(1, 3, i+1, projection='3d')
                ax.plot(data['tx'], data['ty'], data['tz'], 'r--', label='Ideal Path', alpha=0.6)
                ax.plot(data['ax'], data['ay'], data['az'], 'b-', label='Actual Path', linewidth=1.5)
                
                ax.set_title(f"{name}\nMean Spatial Err: {mean_spatial_error:.3f}m\nMax Spatial Err: {max_spatial_error:.3f}m")
                ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
                ax.legend()
                ax.grid(True)
                
                self.get_logger().info(f"Result for {name}: Mean Spatial Err={mean_spatial_error:.4f}m")

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.show()

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