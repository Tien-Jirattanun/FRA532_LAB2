import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree # The key for timing-independent error
import time
from ament_index_python.packages import get_package_share_directory
import os
from scipy.spatial.distance import directed_hausdorff

class TwoDTrajectory(Node):
    def __init__(self):
        super().__init__('two_dimension_spatial_evaluator')
        
        # 1. Load CSV
        try:
            package_share_directory = get_package_share_directory('quad_trajectory')
            csv_path = os.path.join(package_share_directory, 'config', 'dual_axis_trajectories.csv')
            self.df = pd.read_csv(csv_path)
            self.get_logger().info(f'CSV Loaded: {csv_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load CSV: {str(e)}')
            return 

        # 2. Pubs/Subs
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/setpoint', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # 3. Data Storage
        self.trajectories = ['Z_Line', 'Z_Sine', 'Z_Triangle']
        self.history = {name: {'tx': [], 'tz': [], 'ax': [], 'az': []} for name in self.trajectories}
        
        # 4. State Machine
        self.current_traj_idx = 0
        self.step_idx = 0
        self.state = "GO_HOME" 
        self.curr_x, self.curr_z = 0.0, 0.0
        self.z_start = 5.0
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.wait_start_time = None

    def odom_callback(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_z = msg.pose.pose.position.z
        
        if self.state == "EXECUTING":
            name = self.trajectories[self.current_traj_idx]
            self.history[name]['ax'].append(self.curr_x)
            self.history[name]['az'].append(self.curr_z)

    def send_setpoint(self, x, y, z):
        msg = Float32MultiArray()
        msg.data = [float(x), float(y), float(z)]
        self.cmd_pub.publish(msg)

    def control_loop(self):
        if self.state == "FINISHED":
            return

        if self.state in ["GO_HOME", "POST_SEQUENCE_HOME"]:
            self.send_setpoint(0.0, 0.0, self.z_start)
            dist_to_home = np.sqrt(self.curr_x**2 + (self.curr_z - self.z_start)**2)
            
            if dist_to_home < 0.2:
                if self.state == "POST_SEQUENCE_HOME":
                    self.get_logger().info('Mission Complete. Calculating Spatial Errors...')
                    self.final_evaluation_2d()
                    self.state = "FINISHED"
                else:
                    self.state = "WAITING"
                    self.wait_start_time = time.time()

        elif self.state == "WAITING":
            if time.time() - self.wait_start_time > 3.0:
                self.state = "EXECUTING"
                self.step_idx = 0

        elif self.state == "EXECUTING":
            if self.step_idx < len(self.df):
                name = self.trajectories[self.current_traj_idx]
                tx = self.df['X_Position'][self.step_idx]
                tz = self.df[name][self.step_idx]
                
                self.send_setpoint(tx, 0.0, tz)

                # Only advance to next point when drone is close enough
                dist = np.sqrt((self.curr_x - tx)**2 + (self.curr_z - tz)**2)
                if dist < 0.3:  # tune this threshold
                    self.history[name]['tx'].append(tx)
                    self.history[name]['tz'].append(tz)
                    self.step_idx += 1
            else:
                self.next_state()

    def next_state(self):
        if self.current_traj_idx < len(self.trajectories) - 1:
            self.current_traj_idx += 1
            self.state = "GO_HOME"
        else:
            self.state = "POST_SEQUENCE_HOME"

    def final_evaluation_2d(self):
        """Generates plots based on spatial proximity rather than time-steps."""
        fig, axes = plt.subplots(1, len(self.trajectories), figsize=(18, 5))
        fig.suptitle('2D Trajectory', fontsize=16)
        
        summary = []

        for i, name in enumerate(self.trajectories):
            data = self.history[name]
            
            # Create coordinate arrays (X, Z)
            target_pts = np.vstack((data['tx'], data['tz'])).T
            actual_pts = np.vstack((data['ax'], data['az'])).T
            
            if len(target_pts) > 0 and len(actual_pts) > 0:
                # Build KDTree for the ideal path
                tree = KDTree(target_pts)
                
                # For every actual point recorded, find the distance to the closest target point
                distances, _ = tree.query(actual_pts)
                
                mean_err = np.mean(distances)
                max_err = np.max(distances)
                rmse = np.sqrt(np.mean(distances**2))
                
                # Hausdorff
                h1 = directed_hausdorff(actual_pts, target_pts)[0]
                h2 = directed_hausdorff(target_pts, actual_pts)[0]
                hausdorff = max(h1, h2)

                # Path length ratio
                def path_length(pts):
                    return np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1))
                
                ideal_len  = path_length(target_pts)
                actual_len = path_length(actual_pts)
                ratio      = actual_len / ideal_len

                summary.append({
                    'name': name,
                    'mean': mean_err,
                    'rmse': rmse,
                    'max': max_err,
                    'hausdorff': hausdorff,
                    'ratio': ratio
                })
                
                # Plotting
                ax = axes[i]
                ax.plot(data['tx'], data['tz'], 'r--', label='Ideal Path', alpha=0.6)
                ax.plot(data['ax'], data['az'], 'b-', label='Actual Path', linewidth=1.5)
                
                ax.set_title(
                    f'{name}\n'
                    f'Mean: {mean_err:.4f}m  RMSE: {rmse:.4f}m\n'
                    f'Max: {max_err:.4f}m  Hausdorff: {hausdorff:.4f}m\n'
                    f'Path Ratio: {ratio:.3f}'
                )
                ax.set_xlabel('X Position (m)')
                ax.set_ylabel('Z Position (m)')
                ax.legend()
                ax.grid(True)
                
                # self.get_logger().info(f"{name} Results: Mean Spatial Error = {mean_err:.4f}m")

        # Summary table in logger
        self.get_logger().info('\n' + '-'*65)
        self.get_logger().info(f"{'Trajectory':<12} {'Mean':>8} {'RMSE':>8} {'Max':>8} {'Hausdorff':>10} {'Ratio':>7}")
        self.get_logger().info('-'*65)
        for s in summary:
            self.get_logger().info(
                f"{s['name']:<12} {s['mean']:>8.4f} {s['rmse']:>8.4f} "
                f"{s['max']:>8.4f} {s['hausdorff']:>10.4f} {s['ratio']:>7.3f}"
            )
        self.get_logger().info('-'*65)

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.show()

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