import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
import time

class StepResponseEvaluator(Node):
    def __init__(self):
        super().__init__('step_response_evaluator')

        # --- Config ---
        self.z_hover   = 5.0   # hover altitude
        self.step_size = 0.5   # how far to step in X or Y
        self.settle_threshold = 0.05   # ±0.05m = "settled"
        self.settle_duration  = 1.0    # must stay within threshold for this long
        self.timeout          = 15.0   # max wait per step test

        # Axes to test: (label, x, y, z) for start and step targets
        # Pattern: hover → step → back to hover between each
        self.tests = [
            ('X', [0.0, 0.0, self.z_hover], [self.step_size, 0.0, self.z_hover]),
            ('Y', [0.0, 0.0, self.z_hover], [0.0, self.step_size, self.z_hover]),
            ('Z', [0.0, 0.0, self.z_hover], [0.0, 0.0, self.z_hover + self.step_size]),
        ]
        self.current_test_idx = 0

        # Pubs / Subs
        self.cmd_pub  = self.create_publisher(Float32MultiArray, '/setpoint', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # State machine
        self.state = 'GO_HOME'
        self.curr_pos = np.array([0.0, 0.0, 0.0])
        self.wait_start = None

        # Recording
        # history[axis] = {'t': [], 'pos': [], 'setpoint': scalar, 'start_pos': scalar}
        self.history = {}

        self.step_start_time  = None
        self.settle_start     = None
        self.recording        = False

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Step Response Evaluator started.')

    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        self.curr_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ])
        if self.recording:
            label = self.tests[self.current_test_idx][0]
            axis_idx = {'X': 0, 'Y': 1, 'Z': 2}[label]
            elapsed = time.time() - self.step_start_time
            self.history[label]['t'].append(elapsed)
            self.history[label]['pos'].append(self.curr_pos[axis_idx])

    # ------------------------------------------------------------------
    def send_setpoint(self, x, y, z):
        msg = Float32MultiArray()
        msg.data = [float(x), float(y), float(z)]
        self.cmd_pub.publish(msg)

    def dist_to(self, target):
        return np.linalg.norm(self.curr_pos - np.array(target))

    # ------------------------------------------------------------------
    def control_loop(self):
        if self.state == 'FINISHED':
            return

        # ── GO HOME (before each test) ─────────────────────────────────
        if self.state == 'GO_HOME':
            home = [0.0, 0.0, self.z_hover]
            self.send_setpoint(*home)
            if self.dist_to(home) < 0.3:
                self.state = 'WAIT_HOME'
                self.wait_start = time.time()

        # ── WAIT 3s at home to stabilise ──────────────────────────────
        elif self.state == 'WAIT_HOME':
            self.send_setpoint(0.0, 0.0, self.z_hover)
            if time.time() - self.wait_start > 3.0:
                self.state = 'STEP'

        # ── COMMAND STEP ──────────────────────────────────────────────
        elif self.state == 'STEP':
            label, _, target = self.tests[self.current_test_idx]
            axis_idx = {'X': 0, 'Y': 1, 'Z': 2}[label]

            self.history[label] = {
                't':         [],
                'pos':       [],
                'setpoint':  target[axis_idx],
                'start_pos': self.curr_pos[axis_idx],
            }
            self.step_start_time = time.time()
            self.settle_start    = None
            self.recording       = True
            self.state           = 'RECORDING'
            self.get_logger().info(f'Step test: {label} → {target}')

        # ── RECORD UNTIL SETTLED OR TIMEOUT ───────────────────────────
        elif self.state == 'RECORDING':
            label, _, target = self.tests[self.current_test_idx]
            axis_idx = {'X': 0, 'Y': 1, 'Z': 2}[label]
            self.send_setpoint(*target)

            elapsed   = time.time() - self.step_start_time
            error     = abs(self.curr_pos[axis_idx] - target[axis_idx])
            timed_out = elapsed > self.timeout

            if error < self.settle_threshold:
                if self.settle_start is None:
                    self.settle_start = time.time()
                elif time.time() - self.settle_start > self.settle_duration:
                    self.recording = False
                    self.state     = 'NEXT'
            else:
                self.settle_start = None  # reset if it drifts out

            if timed_out:
                self.get_logger().warn(f'{label} axis timed out — did not settle within {self.timeout}s')
                self.recording = False
                self.state     = 'NEXT'

        # ── NEXT TEST OR FINISH ────────────────────────────────────────
        elif self.state == 'NEXT':
            if self.current_test_idx < len(self.tests) - 1:
                self.current_test_idx += 1
                self.state = 'GO_HOME'
            else:
                self.state = 'RETURN_HOME'
                self.get_logger().info('All step tests done. Plotting...')
                
        elif self.state == 'RETURN_HOME':
            self.send_setpoint(0.0, 0.0, self.z_hover)
            if self.dist_to([0.0, 0.0, self.z_hover]) < 0.3:
                self.get_logger().info('Plotting...')
                self.plot_results()
                self.state = 'FINISHED'

    # ------------------------------------------------------------------
    def compute_metrics(self, t, pos, setpoint, start_pos):
        t   = np.array(t)
        pos = np.array(pos)

        step_mag = setpoint - start_pos
        if abs(step_mag) < 1e-6:
            return {}

        # Normalise to [0, 1] response
        response = (pos - start_pos) / step_mag

        # Rise time: 10% → 90%
        rise_start = next((t[i] for i, r in enumerate(response) if r >= 0.10), None)
        rise_end   = next((t[i] for i, r in enumerate(response) if r >= 0.90), None)
        rise_time  = (rise_end - rise_start) if (rise_start and rise_end) else None

        # Overshoot
        peak     = np.max(response)
        overshoot = max(0.0, (peak - 1.0) * 100.0)  # percent

        # Settling time (±5% band, must stay there)
        settling_time = None
        window = max(1, int(0.5 / (t[1] - t[0])) if len(t) > 1 else 1)
        for i in range(len(t) - window):
            window_pos = pos[i:i + window]
            if np.all(np.abs(window_pos - setpoint) < self.settle_threshold):
                settling_time = t[i]
                break

        # Steady state error (mean of last 20% of samples)
        last_n = max(1, int(0.2 * len(pos)))
        steady_state_error = abs(np.mean(pos[-last_n:]) - setpoint)

        return {
            'rise_time':          rise_time,
            'overshoot_pct':      overshoot,
            'settling_time':      settling_time,
            'steady_state_error': steady_state_error,
        }

    # ------------------------------------------------------------------
    def plot_results(self):
        fig, axes = plt.subplots(1, 3, figsize=(18, 5))
        fig.suptitle('Step Response — X / Y / Z', fontsize=15)

        for i, (label, _, target) in enumerate(self.tests):
            if label not in self.history:
                continue

            data     = self.history[label]
            t        = np.array(data['t'])
            pos      = np.array(data['pos'])
            sp       = data['setpoint']
            start    = data['start_pos']
            metrics  = self.compute_metrics(t, pos, sp, start)

            ax = axes[i]
            ax.plot(t, pos, 'b-', linewidth=1.5, label='Actual')
            ax.axhline(sp,    color='r', linestyle='--', alpha=0.7, label='Setpoint')
            # ax.axhline(sp * 1.05, color='g', linestyle=':', alpha=0.5, label='±5% band')
            # ax.axhline(sp * 0.95, color='g', linestyle=':', alpha=0.5)

            # Annotate settling time
            if metrics.get('settling_time'):
                ax.axvline(metrics['settling_time'], color='orange',
                           linestyle='--', alpha=0.7, label=f"Settle={metrics['settling_time']:.2f}s")

            def fmt(v, spec):
                return format(v, spec) if v is not None else 'N/A'

            title = (
                f'{label}-Axis Step Response\n'
                f"Rise: {fmt(metrics.get('rise_time'), '.2f')}s  "
                f"Overshoot: {fmt(metrics.get('overshoot_pct'), '.1f')}%\n"
                f"Settle: {fmt(metrics.get('settling_time'), '.2f')}s  "
                f"SS Err: {fmt(metrics.get('steady_state_error'), '.4f')}m"
            )
            ax.set_title(title)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(f'{label} Position (m)')
            ax.legend(fontsize=8)
            ax.grid(True)

            def fmt(v, spec):
                return format(v, spec) if v is not None else 'N/A'

            self.get_logger().info(
                f"{label}: rise={fmt(metrics.get('rise_time'), '.2f')}s  "
                f"overshoot={fmt(metrics.get('overshoot_pct'), '.1f')}%  "
                f"settle={fmt(metrics.get('settling_time'), '.2f')}s  "
                f"SS_err={fmt(metrics.get('steady_state_error'), '.4f')}m"
            )

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.savefig('step_response.png', dpi=150, bbox_inches='tight')
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = StepResponseEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()