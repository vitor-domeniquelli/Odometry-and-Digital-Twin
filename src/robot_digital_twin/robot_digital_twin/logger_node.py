import os
import csv
import math
import yaml
import subprocess
from datetime import datetime

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Float64MultiArray
from std_srvs.srv import Trigger


class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')

        self._logging = False
        self._log_dir = None
        self._files = {}
        self._writers = {}
        self._counts = {'odom': 0, 'gt': 0}

        self.create_subscription(Odometry,          '/odom_est',      self._odom_cb,    10)
        self.create_subscription(PoseStamped,        '/ground_truth',  self._gt_cb,      10)
        self.create_subscription(Float64,            '/latency_ms',    self._latency_cb, 10)
        self.create_subscription(Float64MultiArray,  '/raw_sensors',   self._raw_cb,     10)

        self.create_service(Trigger, 'start_logging', self._start_cb)
        self.create_service(Trigger, 'stop_logging',  self._stop_cb)

        self.get_logger().info("Logger Node iniciado. Chame 'start_logging' para gravar.")

    # ── helpers ────────────────────────────────────────────────────────────────

    @staticmethod
    def _quat_to_yaw(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9

    def _get_odom_params(self) -> dict:
        defaults = {
            'wheel_radius':   0.068,
            'gear_ratio':     1.5,
            'pulses_per_rev': 40,
            'track_width':    0.130,
            'imu_weight':     0.98,
        }
        ros_names = {
            'wheel_radius':   'wheel_radius',
            'gear_ratio':     'gear_ratio',
            'pulses_per_rev': 'pulses_per_rev',
            'track_width':    'track_width',
            'imu_weight':     'w_imu',
        }
        params = dict(defaults)
        for key, ros_param in ros_names.items():
            try:
                out = subprocess.run(
                    ['ros2', 'param', 'get', '/odom_node', ros_param],
                    capture_output=True, text=True, timeout=2.0,
                ).stdout.strip()
                if 'value is:' in out:
                    raw = out.split('value is:')[-1].strip()
                    val = float(raw)
                    params[key] = int(val) if val == int(val) else val
            except Exception:
                pass
        return params

    # ── subscribers ────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        if not self._logging:
            return
        t     = self._stamp_to_sec(msg.header.stamp)
        x     = msg.pose.pose.position.x
        y     = msg.pose.pose.position.y
        theta = self._quat_to_yaw(msg.pose.pose.orientation)
        self._writers['odom'].writerow([t, x, y, theta])
        self._counts['odom'] += 1

    def _gt_cb(self, msg: PoseStamped):
        if not self._logging:
            return
        t     = self._stamp_to_sec(msg.header.stamp)
        x     = msg.pose.position.x
        y     = msg.pose.position.y
        theta = self._quat_to_yaw(msg.pose.orientation)
        self._writers['gt'].writerow([t, x, y, theta])
        self._counts['gt'] += 1

    def _latency_cb(self, msg: Float64):
        if not self._logging:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        self._writers['latency'].writerow([t, msg.data])

    def _raw_cb(self, msg: Float64MultiArray):
        if not self._logging:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        d = msg.data
        self._writers['raw'].writerow([t, d[0], d[1], d[2], d[3], d[4]])

    # ── services ───────────────────────────────────────────────────────────────

    def _start_cb(self, request, response):
        if self._logging:
            response.success = False
            response.message = 'Já em modo de gravação.'
            return response

        ts      = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_dir = os.path.expanduser(f'~/ros_logs/exp_{ts}')
        os.makedirs(log_dir, exist_ok=True)

        specs = {
            'odom':    ('odom_log.csv',       ['timestamp_ros', 'x', 'y', 'theta']),
            'gt':      ('gt_log.csv',          ['timestamp_ros', 'x', 'y', 'theta']),
            'latency': ('latency_log.csv',     ['timestamp_ros', 'latency_ms']),
            'raw':     ('raw_sensors_log.csv', ['timestamp_ros', 't_esp', 'dt', 'dE', 'dD', 'gz']),
        }
        for key, (fname, headers) in specs.items():
            f = open(os.path.join(log_dir, fname), 'w', newline='')
            writer = csv.writer(f)
            writer.writerow(headers)
            self._files[key]   = f
            self._writers[key] = writer

        params = self._get_odom_params()
        metadata = {'timestamp_inicio': ts, **params, 'notas': ''}
        with open(os.path.join(log_dir, 'metadata.yaml'), 'w') as f:
            yaml.dump(metadata, f, default_flow_style=False, allow_unicode=True)

        self._counts  = {'odom': 0, 'gt': 0}
        self._log_dir = log_dir
        self._logging = True

        self.get_logger().info(f"Gravação iniciada: {log_dir}")
        response.success = True
        response.message = log_dir
        return response

    def _stop_cb(self, request, response):
        if not self._logging:
            response.success = False
            response.message = 'Nenhuma gravação ativa.'
            return response

        self._logging = False
        for f in self._files.values():
            f.close()
        self._files.clear()
        self._writers.clear()

        self.get_logger().info(
            f"Experimento salvo: {self._counts['odom']} amostras odom, "
            f"{self._counts['gt']} amostras gt — {self._log_dir}"
        )
        response.success = True
        response.message = self._log_dir
        return response

    def destroy_node(self):
        if self._logging:
            for f in self._files.values():
                f.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()