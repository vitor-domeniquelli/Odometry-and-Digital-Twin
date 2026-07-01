import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


class TwinNode(Node):
    def __init__(self):
        super().__init__('twin_node')

        # 1. Parâmetros ROS
        self.declare_parameter('max_path_length', 10000)
        self.max_path_length = self.get_parameter('max_path_length').value

        # 2. Inicialização dos Paths (Trajetórias)
        self.path_odom = Path()
        self.path_odom.header.frame_id = 'odom'
        self.path_odom.poses = []

        self.path_gt = Path()
        self.path_gt.header.frame_id = 'odom'
        self.path_gt.poses = []

        self.path_teorico = Path()
        self.path_teorico.header.frame_id = 'odom'
        self.path_teorico.poses = []

        # 3. Última pose conhecida (para markers de texto)
        self._odom_x = self._odom_y = self._odom_theta = 0.0
        self._gt_x   = self._gt_y   = self._gt_theta   = 0.0

        # 4. Subscribers
        self.sub_odom = self.create_subscription(
            Odometry,   '/odom_est',     self.odom_callback,          10)
        self.sub_gt = self.create_subscription(
            PoseStamped, '/ground_truth', self.ground_truth_callback,  10)
        self.sub_mission = self.create_subscription(
            Path,        '/mission_path', self.mission_path_callback,  10)

        # 5. Publishers — trajetórias
        self.pub_path_odom    = self.create_publisher(Path, '/path_odom',    10)
        self.pub_path_gt      = self.create_publisher(Path, '/path_gt',      10)
        self.pub_path_teorico = self.create_publisher(Path, '/path_teorico', 10)

        # 6. Publisher — overlay de texto no RViz (odom + GT num único tópico)
        self.pub_markers = self.create_publisher(MarkerArray, '/marker_telemetry', 10)

        # 7. Serviços
        self.srv_reset = self.create_service(Trigger, 'reset_paths', self.reset_paths_callback)

        # 8. Timer de Publicação (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Twin Node (Agregador de Trajetórias) iniciado.")

    # ── helpers ────────────────────────────────────────────────────────────────

    @staticmethod
    def _quat_to_yaw(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _make_text_marker(self, marker_id, x, y, z, text, r, g, b) -> Marker:
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns              = 'twin_text'
        m.id              = marker_id
        m.type            = Marker.TEXT_VIEW_FACING
        m.action          = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.z         = 0.05
        m.color.r         = r
        m.color.g         = g
        m.color.b         = b
        m.color.a         = 1.0
        m.text            = text
        return m

    # ── callbacks ──────────────────────────────────────────────────────────────

    def odom_callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose   = msg.pose.pose

        self.path_odom.poses.append(pose)
        if len(self.path_odom.poses) > self.max_path_length:
            self.path_odom.poses.pop(0)

        self._odom_x     = msg.pose.pose.position.x
        self._odom_y     = msg.pose.pose.position.y
        self._odom_theta = self._quat_to_yaw(msg.pose.pose.orientation)

    def ground_truth_callback(self, msg: PoseStamped):
        self.path_gt.poses.append(msg)
        if len(self.path_gt.poses) > self.max_path_length:
            self.path_gt.poses.pop(0)

        self._gt_x     = msg.pose.position.x
        self._gt_y     = msg.pose.position.y
        self._gt_theta = self._quat_to_yaw(msg.pose.orientation)

    def mission_path_callback(self, msg: Path):
        self.path_teorico = msg

    def reset_paths_callback(self, request, response):
        self.path_odom.poses.clear()
        self.path_gt.poses.clear()
        self.path_teorico.poses.clear()
        response.success = True
        response.message = 'Paths resetados com sucesso.'
        self.get_logger().info("Todas as trajetórias foram zeradas via serviço.")
        return response

    # ── timer ──────────────────────────────────────────────────────────────────

    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        self.path_odom.header.stamp    = now
        self.path_gt.header.stamp      = now
        self.path_teorico.header.stamp = now

        self.pub_path_odom.publish(self.path_odom)
        self.pub_path_gt.publish(self.path_gt)
        self.pub_path_teorico.publish(self.path_teorico)

        # Textos fixos à direita da arena — posição em coordenadas do mundo
        TEXT_X = 0.70  # à direita da arena (real_width = 0.60 m)
        ODOM_Y = 0.47  # linha superior
        GT_Y   = 0.25  # linha inferior

        arr = MarkerArray()
        arr.markers.append(self._make_text_marker(
            0, TEXT_X, ODOM_Y, 0.0,
            f"Odom\nx={self._odom_x:.3f}m  y={self._odom_y:.3f}m\nθ={math.degrees(self._odom_theta):.1f}°",
            0.0, 0.0, 1.0,
        ))
        arr.markers.append(self._make_text_marker(
            1, TEXT_X, GT_Y, 0.0,
            f"GT\nx={self._gt_x:.3f}m  y={self._gt_y:.3f}m\nθ={math.degrees(self._gt_theta):.1f}°",
            0.0, 1.0, 0.0,
        ))
        self.pub_markers.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = TwinNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()