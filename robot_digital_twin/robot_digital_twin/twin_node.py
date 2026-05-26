import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

class TwinNode(Node):
    def __init__(self):
        super().__init__('twin_node')

        # 1. Parâmetros ROS
        self.declare_parameter('max_path_length', 10000)
        self.max_path_length = self.get_parameter('max_path_length').value

        # 2. Inicialização dos Paths (Trajetórias)
        self.path_real = Path()
        self.path_real.header.frame_id = 'odom'
        self.path_real.poses = []

        self.path_gt = Path()
        self.path_gt.header.frame_id = 'odom'
        self.path_gt.poses = []

        self.path_teorico = Path()
        self.path_teorico.header.frame_id = 'odom'
        self.path_teorico.poses = []

        # 3. Subscribers
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom_est',
            self.odom_callback,
            10
        )

        self.sub_gt = self.create_subscription(
            PoseStamped,
            '/ground_truth',
            self.ground_truth_callback,
            10
        )

        self.sub_mission = self.create_subscription(
            Path,
            '/mission_path',
            self.mission_path_callback,
            10
        )

        # 4. Publishers
        self.pub_path_real = self.create_publisher(Path, '/path_real', 10)
        self.pub_path_gt = self.create_publisher(Path, '/path_gt', 10)
        self.pub_path_teorico = self.create_publisher(Path, '/path_teorico', 10)

        # 5. Serviços
        self.srv_reset = self.create_service(Trigger, 'reset_paths', self.reset_paths_callback)

        # 6. Timer de Publicação (10 Hz)
        timer_period = 0.1  # Segundos
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Twin Node (Agregador de Trajetórias) iniciado.")

    def odom_callback(self, msg: Odometry):
        """Acumula as poses da odometria estimada na trajetória real."""
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        
        self.path_real.poses.append(pose)
        
        # Limita o tamanho do array de poses para evitar estouro de memória
        if len(self.path_real.poses) > self.max_path_length:
            self.path_real.poses.pop(0)

    def ground_truth_callback(self, msg: PoseStamped):
        """Acumula as poses capturadas pela visão computacional na trajetória de GT."""
        self.path_gt.poses.append(msg)
        
        # Limita o tamanho do array
        if len(self.path_gt.poses) > self.max_path_length:
            self.path_gt.poses.pop(0)

    def mission_path_callback(self, msg: Path):
        """Substitui completamente a trajetória teórica ideal quando uma nova missão é iniciada."""
        self.path_teorico = msg

    def reset_paths_callback(self, request, response):
        """Serviço para limpar os históricos das três trajetórias."""
        self.path_real.poses.clear()
        self.path_gt.poses.clear()
        self.path_teorico.poses.clear()
        
        response.success = True
        response.message = 'Paths resetados com sucesso.'
        self.get_logger().info("Todas as trajetórias foram zeradas via serviço.")
        
        return response

    def timer_callback(self):
        """Publica as três trajetórias a 10 Hz para atualização fluida no RViz2."""
        current_time = self.get_clock().now().to_msg()

        # Atualiza os timestamps de cabeçalho
        self.path_real.header.stamp = current_time
        self.path_gt.header.stamp = current_time
        self.path_teorico.header.stamp = current_time

        # Publica
        self.pub_path_real.publish(self.path_real)
        self.pub_path_gt.publish(self.path_gt)
        self.pub_path_teorico.publish(self.path_teorico)

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