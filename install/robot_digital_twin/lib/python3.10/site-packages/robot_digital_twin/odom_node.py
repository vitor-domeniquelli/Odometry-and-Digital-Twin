import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        # Parâmetros Físicos e de Fusão
        self.declare_parameter('track_width', 0.130)
        self.declare_parameter('w_imu', 0.98)
        self.declare_parameter('wheel_radius', 0.068)
        self.declare_parameter('gear_ratio', 1.5)
        self.declare_parameter('pulses_per_rev', 40)

        self.L = self.get_parameter('track_width').value
        self.W_imu = self.get_parameter('w_imu').value

        wheel_radius    = self.get_parameter('wheel_radius').value
        gear_ratio      = self.get_parameter('gear_ratio').value
        pulses_per_rev  = self.get_parameter('pulses_per_rev').value

        self.dist_per_pulse = (2.0 * math.pi * wheel_radius) / \
                              (pulses_per_rev * gear_ratio)

        # Estado do Robô (Pose)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Subscriber para os sensores brutos
        self.sub_raw = self.create_subscription(
            Float64MultiArray,
            '/raw_sensors',
            self.sensor_callback,
            10
        )

        # Publisher e Broadcaster
        self.odom_pub = self.create_publisher(Odometry, '/odom_est', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Serviço de reset
        self.create_service(Trigger, 'reset_odometry', self._reset_cb)

        self.get_logger().info("Nó de Odometria iniciado com Integração Mid-point.")

    def _reset_cb(self, request, response):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.get_logger().info("Odometria zerada.")
        response.success = True
        response.message = 'Odometria zerada'
        return response

    def euler_to_quaternion(self, yaw, pitch=0.0, roll=0.0) -> Quaternion:
        """Converte ângulos de Euler para Quaternion."""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def sensor_callback(self, msg: Float64MultiArray):
        # Extração dos dados do pacote
        # campos = [t_esp, dt, dE, dD, gz]
        dt  = msg.data[1]
        dE  = int(msg.data[2])
        dD  = int(msg.data[3])
        gz  = msg.data[4]

        # ZUPT: robô parado — ignora drift do giroscópio
        if dE == 0 and dD == 0:
            gz = 0.0

        # Converte ticks para metros
        dist_esq = dE * self.dist_per_pulse
        dist_dir = dD * self.dist_per_pulse

        # 1. Cinemática Linear
        d_media  = (dist_esq + dist_dir) / 2.0
        v_linear = d_media / dt if dt > 0 else 0.0

        # 2. Cinemática Angular (Encoders vs IMU)
        d_theta_enc = (dist_dir - dist_esq) / self.L
        gz_rps      = gz * (math.pi / 180.0)
        d_theta_imu = gz_rps * dt
        
        # Fusão Ponderada para mitigar o arrasto da esteira (skid-steer)
        d_theta_fused = (self.W_imu * d_theta_imu) + ((1.0 - self.W_imu) * d_theta_enc)
        v_angular = d_theta_fused / dt if dt > 0 else 0.0

        # 3. Integração Numérica (Mid-point / Runge-Kutta de 2ª Ordem)
        self.x += d_media * math.cos(self.theta + (d_theta_fused / 2.0))
        self.y += d_media * math.sin(self.theta + (d_theta_fused / 2.0))
        self.theta += d_theta_fused

        # Normalização do ângulo entre -pi e pi
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Obter o tempo atual do ROS para carimbar a mensagem
        current_time = self.get_clock().now()
        q_pose = self.euler_to_quaternion(self.theta)

        # 4. Publicar Transformação Espacial (TF2)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q_pose
        
        self.tf_broadcaster.sendTransform(t)

        # 5. Publicar Tópico de Odometria (/odom_est)
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Pose
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = q_pose

        # Velocidade (Twist)
        odom_msg.twist.twist.linear.x = v_linear
        odom_msg.twist.twist.angular.z = v_angular

        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()