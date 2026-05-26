import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # Parâmetros Físicos e de Fusão
        self.declare_parameter('track_width', 0.130) # Bitola L (metros)
        self.declare_parameter('w_imu', 0.98)        # Peso do giroscópio na fusão

        self.L = self.get_parameter('track_width').value
        self.W_imu = self.get_parameter('w_imu').value

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
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Nó de Odometria iniciado com Integração Mid-point.")

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
        dt = msg.data[1]
        dE = msg.data[2]
        dD = msg.data[3]
        gz = msg.data[4]

        # 1. Cinemática Linear
        d_media = (dE + dD) / 2.0
        v_linear = d_media / dt if dt > 0 else 0.0

        # 2. Cinemática Angular (Encoders vs IMU)
        d_theta_enc = (dD - dE) / self.L
        d_theta_imu = gz * dt
        
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

        # 5. Publicar Tópico de Odometria (/odom)
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