import sys
import math
import select
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Parâmetros ROS
        self.declare_parameter('manual_pwm_speed', 160)
        self.declare_parameter('manual_pwm_turn', 160)
        self.declare_parameter('manual_rate_hz', 10.0)

        self.pwm_speed = self.get_parameter('manual_pwm_speed').value
        self.pwm_turn = self.get_parameter('manual_pwm_turn').value
        self.rate_hz = self.get_parameter('manual_rate_hz').value

        # Parâmetros de Missão e Path (Valores padrão do ESP32)
        self.t_reta_ms = 2000.0
        self.v_reta_pwm = 160.0
        self.c_curva_pwm = 180.0
        self.a_curva_deg = 90.0

        # Publishers
        self.cmd_pub = self.create_publisher(String, '/serial_cmd', 10)
        self.path_pub = self.create_publisher(Path, '/mission_path', 10)

        # Estado do Sistema
        self.mode = 'MISSION' # 'MISSION' ou 'MANUAL'
        self.current_key = ''
        self.key_lock = threading.Lock()
        self.running = True

        # Timer para o Modo Manual
        timer_period = 1.0 / self.rate_hz
        self.manual_timer = self.create_timer(timer_period, self.manual_loop)

        # Thread para o menu e leitura de teclado
        self.input_thread = threading.Thread(target=self.menu_loop, daemon=True)
        self.input_thread.start()

        self.get_logger().info("Controller Node iniciado.")

    def send_cmd(self, cmd_str: str):
        """Função auxiliar para publicar na serial."""
        msg = String()
        msg.data = cmd_str
        self.cmd_pub.publish(msg)

    def euler_to_quaternion(self, yaw):
        """Converte yaw para quaternion (necessário para o PoseStamped)."""
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qz, qw

    def generate_theoretical_path(self, shape: str):
        """Gera e publica a trajetória teórica baseada nos parâmetros atuais."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'

        # Estimativa de distância conforme o requisito do prompt
        dist_estimada = (self.v_reta_pwm / 255.0) * 0.5 * (self.t_reta_ms / 1000.0)
        N = 50 # Pontos por segmento

        poses = []
        
        if shape == 'L':
            for i in range(N + 1):
                x = (i / N) * dist_estimada
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = x
                pose.pose.position.y = 0.0
                pose.pose.position.z = 0.0
                qz, qw = self.euler_to_quaternion(0.0)
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                poses.append(pose)

        elif shape == 'Q':
            # Quadrado perfeito: 4 lados de tamanho dist_estimada
            # Lado 1 (Frente -> Y=0)
            for i in range(N + 1):
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = (i / N) * dist_estimada
                pose.pose.position.y = 0.0
                qz, qw = self.euler_to_quaternion(0.0)
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                poses.append(pose)
            
            # Lado 2 (Esquerda -> X=dist)
            for i in range(N + 1):
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = dist_estimada
                pose.pose.position.y = (i / N) * dist_estimada
                qz, qw = self.euler_to_quaternion(math.pi / 2.0)
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                poses.append(pose)
            
            # Lado 3 (Trás -> Y=dist)
            for i in range(N + 1):
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = dist_estimada - ((i / N) * dist_estimada)
                pose.pose.position.y = dist_estimada
                qz, qw = self.euler_to_quaternion(math.pi)
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                poses.append(pose)

            # Lado 4 (Direita -> X=0)
            for i in range(N + 1):
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = 0.0
                pose.pose.position.y = dist_estimada - ((i / N) * dist_estimada)
                qz, qw = self.euler_to_quaternion(3.0 * math.pi / 2.0)
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                poses.append(pose)

        path_msg.poses = poses
        self.path_pub.publish(path_msg)

    def print_menu(self):
        print("\n=== CONTROLLER NODE ===")
        print("[L] Trajetória Linear")
        print("[Q] Trajetória Quadrado")
        print("[M] Modo Manual")
        print("[S] Parar")
        print("[T/V/C/A] Configurar parâmetros")
        print("[X] Sair")
        print("Opção: ", end='', flush=True)

    def get_char_non_blocking(self):
        """Leitura de 1 caractere do terminal (sem necessidade de Enter)."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            # Timeout de 0.1s para não bloquear o thread permanentemente
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                ch = sys.stdin.read(1)
            else:
                ch = ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def menu_loop(self):
        while self.running:
            if self.mode == 'MISSION':
                self.print_menu()
                # Modo bloqueante padrão para menu
                op = sys.stdin.readline().strip().upper()
                
                if op == 'L':
                    self.send_cmd("L")
                    self.generate_theoretical_path('L')
                    print("-> Missão LINEAR iniciada.")
                elif op == 'Q':
                    self.send_cmd("Q")
                    self.generate_theoretical_path('Q')
                    print("-> Missão QUADRADO iniciada.")
                elif op == 'M':
                    self.mode = 'MANUAL'
                    print("\n=== MODO MANUAL === (WASD=mover, X=sair manual)")
                elif op == 'S':
                    self.send_cmd("S")
                    print("-> Comando PARAR enviado.")
                elif op in ['T', 'V', 'C', 'A']:
                    val = input(f"Digite o novo valor para {op}: ")
                    self.send_cmd(f"{op}:{val}")
                    # Atualiza variável local
                    try:
                        v_float = float(val)
                        if op == 'T': self.t_reta_ms = v_float
                        elif op == 'V': self.v_reta_pwm = v_float
                        elif op == 'C': self.c_curva_pwm = v_float
                        elif op == 'A': self.a_curva_deg = v_float
                        print(f"-> Parâmetro {op} atualizado para {val}.")
                    except ValueError:
                        print("-> Valor inválido.")
                elif op == 'X':
                    self.running = False
                    self.send_cmd("S")
                    print("A encerrar...")
                else:
                    print("-> Opção inválida.")
            
            elif self.mode == 'MANUAL':
                # Leitura em tempo real (non-blocking RAW mode)
                ch = self.get_char_non_blocking().upper()
                with self.key_lock:
                    self.current_key = ch
                
                if ch == 'X':
                    self.mode = 'MISSION'
                    self.send_cmd("S")
                    with self.key_lock:
                        self.current_key = ''
                    print("\n-> Saindo do Modo Manual.")

    def manual_loop(self):
        """Executado a manual_rate_hz (ex: 10 Hz) para publicar PWM."""
        if self.mode != 'MANUAL':
            return

        with self.key_lock:
            k = self.current_key

        ve = 0
        vd = 0

        if k == 'W':
            ve = self.pwm_speed
            vd = self.pwm_speed
        elif k == 'S':
            ve = -self.pwm_speed
            vd = -self.pwm_speed
        elif k == 'A':
            ve = -self.pwm_turn
            vd = self.pwm_turn
        elif k == 'D':
            ve = self.pwm_turn
            vd = -self.pwm_turn

        # Envia apenas se não for 'X' (saída), garantindo envio contínuo para o watchdog
        if k != 'X':
            self.send_cmd(f"M:{ve},{vd}")

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.send_cmd("S") # Segurança: Parar robô antes de desligar
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()