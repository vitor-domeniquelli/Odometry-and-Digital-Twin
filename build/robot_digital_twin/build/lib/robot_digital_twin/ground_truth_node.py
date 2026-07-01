import math
import time
import threading
import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

class GroundTruthNode(Node):
    def __init__(self):
        super().__init__('ground_truth_node')

        # 1. Declaração de Parâmetros ROS
        self.declare_parameter('camera_index', 3)
        self.declare_parameter('camera_matrix_path', '')
        self.declare_parameter('dist_coeffs_path', '')
        self.declare_parameter('real_width', 0.95)
        self.declare_parameter('real_height', 0.95)
        self.declare_parameter('robot_marker_id', 4)
        self.declare_parameter('capture_rate_hz', 30.0)
        self.declare_parameter('camera_width', 1920)
        self.declare_parameter('camera_height', 1080)
        self.declare_parameter('calib_width', 1920)
        self.declare_parameter('calib_height', 1080)

        self.camera_index = self.get_parameter('camera_index').value
        self.camera_matrix_path = self.get_parameter('camera_matrix_path').value
        self.dist_coeffs_path = self.get_parameter('dist_coeffs_path').value
        self.real_width = self.get_parameter('real_width').value
        self.real_height = self.get_parameter('real_height').value
        self.robot_marker_id = self.get_parameter('robot_marker_id').value
        self.capture_rate_hz = self.get_parameter('capture_rate_hz').value
        self.camera_width = self.get_parameter('camera_width').value
        self.camera_height = self.get_parameter('camera_height').value
        self.calib_width = self.get_parameter('calib_width').value
        self.calib_height = self.get_parameter('calib_height').value

        # 2. Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/ground_truth', 10)
        self.image_pub = self.create_publisher(Image, '/camera_image', 10)
        self.cv_bridge = CvBridge()

        # 3. Serviços
        self.srv_zero = self.create_service(Trigger, 'zero_ground_truth', self.zero_cb)
        self.srv_reset = self.create_service(Trigger, 'reset_ground_truth', self.reset_cb)

        # 4. Variáveis de Estado Internas
        self.current_frame = None
        self.frame_lock = threading.Lock()
        self.running = True

        self.homography = None
        self.raw_x = 0.0
        self.raw_y = 0.0
        self.raw_theta = 0.0

        self.off_x = 0.0
        self.off_y = 0.0
        self.off_theta = 0.0

        # Mapeamento dos cantos físicos em metros (ordem exigida no prompt)
        self.dst_pts = np.array([
            [0, self.real_height],                  # 0: TOP_LEFT
            [self.real_width, self.real_height],    # 1: TOP_RIGHT
            [self.real_width, 0],                   # 2: BOTTOM_RIGHT
            [0, 0]                                  # 3: BOTTOM_LEFT
        ], dtype=np.float32)

        self.cam_mat = None
        self.dist_coeff = None
        self.calibration_available = False
        self.load_camera_calibration()

        # Contadores para log de FPS
        self._fps_count = 0
        self._fps_t0    = time.time()

        # Configuração do ArUco (Compatibilidade OpenCV 4.5 e 4.7+)
        try:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters()

        # 5. Inicialização de Threads e Timers
        self.capture_thread = threading.Thread(target=self.camera_loop, daemon=True)
        self.capture_thread.start()

        timer_period = 1.0 / self.capture_rate_hz
        self.timer = self.create_timer(timer_period, self.process_frame)

        self.get_logger().info("Ground Truth Node iniciado.")

    def load_camera_calibration(self):
        """Carrega os arquivos .npy da calibração se os caminhos existirem."""
        if self.camera_matrix_path and self.dist_coeffs_path:
            try:
                self.cam_mat    = np.load(self.camera_matrix_path)
                self.dist_coeff = np.load(self.dist_coeffs_path)

                scale_x = self.camera_width  / self.calib_width
                scale_y = self.camera_height / self.calib_height
                self.cam_mat[0] *= scale_x  # fx e cx
                self.cam_mat[1] *= scale_y  # fy e cy

                self.calibration_available = True
                self.get_logger().info("Calibração da câmera carregada com sucesso.")
            except Exception as e:
                self.get_logger().warning(
                    f"Calibração não encontrada: {e}. "
                    "/ground_truth desabilitado até os arquivos serem fornecidos."
                )
        else:
            self.get_logger().warning(
                "Caminhos de calibração não configurados. "
                "/ground_truth desabilitado até os arquivos serem fornecidos."
            )

    def camera_loop(self):
        """Thread isolada para capturar os frames da câmera sem bloquear o ROS."""
        cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)

        cap.set(cv2.CAP_PROP_FRAME_WIDTH,   self.camera_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  self.camera_height)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)   # modo automático

        while self.running:
            if not cap.isOpened():
                self.get_logger().error(f"Câmera {self.camera_index} indisponível. Tentando novamente em 5s...")
                time.sleep(5.0)
                cap.open(self.camera_index)
                continue

            ret, frame = cap.read()
            if ret:
                with self.frame_lock:
                    self.current_frame = frame.copy()

                # Log de FPS a cada 5 segundos
                self._fps_count += 1
                elapsed = time.time() - self._fps_t0
                if elapsed >= 5.0:
                    self.get_logger().info(f'Camera FPS: {self._fps_count / elapsed:.1f}')
                    self._fps_count = 0
                    self._fps_t0    = time.time()
            else:
                self.get_logger().warning("Falha ao ler frame da câmera. Reconectando...")
                cap.release()
                time.sleep(1.0)

        cap.release()

    def zero_cb(self, request, response):
        """Serviço para zerar a origem na posição atual do robô."""
        self.off_x = self.raw_x
        self.off_y = self.raw_y
        self.off_theta = self.raw_theta
        response.success = True
        response.message = 'Ground truth zerado'
        self.get_logger().info("Origem do Ground Truth zerada na pose atual.")
        return response

    def reset_cb(self, request, response):
        """Serviço para remover o offset (volta as coordenadas absolutas da arena)."""
        self.off_x = 0.0
        self.off_y = 0.0
        self.off_theta = 0.0
        response.success = True
        response.message = 'Offsets resetados'
        self.get_logger().info("Offsets de Ground Truth resetados para 0.")
        return response

    def euler_to_quaternion(self, yaw) -> Quaternion:
        """Converte Yaw em radianos para Quaternion."""
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)

    def process_frame(self):
        """Callback principal rodando a capture_rate_hz."""
        with self.frame_lock:
            if self.current_frame is None:
                return
            frame_display = self.current_frame.copy()

        # 1. Undistort apenas no frame de detecção (economiza CPU no display)
        if self.cam_mat is not None and self.dist_coeff is not None:
            frame_detect = cv2.undistort(frame_display, self.cam_mat, self.dist_coeff)
        else:
            frame_detect = frame_display

        gray = cv2.cvtColor(frame_detect, cv2.COLOR_BGR2GRAY)

        # 2. Detectar ArUcos
        try:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        except AttributeError: # Compatibilidade com cv2.aruco.ArucoDetector (OpenCV 4.7+)
            detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            ids = ids.flatten()
            cv2.aruco.drawDetectedMarkers(frame_display, corners, ids)

            # 3. Recalcular Homografia (Se os 4 cantos [0,1,2,3] estiverem visíveis)
            src_pts = []
            for i in range(4):
                if i in ids:
                    idx = np.where(ids == i)[0][0]
                    # Usa o centro do ArUco de canto como ponto de referência
                    c_center = np.mean(corners[idx][0], axis=0)
                    src_pts.append(c_center)

            if len(src_pts) == 4:
                src_pts = np.array(src_pts, dtype=np.float32)
                H, _ = cv2.findHomography(src_pts, self.dst_pts)
                self.homography = H

            # 4. Calcular Pose do Robô se o marcador estiver visível e H existir
            if self.robot_marker_id in ids and self.homography is not None:
                r_idx = np.where(ids == self.robot_marker_id)[0][0]
                r_corners = corners[r_idx][0] # [Top-Left, Top-Right, Bottom-Right, Bottom-Left]

                # a. Centro em pixels -> para metros
                c_px = np.mean(r_corners, axis=0)
                pt_center = np.array([[[c_px[0], c_px[1]]]], dtype=np.float32)
                trans_center = cv2.perspectiveTransform(pt_center, self.homography)
                self.raw_x, self.raw_y = trans_center[0][0]

                # b. Borda superior (média dos pontos 0 e 1) -> calcula frente
                f_px = (r_corners[0] + r_corners[1]) / 2.0
                pt_front = np.array([[[f_px[0], f_px[1]]]], dtype=np.float32)
                trans_front = cv2.perspectiveTransform(pt_front, self.homography)
                fx, fy = trans_front[0][0]

                # c. raw_theta usando atan2
                self.raw_theta = math.atan2(fy - self.raw_y, fx - self.raw_x)

                # d. Aplica offset de transformação de coordenadas
                dx = self.raw_x - self.off_x
                dy = self.raw_y - self.off_y
                
                local_x = dx * math.cos(self.off_theta) + dy * math.sin(self.off_theta)
                local_y = -dx * math.sin(self.off_theta) + dy * math.cos(self.off_theta)
                local_theta = self.raw_theta - self.off_theta

                # Normalização do ângulo final entre -PI e PI
                local_theta = math.atan2(math.sin(local_theta), math.cos(local_theta))

                # e. Publica a pose no tópico /ground_truth
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'odom'
                
                msg.pose.position.x = float(local_x)
                msg.pose.position.y = float(local_y)
                msg.pose.position.z = 0.0
                msg.pose.orientation = self.euler_to_quaternion(local_theta)
                
                if self.calibration_available:
                    self.pose_pub.publish(msg)

        # Publicar Imagem para Debug (RViz)
        img_msg = self.cv_bridge.cv2_to_imgmsg(frame_display, encoding="bgr8")
        self.image_pub.publish(img_msg)

    def destroy_node(self):
        self.running = False
        self.capture_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()