import time
import threading
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray, Float64, MultiArrayDimension

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        # Parâmetros ROS
        self.declare_parameter('serial_port', '/dev/rfcomm0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('sync_retries', 3)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.sync_retries = self.get_parameter('sync_retries').value

        # Publishers
        self.pub_raw = self.create_publisher(Float64MultiArray, '/raw_sensors', 10)
        self.pub_latency = self.create_publisher(Float64, '/latency_ms', 10)

        # Subscriber
        self.sub_cmd = self.create_subscription(
            String,
            '/serial_cmd',
            self.cmd_callback,
            10
        )

        # Estado interno
        self.ser = None
        self.serial_lock = threading.Lock()
        self.clock_offset = 0.0
        self.running = True

        # Inicialização da porta serial
        self.connect_and_sync()

        # Inicia a thread de leitura serial (Daemon)
        self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.read_thread.start()

    def connect_and_sync(self):
        """Tenta abrir a porta serial e executar o handshake SYNC."""
        while self.running:
            try:
                with self.serial_lock:
                    if self.ser and self.ser.is_open:
                        self.ser.close()
                    
                    self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=2.0)
                
                self.get_logger().info(f"Conectado à porta {self.serial_port}.")
                
                # Handshake
                if self.perform_sync():
                    break
                else:
                    self.get_logger().warning("Falha no handshake SYNC. Reconectando...")
            
            except serial.SerialException as e:
                self.get_logger().error(f"Erro ao abrir {self.serial_port}: {e}. Tentando em 3s...")
                time.sleep(3.0)

    def perform_sync(self) -> bool:
        """Executa a lógica de sincronização de relógio com o ESP32."""
        for tentativa in range(self.sync_retries):
            try:
                self.ser.reset_input_buffer()
                
                t1 = time.time()
                with self.serial_lock:
                    self.ser.write(b"SYNC\n")
                
                # Leitura síncrona aguardando a resposta
                linha = self.ser.readline().decode('utf-8').strip()
                t2 = time.time()
                
                if linha.startswith("SYNC:"):
                    t_esp = float(linha.split(":")[1])
                    self.clock_offset = ((t1 + t2) / 2.0) - t_esp
                    rtt_ms = (t2 - t1) * 1000.0
                    
                    self.get_logger().info(
                        f"SYNC ok. offset={self.clock_offset:.4f} s, RTT={rtt_ms:.1f} ms"
                    )
                    return True
            except Exception as e:
                self.get_logger().warning(f"Exceção durante SYNC: {e}")
                
            self.get_logger().warning(f"Tentativa de SYNC {tentativa + 1}/{self.sync_retries} falhou.")
        
        return False

    def cmd_callback(self, msg: String):
        """Callback de comandos via ROS para envio direto ao ESP32."""
        if not self.ser or not self.ser.is_open:
            return

        with self.serial_lock:
            try:
                comando = f"{msg.data}\n"
                self.ser.write(comando.encode('utf-8'))
            except serial.SerialException as e:
                self.get_logger().error(f"Erro ao enviar comando serial: {e}")

    def serial_read_loop(self):
        """Loop infinito da thread secundária para leitura e parsing da serial."""
        while self.running:
            if not self.ser or not self.ser.is_open:
                time.sleep(0.1)
                continue

            try:
                linha = self.ser.readline().decode('utf-8').strip()
                if not linha:
                    continue
                
                # Tratamento de Logs do ESP32
                if linha.startswith('INFO:'):
                    self.get_logger().info(linha[5:])
                    continue
                
                # Resposta tardia ou redundante do SYNC
                if linha.startswith('SYNC:'):
                    continue

                # Tentativa de parse do CSV (t, dt, dE, dD, gz)
                campos = linha.split(',')
                if len(campos) != 5:
                    self.get_logger().warning(f"Descartando pacote inválido: {linha}")
                    continue
                
                t_esp = float(campos[0])
                dt = float(campos[1])
                dE = float(campos[2])
                dD = float(campos[3])
                gz = float(campos[4])

                # 1. Publicar /raw_sensors
                msg_raw = Float64MultiArray()
                msg_raw.layout.dim.append(
                    MultiArrayDimension(label='fields', size=5, stride=5)
                )
                msg_raw.data = [t_esp, dt, dE, dD, gz]
                self.pub_raw.publish(msg_raw)

                # 2. Calcular e publicar /latency_ms
                t_ros = time.time()
                latency_ms = (t_ros - (t_esp + self.clock_offset)) * 1000.0
                
                msg_lat = Float64()
                msg_lat.data = latency_ms
                self.pub_latency.publish(msg_lat)

            except serial.SerialException as e:
                self.get_logger().error(f"Exceção de leitura serial: {e}. Iniciando reconexão...")
                self.connect_and_sync()
            except ValueError as e:
                self.get_logger().warning(f"Erro de conversão numérica (ValueError): {e}")
            except Exception as e:
                self.get_logger().error(f"Erro genérico no loop serial: {e}")

    def destroy_node(self):
        """Garantição de encerramento limpo da porta serial."""
        self.running = False
        if self.ser and self.ser.is_open:
            with self.serial_lock:
                self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Encerrando serial_bridge_node via teclado.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()