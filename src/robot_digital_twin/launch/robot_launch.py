import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'robot_digital_twin'
    
    # Obtém o diretório de instalação do pacote (onde a pasta config/ e urdf/ foram copiadas)
    pkg_share = get_package_share_directory(pkg_name)

    # ==========================================
    # 1. Configurações de Launch (Variáveis)
    # ==========================================
    serial_port = LaunchConfiguration('serial_port')
    use_rviz = LaunchConfiguration('use_rviz')
    imu_weight = LaunchConfiguration('imu_weight')

    # ==========================================
    # 2. Argumentos de Launch (Expostos ao usuário)
    # ==========================================
    arg_serial = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/rfcomm0',
        description='Porta serial Bluetooth do ESP32'
    )
    
    arg_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Inicia o RViz2 automaticamente'
    )
    
    arg_imu = DeclareLaunchArgument(
        'imu_weight',
        default_value='0.98',
        description='Peso do giroscópio na fusão de odometria'
    )

    # ==========================================
    # 3. Caminhos de Arquivos (URDF e Calibração)
    # ==========================================
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    
    # Leitura do URDF em runtime
    try:
        with open(urdf_file, 'r') as infp:
            robot_desc = infp.read()
    except FileNotFoundError:
        robot_desc = ""
        print(f"[AVISO] Arquivo URDF não encontrado em: {urdf_file}")

    camera_matrix_path = os.path.join(pkg_share, 'config', 'camera_matrix.npy')
    dist_coeffs_path = os.path.join(pkg_share, 'config', 'dist_coeffs.npy')
    rviz_config_path = os.path.join(pkg_share, 'config', 'rviz_config.rviz')

    # ==========================================
    # 4. Definição dos Nós
    # ==========================================
    
    # Nó 1: Ponte Serial
    serial_bridge_node = Node(
        package=pkg_name,
        executable='serial_bridge',
        name='serial_bridge_node',
        output='screen',
        parameters=[{'serial_port': serial_port}]
    )

    # Nó 2: Odometria Estimada
    odom_node = Node(
        package=pkg_name,
        executable='odom',
        name='odom_node',
        output='screen',
        parameters=[{'w_imu': imu_weight}]
    )

    # Nó 3: Controlador (Missões e Manual) — abre janela xterm interativa
    controller_node = Node(
        package=pkg_name,
        executable='controller',
        name='controller_node',
        output='screen',
        emulate_tty=True,
        prefix='xterm -e',
    )

    # Nó 4: Ground Truth (Visão Computacional)
    ground_truth_node = Node(
        package=pkg_name,
        executable='ground_truth',
        name='ground_truth_node',
        output='screen',
        parameters=[{
            'camera_matrix_path': camera_matrix_path,
            'dist_coeffs_path': dist_coeffs_path
        }]
    )

    # Nó 5: Gêmeo Digital (Agregador)
    twin_node = Node(
        package=pkg_name,
        executable='twin',
        name='twin_node',
        output='screen'
    )

    # Nó 6: Logger de Experimentos
    logger_node = Node(
        package=pkg_name,
        executable='logger',
        name='logger_node',
        output='screen'
    )

    # Nó 7: Robot State Publisher (Nó nativo do ROS)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Nó 8: RViz2 (Nó nativo, com condicional IfCondition)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz)
    )

    # ==========================================
    # 5. Retorno do LaunchDescription
    # ==========================================
    return LaunchDescription([
        arg_serial,
        arg_rviz,
        arg_imu,
        serial_bridge_node,
        odom_node,
        controller_node,
        ground_truth_node,
        twin_node,
        logger_node,
        rsp_node,
        rviz_node
    ])