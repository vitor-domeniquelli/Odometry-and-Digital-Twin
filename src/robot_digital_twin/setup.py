import os
from setuptools import setup

package_name = 'robot_digital_twin'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Copia os arquivos de Launch e Configuração para a área de install do ROS2
        ('share/' + package_name + '/launch', ['launch/robot_launch.py']),
        ('share/' + package_name + '/config', [
            'config/rviz_config.rviz',
            'config/camera_matrix.npy',
            'config/dist_coeffs.npy',
        ]),
        ('share/' + package_name + '/urdf', ['urdf/robot.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vitor Domeniquelli Chagas',
    maintainer_email='vitor.domeniquelli@aluno.ufabc.edu.br',
    description='Odometria e gêmeo digital para robô de esteira com ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge  = robot_digital_twin.serial_bridge_node:main',
            'odom           = robot_digital_twin.odom_node:main',
            'controller     = robot_digital_twin.controller_node:main',
            'ground_truth   = robot_digital_twin.ground_truth_node:main',
            'twin           = robot_digital_twin.twin_node:main',
            'logger         = robot_digital_twin.logger_node:main',
        ],
    },
)