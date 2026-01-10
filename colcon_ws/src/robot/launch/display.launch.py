#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    package_name = 'robot'
    
    # Obtener rutas - CORREGIDO para evitar problemas con Snap
    pkg_path = get_package_share_directory(package_name)
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.xacro')
    rviz_config = os.path.join(pkg_path, 'rviz', 'robot_display.rviz')
    
    # Configuración de entorno para evitar problemas
    os.environ['QT_QPA_PLATFORM'] = 'xcb'
    
    # Verificar que el archivo XACRO existe
    if not os.path.exists(xacro_file):
        raise FileNotFoundError(f"XACRO no encontrado: {xacro_file}")
    
    print(f"Usando XACRO: {xacro_file}")
    print(f"Usando configuración RViz: {rviz_config}")
    
    return LaunchDescription([
        # Robot State Publisher - Publica el robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', xacro_file]),
                'use_sim_time': False,
                'publish_frequency': 30.0
            }],
            emulate_tty=True  # Para ver colores en la terminal
        ),
        
        # Joint State Publisher (sin GUI para evitar problemas)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'source_list': ['joint_states'],
                'rate': 30
            }],
            emulate_tty=True
        ),
        
        # RViz2 - Visualizador
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            emulate_tty=True,
            parameters=[{'use_sim_time': False}]
        )
    ])