#!/usr/bin/env python3

import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    package_name = 'robot'
    
    pkg_path = get_package_share_directory(package_name)
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.xacro')
    rviz_config = os.path.join(pkg_path, 'rviz', 'robot.rviz')
    
    print(f"Usando XACRO: {xacro_file}")
    
    # Verificar y configurar entorno para RViz
    def get_rviz_env():
        env = os.environ.copy()
        
        # Limpiar Snap del PATH
        if 'PATH' in env:
            paths = env['PATH'].split(':')
            clean_paths = [p for p in paths if '/snap/' not in p]
            env['PATH'] = ':'.join(clean_paths)
        
        # Configurar LD_LIBRARY_PATH con Ogre
        lib_paths = [
            '/usr/lib/x86_64-linux-gnu',
            '/lib/x86_64-linux-gnu', 
            '/opt/ros/humble/lib',
            '/opt/ros/humble/lib/x86_64-linux-gnu'
        ]
        
        # Buscar Ogre
        ogre_found = False
        for lib_dir in lib_paths:
            ogre_lib = os.path.join(lib_dir, 'libOgreMain.so.1.12.1')
            if os.path.exists(ogre_lib):
                print(f"Ogre encontrado en: {ogre_lib}")
                ogre_found = True
                break
        
        if not ogre_found:
            print("ADVERTENCIA: Ogre no encontrado. RViz puede fallar.")
            print("Ejecuta: sudo apt install libogre-1.12.0")
        
        env['LD_LIBRARY_PATH'] = ':'.join(lib_paths)
        env['QT_QPA_PLATFORM'] = 'xcb'
        
        return env
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', xacro_file]),
            }]
        ),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        
        # RViz con entorno personalizado
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            env=get_rviz_env()
        )
    ])