# rsdlab@rsdlab:~/crane_camera_ws/src/crane_aruco_control/launch/aruco_standalone.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # パッケージ名
    package_name = 'crane_aruco_control'
    
    return LaunchDescription([
        # 1. camera_publisher ノードの定義
        Node(
            package=package_name,
            executable='camera_publisher', # setup.py で定義されたエントリーポイント名
            name='camera_publisher_node',
            output='screen',
        ),

        # 2. aruco_controller ノードの定義
        Node(
            package=package_name,
            executable='aruco_controller', # setup.py で定義されたエントリーポイント名
            name='aruco_controller_node',
            output='screen',
        ),
    ])
