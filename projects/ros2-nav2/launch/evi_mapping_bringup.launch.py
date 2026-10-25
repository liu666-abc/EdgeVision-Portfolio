import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    usart_port = LaunchConfiguration("usart_port", default="/dev/ttyACM0")

    wheeltec_dir = get_package_share_directory("turn_on_wheeltec_robot")
    wheeltec_launch_dir = os.path.join(wheeltec_dir, "launch")
    slam_dir = get_package_share_directory("wheeltec_slam_toolbox")
    slam_params = os.path.join(
        slam_dir,
        "config",
        "mapper_params_online_async.yaml",
    )

    common_params = {
        "usart_port_name": usart_port,
        "serial_baud_rate": 115200,
        "robot_frame_id": "base_footprint",
        "odom_frame_id": "odom_combined",
        "cmd_vel": "cmd_vel",
        "akm_cmd_vel": "none",
        "product_number": 0,
        "odom_x_scale": 1.051,
        "odom_y_scale": 1.0,
        "odom_z_scale_positive": 1.0,
        "odom_z_scale_negative": 1.0,
        "car_mode": "mini_akm",
        "ranger_avoid_flag": False,
        "ultrasonic_avoid": False,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("usart_port", default_value="/dev/ttyACM0"),
            Node(
                package="turn_on_wheeltec_robot",
                executable="wheeltec_robot_node",
                output="screen",
                parameters=[common_params],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(wheeltec_launch_dir, "imu_processor.launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(wheeltec_launch_dir, "wheeltec_ekf.launch.py")
                ),
                launch_arguments={
                    "carto_slam": "false",
                    "robot_nav": "false",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        wheeltec_launch_dir,
                        "robot_mode_description.launch.py",
                    )
                ),
                launch_arguments={"car_mode": "mini_akm"}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(wheeltec_launch_dir, "wheeltec_lidar.launch.py")
                )
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_params],
                remappings=[("odom", "odom_combined")],
            ),
        ]
    )

