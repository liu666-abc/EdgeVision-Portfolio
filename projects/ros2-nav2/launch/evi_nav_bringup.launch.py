import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params")
    usart_port = LaunchConfiguration("usart_port", default="/dev/ttyACM0")

    wheeltec_dir = get_package_share_directory("turn_on_wheeltec_robot")
    wheeltec_launch_dir = os.path.join(wheeltec_dir, "launch")
    nav2_dir = get_package_share_directory("wheeltec_nav2")
    nav2_launch_dir = os.path.join(nav2_dir, "launch")

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

    base_node = Node(
        package="turn_on_wheeltec_robot",
        executable="wheeltec_robot_node",
        output="screen",
        parameters=[common_params],
    )
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value="/home/wheeltec/evi/a3_nav2/map_a4_20261018.yaml",
            ),
            DeclareLaunchArgument(
                "params",
                default_value="/home/wheeltec/evi/a3_nav2/param_mini_akm_a3.yaml",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("usart_port", default_value="/dev/ttyACM0"),
            base_node,
            joint_state_publisher,
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
                name="waypoint_cycle",
                package="nav2_waypoint_cycle",
                executable="nav2_waypoint_cycle",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_launch_dir, "bringup_launch.py")
                ),
                launch_arguments={
                    "map": map_file,
                    "params_file": params_file,
                    "use_sim_time": use_sim_time,
                    "autostart": "true",
                }.items(),
            ),
        ]
    )

