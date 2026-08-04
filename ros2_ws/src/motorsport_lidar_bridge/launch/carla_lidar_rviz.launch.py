"""Launch the bridge and RViz. / 启动桥接节点和 RViz。"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
import os


def generate_launch_description():
    """Build the launch graph. / 建立启动流程。"""
    package_share = get_package_share_directory("motorsport_lidar_bridge")
    rviz_config = os.path.join(package_share, "rviz", "motorsport_lidar.rviz")

    return LaunchDescription(
        [
            # Allow headless automated tests. / 允许无窗口自动测试。
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("frames", default_value="1000"),
            DeclareLaunchArgument("speed", default_value="2.0"),
            DeclareLaunchArgument("scenario", default_value="straight"),
            DeclareLaunchArgument("control", default_value="scripted"),
            DeclareLaunchArgument("target_laps", default_value="1"),
            DeclareLaunchArgument("localization_drift_per_m", default_value="0.012"),
            DeclareLaunchArgument("cone_dropout_rate", default_value="0.0"),
            DeclareLaunchArgument("cone_noise_std_m", default_value="0.0"),
            DeclareLaunchArgument("fault_seed", default_value="501"),
            Node(
                package="motorsport_lidar_bridge",
                executable="carla_lidar_bridge",
                name="carla_lidar_bridge",
                output="screen",
                # Close RViz when the finite test ends. / 有限帧测试结束时一起关闭 RViz。
                on_exit=Shutdown(reason="CARLA bridge finished"),
                parameters=[
                    {
                        "simulation_frames": ParameterValue(
                            LaunchConfiguration("frames"),
                            value_type=int,
                        ),
                        "vehicle_speed_mps": ParameterValue(
                            LaunchConfiguration("speed"),
                            value_type=float,
                        ),
                        "scenario_shape": LaunchConfiguration("scenario"),
                        "control_mode": LaunchConfiguration("control"),
                        "target_laps": ParameterValue(
                            LaunchConfiguration("target_laps"),
                            value_type=int,
                        ),
                        "localization_drift_per_m": ParameterValue(
                            LaunchConfiguration("localization_drift_per_m"),
                            value_type=float,
                        ),
                        "cone_dropout_rate": ParameterValue(
                            LaunchConfiguration("cone_dropout_rate"),
                            value_type=float,
                        ),
                        "cone_noise_std_m": ParameterValue(
                            LaunchConfiguration("cone_noise_std_m"),
                            value_type=float,
                        ),
                        "fault_seed": ParameterValue(
                            LaunchConfiguration("fault_seed"),
                            value_type=int,
                        ),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            ),
        ]
    )
