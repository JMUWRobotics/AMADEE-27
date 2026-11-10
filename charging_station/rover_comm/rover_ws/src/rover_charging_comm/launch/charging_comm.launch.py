"""Startet den Kommunikations-Node eines Rovers.

    ros2 launch rover_charging_comm charging_comm.launch.py rover_id:=rover_01 station_host:=192.168.1.10
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    rover_id = LaunchConfiguration("rover_id")
    return LaunchDescription([
        DeclareLaunchArgument("rover_id", default_value="rover_01",
                              description="eindeutige Rover-ID (auch Namespace)"),
        DeclareLaunchArgument("station_host", default_value="raspberrypi.local",
                              description="Hostname oder IP der Ladestation"),
        DeclareLaunchArgument("station_port", default_value="9000",
                              description="TCP-Port der Ladestation"),
        Node(
            package="rover_charging_comm",
            executable="charging_comm_node",
            name="charging_comm",
            namespace=rover_id,
            output="screen",
            parameters=[{
                # value_type erzwingen, sonst würde z. B. die ID "01" als Zahl gelesen
                "rover_id": ParameterValue(rover_id, value_type=str),
                "station_host": ParameterValue(LaunchConfiguration("station_host"), value_type=str),
                "station_port": ParameterValue(LaunchConfiguration("station_port"), value_type=int),
            }],
        ),
    ])
