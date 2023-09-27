import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')),
        launch_arguments={"pointcloud.enable": "true"}.items()
    )

    camera_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="camera_tf_publisher", output="log",
        arguments=['0.0452', '-0.0018', '0.1368', '0.5455', '-0.0223', '0.8328', '-0.0917', 'panda_link8', 'camera_link'],)

    return LaunchDescription([camera_node, camera_tf])
