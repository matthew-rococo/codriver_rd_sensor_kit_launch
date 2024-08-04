#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import subprocess


def generate_launch_description():

    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    #声明命令行替换
    add_launch_arg("namespace",description="name space of this node")
    add_launch_arg("model",description="lidar type")
    add_launch_arg('sensor_ip', description="device ip")
    add_launch_arg('sensor_frame',description="frame id")
    add_launch_arg('topic_name')

    p = subprocess.Popen("echo $ROS_DISTRO", stdout=subprocess.PIPE, shell=True)
    driver_node = ""
    ros_version = p.communicate()[0]  
    if ros_version == b'foxy\n' or ros_version == b'galactic\n' or ros_version == b'humble\n':
        print("ROS VERSION: foxy/galactic/humble")
        
        #在parameters中使用命令行替换
        driver_node = LifecycleNode(package='lslidar_driver',
                                    namespace=LaunchConfiguration('namespace'),
                                    executable='lslidar_driver_node',
                                    name='lslidar_driver_node',
                                    output='screen',
                                    emulate_tty=True,
                                    parameters=[{
                                        "lidar_type":LaunchConfiguration('model'),
                                        "device_ip" : LaunchConfiguration('sensor_ip'),
                                        "frame_id":LaunchConfiguration('sensor_frame'),
                                        "topic_name" :LaunchConfiguration('topic_name'),
                                        }]
                                    )
    else:
        print("Please configure the ros environment")
        exit()

    return LaunchDescription([
        launch_arguments,
        driver_node
    ])