# # Copyright 2020 Tier IV, Inc. All rights reserved.
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #     http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction
import yaml

def launch_setup(context, *args, **kwargs):

    camera_name = LaunchConfiguration("camera_name").perform(context)
    camera_ns = LaunchConfiguration("camera_ns").perform(context)

    camera_param_path=FindPackageShare("jrwt_usb_camera").perform(context)+"/param/"+camera_name+".param.yaml"
    with open(camera_param_path, "r") as f:
        camera_yaml_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    container = ComposableNodeContainer(
        name="camera_container",
        namespace=camera_ns,
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="gscam",
                plugin="gscam::GSCam",
                name="jrwt_usb_camera_node",
                parameters=[{
                    "gscam_config": camera_yaml_param['gscam_config'],
                    "frame_id": camera_yaml_param['frame_id'],
                }],
                remappings=[
                    ("/"+camera_ns+'/camera/image_raw', "/"+camera_ns+"/"+camera_name+"/image_raw"),
                    ("/"+camera_ns+'/camera/camera_info', "/"+camera_ns+"/"+camera_name+"/camera_info"),
                    ("/"+camera_ns+'/camera/image_raw/compressed', "/"+camera_ns+"/"+camera_name+"/image_raw/compressed"),
                    ("/"+camera_ns+'/camera/image_raw/theora', "/"+camera_ns+"/"+camera_name+"/image_raw/theora"),
                    ("/"+camera_ns+'/camera/image_raw/compressedDepth', "/"+camera_ns+"/"+camera_name+"/image_raw/compressedDepth"),

                ],
            ),

        ],

    )
    return [container]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg("camera_name")
    add_launch_arg("camera_ns")


    return launch.LaunchDescription(
        launch_arguments
        + [OpaqueFunction(function=launch_setup)]
    )