# Copyright (c) 2024，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from ament_index_python.packages import get_package_prefix

def generate_launch_description():

    # args that can be set from the command line or a default will be used
    image_width_launch_arg = DeclareLaunchArgument(
        "dosod_image_width", default_value=TextSubstitution(text="1920")
    )
    image_height_launch_arg = DeclareLaunchArgument(
        "dosod_image_height", default_value=TextSubstitution(text="1080")
    )
    msg_pub_topic_name_launch_arg = DeclareLaunchArgument(
        "dosod_msg_pub_topic_name", default_value=TextSubstitution(text="hobot_dosod")
    )
    dump_render_launch_arg = DeclareLaunchArgument(
        "dosod_dump_render_img", default_value=TextSubstitution(text="0")
    )
    dump_raw_launch_arg = DeclareLaunchArgument(
        "dosod_dump_raw_img", default_value=TextSubstitution(text="0")
    )
    model_file_name_launch_arg = DeclareLaunchArgument(
        "dosod_model_file_name", default_value=TextSubstitution(text="config/3x-l_epoch_100_rep-coco80-without-nms-int8.bin")
    )
    dump_ai_launch_arg = DeclareLaunchArgument(
        "dosod_dump_ai_result", default_value=TextSubstitution(text="0")
    )
    dump_raw_path_launch_arg = DeclareLaunchArgument(
        "dosod_dump_raw_path", default_value=TextSubstitution(text=".")
    )
    dump_ai_path_launch_arg = DeclareLaunchArgument(
        "dosod_dump_ai_path", default_value=TextSubstitution(text=".")
    )
    dump_render_path_launch_arg = DeclareLaunchArgument(
        "dosod_dump_render_path", default_value=TextSubstitution(text=".")
    )
    vocabulary_file_name_launch_arg = DeclareLaunchArgument(
        "dosod_vocabulary_file_name", default_value=TextSubstitution(text="config/offline_vocabulary.json")
    )
    score_threshold_launch_arg = DeclareLaunchArgument(
        "dosod_score_threshold", default_value=TextSubstitution(text="0.6")
    )
    trigger_mode_launch_arg = DeclareLaunchArgument(
        "dosod_trigger_mode", default_value=TextSubstitution(text="0")
    )

    # web展示pkg
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_type': 'mjpeg',
            'websocket_image_topic': '/image',
            'websocket_smart_topic': LaunchConfiguration("dosod_msg_pub_topic_name")
        }.items()
    )

    # jpeg图片编码&发布pkg
    jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_out_mode': 'ros',
            'codec_sub_topic': '/image_combine_rectify',
            'codec_pub_topic': '/image'
        }.items()
    )

    # 算法pkg
    dosod_node = Node(
        package='hobot_dosod',
        executable='hobot_dosod',
        output='screen',
        parameters=[
            {"feed_type": 1},
            {"is_shared_mem_sub": 0},
            {"dump_ai_result": LaunchConfiguration(
                "dosod_dump_ai_result")},
            {"dump_raw_img": LaunchConfiguration(
                "dosod_dump_raw_img")},
            {"dump_render_img": LaunchConfiguration(
                "dosod_dump_render_img")},
            {"dump_ai_path": LaunchConfiguration(
                "dosod_dump_ai_path")},
            {"dump_render_path": LaunchConfiguration(
                "dosod_dump_render_path")},
            {"dump_raw_path": LaunchConfiguration(
                "dosod_dump_raw_path")},
            {"msg_pub_topic_name": LaunchConfiguration(
                "dosod_msg_pub_topic_name")},
            {"ros_img_sub_topic_name": "/image_combine_rectify"},
            {"model_file_name": LaunchConfiguration(
                "dosod_model_file_name")},
            {"vocabulary_file_name": LaunchConfiguration(
                "dosod_vocabulary_file_name")},
            {"trigger_mode": LaunchConfiguration(
                "dosod_trigger_mode")},
            {"roi": True},
            {"roi_x1": 0.0},
            {"roi_y1": 336.0},
            {"roi_x2": 896.0},
            {"roi_y2": 672.0},
            {"score_threshold": LaunchConfiguration(
                "dosod_score_threshold")}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # 遥控控制开关
    joy_develop_node = Node(
        package='hobot_joy_develop',
        executable='hobot_joy_develop',
        output='screen',
        parameters=[
            {"is_shared_mem_sub": 0},
            {"ros_img_sub_topic_name": "/image_combine_rectify"},
            {"dump_raw_path": "capture"}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    shared_mem_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('hobot_shm'),
                        'launch/hobot_shm.launch.py'))
            )

    return LaunchDescription([
        image_width_launch_arg,
        image_height_launch_arg,
        msg_pub_topic_name_launch_arg,
        dump_ai_launch_arg,
        dump_render_launch_arg,
        dump_raw_launch_arg,
        dump_ai_path_launch_arg,
        dump_raw_path_launch_arg,
        dump_render_path_launch_arg,
        model_file_name_launch_arg,
        vocabulary_file_name_launch_arg,
        score_threshold_launch_arg,
        trigger_mode_launch_arg,
        jpeg_codec_node,
        # 启动dosod pkg
        dosod_node,
        # 启动web展示pkg
        web_node,
        # 遥控器节点
        joy_develop_node
    ])
