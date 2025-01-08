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
    msg_pub_topic_name_launch_arg = DeclareLaunchArgument(
        "dosod_msg_pub_topic_name", default_value=TextSubstitution(text="hobot_dosod")
    )
    model_file_name_launch_arg = DeclareLaunchArgument(
        "dosod_model_file_name", default_value=TextSubstitution(text="config/dosod_mlp3x_l_rep-int8.bin")
    )
    vocabulary_file_name_launch_arg = DeclareLaunchArgument(
        "dosod_vocabulary_file_name", default_value=TextSubstitution(text="config/offline_vocabulary.json")
    )
    dump_raw_launch_arg = DeclareLaunchArgument(
        "dosod_dump_raw_img", default_value=TextSubstitution(text="0")
    )
    dump_render_launch_arg = DeclareLaunchArgument(
        "dosod_dump_render_img", default_value=TextSubstitution(text="0")
    )
    port_interaction_arg = DeclareLaunchArgument(
        "ws_port_interaction", default_value=TextSubstitution(text="8081")
    )
    score_threshold_launch_arg = DeclareLaunchArgument(
        "dosod_score_threshold", default_value=TextSubstitution(text="0.6")
    )
    trigger_mode_launch_arg = DeclareLaunchArgument(
        "dosod_trigger_mode", default_value=TextSubstitution(text="0")
    )
    # jpeg->nv12
    nv12_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_decode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_out_mode': 'ros',
            'codec_sub_topic': '/image',
            'codec_pub_topic': '/image_raw'
        }.items()
    )

    # web展示pkg
    web_smart_topic_arg = DeclareLaunchArgument(
        'smart_topic',
        default_value='/hobot_dosod',
        description='websocket smart topic')
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image',
            'websocket_smart_topic': LaunchConfiguration('smart_topic')
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
            {"dump_raw_img": LaunchConfiguration(
                "dosod_dump_raw_img")},
            {"dump_render_img": LaunchConfiguration(
                "dosod_dump_render_img")},
            {"msg_pub_topic_name": LaunchConfiguration(
                "dosod_msg_pub_topic_name")},
            {"score_threshold": LaunchConfiguration(
                "dosod_score_threshold")},
            {"iou_threshold": 0.5},
            {"nms_top_k": 50},
            {"is_homography": 0},
            {"y_offset": 800.0},
            {"ros_img_sub_topic_name": '/image_raw'},
            {"ai_msg_pub_topic_name": '/hobot_dosod'},
            {"model_file_name": LaunchConfiguration(
                "dosod_model_file_name")},
            {"vocabulary_file_name": LaunchConfiguration(
                "dosod_vocabulary_file_name")},
            {"trigger_mode": LaunchConfiguration(
                "dosod_trigger_mode")},
            {"port_interaction": LaunchConfiguration("ws_port_interaction")}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    return LaunchDescription([
        web_smart_topic_arg,
        msg_pub_topic_name_launch_arg,
        dump_raw_launch_arg,
        dump_render_launch_arg,
        port_interaction_arg,
        model_file_name_launch_arg,
        vocabulary_file_name_launch_arg,
        score_threshold_launch_arg,
        trigger_mode_launch_arg,
        # 图片编解码&发布pkg
        nv12_codec_node,
        # 启动dosod pkg
        dosod_node,
        # 启动web展示pkg
        web_node
    ])
