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
        "dosod_model_file_name", default_value=TextSubstitution(text="config/dosod_mlp3x_l_rep-int8.bin")
    )
    vocabulary_file_name_launch_arg = DeclareLaunchArgument(
        "dosod_vocabulary_file_name", default_value=TextSubstitution(text="config/offline_vocabulary.json")
    )
    score_threshold_launch_arg = DeclareLaunchArgument(
        "dosod_score_threshold", default_value=TextSubstitution(text="0.2")
    )
    trigger_mode_launch_arg = DeclareLaunchArgument(
        "dosod_trigger_mode", default_value=TextSubstitution(text="0")
    )
    camera_type = os.getenv('CAM_TYPE')
    print("camera_type is ", camera_type)
    
    cam_node = None
    camera_type_mipi = None
    camera_device_arg = None

    if camera_type == "usb":
        # usb cam图片发布pkg
        usb_cam_device_arg = DeclareLaunchArgument(
            'device',
            default_value='/dev/video0',
            description='usb camera device')

        usb_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_usb_cam'),
                    'launch/hobot_usb_cam.launch.py')),
            launch_arguments={
                'usb_image_width': LaunchConfiguration('dosod_image_width'),
                'usb_image_height': LaunchConfiguration('dosod_image_height'),
                'usb_framerate': '30',
                'usb_video_device': LaunchConfiguration('device')
            }.items()
        )
        print("using usb cam")
        cam_node = usb_node
        camera_type_mipi = False
        camera_device_arg = usb_cam_device_arg

    elif camera_type == "fb":
        # 本地图片发布
        feedback_picture_arg = DeclareLaunchArgument(
            'publish_image_source',
            default_value='./config/000000160864.jpg',
            description='feedback picture')

        fb_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_image_publisher'),
                    'launch/hobot_image_publisher.launch.py')),
            launch_arguments={
                'publish_image_source': LaunchConfiguration('publish_image_source'),
                'publish_image_format': 'jpg',
                'publish_is_shared_mem': 'True',
                'publish_message_topic_name': '/hbmem_img',
                'publish_fps': '10',
                'publish_is_loop': 'True',
                'publish_output_image_w': LaunchConfiguration('dosod_image_width'),
                'publish_output_image_h': LaunchConfiguration('dosod_image_height')
            }.items()
        )

        print("using feedback")
        cam_node = fb_node
        camera_type_mipi = True
        camera_device_arg = feedback_picture_arg

    else:
        if camera_type == "mipi":
            print("using mipi cam")
        else:
            print("invalid camera_type ", camera_type,
                ", which is set with export CAM_TYPE=usb/mipi/fb, using default mipi cam")
        # mipi cam图片发布pkg
        mipi_cam_device_arg = DeclareLaunchArgument(
            'device',
            default_value='F37',
            description='mipi camera device')
        mipi_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('mipi_cam'),
                    'launch/mipi_cam.launch.py')),
            launch_arguments={
                'mipi_image_width': LaunchConfiguration('dosod_image_width'),
                'mipi_image_height': LaunchConfiguration('dosod_image_height'),
                'mipi_io_method': 'shared_mem',
                'mipi_frame_ts_type': 'realtime',
                'mipi_video_device': LaunchConfiguration('device')
            }.items()
        )

        cam_node = mipi_node
        camera_type_mipi = True
        camera_device_arg = mipi_cam_device_arg

    # jpeg图片编码&发布pkg
    jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'shared_mem',
            'codec_out_mode': 'ros',
            'codec_sub_topic': '/hbmem_img',
            'codec_pub_topic': '/image'
        }.items()
    )

    # nv12图片解码&发布pkg
    nv12_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_decode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_out_mode': 'shared_mem',
            'codec_sub_topic': '/image',
            'codec_pub_topic': '/hbmem_img'
        }.items()
    )

    # web展示pkg
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image',
            'websocket_image_type': 'mjpeg',
            'websocket_smart_topic': LaunchConfiguration("dosod_msg_pub_topic_name")
        }.items()
    )

    # 算法pkg
    dosod_node = Node(
        package='hobot_dosod',
        executable='hobot_dosod',
        output='screen',
        parameters=[
            {"feed_type": 1},
            {"is_shared_mem_sub": 1},
            {"roi": False},
            {"roi_x1": 0.0},
            {"roi_y1": 320.0},
            {"roi_x2": 640.0},
            {"roi_y2": 640.0},
            {"dump_raw_img": LaunchConfiguration(
                "dosod_dump_raw_img")},
            {"dump_render_img": LaunchConfiguration(
                "dosod_dump_render_img")},
            {"msg_pub_topic_name": LaunchConfiguration(
                "dosod_msg_pub_topic_name")},
            {"model_file_name": LaunchConfiguration(
                "dosod_model_file_name")},
            {"vocabulary_file_name": LaunchConfiguration(
                "dosod_vocabulary_file_name")},
            {"trigger_mode": LaunchConfiguration(
                "dosod_trigger_mode")},
            {"class_mode": 0},
            {"score_threshold": LaunchConfiguration(
                "dosod_score_threshold")}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    shared_mem_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('hobot_shm'),
                        'launch/hobot_shm.launch.py'))
            )

    if camera_type_mipi:
        return LaunchDescription([
            camera_device_arg,
            image_width_launch_arg,
            image_height_launch_arg,
            msg_pub_topic_name_launch_arg,
            dump_render_launch_arg,
            dump_raw_launch_arg,
            model_file_name_launch_arg,
            vocabulary_file_name_launch_arg,
            score_threshold_launch_arg,
            trigger_mode_launch_arg,
            # 启动零拷贝环境配置node
            shared_mem_node,
            # 图片发布pkg
            cam_node,
            # 图片编解码&发布pkg
            jpeg_codec_node,
            # 启动dosod pkg
            dosod_node,
            # 启动web展示pkg
            web_node
        ])
    else:
        return LaunchDescription([
            camera_device_arg,
            image_width_launch_arg,
            image_height_launch_arg,
            msg_pub_topic_name_launch_arg,
            dump_render_launch_arg,
            dump_raw_launch_arg,
            model_file_name_launch_arg,
            vocabulary_file_name_launch_arg,
            score_threshold_launch_arg,
            trigger_mode_launch_arg,
            # 启动零拷贝环境配置node
            shared_mem_node,
            # 图片发布pkg
            cam_node,
            # 图片编解码&发布pkg
            nv12_codec_node,
            # 启动dosod pkg
            dosod_node,
            # 启动web展示pkg
            web_node
        ])
