English| [简体中文](./README_cn.md)

Getting Started with Decoupled Open-Set Object Detector (DOSOD) Example
=======

# Feature Introduction

The DOSOD package is an example of quantized deployment based on [Decoupled Open-Set Object Detector](https://github.com/D-Robotics-AI-Lab/DOSOD). The image data comes from local image feedback and subscribed image messages. Additionally, DOSOD supports custom detection categories detect, which is the biggest difference with conventional detector. Ultimately, intelligent results are published in the post-processing of DOSOD and can be viewed through a web interface.

# Development Environment

- Programming Language: C/C++
- Development Platform: X5
- System Version: Ubuntu 22.04
- Compilation Toolchain: Linaro GCC 11.4.0

# Compilation

- X5 Version: Supports compilation on the X5 Ubuntu system and cross-compilation using Docker on a PC.

It also supports controlling the dependencies and functionality of the compiled pkg through compilation options.

## Dependency Libraries

- OpenCV: 3.4.5

ROS Packages:

- dnn node
- cv_bridge
- sensor_msgs
- hbm_img_msgs
- ai_msgs

hbm_img_msgs is a custom image message format used for image transmission in shared memory scenarios. The hbm_img_msgs pkg is defined in hobot_msgs; therefore, if shared memory is used for image transmission, this pkg is required.

## Compilation Options

1. SHARED_MEM

- Shared memory transmission switch, enabled by default (ON), can be turned off during compilation using the -DSHARED_MEM=OFF command.
- When enabled, compilation and execution depend on the hbm_img_msgs pkg and require the use of tros for compilation.
- When disabled, compilation and execution do not depend on the hbm_img_msgs pkg, supporting compilation using native ROS and tros.
- For shared memory communication, only subscription to nv12 format images is currently supported.## Compile on X5 Ubuntu System

1. Compilation Environment Verification

- The X5 Ubuntu system is installed on the board.
- The current compilation terminal has set up the TogetherROS environment variable: `source PATH/setup.bash`. Where PATH is the installation path of TogetherROS.
- The ROS2 compilation tool colcon is installed. If the installed ROS does not include the compilation tool colcon, it needs to be installed manually. Installation command for colcon: `pip install -U colcon-common-extensions`.
- The dnn node package has been compiled.

2. Compilation

- Compilation command: `colcon build --packages-select hobot_dosod`

## Docker Cross-Compilation for X5 Version

1. Compilation Environment Verification

- Compilation within docker, and TogetherROS has been installed in the docker environment. For instructions on docker installation, cross-compilation, TogetherROS compilation, and deployment, please refer to the README.md in the robot development platform's robot_dev_config repo.
- The dnn node package has been compiled.
- The hbm_img_msgs package has been compiled (see Dependency section for compilation methods).

2. Compilation

- Compilation command:

  ```shell
  # RDK X5
  bash robot_dev_config/build.sh -p X5 -s hobot_dosod
  ```

- Shared memory communication method is enabled by default in the compilation options.

# Instructions

## Dependencies

- mipi_cam package: Publishes image messages
- usb_cam package: Publishes image messages
- websocket package: Renders images and AI perception messages

## Parameters

| Parameter Name      | Explanation                            | Mandatory            | Default Value       | Remarks                                                                 |
| ------------------- | -------------------------------------- | -------------------- | ------------------- | ----------------------------------------------------------------------- |
| feed_type           | Image source, 0: local; 1: subscribe   | No                   | 0                   |                                                                         |
| image               | Local image path                       | No                   | config/000000160864.jpg     |                                                                         |
| is_shared_mem_sub   | Subscribe to images using shared memory communication method | No  | 0                   |  |   |
| score_threshold | boxes confidence threshold | No | 0.2 | |
| iou_threshold | nms iou threshold | No | 0.5 | |
| nms_top_k | Detect the first k boxes | No | 50 | |
| dump_render_img     | Whether to render, 0: no; 1: yes       | No                   | 0                   |   |
| dump_raw_img    | Whether to dump raw img, 0: no; 1: yes            | 否                   | 0                   |   |
| dump_ai_result    | Whether to dump ai result, 0: no; 1: yes            | 否                   | 0                   |   |
| dump_render_path    | Render image dump path            | 否                   | .                   |   |
| dump_raw_path    | Raw img dump path            | 否                   | .                   |   |
| dump_ai_result    | Ai result dump path            | 否                   | .                   |   |
| ai_msg_pub_topic_name | Topic name for publishing intelligent results for web display | No                   | /hobot_dosod | |
| ros_img_sub_topic_name | Topic name for subscribing image msg | No                   | /image | |

## Running

## Running on X5 Ubuntu System

Running method 1, use the executable file to start:
```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.bash
# The config includes models used by the example and local images for filling
# Copy based on the actual installation path (the installation path in the docker is install/lib/hobot_dosod/config/, the copy command is cp -r install/lib/hobot_dosod/config/ .).
cp -r install/hobot_dosod/lib/hobot_dosod/config/ .

# Run mode 1:Use local JPG format images for backflow prediction:

ros2 run hobot_dosod hobot_dosod --ros-args -p feed_type:=0 -p image:=config/000000160864.jpg -p image_type:=0 -p dump_render_img:=1

# Run mode 2:Use the subscribed image msg (topic name: /image) for prediction, set the log level to warn.

ros2 run hobot_dosod hobot_dosod --ros-args -p feed_type:=1 --ros-args --log-level warn

# Run mode 3: Use shared memory communication method (topic name: /hbmem_img) to perform inference in asynchronous mode and set the log level to warn:

ros2 run hobot_dosod hobot_dosod --ros-args -p feed_type:=1 -p is_shared_mem_sub:=1 --ros-args --log-level warn
```

To run in mode 2 using a launch file:

```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# Copy the configuration based on the actual installation path
cp -r install/lib/hobot_dosod/config/ .

# Configure MIPI camera
export CAM_TYPE=mipi

# Start the launch file, publish nv12 format images using shared memory with F37 sensor
# By default, it runs the fcos algorithm, switch algorithms using the config_file parameter in the launch command, e.g.
ros2 launch hobot_dosod dosod.launch.py
```

## Run on X5 Buildroot system:

```shell
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# Copy the configuration used by the example and the local image used for inference
cp -r install/lib/hobot_dosod/config/ .

# Run mode 1:Use local JPG format images for backflow prediction:
./install/lib/hobot_dosod/hobot_dosod --ros-args -p feed_type:=0 -p image:=config/000000160864.jpg -p image_type:=0 -p dump_render_img:=1

# Run mode 2:Use the subscribed image msg (topic name: /image) for prediction, set the log level to warn.
./install/lib/hobot_dosod/hobot_dosod --ros-args -p feed_type:=1 --ros-args --log-level warn

# Run mode 3: Use shared memory communication method (topic name: /hbmem_img) to perform inference in asynchronous mode and set the log level to warn:
./install/lib/hobot_dosod/hobot_dosod --ros-args -p feed_type:=1 -p is_shared_mem_sub:=1 --ros-args --log-level warn
```

# Results Analysis

## X5 Results Display

log:

Command executed: `ros2 run hobot_dosod hobot_dosod --ros-args -p feed_type:=0 -p image:=config/000000160864.jpg -p image_type:=0`

```shell
[WARN] [1736235231.771601056] [hobot_dosod]: This is hobot dosod!
[WARN] [1736235231.847169623] [hobot_dosod]: Parameter:
 model_file_name: config/3x-l_epoch_100_rep-coco80-without-nms-int8.bin
 vocabulary_file_name: config/offline_vocabulary.json
 feed_type(0:local, 1:sub): 0
 image: config/000000160864.jpg
 dump_ai_result: 0
 dump_raw_img: 0
 dump_render_img: 1
 dump_ai_path: .
 dump_raw_path: .
 dump_render_path: .
 is_shared_mem_sub: 0
 score_threshold: 0.2
 iou_threshold: 0.5
 nms_top_k: 50
 is_homography: 0
 trigger_mode: 0
 class_mode: 0
 task_num: 2
 roi: 0
 y_offset: 950
 ai_msg_pub_topic_name: /hobot_dosod
 ros_img_sub_topic_name: /image
[INFO] [1736235231.847331832] [dnn]: Node init.
[INFO] [1736235231.847367582] [hobot_dosod]: Set node para.
[WARN] [1736235231.847396749] [hobot_dosod]: model_file_name_: config/3x-l_epoch_100_rep-coco80-without-nms-int8.bin, task_num: 2
[INFO] [1736235231.847449540] [dnn]: Model init.
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.54.0
[DNN] Runtime version = 1.23.10_(3.15.54 HBRT)
[A][DNN][packed_model.cpp:247][Model](2025-01-07,15:33:52.642.13) [HorizonRT] The model builder version = 1.24.3
[W][DNN]bpu_model_info.cpp:491][Version](2025-01-07,15:33:53.18.898) Model: 3x-l_epoch_100_rep-coco80-without-nms. Inconsistency between the hbrt library version 3.15.54.0 and the model build version 3.15.55.0 detected, in order to ensure correct model results, it is recommended to use compilation tools and the BPU SDK from the same OpenExplorer package.
[INFO] [1736235233.030728321] [dnn]: The model input 0 width is 640 and height is 640
[INFO] [1736235233.031005072] [dnn]:
Model Info:
name: 3x-l_epoch_100_rep-coco80-without-nms.
[input]
 - (0) Layout: NCHW, Shape: [1, 3, 640, 640], Type: HB_DNN_IMG_TYPE_NV12.
[output]
 - (0) Layout: NONE, Shape: [1, 1, 8400, 80], Type: HB_DNN_TENSOR_TYPE_S16.
 - (1) Layout: NONE, Shape: [1, 1, 8400, 4], Type: HB_DNN_TENSOR_TYPE_S16.

[INFO] [1736235233.031092322] [dnn]: Task init.
[INFO] [1736235233.033161116] [dnn]: Set task_num [2]
[WARN] [1736235233.033216533] [hobot_dosod]: Get model name: 3x-l_epoch_100_rep-coco80-without-nms from load model.
[INFO] [1736235233.033263366] [hobot_dosod]: The model input width is 640 and height is 640
[WARN] [1736235233.033804451] [hobot_dosod]: Create ai msg publisher with topic_name: /hobot_dosod
[INFO] [1736235233.065223996] [hobot_dosod]: Dnn node feed with local image: config/000000160864.jpg
[INFO] [1736235233.197520685] [hobot_dosod]: Output from frame_id: feedback, stamp: 0.0
[INFO] [1736235233.211616539] [hobot_dosod]: out box size: 12
[INFO] [1736235233.211774456] [hobot_dosod]: det rect: 290.195 59.2845 521.799 357.288, det type: person, score:0.852277
[INFO] [1736235233.211849289] [hobot_dosod]: det rect: 133.109 86.0883 253.594 295.536, det type: person, score:0.841566
[INFO] [1736235233.211914873] [hobot_dosod]: det rect: 440.071 64.9375 581.802 295.919, det type: person, score:0.840223
[INFO] [1736235233.211979748] [hobot_dosod]: det rect: 6.1081 168.679 188.728 372.618, det type: person, score:0.752026
[INFO] [1736235233.212042956] [hobot_dosod]: det rect: 545.25 47.0923 608.295 259.63, det type: person, score:0.660044
[INFO] [1736235233.212107831] [hobot_dosod]: det rect: 180.896 0.0239533 270.768 85.8248, det type: person, score:0.585611
[INFO] [1736235233.212170790] [hobot_dosod]: det rect: 151.529 228.275 194.333 261.067, det type: baseball glove, score:0.506172
[INFO] [1736235233.212235706] [hobot_dosod]: det rect: 293.165 24.1689 370.079 137.492, det type: baseball bat, score:0.447669
[INFO] [1736235233.212297373] [hobot_dosod]: det rect: 444.718 20.4561 486.804 113.706, det type: baseball bat, score:0.299962
[INFO] [1736235233.212363415] [hobot_dosod]: det rect: 287.105 171.218 361.264 273.835, det type: chair, score:0.290959
[INFO] [1736235233.212429457] [hobot_dosod]: det rect: 252.756 269.978 299.8 281.715, det type: baseball bat, score:0.20618
[INFO] [1736235233.212494665] [hobot_dosod]: det rect: 133.085 42.7807 212.73 108.82, det type: baseball bat, score:0.201419
[INFO] [1736235233.214799377] [ImageUtils]: target size: 12
[INFO] [1736235233.214921710] [ImageUtils]: target type: person, rois.size: 1
[INFO] [1736235233.214965502] [ImageUtils]: roi.type: person, x_offset: 290 y_offset: 59 width: 231 height: 298
[INFO] [1736235233.215333669] [ImageUtils]: target type: person, rois.size: 1
[INFO] [1736235233.215380711] [ImageUtils]: roi.type: person, x_offset: 133 y_offset: 86 width: 120 height: 209
[INFO] [1736235233.215592920] [ImageUtils]: target type: person, rois.size: 1
[INFO] [1736235233.215633795] [ImageUtils]: roi.type: person, x_offset: 440 y_offset: 64 width: 141 height: 230
[INFO] [1736235233.215974003] [ImageUtils]: target type: person, rois.size: 1
[INFO] [1736235233.216016545] [ImageUtils]: roi.type: person, x_offset: 6 y_offset: 168 width: 182 height: 203
[INFO] [1736235233.216221962] [ImageUtils]: target type: person, rois.size: 1
[INFO] [1736235233.216261754] [ImageUtils]: roi.type: person, x_offset: 545 y_offset: 47 width: 63 height: 212
[INFO] [1736235233.216453212] [ImageUtils]: target type: person, rois.size: 1
[INFO] [1736235233.216493337] [ImageUtils]: roi.type: person, x_offset: 180 y_offset: 0 width: 89 height: 85
[INFO] [1736235233.216631421] [ImageUtils]: target type: baseball glove, rois.size: 1
[INFO] [1736235233.216670296] [ImageUtils]: roi.type: baseball glove, x_offset: 151 y_offset: 228 width: 42 height: 32
[INFO] [1736235233.216825630] [ImageUtils]: target type: baseball bat, rois.size: 1
[INFO] [1736235233.216864255] [ImageUtils]: roi.type: baseball bat, x_offset: 293 y_offset: 24 width: 76 height: 113
[INFO] [1736235233.217082922] [ImageUtils]: target type: baseball bat, rois.size: 1
[INFO] [1736235233.217123422] [ImageUtils]: roi.type: baseball bat, x_offset: 444 y_offset: 20 width: 42 height: 93
[INFO] [1736235233.217304755] [ImageUtils]: target type: chair, rois.size: 1
[INFO] [1736235233.217342880] [ImageUtils]: roi.type: chair, x_offset: 287 y_offset: 171 width: 74 height: 102
[INFO] [1736235233.217503672] [ImageUtils]: target type: baseball bat, rois.size: 1
[INFO] [1736235233.217542381] [ImageUtils]: roi.type: baseball bat, x_offset: 252 y_offset: 269 width: 47 height: 11
[INFO] [1736235233.217686839] [ImageUtils]: target type: baseball bat, rois.size: 1
[INFO] [1736235233.217725173] [ImageUtils]: roi.type: baseball bat, x_offset: 133 y_offset: 42 width: 79 height: 66
[WARN] [1736235233.217889839] [ImageUtils]: Draw result to file: 0_0_render.jpg
```

## Render img:
![image](img/render_dosod.jpg)

Note: Preprocessing Image involves scaling and padding.