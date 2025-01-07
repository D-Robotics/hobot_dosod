// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DOSOD_OUTPUT_PARSER_H_
#define DOSOD_OUTPUT_PARSER_H_

#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "dnn_node/dnn_node_data.h"
#include "dnn_node/util/output_parser/perception_common.h"
#include "dnn_node/util/output_parser/detection/nms.h"
#include "tinyxml2.h"

using namespace tinyxml2;

using hobot::dnn_node::output_parser::Bbox;
using hobot::dnn_node::output_parser::Detection;
using hobot::dnn_node::output_parser::DnnParserResult;
using hobot::dnn_node::output_parser::Perception;
using hobot::dnn_node::DNNTensor;

struct Point {
    float x, y;
};

class YoloOutputParser {
 public:
  YoloOutputParser(int num_class, bool roi) {
                num_class_ = num_class;roi_ = roi;}
  ~YoloOutputParser() {}

  int32_t Parse(
      std::shared_ptr<DnnParserResult> &output,
      std::vector<std::shared_ptr<DNNTensor>> &output_tensors,
      std::vector<std::string>& class_names);
 
  int32_t PostProcessWithoutDecode(
      std::vector<std::shared_ptr<DNNTensor>> &tensors,
      std::vector<std::string>& class_names,
      Perception &perception);

  int32_t SetScoreThreshold(float score_threshold) {score_threshold_ = score_threshold; return 0;}
  int32_t SetIouThreshold(float iou_threshold) {iou_threshold_ = iou_threshold; return 0;}
  int32_t SetTopkThreshold(int nms_top_k) {nms_top_k_ = nms_top_k; return 0;}
  int32_t SetClassMode(int class_mode) {class_mode_ = class_mode; return 0;}
  int32_t SetPoint(float x, float y) {
        Point point;
        point.x = x;
        point.y = y;
        points_.push_back(point); return 0;}

  int32_t WriteVOCXML(const std::string &filename, const std::string &imagePath, int imageWidth, int imageHeight, int depth, const std::vector<Detection> &detections);

 private:

  float score_threshold_ = 0.3;
  float iou_threshold_ = 0.5;
  int nms_top_k_ = 100;

  int class_mode_ = 0;
  int num_class_ = 0;
  std::vector<Point> points_;
  bool roi_ = false;
};

#endif  // DOSOD_OUTPUT_PARSER_H_
