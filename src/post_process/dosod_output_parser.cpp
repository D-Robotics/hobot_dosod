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

#include "include/post_process/dosod_output_parser.h"

// 判断检测框的四个顶点是否都在梯形内
bool isBoxInTrapezoid(const std::vector<Point>& trapezoid, const Detection& det) {
    if (det.bbox.xmin < trapezoid[0].x) {
      return false;
    }
    if (det.bbox.xmax > trapezoid[1].x) {
      return false;
    }
    if (det.bbox.ymin < trapezoid[0].y) {
      return false;
    }
    if (det.bbox.ymax > trapezoid[1].y) {
      return false;
    }
    return true;
}

int32_t YoloOutputParser::Parse(
    std::shared_ptr<DnnParserResult> &output,
    std::vector<std::shared_ptr<DNNTensor>> &output_tensors,
    std::vector<std::string>& class_names) {
    
  if (!output) {
    output = std::make_shared<DnnParserResult>();
  }

  int ret = -1;
  if (output_tensors.size() == 2) {
    ret = PostProcessWithoutDecode(output_tensors, class_names, output->perception);
  }

  return ret;
}

int32_t YoloOutputParser::PostProcessWithoutDecode(
    std::vector<std::shared_ptr<DNNTensor>> &tensors,
    std::vector<std::string>& class_names,
    Perception &perception) {
  hbSysFlushMem(&(tensors[0]->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  hbSysFlushMem(&(tensors[1]->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  auto *scores_data = reinterpret_cast<int16_t *>(tensors[0]->sysMem[0].virAddr);
  auto *boxes_data = reinterpret_cast<int16_t *>(tensors[1]->sysMem[0].virAddr);

  perception.type = Perception::DET;
  std::vector<Detection> dets;
  
  int num_pred = 0;
  int num_class = 0;
  int num_class_ailgned = 0;
  if (tensors[0]->properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
    num_pred = tensors[0]->properties.alignedShape.dimensionSize[1];
    num_class_ailgned = tensors[0]->properties.alignedShape.dimensionSize[2];
    num_class = tensors[0]->properties.validShape.dimensionSize[2];
  } else if (tensors[0]->properties.tensorLayout == HB_DNN_LAYOUT_NHWC) {
    num_pred = tensors[0]->properties.alignedShape.dimensionSize[3];
    num_class_ailgned = tensors[0]->properties.alignedShape.dimensionSize[1];
    num_class = tensors[0]->properties.validShape.dimensionSize[1];
  } else {
    num_pred = tensors[0]->properties.alignedShape.dimensionSize[2];
    num_class_ailgned = tensors[0]->properties.alignedShape.dimensionSize[3];
    num_class = tensors[0]->properties.validShape.dimensionSize[3];
  }

  for (int i = 0; i < num_pred; i++) {
    int16_t *score_data = scores_data + i * num_class_ailgned;
    int16_t *box_data = boxes_data + i * 8;
    float max_score = std::numeric_limits<float>::lowest(); // 初始最大值为最小可能值
    int max_index = -1;
    for (int k = 0; k < num_class; ++k) {
      float score = static_cast<float>(score_data[k]) * tensors[0]->properties.scale.scaleData[0];
      if (score > max_score) {
          max_score = score;
          max_index = k;
      }
    }
    if (max_score > score_threshold_) {
      float xmin = static_cast<float>(box_data[0]) * tensors[1]->properties.scale.scaleData[0];
      float ymin = static_cast<float>(box_data[1]) * tensors[1]->properties.scale.scaleData[0];
      float xmax = static_cast<float>(box_data[2]) * tensors[1]->properties.scale.scaleData[0];
      float ymax = static_cast<float>(box_data[3]) * tensors[1]->properties.scale.scaleData[0];
      Bbox bbox(xmin, ymin, xmax, ymax);
      Detection det = Detection(static_cast<int>(max_index),
                      max_score,
                      bbox,
                      class_names[max_index].c_str());
      if (roi_ && !isBoxInTrapezoid(points_, det)) {
        break;
      }
      if (class_mode_ == 1 && class_names[max_index] != "skein") {
        dets.push_back(det);
      } else if (class_mode_ == 0) {
        dets.push_back(det);
      }
    }
  }
  
  nms(dets, iou_threshold_, nms_top_k_, perception.det, true);
  return 0;
}

// 将Detection写入标准VOC格式XML
int YoloOutputParser::WriteVOCXML(const std::string &filename, const std::string &imagePath, int imageWidth, int imageHeight, int depth, const std::vector<Detection> &detections) {
  XMLDocument doc;

  // 根节点annotation
  XMLElement *annotation = doc.NewElement("annotation");
  doc.InsertFirstChild(annotation);

  // folder
  XMLElement *folder = doc.NewElement("folder");
  folder->SetText("VOCImages");
  annotation->InsertEndChild(folder);

  // filename
  XMLElement *filenameElem = doc.NewElement("filename");
  filenameElem->SetText(imagePath.c_str());
  annotation->InsertEndChild(filenameElem);

  // size节点
  XMLElement *sizeElem = doc.NewElement("size");
  XMLElement *widthElem = doc.NewElement("width");
  widthElem->SetText(imageWidth);
  XMLElement *heightElem = doc.NewElement("height");
  heightElem->SetText(imageHeight);
  XMLElement *depthElem = doc.NewElement("depth");
  depthElem->SetText(depth);
  sizeElem->InsertEndChild(widthElem);
  sizeElem->InsertEndChild(heightElem);
  sizeElem->InsertEndChild(depthElem);
  annotation->InsertEndChild(sizeElem);

  // 添加object节点
  for (const auto &det : detections) {
      XMLElement *objectElem = doc.NewElement("object");

      XMLElement *nameElem = doc.NewElement("name");
      nameElem->SetText(det.class_name);
      objectElem->InsertEndChild(nameElem);

      XMLElement *poseElem = doc.NewElement("pose");
      poseElem->SetText("Unspecified");
      objectElem->InsertEndChild(poseElem);

      XMLElement *truncatedElem = doc.NewElement("truncated");
      truncatedElem->SetText(0);
      objectElem->InsertEndChild(truncatedElem);

      XMLElement *difficultElem = doc.NewElement("difficult");
      difficultElem->SetText(0);
      objectElem->InsertEndChild(difficultElem);

      // bndbox节点
      XMLElement *bndboxElem = doc.NewElement("bndbox");

      XMLElement *xminElem = doc.NewElement("xmin");
      xminElem->SetText(det.bbox.xmin);
      XMLElement *yminElem = doc.NewElement("ymin");
      yminElem->SetText(det.bbox.ymin);
      XMLElement *xmaxElem = doc.NewElement("xmax");
      xmaxElem->SetText(det.bbox.xmax);
      XMLElement *ymaxElem = doc.NewElement("ymax");
      ymaxElem->SetText(det.bbox.ymax);

      // 添加置信度score节点
      XMLElement *scoreElem = doc.NewElement("score");
      scoreElem->SetText(det.score);  // 假设confidence是float类型
      objectElem->InsertEndChild(scoreElem);

      bndboxElem->InsertEndChild(xminElem);
      bndboxElem->InsertEndChild(yminElem);
      bndboxElem->InsertEndChild(xmaxElem);
      bndboxElem->InsertEndChild(ymaxElem);
      objectElem->InsertEndChild(bndboxElem);

      annotation->InsertEndChild(objectElem);
  }

  // 保存文件
  XMLError eResult = doc.SaveFile(filename.c_str());
  if (eResult != XML_SUCCESS) {
      return -1;
  }
  return 0;
}