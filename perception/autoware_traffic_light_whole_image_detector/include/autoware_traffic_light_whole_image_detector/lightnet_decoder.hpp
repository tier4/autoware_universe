// Copyright 2024 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.

#ifndef AUTOWARE_TRAFFIC_LIGHT_WHOLE_IMAGE_DETECTOR__LIGHTNET_DECODER_HPP_
#define AUTOWARE_TRAFFIC_LIGHT_WHOLE_IMAGE_DETECTOR__LIGHTNET_DECODER_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

namespace autoware::traffic_light::whole_image_detector
{

struct BBox
{
  float x1, y1, x2, y2;
};

struct BBoxInfo
{
  BBox box;
  int class_id;
  float prob;
  bool is_tlr;
  int sub_class_id;  // TLR color: 0=green, 1=yellow, 2=red
  float cos_angle, sin_angle;
};

/**
 * Decode detection outputs (Scaled-YOLOv4 style) and apply NMS.
 * Compatible with trt-lightnet / CoMLOps detection model.
 */
class LightnetDecoder
{
public:
  LightnetDecoder(
    int num_classes,
    int num_anchors,
    const std::vector<int> & anchors,
    float score_thresh,
    float nms_thresh);

  // Decode standard detection head: chan = (4+1+num_class)*num_anchor
  std::vector<BBoxInfo> decodeTensor(
    int image_h, int image_w,
    int input_h, int input_w,
    const int * anchor, int anchor_num,
    const float * output, int grid_w, int grid_h);

  // Decode TLR head: chan = (4+1+num_class+3+2)*num_anchor
  std::vector<BBoxInfo> decodeTLRTensor(
    int image_h, int image_w,
    int input_h, int input_w,
    const int * anchor, int anchor_num,
    const float * output, int grid_w, int grid_h);

  std::vector<BBoxInfo> nonMaximumSuppression(float nms_thresh, std::vector<BBoxInfo> binfo);

private:
  int num_class_;
  int num_anchor_;
  std::vector<int> anchors_;
  float score_threshold_;
  float nms_threshold_;

  void addBboxProposal(float bx, float by, float bw, float bh,
    uint32_t stride_h, uint32_t stride_w, int max_index, float max_prob,
    uint32_t image_w, uint32_t image_h, uint32_t input_w, uint32_t input_h,
    std::vector<BBoxInfo> & binfo);
  void addTLRBboxProposal(float bx, float by, float bw, float bh,
    uint32_t stride_h, uint32_t stride_w, int max_index, float max_prob, int max_sub_index,
    float cos_a, float sin_a,
    uint32_t image_w, uint32_t image_h, uint32_t input_w, uint32_t input_h,
    std::vector<BBoxInfo> & binfo);
  BBox convertBboxRes(float bx, float by, float bw, float bh,
    uint32_t stride_h, uint32_t stride_w, uint32_t net_w, uint32_t net_h);
};

/** Load class names from file (one per line). */
std::vector<std::string> loadNames(const std::string & path);

/** Resolve path: if not absolute, relative to package share dir or cwd. */
std::string resolvePath(const std::string & path, const std::string & package_share_dir);

}  // namespace autoware::traffic_light::whole_image_detector

#endif  // AUTOWARE_TRAFFIC_LIGHT_WHOLE_IMAGE_DETECTOR__LIGHTNET_DECODER_HPP_
