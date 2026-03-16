// Copyright 2024 TIER IV, Inc.
// Licensed under the Apache License, Version 2.0.

#include "autoware_traffic_light_whole_image_detector/lightnet_decoder.hpp"
#include <algorithm>
#include <fstream>
#include <cmath>

namespace autoware::traffic_light::whole_image_detector
{

#define CLAMP(x, lo, hi) std::max((lo), std::min((hi), (x)))

LightnetDecoder::LightnetDecoder(
  int num_classes,
  int num_anchors,
  const std::vector<int> & anchors,
  float score_thresh,
  float nms_thresh)
: num_class_(num_classes),
  num_anchor_(num_anchors),
  anchors_(anchors),
  score_threshold_(score_thresh),
  nms_threshold_(nms_thresh)
{
}

BBox LightnetDecoder::convertBboxRes(
  float bx, float by, float bw, float bh,
  uint32_t stride_h, uint32_t stride_w,
  uint32_t net_w, uint32_t net_h)
{
  BBox b;
  float x = bx * static_cast<float>(stride_w);
  float y = by * static_cast<float>(stride_h);
  b.x1 = x - bw / 2.f;
  b.x2 = x + bw / 2.f;
  b.y1 = y - bh / 2.f;
  b.y2 = y + bh / 2.f;
  b.x1 = CLAMP(b.x1, 0.f, static_cast<float>(net_w));
  b.x2 = CLAMP(b.x2, 0.f, static_cast<float>(net_w));
  b.y1 = CLAMP(b.y1, 0.f, static_cast<float>(net_h));
  b.y2 = CLAMP(b.y2, 0.f, static_cast<float>(net_h));
  return b;
}

void LightnetDecoder::addBboxProposal(
  float bx, float by, float bw, float bh,
  uint32_t stride_h, uint32_t stride_w, int max_index, float max_prob,
  uint32_t image_w, uint32_t image_h, uint32_t input_w, uint32_t input_h,
  std::vector<BBoxInfo> & binfo)
{
  BBoxInfo bbi;
  bbi.box = convertBboxRes(bx, by, bw, bh, stride_h, stride_w, input_w, input_h);
  if (bbi.box.x1 > bbi.box.x2 || bbi.box.y1 > bbi.box.y2) return;
  bbi.box.x1 = (bbi.box.x1 / input_w) * image_w;
  bbi.box.y1 = (bbi.box.y1 / input_h) * image_h;
  bbi.box.x2 = (bbi.box.x2 / input_w) * image_w;
  bbi.box.y2 = (bbi.box.y2 / input_h) * image_h;
  bbi.class_id = max_index;
  bbi.prob = max_prob;
  bbi.is_tlr = false;
  bbi.sub_class_id = 0;
  bbi.cos_angle = bbi.sin_angle = 0.f;
  binfo.push_back(bbi);
}

void LightnetDecoder::addTLRBboxProposal(
  float bx, float by, float bw, float bh,
  uint32_t stride_h, uint32_t stride_w, int max_index, float max_prob, int max_sub_index,
  float cos_a, float sin_a,
  uint32_t image_w, uint32_t image_h, uint32_t input_w, uint32_t input_h,
  std::vector<BBoxInfo> & binfo)
{
  BBoxInfo bbi;
  bbi.box = convertBboxRes(bx, by, bw, bh, stride_h, stride_w, input_w, input_h);
  if (bbi.box.x1 > bbi.box.x2 || bbi.box.y1 > bbi.box.y2) return;
  bbi.box.x1 = (bbi.box.x1 / input_w) * image_w;
  bbi.box.y1 = (bbi.box.y1 / input_h) * image_h;
  bbi.box.x2 = (bbi.box.x2 / input_w) * image_w;
  bbi.box.y2 = (bbi.box.y2 / input_h) * image_h;
  bbi.class_id = max_index;
  bbi.prob = max_prob;
  bbi.is_tlr = true;
  bbi.sub_class_id = max_sub_index;
  bbi.cos_angle = cos_a;
  bbi.sin_angle = sin_a;
  binfo.push_back(bbi);
}

std::vector<BBoxInfo> LightnetDecoder::decodeTensor(
  int image_h, int image_w,
  int input_h, int input_w,
  const int * anchor, int anchor_num,
  const float * output, int grid_w, int grid_h)
{
  const int elem = 5 + num_class_;
  std::vector<BBoxInfo> binfo;
  const float scale_x_y = 2.0f;
  const float offset = 0.5f * (scale_x_y - 1.0f);

  for (int y = 0; y < grid_h; ++y) {
    for (int x = 0; x < grid_w; ++x) {
      for (int b = 0; b < anchor_num; ++b) {
        const int num_grid_cells = grid_h * grid_w;
        const int bbindex = (y * grid_w + x) + num_grid_cells * b * elem;
        const float objectness = output[bbindex + 4 * num_grid_cells];
        if (objectness < score_threshold_) continue;

        float pw = static_cast<float>(anchor[b * 2]);
        float ph = static_cast<float>(anchor[b * 2 + 1]);
        float bx = x + scale_x_y * output[bbindex] - offset;
        float by = y + scale_x_y * output[bbindex + num_grid_cells] - offset;
        float bw = pw * std::pow(output[bbindex + 2 * num_grid_cells] * 2, 2);
        float bh = ph * std::pow(output[bbindex + 3 * num_grid_cells] * 2, 2);

        float max_prob = 0.f;
        int max_index = -1;
        for (int i = 0; i < num_class_; ++i) {
          float p = output[bbindex + (5 + i) * num_grid_cells];
          if (p > max_prob) { max_prob = p; max_index = i; }
        }
        max_prob *= objectness;
        if (max_prob <= score_threshold_) continue;

        uint32_t stride_h = static_cast<uint32_t>(input_h / grid_h);
        uint32_t stride_w = static_cast<uint32_t>(input_w / grid_w);
        addBboxProposal(bx, by, bw, bh, stride_h, stride_w, max_index, max_prob,
          image_w, image_h, input_w, input_h, binfo);
      }
    }
  }
  return binfo;
}

// TLR layout: bbox(4) obj(1) color(3) type(num_class) angle(2)
static const int TLR_OBJ = 4, TLR_COS = 14, TLR_SIN = 15;

std::vector<BBoxInfo> LightnetDecoder::decodeTLRTensor(
  int image_h, int image_w,
  int input_h, int input_w,
  const int * anchor, int anchor_num,
  const float * output, int grid_w, int grid_h)
{
  const int tlr_elem = 5 + 3 + num_class_ + 2;  // 5+3+num_class+2
  std::vector<BBoxInfo> binfo;
  const float scale_x_y = 2.0f;
  const float offset = 0.5f * (scale_x_y - 1.0f);

  for (int y = 0; y < grid_h; ++y) {
    for (int x = 0; x < grid_w; ++x) {
      for (int b = 0; b < anchor_num; ++b) {
        const int num_grid_cells = grid_h * grid_w;
        const int bbindex = (y * grid_w + x) + num_grid_cells * b * tlr_elem;
        const float objectness = output[bbindex + TLR_OBJ * num_grid_cells];
        if (objectness < score_threshold_) continue;

        float pw = static_cast<float>(anchor[b * 2]);
        float ph = static_cast<float>(anchor[b * 2 + 1]);
        float bx = x + scale_x_y * output[bbindex] - offset;
        float by = y + scale_x_y * output[bbindex + num_grid_cells] - offset;
        float bw = pw * std::pow(output[bbindex + 2 * num_grid_cells] * 2, 2);
        float bh = ph * std::pow(output[bbindex + 3 * num_grid_cells] * 2, 2);

        float max_prob = 0.f;
        int max_index = -1;
        for (int i = 0; i < num_class_; ++i) {
          float p = output[bbindex + (5 + 3 + i) * num_grid_cells];
          if (p > max_prob) { max_prob = p; max_index = i; }
        }
        float max_sub_prob = 0.f;
        int max_sub_index = 0;
        for (int i = 0; i < 3; ++i) {
          float p = output[bbindex + (5 + i) * num_grid_cells];
          if (p > max_sub_prob) { max_sub_prob = p; max_sub_index = i; }
        }
        max_prob *= objectness;
        float cos_a = output[bbindex + TLR_COS * num_grid_cells];
        float sin_a = output[bbindex + TLR_SIN * num_grid_cells];
        if (max_prob <= score_threshold_) continue;

        uint32_t stride_h = static_cast<uint32_t>(input_h / grid_h);
        uint32_t stride_w = static_cast<uint32_t>(input_w / grid_w);
        addTLRBboxProposal(bx, by, bw, bh, stride_h, stride_w, max_index, max_prob, max_sub_index,
          cos_a, sin_a, image_w, image_h, input_w, input_h, binfo);
      }
    }
  }
  return binfo;
}

std::vector<BBoxInfo> LightnetDecoder::nonMaximumSuppression(
  float nms_thresh, std::vector<BBoxInfo> binfo)
{
  auto overlap1D = [](float x1min, float x1max, float x2min, float x2max) -> float {
    if (x1min > x2min) { std::swap(x1min, x2min); std::swap(x1max, x2max); }
    return x1max < x2min ? 0.f : std::min(x1max, x2max) - x2min;
  };
  auto computeIoU = [&overlap1D](BBox & b1, BBox & b2) -> float {
    float ox = overlap1D(b1.x1, b1.x2, b2.x1, b2.x2);
    float oy = overlap1D(b1.y1, b1.y2, b2.y1, b2.y2);
    float a1 = (b1.x2 - b1.x1) * (b1.y2 - b1.y1);
    float a2 = (b2.x2 - b2.x1) * (b2.y2 - b2.y1);
    float inter = ox * oy;
    float u = a1 + a2 - inter;
    return u == 0 ? 0.f : inter / u;
  };

  std::stable_sort(binfo.begin(), binfo.end(),
    [](const BBoxInfo & a, const BBoxInfo & b) { return a.prob > b.prob; });
  std::vector<BBoxInfo> out;
  for (auto & i : binfo) {
    bool keep = true;
    for (auto & j : out) {
      if (computeIoU(i.box, j.box) > nms_thresh) { keep = false; break; }
    }
    if (keep) out.push_back(i);
  }
  return out;
}

std::vector<std::string> loadNames(const std::string & path)
{
  std::vector<std::string> names;
  std::ifstream f(path);
  if (!f) return names;
  std::string line;
  while (std::getline(f, line)) {
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) line.pop_back();
    if (!line.empty()) names.push_back(line);
  }
  return names;
}

}  // namespace autoware::traffic_light::whole_image_detector
