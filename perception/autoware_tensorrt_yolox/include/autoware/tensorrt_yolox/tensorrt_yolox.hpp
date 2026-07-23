// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__TENSORRT_YOLOX__TENSORRT_YOLOX_HPP_
#define AUTOWARE__TENSORRT_YOLOX__TENSORRT_YOLOX_HPP_

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/cuda_utils/stream_unique_ptr.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <autoware/tensorrt_common/tensorrt_conv_calib.hpp>
#include <autoware/tensorrt_common/utils.hpp>
#include <autoware/tensorrt_yolox/preprocess.hpp>
#include <opencv2/opencv.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::tensorrt_yolox
{
using autoware::cuda_utils::CudaUniquePtr;
using autoware::cuda_utils::CudaUniquePtrHost;
using autoware::cuda_utils::makeCudaStream;
using autoware::cuda_utils::StreamUniquePtr;

struct Object
{
  int32_t x_offset;
  int32_t y_offset;
  int32_t height;
  int32_t width;
  float score;
  int32_t type;
};

using ObjectArray = std::vector<Object>;
using ObjectArrays = std::vector<ObjectArray>;
using autoware::tensorrt_common::CalibrationConfig;
using autoware::tensorrt_common::NetworkIOPtr;
using autoware::tensorrt_common::ProfileDimsPtr;
using autoware::tensorrt_common::Profiler;
using autoware::tensorrt_common::TrtCommon;
using autoware::tensorrt_common::TrtCommonConfig;
using autoware::tensorrt_common::TrtConvCalib;

struct GridAndStride
{
  int grid0;
  int grid1;
  int stride;
};

/**
 * @class TrtYoloX
 * @brief TensorRT YOLOX for faster inference
 * @warning  Regarding quantization, we recommend use MinMax calibration due to accuracy drop with
 * Entropy calibration.
 */
class TrtYoloX
{
public:
  /**
   * @brief Construct TrtYoloX.
   * @param[in] trt_config base trt common configuration
   * @param[in] num_class classifier-ed num
   * @param[in] score_threshold threshold for detection
   * @param[in] nms_threshold threshold for NMS
   * @param[in] use_gpu_preprocess whether use cuda gpu for preprocessing
   * @param[in] gpu_id GPU id for inference
   * @param[in] calibration_image_list_path path for calibration files (only require for
   * quantization)
   * @param[in] norm_factor scaling factor for preprocess
   * @param[in] cache_dir unused variable
   * @param[in] calib_config calibration configuration
   * @param[in] enable_center_crop_inference if true, additionally run inference on a
   * native-resolution center crop (of the model input size) and use its trusted detections to
   * override the results from the resized (downscaled) inference
   * @param[in] center_crop_edge_margin margin (px) within which a center-crop detection is treated
   * as touching the crop border and therefore discarded (may be truncated)
   * @param[in] center_crop_overlap_iou_threshold IoU above which a resized-inference detection is
   * overridden by an overlapping center-crop detection
   * @param[in] center_crop_x_ratio horizontal position of the crop center as a fraction of the
   * image width (0.5 = image center); the crop is clamped to stay inside the image
   * @param[in] center_crop_y_ratio vertical position of the crop center as a fraction of the image
   * height (0.5 = image center; use a smaller value to shift the crop up, e.g. for traffic lights)
   */
  TrtYoloX(
    TrtCommonConfig & trt_config, const int num_class = 8, const float score_threshold = 0.3,
    const float nms_threshold = 0.7, const bool use_gpu_preprocess = false,
    const uint8_t gpu_id = 0, std::string calibration_image_list_path = std::string(),
    const double norm_factor = 1.0, [[maybe_unused]] const std::string & cache_dir = "",
    const CalibrationConfig & calib_config = CalibrationConfig(),
    const bool enable_center_crop_inference = false, const int center_crop_edge_margin = 8,
    const float center_crop_overlap_iou_threshold = 0.5f, const float center_crop_x_ratio = 0.5f,
    const float center_crop_y_ratio = 0.5f);
  /**
   * @brief Deconstruct TrtYoloX
   */
  ~TrtYoloX();

  /**
   * @brief select cuda device for a inference
   * @param[in] GPU id
   */
  bool setCudaDeviceId(const uint8_t gpu_id);

  /**
   * @brief return a flag for gpu initialization
   */
  bool isGPUInitialized() const { return is_gpu_initialized_; }

  /**
   * @brief run inference including pre-process and post-process
   * @param[out] objects results for object detection
   * @param[in] images batched images
   */
  bool doInference(
    const std::vector<cv::Mat> & images, ObjectArrays & objects, std::vector<cv::Mat> & masks,
    std::vector<cv::Mat> & color_masks);

  /**
   * @brief run inference including pre-process and post-process
   * @param[out] objects results for object detection
   * @param[in] images batched images
   * @param[in] rois region of interest for inference
   */
  bool doInferenceWithRoi(
    const std::vector<cv::Mat> & images, ObjectArrays & objects, const std::vector<cv::Rect> & roi);

  /**
   * @brief run multi-scale inference including pre-process and post-process
   * @param[out] objects results for object detection
   * @param[in] image
   * @param[in] rois region of interest for inference
   */
  bool doMultiScaleInference(
    const cv::Mat & image, ObjectArrays & objects, const std::vector<cv::Rect> & roi);

  /**
   * @brief allocate buffer for preprocess on GPU
   * @param[in] width original image width
   * @param[in] height original image height
   * @warning if we don't allocate buffers using it, "preprocessGpu" allocates buffers at the
   * beginning
   */
  void initPreprocessBuffer(int width, int height);

  /**
   * @brief output TensorRT profiles for each layer
   */
  void printProfiling(void);

  /**
   * @brief get num for multitask heads
   */
  int getMultitaskNum(void);

  /**
   * @brief get batch size
   * @return batch size
   */
  [[nodiscard]] int getBatchSize() const;

private:
  /**
   * @brief run preprocess including resizing, letterbox, NHWC2NCHW and toFloat on CPU
   * @param[in] images batching images
   */
  void preprocess(const std::vector<cv::Mat> & images);

  /**
   * @brief run preprocess on GPU
   * @param[in] images batching images
   */
  void preprocessGpu(const std::vector<cv::Mat> & images);

  /**
   * @brief run preprocess including resizing, letterbox, NHWC2NCHW and toFloat on CPU
   * @param[in] images batching images
   * @param[in] rois region of interest
   */
  void preprocessWithRoi(const std::vector<cv::Mat> & images, const std::vector<cv::Rect> & rois);

  /**
   * @brief run preprocess on GPU
   * @param[in] images batching images
   * @param[in] rois region of interest
   */
  void preprocessWithRoiGpu(
    const std::vector<cv::Mat> & images, const std::vector<cv::Rect> & rois);

  /**
   * @brief run multi-scale preprocess including resizing, letterbox, NHWC2NCHW and toFloat on CPU
   * @param[in] images batching images
   * @param[in] rois region of interest
   */
  void multiScalePreprocess(const cv::Mat & image, const std::vector<cv::Rect> & rois);

  /**
   * @brief run multi-scale preprocess including resizing, letterbox, NHWC2NCHW and toFloat on GPU
   * @param[in] images batching images
   * @param[in] rois region of interest
   */
  void multiScalePreprocessGpu(const cv::Mat & image, const std::vector<cv::Rect> & rois);

  bool multiScaleFeedforward(const cv::Mat & image, int batch_size, ObjectArrays & objects);
  bool multiScaleFeedforwardAndDecode(
    const cv::Mat & images, int batch_size, ObjectArrays & objects);

  bool feedforward(const std::vector<cv::Mat> & images, ObjectArrays & objects);
  bool feedforwardAndDecode(
    const std::vector<cv::Mat> & images, ObjectArrays & objects, std::vector<cv::Mat> & masks,
    std::vector<cv::Mat> & color_masks);

  /**
   * @brief run a single batch-2 inference that processes the whole (resized) image together with a
   * native-resolution center crop of the model input size, then merge the trusted (non
   * border-touching) crop detections into the whole-image detections. The two views occupy the two
   * batch slots of a batch-2 engine (batch element 0: whole image, element 1: center crop).
   * @param[in] images batched images (expects a single image)
   * @param[out] objects merged detection results
   * @param[out] masks segmentation masks (only whole-image mask is propagated)
   * @param[out] color_masks color segmentation masks
   * @return true on success
   */
  bool inferenceWithCenterCropBatch(
    const std::vector<cv::Mat> & images, ObjectArrays & objects, std::vector<cv::Mat> & masks,
    std::vector<cv::Mat> & color_masks);

  /**
   * @brief whether a native-resolution center-crop pass can be run for the given images. Requires
   * the option enabled, a batch-2 detection-only (non-multitask) model, a single input image, and
   * an image at least as large as the model input in both dimensions.
   */
  bool canRunCenterCrop(const std::vector<cv::Mat> & images) const;

  /**
   * @brief merge trusted center-crop detections into resized-inference detections. Center-crop
   * detections touching the crop border (within center_crop_edge_margin_) are discarded since they
   * may be truncated; the remaining ones override overlapping resized-inference detections.
   * @param[in,out] base_objects detections from resized inference in original image coordinates
   * @param[in] crop_objects detections from the center crop in crop-local coordinates
   * @param[in] crop_origin top-left corner of the crop in original image coordinates
   * @param[in] crop_width width of the center crop
   * @param[in] crop_height height of the center crop
   */
  void mergeCropObjects(
    ObjectArray & base_objects, const ObjectArray & crop_objects, const cv::Point & crop_origin,
    int crop_width, int crop_height) const;
  void decodeOutputs(float * prob, ObjectArray & objects, float scale, cv::Size & img_size) const;
  void generateGridsAndStride(
    const int target_w, const int target_h, const std::vector<int> & strides,
    std::vector<GridAndStride> & grid_strides) const;
  void generateYoloxProposals(
    std::vector<GridAndStride> grid_strides, float * feat_blob, float prob_threshold,
    ObjectArray & objects) const;
  void qsortDescentInplace(ObjectArray & face_objects, int left, int right) const;
  inline void qsortDescentInplace(ObjectArray & objects) const
  {
    if (objects.empty()) {
      return;
    }
    qsortDescentInplace(objects, 0, objects.size() - 1);
  }

  inline float intersectionArea(const Object & a, const Object & b) const
  {
    cv::Rect a_rect(a.x_offset, a.y_offset, a.width, a.height);
    cv::Rect b_rect(b.x_offset, b.y_offset, b.width, b.height);
    cv::Rect_<float> inter = a_rect & b_rect;
    return inter.area();
  }

  // cspell: ignore Bboxes
  void nmsSortedBboxes(
    const ObjectArray & face_objects, std::vector<int> & picked, float nms_threshold) const;

  /**
   * @brief get a mask image for a segmentation head
   * @param[out] argmax argmax results
   * @param[in] prob probability map
   * @param[in] dims dimension for probability map
   * @param[in] out_w mask width excluding letterbox
   * @param[in] out_h mask height excluding letterbox
   */
  cv::Mat getMaskImage(float * prob, nvinfer1::Dims dims, int out_w, int out_h);

  /**
   * @brief get a mask image on GPUs for a segmentation head
   * @param[out] mask image
   * @param[in] prob probability map on device
   * @param[in] out_w mask width excluding letterbox
   * @param[in] out_h mask height excluding letterbox
   * @param[in] b current batch
   */
  cv::Mat getMaskImageGpu(float * d_prob, nvinfer1::Dims dims, int out_w, int out_h, int b);

  std::unique_ptr<TrtConvCalib> trt_common_;

  std::vector<float> input_h_;
  CudaUniquePtr<float[]> input_d_;
  CudaUniquePtr<int32_t[]> out_num_detections_d_;
  CudaUniquePtr<float[]> out_boxes_d_;
  CudaUniquePtr<float[]> out_scores_d_;
  CudaUniquePtr<int32_t[]> out_classes_d_;

  bool needs_output_decode_;
  size_t out_elem_num_;
  size_t out_elem_num_per_batch_;
  CudaUniquePtr<float[]> out_prob_d_;

  StreamUniquePtr stream_;

  int32_t max_detections_;
  std::vector<float> scales_;

  int num_class_;
  float score_threshold_;
  float nms_threshold_;
  int32_t batch_size_;
  CudaUniquePtrHost<float[]> out_prob_h_;

  // flag whether preprocess are performed on GPU
  bool use_gpu_preprocess_;
  // GPU id for inference
  const uint8_t gpu_id_;
  // flag for gpu initialization
  bool is_gpu_initialized_;
  // host buffer for preprocessing on GPU
  CudaUniquePtrHost<unsigned char[]> image_buf_h_;
  // device buffer for preprocessing on GPU
  CudaUniquePtr<unsigned char[]> image_buf_d_;
  // normalization factor used for preprocessing
  double norm_factor_;

  // flag whether to additionally run inference on a native-resolution center crop
  bool enable_center_crop_inference_;
  // margin (px) within which a center-crop detection is treated as touching the crop border
  int center_crop_edge_margin_;
  // IoU above which a resized-inference detection is overridden by a center-crop detection
  float center_crop_overlap_iou_threshold_;
  // crop center position as a fraction of the image size (0.5 = image center). A smaller y ratio
  // shifts the crop toward the top of the image (e.g. to cover traffic lights).
  float center_crop_x_ratio_;
  float center_crop_y_ratio_;

  std::vector<int> output_strides_;

  int src_width_;
  int src_height_;

  // host pointer for ROI
  CudaUniquePtrHost<Roi[]> roi_h_;
  // device pointer for ROI
  CudaUniquePtr<Roi[]> roi_d_;

  // flag whether model has multitasks
  int multitask_;
  // buff size for segmentation heads
  CudaUniquePtr<float[]> segmentation_out_prob_d_;
  CudaUniquePtrHost<float[]> segmentation_out_prob_h_;
  size_t segmentation_out_elem_num_;
  size_t segmentation_out_elem_num_per_batch_;
  std::vector<cv::Mat> segmentation_masks_;
  // host buffer for argmax postprocessing on GPU
  CudaUniquePtrHost<unsigned char[]> argmax_buf_h_;
  // device buffer for argmax postprocessing  on GPU
  CudaUniquePtr<unsigned char[]> argmax_buf_d_;
};

}  // namespace autoware::tensorrt_yolox

#endif  // AUTOWARE__TENSORRT_YOLOX__TENSORRT_YOLOX_HPP_
