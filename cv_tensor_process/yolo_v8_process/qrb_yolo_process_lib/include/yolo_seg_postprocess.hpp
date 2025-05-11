// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef _QRB_YOLO_SEGMENTATION_POST_PROCESS_CPP_
#define _QRB_YOLO_SEGMENTATION_POST_PROCESS_CPP_

#include <iostream>
#include <map>

#include "common.hpp"

namespace qrb::yolo_process
{
class YoloSegPostProcessor
{
public:
  /**
   * \brief YoloSegPostProcessor Constructor
   * \param label_file Yaml file path that contains the "class_id <--> string" mapping
   * \param score_thres Score threshold (0~1) for NMS(non-maximum-suppression)
   * \param iou_thres Iou threshold (0~1) for NMS(non-maximum-suppression)
   */
  YoloSegPostProcessor(const std::string & label_file,
      float score_thres = 0.5f,
      float iou_thres = 0.3f,
      float eta = 1.f,
      int top_k = 0);
  ~YoloSegPostProcessor() = default;

  /**
   * \brief Handle the output tensors of YOLO segmentation model and return instance
   * info, including bounding box / score / label / binary mask
   * \param tensors Tensors output from YOLO segmentation model.
   * \param instances (Output) Detected instances info will be stored here
   */
  void process(const std::vector<Tensor> & tensors, std::vector<YoloInstance> & instances);

private:
  /**
   * \brief Calculate binary mask(in mono8 format) for detected objects
   * \param mask_in 2D vector with shape [n, mask_dim].
   * \param protos  2D vector with shape [mask_dim, mask_h*mask_w].
   * \param bboxes  2D vector with shape [n, 4].
   * \param input_width width of image input required by model
   * \param input_height height of image input required by model
   * \param mask_width width of mask output by model
   * \param mask_height height of mask output by model
   * \param bin_masks (output) 2D vector with shape [n, mask_h * mask_w], final binary mask for
   * objects will store here
   */
  void process_mask(const std::vector<std::vector<float>> & protos,
      const std::vector<std::vector<float>> & mask_in,
      const std::vector<BoundingBox> & bboxes,
      const int input_width,
      const int input_height,
      const int mask_width,
      const int mask_height,
      std::vector<std::vector<uint8_t>> & bin_masks);

  /**
   * \brief Zero out pixels that outside the bounding box range, i.e., crop the mask that
   * doesn't belong to detected instance.
   */
  void crop_masks(std::vector<std::vector<uint8_t>> & bin_masks,
      const std::vector<BoundingBox> & bboxes,
      const int input_width,
      const int input_height,
      const int mask_width,
      const int mask_height);

  std::map<int, std::string> label_map_;
  std::vector<TensorSpec> tensor_specs_;

  // member for NMS
  float score_thres_;
  float iou_thres_;
  float eta_;
  int top_k_;
};

}  // namespace qrb::yolo_process
#endif  // _QRB_YOLO_SEGMENTATION_POST_PROCESS_CPP_