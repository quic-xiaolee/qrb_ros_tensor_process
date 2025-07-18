// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "yolo_seg_postprocess.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"

namespace qrb::yolo_process
{
YoloSegPostProcessor::YoloSegPostProcessor(const std::string & label_file,
    float score_thres,
    float iou_thres,
    float eta,
    int top_k)
  : score_thres_(score_thres), iou_thres_(iou_thres), eta_(eta), top_k_(top_k)
{
  // args range check
  auto out_of_range = [](float value) { return value <= 0.0f || value >= 1.0f; };

  if (out_of_range(score_thres_) || out_of_range(iou_thres_)) {
    throw std::invalid_argument("Err: thres out of range..");
  }

  // init label map from yaml
  try {
    YAML::Node label_yml = YAML::LoadFile(label_file);
    const YAML::Node & nd_name = label_yml["names"];

    for (YAML::const_iterator it = nd_name.begin(); it != nd_name.end(); ++it) {
      label_map_[it->first.as<int>()] = it->second.as<std::string>();
    }

  } catch (const YAML::Exception & e) {
    std::cerr << "YAML Exception: " << e.what() << std::endl;
    label_map_.clear();
  }

  // init tensor specs
  tensor_specs_ = {
    { "boxes", TensorDataType::FLOAT32, { 1, 8400, 4 } },
    { "scores", TensorDataType::FLOAT32, { 1, 8400 } },
    { "masks", TensorDataType::FLOAT32, { 1, 8400, 32 } },
    { "class_idx", TensorDataType::UINT8, { 1, 8400 } },
    { "protos", TensorDataType::FLOAT32, { 1, 32, 160, 160 } },
  };
}

void YoloSegPostProcessor::crop_masks(std::vector<std::vector<uint8_t>> & bin_masks,
    const std::vector<BoundingBox> & bboxes,
    const int input_width,
    const int input_height,
    const int mask_width,
    const int mask_height)
{
  float width_ratio = static_cast<float>(mask_width) / static_cast<float>(input_width);
  float height_ratio = static_cast<float>(mask_height) / static_cast<float>(input_height);
  if (bin_masks.size() != bboxes.size()) {
    throw std::invalid_argument("vector size not match");
  }

  for (size_t i = 0; i < bin_masks.size(); i++) {
    // create a mask for bbounding box area
    BoundingBox bbox = bboxes[i];

    std::vector<float> vec_box = bbox.to_tlbr_coords();
    cv::Mat mask = cv::Mat::ones(mask_width, mask_height, CV_8UC1);
    float tl_x = vec_box[0] * width_ratio + 1.0f;
    float tl_y = vec_box[1] * height_ratio + 1.0f;
    float br_x = vec_box[2] * width_ratio - 1.0f;
    float br_y = vec_box[3] * height_ratio - 1.0f;
    cv::Point top_left(tl_x, tl_y);
    cv::Point bottom_right(br_x, br_y);
    cv::rectangle(mask, top_left, bottom_right, cv::Scalar(0), cv::FILLED);

    // wrap bin_masks[i] into cv::Mat, clear area not coevered by "mask"
    cv::Mat bin_mask_img(mask_width, mask_height, CV_8UC1, bin_masks[i].data());
    bin_mask_img.setTo(0, mask);
  }
}

void YoloSegPostProcessor::process_mask(const std::vector<std::vector<float>> & protos,
    const std::vector<std::vector<float>> & mask_in,
    const std::vector<BoundingBox> & bboxes,
    const int input_width,
    const int input_height,
    const int mask_width,
    const int mask_height,
    std::vector<std::vector<uint8_t>> & bin_masks)
{
  size_t n = mask_in.size();            // number of valid instance
  size_t mask_dim = mask_in[0].size();  // YOLOv8_SEG: 32
  size_t mask_size = protos[0].size();  // mask_h* mask_w, YOLOv8_SEG: 160*160

  if (protos.size() != mask_dim) {
    throw std::invalid_argument("Invalid matrix dimensions for multiplication");
  }

  // alloc buf for vector in advance
  bin_masks.resize(n);
  for (auto & bin_mask : bin_masks) {
    bin_mask.reserve(mask_size);
  }

  // matrix multiplication
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = 0; j < mask_size; ++j) {
      double res_ij = 0.0;

      for (size_t k = 0; k < mask_dim; ++k) {
        res_ij += mask_in[i][k] * protos[k][j];
      }

      if (res_ij >= 0.0) {
        bin_masks[i].push_back(255);
      } else {
        bin_masks[i].push_back(0);
      }
    }
  }

  crop_masks(bin_masks, bboxes, input_width, input_height, mask_width, mask_height);
}

void YoloSegPostProcessor::process(const std::vector<Tensor> & tensors,
    std::vector<YoloInstance> & instances)
{
  validate_tensors(tensors, tensor_specs_);
  std::vector<int> indices;

  non_maximum_suppression(tensors, score_thres_, iou_thres_, indices, eta_, top_k_);

  const int n = indices.size();
  if (n == 0) {
    return;
  }

  const int idx_boxes = get_tensor_idx(tensors, "boxes");
  const int idx_scores = get_tensor_idx(tensors, "scores");
  const int idx_masks = get_tensor_idx(tensors, "masks");
  const int idx_class = get_tensor_idx(tensors, "class_idx");
  const int idx_protos = get_tensor_idx(tensors, "protos");

  const float(*const ptr_bbox)[4] = reinterpret_cast<float(*)[4]>(tensors[idx_boxes].p_vec->data());
  const float * const ptr_score = reinterpret_cast<float *>(tensors[idx_scores].p_vec->data());
  const float * const ptr_mask = reinterpret_cast<float *>(tensors[idx_masks].p_vec->data());
  const uint8_t * const ptr_label = reinterpret_cast<uint8_t *>(tensors[idx_class].p_vec->data());
  const float * const ptr_proto_mask = reinterpret_cast<float *>(tensors[idx_protos].p_vec->data());

  // fixed value required by model
  const std::vector<int> input_shape = { 640, 640 };
  const std::vector<int> mask_shape = { 160, 160 };
  int mask_size = mask_shape[0] * mask_shape[1];
  int mask_dims = 32;

  // populate proto mask matrix, [1, mask_dims, 160, 160] -> [mask_dims, 160*160]
  std::vector<std::vector<float>> vec_proto_mask;
  vec_proto_mask.reserve(mask_dims);
  for (int i = 0; i < mask_dims; ++i) {
    // Get the start and end pointers for the current dimension
    const float * const p_head = ptr_proto_mask + i * mask_size;
    const float * const p_tail = p_head + mask_size;

    vec_proto_mask.emplace_back(p_head, p_tail);
  }

  // populate valid instance mask matrix, [1, num_preds, mask_dims] -> [n, mask_dims]
  std::vector<std::vector<float>> vec_mask(n, std::vector<float>(mask_dims));
  for (int i = 0; i < n; i++) {
    const float * const ptr = ptr_mask + indices[i] * mask_dims;

    // copy the entire row o mask
    std::memcpy(vec_mask[i].data(), ptr, mask_dims * sizeof(float));
  }

  // instance mask processing: [n, mask_dims] * [mask_dims, 160*160] = [n, 160*160]
  std::vector<BoundingBox> vec_bbox;
  vec_bbox.reserve(n);
  for (auto & idx : indices) {
    const float * const p = ptr_bbox[idx];
    BoundingBox bbox = BoundingBox({ p[0], p[1], p[2], p[3] }, BoundingBox::BoxFmt::TLBR);
    vec_bbox.push_back(bbox);
  }

  std::vector<std::vector<uint8_t>> bin_masks;  // binary mask for all valid instance
  process_mask(vec_proto_mask, vec_mask, vec_bbox, input_shape[0], input_shape[1], mask_shape[0],
      mask_shape[1], bin_masks);

  int iter_count = 0;
  for (auto & idx : indices) {
    float score = ptr_score[idx];

    std::string label;
    try {
      label = label_map_.at(ptr_label[idx]);
    } catch (const std::out_of_range & e) {
      label = "unknown";
    }
    YoloInstance instance(ptr_bbox[idx][0], ptr_bbox[idx][1], ptr_bbox[idx][2], ptr_bbox[idx][3],
        BoundingBox::BoxFmt::TLBR, score, label);
    instance.mask = std::move(bin_masks[iter_count]);

    instances.push_back(instance);
    iter_count++;
  }
}

}  // namespace qrb::yolo_process
