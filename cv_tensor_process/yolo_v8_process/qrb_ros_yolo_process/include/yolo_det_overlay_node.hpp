// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef _QRB_ROS_YOLO_PROCESS_OVERLAY_HPP_
#define _QRB_ROS_YOLO_PROCESS_OVERLAY_HPP_

#include <memory>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "qrb_yolo_process/bounding_box.hpp"
#include "qrb_yolo_process/common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace qrb_ros::yolo_process
{
using namespace qrb::yolo_process;
using ExactPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image,
    vision_msgs::msg::Detection2DArray>;
using ExactSync = message_filters::Synchronizer<ExactPolicy>;
using namespace std::placeholders;

class YoloDetOverlayNode : public rclcpp::Node
{
public:
  YoloDetOverlayNode(const rclcpp::NodeOptions & options);
  ~YoloDetOverlayNode() = default;

private:
  void draw_inplace(std::vector<YoloInstance> & instances, cv::Mat & img);
  int resize_width_;
  int resize_height_;
  std::unique_ptr<ExactSync> exact_sync_{ nullptr };
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub_{ nullptr };
  message_filters::Subscriber<vision_msgs::msg::Detection2DArray> yolo_det_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> img_sub_;

  void msg_callback(sensor_msgs::msg::Image::ConstSharedPtr img_msg,
      vision_msgs::msg::Detection2DArray::ConstSharedPtr yolo_msg);
};

}  // namespace qrb_ros::yolo_process
#endif  // _QRB_ROS_YOLO_PROCESS_OVERLAY_HPP_
