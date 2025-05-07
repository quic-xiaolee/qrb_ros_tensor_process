#include "qrb_ros_tensor_list_msgs/msg/tensor.hpp"
#include "qrb_ros_tensor_list_msgs/msg/tensor_list.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/model.h"
#include "tensorflow/lite/tools/gen_op_registration.h"

namespace qrb_ros::tflite_inference
{
namespace custom_msg = qrb_ros_tensor_list_msgs::msg;

class InferenceNode : public rclcpp::Node
{
public:
  InferenceNode(const rclcpp::NodeOptions & options);
  ~InferenceNode() = default;

  void msg_callback(const custom_msg::TensorList::SharedPtr msg);
  void process_and_publish(const custom_msg::TensorList::SharedPtr & msg);

private:
  std::unique_ptr<tflite::FlatBufferModel> model_;
  std::unique_ptr<tflite::Interpreter> interpreter_;
  std::string model_path_;

  rclcpp::Subscription<custom_msg::TensorList>::SharedPtr sub_{ nullptr };
  rclcpp::Publisher<custom_msg::TensorList>::SharedPtr pub_{ nullptr };
};
}  // namespace qrb_ros::tflite_inference
