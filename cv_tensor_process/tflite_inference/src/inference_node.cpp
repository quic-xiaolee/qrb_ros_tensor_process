#include <inference_node.hpp>

namespace qrb_ros::tflite_inference
{

InferenceNode::InferenceNode(const rclcpp::NodeOptions & options) : Node("inference_node", options)
{
  this->declare_parameter<std::string>("model_path", "");
  this->get_parameter("model_path", model_path_);
  if (model_path_.empty()) {
    throw std::invalid_argument("model_path not specified.");
  }

  // initialize TFLite interpreter
  model_ = tflite::FlatBufferModel::BuildFromFile(model_path_.c_str());
  if (!model_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to load TFLite model");
    throw std::runtime_error("Model loading failed");
  }

  tflite::ops::builtin::BuiltinOpResolver resolver;
  tflite::InterpreterBuilder builder(*model_, resolver);
  builder(&interpreter_);

  if (!interpreter_ || interpreter_->AllocateTensors() != kTfLiteOk) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "TFLite interpreter initialization failed");
    throw std::runtime_error("Interpreter init failed");
  }

  // print model input & output information
  {
    std::ostringstream oss_in;
    oss_in << "Output tensor info:" << std::endl;
    for (int input_idx : interpreter_->inputs()) {
      TfLiteTensor * input_tensor = interpreter_->tensor(input_idx);
      oss_in << "  Input " << input_idx << ":"
             << "  Name: " << input_tensor->name
             << "  Type: " << TfLiteTypeGetName(input_tensor->type) << "  Shape: [";
      for (int i = 0; i < input_tensor->dims->size; ++i) {
        oss_in << input_tensor->dims->data[i];
        if (i < input_tensor->dims->size - 1)
          oss_in << ", ";
      }
      oss_in << "]" << std::endl;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), oss_in.str());

    std::ostringstream oss_out;
    oss_out << "Output tensor info:" << std::endl;
    for (int output_idx : interpreter_->outputs()) {
      TfLiteTensor * output_tensor = interpreter_->tensor(output_idx);
      oss_out << "  Output " << output_idx << ":"
              << "  Name: " << output_tensor->name
              << "  Type: " << TfLiteTypeGetName(output_tensor->type) << "  Shape: [";
      for (int i = 0; i < output_tensor->dims->size; ++i) {
        oss_out << output_tensor->dims->data[i];
        if (i < output_tensor->dims->size - 1)
          oss_out << ", ";
      }
      oss_out << "]" << std::endl;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), oss_out.str());
  }

  if (interpreter_->inputs().size() != 1) {
    std::ostringstream oss;
    oss << "Error: tflite inference node: so far only support model with 1 input tensor, but "
           "current model requires "
        << interpreter_->inputs().size() << std::endl;
    RCLCPP_ERROR_STREAM(this->get_logger(), oss.str());
    throw std::runtime_error(oss.str());
  }

  // create publisher & subscriber
  sub_ = this->create_subscription<custom_msg::TensorList>(
      "input_tensor", 10, std::bind(&InferenceNode::msg_callback, this, std::placeholders::_1));

  pub_ = this->create_publisher<custom_msg::TensorList>("output_tensor", 10);
}

void InferenceNode::msg_callback(const custom_msg::TensorList::SharedPtr msg)
{
  TfLiteTensor * input_tensor = interpreter_->input_tensor(0);
  RCLCPP_INFO_STREAM(this->get_logger(), "<<< msg callback begin");

  // check input tensor shape & size & dtype
  if (msg->tensor_list[0].data_type != 2) {  // 2: FLOAT32
    RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid input tensor type "
                                                << msg->tensor_list[0].data_type
                                                << ", so far only support dtype 2(float32)");
    return;
  }

  if (msg->tensor_list.size() != 1) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
        "Invalid input tensor number, expected: 1, got: " << msg->tensor_list.size());
    return;
  }

  if (msg->tensor_list[0].shape.size() != static_cast<uint32_t>(input_tensor->dims->size)) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
        "Invalid tensor dims size, expected: " << input_tensor->dims->size
                                               << ", got: " << msg->tensor_list[0].shape.size());
    return;
  }

  for (int i = 0; i < input_tensor->dims->size; ++i) {
    if (msg->tensor_list[0].shape[i] != static_cast<uint32_t>(input_tensor->dims->data[i])) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
          "Invalid tensor shape, expected: " << input_tensor->dims->data[i]
                                             << ", got: " << msg->tensor_list[0].shape[i]);
      return;
    }
  }

  if (msg->tensor_list[0].data.size() != input_tensor->bytes) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
        "Invalid input tensor size, expected: " << input_tensor->bytes
                                                << ", got: " << msg->tensor_list[0].data.size());
    return;
  }

  // fill tensor data to interpreter buffer
  void * input_ptr = static_cast<void *>(interpreter_->typed_input_tensor<float>(0));
  std::memcpy(input_ptr, msg->tensor_list[0].data.data(), input_tensor->bytes);

  // invoke interpreter
  if (interpreter_->Invoke() != kTfLiteOk) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "tensorflow inference failed");
    return;
  }

  process_and_publish(msg);
  RCLCPP_INFO_STREAM(this->get_logger(), "<<< msg callback end");
}

void InferenceNode::process_and_publish(const custom_msg::TensorList::SharedPtr & msg)
{
  //   custom_msg::TensorList::SharedPtr msg;
  auto msg_tensor_list = std::make_unique<custom_msg::TensorList>();

  auto get_tensor_shape = [this](const TfLiteTensor * tensor) -> std::vector<uint32_t> {
    std::vector<uint32_t> shape;
    if (tensor && tensor->dims) {
      for (int i = 0; i < tensor->dims->size; ++i) {
        shape.push_back(static_cast<uint32_t>(tensor->dims->data[i]));
      }
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Tensor or its dimensions are null!");
    }
    return shape;
  };

  // traversal all output tensors
  for (int output_idx : interpreter_->outputs()) {
    const TfLiteTensor * tensor = interpreter_->tensor(output_idx);  // get tensor ptr
    if (!tensor || !tensor->data.f) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid output tensor, got null pointer");
      return;
    }

    custom_msg::Tensor msg_tensor;
    msg_tensor.name = tensor->name;
    msg_tensor.shape = get_tensor_shape(tensor);
    msg_tensor.data.resize(tensor->bytes);

    void * ptr_tensor_data = static_cast<void *>(tensor->data.f);
    std::memcpy(msg_tensor.data.data(), ptr_tensor_data, tensor->bytes);

    msg_tensor_list->tensor_list.push_back(std::move(msg_tensor));
  }
  msg_tensor_list->header = msg->header;

  pub_->publish(std::move(msg_tensor_list));
}

}  // namespace qrb_ros::tflite_inference

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::tflite_inference::InferenceNode)
