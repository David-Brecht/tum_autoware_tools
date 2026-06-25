#include "tod_perception_tools/tod_detected_object_merger_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace tod_perception_tools
{

TodDetectedObjectMergerNode::TodDetectedObjectMergerNode()
: Node("tod_detected_object_merger")
{
  input_topics_[0] = this->declare_parameter<std::string>("input_topic0", input_topics_[0]);
  input_topics_[1] = this->declare_parameter<std::string>("input_topic1", input_topics_[1]);
  output_topic_name_ = this->declare_parameter<std::string>("output_topic_name", output_topic_name_);
  new_frame_id_ = this->declare_parameter<std::string>("new_frame_id", new_frame_id_);
  update_rate_hz_ = this->declare_parameter<double>("update_rate_hz", update_rate_hz_);
  timeout_threshold_ = this->declare_parameter<double>("timeout_threshold", timeout_threshold_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  for (size_t i = 0; i < kNumInputs; ++i) {
    std::function<void(const DetectedObjects::ConstSharedPtr)> callback =
      std::bind(&TodDetectedObjectMergerNode::on_data, this, std::placeholders::_1, i);
    sub_objects_[i] = this->create_subscription<DetectedObjects>(
      input_topics_[i], rclcpp::QoS{1}.best_effort(), callback);
  }

  pub_objects_ = this->create_publisher<DetectedObjects>(
    output_topic_name_, rclcpp::QoS{1}.reliable());

  const double clamped_rate = std::max(1e-3, update_rate_hz_);
  const auto period = std::chrono::duration<double>(1.0 / clamped_rate);
  timer_ = rclcpp::create_timer(
    this, this->get_clock(),
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&TodDetectedObjectMergerNode::on_timer, this));
}

void TodDetectedObjectMergerNode::on_data(
  const DetectedObjects::ConstSharedPtr msg, const size_t input_index)
{
  objects_data_[input_index] = msg;
}

TodDetectedObjectMergerNode::DetectedObjects::SharedPtr
TodDetectedObjectMergerNode::transform_objects(
  const DetectedObjects::ConstSharedPtr & objects, const std::string & target_frame_id)
{
  auto output = std::make_shared<DetectedObjects>(*objects);

  if (objects->header.frame_id.empty() || objects->header.frame_id == target_frame_id) {
    output->header.frame_id = target_frame_id;
    return output;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      target_frame_id, objects->header.frame_id, objects->header.stamp,
      rclcpp::Duration::from_seconds(0.01));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "Could not transform objects from '%s' to '%s': %s", objects->header.frame_id.c_str(),
      target_frame_id.c_str(), ex.what());
    return nullptr;
  }

  output->header.frame_id = target_frame_id;
  for (auto & object : output->objects) {
    auto & pose = object.kinematics.pose_with_covariance.pose;
    tf2::doTransform(pose, pose, transform);
  }

  return output;
}

void TodDetectedObjectMergerNode::on_timer()
{
  rclcpp::Time latest_stamp{0, 0, this->get_clock()->get_clock_type()};
  bool has_valid_input = false;

  for (size_t i = 0; i < kNumInputs; ++i) {
    if (objects_data_[i]) {
      const rclcpp::Time stamp = objects_data_[i]->header.stamp;
      if (!has_valid_input || stamp > latest_stamp) {
        latest_stamp = stamp;
        has_valid_input = true;
      }
    }
  }

  DetectedObjects output_objects;
  output_objects.header.frame_id = new_frame_id_;

  if (!has_valid_input) {
    output_objects.header.stamp = this->now();
    pub_objects_->publish(output_objects);
    return;
  }

  output_objects.header.stamp = latest_stamp;

  for (size_t i = 0; i < kNumInputs; ++i) {
    if (!objects_data_[i]) {
      continue;
    }

    const double time_diff =
      rclcpp::Time(objects_data_[i]->header.stamp).seconds() - latest_stamp.seconds();
    if (std::abs(time_diff) >= timeout_threshold_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Topic '%s' is timed out by %f sec", input_topics_[i].c_str(), time_diff);
      continue;
    }

    auto transformed_objects = transform_objects(objects_data_[i], new_frame_id_);
    if (!transformed_objects) {
      continue;
    }

    output_objects.objects.insert(
      output_objects.objects.end(), transformed_objects->objects.begin(),
      transformed_objects->objects.end());
  }

  pub_objects_->publish(output_objects);
}

}  // namespace tod_perception_tools

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tod_perception_tools::TodDetectedObjectMergerNode>());
  rclcpp::shutdown();
  return 0;
}
