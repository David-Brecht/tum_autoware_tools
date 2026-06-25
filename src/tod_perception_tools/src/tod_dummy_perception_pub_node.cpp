#include "tod_perception_tools/tod_dummy_perception_pub_node.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "autoware_perception_msgs/msg/detected_object_kinematics.hpp"
#include "autoware_perception_msgs/msg/object_classification.hpp"
#include "autoware_perception_msgs/msg/shape.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "yaml-cpp/yaml.h"

namespace
{

template <typename T>
T get_value(const YAML::Node & node, const std::string & key, const T & default_value)
{
  if (!node || !node[key]) {
    return default_value;
  }
  return node[key].as<T>();
}

geometry_msgs::msg::Point32 parse_point32(const YAML::Node & node)
{
  geometry_msgs::msg::Point32 point;
  point.x = get_value<float>(node, "x", 0.0f);
  point.y = get_value<float>(node, "y", 0.0f);
  point.z = get_value<float>(node, "z", 0.0f);
  return point;
}

}  // namespace

namespace tod_perception_tools
{

TodDummyPerceptionPubNode::TodDummyPerceptionPubNode()
: Node("tod_dummy_perception_pub"), rng_(std::random_device{}())
{
  publish_frequency_ = this->declare_parameter<double>("publish_frequency", publish_frequency_);
  output_topic_name_ = this->declare_parameter<std::string>("output_topic_name", output_topic_name_);
  position_covariance_ = this->declare_parameter<double>("position_covariance", position_covariance_);

  const auto pkg_share = ament_index_cpp::get_package_share_directory("tod_perception_tools");
  detected_objects_path_ = pkg_share + "/config/detected_objects_list.yaml";
  try {
    base_message_ = load_detected_objects(detected_objects_path_);
  } catch (const YAML::Exception & ex) {
    RCLCPP_FATAL(this->get_logger(), "Failed to load detected objects YAML: %s", ex.what());
    throw;
  }

  const double clamped_covariance = std::max(0.0, position_covariance_);
  position_noise_ =
    std::normal_distribution<double>(0.0, std::sqrt(clamped_covariance));

  pub_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
    output_topic_name_, rclcpp::QoS(rclcpp::KeepLast(10)));

  const double clamped_frequency = std::max(1e-3, publish_frequency_);
  const auto period = std::chrono::duration<double>(1.0 / clamped_frequency);
  // timer_ = this->create_wall_timer(
  //   std::chrono::duration_cast<std::chrono::milliseconds>(period),
  //   std::bind(&TodDummyPerceptionPubNode::timer_callback, this));
  timer_ = rclcpp::create_timer(
    this,
    this->get_clock(),
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&TodDummyPerceptionPubNode::timer_callback, this));
}

autoware_perception_msgs::msg::DetectedObjects TodDummyPerceptionPubNode::load_detected_objects(
  const std::string & path)
{
  autoware_perception_msgs::msg::DetectedObjects msg;
  YAML::Node root = YAML::LoadFile(path);

  const auto header_node = root["header"];
  const auto stamp_node = header_node ? header_node["stamp"] : YAML::Node();
  msg.header.stamp.sec = get_value<int32_t>(stamp_node, "sec", 0);
  msg.header.stamp.nanosec = get_value<uint32_t>(stamp_node, "nanosec", 0u);
  msg.header.frame_id = get_value<std::string>(header_node, "frame_id", "");

  const auto objects_node = root["objects"];
  if (!objects_node || !objects_node.IsSequence()) {
    return msg;
  }

  for (const auto & object_node : objects_node) {
    autoware_perception_msgs::msg::DetectedObject obj;
    obj.existence_probability = get_value<float>(object_node, "existence_probability", 0.0f);

    const auto classifications_node = object_node["classification"];
    if (classifications_node && classifications_node.IsSequence()) {
      for (const auto & class_node : classifications_node) {
        autoware_perception_msgs::msg::ObjectClassification classification;
        classification.label =
          static_cast<uint8_t>(get_value<int>(class_node, "label", 0));
        classification.probability = get_value<float>(class_node, "probability", 0.0f);
        obj.classification.push_back(classification);
      }
    }

    const auto kinematics_node = object_node["kinematics"];
    if (kinematics_node) {
      const auto pose_with_cov_node = kinematics_node["pose_with_covariance"];
      const auto pose_node = pose_with_cov_node ? pose_with_cov_node["pose"] : YAML::Node();
      const auto position_node = pose_node ? pose_node["position"] : YAML::Node();
      obj.kinematics.pose_with_covariance.pose.position.x =
        get_value<double>(position_node, "x", 0.0);
      obj.kinematics.pose_with_covariance.pose.position.y =
        get_value<double>(position_node, "y", 0.0);
      obj.kinematics.pose_with_covariance.pose.position.z =
        get_value<double>(position_node, "z", 0.0);

      const auto orientation_node = pose_node ? pose_node["orientation"] : YAML::Node();
      obj.kinematics.pose_with_covariance.pose.orientation.x =
        get_value<double>(orientation_node, "x", 0.0);
      obj.kinematics.pose_with_covariance.pose.orientation.y =
        get_value<double>(orientation_node, "y", 0.0);
      obj.kinematics.pose_with_covariance.pose.orientation.z =
        get_value<double>(orientation_node, "z", 0.0);
      obj.kinematics.pose_with_covariance.pose.orientation.w =
        get_value<double>(orientation_node, "w", 1.0);

      obj.kinematics.pose_with_covariance.covariance.fill(0.0);
      const auto covariance_node =
        pose_with_cov_node ? pose_with_cov_node["covariance"] : YAML::Node();
      if (covariance_node && covariance_node.IsSequence()) {
        const size_t count = std::min<size_t>(covariance_node.size(), 36);
        for (size_t index = 0; index < count; ++index) {
          obj.kinematics.pose_with_covariance.covariance[index] =
            covariance_node[index].as<double>();
        }
      }

      obj.kinematics.has_position_covariance =
        get_value<bool>(kinematics_node, "has_position_covariance", false);
      obj.kinematics.orientation_availability =
        static_cast<uint8_t>(get_value<int>(
          kinematics_node, "orientation_availability",
          autoware_perception_msgs::msg::DetectedObjectKinematics::UNAVAILABLE));

      const auto twist_with_cov_node = kinematics_node["twist_with_covariance"];
      const auto twist_node = twist_with_cov_node ? twist_with_cov_node["twist"] : YAML::Node();
      const auto linear_node = twist_node ? twist_node["linear"] : YAML::Node();
      obj.kinematics.twist_with_covariance.twist.linear.x =
        get_value<double>(linear_node, "x", 0.0);
      obj.kinematics.twist_with_covariance.twist.linear.y =
        get_value<double>(linear_node, "y", 0.0);
      obj.kinematics.twist_with_covariance.twist.linear.z =
        get_value<double>(linear_node, "z", 0.0);

      const auto angular_node = twist_node ? twist_node["angular"] : YAML::Node();
      obj.kinematics.twist_with_covariance.twist.angular.x =
        get_value<double>(angular_node, "x", 0.0);
      obj.kinematics.twist_with_covariance.twist.angular.y =
        get_value<double>(angular_node, "y", 0.0);
      obj.kinematics.twist_with_covariance.twist.angular.z =
        get_value<double>(angular_node, "z", 0.0);

      obj.kinematics.twist_with_covariance.covariance.fill(0.0);
      const auto twist_covariance_node =
        twist_with_cov_node ? twist_with_cov_node["covariance"] : YAML::Node();
      if (twist_covariance_node && twist_covariance_node.IsSequence()) {
        const size_t count = std::min<size_t>(twist_covariance_node.size(), 36);
        for (size_t index = 0; index < count; ++index) {
          obj.kinematics.twist_with_covariance.covariance[index] =
            twist_covariance_node[index].as<double>();
        }
      }

      obj.kinematics.has_twist = get_value<bool>(kinematics_node, "has_twist", false);
      obj.kinematics.has_twist_covariance =
        get_value<bool>(kinematics_node, "has_twist_covariance", false);
    }

    const auto shape_node = object_node["shape"];
    if (shape_node) {
      obj.shape.type = static_cast<uint8_t>(get_value<int>(
        shape_node, "type", autoware_perception_msgs::msg::Shape::BOUNDING_BOX));

      const auto footprint_node = shape_node["footprint"];
      const auto points_node = footprint_node ? footprint_node["points"] : YAML::Node();
      if (points_node && points_node.IsSequence()) {
        for (const auto & point_node : points_node) {
          obj.shape.footprint.points.push_back(parse_point32(point_node));
        }
      }

      const auto dimensions_node = shape_node["dimensions"];
      obj.shape.dimensions.x = get_value<double>(dimensions_node, "x", 0.0);
      obj.shape.dimensions.y = get_value<double>(dimensions_node, "y", 0.0);
      obj.shape.dimensions.z = get_value<double>(dimensions_node, "z", 0.0);
    }

    msg.objects.push_back(obj);
  }

  return msg;
}

void TodDummyPerceptionPubNode::apply_position_noise(
  autoware_perception_msgs::msg::DetectedObjects & msg)
{
  if (position_covariance_ <= 0.0) {
    return;
  }

  for (auto & obj : msg.objects) {
    auto & pose = obj.kinematics.pose_with_covariance.pose;
    pose.position.x += position_noise_(rng_);
    pose.position.y += position_noise_(rng_);
    pose.position.z += position_noise_(rng_);

    obj.kinematics.has_position_covariance = true;
    obj.kinematics.pose_with_covariance.covariance.fill(0.0);
    obj.kinematics.pose_with_covariance.covariance[0] = position_covariance_;
    obj.kinematics.pose_with_covariance.covariance[7] = position_covariance_;
    obj.kinematics.pose_with_covariance.covariance[14] = position_covariance_;
  }
}

void TodDummyPerceptionPubNode::timer_callback()
{
  auto msg = base_message_;
  msg.header.stamp = this->get_clock()->now();
  apply_position_noise(msg);
  pub_->publish(msg);
}

}  // namespace tod_perception_tools

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tod_perception_tools::TodDummyPerceptionPubNode>());
  rclcpp::shutdown();
  return 0;
}
