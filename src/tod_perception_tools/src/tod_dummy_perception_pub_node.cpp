#include "tod_perception_tools/tod_dummy_perception_pub_node.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "autoware_perception_msgs/msg/detected_object_kinematics.hpp"
#include "autoware_perception_msgs/msg/object_classification.hpp"
#include "autoware_perception_msgs/msg/predicted_path.hpp"
#include "autoware_perception_msgs/msg/shape.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
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

void parse_covariance(const YAML::Node & covariance_node, std::array<double, 36> & out)
{
  out.fill(0.0);
  if (covariance_node && covariance_node.IsSequence()) {
    const size_t count = std::min<size_t>(covariance_node.size(), 36);
    for (size_t index = 0; index < count; ++index) {
      out[index] = covariance_node[index].as<double>();
    }
  }
}

geometry_msgs::msg::Pose parse_pose(const YAML::Node & pose_node)
{
  geometry_msgs::msg::Pose pose;
  const auto position_node = pose_node ? pose_node["position"] : YAML::Node();
  pose.position.x = get_value<double>(position_node, "x", 0.0);
  pose.position.y = get_value<double>(position_node, "y", 0.0);
  pose.position.z = get_value<double>(position_node, "z", 0.0);

  const auto orientation_node = pose_node ? pose_node["orientation"] : YAML::Node();
  pose.orientation.x = get_value<double>(orientation_node, "x", 0.0);
  pose.orientation.y = get_value<double>(orientation_node, "y", 0.0);
  pose.orientation.z = get_value<double>(orientation_node, "z", 0.0);
  pose.orientation.w = get_value<double>(orientation_node, "w", 1.0);
  return pose;
}

void parse_pose_with_covariance(
  const YAML::Node & pose_with_cov_node, geometry_msgs::msg::PoseWithCovariance & out)
{
  out.pose = parse_pose(pose_with_cov_node ? pose_with_cov_node["pose"] : YAML::Node());
  parse_covariance(
    pose_with_cov_node ? pose_with_cov_node["covariance"] : YAML::Node(), out.covariance);
}

void parse_twist_with_covariance(
  const YAML::Node & twist_with_cov_node, geometry_msgs::msg::TwistWithCovariance & out)
{
  const auto twist_node = twist_with_cov_node ? twist_with_cov_node["twist"] : YAML::Node();
  const auto linear_node = twist_node ? twist_node["linear"] : YAML::Node();
  out.twist.linear.x = get_value<double>(linear_node, "x", 0.0);
  out.twist.linear.y = get_value<double>(linear_node, "y", 0.0);
  out.twist.linear.z = get_value<double>(linear_node, "z", 0.0);

  const auto angular_node = twist_node ? twist_node["angular"] : YAML::Node();
  out.twist.angular.x = get_value<double>(angular_node, "x", 0.0);
  out.twist.angular.y = get_value<double>(angular_node, "y", 0.0);
  out.twist.angular.z = get_value<double>(angular_node, "z", 0.0);

  parse_covariance(
    twist_with_cov_node ? twist_with_cov_node["covariance"] : YAML::Node(), out.covariance);
}

void parse_classifications(
  const YAML::Node & object_node,
  std::vector<autoware_perception_msgs::msg::ObjectClassification> & out)
{
  const auto classifications_node = object_node["classification"];
  if (classifications_node && classifications_node.IsSequence()) {
    for (const auto & class_node : classifications_node) {
      autoware_perception_msgs::msg::ObjectClassification classification;
      classification.label = static_cast<uint8_t>(get_value<int>(class_node, "label", 0));
      classification.probability = get_value<float>(class_node, "probability", 0.0f);
      out.push_back(classification);
    }
  }
}

void parse_shape(const YAML::Node & shape_node, autoware_perception_msgs::msg::Shape & out)
{
  if (!shape_node) {
    return;
  }
  out.type = static_cast<uint8_t>(
    get_value<int>(shape_node, "type", autoware_perception_msgs::msg::Shape::BOUNDING_BOX));

  const auto footprint_node = shape_node["footprint"];
  const auto points_node = footprint_node ? footprint_node["points"] : YAML::Node();
  if (points_node && points_node.IsSequence()) {
    for (const auto & point_node : points_node) {
      out.footprint.points.push_back(parse_point32(point_node));
    }
  }

  const auto dimensions_node = shape_node["dimensions"];
  out.dimensions.x = get_value<double>(dimensions_node, "x", 0.0);
  out.dimensions.y = get_value<double>(dimensions_node, "y", 0.0);
  out.dimensions.z = get_value<double>(dimensions_node, "z", 0.0);
}

void parse_predicted_paths(
  const YAML::Node & predicted_paths_node,
  std::vector<autoware_perception_msgs::msg::PredictedPath> & out)
{
  if (!predicted_paths_node || !predicted_paths_node.IsSequence()) {
    return;
  }

  for (const auto & path_node : predicted_paths_node) {
    autoware_perception_msgs::msg::PredictedPath predicted_path;
    predicted_path.confidence = get_value<float>(path_node, "confidence", 0.0f);

    const auto time_step_node = path_node["time_step"];
    predicted_path.time_step.sec = get_value<int32_t>(time_step_node, "sec", 0);
    predicted_path.time_step.nanosec = get_value<uint32_t>(time_step_node, "nanosec", 0u);

    const auto path_points_node = path_node["path"];
    if (path_points_node && path_points_node.IsSequence()) {
      for (const auto & pose_node : path_points_node) {
        predicted_path.path.push_back(parse_pose(pose_node));
      }
    }

    out.push_back(predicted_path);
  }
}

// Resolve the frame_id for the message header. Prefer an explicit top-level
// header.frame_id, then a top-level frame_id, and finally fall back to the
// frame_id defined on the individual objects (which is how the lists are authored).
std::string resolve_header_frame_id(const YAML::Node & root)
{
  const auto header_node = root["header"];
  std::string frame_id = get_value<std::string>(header_node, "frame_id", "");
  if (!frame_id.empty()) {
    return frame_id;
  }

  frame_id = get_value<std::string>(root, "frame_id", "");
  if (!frame_id.empty()) {
    return frame_id;
  }

  const auto objects_node = root["objects"];
  if (objects_node && objects_node.IsSequence()) {
    for (const auto & object_node : objects_node) {
      frame_id = get_value<std::string>(object_node, "frame_id", "");
      if (!frame_id.empty()) {
        return frame_id;
      }
    }
  }

  return frame_id;
}

}  // namespace

namespace tod_perception_tools
{

TodDummyPerceptionPubNode::TodDummyPerceptionPubNode()
: Node("tod_dummy_perception_pub"), rng_(std::random_device{}())
{
  publish_frequency_ = this->declare_parameter<double>("publish_frequency", publish_frequency_);
  output_topic_name_ = this->declare_parameter<std::string>("output_topic_name", output_topic_name_);
  std_dev_x_ = this->declare_parameter<double>("std_dev_x", std_dev_x_);
  std_dev_y_ = this->declare_parameter<double>("std_dev_y", std_dev_y_);
  std_dev_z_ = this->declare_parameter<double>("std_dev_z", std_dev_z_);
  std_dev_theta_ = this->declare_parameter<double>("std_dev_theta", std_dev_theta_);
  publish_predicted_objects_ =
    this->declare_parameter<bool>("publish_predicted_objects", publish_predicted_objects_);

  const auto pkg_share = ament_index_cpp::get_package_share_directory("tod_perception_tools");
  detected_objects_path_ = pkg_share + "/config/detected_objects_list.yaml";
  try {
    if (publish_predicted_objects_) {
      predicted_base_message_ = load_predicted_objects(detected_objects_path_);
    } else {
      detected_base_message_ = load_detected_objects(detected_objects_path_);
    }
  } catch (const YAML::Exception & ex) {
    RCLCPP_FATAL(this->get_logger(), "Failed to load objects YAML: %s", ex.what());
    throw;
  }

  std_dev_x_ = std::max(0.0, std_dev_x_);
  std_dev_y_ = std::max(0.0, std_dev_y_);
  std_dev_z_ = std::max(0.0, std_dev_z_);
  std_dev_theta_ = std::max(0.0, std_dev_theta_);

  if (publish_predicted_objects_) {
    predicted_pub_ = this->create_publisher<autoware_perception_msgs::msg::PredictedObjects>(
      output_topic_name_, rclcpp::QoS(rclcpp::KeepLast(10)));
  } else {
    detected_pub_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
      output_topic_name_, rclcpp::QoS(rclcpp::KeepLast(10)));
  }

  const double clamped_frequency = std::max(1e-3, publish_frequency_);
  const auto period = std::chrono::duration<double>(1.0 / clamped_frequency);
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
  msg.header.frame_id = resolve_header_frame_id(root);

  const auto objects_node = root["objects"];
  if (!objects_node || !objects_node.IsSequence()) {
    return msg;
  }

  for (const auto & object_node : objects_node) {
    autoware_perception_msgs::msg::DetectedObject obj;
    obj.existence_probability = get_value<float>(object_node, "existence_probability", 0.0f);

    parse_classifications(object_node, obj.classification);

    const auto kinematics_node = object_node["kinematics"];
    if (kinematics_node) {
      parse_pose_with_covariance(
        kinematics_node["pose_with_covariance"], obj.kinematics.pose_with_covariance);
      parse_twist_with_covariance(
        kinematics_node["twist_with_covariance"], obj.kinematics.twist_with_covariance);

      obj.kinematics.has_position_covariance =
        get_value<bool>(kinematics_node, "has_position_covariance", false);
      obj.kinematics.orientation_availability =
        static_cast<uint8_t>(get_value<int>(
          kinematics_node, "orientation_availability",
          autoware_perception_msgs::msg::DetectedObjectKinematics::UNAVAILABLE));
      obj.kinematics.has_twist = get_value<bool>(kinematics_node, "has_twist", false);
      obj.kinematics.has_twist_covariance =
        get_value<bool>(kinematics_node, "has_twist_covariance", false);
    }

    parse_shape(object_node["shape"], obj.shape);

    msg.objects.push_back(obj);
  }

  return msg;
}

autoware_perception_msgs::msg::PredictedObjects TodDummyPerceptionPubNode::load_predicted_objects(
  const std::string & path)
{
  autoware_perception_msgs::msg::PredictedObjects msg;
  YAML::Node root = YAML::LoadFile(path);

  const auto header_node = root["header"];
  const auto stamp_node = header_node ? header_node["stamp"] : YAML::Node();
  msg.header.stamp.sec = get_value<int32_t>(stamp_node, "sec", 0);
  msg.header.stamp.nanosec = get_value<uint32_t>(stamp_node, "nanosec", 0u);
  msg.header.frame_id = resolve_header_frame_id(root);

  const auto objects_node = root["objects"];
  if (!objects_node || !objects_node.IsSequence()) {
    return msg;
  }

  for (const auto & object_node : objects_node) {
    autoware_perception_msgs::msg::PredictedObject obj;
    obj.existence_probability = get_value<float>(object_node, "existence_probability", 0.0f);

    parse_classifications(object_node, obj.classification);

    const auto kinematics_node = object_node["kinematics"];
    if (kinematics_node) {
      parse_pose_with_covariance(
        kinematics_node["pose_with_covariance"], obj.kinematics.initial_pose_with_covariance);
      parse_twist_with_covariance(
        kinematics_node["twist_with_covariance"], obj.kinematics.initial_twist_with_covariance);
    }

    // predicted_paths stays empty unless explicitly provided in the YAML. It may be
    // authored either at the object level or nested inside kinematics. Use copy
    // initialization (never assignment) to avoid mutating the source YAML node.
    const YAML::Node predicted_paths_node =
      object_node["predicted_paths"]
        ? object_node["predicted_paths"]
        : (kinematics_node ? kinematics_node["predicted_paths"] : YAML::Node());
    parse_predicted_paths(predicted_paths_node, obj.kinematics.predicted_paths);

    parse_shape(object_node["shape"], obj.shape);

    msg.objects.push_back(obj);
  }

  return msg;
}

double TodDummyPerceptionPubNode::sample_noise(double std_dev)
{
  if (std_dev <= 0.0) {
    return 0.0;
  }
  std::normal_distribution<double> dist(0.0, std_dev);
  return dist(rng_);
}

void TodDummyPerceptionPubNode::apply_pose_noise(
  geometry_msgs::msg::PoseWithCovariance & pose_with_covariance)
{
  auto & pose = pose_with_covariance.pose;
  pose.position.x += sample_noise(std_dev_x_);
  pose.position.y += sample_noise(std_dev_y_);
  pose.position.z += sample_noise(std_dev_z_);

  // Perturb the yaw while preserving any existing roll/pitch by composing a
  // delta rotation about the z-axis in the parent frame.
  if (std_dev_theta_ > 0.0) {
    tf2::Quaternion q_orig;
    tf2::fromMsg(pose.orientation, q_orig);
    tf2::Quaternion q_delta;
    q_delta.setRPY(0.0, 0.0, sample_noise(std_dev_theta_));
    tf2::Quaternion q_new = q_delta * q_orig;
    q_new.normalize();
    pose.orientation = tf2::toMsg(q_new);
  }

  // Encode the configured standard deviations as the pose covariance diagonal
  // (x, y, z, roll, pitch, yaw) in the 6x6 row-major matrix.
  pose_with_covariance.covariance.fill(0.0);
  pose_with_covariance.covariance[0] = std_dev_x_ * std_dev_x_;
  pose_with_covariance.covariance[7] = std_dev_y_ * std_dev_y_;
  pose_with_covariance.covariance[14] = std_dev_z_ * std_dev_z_;
  pose_with_covariance.covariance[35] = std_dev_theta_ * std_dev_theta_;
}

void TodDummyPerceptionPubNode::apply_position_noise(
  autoware_perception_msgs::msg::DetectedObjects & msg)
{
  for (auto & obj : msg.objects) {
    apply_pose_noise(obj.kinematics.pose_with_covariance);
    obj.kinematics.has_position_covariance = true;
  }
}

void TodDummyPerceptionPubNode::apply_position_noise(
  autoware_perception_msgs::msg::PredictedObjects & msg)
{
  for (auto & obj : msg.objects) {
    apply_pose_noise(obj.kinematics.initial_pose_with_covariance);
  }
}

void TodDummyPerceptionPubNode::timer_callback()
{
  if (publish_predicted_objects_) {
    auto msg = predicted_base_message_;
    msg.header.stamp = this->get_clock()->now();
    apply_position_noise(msg);
    predicted_pub_->publish(msg);
  } else {
    auto msg = detected_base_message_;
    msg.header.stamp = this->get_clock()->now();
    apply_position_noise(msg);
    detected_pub_->publish(msg);
  }
}

}  // namespace tod_perception_tools

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tod_perception_tools::TodDummyPerceptionPubNode>());
  rclcpp::shutdown();
  return 0;
}
