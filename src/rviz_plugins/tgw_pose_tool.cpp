#include <memory>
#include <string>

#include <QEvent>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/qos.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_default_plugins/tools/pose/pose_tool.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace tgw_planner::rviz_plugins
{

class Tgw3DPoseTool : public rviz_default_plugins::tools::PoseTool
{
public:
  Tgw3DPoseTool(std::string topic, std::string label, double default_z)
  : topic_(std::move(topic)), label_(std::move(label)), current_z_(default_z)
  {
  }

  void onInitialize() override
  {
    PoseTool::onInitialize();
    z_property_ = new rviz_common::properties::FloatProperty(
      "Z Height", static_cast<float>(current_z_),
      "Height used by this 3D pose tool. Mouse wheel updates this value.",
      getPropertyContainer());
    z_property_->setMin(-50.0F);
    z_property_->setMax(100.0F);
    auto node_abstraction = context_->getRosNodeAbstraction().lock();
    if (!node_abstraction) {
      setStatus("No RViz ROS node available");
      return;
    }
    node_ = node_abstraction->get_raw_node();
    publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      topic_, rclcpp::QoS(1).transient_local().reliable());
    preview_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "/tgw_pose_tool_preview", rclcpp::QoS(10).reliable());
    updateStatus();
  }

  void deactivate() override
  {
    clearPreview();
    PoseTool::deactivate();
  }

  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override
  {
    if (event.type == QEvent::Wheel) {
      const double direction = event.wheel_delta >= 0 ? 1.0 : -1.0;
      current_z_ += direction * z_step_m_;
      if (z_property_) {
        z_property_->setFloat(static_cast<float>(current_z_));
      }
      updateStatus();
      updateArrowHeight();
      publishPreview();
      return Render;
    }
    const int result = PoseTool::processMouseEvent(event);
    updateArrowHeight();
    publishPreview();
    return result;
  }

protected:
  void onPoseSet(double x, double y, double theta) override
  {
    if (!publisher_ || !node_) {
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = node_->now();
    pose.header.frame_id = context_->getFixedFrame().toStdString();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = current_z_;
    pose.pose.orientation = orientationAroundZAxis(theta);
    publisher_->publish(pose);
    clearPreview();
    updateStatus();
  }

private:
  static geometry_msgs::msg::Point makePoint(double x, double y, double z)
  {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  }

  void publishPreview()
  {
    if (!preview_pub_ || !node_ || !context_) {
      return;
    }

    const auto frame = context_->getFixedFrame().toStdString();
    const double x = arrow_position_.x;
    const double y = arrow_position_.y;
    const double ground_z = 0.0;
    const double marker_z = current_z_;
    const bool is_goal = topic_.find("goal") != std::string::npos;
    const float r = is_goal ? 1.0F : 0.0F;
    const float g = is_goal ? 0.82F : 0.95F;
    const float b = is_goal ? 0.0F : 1.0F;

    visualization_msgs::msg::Marker line;
    line.header.stamp = node_->now();
    line.header.frame_id = frame;
    line.ns = "tgw_pose_preview";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_LIST;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.08;
    line.color.r = r;
    line.color.g = g;
    line.color.b = b;
    line.color.a = 1.0;
    line.points.push_back(makePoint(x, y, ground_z));
    line.points.push_back(makePoint(x, y, marker_z));
    preview_pub_->publish(line);

    visualization_msgs::msg::Marker text = line;
    text.id = 1;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.points.clear();
    text.pose.position = makePoint(x, y, marker_z + 0.75);
    text.scale.z = 0.45;
    text.text = label_ + " z=" + std::to_string(marker_z).substr(0, 4) + "m";
    preview_pub_->publish(text);
  }

  void clearPreview()
  {
    if (!preview_pub_ || !node_ || !context_) {
      return;
    }
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = node_->now();
    marker.header.frame_id = context_->getFixedFrame().toStdString();
    marker.ns = "tgw_pose_preview";
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    preview_pub_->publish(marker);
  }

  void updateArrowHeight()
  {
    if (!arrow_) {
      return;
    }
    Ogre::Vector3 elevated_position = arrow_position_;
    elevated_position.z = static_cast<float>(current_z_);
    arrow_->setPosition(elevated_position);
  }

  void updateStatus()
  {
    setStatus(
      QString("%1 z=%2 m, mouse wheel changes z by %3 m")
        .arg(QString::fromStdString(label_))
        .arg(current_z_, 0, 'f', 2)
        .arg(z_step_m_, 0, 'f', 2));
  }

  std::string topic_;
  std::string label_;
  double current_z_{0.0};
  double z_step_m_{0.10};
  rviz_common::properties::FloatProperty * z_property_{nullptr};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr preview_pub_;
};

class Tgw3DStartTool : public Tgw3DPoseTool
{
public:
  Tgw3DStartTool()
  : Tgw3DPoseTool("/start_pose", "Tgw 3D Start", 3.5)
  {
    shortcut_key_ = 's';
  }
};

class Tgw3DGoalTool : public Tgw3DPoseTool
{
public:
  Tgw3DGoalTool()
  : Tgw3DPoseTool("/goal_pose", "Tgw 3D Goal", 3.5)
  {
    shortcut_key_ = 'g';
  }
};

}  // namespace tgw_planner::rviz_plugins

PLUGINLIB_EXPORT_CLASS(tgw_planner::rviz_plugins::Tgw3DStartTool, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(tgw_planner::rviz_plugins::Tgw3DGoalTool, rviz_common::Tool)
