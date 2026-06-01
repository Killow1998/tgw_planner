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
    updateStatus();
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
      return Render;
    }
    return PoseTool::processMouseEvent(event);
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
    updateStatus();
  }

private:
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
