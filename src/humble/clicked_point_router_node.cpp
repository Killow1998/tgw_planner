#include <memory>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "tgw_planner/srv/set_click_mode.hpp"

class ClickedPointRouterNode : public rclcpp::Node
{
public:
  ClickedPointRouterNode()
  : Node("tgw_clicked_point_router_node")
  {
    declare_parameter<std::string>("click_mode", "none");
    mode_ = get_parameter("click_mode").as_string();

    const auto qos = rclcpp::QoS(1).transient_local().reliable();
    start_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/start_point", qos);
    goal_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/goal_point", qos);
    clicked_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", rclcpp::QoS(10),
      std::bind(&ClickedPointRouterNode::onClickedPoint, this, std::placeholders::_1));
    set_mode_srv_ = create_service<tgw_planner::srv::SetClickMode>(
      "/rviz_click_router/set_mode",
      std::bind(
        &ClickedPointRouterNode::handleSetMode, this, std::placeholders::_1,
        std::placeholders::_2));
  }

private:
  void onClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (mode_ == "start") {
      start_pub_->publish(*msg);
    } else if (mode_ == "goal") {
      goal_pub_->publish(*msg);
    }
  }

  void handleSetMode(
    const std::shared_ptr<tgw_planner::srv::SetClickMode::Request> request,
    std::shared_ptr<tgw_planner::srv::SetClickMode::Response> response)
  {
    if (request->mode != "start" && request->mode != "goal" && request->mode != "none") {
      response->success = false;
      response->message = "mode must be start, goal, or none";
      return;
    }
    mode_ = request->mode;
    response->success = true;
    response->message = "mode set to " + mode_;
  }

  std::string mode_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr start_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_sub_;
  rclcpp::Service<tgw_planner::srv::SetClickMode>::SharedPtr set_mode_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::InitOptions init_options;
  init_options.auto_initialize_logging(false);
  rclcpp::init(argc, argv, init_options);
  rclcpp::spin(std::make_shared<ClickedPointRouterNode>());
  rclcpp::shutdown();
  return 0;
}
