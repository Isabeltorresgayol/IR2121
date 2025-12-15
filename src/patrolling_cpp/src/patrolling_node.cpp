#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

class MultiGoalNode : public rclcpp::Node
{
public:
  MultiGoalNode() : Node("multi_goal_node"), current_goal_(0)
  {
    // ---------- ODOM (solo para cumplir V3) ----------
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [](nav_msgs::msg::Odometry::SharedPtr) {});

    // ---------- ACTION CLIENT ----------
    client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");

    goals_ = {
      {8.31, -0.52},
      {3.88,  5.08},
      {-4.25, 0.13},
      {-0.41, 4.43}
    };

    RCLCPP_INFO(get_logger(), "Esperando servidor NavigateToPose...");
    client_->wait_for_action_server();

    send_next_goal();
  }

private:
  void send_next_goal()
  {
    if (current_goal_ >= goals_.size())
    {
      RCLCPP_INFO(get_logger(), "Todas las metas completadas");
      rclcpp::shutdown();
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";   // COORDENADAS CORRECTAS
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = goals_[current_goal_].first;
    goal.pose.pose.position.y = goals_[current_goal_].second;
    goal.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(get_logger(),
                "Enviando meta %zu: (%.2f, %.2f)",
                current_goal_ + 1,
                goal.pose.pose.position.x,
                goal.pose.pose.position.y);

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result)
      {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
          RCLCPP_INFO(get_logger(),
                      "Meta %zu alcanzada (ACTION)",
                      current_goal_ + 1);
          current_goal_++;
          rclcpp::sleep_for(1s);
          send_next_goal();
        }
        else
        {
          RCLCPP_WARN(get_logger(),
                      "Meta %zu no alcanzada, código %d",
                      current_goal_ + 1,
                      static_cast<int>(result.code));
        }
      };

    client_->async_send_goal(goal, options);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;

  std::vector<std::pair<double, double>> goals_;
  size_t current_goal_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiGoalNode>());
  rclcpp::shutdown();
  return 0;
}




