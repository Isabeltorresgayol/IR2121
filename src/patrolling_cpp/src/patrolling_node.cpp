#include <chrono>
#include <iostream>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using namespace std::chrono_literals;

// ================= CONFIG =================
constexpr double GOAL_TOLERANCE = 0.45;   // mayor tolerancia para puertas estrechas
constexpr double NEAR_DOOR_DIST = 0.7;    // distancia para aviso de estrechamiento

double current_x = 0.0;
double current_y = 0.0;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("multi_goal_patrol_node");

    // Posición global del robot
    auto amcl_sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        [](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
        {
            current_x = msg->pose.pose.position.x;
            current_y = msg->pose.pose.position.y;
        });

    auto goal_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    // Lista de metas
    std::vector<std::pair<double, double>> goals = {
        {8.31, -0.52},
        {3.88, 5.08},
        {-4.25, 0.13},
        {-0.41, 4.43}
    };

    size_t current_goal = 0;
    rclcpp::Time last_publish_time = node->get_clock()->now();

    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.frame_id = "map";
    goal_msg.pose.orientation.w = 1.0;

    rclcpp::WallRate rate(10);

    RCLCPP_INFO(node->get_logger(), "🚀 Patrulla iniciada con ajuste para puertas");

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        if (current_goal >= goals.size())
        {
            RCLCPP_INFO(node->get_logger(), "✅ Todas las metas completadas");
            break;
        }

        // ===== ENVÍO SUAVE DEL GOAL =====
        if ((node->get_clock()->now() - last_publish_time).seconds() > 1.0)
        {
            goal_msg.header.stamp = node->get_clock()->now();
            goal_msg.pose.position.x = goals[current_goal].first;
            goal_msg.pose.position.y = goals[current_goal].second;
            goal_pub->publish(goal_msg);
            last_publish_time = node->get_clock()->now();
        }

        // ===== DISTANCIA REAL A META =====
        double goal_x = goals[current_goal].first;
        double goal_y = goals[current_goal].second;
        double distance = std::hypot(goal_x - current_x, goal_y - current_y);

        // Aviso si está cerca de una puerta/estrechamiento
        if (distance < NEAR_DOOR_DIST && distance >= GOAL_TOLERANCE)
        {
            RCLCPP_INFO_THROTTLE(node->get_logger(),
                                 *node->get_clock(),
                                 3000,
                                 "⚠️ Aproximándose a estrechamiento (distancia %.2f m) - meta %zu",
                                 distance,
                                 current_goal + 1);
        }

        // Llegada a meta
        if (distance < GOAL_TOLERANCE)
        {
            RCLCPP_INFO(node->get_logger(),
                        "✅ Meta %zu alcanzada (distancia %.2f m)",
                        current_goal + 1,
                        distance);
            current_goal++;
        }
        else
        {
            RCLCPP_INFO_THROTTLE(node->get_logger(),
                                 *node->get_clock(),
                                 3000,
                                 "🚗 Meta %zu - Distancia %.2f m",
                                 current_goal + 1,
                                 distance);
        }

        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

