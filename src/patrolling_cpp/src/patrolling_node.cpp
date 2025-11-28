#include <chrono>
#include <iostream>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using namespace std::chrono_literals;

// ================= CONFIGURACIÓN =================
constexpr double GOAL_TOLERANCE = 0.35;   // Distancia real para considerar llegada

// ================= POSICIÓN GLOBAL DEL ROBOT (map) =================
double current_x = 0.0;
double current_y = 0.0;

// ================= PROGRAMA PRINCIPAL =================
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("multi_goal_monitor_node");

    // ✅ Posición real del robot en el mapa (AMCL)
    auto amcl_sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose",
        10,
        [](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
        {
            current_x = msg->pose.pose.position.x;
            current_y = msg->pose.pose.position.y;
        });

    // ✅ QoS correcto para comportamiento tipo RViz
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos.transient_local();
    qos.reliable();

    auto goal_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", qos);

    // ================= LISTA DE METAS =================
    std::vector<std::pair<double, double>> goals = {
        {8.31, -0.52},
        {3.88, 5.08},
        {-4.25, 0.13},
        {-0.41, 4.43}
    };

    size_t current_goal = 0;

    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.frame_id = "map";
    goal_msg.pose.position.z = 0.0;
    goal_msg.pose.orientation.w = 1.0;

    rclcpp::WallRate loop(5);

    RCLCPP_INFO(node->get_logger(), "🚀 Sistema de patrullaje iniciado correctamente");

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop.sleep();

        if (current_goal >= goals.size())
        {
            RCLCPP_INFO(node->get_logger(), "✅ Todas las metas han sido alcanzadas.");
            break;
        }

        // ===== PUBLICAR META DE FORMA CONTINUA =====
        goal_msg.header.stamp = node->get_clock()->now();
        goal_msg.pose.position.x = goals[current_goal].first;
        goal_msg.pose.position.y = goals[current_goal].second;
        goal_pub->publish(goal_msg);

        // ===== CALCULAR DISTANCIA REAL A META =====
        double goal_x = goals[current_goal].first;
        double goal_y = goals[current_goal].second;

        double distance = std::hypot(goal_x - current_x,
                                     goal_y - current_y);

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
                                 2000,
                                 "🚗 Yendo a meta %zu - Distancia %.2f m",
                                 current_goal + 1,
                                 distance);
        }
    }

    rclcpp::shutdown();
    return 0;
}

