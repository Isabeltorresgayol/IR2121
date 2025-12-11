#include <chrono>
#include <iostream>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

// ================= CONFIG =================
constexpr double GOAL_TOLERANCE_CLOSE = 0.45; // para puertas estrechas
constexpr double GOAL_TOLERANCE_FAR   = 0.30; // espacios abiertos
constexpr double NEAR_DOOR_DIST       = 1.0;  // distancia para aviso
constexpr double STOP_THRESHOLD       = 0.02; // mínimo movimiento para detectar parada
constexpr double SPEED_THRESHOLD      = 0.05; // velocidad mínima para ajuste de tolerancia

// ---- Función distancia ----
double distance_to(double x1, double y1, double x2, double y2)
{
    return std::hypot(x1 - x2, y1 - y2);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("multi_goal_patrol_tf2_node");

    // ===== Inicializar TF2 =====
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    auto goal_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    // ===== Lista de metas =====
    std::vector<std::pair<double, double>> goals = {
        {8.31, -0.52},
        {3.88, 5.08},
        {-4.25, 0.13},
        {-0.41, 4.43}
    };

    size_t current_goal = 0;
    rclcpp::Time last_publish_time = node->get_clock()->now();

    double prev_x = 0.0, prev_y = 0.0;  // posición previa para velocidad

    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.frame_id = "map";
    goal_msg.pose.orientation.w = 1.0;

    rclcpp::WallRate rate(10);

    RCLCPP_INFO(node->get_logger(), "Patrulla iniciada (tf2, tolerancia dinámica)");

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        if (current_goal >= goals.size())
        {
            RCLCPP_INFO(node->get_logger(), "Todas las metas completadas");
            break;
        }

        // ===== Leer TF2: map -> base_link =====
        double current_x = prev_x;
        double current_y = prev_y;

        try
        {
            auto transform = tf_buffer->lookupTransform(
                "map", "base_link", tf2::TimePointZero);

            current_x = transform.transform.translation.x;
            current_y = transform.transform.translation.y;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(node->get_logger(),
                                 *node->get_clock(), 2000,
                                 "Esperando TF map->base_link: %s", ex.what());
            rate.sleep();
            continue;
        }

        // ===== Calcular movimiento y velocidad =====
        double moved = distance_to(prev_x, prev_y, current_x, current_y);
        double speed = moved * 10.0;
        bool robot_stopped = moved < STOP_THRESHOLD;

        // ===== Publicar goal suavemente =====
        if ((node->get_clock()->now() - last_publish_time).seconds() > 1.0 || robot_stopped)
        {
            goal_msg.header.stamp = node->get_clock()->now();
            goal_msg.pose.position.x = goals[current_goal].first;
            goal_msg.pose.position.y = goals[current_goal].second;
            goal_pub->publish(goal_msg);
            last_publish_time = node->get_clock()->now();
        }

        prev_x = current_x;
        prev_y = current_y;

        // ===== Cálculo de distancia a la meta =====
        double goal_x = goals[current_goal].first;
        double goal_y = goals[current_goal].second;
        double distance = distance_to(current_x, current_y, goal_x, goal_y);

        double tolerance =
            (speed < SPEED_THRESHOLD) ? GOAL_TOLERANCE_CLOSE : GOAL_TOLERANCE_FAR;

        // Aviso cerca de puerta
        if (distance < NEAR_DOOR_DIST && distance >= tolerance)
        {
            RCLCPP_INFO_THROTTLE(node->get_logger(),
                                 *node->get_clock(), 3000,
                                 "Aproximándose a estrechamiento - meta %zu, distancia %.2f m, velocidad %.2f m/s",
                                 current_goal + 1,
                                 distance,
                                 speed);
        }

        // ===== Llegada a meta =====
        if (distance < tolerance)
        {
            RCLCPP_INFO(node->get_logger(),
                        "Meta %zu alcanzada (distancia %.2f m, velocidad %.2f m/s)",
                        current_goal + 1,
                        distance,
                        speed);
            current_goal++;
        }
        else
        {
            RCLCPP_INFO_THROTTLE(node->get_logger(),
                                 *node->get_clock(), 3000,
                                 "Meta %zu - Distancia %.2f m, velocidad %.2f m/s",
                                 current_goal + 1,
                                 distance,
                                 speed);
        }

        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}


