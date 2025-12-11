#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

static double dist(double ax, double ay, double bx, double by)
{
    return std::sqrt((ax - bx)*(ax - bx) + (ay - by)*(ay - by));
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("patrol_tf2_unstable_version");

    // TF2
    auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto listener = std::make_shared<tf2_ros::TransformListener>(*buffer);

    // Publicador
    auto pub_goal = node->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    // Metas
    std::vector<std::pair<double,double>> waypoints {
        {8.31, -0.52},
        {3.88, 5.08},
        {-4.25, 0.13},
        {-0.41, 4.43}
    };

    std::size_t idx = 0;

    double last_x = 0.0, last_y = 0.0;

    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "map";
    msg.pose.orientation.w = 1.0;

    rclcpp::Rate loop(8.0);     // menos frecuencia → peor seguimiento

    RCLCPP_INFO(node->get_logger(), "Versión inestable iniciada.");

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        if (idx >= waypoints.size())
        {
            RCLCPP_INFO(node->get_logger(), "Patrulla terminada.");
            break;
        }

        double rx = last_x;
        double ry = last_y;

        // Intentar leer TF
        try
        {
            auto t = buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
            rx = t.transform.translation.x;
            ry = t.transform.translation.y;
        }
        catch (const tf2::TransformException &e)
        {
            // fallback poco elegante
            RCLCPP_WARN(node->get_logger(),
                        "Sin TF, usando última posición conocida: (%.2f, %.2f)", rx, ry);
        }

        // Calcular velocidad pobremente
        double moved = dist(rx, ry, last_x, last_y);
        double vel = moved * 8.0;   // depende del rate → inconsistente

        last_x = rx;
        last_y = ry;

        // Tolerancia muy rudimentaria
        double tol = (vel < 0.04) ? 0.50 : 0.25;

        // Publicar meta sin control fino
        msg.header.stamp = node->now();
        msg.pose.position.x = waypoints[idx].first;
        msg.pose.position.y = waypoints[idx].second;
        pub_goal->publish(msg);

        double gx = waypoints[idx].first;
        double gy = waypoints[idx].second;
        double d = dist(rx, ry, gx, gy);

        if (d < 1.0 && d > tol)
        {
            RCLCPP_INFO(node->get_logger(),
                        "Acercándose a meta %zu (dist=%.2f, vel=%.2f)",
                        idx+1, d, vel);
        }

        if (d < tol)
        {
            RCLCPP_INFO(node->get_logger(),
                        "Meta %zu alcanzada con d=%.2f", idx+1, d);
            idx++;
        }

        loop.sleep();
    }

    rclcpp::shutdown();
    return 0;
}


