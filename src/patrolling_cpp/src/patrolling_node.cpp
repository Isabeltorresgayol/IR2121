#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <vector>
#include <array>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

// Parámetros
const double GOAL_TOLERANCE = 0.4;   // Tolerancia para considerar llegada
const int NAV_START_DELAY = 2;        // s

// Variables globales AMCL
double amcl_x = 0.0;
double amcl_y = 0.0;
bool amcl_recibido = false;

// Callback AMCL
void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    amcl_x = msg->pose.pose.position.x;
    amcl_y = msg->pose.pose.position.y;
    amcl_recibido = true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("goal_publisher_v4_amcl");

    auto publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10);

    auto amcl_sub = node->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose",
        10,
        amclCallback);

    // Waypoints (x, y, z)
    std::vector<std::array<double, 3>> waypoints = {
        {-3.89,  7.86, -0.00143},
        { 3.42, 16.60, -0.00143},
        {-5.62, 24.30, -0.00143},
        {-12.6, 16.10, -0.00143},
        {-4.57,  0.921, -0.00137}
    };

    // Esperar arranque de Nav2 / AMCL
    rclcpp::sleep_for(std::chrono::seconds(NAV_START_DELAY));

    RCLCPP_INFO(node->get_logger(),
                "Iniciando versión 4: detección de llegada basada en AMCL");

    rclcpp::Rate rate(5);

    // --- BUCLE DE WAYPOINTS ---
    for (size_t i = 0; i < waypoints.size() && rclcpp::ok(); i++)
    {
        auto &wp = waypoints[i];

        // Crear objetivo
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = node->get_clock()->now();

        goal.pose.position.x = wp[0];
        goal.pose.position.y = wp[1];
        goal.pose.position.z = wp[2];

        // Orientación (yaw = 0 rad)
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = 0.0;
        goal.pose.orientation.w = 1.0;

        // Enviar objetivo
        publisher->publish(goal);
        RCLCPP_INFO(node->get_logger(),
                    "Objetivo %zu enviado → (%.3f, %.3f, %.3f)",
                    i + 1, wp[0], wp[1], wp[2]);

        // --- Esperar llegada usando AMCL ---
        amcl_recibido = false;

        while (rclcpp::ok())
        {
            rclcpp::spin_some(node);

            if (!amcl_recibido)
            {
                RCLCPP_WARN_THROTTLE(
                    node->get_logger(),
                    *node->get_clock(),
                    2000,
                    "Esperando pose de AMCL...");
                rate.sleep();
                continue;
            }

            double dx = wp[0] - amcl_x;
            double dy = wp[1] - amcl_y;
            double distancia = std::hypot(dx, dy);

            RCLCPP_INFO(node->get_logger(),
                        "Distancia AMCL al objetivo %zu: %.3f m",
                        i + 1, distancia);

            if (distancia < GOAL_TOLERANCE)
            {
                RCLCPP_INFO(node->get_logger(),
                            "Llegada al objetivo %zu (basado en AMCL)",
                            i + 1);
                break;
            }

            rate.sleep();
        }

        rclcpp::sleep_for(500ms);
    }

    RCLCPP_INFO(node->get_logger(),
                "Patrulla completada (versión 4).");

    rclcpp::shutdown();
    return 0;
}

