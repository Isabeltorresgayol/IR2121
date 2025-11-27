#include <chrono>
#include <iostream>
#include <cmath>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

// Isabel Torres Gayol
using namespace std::chrono_literals;

// CONFIGURACIÓN ORIGINAL
constexpr double POS_THRESHOLD = 0.20;       // movimiento lineal mínimo
constexpr double ANG_THRESHOLD = 0.087266;   // ~ 5 grados

// VARIABLES DE ESTADO
bool robot_detected_motion = false;
double prev_x = 0.0;
double prev_y = 0.0;
double prev_yaw = 0.0;


// OBTENER YAW DEL CUATERNION
double extract_yaw(const geometry_msgs::msg::Quaternion &q)
{
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}


// CALLBACK DE ODOMETRÍA
void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    double x_now = msg->pose.pose.position.x;
    double y_now = msg->pose.pose.position.y;
    double yaw_now = extract_yaw(msg->pose.pose.orientation);

    double dist = std::hypot(x_now - prev_x, y_now - prev_y);
    double ang_diff = std::fabs(yaw_now - prev_yaw);

    if (ang_diff > M_PI)
        ang_diff = 2 * M_PI - ang_diff;

    robot_detected_motion = (dist > POS_THRESHOLD || ang_diff > ANG_THRESHOLD);

    prev_x = x_now;
    prev_y = y_now;
    prev_yaw = yaw_now;
}


// PROGRAMA PRINCIPAL CON MÚLTIPLES METAS (VERSIÓN ORIGINAL MEJORADA)
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("multi_goal_monitor_node");

    auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, odom_callback);

    auto goal_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10);

    // LISTA DE METAS
    std::vector<std::pair<double, double>> goals = {
        {8.31, -0.52},
        {3.92,  5.10},
        {-4.09, 0.78},
        {-0.27, 4.58}
    };

    size_t current_goal = 0;
    bool goal_sent = false;

    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.frame_id = "map";
    goal_msg.pose.position.z = 0.0;
    goal_msg.pose.orientation.w = 1.0;

    rclcpp::WallRate loop(2s);

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop.sleep();

        if (current_goal >= goals.size())
        {
            RCLCPP_INFO(node->get_logger(), "Todas las metas han sido alcanzadas.");
            break;
        }

        // Enviar meta actual
        if (!goal_sent)
        {
            goal_msg.header.stamp = node->get_clock()->now();
            goal_msg.pose.position.x = goals[current_goal].first;
            goal_msg.pose.position.y = goals[current_goal].second;

            RCLCPP_INFO(node->get_logger(), "Enviando meta %zu -> x: %.2f y: %.2f",
                        current_goal + 1,
                        goal_msg.pose.position.x,
                        goal_msg.pose.position.y);

            goal_pub->publish(goal_msg);
            goal_sent = true;
            continue;
        }

        // DETECCIÓN DE LLEGADA (como tu versión original)
        if (!robot_detected_motion)
        {
            RCLCPP_INFO(node->get_logger(), "Meta %zu alcanzada.", current_goal + 1);
            current_goal++;
            goal_sent = false; // permitir envío de la siguiente meta
        }
        else
        {
            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                "El robot se desplaza hacia la meta %zu...",
                current_goal + 1);
        }
    }

    rclcpp::shutdown();
    return 0;
}
