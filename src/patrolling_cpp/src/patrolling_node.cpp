#include <chrono>
#include <iostream>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

//Isabel Torres Gayol. He utilizado la IA para entender la lógica de cómo poder lanzar el mensaje de llegada. Lo justifico todo en el pdf.

using namespace std::chrono_literals;

// CONFI
constexpr double POS_THRESHOLD = 0.10;       // movimiento lineal mínimo
constexpr double ANG_THRESHOLD = 0.087266;   // ~ 5 grados

// VARIABLES DE ESTADO

bool robot_detected_motion = false;
bool first_goal_sent = false;

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

    // corregir salto angular
    if (ang_diff > M_PI)
        ang_diff = 2 * M_PI - ang_diff;

    // detección de movimiento
    robot_detected_motion = (dist > POS_THRESHOLD || ang_diff > ANG_THRESHOLD);

    // actualizar referencias
    prev_x = x_now;
    prev_y = y_now;
    prev_yaw = yaw_now;
}


// PROGRAMA PRINCIPAL - VERSIÓN DIFERENTE PERO MISMO COMPORTAMIENTO

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("goal_monitor_node");

    // suscriptor de odometría
    auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, odom_callback);

    // publicador de goal
    auto goal_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10);

    // definir objetivo
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = 8.31;
    goal.pose.position.y = -0.52;
    goal.pose.position.z = 0.0;
    goal.pose.orientation.w = 1.0;

    rclcpp::WallRate loop(2s);

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop.sleep();

        if (!first_goal_sent)
        {
            goal.header.stamp = node->get_clock()->now();
            RCLCPP_INFO(node->get_logger(), "Enviando objetivo inicial...");
            goal_pub->publish(goal);
            first_goal_sent = true;
            continue;
        }

        // cuando ya enviamos goal
        if (!robot_detected_motion)
        {
            RCLCPP_INFO(node->get_logger(), "El robot ha alcanzado el objetivo.");
            break;
        }
        else
        {
            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                "El robot sigue desplazándose hacia el objetivo...");
        }
    }

    rclcpp::shutdown();
    return 0;
}
