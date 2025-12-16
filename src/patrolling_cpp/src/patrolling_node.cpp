#include <chrono>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

using namespace std::chrono_literals;

// ================= PARÁMETROS =================
constexpr double DIST_TOL_CLOSE = 0.45;
constexpr double DIST_TOL_FAR   = 0.30;
constexpr double YAW_TOL        = 0.30;   // tolerancia relajada
constexpr double STOP_EPS       = 0.02;
constexpr double SPEED_EPS      = 0.05;

// ----------------- Utils -----------------
double planarDistance(double x1, double y1, double x2, double y2)
{
    return std::hypot(x1 - x2, y1 - y2);
}

double yawError(double yaw1, double yaw2)
{
    double err = yaw1 - yaw2;
    while (err > M_PI)  err -= 2.0 * M_PI;
    while (err < -M_PI) err += 2.0 * M_PI;
    return std::fabs(err);
}

double quatToYaw(const geometry_msgs::msg::Quaternion &q)
{
    tf2::Quaternion tf_q;
    tf2::fromMsg(q, tf_q);
    return tf2::getYaw(tf_q);
}

// ---------------- Waypoint ----------------
struct Waypoint
{
    double x;
    double y;
    double yaw;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("patrol_waypoints_tf2_yaw_v2");

    auto tf_buffer   = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    auto goal_pub =
        node->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    // ========= WAYPOINTS =========
    std::vector<Waypoint> waypoints = {
        {-3.89,  7.86, -0.00143},
        { 3.42, 16.60, -0.00143},
        {-5.62, 24.30, -0.00143},
        {-12.6, 16.10, -0.00143},
        {-4.57,  0.921, -0.00137}
    };

    size_t wp_index = 0;
    rclcpp::Time last_goal_sent = node->get_clock()->now();

    double last_x = 0.0;
    double last_y = 0.0;

    bool position_reached = false;

    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.frame_id = "map";

    rclcpp::WallRate loop_rate(10);

    RCLCPP_INFO(node->get_logger(), "Patrulla iniciada (2 fases: posición + yaw)");

    while (rclcpp::ok() && wp_index < waypoints.size())
    {
        rclcpp::spin_some(node);

        double rx, ry, ryaw;

        try
        {
            auto tf = tf_buffer->lookupTransform(
                "map", "base_link", tf2::TimePointZero);

            rx   = tf.transform.translation.x;
            ry   = tf.transform.translation.y;
            ryaw = quatToYaw(tf.transform.rotation);
        }
        catch (tf2::TransformException &e)
        {
            RCLCPP_WARN_THROTTLE(
                node->get_logger(), *node->get_clock(), 2000,
                "Esperando TF map->base_link");
            loop_rate.sleep();
            continue;
        }

        double delta_move = planarDistance(last_x, last_y, rx, ry);
        double est_speed  = delta_move * 10.0;
        bool stopped      = delta_move < STOP_EPS;

        // ---- Publicar goal periódicamente ----
        if ((node->get_clock()->now() - last_goal_sent).seconds() > 1.0 || stopped)
        {
            const auto &wp = waypoints[wp_index];

            goal_msg.header.stamp = node->get_clock()->now();
            goal_msg.pose.position.x = wp.x;
            goal_msg.pose.position.y = wp.y;

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, wp.yaw);
            goal_msg.pose.orientation = tf2::toMsg(q);

            goal_pub->publish(goal_msg);
            last_goal_sent = node->get_clock()->now();
        }

        last_x = rx;
        last_y = ry;

        const auto &wp = waypoints[wp_index];

        double dist_err = planarDistance(rx, ry, wp.x, wp.y);
        double yaw_err  = yawError(ryaw, wp.yaw);

        double dist_tol =
            (est_speed < SPEED_EPS) ? DIST_TOL_CLOSE : DIST_TOL_FAR;

        // -------- FASE 1: posición --------
        if (!position_reached && dist_err < dist_tol)
        {
            position_reached = true;
            RCLCPP_INFO(node->get_logger(),
                        "WP %zu: posición alcanzada, ajustando orientación...",
                        wp_index + 1);
        }

        // -------- FASE 2: yaw --------
        if (position_reached && yaw_err < YAW_TOL)
        {
            RCLCPP_INFO(node->get_logger(),
                        "Waypoint %zu COMPLETADO (dist=%.2f, yaw=%.2f)",
                        wp_index + 1, dist_err, yaw_err);

            position_reached = false;
            wp_index++;
        }

        loop_rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "Ruta finalizada correctamente");
    rclcpp::shutdown();
    return 0;
}


