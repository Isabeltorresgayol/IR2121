#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PatrollingNode : public rclcpp::Node {
public:
    PatrollingNode() : Node("patrolling_node"), goal_sent_(false) {
        pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        // Timer que se dispara cada 2 segundos
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&PatrollingNode::send_goal, this)
        );
    }

private:
    void send_goal() {
        if (goal_sent_) {
            // Ya enviamos el goal, no hacemos nada
            return;
        }

        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp = this->now();
        goal.header.frame_id = "map";

        // Coordenadas del goal (cambia según tu mapa)
        goal.pose.position.x = 8.32;
        goal.pose.position.y = -0.52;
        goal.pose.position.z = 0.0;

        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = 0.0;
        goal.pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "Enviando goal a (%f, %f)", goal.pose.position.x, goal.pose.position.y);

        pub_->publish(goal);

        // Marcamos que ya enviamos el goal
        goal_sent_ = true;
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool goal_sent_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PatrollingNode>());
    rclcpp::shutdown();
    return 0;
}
