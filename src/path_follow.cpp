#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
#include <functional>

using namespace std;

class PathFollow : public rclcpp::Node {
public:
    PathFollow() : Node("path_follow"), current_pose_{0.0, 0.0, 0.0}, lookahead_steps_(3) {
        // Publishers and subscribers
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pose_subscriber_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 10, bind(&PathFollow::poseCallback, this, placeholders::_1));
        marker_subscriber_ = this->create_subscription<visualization_msgs::msg::Marker>(
            "/trajectory_marker", 10, bind(&PathFollow::markerCallback, this, placeholders::_1));

        control_timer_ = this->create_wall_timer(
            chrono::milliseconds(100), bind(&PathFollow::controlCallback, this));

        RCLCPP_INFO(this->get_logger(), "Path control has been started.");
    }

private:
    struct Pose {
        double x;
        double y;
        double yaw;
    };

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_subscriber_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    Pose current_pose_;
    vector<pair<double, double>> path_;
    int lookahead_steps_;

    void poseCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        for (const auto &transform : msg->transforms) {
            if (transform.header.frame_id == "odom" && transform.child_frame_id == "base_link") {
                double x = transform.transform.translation.x;
                double y = transform.transform.translation.y;

                double qx = transform.transform.rotation.x;
                double qy = transform.transform.rotation.y;
                double qz = transform.transform.rotation.z;
                double qw = transform.transform.rotation.w;

                double yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
                current_pose_ = {x, y, yaw};

                RCLCPP_INFO(this->get_logger(), "Updated Pose: (%.2f, %.2f), Yaw: %.4f radians", x, y, yaw);
            }
        }
    }

    void markerCallback(const visualization_msgs::msg::Marker::SharedPtr msg) {
        path_.clear();
        for (const auto &point : msg->points) {
            path_.emplace_back(point.x, point.y);
        }
        RCLCPP_INFO(this->get_logger(), "Received trajectory with %lu points.", path_.size());
    }

    void controlCallback() {
        if (path_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Path not available, skipping control.");
            return;
        }

        // Extract robot's current pose
        double x = current_pose_.x;
        double y = current_pose_.y;
        double yaw = current_pose_.yaw;

        // Find the closest waypoint in the path
        auto closest_it = std::min_element(path_.begin(), path_.end(), [&](const auto &wp1, const auto &wp2) {
            double dist1 = std::hypot(wp1.first - x, wp1.second - y);
            double dist2 = std::hypot(wp2.first - x, wp2.second - y);
            return dist1 < dist2;
        });

        if (closest_it == path_.end()) {
            RCLCPP_WARN(this->get_logger(), "Failed to find closest waypoint.");
            return;
        }

        int closest_index = distance(path_.begin(), closest_it);
        int lookahead_index = min(closest_index + lookahead_steps_, static_cast<int>(path_.size() - 1));

        auto [goal_x, goal_y] = path_[lookahead_index];

        // Control calculation
        double distance = hypot(goal_x - x, goal_y - y);
        double angle_to_goal = atan2(goal_y - y, goal_x - x);
        double angle_error = angle_to_goal - yaw;

        double goal_threshold = 0.3;

        // Normalize angle error to [-pi, pi]
        angle_error = fmod(angle_error + M_PI, 2 * M_PI);
        if (angle_error < 0) {
            angle_error += 2 * M_PI;
        }
        angle_error -= M_PI;

        int sign_angle = 0;

        if (angle_error >= 0){
            sign_angle = 1;
        }else if( angle_error < 0){
            sign_angle = -1;
        }

        double k_linear = 0.5;
        double k_angular = 0.8;
        double k_e = 0.8;
        if(abs(angle_error) <= 0.1 ){
            k_e = 0;
        }else{
            k_e = 0.8;
        }

        double linear_velocity = k_linear * distance;
        double angular_velocity = k_angular * (angle_error + sign_angle*std::atan(k_e * distance / (linear_velocity + 1e-5)));

        // Publish control commands
        auto cmd = geometry_msgs::msg::Twist();

        if(distance < goal_threshold){
            linear_velocity = 0;
            angular_velocity = 0;
            RCLCPP_INFO(this->get_logger(),"ROBOT ARRIVED");
        }

        cmd.linear.x = linear_velocity;
        cmd.angular.z = angular_velocity;
        cmd_vel_publisher_->publish(cmd);

        RCLCPP_INFO(this->get_logger(), "Control: Linear=%.2f, Angular=%.2f", linear_velocity, angular_velocity);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = make_shared<PathFollow>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
