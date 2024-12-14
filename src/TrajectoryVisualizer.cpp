#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <chrono>

class TrajectoryVisualizer : public rclcpp::Node
{
public:
    TrajectoryVisualizer()
    : Node("trajectory_visualizer")
    {
        RCLCPP_INFO(this->get_logger(), "Starting...");
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("trajectory_marker", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TrajectoryVisualizer::publish_marker, this)
        );
    }

private:
    void publish_marker()
    {
        std::string file_path = "/home/daniele/ros2_ws/src/my_bot/data/path.txt";
        double scaling_factor = 0.5;

        std::vector<std::vector<double>> trajectory = create_trajectory(file_path);

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = this->get_clock()->now();
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;  // or Marker::SPHERE_LIST for individual points
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.1;  // Line width
        marker.color.a = 1.0;  // Alpha
        marker.color.r = 1.0;  // Red
        marker.color.g = 0.0;  // Green
        marker.color.b = 0.0;  // Blue

        // Add trajectory points to the marker
        for (const auto& point : trajectory)
        {
            geometry_msgs::msg::Point p;
            p.x = (point[0]) * scaling_factor;
            p.y = point[1] * scaling_factor ;
            p.z = 0.0;  // Flat trajectory
            marker.points.push_back(p);
        }

        publisher_->publish(marker);
        RCLCPP_INFO(this->get_logger(), "Published trajectory marker");
    }

    std::vector<std::vector<double>> create_trajectory(const std::string& file_path)
    {
        RCLCPP_INFO(this->get_logger(), "Parsing Function called");
        std::vector<std::vector<double>> data;

        std::ifstream file(file_path);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path.c_str());
            return data;
        }

        std::string line;
        while (std::getline(file, line))
        {
            std::istringstream ss(line);
            double x, y;
            if (ss >> x >> y)
            {
                data.push_back({x, y});
            }
        }

        file.close();
        RCLCPP_INFO(this->get_logger(), "File correctly parsed");

        return data;
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryVisualizer>());
    rclcpp::shutdown();
    return 0;
}