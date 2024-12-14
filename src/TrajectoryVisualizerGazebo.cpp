#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <chrono>

class TrajectoryVisualizer : public rclcpp::Node
{
public:
    TrajectoryVisualizer() : Node("trajectory_visualizer")
    {
        RCLCPP_INFO(this->get_logger(), "Starting...");
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TrajectoryVisualizer::spawn_trajectory_spheres, this)
        );
    }

private:
    void spawn_trajectory_spheres()
    {
        std::string file_path = "/home/daniele/ros2_ws/src/my_bot/worlds/path.txt";
        double scaling_factor = 1.0;

        std::vector<std::vector<double>> trajectory = create_trajectory(file_path);

        for (size_t i = 0; i < trajectory.size(); ++i)
        {
            double x = (trajectory[i][0]) / scaling_factor;
            double y = trajectory[i][1] / scaling_factor;
            double z = 0.0;

            // Call spawn_entity service to spawn a sphere
            spawn_sphere(x, y, z, i);
        }
    }

    void spawn_sphere(double x, double y, double z, size_t index)
    {
        auto service_node = rclcpp::Node::make_shared("spawn_service_node");
        auto spawn_client = service_node->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

        if (!spawn_client->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "SpawnEntity service not available");
            return;
        }

        auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        request->name = "trajectory_sphere_" + std::to_string(index);
        request->xml = R"(
            <sdf version="1.6">
                <model name="{name}">
                    <static>true</static>
                    <link name="link">
                        <visual name="visual">
                            <geometry>
                                <sphere>
                                    <radius>0.02</radius>
                                </sphere>
                            </geometry>
                            <material>
                                <ambient>1.0 0.0 0.0 1.0</ambient>
                                <diffuse>1.0 0.0 0.0 1.0</diffuse> <!-- Red color -->
                                <specular>0.5 0.5 0.5 1.0</specular> <!-- Light reflection -->
                            </material>
                        </visual>
                    </link>
                </model>
            </sdf>
            )";
        request->robot_namespace = "/";
        request->initial_pose.position.x = x;
        request->initial_pose.position.y = y;
        request->initial_pose.position.z = z;
        request->initial_pose.orientation.w = 1.0;

        auto result_future = spawn_client->async_send_request(request);

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(service_node);
        executor.spin_until_future_complete(result_future);

        if (result_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call SpawnEntity service");
            return;
        }

        auto result = result_future.get();
        if (result->success)
        {
            RCLCPP_INFO(this->get_logger(), "Spawned sphere at (%.2f, %.2f, %.2f)", x, y, z);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to spawn sphere: %s", result->status_message.c_str());
        }
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

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryVisualizer>());
    rclcpp::shutdown();
    return 0;
}
