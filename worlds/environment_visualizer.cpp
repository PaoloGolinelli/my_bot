#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"
#include <thread>
#include <chrono>
#include <vector>
#include <string>

struct Box {
    std::string name;
    double x, y, z;
    double size_x, size_y, size_z;
};

class BoxSpawner : public rclcpp::Node {
public:
    BoxSpawner() : Node("box_spawner") {
        spawn_client_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
        delete_client_ = this->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");
    }

    void spawnBoxes(const std::vector<Box> &boxes) {
        for (const auto &box : boxes) {
            auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
            request->name = box.name;
            request->xml = createBoxSDF(box);
            request->initial_pose.position.x = box.x;
            request->initial_pose.position.y = box.y;
            request->initial_pose.position.z = box.z;

            while (!spawn_client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_INFO(this->get_logger(), "Waiting for /spawn_entity service...");
            }

            auto result = spawn_client_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Box '%s' spawned successfully!", box.name.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to spawn box '%s'.", box.name.c_str());
            }
        }
    }

    void despawnBoxes(const std::vector<std::string> &box_names) {
        for (const auto &name : box_names) {
            auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
            request->name = name;

            while (!delete_client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_INFO(this->get_logger(), "Waiting for /delete_entity service...");
            }

            auto result = delete_client_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Box '%s' despawned successfully!", name.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to despawn box '%s'.", name.c_str());
            }
        }
    }

private:
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client_;
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client_;

    std::string createBoxSDF(const Box &box) {
        return R"(
            <sdf version="1.6">
              <model name=")" + box.name + R"(">
                <static>true</static>
                <link name="link">
                  <collision name="collision">
                    <geometry>
                      <box>
                        <size>)" + std::to_string(box.size_x) + " " + std::to_string(box.size_y) + " " + std::to_string(box.size_z) + R"(</size>
                      </box>
                    </geometry>
                  </collision>
                  <visual name="visual">
                    <geometry>
                      <box>
                        <size>)" + std::to_string(box.size_x) + " " + std::to_string(box.size_y) + " " + std::to_string(box.size_z) + R"(</size>
                      </box>
                    </geometry>
                  </visual>
                </link>
              </model>
            </sdf>
        )";
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<BoxSpawner>();

    // Define a list of boxes
    std::vector<Box> boxes = {
        {"box_1", 0.0, 0.0, 0.5, 1.0, 1.0, 1.0},
        {"box_2", 2.0, 2.0, 0.5, 1.5, 1.0, 0.5},
        {"box_3", -2.0, -2.0, 0.5, 0.8, 0.8, 0.8}
    };

    std::vector<std::string> box_names;
    for (const auto &box : boxes) {
        box_names.push_back(box.name);
    }

    // Spawn boxes
    node->spawnBoxes(boxes);

    // Wait for a few seconds
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Despawn boxes
    node->despawnBoxes(box_names);

    // Wait for a few seconds
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Respawn boxes
    node->spawnBoxes(boxes);

    rclcpp::shutdown();
    return 0;
}
