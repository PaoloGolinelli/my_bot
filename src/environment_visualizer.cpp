#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <gazebo_msgs/srv/delete_entity.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <filesystem>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>

using namespace std;
namespace fs = std::filesystem;

class EnvironmentVisualizer : public rclcpp::Node {
public:
    EnvironmentVisualizer()
        : Node("environment_visualizer"), counter_(0){
        spawn_client_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
        delete_client_ = this->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");

        // Timer to execute update logic every 10 seconds
        timer_ = this->create_wall_timer(
            chrono::milliseconds(500), bind(&EnvironmentVisualizer::update_boxes, this));

        update_boxes();
    }

private:

    double scaling_factor;
    fs::file_time_type lastWriteTime;
    bool first = true;

    // struct containing the informations of a block made by consecutive equal height pixels
    struct block { 
        int idx_b;      // beginning index
        int idx_f;      // ending index
        int height;     // height of consecutive pixels
    };

    // struct containing informations of a box (obstacle)
    struct Box {
        string name;
        geometry_msgs::msg::Pose pose;
        array<double, 3> size; // width, length, height
    };

    vector<Box> box_list_;
    vector<pair<double, double>> trajectory_;
    size_t counter_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client_;
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client_;

    void update_boxes() {
        string file_name = "data/input_mat.txt"; // Path relative to the package share directory

        // get the time the input file has last been modified 
        fs::file_time_type currentWriteTime = fs::last_write_time("srs/my_bot/data/input_mat.txt");

        // if the time changed it updates the visualization
        if (currentWriteTime != lastWriteTime) {
            if (first) {    // the first time sleeps for two seconds in order to allow spawn_entity to start
                std::this_thread::sleep_for(std::chrono::seconds(10));
                first = false;
            }
            lastWriteTime = currentWriteTime;

            RCLCPP_INFO(this->get_logger(), "The map has been modified! Updating visualization...");

            // compute the new visualization
            delete_unified_model();
            update_box_list(file_name);
            // PATH PLANNING ALGORITHM CALLED HERE
            load_trajectory();
            spawn_unified_model();
        }
    }

    void update_box_list(string file_name) {
        box_list_.clear();

        string package_name = "my_bot";
        string package_path = ament_index_cpp::get_package_share_directory(package_name);
        string full_path = package_path + "/" + file_name;

        ifstream Ifile(full_path);
        if (!Ifile.is_open()) {
            cerr << "Error: Could not open file " << full_path << endl;
        } else {
            cout << "File opened successfully: " << full_path << endl;
        }

        // Read the header
        int m_width, m_height;
        double block_L_scale, block_H_scale;
        Ifile >> m_width;           // read matrix's width
        Ifile >> m_height;          // read matrix's height
        Ifile >> block_L_scale;     // dimension of one cell of the matrix
        Ifile >> block_H_scale;     // dimension of one cell of the matrix

        scaling_factor = block_L_scale;

        // Compress the rows
        // initialize support variables
        int value, prev_value;  // values read element from .txt file
        string line;            // support variable to read a line of .txt
        block blk_tmp;          // support variable to contain info on current parsed block

        // initialize variables containing the parsed matrix  
        vector<block> parsed_row;                   // contains all blocks of one row
        vector<vector<block>> parsed_mat;           // contains all parsed rows

        // Read the matrix from the file
        while (getline(Ifile, line)) {              // repeat over all rows
            istringstream iss(line);

            prev_value = 0;
            for (int i = 0; iss >> value; i++) {    // repeat over all elements in the row
                
                // if it detects a change in height -> new block
                if (value != prev_value) {          
                    // the block is saved only if it's non-zero
                    if (prev_value != 0) {
                        blk_tmp.idx_f = i-1;
                        blk_tmp.height = prev_value;

                        // save block into row_vector
                        parsed_row.push_back(blk_tmp);
                    }
                    if (value != 0)
                        blk_tmp.idx_b = i;
                }

                prev_value = value;
            }

            // if object ends at the right border
            if (prev_value != 0) {
                blk_tmp.idx_f = m_width-1;
                blk_tmp.height = prev_value;

                // save block into row_vector
                parsed_row.push_back(blk_tmp);
            }

            // save parsed row into the matrix
            parsed_mat.push_back(parsed_row);
        }

        // Close the file
        Ifile.close();

        // Extract boxes
        block blk_comp;
        bool affinity;
        Box box_pars;
        double x1, x2, y1, y2, height;

        for (size_t r = 0; r < parsed_mat.size(); r++) {    // iterate over all parsed rows      
            for (size_t b = parsed_mat[r-1].size(); b < parsed_mat[r].size(); b++) { // iterate over all blocks in the row
                
                // extract current block
                blk_tmp = parsed_mat[r][b];
                if (blk_tmp.height == -1)           // skips the block if it is already been parsed in a box
                    continue;

                // initial parsing of the box
                x1 = blk_tmp.idx_b;
                x2 = blk_tmp.idx_f;
                y1 = r-1;
                y2 = r-1;                  // at fist box is just a horizontal line
                height = blk_tmp.height;

                for (size_t nl = r+1; nl < parsed_mat.size(); nl++) { // iterate over next lines
                    
                    affinity = false;
                    for (size_t nb = parsed_mat[nl-1].size(); nb < parsed_mat[nl].size(); nb++) { // iterate over blocks
                        blk_comp = parsed_mat[nl][nb];
                        
                        // check if the blocks are affine, aka: same idxs and height
                        // if the starting idx of the compare block is beyond the starting idx of current block 
                        if (blk_tmp.idx_b < blk_comp.idx_b) { // no affinity found -> box terminated
                            break;
                        }
                        if (checkBlocks(blk_tmp, blk_comp)) {
                            // remove the element just found
                            parsed_mat[nl][nb].height = -1;   // alternative to removing block
                            affinity = true;
                            y2 = nl-1;              // update box height
                            break;                  // continune to check next row
                        }
                    }

                    if (!affinity) { // box terminated
                        // Scale the box by the dimension of the grid
                        x1 *= block_L_scale;
                        y1 *= block_H_scale;
                        x2 = (x2 + 1) * block_L_scale;
                        y2 = (y2 + 1) * block_H_scale;
                        height *= (block_L_scale+block_H_scale)/2.0;

                        box_pars.name = "box_" + to_string(counter_++);
                        box_pars.pose.position.x = (x1 + x2) / 2.0;
                        box_pars.pose.position.y = (y1 + y2) / 2.0;
                        box_pars.pose.position.z = height / 2.0;
                        box_pars.size = {abs(x2-x1), abs(y2-y1), height};
                        box_list_.push_back(box_pars);
                        break;
                    }
                }
            }
        }
    }

    void load_trajectory() {
        trajectory_.clear();
        string package_name = "my_bot";
        string file_name = "data/path.txt"; // Path relative to the package share directory

        string package_path = ament_index_cpp::get_package_share_directory(package_name);
        string full_path = package_path + "/" + file_name;

        ifstream Ifile(full_path);
        if (!Ifile.is_open()) {
            cerr << "Error: Could not open file " << full_path << endl;
            return;
        }

        double x, y;
        while (Ifile >> x >> y) {
            trajectory_.emplace_back(x*scaling_factor, y*scaling_factor);
        }
        Ifile.close();
    }

    void spawn_unified_model() {
        auto request = ::std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        request->name = "unified_boxes_model";
        request->xml = generate_unified_sdf(box_list_, trajectory_);
        request->robot_namespace = "";
        request->initial_pose = geometry_msgs::msg::Pose(); // Default origin
        request->reference_frame = "world";

        spawn_client_->async_send_request(request, [this](rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedFuture future) {
            try {
                if (future.get()) {
                    RCLCPP_INFO(this->get_logger(), "Spawned unified model successfully.");
                }
            } catch (const exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to spawn unified model: %s", e.what());
            }
        });
    }

    void delete_unified_model() {
        auto request = ::std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
        request->name = "unified_boxes_model";

        delete_client_->async_send_request(request, [this](rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedFuture future) {
            try {
                if (future.get()) {
                    RCLCPP_INFO(this->get_logger(), "Deleted unified model successfully.");
                }
            } catch (const exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to delete unified model: %s", e.what());
            }
        });
    }

    string generate_unified_sdf(const vector<Box>& boxes, const vector<pair<double, double>>& trajectory) const {
        string sdf = R"(
        <sdf version='1.6'>
            <model name='unified_boxes_model'>
        )";

        for (size_t i = 0; i < boxes.size(); ++i) {
            const auto& box = boxes[i];
            sdf += R"(
            <link name='box_)" + to_string(i) + R"('>
                <visual name='visual'>
                    <geometry>
                        <box>
                            <size>)" + to_string(box.size[0]) + " " +
                to_string(box.size[1]) + " " +
                to_string(box.size[2]) + R"(</size>
                        </box>
                    </geometry>
                    <material>
                        <ambient>0.7 0.7 0.7 1</ambient>
                    </material>
                </visual>
                <collision name='collision'>
                    <geometry>
                        <box>
                            <size>)" + to_string(box.size[0]) + " " +
                to_string(box.size[1]) + " " +
                to_string(box.size[2]) + R"(</size>
                        </box>
                    </geometry>
                </collision>
                <pose>)" + to_string(box.pose.position.x) + " " +
                to_string(box.pose.position.y) + " " +
                to_string(box.pose.position.z) + R"( 0 0 0</pose>
            </link>
            )";
        }

        for (size_t i = 0; i < trajectory.size(); ++i) {
            sdf += R"(
            <static>true</static>
            <link name='sphere_)" + to_string(i) + R"('>
                <visual name='visual'>
                    <geometry>
                        <sphere>
                            <radius>0.1</radius>
                        </sphere>
                    </geometry>
                    <material>
                        <ambient>1.0 0.0 0.0 1.0</ambient>
                        <diffuse>1.0 0.0 0.0 1.0</diffuse> 
                        <specular>0.5 0.5 0.5 1.0</specular> 
                    </material>
                </visual>
                <pose>)" + to_string(trajectory[i].first) + " " +
                to_string(trajectory[i].second) + R"( 0 0 0 0</pose>
            </link>
            )";
        }

        sdf += R"(
            </model>
        </sdf>
        )";

        return sdf;
    }

    /* ------------------------------------------------- /
    /   Functions to extract boxes from input matrix     /
    / ------------------------------------------------- */
    bool checkBlocks(block blk_from, block blk_to) { // checks if block are identical
        if (blk_from.idx_b == blk_to.idx_b && blk_from.idx_f == blk_to.idx_f && blk_from.height == blk_to.height)
            return true;
        else
            return false;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = ::std::make_shared<EnvironmentVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}