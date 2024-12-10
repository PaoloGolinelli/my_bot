#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

using namespace std;

// struct containing informations of a box
struct box {
    double x1, y1;  // Top-left vertex
    double x2, y2;  // Bottom-right vertex
    double height;  // Height of the box
};

vector<box> parseBoxes(const string &filePath, bool verbose = false) {
    vector<box> boxes;
    ifstream inputFile(filePath);

    if (!inputFile) {
        cerr << "Error: Unable to open file " << filePath << endl;
        return boxes;
    }

    string line;
    while (getline(inputFile, line)) {
        istringstream lineStream(line);
        istringstream iss(line);
        box box;
        if (lineStream >> box.x1 >> box.y1 >> box.x2 >> box.y2 >> box.height) {
            if (verbose)
                cout << box.x1 << ", " << box.y1 << ", " << box.x2 << ", " << box.y2 << ", " << box.height << endl; // debug
            boxes.push_back(box);
        } else {
            if (verbose)
                cerr << "Warning: Skipping invalid line: " << line << endl;
        }
    }

    inputFile.close();
    return boxes;
}

void generateWorld(const vector<box> &boxes, const string &OWorldPath, bool verbose) {
    ofstream OWorldFile(OWorldPath);

    if (!OWorldFile) {
        cerr << "Error: Unable to open file " << OWorldPath << endl;
        return;
    }

    // Write the header of the .world file
    OWorldFile << "<?xml version=\"1.0\" ?>\n";
    OWorldFile << "<sdf version=\"1.6\">\n";
    OWorldFile << "  <world name=\"obstacles\">\n";
    OWorldFile << "    <plugin name=\"gazebo_ros_factory\" filename=\"libgazebo_ros_factory.so\"/>\n";
    OWorldFile << endl;
    OWorldFile << "    <!-- A global light source -->\n";
    OWorldFile << "    <include>\n";
    OWorldFile << "      <uri>model://sun</uri>\n";
    OWorldFile << "    </include>\n";
    OWorldFile << "    <!-- A ground plane -->\n";
    OWorldFile << "    <include>\n";
    OWorldFile << "      <uri>model://ground_plane</uri>\n";
    OWorldFile << "    </include>\n";
    OWorldFile << endl;

    // Write each box as a model
    for (size_t i = 0; i < boxes.size(); ++i) {
        const box& box = boxes[i];
        double length = abs(box.x2 - box.x1);
        double width = abs(box.y2 - box.y1);
        double depth = box.height;
        double centerX = (box.x1 + box.x2) / 2.0;
        double centerY = (box.y1 + box.y2) / 2.0;
        double centerZ = depth / 2.0;

        OWorldFile << "    <model name=\"box_" << i << "\">\n";
        OWorldFile << "      <static>true</static>\n";
        OWorldFile << "      <link name=\"link_" << i << "\">\n";
        OWorldFile << "        <visual name=\"visual_" << i << "\">\n";
        OWorldFile << "          <pose>" << centerX << " " << centerY << " " << centerZ << " 0 0 0</pose>\n";
        OWorldFile << "          <geometry>\n";
        OWorldFile << "            <box>\n";
        OWorldFile << "              <size>" << length << " " << width << " " << depth << "</size>\n";
        OWorldFile << "            </box>\n";
        OWorldFile << "          </geometry>\n";
        OWorldFile << "        </visual>\n";
        OWorldFile << "        <collision name=\"collision_" << i << "\">\n";
        OWorldFile << "          <pose>" << centerX << " " << centerY << " " << centerZ << " 0 0 0</pose>\n";
        OWorldFile << "          <geometry>\n";
        OWorldFile << "            <box>\n";
        OWorldFile << "              <size>" << length << " " << width << " " << depth << "</size>\n";
        OWorldFile << "            </box>\n";
        OWorldFile << "          </geometry>\n";
        OWorldFile << "        </collision>\n";
        OWorldFile << "      </link>\n";
        OWorldFile << "    </model>\n";
        OWorldFile << endl;
    }

    // Write the footer of the .world file
    OWorldFile << "  </world>\n";
    OWorldFile << "</sdf>\n";

    OWorldFile.close();
    if (verbose)
        cout << "World file generated: " << OWorldPath << endl;
}

int main(int argc, char* argv[]) {
    
    bool verbose; 
    if (argc == 3)
        verbose = atoi(argv[2]);    // true to print feedback from algorithm (debug)
    else
        verbose = false;            // default is false

    // Check if file name is provided as argument
    if (argc < 2) {
        cerr << "Required input: " << argv[0] << " <filename>" << endl;
        return 1;
    }

    // Open the file
    string IFilePath = argv[1];             // Path to the input .txt file
    string OFilePath = "obstacles.world";    // Path to the output .world file

    // Parse the boxes from the input file
    vector<box> boxes = parseBoxes(IFilePath, verbose);
    if (boxes.empty()) {
        cerr << "Error: No valid box data found in file " << IFilePath << endl;
        return 1;
    }

    // Generate the .world file
    generateWorld(boxes, OFilePath, verbose);

    return 0;
}
