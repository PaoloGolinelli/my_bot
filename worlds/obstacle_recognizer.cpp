#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

using namespace std;

// struct containing the informations of a block made by consecutive equal height pixels
struct block {
    int idx_b;      // beginning index
    int idx_f;      // ending index
    int height;     // height of consecutive pixels
};

// struct containing informations of a box (obstacle)
struct box {
    double x1, y1;     // top-left corner
    double x2, y2;     // bottom-right corner
    double height;     // box height
};

// functions
bool checkBlocks(block blk_from, block blk_to) { // checks if block are identical
    if (blk_from.idx_b == blk_to.idx_b && blk_from.idx_f == blk_to.idx_f && blk_from.height == blk_to.height)
        return true;
    else
        return false;
}

int main(int argc, char* argv[]) {
    
    bool verbose; 
    if (argc == 3)
        verbose = atoi(argv[2]);    // true to print feedback from algorithm (debug)
    else
        verbose = false;            // default is false

    /*----------------------------/  
    /  Input file initialization  /
    /----------------------------*/
    // Check if file name is provided as argument
    if (argc < 2) {
        cerr << "Required input: " << argv[0] << " <filename>" << endl;
        return 1;
    }
    // Open the file
    ifstream Ifile(argv[1]);
    if (!Ifile.is_open()) {
        cerr << "Error: Could not open file " << argv[1] << endl;
        return 1;
    }

    /*---------------------/  
    /     Read header      /
    /---------------------*/
    int m_width, m_height;
    double block_L_scale, block_H_scale;
    Ifile >> m_width;           // read matrix's width
    Ifile >> m_height;          // read matrix's height
    Ifile >> block_L_scale;     // dimension of one cell of the matrix
    Ifile >> block_H_scale;     // dimension of one cell of the matrix
    if (verbose) {
        cout << "matrix: " << m_width << "x" << m_height << ")" << endl;
        cout << "grid dimension: " << block_L_scale << ", " << block_H_scale << " m" << endl;
    }

    /*----------------------------/  
    /  Output file initialization  /
    /----------------------------*/
    string OBoxFilename = "obstacles.txt";
    ofstream OBoxfile;
    OBoxfile.open(OBoxFilename);

    if (!OBoxfile.is_open()) {
        std::cerr << "Error: Could not open the file!" << std::endl;
        return 1;
    }

    /*---------------------/  
    /  Compress the rows   /
    /---------------------*/
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
                    if (verbose)
                        cout << i-1 << "), " << prev_value << "); ";
                    blk_tmp.idx_f = i-1;
                    blk_tmp.height = prev_value;

                    // save block into row_vector
                    parsed_row.push_back(blk_tmp);
                }
                if (value != 0) {
                    if (verbose)
                        cout << "((" << i << " ";
                    blk_tmp.idx_b = i;
                }
            }

            prev_value = value;
        }

        // if object ends at the right border
        if (prev_value != 0) {
            if (verbose)
                cout << m_width-1 << "), " << prev_value << ")";
            blk_tmp.idx_f = m_width-1;
            blk_tmp.height = prev_value;

            // save block into row_vector
            parsed_row.push_back(blk_tmp);
        }

        if (verbose)
            cout << endl;

        // save parsed row into the matrix
        parsed_mat.push_back(parsed_row);
    }

    // Close the file
    Ifile.close();

    /*---------------------/  
    /    Extract boxes     /
    /---------------------*/
    block blk_comp;
    bool affinity;
    box box_pars;
    for (size_t r = 0; r < parsed_mat.size(); r++) {    // iterate over all parsed rows      
        for (size_t b = parsed_mat[r-1].size(); b < parsed_mat[r].size(); b++) { // iterate over all blocks in the row
            
            // extract current block
            blk_tmp = parsed_mat[r][b];
            if (blk_tmp.height == -1)           // skips the block if it is already been parsed in a box
                continue;

            // initial parsing of the box
            box_pars.x1 = blk_tmp.idx_b;
            box_pars.x2 = blk_tmp.idx_f;
            box_pars.y1 = r-1;
            box_pars.y2 = r-1;                  // at fist box is just a horizontal line
            box_pars.height = blk_tmp.height;

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
                        box_pars.y2 = nl-1;     // update box height
                        break;                  // continune to check next row
                    }
                }

                if (!affinity) { // box terminated
                    // Scale the box by the dimension of the grid
                    box_pars.x1 *= block_L_scale;
                    box_pars.y1 *= block_H_scale;
                    box_pars.x2 = (box_pars.x2 + 1) * block_L_scale;
                    box_pars.y2 = (box_pars.y2 + 1) * block_H_scale;
                    box_pars.height *= (block_L_scale+block_H_scale)/2.0;

                    if (verbose)
                        cout << box_pars.x1 << ", " << box_pars.y1 << ", " << box_pars.x2 << ", " << box_pars.y2 << "\n";

                    OBoxfile << box_pars.x1 << " " << box_pars.y1 << " " << box_pars.x2 << " " << box_pars.y2 << " " << box_pars.height << "\n";
                    break;
                }
            }
        }
    }

    return 0;
}
