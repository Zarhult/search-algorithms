#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <algorithm>

class node {
private:
    std::string state; // State that the node corresponds to
    std::vector<std::string> connections; // Nodes that this node can lead to
    std::vector<unsigned> costs; // Path costs to reach each connected node, respectively
public:
    node(std::string s, std::vector<std::string> cons, std::vector<unsigned> csts) {
        state = s;
        connections = cons;
        costs = csts;
    }

    std::string getState;
    std::string getConnection;
    std::string getCost;
};

void delSpaces(std::string &input) {
    input.erase(std::remove(input.begin(), input.end(), ' '), input.end());
}

int main() {
    std::ifstream ifs("input.txt");

    if (ifs.fail()) {
        std::cerr << "Failed to open input.txt." << std::endl;
        return -1;
    }

    // First three lines are always the algorithm followed by start and goal state
    std::string algo, start_state, goal_state;
    std::getline(ifs, algo);
    delSpaces(algo);
    std::getline(ifs, start_state);
    delSpaces(start_state);
    std::getline(ifs, goal_state);
    delSpaces(goal_state);

    // Grab the number of live traffic lines as an (unsigned) integer
    std::string live_line_count_string;
    std::getline(ifs, live_line_count_string);
    unsigned live_lines = static_cast<unsigned>(stoi(live_line_count_string));

    return 0;
}
