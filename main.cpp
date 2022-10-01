#include <iostream>
#include <fstream>
#include <string>
#include <vector>

struct edge {
    // An edge from A to B may not have the same cost as B to A (not interchangeable)
    const std::string start_state = "";
    const std::string end_state = "";
    const unsigned cost = 0;
};

struct heuristic_data {
    const std::string state = "";
    const unsigned value = 0;
};

class node {
private:
    const std::string state; // State that this node corresponds to
    const std::vector<edge> edges; // All edges from this node to another, with path cost
    const unsigned heuristic; // Heuristic value of this node
public:
    node(std::string s, std::vector<edge> e, unsigned h)
        : state(s), edges(e), heuristic(h) { };

    std::string getState() { return state; };
    edge getEdge(unsigned i) { return edges.at(i); };
    unsigned getHeuristic() { return heuristic; };
};

std::string delTrailingWhitespace(std::string &input) {
    return input.substr(0, input.find_last_not_of(" ") + 1);
}

int main() {
    /* First collect all data from input.txt */
    std::string const input_filename = "input.txt";
    std::ifstream ifs(input_filename, std::ios::in);

    if (ifs.fail()) {
        std::cerr << "Failed to open " << input_filename << std::endl;
        return -1;
    }

    // First three lines are always the algorithm followed by start and goal state
    std::string algo, start_state, goal_state;
    std::getline(ifs, algo);
    algo = delTrailingWhitespace(algo);
    std::getline(ifs, start_state);
    start_state = delTrailingWhitespace(start_state);
    std::getline(ifs, goal_state);
    goal_state = delTrailingWhitespace(goal_state);

    // Grab the number of live traffic lines as an (unsigned) integer
    std::string live_line_count_string;
    std::getline(ifs, live_line_count_string);
    const unsigned live_lines = static_cast<unsigned>(stoi(live_line_count_string));

    // Create vector containing all the live traffic data (graph edges)
    std::vector<edge> live_traffic_data;
    for (int i = 0; i < live_lines; ++i) {
        std::string line;
        std::getline(ifs, line);
        line = delTrailingWhitespace(line);

        // Assuming data is separated by single spaces
        unsigned first_space_pos = line.find_first_of(" ");
        unsigned last_space_pos = line.find_last_of(" ");
        edge e = {
            line.substr(0, first_space_pos),
            line.substr(first_space_pos + 1, last_space_pos - first_space_pos - 1),
            static_cast<unsigned>(stoi(line.substr(last_space_pos + 1, line.size() - 1)))
        };

        live_traffic_data.push_back(e);
    }

    // Grab the number of sunday traffic lines as an (unsigned) integer
    std::string sunday_line_count_string;
    std::getline(ifs, sunday_line_count_string);
    const unsigned sunday_lines = static_cast<unsigned>(stoi(sunday_line_count_string));

    // Create vector containing all the sunday traffic data
    std::vector<heuristic_data> sunday_traffic_data;
    for (int i = 0; i < sunday_lines; ++i) {
        std::string line;
        std::getline(ifs, line);
        line = delTrailingWhitespace(line);

        //Assuming data is separated by a single space
        unsigned space_pos = line.find_first_of(" ");
        heuristic_data d = {
            line.substr(0, space_pos),
            static_cast<unsigned>(stoi(line.substr(space_pos + 1, line.size() - 1)))
        };

        sunday_traffic_data.push_back(d);
    }

    // test
    /*
    for (int i = 0; i < live_traffic_data.size(); ++i) {
        std::cout << live_traffic_data.at(i).start_state << std::endl
                  << live_traffic_data.at(i).end_state << std::endl
                  << live_traffic_data.at(i).cost << std::endl;
    }

    for (int i = 0; i < sunday_traffic_data.size(); ++i) {
        std::cout << sunday_traffic_data.at(i).state << std::endl
                  << sunday_traffic_data.at(i).value << std::endl;
    }
    */

    // All data has been collected, won't need input file again
    ifs.close();

    /* Generate the graph using all the provided data */

    /* Apply the specified algorithm and find out the solution */

    /* Output the solution to output.txt */

    return 0;
}
