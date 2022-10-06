#include <iostream>
#include <fstream>
#include <string>
#include <vector>

const bool primitive_test = true; // Whether or not to run the lazy cout tests

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
    const unsigned heuristic; // Heuristic value of this node
    std::vector<edge> edges; // All edges from this node to another, with path cost
public:
    node(std::string s, unsigned h, std::vector<edge> e)
        : state(s), heuristic(h), edges(e) { };

    std::string getState() { return state; };
    edge getEdge(unsigned i) { return edges.at(i); };
    unsigned getEdgeCount() { return edges.size(); };
    unsigned getHeuristic() { return heuristic; };

    void addEdge(edge e) { edges.push_back(e); };
};

std::string delTrailingWhitespace(std::string &input) {
    return input.substr(0, input.find_last_not_of(" ") + 1);
}

int getHeuristicFromVector(std::string state, std::vector<heuristic_data> data) {
    for (int i = 0; i < data.size(); ++i) {
        if (data.at(i).state == state) {
            return data.at(i).value;
        }
    }

    // Failure
    std::cerr << "Failed to find heuristic information for a node of state " << state
              << std::endl;
    return -1;
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

    // Test that the data was read correctly
    if (primitive_test) {
        std::cout << "Edges: " << std::endl;
        for (int i = 0; i < live_traffic_data.size(); ++i) {
            std::cout << "From " << live_traffic_data.at(i).start_state
                      << " to " << live_traffic_data.at(i).end_state
                      << " with cost " << live_traffic_data.at(i).cost
                      << std::endl;
        }
        std::cout << "Heuristics: " << std::endl;
        for (int i = 0; i < sunday_traffic_data.size(); ++i) {
            std::cout << "Heuristic of state " << sunday_traffic_data.at(i).state
                      << " is " << sunday_traffic_data.at(i).value << std::endl;
        }
        std::cout << std::endl;
    }

    // All data has been read, won't need input file again
    ifs.close();

    /* Generate the graph using all the provided data */
    // First all nodes that have edges leading away from them
    std::vector<node> graph;
    for (int i = 0; i < live_traffic_data.size(); ++i) {
        const std::string node_state = live_traffic_data.at(i).start_state;

        // Does a node at the start of this edge already exist?
        bool node_already_exists = false;
        int existing_node_index = -1;
        for (int j = 0; j < graph.size(); ++j) {
            if (graph.at(j).getState() == node_state) {
                node_already_exists = true;
                existing_node_index = j;
            }
        }

        if (node_already_exists) { // If so, just add this edge to the existing node
            graph.at(existing_node_index).addEdge(live_traffic_data.at(i));
        } else { // Otherwise, create new node with this edge
            // Have to set the heuristic value while creating a new node
            const int h = getHeuristicFromVector(node_state, sunday_traffic_data);
            std::vector<edge> e;
            e.push_back(live_traffic_data.at(i));

            node n(node_state, h, e);
            graph.push_back(n);
        }
    }
    // Now go through and add the node(s) that only exist at the ends of edges, if any
    for (int i = 0; i < live_traffic_data.size(); ++i) {
        const std::string node_state = live_traffic_data.at(i).end_state;

        // Does the node at the end of this edge already exist?
        bool node_already_exists = false;
        for (int j = 0; j < graph.size(); ++j) {
            if (graph.at(j).getState() == node_state) {
                node_already_exists = true;
            }
        }

        // If not, it must be a node with no edges leaving it, since already added the rest
        if (!node_already_exists) {
            const int h = getHeuristicFromVector(node_state, sunday_traffic_data);
            std::vector<edge> e; // Leave vector empty since this node has no edges

            node n(node_state, h, e);
            graph.push_back(n);
        }
    }

    // Test that the graph is constructed correctly
    if (primitive_test) {
        std::cout << "Graph vector: " << std::endl;
        for (int i = 0; i < graph.size(); ++i) {
            std::cout << "Node: " << graph.at(i).getState()
                      << ", Heuristic: " << graph.at(i).getHeuristic() << std::endl;

            std::cout << "Edges: " << std::endl;

            unsigned edgeCount = graph.at(i).getEdgeCount();
            if (edgeCount > 0) {
                for (int j = 0; j < graph.at(i).getEdgeCount(); ++j) {
                    std::cout << "From " << graph.at(i).getEdge(j).start_state
                              << " to " << graph.at(i).getEdge(j).end_state
                              << " with cost " << graph.at(i).getEdge(j).cost
                              << std::endl;
                }
            } else {
                std::cout << "None." << std::endl;
            }
        }
    }

    /* Apply the specified algorithm and find out the solution */

    /* Output the solution to output.txt */

    return 0;
}
