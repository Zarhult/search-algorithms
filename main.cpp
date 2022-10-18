#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <deque>

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
    std::vector<std::string> path; // List, in order, of states traversed to reach this node
    std::vector<unsigned> costs; // List in order of accumulated path costs along path
public:
    node(std::string s, unsigned h)
        : state(s), heuristic(h) { };

    std::string getState() { return state; };
    unsigned getHeuristic() { return heuristic; };
    unsigned getEvaluationCost() { return costs.at(costs.size() - 1) + heuristic; };
    edge getEdge(unsigned i) { return edges.at(i); };
    unsigned getEdgeCount() { return edges.size(); };
    std::string getPathMember(unsigned i) { return path.at(i); };
    unsigned getPathSize() { return path.size(); };
    unsigned getCostsMember(unsigned i) { return costs.at(i); };
    unsigned getCostsSize() { return costs.size(); };
    unsigned getTotalCost() { return costs.at(costs.size() - 1); };

    void addEdge(edge e) { edges.push_back(e); };
    void addToPath(std::string s) { path.push_back(s); };
    void clearPath() { path.clear(); };
    void addToCosts(unsigned n) { costs.push_back(n); };
    void clearCosts() { costs.clear(); };
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

int getIndexOfState(std::vector<node> graph, std::string state) {
    for (int i = 0; i < graph.size(); ++i) {
        if (graph.at(i).getState() == state) {
            return i;
        }
    }

    // Failure
    std::cerr << "Failed to find a node of state " << state << std::endl;
    return -1;
}

node BFS(std::vector<node> graph, unsigned start_index, unsigned goal_index) {
    // Frontier and explored contain indices of nodes
    std::deque<unsigned> frontier; // Need deque to be able to search the frontier easily
    std::vector<unsigned> explored;
    frontier.push_front(start_index); // Initial state of frontier is just the start node

    // Initialize start state's path and costs vectors
    graph.at(start_index).clearPath();
    graph.at(start_index).addToPath(graph.at(start_index).getState());
    graph.at(start_index).clearCosts();
    graph.at(start_index).addToCosts(0);

    while (!frontier.empty()) {
        unsigned n = frontier.front(); // FIFO
        frontier.pop_front();

        // Goal test
        if (n == goal_index) {
            return graph.at(n);
        }

        explored.push_back(n);

        // Tie break: edge that appears earliest in input file (lowest index in node.edges)
        // is enqueued first
        for (int i = 0; i < graph.at(n).getEdgeCount(); ++i) {
            std::string child_state = graph.at(n).getEdge(i).end_state;
            unsigned child_state_index = getIndexOfState(graph, child_state);

            // Ignore children that are already in explored or frontier
            bool already_in_explored = std::find(explored.begin(), explored.end(),
                                                 child_state_index) != explored.end();
            bool already_in_frontier = std::find(frontier.begin(), frontier.end(),
                                                 child_state_index) != frontier.end();
            if (!already_in_explored && !already_in_frontier) {
                // Set child's path and cost vectors to the parent's
                graph.at(child_state_index).clearPath();
                for (int j = 0; j < graph.at(n).getPathSize(); ++j) {
                    graph.at(child_state_index).addToPath(graph.at(n).getPathMember(j));
                }
                graph.at(child_state_index).clearCosts();
                for (int j = 0; j < graph.at(n).getCostsSize(); ++j) {
                    graph.at(child_state_index).addToCosts(graph.at(n).getCostsMember(j));
                }

                // Now update path/costs to account for the transition from parent to child
                graph.at(child_state_index).addToPath(child_state);
                unsigned parentCost =
                    graph.at(n).getCostsMember(graph.at(n).getCostsSize() - 1);
                graph.at(child_state_index).addToCosts(parentCost
                                                       + graph.at(n).getEdge(i).cost);

                // Finally add the child to the frontier
                frontier.push_back(child_state_index);
            }
        }
    }

    // Only reach this line if fail to find a solution in the loop, so return failure
    return node("", 0); // Represent failure as a stateless node
}

/* Exact same as BFS except frontier is LIFO */
node DFS(std::vector<node> graph, unsigned start_index, unsigned goal_index) {
    // Frontier and explored contain indices of nodes
    std::deque<unsigned> frontier; // Need deque to be able to search the frontier easily
    std::vector<unsigned> explored;
    frontier.push_front(start_index); // Initial state of frontier is just the start node

    // Initialize start state's path and costs vectors
    graph.at(start_index).clearPath();
    graph.at(start_index).addToPath(graph.at(start_index).getState());
    graph.at(start_index).clearCosts();
    graph.at(start_index).addToCosts(0);

    while (!frontier.empty()) {
        unsigned n = frontier.back(); // LIFO
        frontier.pop_back();

        // Goal test
        if (n == goal_index) {
            return graph.at(n);
        }

        explored.push_back(n);

        // Tie break: edge that appears earliest in input file (lowest index in node.edges)
        // is enqueued first
        for (int i = 0; i < graph.at(n).getEdgeCount(); ++i) {
            std::string child_state = graph.at(n).getEdge(i).end_state;
            unsigned child_state_index = getIndexOfState(graph, child_state);

            // Ignore children that are already in explored or frontier
            bool already_in_explored = std::find(explored.begin(), explored.end(),
                                                 child_state_index) != explored.end();
            bool already_in_frontier = std::find(frontier.begin(), frontier.end(),
                                                 child_state_index) != frontier.end();
            if (!already_in_explored && !already_in_frontier) {
                // Set child's path and cost vectors to the parent's
                graph.at(child_state_index).clearPath();
                for (int j = 0; j < graph.at(n).getPathSize(); ++j) {
                    graph.at(child_state_index).addToPath(graph.at(n).getPathMember(j));
                }
                graph.at(child_state_index).clearCosts();
                for (int j = 0; j < graph.at(n).getCostsSize(); ++j) {
                    graph.at(child_state_index).addToCosts(graph.at(n).getCostsMember(j));
                }

                // Now update path/costs to account for the transition from parent to child
                graph.at(child_state_index).addToPath(child_state);
                unsigned parentCost =
                    graph.at(n).getCostsMember(graph.at(n).getCostsSize() - 1);
                graph.at(child_state_index).addToCosts(parentCost
                                                       + graph.at(n).getEdge(i).cost);

                // Finally add the child to the frontier
                frontier.push_back(child_state_index);
            }
        }
    }

    // Only reach this line if fail to find a solution in the loop, so return failure
    return node("", 0); // Represent failure as a stateless node
}

node UCS(std::vector<node> graph, unsigned start_index, unsigned goal_index) {
    // Frontier and explored contain indices of nodes
    // For convenience, store frontier in descending order of path costs and treat
    // the last element as the front of the frontier, so that we can pop that element
    // with pop_back
    std::vector<unsigned> frontier;
    std::vector<unsigned> explored;
    frontier.push_back(start_index); // Initial state of frontier is just the start node

    // Initialize start state's path and costs vectors
    graph.at(start_index).clearPath();
    graph.at(start_index).addToPath(graph.at(start_index).getState());
    graph.at(start_index).clearCosts();
    graph.at(start_index).addToCosts(0);

    while (!frontier.empty()) {
        unsigned n = frontier.back();
        frontier.pop_back();

        // Goal test
        if (n == goal_index) {
            return graph.at(n);
        }

        explored.push_back(n);

        // Create vector of the children indices first
        std::vector<unsigned> children;
        for (int i = 0; i < graph.at(n).getEdgeCount(); ++i) {
            std::string child_state = graph.at(n).getEdge(i).end_state;
            unsigned child_state_index = getIndexOfState(graph, child_state);

            if (primitive_test) {
                std::cout << std::endl << "Checking "
                          << graph.at(n).getState() << "'s child "
                          << graph.at(child_state_index).getState() << std::endl;
            }

            // Ignore children that are already in explored
            bool already_in_explored = std::find(explored.begin(), explored.end(),
                                                 child_state_index) != explored.end();

            if (primitive_test) {
                std::cout << child_state << " is already in explored?: "
                          << already_in_explored << std::endl;
            }

            if (!already_in_explored) {
                auto child_in_frontier = std::find(frontier.begin(), frontier.end(),
                                                   child_state_index);
                // Node already exists in frontier?
                bool already_in_frontier = child_in_frontier != frontier.end();

                if (primitive_test) {
                    std::cout << child_state << " is already in frontier?: "
                              << already_in_frontier << std::endl;
                }

                // Node with lower/equal path cost already exists in frontier?
                bool already_in_frontier_better = false;
                unsigned parent_cost = graph.at(n).getTotalCost();
                unsigned new_cost = parent_cost + graph.at(n).getEdge(i).cost;
                if (already_in_frontier) {
                    unsigned old_cost = graph.at(child_state_index).getTotalCost();
                    if (old_cost <= new_cost) {
                        already_in_frontier_better = true;
                    }
                }

                if (primitive_test) {
                    std::cout << child_state
                              << " is already in frontier with a better/equal cost?: "
                              << already_in_frontier_better << std::endl;
                }

                // Ignore children that do not improve over what is already in the frontier
                if (!already_in_frontier_better) {
                    if (already_in_frontier) {
                        std:: cout << "Deleting inferior path " << child_state
                                   << " from frontier" << std::endl;
                        // Child is already in frontier but inferior to what we just found,
                        // so we have to remove it before we add the new child.
                        frontier.erase(child_in_frontier);
                    }

                    // Set child's path and cost vectors to the parent's
                    graph.at(child_state_index).clearPath();
                    for (int j = 0; j < graph.at(n).getPathSize(); ++j) {
                        graph.at(child_state_index).addToPath(graph.at(n).getPathMember(j));
                    }
                    graph.at(child_state_index).clearCosts();
                    for (int j = 0; j < graph.at(n).getCostsSize(); ++j) {
                        unsigned cost = graph.at(n).getCostsMember(j);
                        graph.at(child_state_index).addToCosts(cost);
                    }

                    // Now update path/costs to account for the transition from
                    // parent to child
                    graph.at(child_state_index).addToPath(child_state);
                    graph.at(child_state_index).addToCosts(parent_cost +
                                                           graph.at(n).getEdge(i).cost);

                    if (primitive_test) {
                        std::cout << "Path of " << child_state << " is ";
                        for (int j = 0; j < graph.at(child_state_index).getPathSize(); ++j) {
                            std::cout << graph.at(child_state_index).getPathMember(j) << " ";
                        }
                        std::cout << std::endl;

                        std::cout << "Costs of " << child_state << " are ";
                        unsigned costs_size = graph.at(child_state_index).getCostsSize();
                        for (int j = 0; j < costs_size; ++j) {
                            std::cout << graph.at(child_state_index).getCostsMember(j)
                                      << " ";
                        }
                        std::cout << std::endl;
                    }

                    // Finally, we can add the child to the children vector for handling
                    children.push_back(child_state_index);
                }
            }
        }

        if (primitive_test) {
            if (children.size() > 0) {
                std::cout << std::endl << "Children vector is ";
                for (int i = 0; i < children.size(); ++i) {
                    std::cout << graph.at(children.at(i)).getState() << " ";
                }
                std::cout << std::endl;
            } else {
                std::cout << std::endl << "Children vector is empty"
                          << " (none of this node's children need to be added to frontier)"
                          << std::endl;
            }
        }

        // Still have to handle tie breaks: if multiple children have the same cost, insert
        // them in the order they appear in the input file, and if inserting a child with
        // equal path cost to a node that already exists in the frontier,
        // enqueue the child right after that node.
        for (int i = 0; i < children.size(); ++i) {
            unsigned child_cost = graph.at(children.at(i)).getTotalCost();
            // Vector of indices in the children vector that this child ties with
            std::vector<unsigned> children_vector_tie_indices;
            for (int j = 0; j < children.size(); ++j) {
                if (j != i && child_cost == graph.at(children.at(j)).getTotalCost()) {
                    children_vector_tie_indices.push_back(j);
                }
            }

            if (primitive_test) {
                if (children_vector_tie_indices.size() > 0) {
                    std::cout << std::endl << "Found ties: ";
                    for (int j = 0; j < children_vector_tie_indices.size(); ++j) {
                        unsigned tie_children_index = children_vector_tie_indices.at(j);
                        unsigned tie_graph_index = children.at(tie_children_index);
                        std::cout << graph.at(tie_graph_index).getState() << " ";
                    }
                }
                std::cout << std::endl;
            }

            // Find where the child and the children it ties with, if any, should be
            // inserted. Insert at end of vector (front of frontier) if fail to find
            // any nodes in frontier with cost less than or equal to the child's cost
            unsigned child_insert_position = frontier.size();
            bool found_insert_position = false;
            for (int j = 0; j < frontier.size() && !found_insert_position; ++j) {
                if (graph.at(frontier.at(j)).getTotalCost() <= child_cost) {
                    child_insert_position = j;
                    found_insert_position = true;
                }
            }

            if (primitive_test) {
                std::cout << "Frontier before adding child "
                          << graph.at(children.at(i)).getState() << " and ties: ";
                for (int j = 0; j < frontier.size(); ++j) {
                    std::cout << graph.at(frontier.at(j)).getState() << " ";
                }
                std::cout << std::endl;
            }

            // Add the children to the frontier. For ties, the node earliest in the
            // input file (= lowest index in children vector) goes closest to the front
            // of the frontier (front of frontier = last element, which has lowest cost)
            frontier.insert(frontier.begin()+child_insert_position, children.at(i));
            for (int j = 0; j < children_vector_tie_indices.size(); ++j) {
                unsigned tie_children_index = children_vector_tie_indices.at(j);
                unsigned tie_graph_index = children.at(tie_children_index);
                frontier.insert(frontier.begin()+child_insert_position, tie_graph_index);
            }

            if (primitive_test) {
                std::cout << "Frontier after adding child "
                          << graph.at(children.at(i)).getState() << " and ties: ";
                for (int j = 0; j < frontier.size(); ++j) {
                    std::cout << graph.at(frontier.at(j)).getState() << " ";
                }
                std::cout << std::endl;
            }

            // Delete the children that tied from the children vector, so they aren't
            // added a second time
            for (int j = 0; j < children_vector_tie_indices.size(); ++j) {
                children.erase(children.begin()+children_vector_tie_indices.at(j));
            }
        }
    }

    // Only reach this line if fail to find a solution in the loop, so return failure
    return node("", 0); // Represent failure as a stateless node
}

/* Exact same as UCS except path cost -> path cost + heuristic */
node A_STAR(std::vector<node> graph, unsigned start_index, unsigned goal_index) {
    // Frontier and explored contain indices of nodes
    // For convenience, store frontier in descending order of evaluation costs and treat
    // the last element as the front of the frontier, so that we can pop that element
    // with pop_back
    std::vector<unsigned> frontier;
    std::vector<unsigned> explored;
    frontier.push_back(start_index); // Initial state of frontier is just the start node

    // Initialize start state's path and costs vectors
    // Note that the costs vector is still only path costs, just like in UCS
    graph.at(start_index).clearPath();
    graph.at(start_index).addToPath(graph.at(start_index).getState());
    graph.at(start_index).clearCosts();
    graph.at(start_index).addToCosts(0);

    while (!frontier.empty()) {
        unsigned n = frontier.back();
        frontier.pop_back();

        // Goal test
        if (n == goal_index) {
            return graph.at(n);
        }

        explored.push_back(n);

        // Create vector of the children indices first
        std::vector<unsigned> children;
        for (int i = 0; i < graph.at(n).getEdgeCount(); ++i) {
            std::string child_state = graph.at(n).getEdge(i).end_state;
            unsigned child_state_index = getIndexOfState(graph, child_state);

            if (primitive_test) {
                std::cout << std::endl << "Checking "
                          << graph.at(n).getState() << "'s child "
                          << graph.at(child_state_index).getState() << std::endl;
            }

            // Ignore children that are already in explored
            bool already_in_explored = std::find(explored.begin(), explored.end(),
                                                 child_state_index) != explored.end();

            if (primitive_test) {
                std::cout << child_state << " is already in explored?: "
                          << already_in_explored << std::endl;
            }

            if (!already_in_explored) {
                auto child_in_frontier = std::find(frontier.begin(), frontier.end(),
                                                   child_state_index);
                // Node already exists in frontier?
                bool already_in_frontier = child_in_frontier != frontier.end();

                if (primitive_test) {
                    std::cout << child_state << " is already in frontier?: "
                              << already_in_frontier << std::endl;
                }

                // Node with lower/equal evaluation cost already exists in frontier?
                bool already_in_frontier_better = false;
                unsigned parent_path_cost = graph.at(n).getTotalCost();
                unsigned child_path_cost = parent_path_cost + graph.at(n).getEdge(i).cost;
                unsigned child_heuristic = graph.at(child_state_index).getHeuristic();
                // Cannot use getEvaluationCost since child node hasn't had its costs vector
                // updated yet, so have to calculate it like this
                unsigned new_cost = child_path_cost + child_heuristic;
                if (already_in_frontier) {
                    unsigned old_cost = graph.at(child_state_index).getEvaluationCost();
                    if (old_cost <= new_cost) {
                        already_in_frontier_better = true;
                    }
                }

                if (primitive_test) {
                    std::cout << child_state
                              << " is already in frontier with a better/equal cost?: "
                              << already_in_frontier_better << std::endl;
                }

                // Ignore children that do not improve over what is already in the frontier
                if (!already_in_frontier_better) {
                    if (already_in_frontier) {
                        std:: cout << "Deleting inferior path " << child_state
                                   << " from frontier" << std::endl;
                        // Child is already in frontier but inferior to what we just found,
                        // so we have to remove it before we add the new child.
                        frontier.erase(child_in_frontier);
                    }

                    // Set child's path and cost vectors to the parent's
                    graph.at(child_state_index).clearPath();
                    for (int j = 0; j < graph.at(n).getPathSize(); ++j) {
                        graph.at(child_state_index).addToPath(graph.at(n).getPathMember(j));
                    }
                    graph.at(child_state_index).clearCosts();
                    for (int j = 0; j < graph.at(n).getCostsSize(); ++j) {
                        unsigned cost = graph.at(n).getCostsMember(j);
                        graph.at(child_state_index).addToCosts(cost);
                    }

                    // Now update path/costs to account for the transition from
                    // parent to child
                    graph.at(child_state_index).addToPath(child_state);
                    graph.at(child_state_index).addToCosts(parent_path_cost +
                                                           graph.at(n).getEdge(i).cost);

                    if (primitive_test) {
                        std::cout << "Path of " << child_state << " is ";
                        for (int j = 0; j < graph.at(child_state_index).getPathSize(); ++j) {
                            std::cout << graph.at(child_state_index).getPathMember(j) << " ";
                        }
                        std::cout << std::endl;

                        std::cout << "Costs of " << child_state << " are ";
                        unsigned costs_size = graph.at(child_state_index).getCostsSize();
                        for (int j = 0; j < costs_size; ++j) {
                            std::cout << graph.at(child_state_index).getCostsMember(j)
                                      << " ";
                        }
                        std::cout << std::endl;
                    }

                    // Finally, we can add the child to the children vector for handling
                    children.push_back(child_state_index);
                }
            }
        }

        if (primitive_test) {
            if (children.size() > 0) {
                std::cout << std::endl << "Children vector is ";
                for (int i = 0; i < children.size(); ++i) {
                    std::cout << graph.at(children.at(i)).getState() << " ";
                }
                std::cout << std::endl;
            } else {
                std::cout << std::endl << "Children vector is empty"
                          << " (none of this node's children need to be added to frontier)"
                          << std::endl;
            }
        }

        // Still have to handle tie breaks: if multiple children have the same cost, insert
        // them in the order they appear in the input file, and if inserting a child with
        // equal evaluation cost to a node that already exists in the frontier,
        // enqueue the child right after that node.
        for (int i = 0; i < children.size(); ++i) {
            unsigned child_evaluation_cost = graph.at(children.at(i)).getEvaluationCost();
            // Vector of indices in the children vector that this child ties with
            std::vector<unsigned> children_vector_tie_indices;
            for (int j = 0; j < children.size(); ++j) {
                unsigned evaluation_cost = graph.at(children.at(j)).getEvaluationCost();
                bool tie = (child_evaluation_cost == evaluation_cost);
                if (j != i && tie) {
                    children_vector_tie_indices.push_back(j);
                }
            }

            if (primitive_test) {
                if (children_vector_tie_indices.size() > 0) {
                    std::cout << std::endl << "Found ties: ";
                    for (int j = 0; j < children_vector_tie_indices.size(); ++j) {
                        unsigned tie_children_index = children_vector_tie_indices.at(j);
                        unsigned tie_graph_index = children.at(tie_children_index);
                        std::cout << graph.at(tie_graph_index).getState() << " ";
                    }
                }
                std::cout << std::endl;
           }

            // Find where the child and the children it ties with, if any, should be
            // inserted. Insert at end of vector (front of frontier) if fail to find
            // any nodes in frontier with cost less than or equal to the child's cost
            unsigned child_insert_position = frontier.size();
            bool found_insert_position = false;
            for (int j = 0; j < frontier.size() && !found_insert_position; ++j) {
                if (graph.at(frontier.at(j)).getEvaluationCost() <= child_evaluation_cost) {
                    child_insert_position = j;
                    found_insert_position = true;
                }
            }

            if (primitive_test) {
                std::cout << "Frontier before adding child "
                          << graph.at(children.at(i)).getState() << " and ties: ";
                for (int j = 0; j < frontier.size(); ++j) {
                    std::cout << graph.at(frontier.at(j)).getState() << " ";
                }
                std::cout << std::endl;
            }

            // Add the children to the frontier. For ties, the node earliest in the
            // input file (= lowest index in children vector) goes closest to the front
            // of the frontier (front of frontier = last element, which has lowest cost)
            frontier.insert(frontier.begin()+child_insert_position, children.at(i));
            for (int j = 0; j < children_vector_tie_indices.size(); ++j) {
                unsigned tie_children_index = children_vector_tie_indices.at(j);
                unsigned tie_graph_index = children.at(tie_children_index);
                frontier.insert(frontier.begin()+child_insert_position, tie_graph_index);
            }

            if (primitive_test) {
                std::cout << "Frontier after adding child "
                          << graph.at(children.at(i)).getState() << " and ties: ";
                for (int j = 0; j < frontier.size(); ++j) {
                    std::cout << graph.at(frontier.at(j)).getState() << " ";
                }
                std::cout << std::endl;
            }

            // Delete the children that tied from the children vector, so they aren't
            // added a second time
            for (int j = 0; j < children_vector_tie_indices.size(); ++j) {
                children.erase(children.begin()+children_vector_tie_indices.at(j));
            }
        }
    }

    // Only reach this line if fail to find a solution in the loop, so return failure
    return node("", 0); // Represent failure as a stateless node
}

node getSolution(std::vector<node> graph,
                 unsigned start_index, unsigned goal_index, std::string alg) {
    if (alg == "BFS") {
        return BFS(graph, start_index, goal_index);
    } else if (alg == "DFS") {
        return DFS(graph, start_index, goal_index);
    } else if (alg == "UCS") {
        return UCS(graph, start_index, goal_index);
    } else if (alg == "A*") {
        return A_STAR(graph, start_index, goal_index);
    } else {
        std::cerr << "Invalid algorithm name \"" << alg << "\". Defaulting to UCS." << std::endl;
        return UCS(graph, start_index, goal_index);
    }
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

        // Assuming data is separated by a single space
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

            node n(node_state, h);
            n.addEdge(live_traffic_data.at(i));
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
            node n(node_state, h);
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
    // Start state is not necessarily the first node in the graph vector
    unsigned start_index = getIndexOfState(graph, start_state);
    unsigned goal_index = getIndexOfState(graph, goal_state);

    node solution = getSolution(graph, start_index, goal_index, algo);

    if (solution.getState() == "") { // Empty state represents failure
        std::cout << "Failed to find a solution.";

        return -1;
    }

    if (primitive_test) {
        std::cout << "Found solution node of state " << solution.getState()
                  << " using " << algo << std::endl;

        std::cout << "Solution path is: " << std::endl;
        for (int i = 0; i < solution.getPathSize(); ++i) {
            std::cout << solution.getPathMember(i)
                      << " reached with total cost "
                      << solution.getCostsMember(i) << std::endl;
        }
    }

    /* Output the solution to output.txt */

    return 0;
}
