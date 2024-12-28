#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "graph.h"
#include <vector>
#include <string>
#include <chrono>

using namespace std;

/**
 * @class Algorithms
 * @brief Contains pathfinding algorithms and utility functions for graph traversal.
 */
class Algorithms
{
public:
    /**
     * @brief Calculates the great-circle distance between two geographical points using the Haversine formula
     * @param node1 First geographical point (latitude/longitude)
     * @param node2 Second geographical point (latitude/longitude)
     * @return Distance in kilometers between the two points
     */
    static double calculateDistance(const Node &node1, const Node &node2);

    /**
     * @brief Finds the shortest path using Dijkstra's algorithm.
     * @param graph The graph to traverse.
     * @param start The starting node ID.
     * @param end The destination node ID.
     * @return A vector of node IDs representing the shortest path.
     */
    static vector<string> dijkstra(const Graph &graph, const string &start, const string &end);

    /**
     * @brief Finds the shortest path using the A* algorithm.
     * @param graph The graph to traverse.
     * @param start The starting node ID.
     * @param end The destination node ID.
     * @return A vector of node IDs representing the shortest path.
     */
    static vector<string> astar(const Graph &graph, const string &start, const string &end);

    /**
     * @brief Finds a path using Depth-First Search (DFS).
     * @param graph The graph to traverse.
     * @param start The starting node ID.
     * @param end The destination node ID.
     * @return A vector of node IDs representing the path.
     */
    static vector<string> dfs(const Graph &graph, const string &start, const string &end);

    /**
     * @brief Calculates the total distance of a path.
     * @param path The path as a vector of node IDs.
     * @param graph The graph containing the path.
     * @return The total distance of the path.
     */
    static double totalDistance(const vector<string> &path, const Graph &graph);

    /**
     * @brief Gets the execution time of the last run algorithm
     * @return execution time
     */
    static double getLastExecutionTime();

private:
    /**
     * @brief A comparator for priority queues used in pathfinding algorithms.
     */
    struct CompareDistance
    {
        bool operator()(pair<string, double> const &p1, pair<string, double> const &p2)
        {
            return p1.second > p2.second;
        }
    };

    static double lastExecutionTime;
};

#endif
