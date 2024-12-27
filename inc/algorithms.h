#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "graph.h"
#include <vector>
#include <string>
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>

using namespace std;

/**
 * @class Algorithms
 * @brief Contains pathfinding algorithms and utility functions for graph traversal.
 */
class Algorithms
{
public:
    /**
     * @brief Calculates the Euclidean distance between two nodes.
     * @param node1 The first node.
     * @param node2 The second node.
     * @return The distance between node1 and node2.
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
     * @brief Displays the path and its details.
     * @param path The path to display as a vector of node IDs.
     * @param algorithm The algorithm used to find the path.
     * @param graph The graph containing the path.
     */
    static void displayPath(const vector<string> &path, const string &algorithm, const Graph &graph);

    /**
     * @brief Calculates the total distance of a path.
     * @param path The path as a vector of node IDs.
     * @param graph The graph containing the path.
     * @return The total distance of the path.
     */
    static double totalDistance(const vector<string> &path, const Graph &graph);

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
};

#endif
