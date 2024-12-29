#include "algorithms.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <limits>
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <unordered_set>

double Algorithms::lastExecutionTime = 0.0;

double Algorithms::getLastExecutionTime()
{
    return lastExecutionTime;
}

/**
 * @brief Calculates the great-circle distance between two geographical points using the Euclidean formula
 * @param node1 First geographical point (latitude/longitude)
 * @param node2 Second geographical point (latitude/longitude)
 * @return Distance in kilometers between the two points
 */
double Algorithms::calculateDistance(const Node &node1, const Node &node2)
{
    return sqrt((node2.x - node1.x) * (node2.x - node1.x) + (node2.y - node1.y) * (node2.y - node1.y));
}

/**
 * @brief Implements Dijkstra's shortest path algorithm
 * @param graph Graph containing nodes and edges
 * @param start ID of the starting node
 * @param end ID of the destination node
 * @return Vector of node IDs representing the shortest path, empty if no path exists
 */
vector<string> Algorithms::dijkstra(const Graph &graph, const string &start, const string &end)
{
    auto startTime = chrono::high_resolution_clock::now();

    // Retrieve nodes and adjacency list from the graph
    auto nodes = graph.getNodes();
    auto adjacencyList = graph.getAdjacencyList();

    // Initialize distances to all nodes as infinity and the start node's distance to 0
    unordered_map<string, double> distances;
    unordered_map<string, string> previous;                                           // To store the previous node for path reconstruction
    priority_queue<pair<double, string>, vector<pair<double, string>>, greater<>> pq; // Min-heap for selecting the closest node {weight, id}

    for (const auto &node : nodes)
    {
        distances[node.id] = numeric_limits<double>::infinity();
    }
    distances[start] = 0;
    pq.emplace(0, start); // Push the start node with a distance of 0

    while (!pq.empty())
    {
        // Get the node with the smallest distance
        auto [currentDistance, current] = pq.top();
        pq.pop();

        // Iterate over neighbors of the current node
        if (adjacencyList.find(current) != adjacencyList.end())
        {
            for (const auto &edge : adjacencyList.at(current))
            {
                string next = edge.to.id;
                double newDist = distances[current] + edge.weight; // Calculate new distance

                // If a shorter path is found, update distances and priority queue
                if (newDist < distances[next])
                {
                    distances[next] = newDist;
                    previous[next] = current; // Update previous node for path reconstruction
                    pq.emplace(newDist, next);
                }
            }
        }
    }

    // Path reconstruction
    vector<string> path;
    for (string current = end; current != start; current = previous[current])
    {
        if (previous.find(current) == previous.end())
        {
            return {}; // No path found
        }
        path.push_back(current);
    }
    path.push_back(start);
    reverse(path.begin(), path.end()); // Reverse the path to get it in the correct order

    auto endTime = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(endTime - startTime);
    lastExecutionTime = duration.count() / 1000.0; // Record execution time

    return path;
}

/**
 * @brief Implements A* pathfinding algorithm
 * @param graph Graph containing nodes and edges
 * @param start ID of the starting node
 * @param end ID of the destination node
 * @return Vector of node IDs representing the shortest path, empty if no path exists
 */
vector<string> Algorithms::astar(const Graph &graph, const string &start, const string &end)
{
    auto startTime = chrono::high_resolution_clock::now();

    auto nodes = graph.getNodes();
    auto adjacencyList = graph.getAdjacencyList();

    Node startNode, endNode;
    for (const auto &node : nodes)
    {
        if (node.id == start)
            startNode = node;
        if (node.id == end)
            endNode = node;
    }

    // Initialize gScore (cost from start) and fScore (estimated total cost) for all nodes
    unordered_map<string, double> gScore, fScore;
    unordered_map<string, string> previous;
    priority_queue<pair<double, string>, vector<pair<double, string>>, greater<>> openSet;

    for (const auto &node : nodes)
    {
        gScore[node.id] = numeric_limits<double>::infinity();
        fScore[node.id] = numeric_limits<double>::infinity();
    }

    gScore[start] = 0;
    fScore[start] = calculateDistance(startNode, endNode); // Heuristic: straight-line distance to the target
    openSet.emplace(fScore[start], start);                 // Push the start node

    while (!openSet.empty())
    {
        // Get the node with the lowest estimated cost
        auto [currentfScore, current] = openSet.top();
        openSet.pop();

        if (current == end) // If we reached the target, stop
            break;

        // Iterate over neighbors of the current node
        if (adjacencyList.find(current) != adjacencyList.end())
        {
            for (const auto &edge : adjacencyList.at(current))
            {
                string next = edge.to.id;
                double tentativeGScore = gScore[current] + edge.weight; // Calculate cost to neighbor

                // If this path is better, update the scores and openSet
                if (tentativeGScore < gScore[next])
                {
                    previous[next] = current;
                    gScore[next] = tentativeGScore;
                    fScore[next] = gScore[next] + calculateDistance(edge.to, endNode); // Update heuristic
                    openSet.emplace(fScore[next], next);
                }
            }
        }
    }

    // Path reconstruction
    vector<string> path;
    for (string current = end; current != start; current = previous[current])
    {
        if (previous.find(current) == previous.end())
        {
            return {}; // No path found
        }
        path.push_back(current);
    }
    path.push_back(start);
    reverse(path.begin(), path.end());

    auto endTime = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(endTime - startTime);
    lastExecutionTime = duration.count() / 1000.0; // Record execution time

    return path;
}

/**
 * @brief Helper function for DFS implementation
 * @param graph Graph containing nodes and edges
 * @param current Current node being explored
 * @param end Target node
 * @param visited Set of visited nodes
 * @param path Current path being built
 * @param finalPath Reference to store the final path if found
 * @return true if path is found, false otherwise
 */
bool dfsHelper(const Graph &graph, const string &current, const string &end,
               unordered_set<string> &visited, vector<string> &path,
               vector<string> &finalPath)
{
    if (current == end)
    {
        finalPath = path;
        return true;
    }

    visited.insert(current);
    auto adjacencyList = graph.getAdjacencyList();

    if (adjacencyList.find(current) != adjacencyList.end())
    {
        for (const auto &edge : adjacencyList.at(current))
        {
            string next = edge.to.id;
            if (visited.find(next) == visited.end())
            {
                path.push_back(next);
                if (dfsHelper(graph, next, end, visited, path, finalPath))
                {
                    return true;
                }
                path.pop_back();
            }
        }
    }

    return false;
}

/**
 * @brief Implements Depth-First Search (DFS) algorithm.
 * @param graph Graph containing nodes and edges.
 * @param start ID of the starting node.
 * @param end ID of the destination node.
 * @return Vector of node IDs representing the path, or empty if no path exists.
 */
vector<string> Algorithms::dfs(const Graph &graph, const string &start, const string &end)
{
    auto startTime = chrono::high_resolution_clock::now();
    unordered_set<string> visited;
    vector<string> path = {start};
    vector<string> finalPath;

    if (dfsHelper(graph, start, end, visited, path, finalPath))
    {
        return finalPath;
    }
    auto endTime = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(endTime - startTime);
    lastExecutionTime = duration.count() / 1000.0;

    return vector<string>();
}

/**
 * @brief Calculates total distance of a given path
 * @param path Vector of node IDs representing the path
 * @param graph Graph containing nodes and edges
 * @return Total distance of the path in kilometers
 * @throws runtime_error if no valid edge exists between consecutive nodes
 */
double Algorithms::totalDistance(const vector<string> &path, const Graph &graph)
{
    double totalDistance = 0.0;
    auto adjacencyList = graph.getAdjacencyList();

    for (size_t i = 0; i < path.size() - 1; ++i)
    {
        string current = path[i];
        string next = path[i + 1];
        bool foundEdge = false;

        if (adjacencyList.find(current) != adjacencyList.end())
        {
            for (const auto &edge : adjacencyList.at(current))
            {
                if (edge.to.id == next)
                {
                    totalDistance += edge.weight;
                    foundEdge = true;
                    break;
                }
            }
        }

        if (!foundEdge)
        {
            throw runtime_error("No valid edge between " + current + " and " + next);
        }
    }

    return totalDistance;
}