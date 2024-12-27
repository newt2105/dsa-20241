#include "algorithms.h"
#include <vector>
#include <string>
#include <cmath>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <iostream>
#include <iomanip>

using namespace std;

/**
 * @brief Calculates the great-circle distance between two geographical points using the Haversine formula
 * @param node1 First geographical point (latitude/longitude)
 * @param node2 Second geographical point (latitude/longitude)
 * @return Distance in kilometers between the two points
 */
double Algorithms::calculateDistance(const Node &node1, const Node &node2)
{
    const double R = 6371.0;
    double lat1 = node1.x * M_PI / 180.0;
    double lat2 = node2.x * M_PI / 180.0;
    double dLat = (node2.x - node1.x) * M_PI / 180.0;
    double dLon = (node2.y - node1.y) * M_PI / 180.0;

    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1) * cos(lat2) *
                   sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
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
    auto nodes = graph.getNodes();
    auto adjacencyList = graph.getAdjacencyList();

    unordered_map<string, double> distances;
    unordered_map<string, string> previous;
    priority_queue<pair<double, string>,
                   vector<pair<double, string>>,
                   greater<>>
        pq;

    for (const auto &node : nodes)
    {
        distances[node.id] = numeric_limits<double>::infinity();
    }
    distances[start] = 0;
    pq.emplace(0, start);

    while (!pq.empty())
    {
        auto [currentDistance, current] = pq.top();
        pq.pop();

        if (current == end)
            break;

        if (adjacencyList.find(current) != adjacencyList.end())
        {
            for (const auto &edge : adjacencyList.at(current))
            {
                string next = edge.to.id;
                double newDist = distances[current] + edge.weight;

                if (newDist < distances[next])
                {
                    distances[next] = newDist;
                    previous[next] = current;
                    pq.emplace(newDist, next);
                }
            }
        }
    }

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

    unordered_map<string, double> gScore, fScore;
    unordered_map<string, string> previous;
    priority_queue<pair<double, string>,
                   vector<pair<double, string>>,
                   greater<>>
        openSet;

    for (const auto &node : nodes)
    {
        gScore[node.id] = numeric_limits<double>::infinity();
        fScore[node.id] = numeric_limits<double>::infinity();
    }

    gScore[start] = 0;
    fScore[start] = calculateDistance(startNode, endNode);
    openSet.emplace(fScore[start], start);

    while (!openSet.empty())
    {
        auto [currentFScore, current] = openSet.top();
        openSet.pop();

        if (current == end)
            break;

        if (adjacencyList.find(current) != adjacencyList.end())
        {
            for (const auto &edge : adjacencyList.at(current))
            {
                string next = edge.to.id;
                double tentativeGScore = gScore[current] + edge.weight;

                if (tentativeGScore < gScore[next])
                {
                    previous[next] = current;
                    gScore[next] = tentativeGScore;
                    fScore[next] = gScore[next] + calculateDistance(edge.to, endNode);
                    openSet.emplace(fScore[next], next);
                }
            }
        }
    }

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
    unordered_set<string> visited;
    vector<string> path = {start};
    vector<string> finalPath;

    if (dfsHelper(graph, start, end, visited, path, finalPath))
    {
        return finalPath;
    }
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

/**
 * @brief Displays path information including total distance
 * @param path Vector of node IDs representing the path
 * @param algorithm Name of the algorithm used to find the path
 * @param graph Graph containing nodes and edges
 */
void Algorithms::displayPath(const vector<string> &path, const string &algorithm, const Graph &graph)
{
    if (path.empty())
    {
        cout << algorithm << " found no valid path!" << endl;
        return;
    }

    try
    {
        double totalDist = totalDistance(path, graph);

        cout << algorithm << " path: ";
        for (size_t i = 0; i < path.size(); ++i)
        {
            cout << path[i];
            if (i < path.size() - 1)
                cout << " -> ";
        }
        cout << "\nTotal distance: " << fixed << setprecision(1) << totalDist << " km" << endl;
    }
    catch (const exception &e)
    {
        cout << algorithm << " error: " << e.what() << endl;
    }
}
