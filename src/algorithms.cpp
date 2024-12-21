// algorithms.cpp
#include "algorithms.h"
#include <vector>
#include <string>
#include <cmath>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <iostream>
#include <iomanip>

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

std::vector<std::string> Algorithms::dijkstra(const Graph &graph, const std::string &start, const std::string &end)
{
    auto nodes = graph.getNodes();
    auto adjacencyList = graph.getAdjacencyList();

    std::unordered_map<std::string, double> distances;
    std::unordered_map<std::string, std::string> previous;
    std::priority_queue<std::pair<double, std::string>,
                        std::vector<std::pair<double, std::string>>,
                        std::greater<>>
        pq;

    for (const auto &node : nodes)
    {
        distances[node.id] = std::numeric_limits<double>::infinity();
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
                std::string next = edge.to.id;
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

    std::vector<std::string> path;
    for (std::string current = end; current != start; current = previous[current])
    {
        if (previous.find(current) == previous.end())
        {
            return {}; // No path found
        }
        path.push_back(current);
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());

    return path;
}

std::vector<std::string> Algorithms::astar(const Graph &graph, const std::string &start, const std::string &end)
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

    std::unordered_map<std::string, double> gScore, fScore;
    std::unordered_map<std::string, std::string> previous;
    std::priority_queue<std::pair<double, std::string>,
                        std::vector<std::pair<double, std::string>>,
                        std::greater<>>
        openSet;

    for (const auto &node : nodes)
    {
        gScore[node.id] = std::numeric_limits<double>::infinity();
        fScore[node.id] = std::numeric_limits<double>::infinity();
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
                std::string next = edge.to.id;
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

    std::vector<std::string> path;
    for (std::string current = end; current != start; current = previous[current])
    {
        if (previous.find(current) == previous.end())
        {
            return {}; // No path found
        }
        path.push_back(current);
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());

    return path;
}

void Algorithms::displayPath(const std::vector<std::string> &path, const std::string &algorithm, const Graph &graph)
{
    if (path.empty())
    {
        std::cout << algorithm << " found no valid path!" << std::endl;
        return;
    }

    double totalDistance = 0.0;
    auto adjacencyList = graph.getAdjacencyList();

    for (size_t i = 0; i < path.size() - 1; ++i)
    {
        std::string current = path[i];
        std::string next = path[i + 1];
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
            std::cout << algorithm << " error: No valid edge between " << current << " and " << next << std::endl;
            return;
        }
    }

    std::cout << algorithm << " path: ";
    for (size_t i = 0; i < path.size(); ++i)
    {
        std::cout << path[i];
        if (i < path.size() - 1)
            std::cout << " -> ";
    }
    std::cout << "\nTotal distance: " << std::fixed << std::setprecision(1) << totalDistance << " km" << std::endl;
}
