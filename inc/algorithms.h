#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "graph.h"
#include <vector>
#include <string>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>

class Algorithms
{
public:
    static double calculateDistance(const Node &node1, const Node &node2);
    static std::vector<std::string> dijkstra(const Graph &graph, const std::string &start, const std::string &end);
    static std::vector<std::string> astar(const Graph &graph, const std::string &start, const std::string &end);
    static void displayPath(const std::vector<std::string> &path, const std::string &algorithm, const Graph &graph);

private:
    struct CompareDistance
    {
        bool operator()(std::pair<std::string, double> const &p1,
                        std::pair<std::string, double> const &p2)
        {
            return p1.second > p2.second;
        }
    };
};

#endif
