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

using namespace std;

class Algorithms
{
public:
    static double calculateDistance(const Node &node1, const Node &node2);
    static vector<string> dijkstra(const Graph &graph, const string &start, const string &end);
    static vector<string> astar(const Graph &graph, const string &start, const string &end);
    static void displayPath(const vector<string> &path, const string &algorithm, const Graph &graph);

private:
    struct CompareDistance
    {
        bool operator()(pair<string, double> const &p1,
                        pair<string, double> const &p2)
        {
            return p1.second > p2.second;
        }
    };
};

#endif
