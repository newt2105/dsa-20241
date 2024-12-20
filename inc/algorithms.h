#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include <queue>
#include <unordered_map>
#include <limits>
#include <cmath>
#include "graph.h"
using namespace std;

class Algorithms
{
private:
    struct CompareDistance
    {
        bool operator()(pair<string, double> const &p1,
                        pair<string, double> const &p2);
    };

public:
    static double calculateDistance(const Node &node1, const Node &node2);
    static vector<string> dijkstra(const Graph &graph,
                                   const string &start,
                                   const string &end);
    static vector<string> astar(const Graph &graph,
                                const string &start,
                                const string &end);
    static void displayPath(const vector<string> &path,
                            const string &algorithm);
};

#endif /* ALGORITHMS_H */