#ifndef GRAPH_H
#define GRAPH_H

#include <string>
#include <vector>
#include <map>

using namespace std;

struct Node
{
    string id;
    double x, y;
};

struct Edge
{
    bool direction; // true = one-way, false = two-way
    double weight;
    Node from;
    Node to;
    Edge(bool direction, double weight, const Node &from, const Node &to)
        : direction(direction), weight(weight), from(from), to(to) {}
};

class Graph
{
public:
    Graph();
    void addNode(const string &id, double x, double y);
    void addEdge(const string &fromId, const string &toId, bool isOneWay, double weight);
    vector<Node> getNodes() const;
    vector<Edge> getEdges() const;
    map<string, vector<Edge>> getAdjacencyList() const;
    void displayGraph();

private:
    vector<Node> nodes;
    vector<Edge> edges;
    map<string, vector<Edge>> adjacencyList;

    void setupMap();
    void draw();
};

#endif
