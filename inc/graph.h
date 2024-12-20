#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <fstream>
#include <sstream>
using namespace std;

struct Node
{
    std::string id;
    double x;
    double y;
};

struct Edge
{
    bool direction;
    double weight;
    double from;
    double to;

    Edge(bool dir, double w, double f, double t)
        : direction(dir), weight(w), from(f), to(t) {}
};

class Graph
{
private:
    vector<Node> nodes;
    vector<Edge> edges;
    map<string, vector<Edge>> adjacencyList;

    void draw();
    void setupMap();

public:
    Graph();
    void addNode(string id, double x, double y);
    void addEdge(string fromId, string toId, bool isOneWay, double weight);
    vector<Node> getNodes() const;
    vector<Edge> getEdges() const;
    map<string, vector<Edge>> getAdjacencyList() const;
    void displayGraph();
};

#endif /* GRAPH_H */