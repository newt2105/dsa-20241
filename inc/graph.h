#ifndef GRAPH_H
#define GRAPH_H

#include <string>
#include <vector>
#include <map>

struct Node {
    std::string id;
    double x, y;
};

struct Edge {
    bool direction; // true = one-way, false = two-way
    double weight;
    Node from; // Changed from index to Node
    Node to;   // Changed from index to Node

    Edge(bool direction, double weight, const Node &from, const Node &to)
        : direction(direction), weight(weight), from(from), to(to) {}
};

class Graph {
public:
    Graph();
    void addNode(const std::string &id, double x, double y);
    void addEdge(const std::string &fromId, const std::string &toId, bool isOneWay, double weight);
    std::vector<Node> getNodes() const;
    std::vector<Edge> getEdges() const;
    std::map<std::string, std::vector<Edge>> getAdjacencyList() const;
    void displayGraph();

private:
    std::vector<Node> nodes;
    std::vector<Edge> edges;
    std::map<std::string, std::vector<Edge>> adjacencyList;

    void setupMap();
    void draw();
};

#endif
