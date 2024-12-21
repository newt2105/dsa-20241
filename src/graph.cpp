#include "graph.h"
#include <algorithm>
#include <iostream>
#include <fstream>
#include <cmath>
#include <filesystem>

using namespace std;

Graph::Graph()
{
    setupMap();
    draw();
}

void Graph::addNode(const string &id, double x, double y)
{
    Node node = {id, x, y};
    nodes.push_back(node);
}

void Graph::addEdge(const string &fromId, const string &toId, bool isOneWay, double weight)
{
    auto fromNode = find_if(nodes.begin(), nodes.end(),
                            [fromId](const Node &node)
                            { return node.id == fromId; });
    auto toNode = find_if(nodes.begin(), nodes.end(),
                          [toId](const Node &node)
                          { return node.id == toId; });

    if (fromNode != nodes.end() && toNode != nodes.end())
    {
        Edge edge(isOneWay, weight, *fromNode, *toNode);
        edges.push_back(edge);
        adjacencyList[fromId].push_back(edge);

        if (!isOneWay)
        {
            Edge reverseEdge(isOneWay, weight, *toNode, *fromNode);
            adjacencyList[toId].push_back(reverseEdge);
        }
    }
}

vector<Node> Graph::getNodes() const
{
    return nodes;
}

vector<Edge> Graph::getEdges() const
{
    return edges;
}

map<string, vector<Edge>> Graph::getAdjacencyList() const
{
    return adjacencyList;
}

void Graph::displayGraph()
{
    cout << "Hanoi Map Graph:\n\nNodes:" << endl;
    for (const auto &node : nodes)
    {
        cout << node.id << " (" << node.x << ", " << node.y << ")" << endl;
    }

    cout << "\nEdges:" << endl;
    for (const auto &edge : edges)
    {
        cout << edge.from.id << " -> " << edge.to.id;
        cout << " (Weight: " << edge.weight << "km)";
        cout << (edge.direction ? " (One-way)" : " (Two-way)") << endl;
    }
}

void Graph::setupMap()
{
    addNode("HoanKiem", 21.0285, 105.8542);
    addNode("WestLake", 21.0587, 105.8229);
    addNode("LongBien", 21.0437, 105.8625);
    addNode("CauGiay", 21.0359, 105.7929);
    addNode("ThanhXuan", 21.0071, 105.8135);
    addNode("HaDong", 20.9718, 105.7852);

    addEdge("HoanKiem", "WestLake", true, 5.2);
    addEdge("WestLake", "CauGiay", true, 4.1);
    addEdge("LongBien", "HoanKiem", true, 3.8);
    addEdge("CauGiay", "ThanhXuan", false, 4.5);
    addEdge("ThanhXuan", "HaDong", true, 6.3);
    addEdge("HoanKiem", "ThanhXuan", false, 5.0);
    addEdge("LongBien", "WestLake", false, 4.7);
}

void Graph::draw()
{
    const string outputDir = "../../results/";
    filesystem::create_directories(outputDir);

    ofstream dotFile(outputDir + "hanoi_map.dot");

    dotFile << "digraph HanoiMap {\n";
    dotFile << "    rankdir=LR;\n";
    dotFile << "    node [shape=circle, style=filled, fillcolor=lightblue];\n";

    for (const auto &node : nodes)
    {
        dotFile << "    " << node.id << " [label=\"" << node.id << "\\n("
                << node.x << ", " << node.y << ")\"]\n";
    }

    for (const auto &edge : edges)
    {
        dotFile << "    " << edge.from.id << " -> " << edge.to.id
                << " [label=\"" << edge.weight << "km\"";

        if (edge.direction)
        {
            dotFile << ", color=black, fontcolor=black";
        }
        else
        {
            dotFile << ", dir=both, color=blue, fontcolor=blue";
        }

        dotFile << "]\n";
    }

    dotFile << "}\n";
    dotFile.close();

    string command = "dot -Tpng " + outputDir + "hanoi_map.dot -o " + outputDir + "hanoi_map.png";
    system(command.c_str());
}
