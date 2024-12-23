#include "graph.h"
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <filesystem>

using namespace std;

/**
 * @brief Constructor for Graph class
 */
Graph::Graph()
{
    loadFromFile("/home/dihnhuunam/Workspace/dsa-20241/data.txt");
    draw();
}

/**
 * @brief Adds a new node to the graph
 * @param id Node identifier
 * @param x Latitude coordinate
 * @param y Longitude coordinate
 */
void Graph::addNode(const string &id, double x, double y)
{
    Node node = {id, x, y};
    nodes.push_back(node);
}

/**
 * @brief Adds a new edge to the graph
 * @param fromId Source node ID
 * @param toId Destination node ID
 * @param isOneWay True if edge is one-way, false if bidirectional
 * @param weight Edge weight (distance)
 */
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

/**
 * @brief Gets all nodes in the graph
 * @return Vector of all nodes
 */
vector<Node> Graph::getNodes() const
{
    return nodes;
}

/**
 * @brief Gets all edges in the graph
 * @return Vector of all edges
 */
vector<Edge> Graph::getEdges() const
{
    return edges;
}

/**
 * @brief Gets the adjacency list representation of the graph
 * @return Map of node IDs to their adjacent edges
 */
map<string, vector<Edge>> Graph::getAdjacencyList() const
{
    return adjacencyList;
}

/**
 * @brief Displays graph information to console
 */
void Graph::displayGraph()
{
    if (nodes.empty())
    {
        cout << "Warning: No nodes loaded\n";
        return;
    }

    cout << "Nodes:\n";
    for (const auto &node : nodes)
    {
        cout << "- " << node.id << " (" << node.x << ", " << node.y << ")\n";
    }

    if (edges.empty())
    {
        cout << "Warning: No edges loaded\n";
        return;
    }

    cout << "\nEdges:\n";
    for (const auto &edge : edges)
    {
        cout << edge.from.id << " -> " << edge.to.id
             << " (Weight: " << edge.weight
             << ", Direction: " << (edge.direction ? "One-way" : "Two-way") << ")\n";
    }
}

/**
 * @brief Loads graph data from a file
 * @param filePath Path to the input file
 */
void Graph::loadFromFile(const string &filePath)
{
    ifstream inputFile(filePath);

    if (!inputFile.is_open())
    {
        cerr << "Error: Unable to open file " << filePath << endl;
        return;
    }

    string line, section;
    while (getline(inputFile, line))
    {
        // Bỏ qua dòng trống hoặc dòng chú thích
        if (line.empty() || line[0] == '#')
            continue;

        if (line == "Nodes")
        {
            section = "Nodes";
            continue;
        }
        else if (line == "Edges")
        {
            section = "Edges";
            continue;
        }

        if (section == "Nodes")
        {
            string id;
            double x, y;
            istringstream iss(line);
            if (!(iss >> id >> x >> y))
            {
                cerr << "Error parsing Node: " << line << endl;
                continue;
            }
            addNode(id, x, y);
        }
        else if (section == "Edges")
        {
            string fromId, toId;
            double weight;
            int direction;
            istringstream iss(line);
            if (!(iss >> fromId >> toId >> weight >> direction))
            {
                cerr << "Error parsing Edge: " << line << endl;
                continue;
            }
            addEdge(fromId, toId, direction == 1, weight);
        }
    }

    inputFile.close();
}

/**
 * @brief Generates a visual representation of the graph
 */
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
    int dot = system(command.c_str());
}
