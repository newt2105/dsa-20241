#include "graph.h"
#include "ultis.h"
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <set>
#include <filesystem>

using namespace std;

/**
 * @brief Constructor for the Graph class. Initializes the graph by loading data from a file and drawing the graph.
 */
Graph::Graph()
{
    loadFromFile(DATA_FILE_PATH);
    draw();
}

/**
 * @brief Adds a new node to the graph.
 * @param id Unique identifier for the node.
 * @param x Latitude coordinate of the node.
 * @param y Longitude coordinate of the node.
 */
void Graph::addNode(const string &id, double x, double y)
{
    Node node = {id, x, y};
    nodes.push_back(node);
}

/**
 * @brief Adds a new edge between two nodes in the graph.
 * @param fromId Identifier of the source node.
 * @param toId Identifier of the destination node.
 * @param isOneWay Specifies if the edge is one-way or bidirectional.
 * @param weight Weight of the edge (e.g., distance between nodes).
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
 * @brief Retrieves all nodes in the graph.
 * @return A vector containing all nodes.
 */
vector<Node> Graph::getNodes() const
{
    return nodes;
}

/**
 * @brief Retrieves all edges in the graph.
 * @return A vector containing all edges.
 */
vector<Edge> Graph::getEdges() const
{
    return edges;
}

/**
 * @brief Retrieves the adjacency list representation of the graph.
 * @return A map where the keys are node IDs and the values are vectors of adjacent edges.
 */
map<string, vector<Edge>> Graph::getAdjacencyList() const
{
    return adjacencyList;
}

/**
 * @brief Displays the graph's nodes and edges in a readable format on the console.
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
 * @brief Generates a DOT file and a PNG visual representation of the graph.
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

/**
 * @brief Loads graph data from a specified file. Expects sections "Nodes" and "Edges" with specific formatting.
 * @param filePath Path to the file containing the graph data.
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
 * @brief Saves the current graph data to a file in the specified format.
 */
void Graph::saveToFile() const
{
    std::ofstream outFile(DATA_FILE_PATH);
    if (!outFile.is_open())
    {
        std::cerr << "Error: Unable to open file for writing: " << DATA_FILE_PATH << std::endl;
        return;
    }

    outFile << "Nodes\n";
    for (const auto &node : nodes)
    {
        outFile << node.id << " " << node.x << " " << node.y << "\n";
    }

    outFile << "\nEdges\n";

    std::set<std::pair<std::string, std::string>> processedEdges;

    for (const auto &edge : edges)
    {
        std::string fromId = edge.from.id;
        std::string toId = edge.to.id;

        auto edgeKey = std::make_pair(std::min(fromId, toId), std::max(fromId, toId));

        if (processedEdges.count(edgeKey) > 0)
            continue;

        bool isTwoWay = false;
        for (const auto &reverseEdge : edges)
        {
            if (reverseEdge.from.id == toId && reverseEdge.to.id == fromId &&
                !reverseEdge.direction && !edge.direction)
            {
                isTwoWay = true;
                break;
            }
        }

        outFile << fromId << " " << toId << " "
                << edge.weight << " " << (isTwoWay ? "2" : (edge.direction ? "1" : "0")) << "\n";

        processedEdges.insert(edgeKey);
    }

    outFile.close();
}
