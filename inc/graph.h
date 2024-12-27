#ifndef GRAPH_H
#define GRAPH_H

#include <string>
#include <vector>
#include <map>

/**
 * @struct Node
 * @brief Represents a location on the graph with an ID and coordinates.
 *
 * Attributes:
 * @param id Unique identifier for the node.
 * @param x Latitude (x-coordinate) of the node.
 * @param y Longitude (y-coordinate) of the node.
 */
struct Node
{
    std::string id;
    double x;
    double y;
};

/**
 * @struct Edge
 * @brief Represents a connection between two nodes.
 *
 * Attributes:
 * @param direction Specifies if the edge is one-way (true) or two-way (false).
 * @param weight The weight (e.g., distance) of the edge.
 * @param from The starting node of the edge.
 * @param to The destination node of the edge.
 */
struct Edge
{
    bool direction;
    double weight;
    Node from;
    Node to;
    /**
     * @brief Constructs an Edge object.
     * @param dir Direction of the edge.
     * @param w Weight of the edge.
     * @param f Starting node of the edge.
     * @param t Destination node of the edge.
     */
    Edge(bool dir, double w, const Node &f, const Node &t)
        : direction(dir), weight(w), from(f), to(t) {}
};

/**
 * @class Graph
 * @brief Represents a graph structure with nodes and edges.
 *
 * The graph supports operations such as adding nodes, adding edges,
 * retrieving nodes and edges, displaying the graph, and file operations.
 */
class Graph
{
private:
    std::vector<Node> nodes;
    std::vector<Edge> edges;
    std::map<std::string, std::vector<Edge>> adjacencyList;

public:
    /**
     * @brief Constructs an empty Graph object.
     */
    Graph();

    /**
     * @brief Adds a node to the graph.
     * @param id Unique identifier for the node.
     * @param x Latitude (x-coordinate) of the node.
     * @param y Longitude (y-coordinate) of the node.
     */
    void addNode(const std::string &id, double x, double y);

    /**
     * @brief Adds an edge to the graph.
     * @param fromId ID of the starting node.
     * @param toId ID of the destination node.
     * @param isOneWay Specifies if the edge is one-way (true) or two-way (false).
     * @param weight Weight (e.g., distance) of the edge.
     */
    void addEdge(const std::string &fromId, const std::string &toId, bool isOneWay, double weight);

    /**
     * @brief Retrieves all nodes in the graph.
     * @return A vector containing all nodes.
     */
    std::vector<Node> getNodes() const;

    /**
     * @brief Retrieves all edges in the graph.
     * @return A vector containing all edges.
     */
    std::vector<Edge> getEdges() const;

    /**
     * @brief Retrieves the adjacency list of the graph.
     * @return A map representing the adjacency list.
     */
    std::map<std::string, std::vector<Edge>> getAdjacencyList() const;

    /**
     * @brief Displays the graph in a human-readable format.
     */
    void displayGraph();

    /**
     * @brief Visualizes the graph (implementation-specific).
     */
    void draw();

    /**
     * @brief Loads graph data from a file.
     * @param filePath Path to the file containing the graph data.
     */
    void loadFromFile(const std::string &filePath);

    /**
     * @brief Saves the current graph data to a file.
     * @param filePath Path to the file where the graph data will be saved.
     */
    void saveToFile() const;
};

#endif
