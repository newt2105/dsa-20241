#include "algorithms.h"

bool Algorithms::CompareDistance::operator()(pair<string, double> const &p1,
                                             pair<string, double> const &p2)
{
    return p1.second > p2.second;
}

double Algorithms::calculateDistance(const Node &node1, const Node &node2)
{
    const double R = 6371.0;
    double lat1 = node1.x * M_PI / 180.0;
    double lat2 = node2.x * M_PI / 180.0;
    double dLat = (node2.x - node1.x) * M_PI / 180.0;
    double dLon = (node2.y - node1.y) * M_PI / 180.0;

    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1) * cos(lat2) *
                   sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}

vector<string> Algorithms::dijkstra(const Graph &graph,
                                    const string &start,
                                    const string &end)
{
    auto nodes = graph.getNodes();
    auto adjacencyList = graph.getAdjacencyList();

    unordered_map<string, double> distances;
    unordered_map<string, string> previous;
    priority_queue<pair<string, double>,
                   vector<pair<string, double>>,
                   CompareDistance>
        pq;

    for (const auto &node : nodes)
    {
        distances[node.id] = numeric_limits<double>::infinity();
    }
    distances[start] = 0;
    pq.push({start, 0});

    while (!pq.empty())
    {
        string current = pq.top().first;
        pq.pop();

        if (current == end)
            break;

        if (adjacencyList.find(current) != adjacencyList.end())
        {
            for (const auto &edge : adjacencyList.at(current))
            {
                string next = nodes[edge.to].id;
                double newDist = distances[current] + edge.weight;

                if (newDist < distances[next])
                {
                    distances[next] = newDist;
                    previous[next] = current;
                    pq.push({next, newDist});
                }
            }
        }
    }

    vector<string> path;
    string current = end;
    while (current != start)
    {
        if (previous.find(current) == previous.end())
        {
            return vector<string>();
        }
        path.push_back(current);
        current = previous[current];
    }
    path.push_back(start);
    reverse(path.begin(), path.end());

    return path;
}

vector<string> Algorithms::astar(const Graph &graph,
                                 const string &start,
                                 const string &end)
{
    auto nodes = graph.getNodes();
    auto adjacencyList = graph.getAdjacencyList();

    unordered_map<string, double> gScore;
    unordered_map<string, double> fScore;
    unordered_map<string, string> previous;
    priority_queue<pair<string, double>,
                   vector<pair<string, double>>,
                   CompareDistance>
        openSet;

    for (const auto &node : nodes)
    {
        gScore[node.id] = numeric_limits<double>::infinity();
        fScore[node.id] = numeric_limits<double>::infinity();
    }

    Node startNode, endNode;
    for (const auto &node : nodes)
    {
        if (node.id == start)
            startNode = node;
        if (node.id == end)
            endNode = node;
    }

    gScore[start] = 0;
    fScore[start] = calculateDistance(startNode, endNode);
    openSet.push({start, fScore[start]});

    while (!openSet.empty())
    {
        string current = openSet.top().first;
        openSet.pop();

        if (current == end)
            break;

        if (adjacencyList.find(current) != adjacencyList.end())
        {
            for (const auto &edge : adjacencyList.at(current))
            {
                string next = nodes[edge.to].id;
                double tentativeGScore = gScore[current] + edge.weight;

                if (tentativeGScore < gScore[next])
                {
                    previous[next] = current;
                    gScore[next] = tentativeGScore;

                    Node nextNode;
                    for (const auto &node : nodes)
                    {
                        if (node.id == next)
                        {
                            nextNode = node;
                            break;
                        }
                    }

                    fScore[next] = gScore[next] + calculateDistance(nextNode, endNode);
                    openSet.push({next, fScore[next]});
                }
            }
        }
    }

    vector<string> path;
    string current = end;
    while (current != start)
    {
        if (previous.find(current) == previous.end())
        {
            return vector<string>();
        }
        path.push_back(current);
        current = previous[current];
    }
    path.push_back(start);
    reverse(path.begin(), path.end());

    return path;
}

void Algorithms::displayPath(const vector<string> &path,
                             const string &algorithm)
{
    if (path.empty())
    {
        cout << "No path found using " << algorithm << "!" << endl;
        return;
    }

    cout << algorithm << " path: ";
    for (size_t i = 0; i < path.size(); ++i)
    {
        cout << path[i];
        if (i < path.size() - 1)
            cout << " -> ";
    }
    cout << endl;
}