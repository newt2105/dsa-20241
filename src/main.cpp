#include "algorithms.h"
#include "graph.h"
using namespace std;

int main()
{
    Graph hanoiMap;

    // Display the graph structure
    hanoiMap.displayGraph();

    // Test pathfinding algorithms
    cout << "\nFinding paths from HoanKiem to HaDong:\n"
         << endl;

    // Test Dijkstra's algorithm
    vector<string> dijkstraPath = Algorithms::dijkstra(hanoiMap, "HoanKiem", "HaDong");
    Algorithms::displayPath(dijkstraPath, "Dijkstra");

    // Test A* algorithm
    vector<string> astarPath = Algorithms::astar(hanoiMap, "HoanKiem", "HaDong");
    Algorithms::displayPath(astarPath, "A*");

    return 0;
}