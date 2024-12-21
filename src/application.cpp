#include "application.h"
#include <iostream>
#include <limits>
#include <algorithm>

using namespace std;

Application::Application() : firstRun(true) {}

void Application::clearScreen()
{
#ifdef _WIN32
    system("cls");
#else
    system("clear");
#endif
}

void Application::waitForEnter()
{
    cout << "\nPress Enter to return to the main menu...";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
}

void Application::clearInputBuffer()
{
    cin.clear();
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
}

void Application::displayMenu()
{
    clearScreen();
    cout << "\n====== Hanoi Map Pathfinding System ======\n";
    cout << "1. Display map information\n";
    cout << "2. Find path using Dijkstra's algorithm\n";
    cout << "3. Find path using A* algorithm\n";
    cout << "4. Compare both algorithms\n";
    cout << "5. Exit\n";
    cout << "=========================================\n";
    cout << "Enter your choice (1-5): ";
}

void Application::displayAvailableLocations()
{
    cout << "\nAvailable locations:\n";
    vector<Node> nodes = hanoiMap.getNodes();
    for (const auto &node : nodes)
    {
        cout << "- " << node.id << endl;
    }
}

bool Application::isValidLocation(const string &location)
{
    vector<Node> nodes = hanoiMap.getNodes();
    return find_if(nodes.begin(), nodes.end(),
                   [&location](const Node &node)
                   {
                       return node.id == location;
                   }) != nodes.end();
}

pair<string, string> Application::getSourceAndDestination()
{
    string source, destination;

    displayAvailableLocations();

    while (true)
    {
        cout << "\nEnter starting location: ";
        getline(cin, source);

        if (isValidLocation(source))
        {
            break;
        }
        cout << "Invalid location! Please choose from the available locations.\n";
    }

    while (true)
    {
        cout << "Enter destination: ";
        getline(cin, destination);

        if (isValidLocation(destination))
        {
            if (source != destination)
            {
                break;
            }
            cout << "Destination cannot be the same as the starting location!\n";
        }
        else
        {
            cout << "Invalid location! Please choose from the available locations.\n";
        }
    }

    return make_pair(source, destination);
}

void Application::findPath(const string &algorithm, const string &source, const string &destination)
{
    vector<string> path;

    if (algorithm == "Dijkstra")
    {
        path = Algorithms::dijkstra(hanoiMap, source, destination);
    }
    else if (algorithm == "A*")
    {
        path = Algorithms::astar(hanoiMap, source, destination);
    }

    Algorithms::displayPath(path, algorithm, hanoiMap);
}

void Application::handleChoice(int choice)
{
    clearScreen();

    switch (choice)
    {
    case 1:
    {
        cout << "\n=== Map Information ===\n";
        hanoiMap.displayGraph();
        break;
    }

    case 2:
    {
        cout << "\n=== Dijkstra's Algorithm Pathfinding ===\n";
        auto [source, destination] = getSourceAndDestination();
        findPath("Dijkstra", source, destination);
        break;
    }

    case 3:
    {
        cout << "\n=== A* Algorithm Pathfinding ===\n";
        auto [source, destination] = getSourceAndDestination();
        findPath("A*", source, destination);
        break;
    }

    case 4:
    {
        cout << "\n=== Algorithm Comparison ===\n";
        auto [source, destination] = getSourceAndDestination();
        cout << "\nResults:\n";
        findPath("Dijkstra", source, destination);
        findPath("A*", source, destination);
        break;
    }

    case 5:
    {
        cout << "\nThank you for using the Hanoi Map Pathfinding System!\n";
        exit(0);
    }

    default:
    {
        cout << "Invalid choice! Please enter a number between 1 and 5.\n";
        break;
    }
    }
}

void Application::run()
{
    string input;
    int choice;

    while (true)
    {
        if (!firstRun)
        {
            waitForEnter();
        }
        firstRun = false;

        displayMenu();
        getline(cin, input);

        try
        {
            choice = stoi(input);
        }
        catch (...)
        {
            clearScreen();
            cout << "Invalid input! Please enter a number between 1 and 5.\n";
            continue;
        }

        handleChoice(choice);
    }
}
