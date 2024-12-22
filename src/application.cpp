#include "application.h"
#include "ultis.h"
#include <iostream>
#include <limits>
#include <algorithm>
#include <iomanip>

using namespace std;

Application::Application() : firstRun(true) {}

void Application::clearScreen()
{
#ifdef _WIN32
    system("cls");
#else
    int clear = system("clear");
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
    ConsoleTable table(1);
    Row header = {"Ha Noi Path Finding System"};
    Row row1 = {"1. Display map information"};
    Row row2 = {"2. Find path using Dijkstra's algorithm"};
    Row row3 = {"3. Find path using A* algorithm"};
    Row row4 = {"4. Compare both algorithms"};
    Row row5 = {"5. Exit"};

    table.AddNewRow(header);
    table.AddNewRow(row1);
    table.AddNewRow(row2);
    table.AddNewRow(row3);
    table.AddNewRow(row4);
    table.AddNewRow(row5);

    table.WriteTable(Align::Center);
    cout << "Enter your choice: ";
}

void Application::displayAvailableLocationsWithHeader(const string &headerText)
{
    ConsoleTable table(1);
    table.AddNewRow({headerText});
    table.AddNewRow({"Available Locations"});

    vector<Node> nodes = hanoiMap.getNodes();
    for (const auto &node : nodes)
    {
        table.AddNewRow({node.id});
    }

    table.WriteTable(Align::Center);
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

pair<string, string> Application::getSourceAndDestinationWithHeader(const string &headerText)
{
    string source, destination;

    displayAvailableLocationsWithHeader(headerText);

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

    ConsoleTable resultTable(1);
    resultTable.AddNewRow({algorithm + " Path Results"});

    if (algorithm == "Dijkstra")
    {
        path = Algorithms::dijkstra(hanoiMap, source, destination);
    }
    else if (algorithm == "A*")
    {
        path = Algorithms::astar(hanoiMap, source, destination);
    }

    if (!path.empty())
    {
        string pathStr;
        for (size_t i = 0; i < path.size(); ++i)
        {
            pathStr += path[i];
            if (i < path.size() - 1)
                pathStr += " -> ";
        }

        double distance = Algorithms::totalDistance(path, hanoiMap);
        stringstream distanceStr;
        distanceStr << fixed << setprecision(1) << distance << " km";

        resultTable.AddNewRow({"Path: " + pathStr});
        resultTable.AddNewRow({"Total Distance: " + distanceStr.str()});
    }
    else
    {
        resultTable.AddNewRow({"No valid path found!"});
    }

    resultTable.WriteTable(Align::Left);
}

void Application::handleChoice(int choice)
{
    clearScreen();

    switch (choice)
    {
    case 1:
    {
        cout << "Map Information\n";
        hanoiMap.displayGraph();
        break;
    }

    case 2:
    {
        auto [source, destination] = getSourceAndDestinationWithHeader("Dijkstra's Algorithm Pathfinding");
        clearScreen();
        findPath("Dijkstra", source, destination);
        break;
    }

    case 3:
    {
        auto [source, destination] = getSourceAndDestinationWithHeader("A* Algorithm Pathfinding");
        clearScreen();
        findPath("A*", source, destination);
        break;
    }

    case 4:
    {
        auto [source, destination] = getSourceAndDestinationWithHeader("Algorithm Comparison");

        clearScreen();
        findPath("Dijkstra", source, destination);
        findPath("A*", source, destination);
        break;
    }

    case 5:
    {
        ConsoleTable exitTable(1);
        exitTable.AddNewRow({"Thank you for using the Hanoi Map Pathfinding System!"});
        exitTable.WriteTable(Align::Center);
        exit(0);
    }

    default:
    {
        ConsoleTable errorTable(1);
        errorTable.AddNewRow({"Invalid choice! Please enter a number between 1 and 5."});
        errorTable.WriteTable(Align::Center);
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
            ConsoleTable errorTable(1);
            errorTable.AddNewRow({"Invalid input! Please enter a number between 1 and 5."});
            errorTable.WriteTable(Align::Center);
            continue;
        }

        handleChoice(choice);
    }
}
