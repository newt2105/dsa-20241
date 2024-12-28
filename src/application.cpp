#include "application.h"
#include "algorithms.h"
#include "ultis.h"
#include "graph.h"
#include <iostream>
#include <iomanip>
#include <limits>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>

using namespace std;

/**
 * @brief Constructor for Application class
 */
Application::Application() : firstRun(true) {}

/**
 * @brief Clears the console screen
 */
void Application::clearScreen()
{
#ifdef _WIN32
    system("cls");
#else
    int clear = system("clear");
#endif
}

/**
 * @brief Waits for user to press Enter key
 */
void Application::waitForEnter()
{
    cout << "\nPress Enter to return to the main menu...";
    cin.get();
}

/**
 * @brief Displays the main menu interface
 */
void Application::displayMenu()
{
    clearScreen();
    ConsoleTable table(1);
    Row header = {"Ha Noi Path Finding System"};
    Row row1 = {"1. Display map information"};
    Row row2 = {"2. Find path using Dijkstra's algorithm"};
    Row row3 = {"3. Find path using A* algorithm"};
    Row row4 = {"4. Find path using DFS algorithm"};
    Row row5 = {"5. Compare all algorithms"};
    Row row6 = {"6. Add new location"};
    Row row7 = {"7. Exit"};

    table.AddNewRow(header);
    table.AddNewRow(row1);
    table.AddNewRow(row2);
    table.AddNewRow(row3);
    table.AddNewRow(row4);
    table.AddNewRow(row5);
    table.AddNewRow(row6);
    table.AddNewRow(row7);

    table.WriteTable(Align::Center);
    cout << "Enter your choice: ";
}

/**
 * @brief Adds a new location to the map, connects it to another location, and saves the updates.
 */
void Application::addNewLocation()
{
    clearScreen();
    ConsoleTable table(1);
    table.AddNewRow({"Add New Location"});
    table.WriteTable(Align::Center);

    string id;
    double x, y;

    while (true)
    {
        cout << "Enter location ID (no spaces): ";
        getline(cin, id);

        if (id.find(' ') != string::npos)
        {
            ConsoleTable errorTable(1);
            errorTable.AddNewRow({"Error: Location ID cannot contain spaces! Please try again."});
            errorTable.WriteTable(Align::Center);
            continue;
        }

        if (isValidLocation(id))
        {
            ConsoleTable errorTable(1);
            errorTable.AddNewRow({"Error: Location ID already exists! Please try again."});
            errorTable.WriteTable(Align::Center);
            continue;
        }
        break;
    }

    while (true)
    {
        cout << "Enter latitude (x coordinate): ";
        string xInput;
        getline(cin, xInput);
        try
        {
            x = stod(xInput);
            if (x < -90 || x > 90)
                throw out_of_range("Latitude must be between -90 and 90");
            break;
        }
        catch (const exception &e)
        {
            ConsoleTable errorTable(1);
            errorTable.AddNewRow({"Error: Invalid latitude! " + string(e.what()) + " Please try again."});
            errorTable.WriteTable(Align::Center);
        }
    }

    while (true)
    {
        cout << "Enter longitude (y coordinate): ";
        string yInput;
        getline(cin, yInput);
        try
        {
            y = stod(yInput);
            if (y < -180 || y > 180)
                throw out_of_range("Longitude must be between -180 and 180");
            break;
        }
        catch (const exception &e)
        {
            ConsoleTable errorTable(1);
            errorTable.AddNewRow({"Error: Invalid longitude! " + string(e.what()) + " Please try again."});
            errorTable.WriteTable(Align::Center);
        }
    }

    Node newNode = {id, x, y};

    ConsoleTable locationTable(3);
    locationTable.AddNewRow({"Location ID", "Coordinates", "Distance (km)"});

    vector<Node> nodes = hanoiMap.getNodes();
    vector<pair<string, double>> distances;

    for (const auto &node : nodes)
    {
        double distance = Algorithms::calculateDistance(newNode, node);
        distances.push_back({node.id, distance});

        stringstream coords;
        coords << fixed << setprecision(6) << "(" << node.x << ", " << node.y << ")";

        stringstream distStr;
        distStr << fixed << setprecision(2) << distance;

        locationTable.AddNewRow({node.id, coords.str(), distStr.str()});
    }

    locationTable.WriteTable(Align::Left);

    int numConnections;
    while (true)
    {
        cout << "\nEnter number of locations to connect to: ";
        string numConnectionsInput;
        getline(cin, numConnectionsInput);
        try
        {
            numConnections = stoi(numConnectionsInput);
            if (numConnections <= 0 || numConnections > nodes.size())
                throw out_of_range("Number must be between 1 and " + to_string(nodes.size()));
            break;
        }
        catch (const exception &e)
        {
            ConsoleTable errorTable(1);
            errorTable.AddNewRow({"Error: Invalid number! " + string(e.what()) + " Please try again."});
            errorTable.WriteTable(Align::Center);
        }
    }

    vector<tuple<string, int, double>> connections;

    for (int i = 0; i < numConnections; i++)
    {
        cout << "\nConnection " << (i + 1) << ":\n";
        string connectTo;

        while (true)
        {
            cout << "Enter ID of location to connect to: ";
            getline(cin, connectTo);

            if (!isValidLocation(connectTo))
            {
                ConsoleTable errorTable(1);
                errorTable.AddNewRow({"Error: Invalid location! Please try again."});
                errorTable.WriteTable(Align::Center);
                continue;
            }
            break;
        }

        auto it = find_if(distances.begin(), distances.end(),
                          [&connectTo](const pair<string, double> &p)
                          {
                              return p.first == connectTo;
                          });
        double calculatedDistance = it->second;

        cout << "Calculated distance to " << connectTo << ": "
             << fixed << setprecision(2) << calculatedDistance << " km\n";

        int direction;
        while (true)
        {
            cout << "Type of direction (0 for two-way, 1 for one-way): ";
            string directionInput;
            getline(cin, directionInput);
            try
            {
                direction = stoi(directionInput);
                if (direction != 0 && direction != 1)
                    throw invalid_argument("Must be 0 or 1");
                break;
            }
            catch (...)
            {
                ConsoleTable errorTable(1);
                errorTable.AddNewRow({"Error: Invalid direction! Must be 0 or 1. Please try again."});
                errorTable.WriteTable(Align::Center);
            }
        }

        connections.push_back({connectTo, direction, calculatedDistance});
    }

    try
    {
        hanoiMap.addNode(id, x, y);

        for (const auto &[connectTo, direction, weight] : connections)
        {
            hanoiMap.addEdge(id, connectTo, direction == 1, weight);
            if (direction == 0)
            {
                hanoiMap.addEdge(connectTo, id, true, weight);
            }
        }

        hanoiMap.saveToFile();
        hanoiMap.draw();
        hanoiMap = Graph();

        ConsoleTable successTable(1);
        successTable.AddNewRow({"Location added successfully!"});
        successTable.WriteTable(Align::Center);
    }
    catch (const exception &e)
    {
        ConsoleTable errorTable(1);
        errorTable.AddNewRow({"Error: Failed to add location! " + string(e.what())});
        errorTable.WriteTable(Align::Center);
    }
}

/**
 * @brief Displays available locations with a header
 * @param headerText Text to display as header
 */
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

/**
 * @brief Checks if a location exists in the graph
 * @param location Location ID to check
 * @return true if location exists, false otherwise
 */
bool Application::isValidLocation(const string &location)
{
    vector<Node> nodes = hanoiMap.getNodes();
    return find_if(nodes.begin(), nodes.end(),
                   [&location](const Node &node)
                   {
                       return node.id == location;
                   }) != nodes.end();
}

/**
 * @brief Gets source and destination locations from user input
 * @param headerText Text to display as header
 * @return Pair of strings containing source and destination locations
 */
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

/**
 * @brief Finds and displays path between two locations using specified algorithm
 * @param algorithm Name of the algorithm to use
 * @param source Starting location
 * @param destination End location
 */
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
    else if (algorithm == "DFS")
    {
        path = Algorithms::dfs(hanoiMap, source, destination);
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

        double execTime = Algorithms::getLastExecutionTime();
        stringstream timeStr;
        timeStr << fixed << setprecision(2) << execTime << " ms";

        resultTable.AddNewRow({"Path: " + pathStr});
        resultTable.AddNewRow({"Total Distance: " + distanceStr.str()});
        resultTable.AddNewRow({"Execution Time: " + timeStr.str()});
    }
    else
    {
        resultTable.AddNewRow({"No valid path found!"});

        double execTime = Algorithms::getLastExecutionTime();
        stringstream timeStr;
        timeStr << fixed << setprecision(2) << execTime << " ms";
        resultTable.AddNewRow({"Execution Time: " + timeStr.str()});
    }

    resultTable.WriteTable(Align::Left);
}

/**
 * @brief Handles user menu choice
 * @param choice User's menu selection
 */
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
        auto [source, destination] = getSourceAndDestinationWithHeader("DFS Algorithm Pathfinding");
        clearScreen();
        findPath("DFS", source, destination);
        break;
    }

    case 5:
    {
        auto [source, destination] = getSourceAndDestinationWithHeader("Algorithm Comparison");
        clearScreen();
        findPath("Dijkstra", source, destination);
        findPath("A*", source, destination);
        findPath("DFS", source, destination);
        break;
    }

    case 6:
    {
        addNewLocation();
        break;
    }

    case 7:
    {
        ConsoleTable exitTable(1);
        exitTable.AddNewRow({"Thank you for using the Hanoi Map Pathfinding System!"});
        exitTable.WriteTable(Align::Center);
        exit(0);
    }

    default:
    {
        ConsoleTable errorTable(1);
        errorTable.AddNewRow({"Invalid choice! Please enter a number between 1 and 7."});
        errorTable.WriteTable(Align::Center);
        break;
    }
    }
}

/**
 * @brief Main application loop
 */
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
            errorTable.AddNewRow({"Invalid input! Please enter a number between 1 and 7."});
            errorTable.WriteTable(Align::Center);
            continue;
        }

        handleChoice(choice);
    }
}
