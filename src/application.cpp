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
Application::Application() : firstRun(true)
{
    try
    {
        // Initialize the map and other resources
        hanoiMap = Graph();
    }
    catch (const std::exception &e)
    {
        cerr << "Failed to initialize application: " << e.what() << endl;
        exit(1);
    }
}

/**
 * @brief Clears the console screen
 */
void Application::clearScreen()
{
    try
    {
#ifdef _WIN32
        system("cls");
#else
        int clear = system("clear");
        if (clear != 0)
        {
            throw runtime_error("Failed to clear screen");
        }
#endif
    }
    catch (const std::exception &e)
    {
        cerr << "Error clearing screen: " << e.what() << endl;
    }
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
    try
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
    catch (const std::exception &e)
    {
        cerr << "Error displaying menu: " << e.what() << endl;
    }
}

/**
 * @brief Calculates the simple Euclidean distance between two points.
 * @param x1 Latitude of the first point.
 * @param y1 Longitude of the first point.
 * @param x2 Latitude of the second point.
 * @param y2 Longitude of the second point.
 * @return The Euclidean distance between the two points.
 */
double simpleDistance(double x1, double y1, double x2, double y2)
{
    try
    {
        return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    }
    catch (const std::exception &e)
    {
        cerr << "Error calculating distance: " << e.what() << endl;
        return -1.0;
    }
}

/**
 * @brief Adds a new location to the map, connects it to another location, and saves the updates.
 */
void Application::addNewLocation()
{
    try
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
            if (!getline(cin, id))
            {
                throw runtime_error("Failed to read location ID");
            }

            if (id.empty())
            {
                throw runtime_error("Location ID cannot be empty");
            }

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

        // Read latitude
        while (true)
        {
            cout << "Enter latitude (x coordinate): ";
            string xInput;
            if (!getline(cin, xInput))
            {
                throw runtime_error("Failed to read latitude");
            }
            try
            {
                x = stod(xInput);
                if (x < -90 || x > 90)
                {
                    throw out_of_range("Latitude must be between -90 and 90");
                }
                break;
            }
            catch (const exception &e)
            {
                ConsoleTable errorTable(1);
                errorTable.AddNewRow({"Error: Invalid latitude! " + string(e.what()) + " Please try again."});
                errorTable.WriteTable(Align::Center);
            }
        }

        // Read longitude
        while (true)
        {
            cout << "Enter longitude (y coordinate): ";
            string yInput;
            if (!getline(cin, yInput))
            {
                throw runtime_error("Failed to read longitude");
            }
            try
            {
                y = stod(yInput);
                if (y < -180 || y > 180)
                {
                    throw out_of_range("Longitude must be between -180 and 180");
                }
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

        // Display nearby locations
        ConsoleTable locationTable(3);
        locationTable.AddNewRow({"Location ID", "Coordinates", "Distance (km)"});

        vector<Node> nodes = hanoiMap.getNodes();
        vector<pair<string, double>> distances;

        for (const auto &node : nodes)
        {
            double distance = simpleDistance(newNode.x, newNode.y, node.x, node.y);
            if (distance < 0)
            {
                throw runtime_error("Failed to calculate distance");
            }
            distances.push_back({node.id, distance});

            stringstream coords;
            coords << fixed << setprecision(6) << "(" << node.x << ", " << node.y << ")";

            stringstream distStr;
            distStr << fixed << setprecision(2) << distance;

            locationTable.AddNewRow({node.id, coords.str(), distStr.str()});
        }

        locationTable.WriteTable(Align::Left);

        // Get number of connections
        int numConnections;
        while (true)
        {
            cout << "\nEnter number of locations to connect to: ";
            string numConnectionsInput;
            if (!getline(cin, numConnectionsInput))
            {
                throw runtime_error("Failed to read number of connections");
            }
            try
            {
                numConnections = stoi(numConnectionsInput);
                if (numConnections <= 0 || numConnections > nodes.size())
                {
                    throw out_of_range("Number must be between 1 and " + to_string(nodes.size()));
                }
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

        // Get connection details
        for (int i = 0; i < numConnections; i++)
        {
            cout << "\nConnection " << (i + 1) << ":\n";
            string connectTo;

            while (true)
            {
                cout << "Enter ID of location to connect to: ";
                if (!getline(cin, connectTo))
                {
                    throw runtime_error("Failed to read connection ID");
                }

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

            if (it == distances.end())
            {
                throw runtime_error("Location not found in distances");
            }

            double calculatedDistance = it->second;

            cout << "Calculated distance to " << connectTo << ": "
                 << fixed << setprecision(2) << calculatedDistance << " km\n";

            int direction;
            while (true)
            {
                cout << "Type of direction (0 for two-way, 1 for one-way): ";
                string directionInput;
                if (!getline(cin, directionInput))
                {
                    throw runtime_error("Failed to read direction");
                }
                try
                {
                    direction = stoi(directionInput);
                    if (direction != 0 && direction != 1)
                    {
                        throw invalid_argument("Must be 0 or 1");
                    }
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

        // Add new location and connections
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
    try
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
    catch (const exception &e)
    {
        cerr << "Error displaying locations: " << e.what() << endl;
    }
}

/**
 * @brief Checks if a location exists in the graph
 * @param location Location ID to check
 * @return true if location exists, false otherwise
 */
bool Application::isValidLocation(const string &location)
{
    try
    {
        vector<Node> nodes = hanoiMap.getNodes();
        return find_if(nodes.begin(), nodes.end(),
                       [&location](const Node &node)
                       {
                           return node.id == location;
                       }) != nodes.end();
    }
    catch (const exception &e)
    {
        cerr << "Error checking location validity: " << e.what() << endl;
        return false;
    }
}

/**
 * @brief Gets source and destination locations from user input
 * @param headerText Text to display as header
 * @return Pair of strings containing source and destination locations
 */
pair<string, string> Application::getSourceAndDestinationWithHeader(const string &headerText)
{
    string source, destination;

    try
    {
        displayAvailableLocationsWithHeader(headerText);

        while (true)
        {
            cout << "\nEnter starting location: ";
            if (!getline(cin, source))
            {
                throw runtime_error("Failed to read starting location");
            }

            if (isValidLocation(source))
            {
                break;
            }
            cout << "Invalid location! Please choose from the available locations.\n";
        }

        while (true)
        {
            cout << "Enter destination: ";
            if (!getline(cin, destination))
            {
                throw runtime_error("Failed to read destination");
            }

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
    catch (const exception &e)
    {
        cerr << "Error getting source and destination: " << e.what() << endl;
        return make_pair("", "");
    }
}

/**
 * @brief Finds and displays path between two locations using specified algorithm
 * @param algorithm Name of the algorithm to use
 * @param source Starting location
 * @param destination End location
 */
void Application::findPath(const string &algorithm, const string &source, const string &destination)
{
    try
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
        else
        {
            throw runtime_error("Invalid algorithm specified");
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
    catch (const exception &e)
    {
        ConsoleTable errorTable(1);
        errorTable.AddNewRow({"Error finding path: " + string(e.what())});
        errorTable.WriteTable(Align::Center);
    }
}

/**
 * @brief Handles user menu choice
 * @param choice User's menu selection
 */
void Application::handleChoice(int choice)
{
    try
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
            if (!source.empty() && !destination.empty())
            {
                clearScreen();
                findPath("Dijkstra", source, destination);
            }
            break;
        }
        case 3:
        {
            auto [source, destination] = getSourceAndDestinationWithHeader("A* Algorithm Pathfinding");
            if (!source.empty() && !destination.empty())
            {
                clearScreen();
                findPath("A*", source, destination);
            }
            break;
        }
        case 4:
        {
            auto [source, destination] = getSourceAndDestinationWithHeader("DFS Algorithm Pathfinding");
            if (!source.empty() && !destination.empty())
            {
                clearScreen();
                findPath("DFS", source, destination);
            }
            break;
        }
        case 5:
        {
            auto [source, destination] = getSourceAndDestinationWithHeader("Algorithm Comparison");
            if (!source.empty() && !destination.empty())
            {
                clearScreen();
                findPath("Dijkstra", source, destination);
                findPath("A*", source, destination);
                findPath("DFS", source, destination);
            }
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
    catch (const exception &e)
    {
        ConsoleTable errorTable(1);
        errorTable.AddNewRow({"Error handling choice: " + string(e.what())});
        errorTable.WriteTable(Align::Center);
    }
}

/**
 * @brief Main application loop
 */
void Application::run()
{
    try
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
            if (!getline(cin, input))
            {
                throw runtime_error("Failed to read user input");
            }

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
    catch (const exception &e)
    {
        ConsoleTable errorTable(1);
        errorTable.AddNewRow({"Critical error: " + string(e.what())});
        errorTable.WriteTable(Align::Center);
        exit(1);
    }
}