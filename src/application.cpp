#include "application.h"
#include "ultis.h"
#include "color.h"
#include <iostream>
#include <limits>
#include <algorithm>
#include <iomanip>

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
    cout << YELLOW "\nPress Enter to return to the main menu..." RESET;
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
}

/**
 * @brief Displays the main menu interface
 */
void Application::displayMenu()
{
    clearScreen();
    ConsoleTable table(1);
    Row header = {BOLD BLUE "Ha Noi Path Finding System" RESET};
    Row row1 = {GREEN "1. Display map information" RESET};
    Row row2 = {GREEN "2. Find path using Dijkstra's algorithm" RESET};
    Row row3 = {GREEN "3. Find path using A* algorithm" RESET};
    Row row4 = {GREEN "4. Find path using DFS algorithm" RESET};
    Row row5 = {GREEN "5. Compare all algorithms" RESET};
    Row row6 = {RED "6. Exit" RESET};

    table.AddNewRow(header);
    table.AddNewRow(row1);
    table.AddNewRow(row2);
    table.AddNewRow(row3);
    table.AddNewRow(row4);
    table.AddNewRow(row5);
    table.AddNewRow(row6);

    table.WriteTable(Align::Center);
    cout << CYAN "Enter your choice: " RESET;
}

/**
 * @brief Displays available locations with a header
 * @param headerText Text to display as header
 */
void Application::displayAvailableLocationsWithHeader(const string &headerText)
{
    ConsoleTable table(1);
    table.AddNewRow({BOLD BLUE + headerText + RESET});
    table.AddNewRow({BOLD YELLOW "Available Locations" RESET});

    vector<Node> nodes = hanoiMap.getNodes();
    for (const auto &node : nodes)
    {
        table.AddNewRow({CYAN + node.id + RESET});
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
    resultTable.AddNewRow({BOLD BLUE + algorithm + " Path Results" RESET});

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
            pathStr += GREEN + path[i] + RESET;
            if (i < path.size() - 1)
                pathStr += YELLOW " -> " RESET;
        }

        double distance = Algorithms::totalDistance(path, hanoiMap);
        stringstream distanceStr;
        distanceStr << fixed << setprecision(1) << CYAN << distance << " km" << RESET;

        resultTable.AddNewRow({"Path: " + pathStr});
        resultTable.AddNewRow({"Total Distance: " + distanceStr.str()});
    }
    else
    {
        resultTable.AddNewRow({RED "No valid path found!" RESET});
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
        cout << BOLD BLUE "Map Information\n" RESET;
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
        cout << "\n";
        findPath("A*", source, destination);
        cout << "\n";
        findPath("DFS", source, destination);
        break;
    }

    case 6:
    {
        ConsoleTable exitTable(1);
        exitTable.AddNewRow({BOLD GREEN "Thank you for using the Hanoi Map Pathfinding System!" RESET});
        exitTable.WriteTable(Align::Center);
        exit(0);
    }

    default:
    {
        ConsoleTable errorTable(1);
        errorTable.AddNewRow({RED "Invalid choice! Please enter a number between 1 and 6." RESET});
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
            errorTable.AddNewRow({"Invalid input! Please enter a number between 1 and 5."});
            errorTable.WriteTable(Align::Center);
            continue;
        }

        handleChoice(choice);
    }
}
