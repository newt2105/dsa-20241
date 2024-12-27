#ifndef APPLICATION_H
#define APPLICATION_H

#include "graph.h"
#include <string>
#include <vector>

/**
 * @class Application
 * Manages the Hanoi map application and user interaction.
 */
class Application
{
private:
    Graph hanoiMap; // Graph representing the Hanoi map.
    bool firstRun;  // Indicates if the application is running for the first time.

    /**
     * Clears the console screen.
     */
    void clearScreen();

    /**
     * Waits for the user to press Enter.
     */
    void waitForEnter();

    /**
     * Displays the main menu to the user.
     */
    void displayMenu();

    /**
     * Displays a list of available locations with a custom header.
     * @param headerText Header to display above the list of locations.
     */
    void displayAvailableLocationsWithHeader(const std::string &headerText);

    /**
     * Checks if a given location is valid.
     * @param location The location ID to validate.
     * @return True if the location exists in the graph, false otherwise.
     */
    bool isValidLocation(const std::string &location);

    /**
     * Gets source and destination locations from the user with a header message.
     * @param headerText Header to display during input.
     * @return A pair of source and destination location IDs.
     */
    std::pair<std::string, std::string> getSourceAndDestinationWithHeader(const std::string &headerText);

    /**
     * Finds and displays a path between two locations using the specified algorithm.
     * @param algorithm The path-finding algorithm to use (e.g., "A*").
     * @param source The source location ID.
     * @param destination The destination location ID.
     */
    void findPath(const std::string &algorithm, const std::string &source, const std::string &destination);

    /**
     * Handles a menu choice selected by the user.
     * @param choice The menu choice to handle.
     */
    void handleChoice(int choice);

    /**
     * Allows the user to add a new location to the graph.
     */
    void addNewLocation();

public:
    /**
     * Constructor for the Application class.
     * Initializes the application and its components.
     */
    Application();

    /**
     * Runs the main application loop.
     */
    void run();
};

#endif
