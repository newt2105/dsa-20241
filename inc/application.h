#ifndef APPLICATION_H
#define APPLICATION_H

#include "graph.h"
#include "algorithms.h"
#include <string>

class Application
{
public:
    Application();
    void run();

private:
    Graph hanoiMap;
    bool firstRun;

    void clearScreen();
    void waitForEnter();
    void clearInputBuffer();
    void displayMenu();
    void displayAvailableLocations();
    bool isValidLocation(const std::string &location);
    std::pair<std::string, std::string> getSourceAndDestination();
    void findPath(const std::string &algorithm, const std::string &source, const std::string &destination);
    void handleChoice(int choice);
};

#endif
