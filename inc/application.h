#ifndef APPLICATION_H
#define APPLICATION_H

#include "graph.h"
#include "algorithms.h"
#include <string>

using namespace std;

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
    bool isValidLocation(const string &location);
    pair<string, string> getSourceAndDestination();
    void findPath(const string &algorithm, const string &source, const string &destination);
    void handleChoice(int choice);
};

#endif
