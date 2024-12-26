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
    void displayMenu();
    bool isValidLocation(const string &location);
    void displayAvailableLocationsWithHeader(const string &headerText); 
    pair<string, string> getSourceAndDestinationWithHeader(const string &headerText); 
    void findPath(const string &algorithm, const string &source, const string &destination);
    void handleChoice(int choice);
    void addNewLocation();
};

#endif
