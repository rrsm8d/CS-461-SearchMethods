#include "FileReading.h"
#include "SearchFunctions.h"
using namespace std;

int main()
{
    try {
        std::map<std::string, std::vector<std::string>> adjacencyMap = fillAdjacencies("Adjacencies.txt");
        printAdjacencies(adjacencyMap);
        std::map<std::string, std::pair<double, double>> coordinateMap = fillCoordinates("coordinates.csv");
        printCoordinates(coordinateMap);
        std::cout << "***Ending debug print***" << std::endl;

        BFS(adjacencyMap, "Abilene");
        std::cout << std::endl;
        DFS(adjacencyMap, "Abilene");
        std::cout << std::endl;
        IDDFS(adjacencyMap, "Abilene", "Wichita", 100);
        std::cout << std::endl;
    }
    catch (std::runtime_error &e) {
        std::cout << e.what() << std::endl;
    }

    return 0;
}
