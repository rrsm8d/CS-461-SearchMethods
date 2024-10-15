#include "FileReading.h"
#include "SearchFunctions.h"

void printPath(std::vector<std::string> path, double &distance) {
    for(const auto &city : path) {
        std::cout << city << " -> ";
    }
    std::cout << std::endl;
    std::cout << "TOTAL DISTANCE: " << distance << std::endl;
}

void printSelectionMenu() {
    std::cout << "***Enter a selection***" << std::endl;
    std::cout << "A - BFS" << std::endl;
    std::cout << "B - DFS" << std::endl;
    std::cout << "C - ID-DFS" << std::endl;
    std::cout << "D - A*" << std::endl;
    std::cout << "E - Best-first" << std::endl;
    std::cout << "X - Exit program" << std::endl;
}

int main()
{
    try {

        std::string startCity, goalCity;
        std::cout << "Enter starting city: ";
        //std::ostream::flush(); // Something weird is going on and I need to flush the io stream for some reason
        std::cin >> startCity;
        std::cout << std::endl << "Enter goal city: ";
        //std::ostream::flush();
        std::cin >> goalCity;

        std::map<std::string, std::vector<std::string>> adjacencyMap = fillAdjacencies("Adjacencies.txt");
        std::map<std::string, std::pair<double, double>> coordinateMap = fillCoordinates("coordinates.csv");

        if(adjacencyMap.count(startCity) <= 0 || adjacencyMap.count(goalCity) <= 0) {
            throw std::runtime_error("One of the cities does not exist. Check your spelling.");
        }

        char sel = 'Z';
        double distance = 0.0;
        while(sel != 'X') {
            distance = 0.0;
            printSelectionMenu();
            std::cin >> sel;
            sel = std::toupper(sel);
            switch(sel) {
                case 'A':
                    BFS(adjacencyMap, startCity, goalCity);
                    break;
                case 'B':
                    DFS(adjacencyMap, startCity, goalCity);
                    break;
                case 'C':
                    IDDFS(adjacencyMap, startCity, goalCity, 100);
                    break;
                case 'D':
                {
                    std::vector<std::string> visited = AStarSearch(adjacencyMap, coordinateMap, startCity, goalCity);
                    printPath(visited, distance);
                    break;
                }
                case 'E':
                {
                    std::vector<std::string> visited = BestFirstSearch(adjacencyMap, coordinateMap, startCity, goalCity);
                    printPath(visited, distance);
                    break;
                }
                case 'X':
                    break;
                default:
                std::cout << "Invalid" << std::endl;
            }
        }
        std::cout << "Ending  program" << std::endl;
    }
    catch (std::runtime_error &e) {
        std::cout << e.what() << std::endl;
    }
    return 0;
}
