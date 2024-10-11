#include "FileReading.h"
using namespace std;

int main()
{
    try {
        std::map<std::string, std::vector<std::string>> adjacencyMap = fillAdjacencies("Adjacencies.txt");
        std::map<std::string, std::pair<double, double>> coordinateMap = fillCoordinates("coordinates.csv");
    }
    catch (std::runtime_error &e) {
        std::cout << e.what() << std::endl;
    }

    return 0;
}
