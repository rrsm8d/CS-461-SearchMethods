#include "SearchFunctions.h"

void BFS(const std::map<std::string, std::vector<std::string>> &adjacencyMap, std::string startCity) {

    std::queue<std::string> toVisit;
    std::set<std::string> visited;

    toVisit.push(startCity);
    visited.insert(startCity);

    while(!toVisit.empty()) {
        std::string currentCity = toVisit.front();
        toVisit.pop();
        std::cout << currentCity << " ";

        // Iterate through the vector value at currentCity
        for(const auto& adjacentCity : adjacencyMap.at(currentCity)) {
            if(visited.find(adjacentCity) == visited.end()) {
                toVisit.push(adjacentCity);
                visited.insert(adjacentCity);
            }
        }
    }
    std::cout << std::endl;
    return;
}

/* // Old, I don't like how I implemented it. Proabably wrong and just real fuckin ugly
void DFS(const std::map<std::string, std::vector<std::string>> &adjacencyMap, std::string start) {
    std::set<std::string> visited;
    std::string currentCity = start;
    visited.insert(currentCity);
    std::cout << currentCity << " ";

    for (const auto& adjacentCity : adjacencyMap.at(currentCity)) {
        if (visited.find(adjacentCity) == visited.end()) {
            currentCity = adjacentCity;
            visited.insert(adjacentCity);
            std::cout << currentCity << " ";
        }
    }

    std::cout << std::endl;
}
*/

void DFSRecursive(const std::map<std::string, std::vector<std::string>> &adjacencyMap, std::string currentCity, std::set<std::string> &visited) {
    std::cout << currentCity << " ";
    visited.insert(currentCity);

    for (const auto adjacentCity : adjacencyMap.at(currentCity)) {
        if (visited.find(adjacentCity) == visited.end()) {
            DFSRecursive(adjacencyMap, adjacentCity, visited);
        }
    }
}

void DFS(const std::map<std::string, std::vector<std::string>> &adjacencyMap, std::string startCity) {
    std::set<std::string> visited;
    DFSRecursive(adjacencyMap, startCity, visited);
    std::cout << std::endl;
}


// Iterative Deepening DFS
// https://www.geeksforgeeks.org/iterative-deepening-searchids-iterative-deepening-depth-first-searchiddfs/
bool DLS(const std::map<std::string, std::vector<std::string>>& adjacencyMap, std::string currentCity, std::string goalCity, int limit, std::set<std::string> &visited) {
    std::cout << currentCity << " ";
    if (currentCity == goalCity) {
        return true;  // Goal found
    }

    if (limit <= 0) {
        return false;  // Depth limit reached
    }

    visited.insert(currentCity);

    for (const auto &adjacentCity : adjacencyMap.at(currentCity)) {
        if (visited.find(adjacentCity) == visited.end()) {
            if (DLS(adjacencyMap, adjacentCity, goalCity, --limit, visited)) {
                return true;
            }
        }
    }

    return false;
}

bool IDDFS(const std::map<std::string, std::vector<std::string>> &adjacencyMap, std::string startCity, std::string goalCity, int maxDepth) {

    for (int depth = 0; depth <= maxDepth; depth++) {
        std::set<std::string> visited;
        std::cout << "Depth: " << depth << " -> ";

        if (DLS(adjacencyMap, startCity, goalCity, depth, visited)) {
            std::cout << "\nGoal found at depth " << depth << std::endl;
            return true;
        }
        std::cout << std::endl;
    }

    std::cout << "Goal not found within depth " << maxDepth << std::endl;
    return false;
}

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    // Euclidean distance
    return std::sqrt(std::pow(lat2 - lat1, 2) + std::pow(lon2 - lon1, 2));
}


// Made with help with tweaks from GPT-4o
// Prompt:
//
// Given two variables, a std::map<std::string, std::vector<std::string>> adjacencyMap,
// and a std::map<std::string, std::pair<double, double>> coordinateMap,
// How would I implement the A* algorithm to find the optimal path from a startCity to an end City using C++?
// Assume the following:
// The adjacencyMap stores a city as it's key, and a list of all adjacent cities.
// the coordinateMap stores a city as it's key, and a pair of it's x y coordinates.
// the data within these maps has already been filled
// not using std namespace
std::vector<std::string> AStarSearch(
    const std::map<std::string, std::vector<std::string>> &adjacencyMap,
    const std::map<std::string, std::pair<double, double>> &coordinateMap,
    const std::string startCity,
    const std::string goalCity)
{
    // Priority queue to store (f(n), city)
    using CityInfo = std::pair<double, std::string>;
    std::priority_queue<CityInfo, std::vector<CityInfo>, std::greater<CityInfo>> openSet;

    // Maps for storing g(n) values and predecessors
    std::map<std::string, double> gScore;
    std::map<std::string, std::string> cameFrom;

    // Initialize gScore with infinity and 0 for startCity
    for (const auto& pair : adjacencyMap) {
        gScore[pair.first] = std::numeric_limits<double>::infinity();
    }
    gScore[startCity] = 0;

    // Push the start city into the priority queue
    openSet.push({0, startCity});

    while (!openSet.empty()) {
        // Get the city with the lowest f(n) from the queue
        std::string currentCity = openSet.top().second;
        openSet.pop();

        // If we have reached the goal, reconstruct the path
        if (currentCity == goalCity) {
            std::vector<std::string> path;
            while (currentCity != startCity) {
                path.push_back(currentCity);
                currentCity = cameFrom[currentCity];
            }
            path.push_back(startCity);
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Explore neighbors
        for (const auto& neighbor : adjacencyMap.at(currentCity)) {
            double tentative_gScore = gScore[currentCity] + calculateDistance(
                                          coordinateMap.at(currentCity).first, coordinateMap.at(currentCity).second,
                                          coordinateMap.at(neighbor).first, coordinateMap.at(neighbor).second
                                          );

            // If this path to the neighbor is better than any previous one
            if (tentative_gScore < gScore[neighbor]) {
                cameFrom[neighbor] = currentCity;
                gScore[neighbor] = tentative_gScore;

                // f(n) = g(n) + h(n), where h(n) is the heuristic (Euclidean distance to goal)
                double fScore = tentative_gScore + calculateDistance(
                                    coordinateMap.at(neighbor).first, coordinateMap.at(neighbor).second,
                                    coordinateMap.at(goalCity).first, coordinateMap.at(goalCity).second
                                    );

                openSet.push({fScore, neighbor});
            }
        }
    }

    // If we reach here, no path was found
    return {};
}
