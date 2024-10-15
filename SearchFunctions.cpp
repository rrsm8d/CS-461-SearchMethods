#include "SearchFunctions.h"

void BFS(const std::map<std::string, std::vector<std::string>> &adjacencyMap, std::string startCity, std::string goalCity) {

    std::queue<std::string> toVisit;
    std::set<std::string> visited;

    toVisit.push(startCity);
    visited.insert(startCity);

    // Until it's impossible to progress, keep adding more
    while(!toVisit.empty()) {
        std::string currentCity = toVisit.front();
        // We visited it, now remove it
        toVisit.pop();
        std::cout << currentCity << " -> ";
        if(currentCity == goalCity) {
            std::cout << " Goal was found " << std::endl;
            return;
        }

        // Look at the adjacent cities of the current city that was dequeued
        for(const auto &adjacentCity : adjacencyMap.at(currentCity)) {
            // If an adjacent city wasn't visited, then mark as visited and insert it to the queue
            if(visited.find(adjacentCity) == visited.end()) {
                toVisit.push(adjacentCity);
                visited.insert(adjacentCity);
            }
        }
    }
    std::cout << std::endl;
    return;
}

void DFSRecursive(const std::map<std::string, std::vector<std::string>> &adjacencyMap, std::string currentCity, std::set<std::string> &visited, std::string goalCity) {
    visited.insert(currentCity);

    if(currentCity == goalCity) {
        // Print the path we took
        for(auto &city : visited) {
            std::cout << city << " -> ";
        }
        std::cout << goalCity << " Goal was found " << std::endl;
        return;
    }

    // For the adjacent cities, select one
    for (const auto adjacentCity : adjacencyMap.at(currentCity)) {
        if (visited.find(adjacentCity) == visited.end()) {
            // And then explore that city's adjacent cities, provided they haven't already been visited.
            DFSRecursive(adjacencyMap, adjacentCity, visited, goalCity);
        }
    }
}

void DFS(const std::map<std::string, std::vector<std::string>> &adjacencyMap, std::string startCity, std::string goalCity) {
    std::set<std::string> visited;
    DFSRecursive(adjacencyMap, startCity, visited, goalCity);
    std::cout << std::endl;
}


// Iterative Deepening DFS
// https://www.geeksforgeeks.org/iterative-deepening-searchids-iterative-deepening-depth-first-searchiddfs/
bool DLS(const std::map<std::string, std::vector<std::string>> &adjacencyMap, std::string currentCity, std::string goalCity, int limit, std::set<std::string> &visited) {
    // Goal found
    if (currentCity == goalCity) {
        return true;
    }

    // Went too far from given depth limit
    if (limit <= 0) {
        return false;
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

        if (DLS(adjacencyMap, startCity, goalCity, depth, visited)) {
            // Print the path taken by IDDFS
            for(auto &city : visited) {
                std::cout << city << " -> ";
            }

            std::cout << std::endl << "\nGoal found at depth " << depth << std::endl;
            return true;
        }
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
// The adjacencyMap stores a city as it's key, and a list of all adjacent cities for value.
// the coordinateMap stores a city as it's key, and a pair of it's x y coordinates for value.
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
    for (const auto &pair : adjacencyMap) {
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

        // Explore adjacent cities
        for (const auto &adjacentCity : adjacencyMap.at(currentCity)) {
            double tentative_gScore = gScore[currentCity] + calculateDistance(
                                          coordinateMap.at(currentCity).first, coordinateMap.at(currentCity).second,
                                          coordinateMap.at(adjacentCity).first, coordinateMap.at(adjacentCity).second
                                          );

            // If this path to the adjacent city is better than any previous one
            if (tentative_gScore < gScore[adjacentCity]) {
                cameFrom[adjacentCity] = currentCity;
                gScore[adjacentCity] = tentative_gScore;

                // f(n) = g(n) + h(n), where h(n) is the heuristic (Euclidean distance to goal)
                double fScore = tentative_gScore + calculateDistance(
                                    coordinateMap.at(adjacentCity).first, coordinateMap.at(adjacentCity).second,
                                    coordinateMap.at(goalCity).first, coordinateMap.at(goalCity).second
                                    );

                openSet.push({fScore, adjacentCity});
            }
        }
    }

    // If we reach here, no path was found
    return {};
}

std::vector<std::string> BestFirstSearch(
    const std::map<std::string, std::vector<std::string>> &adjacencyMap,
    const std::map<std::string, std::pair<double, double>> &coordinateMap,
    const std::string startCity,
    const std::string goalCity)
{
    // Priority queue to store (heuristic distance, city)
    using CityInfo = std::pair<double, std::string>;
    std::priority_queue<CityInfo, std::vector<CityInfo>, std::greater<CityInfo>> openSet;

    // Set to store visited cities
    std::set<std::string> visited;

    // Map to track the path (for backtracking)
    std::map<std::string, std::string> cameFrom;

    // Push the start city into the priority queue with heuristic distance to goal
    double hStart = calculateDistance(
        coordinateMap.at(startCity).first, coordinateMap.at(startCity).second,
        coordinateMap.at(goalCity).first, coordinateMap.at(goalCity).second
        );
    openSet.push({hStart, startCity});

    while (!openSet.empty()) {
        // Get the city with the lowest heuristic value
        std::string currentCity = openSet.top().second;
        openSet.pop();

        // If we reached the goal, reconstruct the path
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

        // Mark the current city as visited
        visited.insert(currentCity);

        // Explore adjacentCitys
        for (const auto &adjacentCity : adjacencyMap.at(currentCity)) {
            if (visited.find(adjacentCity) == visited.end()) {
                // Calculate the heuristic for the adjacentCity
                double distance = calculateDistance(
                    coordinateMap.at(adjacentCity).first, coordinateMap.at(adjacentCity).second,
                    coordinateMap.at(goalCity).first, coordinateMap.at(goalCity).second
                    );

                // Add the adjacent city to the priority queue and track the path
                openSet.push({distance, adjacentCity});
                cameFrom[adjacentCity] = currentCity;
            }
        }
    }

    // If no path is found, return an empty vector
    return {};
}
