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
